// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include "vpJointPosTrajGenerator_impl.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>


vpJointPosTrajGenerator::vpJointPosTrajGenerator(double speed_factor, const std::array<double, 7> q_goal)
  : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool vpJointPosTrajGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync_ = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
            (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] = delta_q_[i] +
            0.5 *
            (1.0 / std::pow(delta_t_2_sync_[i], 3.0) *
             (t - t_1_sync_[i] - 2.0 * delta_t_2_sync_[i] - t_d[i]) *
             std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
             (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync_[i] - 2.0 * t_d[i])) *
            dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void vpJointPosTrajGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions vpJointPosTrajGenerator::operator()(const franka::RobotState& robot_state,
                                                           franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    for(int i=0; i<7; i++) {
      std::cout << " " << q_start_[i];
    }
    std::cout << std::endl;

    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}
