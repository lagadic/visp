// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example franka_joint_point_to_point_motion.cpp
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 *
 * This example is part of libfranka FCI C++ API: https://frankaemika.github.io/libfranka
 * See https://frankaemika.github.io/docs for more details.
 */

constexpr double kDeltaQMotionFinished = 1e-6;

inline int sgn(double x) {
  if (x == 0) {
    return 0;
  }
  return (x > 0) ? 1 : -1;
}

std::array<double, 7> add(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  std::array<double, 7> result;
  for (size_t i = 0; i < a.size(); i++) {
    result[i] = a[i] + b[i];
  }
  return result;
}

std::array<double, 7> subtract(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  std::array<double, 7> result;
  for (size_t i = 0; i < a.size(); i++) {
    result[i] = a[i] - b[i];
  }
  return result;
}

bool calculateDesiredValues(double t,
                            const std::array<double, 7>& delta_q,
                            const std::array<double, 7>& dq_max,
                            const std::array<double, 7>& t_1,
                            const std::array<double, 7>& t_2,
                            const std::array<double, 7>& t_f,
                            const std::array<double, 7>& q_1,
                            std::array<double, 7>* delta_q_d);

void calculateSynchronizedValues(const std::array<double, 7>& delta_q,
                                 const std::array<double, 7>& dq_max,
                                 const std::array<double, 7>& ddq_max_start,
                                 const std::array<double, 7>& ddq_max_goal,
                                 std::array<double, 7>* dq_max_sync,
                                 std::array<double, 7>* t_1_sync,
                                 std::array<double, 7>* t_2_sync,
                                 std::array<double, 7>* t_f_sync,
                                 std::array<double, 7>* q_1);

int main(int argc, char** argv) {
  if (argc != 10) {
    std::cerr << "Usage: ./generate_joint_pose_motion "
              << "<robot-hostname> <goal-position> <speed-factor>" << std::endl
              << "speed-factor must be between zero and one." << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    std::array<double, 7> q_goal;
    for (size_t i = 0; i < 7; i++) {
      q_goal[i] = std::stod(argv[i + 2]);
    }
    double speed_factor = std::stod(argv[9]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    std::array<double, 7> q_start = robot.readOnce().q_d;

    std::array<double, 7> dq_max{{2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5}};
    std::array<double, 7> ddq_max_start{{5, 5, 5, 5, 5, 5, 5}};
    std::array<double, 7> ddq_max_goal{{5, 5, 5, 5, 5, 5, 5}};
    for (size_t i = 0; i < 7; i++) {
      dq_max[i] = speed_factor * dq_max[i];
      ddq_max_start[i] = speed_factor * ddq_max_start[i];
      ddq_max_goal[i] = speed_factor * ddq_max_goal[i];
    }

    double time = 0.0;

    std::array<double, 7> dq_max_sync{};
    std::array<double, 7> t_1_sync{};
    std::array<double, 7> t_2_sync{};
    std::array<double, 7> t_f_sync{};
    std::array<double, 7> q_1{};
    std::array<double, 7> delta_q = subtract(q_goal, q_start);

    calculateSynchronizedValues(delta_q, dq_max, ddq_max_start, ddq_max_goal, &dq_max_sync,
                                &t_1_sync, &t_2_sync, &t_f_sync, &q_1);
    robot.control([=, &time](const franka::RobotState&,
                             franka::Duration time_step) -> franka::JointPositions {
      time += time_step.toSec();

      std::array<double, 7> delta_q_d;
      bool motion_finished = calculateDesiredValues(time, delta_q, dq_max_sync, t_1_sync, t_2_sync,
                                                    t_f_sync, q_1, &delta_q_d);

      franka::JointPositions output = add(q_start, delta_q_d);
      output.motion_finished = motion_finished;
      return output;
    });
    std::cout << std::endl << "Motion finished" << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

bool calculateDesiredValues(double t,
                            const std::array<double, 7>& delta_q,
                            const std::array<double, 7>& dq_max,
                            const std::array<double, 7>& t_1,
                            const std::array<double, 7>& t_2,
                            const std::array<double, 7>& t_f,
                            const std::array<double, 7>& q_1,
                            std::array<double, 7>* delta_q_d) {
  std::array<int, 7> sign_delta_q;
  std::array<double, 7> t_d = subtract(t_2, t_1);
  std::array<double, 7> delta_t_2 = subtract(t_f, t_2);
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    sign_delta_q[i] = sgn(delta_q[i]);
    if (std::abs(delta_q[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1[i], 3) * dq_max[i] * sign_delta_q[i] *
                          (0.5 * t - t_1[i]) * std::pow(t, 3);
      } else if (t >= t_1[i] && t < t_2[i]) {
        (*delta_q_d)[i] = q_1[i] + (t - t_1[i]) * dq_max[i] * sign_delta_q[i];
      } else if (t >= t_2[i] && t < t_f[i]) {
        (*delta_q_d)[i] =
            delta_q[i] +
            0.5 * (1.0 / std::pow(delta_t_2[i], 3) * (t - t_1[i] - 2 * delta_t_2[i] - t_d[i]) *
                       std::pow((t - t_1[i] - t_d[i]), 3) +
                   (2.0 * t - 2.0 * t_1[i] - delta_t_2[i] - 2.0 * t_d[i])) *
                dq_max[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void calculateSynchronizedValues(const std::array<double, 7>& delta_q,
                                 const std::array<double, 7>& dq_max,
                                 const std::array<double, 7>& ddq_max_start,
                                 const std::array<double, 7>& ddq_max_goal,
                                 std::array<double, 7>* dq_max_sync,
                                 std::array<double, 7>* t_1_sync,
                                 std::array<double, 7>* t_2_sync,
                                 std::array<double, 7>* t_f_sync,
                                 std::array<double, 7>* q_1) {
  std::array<double, 7> dq_max_reach = dq_max;
  std::array<double, 7> t_f{};
  std::array<double, 7> delta_t_2{};
  std::array<double, 7> t_1{};
  std::array<double, 7> delta_t_2_sync{};
  int sign_delta_q[7];
  for (size_t i = 0; i < 7; i++) {
    sign_delta_q[i] = sgn(delta_q[i]);
    if (std::abs(delta_q[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q[i]) < (3.0 / 4.0 * (std::pow(dq_max[i], 2) / ddq_max_start[i]) +
                                  3.0 / 4.0 * (std::pow(dq_max[i], 2) / ddq_max_goal[i]))) {
        dq_max_reach[i] =
            std::sqrt(4.0 / 3.0 * delta_q[i] * sign_delta_q[i] *
                      (ddq_max_start[i] * ddq_max_goal[i]) / (ddq_max_start[i] + ddq_max_goal[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q[i]) / dq_max_reach[i];
    }
  }

  double max_t_f = *std::max_element(t_f.begin(), t_f.end());
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal[i] + ddq_max_start[i]);
      double b = -1.0 * max_t_f * ddq_max_goal[i] * ddq_max_start[i];
      double c = std::abs(delta_q[i]) * ddq_max_goal[i] * ddq_max_start[i];
      double delta = b * b - 4.0 * a * c;
      (*dq_max_sync)[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      (*t_1_sync)[i] = 1.5 * (*dq_max_sync)[i] / ddq_max_start[i];
      delta_t_2_sync[i] = 1.5 * (*dq_max_sync)[i] / ddq_max_goal[i];
      (*t_f_sync)[i] =
          (*t_1_sync)[i] / 2 + delta_t_2_sync[i] / 2 + std::abs(delta_q[i] / (*dq_max_sync)[i]);
      (*t_2_sync)[i] = (*t_f_sync)[i] - delta_t_2_sync[i];
      (*q_1)[i] = (*dq_max_sync)[i] * sign_delta_q[i] * (0.5 * (*t_1_sync)[i]);
    }
  }
}

#else
int main()
{
  std::cout << "This example needs libfranka to control Panda robot." << std::endl;
}
#endif
