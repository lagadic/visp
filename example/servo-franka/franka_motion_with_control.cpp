// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example franka_motion_with_control.cpp
 * An example showing how to use a joint velocity motion generator and torque control.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 *
 * This example is part of libfranka FCI C++ API: https://frankaemika.github.io/libfranka
 * See https://frankaemika.github.io/docs for more details.
 */

namespace {

class Controller {
 public:
  Controller(size_t dq_filter_size,
             const std::array<double, 7>& K_P,  // NOLINT
             const std::array<double, 7>& K_D)  // NOLINT
      : dq_current_filter_position_(0),
        dq_filter_size_(dq_filter_size),
        K_P_(K_P),
        K_D_(K_D) {
    std::fill(dq_d_.begin(), dq_d_.end(), 0);
    dq_buffer_.reset(new double[dq_filter_size_ * 7]);
    std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
  }

  inline franka::Torques step(const franka::RobotState& state) {
    updateDQFilter(state);

    std::array<double, 7> tau_J_d;  // NOLINT
    for (size_t i = 0; i < 7; i++) {
      tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i));
    }
    return tau_J_d;
  }

  void updateDQFilter(const franka::RobotState& state) {
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }

  double getDQFiltered(size_t index) const {
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_.get()[i];
    }
    return value / dq_filter_size_;
  }

 private:
  size_t dq_current_filter_position_;
  size_t dq_filter_size_;

  const std::array<double, 7> K_P_;  // NOLINT
  const std::array<double, 7> K_D_;  // NOLINT

  std::array<double, 7> dq_d_;
  std::unique_ptr<double[]> dq_buffer_;
};

std::vector<double> generateTrajectory(double a_max) {
  // Generating a motion with smooth velocity and acceleration.
  // Squared sine is used for the acceleration/deceleration phase.
  std::vector<double> trajectory;
  constexpr double kTimeStep = 0.001;          // [s]
  constexpr double kAccelerationTime = 1;      // time spend accelerating and decelerating [s]
  constexpr double kConstantVelocityTime = 1;  // time spend with constant speed [s]
  // obtained during the speed up
  // and slow down [rad/s^2]
  double a = 0;  // [rad/s^2]
  double v = 0;  // [rad/s]
  double t = 0;  // [s]
  while (t < (2 * kAccelerationTime + kConstantVelocityTime)) {
    if (t <= kAccelerationTime) {
      a = pow(sin(t * M_PI / kAccelerationTime), 2) * a_max;
    } else if (t <= (kAccelerationTime + kConstantVelocityTime)) {
      a = 0;
    } else {
      const double deceleration_time =
          (kAccelerationTime + kConstantVelocityTime) - t;  // time spent in the deceleration phase
      a = -pow(sin(deceleration_time * M_PI / kAccelerationTime), 2) * a_max;
    }
    v += a * kTimeStep;
    t += kTimeStep;
    trajectory.push_back(v);
  }
  return trajectory;
}

}  // anonymous namespace

int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr << "Usage: ./" << argv[0] << " <robot-hostname>"
              << " <filter size>"
              << " <K_P>"
              << " <K_D>"
              << " <joint>"
              << " <a_max>" << std::endl;
    return -1;
  }

  size_t filter_size = std::stoul(argv[2]);
  std::array<double, 7> K_P;  // NOLINT
  std::array<double, 7> K_D;  // NOLINT
  for (size_t i = 0; i < 7; i++) {
    K_P[i] = std::stod(argv[3]);
    K_D[i] = std::stod(argv[4]);
  }

  std::cout << "Initializing controller: " << std::endl;
  for (size_t i = 0; i < 7; i++) {
    std::cout << i + 1 << ": K_P = " << K_P[i] << "\tK_D = " << K_D[i] << std::endl;
  }
  std::cout << "dq filter size: " << filter_size << std::endl;
  Controller controller(filter_size, K_P, K_D);

  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    size_t index = 0;
    int joint_number = std::stoi(argv[5]);
    std::vector<double> trajectory = generateTrajectory(std::stod(argv[6]));

    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
          return controller.step(robot_state);
        },
        [&](const franka::RobotState&, franka::Duration time_step) -> franka::JointVelocities {
          index += time_step.toMSec();

          if (index >= trajectory.size()) {
            index = trajectory.size() - 1;
          }

          franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
          velocities.dq[joint_number] = trajectory[index];

          if (index >= trajectory.size() - 1) {
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

#else
int main()
{
  std::cout << "This example needs libfranka to control Panda robot." << std::endl;
}
#endif
