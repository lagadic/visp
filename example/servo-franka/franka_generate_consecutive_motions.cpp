// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example franka_generate_consecutive_motions.cpp
 * An example showing how to execute consecutive motions with error recovery.
 *
 * @warning Before executing this example, make sure there is enough space in front and to the side
 * of the robot.
 *
 * This example is part of libfranka FCI C++ API: https://frankaemika.github.io/libfranka
 * See https://frankaemika.github.io/docs for more details.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: ./generate_consecutive_motions <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
        {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
        {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}},
        {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}});

    for (int i = 0; i < 5; i++) {
      std::cout << "Executing motion." << std::endl;
      try {
        double time_max = 4.0;
        double omega_max = 0.2;
        double time = 0.0;
        robot.control([=, &time](const franka::RobotState&,
                                 franka::Duration time_step) -> franka::JointVelocities {
          time += time_step.toSec();

          double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
          double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

          franka::JointVelocities velocities = {{0.0, 0.0, omega, 0.0, 0.0, 0.0, 0.0}};
          if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion." << std::endl;
            return franka::MotionFinished(velocities);
          }
          return velocities;
        });
      } catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot.automaticErrorRecovery();
      }
    }
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  std::cout << "Finished." << std::endl;

  return 0;
}
