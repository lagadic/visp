// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example franka_generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in
 * front of the robot.
 *
 * This example is part of libfranka FCI C++ API:
 * https://frankaemika.github.io/libfranka See
 * https://frankaemika.github.io/docs for more details.
 */

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: ./generate_cartesian_pose_motion <robot-hostname>"
              << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    // Set additional parameters always before the control loop, NEVER in the
    // control loop! Set collision behavior.
    robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    auto initial_pose = robot.readOnce().O_T_EE_d;
    double radius = 0.3;
    double time = 0.0;
    robot.control([=, &time](
                      const franka::RobotState &,
                      franka::Duration time_step) -> franka::CartesianPose {
      time += time_step.toSec();

      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = radius * std::sin(angle);
      double delta_z = radius * (std::cos(angle) - 1);

      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;

      if (time >= 10.0) {
        std::cout << std::endl
                  << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

#else
int main()
{
  std::cout << "This example needs libfranka to control Panda robot."
            << std::endl;
}
#endif
