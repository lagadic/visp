// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <iterator>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <franka/exception.h>
#include <franka/model.h>

/**
 * @example franka_print_joint_positions.cpp
 * An example showing how to use the model library.
 *
 * This example is part of libfranka FCI C++ API:
 * https://frankaemika.github.io/libfranka See
 * https://frankaemika.github.io/docs for more details.
 */

template <class T, size_t N> std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
{
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: ./print_joint_positions <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    franka::RobotState state = robot.readOnce();

    franka::Model model(robot.loadModel());
    for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector; frame++) {
      std::cout << model.pose(frame, state) << std::endl;
    }
  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

#else
int main() { std::cout << "This example needs libfranka to control Panda robot." << std::endl; }
#endif
