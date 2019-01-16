/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Test Viper 650 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRobotViper650-frames.cpp

  Test Viper 650 robot joint and cartesian control.
*/

#include <iostream>
#include <visp3/robot/vpRobotViper650.h>

#ifdef VISP_HAVE_VIPER650

bool pose_equal(const vpHomogeneousMatrix &M1, const vpHomogeneousMatrix &M2, double epsilon = 1e-6)
{
  vpTranslationVector t1, t2;
  M1.extract(t1);
  M2.extract(t2);
  vpThetaUVector tu1, tu2;
  M1.extract(tu1);
  M2.extract(tu2);

  for (unsigned int i = 0; i < 3; i++) {
    if (std::fabs(t1[i] - t2[i]) > epsilon)
      return false;
    if (std::fabs(tu1[i] - tu2[i]) > epsilon)
      return false;
  }
  return true;
}

bool joint_equal(const vpColVector &q1, const vpColVector &q2, double epsilon = 1e-6)
{
  for (unsigned int i = 0; i < q1.size(); i++) {
    if (std::fabs(q1[i] - q2[i]) > epsilon) {
      return false;
    }
  }
  return true;
}

int main()
{
  try {
    //********* Define transformation from end effector to tool frame
    vpHomogeneousMatrix eMt;

#if 0
    // In this case, we set tool frame to the end of the two fingers pneumatic gripper
    eMt[0][0] = 0;
    eMt[1][0] = 0;
    eMt[2][0] = -1;

    eMt[0][1] = -sqrt(2)/2;
    eMt[1][1] = -sqrt(2)/2;
    eMt[2][1] = 0;

    eMt[0][2] = -sqrt(2)/2;
    eMt[1][2] = sqrt(2)/2;
    eMt[2][2] = 0;

    eMt[0][3] = -0.177;
    eMt[1][3] = 0.177;
    eMt[2][3] = 0.077;
#else
    // In this case, we set tool frame to the PTGrey Flea2 camera frame
    vpTranslationVector etc(-0.04437278107, -0.001192883711, 0.07808296844);
    vpRxyzVector erxyzc(vpMath::rad(0.7226737722), vpMath::rad(2.103893926), vpMath::rad(-90.46213439));
    eMt.buildFrom(etc, vpRotationMatrix(erxyzc));
#endif
    std::cout << "eMt:\n" << eMt << std::endl;

    //********* Init robot
    std::cout << "Connection to Viper 650 robot" << std::endl;
    vpRobotViper650 robot;
    robot.init(vpViper650::TOOL_CUSTOM, eMt);

    // Move robot to repos position
    vpColVector repos(6);        // q1, q4, q6 = 0
    repos[1] = vpMath::rad(-90); // q2
    repos[2] = vpMath::rad(180); // q3
    repos[4] = vpMath::rad(90);  // q5

    robot.setPosition(vpRobot::ARTICULAR_FRAME, repos);

    vpColVector q;
    robot.getPosition(vpRobotViper650::ARTICULAR_FRAME, q);

    std::cout << "q: " << q.t() << std::endl;

    vpHomogeneousMatrix fMw, fMe, fMt, cMe;
    robot.get_fMw(q, fMw);
    robot.get_fMe(q, fMe);
    robot.get_fMc(q, fMt);
    robot.get_cMe(cMe);

    std::cout << "fMw:\n" << fMw << std::endl;
    std::cout << "fMe:\n" << fMe << std::endl;
    std::cout << "fMt:\n" << fMt << std::endl;
    std::cout << "eMc:\n" << cMe.inverse() << std::endl;

    //********* Check if retrieved eMt transformation is the one that was set
    // during init
    if (1) {
      vpHomogeneousMatrix eMt_ = fMe.inverse() * fMt;
      std::cout << "eMt_:\n" << eMt_ << std::endl;

      // Compare pose
      std::cout << "Compare pose eMt and eMt_:" << std::endl;
      if (!pose_equal(eMt, eMt_, 1e-4)) {
        std::cout << "  Error: Pose eMt differ" << std::endl;
        std::cout << "\nTest failed" << std::endl;
        return -1;
      }
      std::cout << "  They are the same, we can continue" << std::endl;

      //********* Check if retrieved eMc transformation is the one that was
      // set

      std::cout << "eMc:\n" << cMe.inverse() << std::endl;
      // Compare pose
      std::cout << "Compare pose eMt and eMc:" << std::endl;
      if (!pose_equal(eMt, cMe.inverse(), 1e-4)) {
        std::cout << "  Error: Pose eMc differ" << std::endl;
        std::cout << "\nTest failed" << std::endl;
        return -1;
      }
      std::cout << "  They are the same, we can continue" << std::endl;
    }

    //********* Check if position in reference frame is equal to fMt
    if (1) {
      vpColVector f_pose_t; // translation vector + rxyz vector
      robot.getPosition(vpRobot::REFERENCE_FRAME, f_pose_t);
      // Compute homogeneous transformation
      vpTranslationVector f_t_t;
      vpRxyzVector f_rxyz_t;
      for (unsigned int i = 0; i < 3; i++) {
        f_t_t[i] = f_pose_t[i];
        f_rxyz_t[i] = f_pose_t[i + 3];
      }
      vpHomogeneousMatrix fMt_(f_t_t, vpRotationMatrix(f_rxyz_t));
      std::cout << "fMt_ (from ref frame):\n" << fMt_ << std::endl;

      std::cout << "Compare pose fMt and fMt_:" << std::endl;
      if (!pose_equal(fMt, fMt_, 1e-4)) {
        std::cout << "  Error: Pose fMt differ" << std::endl;
        std::cout << "\nTest failed" << std::endl;
        return -1;
      }
      std::cout << "  They are the same, we can continue" << std::endl;
    }

    //********* Test inverse kinematics
    if (1) {
      vpColVector q1;
      robot.getInverseKinematics(fMt, q1);

      std::cout << "Move robot in joint (the robot should not move)" << std::endl;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setPosition(vpRobotViper650::ARTICULAR_FRAME, q1);

      vpColVector q2;
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q2);
      std::cout << "Reach joint position q2: " << q2.t() << std::endl;

      std::cout << "Compare joint position q and q2:" << std::endl;
      if (!joint_equal(q, q2, 1e-4)) {
        std::cout << "  Error: Joint position differ" << std::endl;
        std::cout << "\nTest failed" << std::endl;
        return -1;
      }
      std::cout << "  They are the same, we can continue" << std::endl;
    }

    //********* Check if fMt position can be set in reference frame
    if (1) {
      vpColVector f_pose_t(6);
      vpTranslationVector f_t_t = fMt.getTranslationVector();
      vpRxyzVector f_rxyz_t(fMt.getRotationMatrix());
      for (unsigned int i = 0; i < 3; i++) {
        f_pose_t[i] = f_t_t[i];
        f_pose_t[i + 3] = f_rxyz_t[i];
      }

      std::cout << "Move robot in reference frame (the robot should not move)" << std::endl;
      robot.setPosition(vpRobot::REFERENCE_FRAME, f_pose_t);
      vpColVector q3;
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q3);
      std::cout << "Reach joint position q3: " << q3.t() << std::endl;
      std::cout << "Compare joint position q and q3:" << std::endl;
      if (!joint_equal(q, q3, 1e-4)) {
        std::cout << "  Error: Joint position differ" << std::endl;
        std::cout << "\nTest failed" << std::endl;
        return -1;
      }
      std::cout << "  They are the same, we can continue" << std::endl;
    }

    //********* Position control in tool frame
    if (1) {
      // from the current position move the tool frame
      vpHomogeneousMatrix tMt;
      // tMt[0][3] = 0.05; // along x_t
      tMt[1][3] = 0.05; // along y_t
      //  tMt[2][3] = 0.05; // along z_t

      vpHomogeneousMatrix fMt_ = fMt * tMt; // New position to reach
      robot.getInverseKinematics(fMt_, q);

      std::cout << "fMt_:\n" << fMt_ << std::endl;

      std::cout << "Move robot in joint position to reach fMt_" << std::endl;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setPosition(vpRobotViper650::ARTICULAR_FRAME, q);

      vpPoseVector fpt_;
      robot.getPosition(vpRobot::REFERENCE_FRAME, fpt_);

      std::cout << "fpt_:\n" << vpHomogeneousMatrix(fpt_) << std::endl;

      std::cout << "Compare pose fMt_ and fpt_:" << std::endl;
      if (!pose_equal(fMt_, vpHomogeneousMatrix(fpt_), 1e-4)) {
        std::cout << "  Error: Pose fMt_ differ" << std::endl;
        std::cout << "\nTest failed" << std::endl;
        return -1;
      }
      std::cout << "  They are the same, we can continue" << std::endl;
    }

    //********* Velocity control in tool frame along z
    if (1) {
      double t_init = vpTime::measureTimeMs();
      vpColVector v_t(6);
      v_t = 0;
      // v_t[2] = 0.01; // translation velocity along z_t
      v_t[5] = vpMath::rad(5); // rotation velocity along z_t

      std::cout << "Move robot in camera velocity" << std::endl;
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
      while (vpTime::measureTimeMs() - t_init < 6000) {
        // std::cout << "send vel: " << v_t() << std::endl;
        robot.setVelocity(vpRobotViper650::CAMERA_FRAME, v_t);
      }
    }

    //********* Velocity control in tool frame along z using joint velocity
    if (1) {
      // We need to stop the robot before changing velocity control from joint
      // to cartesian
      robot.setRobotState(vpRobot::STATE_STOP);
      vpVelocityTwistMatrix tVe(eMt.inverse());
      vpMatrix eJe;

      double t_init = vpTime::measureTimeMs();
      vpColVector v_t(6), q_dot;
      v_t = 0;
      // v_t[2] = -0.01; // translation velocity along z_t
      v_t[5] = vpMath::rad(-5); // rotation velocity along z_t

      std::cout << "Move robot in joint velocity" << std::endl;
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
      while (vpTime::measureTimeMs() - t_init < 6000) {
        robot.get_eJe(eJe);
        vpMatrix tJt = tVe * eJe;
        q_dot = tJt.pseudoInverse() * v_t;
        // std::cout << "send vel: " << q_dot.t() << std::endl;
        robot.setVelocity(vpRobotViper650::ARTICULAR_FRAME, q_dot);
      }
    }

    //********* Velocity control in tool frame along x
    if (1) {
      robot.setRobotState(vpRobot::STATE_STOP);
      double t_init = vpTime::measureTimeMs();
      vpColVector v_t(6);
      v_t = 0;
      v_t[3] = vpMath::rad(5); // rotation velocity along x_t

      std::cout << "Move robot in camera velocity" << std::endl;
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
      while (vpTime::measureTimeMs() - t_init < 3000) {
        // std::cout << "send vel: " << v_t() << std::endl;
        robot.setVelocity(vpRobotViper650::CAMERA_FRAME, v_t);
      }
    }

    //********* Velocity control in tool frame along x using joint velocity
    if (1) {
      // We need to stop the robot before changing velocity control from joint
      // to cartesian
      robot.setRobotState(vpRobot::STATE_STOP);
      vpVelocityTwistMatrix tVe(eMt.inverse());
      vpMatrix eJe;

      double t_init = vpTime::measureTimeMs();
      vpColVector v_t(6), q_dot;
      v_t = 0;
      v_t[3] = vpMath::rad(-5); // rotation velocity along x_t

      std::cout << "Move robot in joint velocity" << std::endl;
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
      while (vpTime::measureTimeMs() - t_init < 3000) {
        robot.get_eJe(eJe);
        vpMatrix tJt = tVe * eJe;
        q_dot = tJt.pseudoInverse() * v_t;
        // std::cout << "send vel: " << q_dot.t() << std::endl;
        robot.setVelocity(vpRobotViper650::ARTICULAR_FRAME, q_dot);
      }
    }

    //********* Position control in tool frame
    if (1) {
      robot.setRobotState(vpRobot::STATE_STOP);
      // get current position
      robot.getPosition(vpRobotViper650::ARTICULAR_FRAME, q);

      robot.get_fMc(q, fMt);

      vpHomogeneousMatrix tMt; // initialized to identity
      // tMt[0][3] = -0.05; // along x_t
      tMt[1][3] = -0.05; // along y_t
      //  tMt[2][3] = -0.05; // along z_t

      robot.getInverseKinematics(fMt * tMt, q);

      std::cout << "Move robot in joint position" << std::endl;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setPosition(vpRobotViper650::ARTICULAR_FRAME, q);
    }
    std::cout << "The end" << std::endl;
    std::cout << "Test succeed" << std::endl;
  } catch (const vpException &e) {
    std::cout << "Test failed with exception: " << e.getMessage() << std::endl;
  }
}

#else
int main()
{
  std::cout << "The real Viper650 robot controller is not available." << std::endl;
  return 0;
}

#endif
