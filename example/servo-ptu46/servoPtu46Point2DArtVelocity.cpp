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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in articular
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoPtu46Point2DArtVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  ptu-46 robot (pan-tilt head provided by Directed Perception). The velocity
  is computed in articular. The visual feature is the center of gravity of a
  point.

*/

/*!
  \example servoPtu46Point2DArtVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the ptu-46
  robot (pan-tilt head provided by Directed Perception). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <unistd.h>
#endif
#include <signal.h>

#if (defined(VISP_HAVE_PTU46) & defined(VISP_HAVE_DC1394))

#ifdef VISP_HAVE_PTHREAD
#include <pthread.h>
#endif

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#include <visp3/robot/vpRobotPtu46.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp3/blob/vpDot2.h>

#ifdef VISP_HAVE_PTHREAD
pthread_mutex_t mutexEndLoop = PTHREAD_MUTEX_INITIALIZER;
#endif

void signalCtrC(int signumber)
{
  (void)(signumber);
#ifdef VISP_HAVE_PTHREAD
  pthread_mutex_unlock(&mutexEndLoop);
#endif
  usleep(1000 * 10);
  vpTRACE("Ctrl-C pressed...");
}

int main()
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << " Test program for vpServo " << std::endl;
  std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
  std::cout << " Simulation " << std::endl;
  std::cout << " task : servo a point " << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  try {

#ifdef VISP_HAVE_PTHREAD
    pthread_mutex_lock(&mutexEndLoop);
#endif
    signal(SIGINT, &signalCtrC);

    vpRobotPtu46 robot;
    {
      vpColVector q(2);
      q = 0;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
    }

    vpImage<unsigned char> I;

    vp1394TwoGrabber g;

    g.open(I);

    try {
      g.acquire(I);
    } catch (...) {
      vpERROR_TRACE(" Error caught");
      return (-1);
    }

    vpDisplayX display(I, 100, 100, "testDisplayX.cpp ");
    vpTRACE(" ");

    try {
      vpDisplay::display(I);
      vpDisplay::flush(I);
    } catch (...) {
      vpERROR_TRACE(" Error caught");
      return (-1);
    }

    vpServo task;

    vpDot2 dot;

    try {
      vpERROR_TRACE("start dot.initTracking(I) ");
      vpImagePoint germ;
      vpDisplay::getClick(I, germ);
      dot.setCog(germ);
      vpDEBUG_TRACE(25, "Click!");
      // dot.initTracking(I) ;
      dot.track(I);
      vpERROR_TRACE("after dot.initTracking(I) ");
    } catch (...) {
      vpERROR_TRACE(" Error caught ");
      return (-1);
    }

    vpCameraParameters cam;

    vpTRACE("sets the current position of the visual feature ");
    vpFeaturePoint p;
    vpFeatureBuilder::create(p, cam, dot); // retrieve x,y and Z of the vpPoint structure

    p.set_Z(1);
    vpTRACE("sets the desired position of the visual feature ");
    vpFeaturePoint pd;
    pd.buildFrom(0, 0, 1);

    vpTRACE("define the task");
    vpTRACE("\t we want an eye-in-hand control law");
    vpTRACE("\t articular velocity are computed");
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

    vpTRACE("Set the position of the camera in the end-effector frame ");
    vpHomogeneousMatrix cMe;
    //  robot.get_cMe(cMe) ;

    vpVelocityTwistMatrix cVe;
    robot.get_cVe(cVe);
    std::cout << cVe << std::endl;
    task.set_cVe(cVe);

    vpDisplay::getClick(I);
    vpTRACE("Set the Jacobian (expressed in the end-effector frame)");
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    vpTRACE("\t we want to see a point on a point..");
    std::cout << std::endl;
    task.addFeature(p, pd);

    vpTRACE("\t set the gain");
    task.setLambda(0.1);

    vpTRACE("Display task information ");
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    unsigned int iter = 0;
    vpTRACE("\t loop");
#ifdef VISP_HAVE_PTHREAD
    while (0 != pthread_mutex_trylock(&mutexEndLoop))
#else
    for (;;)
#endif
    {
      std::cout << "---------------------------------------------" << iter << std::endl;

      g.acquire(I);
      vpDisplay::display(I);

      dot.track(I);

      //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
      //			   10,vpColor::green) ;

      vpFeatureBuilder::create(p, cam, dot);

      // get the jacobian
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      //  std::cout << (vpMatrix)cVe*eJe << std::endl ;

      vpColVector v;
      v = task.computeControlLaw();

      vpServoDisplay::display(task, cam, I);
      std::cout << v.t();
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);
      vpDisplay::flush(I);

      vpTRACE("\t\t || s - s* || = %f ", (task.getError()).sumSquare());
    }

    vpTRACE("Display task information ");
    task.print();
    task.kill();
  }
  catch (const vpException &e) {
    std::cout << "Sorry PtU46 not available. Got exception: " << e << std::endl;
    return EXIT_FAILURE
  }
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "You do not have an PTU46 PT robot connected to your computer..." << std::endl;
}
#endif
