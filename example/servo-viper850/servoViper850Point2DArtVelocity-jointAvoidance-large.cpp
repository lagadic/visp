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
 * Eric Marchand
 * Fabien Spindler
 * Giovanni Claudio
 *
 *****************************************************************************/

/*!
  \example servoViper850Point2DArtVelocity-jointAvoidance-large.cpp

  Joint limits avoidance using a secondary task for joint limit avoidance
  \cite Marey:2010b using the new large projection operator (see equation(24)
  in the paper \cite Marey:2010).
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#if (defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_DISPLAY))

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
  try {
    vpRobotViper850 robot;

    vpServo task;

    vpImage<unsigned char> I;

    bool reset = false;
    vp1394TwoGrabber g(reset);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I);

    g.acquire(I);

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 800, 100, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, 800, 100, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 800, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpColVector jointMin(6), jointMax(6);
    jointMin = robot.getJointMin();
    jointMax = robot.getJointMax();

    vpColVector Qmiddle(6);
    vpColVector data(12);

    Qmiddle = (jointMin + jointMax) / 2.;
    //    double rho1 = 0.1 ;

    double rho = 0.1;
    double rho1 = 0.3;

    vpColVector q(6);

    // Create a window with two graphics
    // - first graphic to plot q(t), Qmin, Qmax, Ql0min, Ql1min, Ql0max and
    // Ql1max
    vpPlot plot(2);

    // The first graphic contains 12 data to plot: q(t), Low Limits, Upper
    // Limits, ql0min, ql1min, ql0max and ql1max
    plot.initGraph(0, 12);
    // The second graphic contains the values of the secondaty task velocities
    plot.initGraph(1, 6);

    // For the first graphic :
    // - along the x axis the expected values are between 0 and 200
    // - along the y axis the expected values are between -1.2 and 1.2
    plot.initRange(0, 0., 200., -1.2, 1.2);
    plot.setTitle(0, "Joint behavior");

    // For the second graphic :
    plot.setTitle(1, "Q secondary task");

    // For the first and second graphic, set the curves legend
    char legend[10];
    for (unsigned int i = 0; i < 6; i++) {
      sprintf(legend, "q%u", i + 1);
      plot.setLegend(0, i, legend);
      plot.setLegend(1, i, legend);
    }
    plot.setLegend(0, 6, "Low Limit");
    plot.setLegend(0, 7, "Upper Limit");
    plot.setLegend(0, 8, "ql0 min");
    plot.setLegend(0, 9, "ql0 max");
    plot.setLegend(0, 10, "ql1 min");
    plot.setLegend(0, 11, "ql1 max");

    // Set the curves color
    plot.setColor(0, 0, vpColor::red);
    plot.setColor(0, 1, vpColor::green);
    plot.setColor(0, 2, vpColor::blue);
    plot.setColor(0, 3, vpColor::orange);
    plot.setColor(0, 4, vpColor(0, 128, 0));
    plot.setColor(0, 5, vpColor::cyan);
    for (unsigned int i = 6; i < 12; i++)
      plot.setColor(0, i, vpColor::black); // for Q and tQ [min,max]

    vpColVector sec_task(6);

    vpDot2 dot;

    std::cout << "Click on a dot..." << std::endl;
    dot.initTracking(I);
    vpImagePoint cog = dot.getCog();
    vpDisplay::displayCross(I, cog, 10, vpColor::blue);
    vpDisplay::flush(I);

    vpCameraParameters cam;
    // Update camera parameters
    robot.getCameraParameters(cam, I);

    // sets the current position of the visual feature
    vpFeaturePoint p;
    vpFeatureBuilder::create(p, cam, dot); // retrieve x,y and Z of the vpPoint structure

    p.set_Z(1);
    // sets the desired position of the visual feature
    vpFeaturePoint pd;
    pd.buildFrom(0, 0, 1);

    // Define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

    vpVelocityTwistMatrix cVe;
    robot.get_cVe(cVe);
    std::cout << cVe << std::endl;
    task.set_cVe(cVe);

    // - Set the Jacobian (expressed in the end-effector frame)") ;
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    // - we want to see a point on a point..") ;
    std::cout << std::endl;
    task.addFeature(p, pd);

    // - set the gain
    task.setLambda(0.8);

    // Display task information " ) ;
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    int iter = 0;
    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    for (;;) {
      iter++;
      // Acquire a new image from the camera
      g.acquire(I);

      // Display this image
      vpDisplay::display(I);

      // Achieve the tracking of the dot in the image
      dot.track(I);
      cog = dot.getCog();

      // Display a green cross at the center of gravity position in the image
      vpDisplay::displayCross(I, cog, 10, vpColor::green);

      // Get the measured joint positions of the robot
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q);

      // Update the point feature from the dot location
      vpFeatureBuilder::create(p, cam, dot);

      // Get the jacobian of the robot
      robot.get_eJe(eJe);
      // Update this jacobian in the task structure. It will be used to
      // compute the velocity skew (as an articular velocity) qdot = -lambda *
      // L^+ * cVe * eJe * (s-s*)
      task.set_eJe(eJe);

      vpColVector prim_task;
      // Compute the visual servoing skew vector
      prim_task = task.computeControlLaw();

      // Compute the secondary task for the joint limit avoidance
      sec_task = task.secondaryTaskJointLimitAvoidance(q, prim_task, jointMin, jointMax, rho, rho1);

      vpColVector v;
      v = prim_task + sec_task;

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed joint velocities to the robot
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

      {
        // Add the material to plot curves

        // q normalized between (entre -1 et 1)
        for (unsigned int i = 0; i < 6; i++) {
          data[i] = (q[i] - Qmiddle[i]);
          data[i] /= (jointMax[i] - jointMin[i]);
          data[i] *= 2;
        }

        data[6] = -1.0;
        data[7] = 1.0;

        unsigned int joint = 2;
        double tQmin_l0 = jointMin[joint] + rho * (jointMax[joint] - jointMin[joint]);
        double tQmax_l0 = jointMax[joint] - rho * (jointMax[joint] - jointMin[joint]);

        double tQmin_l1 = tQmin_l0 - rho * rho1 * (jointMax[joint] - jointMin[joint]);
        double tQmax_l1 = tQmax_l0 + rho * rho1 * (jointMax[joint] - jointMin[joint]);

        data[8] = 2 * (tQmin_l0 - Qmiddle[joint]) / (jointMax[joint] - jointMin[joint]);
        data[9] = 2 * (tQmax_l0 - Qmiddle[joint]) / (jointMax[joint] - jointMin[joint]);
        data[10] = 2 * (tQmin_l1 - Qmiddle[joint]) / (jointMax[joint] - jointMin[joint]);
        data[11] = 2 * (tQmax_l1 - Qmiddle[joint]) / (jointMax[joint] - jointMin[joint]);
        plot.plot(0, iter, data);     // plot q(t), Low Limits, Upper Limits,
                                      // ql0min, ql1min, ql0max and ql1max
        plot.plot(1, iter, sec_task); // plot secondary task velocities
      }

      vpDisplay::flush(I);
    }

    // Display task information
    task.print();
    task.kill();
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have an Viper 850 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
