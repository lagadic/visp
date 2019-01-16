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
 *
 *****************************************************************************/

/*!
  \example servoViper850Point2DArtVelocity-jointAvoidance-gpa.cpp

  Joint limits avoidance using a gradient projection approach.

  Implemented from \cite Marchand96f and section II.B in \cite Chaumette01c.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#if (defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394) && defined(VISP_HAVE_DISPLAY))

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

    vpColVector Qmin(6), tQmin(6);
    vpColVector Qmax(6), tQmax(6);
    vpColVector Qmiddle(6);
    vpColVector data(10);

    double rho = 0.15;
    for (unsigned int i = 0; i < 6; i++) {
      Qmin[i] = jointMin[i] + 0.5 * rho * (jointMax[i] - jointMin[i]);
      Qmax[i] = jointMax[i] - 0.5 * rho * (jointMax[i] - jointMin[i]);
    }
    Qmiddle = (Qmin + Qmax) / 2.;
    double rho1 = 0.1;

    for (unsigned int i = 0; i < 6; i++) {
      tQmin[i] = Qmin[i] + 0.5 * (rho1) * (Qmax[i] - Qmin[i]);
      tQmax[i] = Qmax[i] - 0.5 * (rho1) * (Qmax[i] - Qmin[i]);
    }

    vpColVector q(6);

    // Create a window with two graphics
    // - first graphic to plot q(t), Qmin, Qmax, tQmin and tQmax
    // - second graphic to plot the cost function h_s
    vpPlot plot(2);

    // The first graphic contains 10 data to plot: q(t), Qmin, Qmax, tQmin and
    // tQmax
    plot.initGraph(0, 10);
    // The second graphic contains 1 curve, the cost function h_s
    plot.initGraph(1, 1);

    // For the first graphic :
    // - along the x axis the expected values are between 0 and 200
    // - along the y axis the expected values are between -1.2 and 1.2
    plot.initRange(0, 0., 200., -1.2, 1.2);
    plot.setTitle(0, "Joint behavior");

    // For the second graphic :
    // - along the x axis the expected values are between 0 and 200 and
    //   the step is 1
    // - along the y axis the expected values are between 0 and 0.0001 and the
    //   step is 0.00001
    plot.initRange(1, 0., 200., 0., 1e-4);
    plot.setTitle(1, "Cost function");

    // For the first graphic, set the curves legend
    char legend[10];
    for (unsigned int i = 0; i < 6; i++) {
      sprintf(legend, "q%u", i + 1);
      plot.setLegend(0, i, legend);
    }
    plot.setLegend(0, 6, "tQmin");
    plot.setLegend(0, 7, "tQmax");
    plot.setLegend(0, 8, "Qmin");
    plot.setLegend(0, 9, "Qmax");

    // Set the curves color
    plot.setColor(0, 0, vpColor::red);
    plot.setColor(0, 1, vpColor::green);
    plot.setColor(0, 2, vpColor::blue);
    plot.setColor(0, 3, vpColor::orange);
    plot.setColor(0, 4, vpColor(0, 128, 0));
    plot.setColor(0, 5, vpColor::cyan);
    for (unsigned int i = 6; i < 10; i++)
      plot.setColor(0, i, vpColor::black); // for Q and tQ [min,max]

    // For the second graphic, set the curves legend
    plot.setLegend(1, 0, "h_s");

    double beta = 1;

    // Set the amplitude of the control law due to the secondary task
    std::cout << " Give the parameters beta (1) : ";
    std::cin >> beta;

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
      vpColVector e2(6);
      // Compute the visual servoing skew vector
      prim_task = task.computeControlLaw();

      vpColVector sec_task(6);
      double h_s = 0;
      {
        // joint limit avoidance with secondary task

        vpColVector de2dt(6);
        de2dt = 0;
        e2 = 0;
        for (unsigned int i = 0; i < 6; i++) {
          double S = 0;
          if (q[i] > tQmax[i])
            S = q[i] - tQmax[i];
          if (q[i] < tQmin[i])
            S = q[i] - tQmin[i];
          double D = (Qmax[i] - Qmin[i]);
          h_s += vpMath::sqr(S) / D;
          e2[i] = S / D;
        }
        h_s = beta * h_s / 2.0; // cost function
        e2 *= beta;
        //	std::cout << e2.t() << std::endl;
        std::cout << "Cost function h_s: " << h_s << std::endl;

        sec_task = task.secondaryTask(e2, de2dt);
      }

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
          data[i] /= (Qmax[i] - Qmin[i]);
          data[i] *= 2;
        }
        unsigned int joint = 2;
        data[6] = 2 * (tQmin[joint] - Qmiddle[joint]) / (Qmax[joint] - Qmin[joint]);
        data[7] = 2 * (tQmax[joint] - Qmiddle[joint]) / (Qmax[joint] - Qmin[joint]);
        data[8] = -1;
        data[9] = 1;
        plot.plot(0, iter, data);   // plot q, Qmin, Qmax, tQmin, tQmax
        plot.plot(1, 0, iter, h_s); // plot the cost function
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
