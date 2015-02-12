/****************************************************************************
 *
 * $Id: testFeature.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Visual feature manipulation (segment).
 *
 * Author:
 * Filip Novotny
 *
 *****************************************************************************/

#include <fstream>
#include <iostream>
#include <vector>
#include <numeric>

#include <visp/vpConfig.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureSegment.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpMath.h>
#include <visp/vpParseArgv.h>
#include <visp/vpPlot.h>
#include <visp/vpPoint.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpServo.h> //visual servoing task

/*!

  \example testFeatureSegment.cpp

  Shows how to build a task with a segment visual feature.

*/
int main(int argc, const char **argv)
{  
  try {
#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
    int opt_no_display = 0;
    int opt_curves = 1;
#endif
    int opt_normalized = 1;

    // Parse the command line to set the variables
    vpParseArgv::vpArgvInfo argTable[] =
    {
  #if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
      {"-d", vpParseArgv::ARGV_CONSTANT, 0, (char *) &opt_no_display,
       "Disable display and graphics viewer."},
  #endif
      {"-normalized", vpParseArgv::ARGV_INT, (char*) NULL, (char *) &opt_normalized,
       "1 to use normalized features, 0 for non normalized."},
      {"-h", vpParseArgv::ARGV_HELP, (char*) NULL, (char *) NULL,
       "Print the help."},
      {(char*) NULL, vpParseArgv::ARGV_END, (char*) NULL, (char*) NULL, (char*) NULL}
    } ;

    // Read the command line options
    if(vpParseArgv::parse(&argc, argv, argTable,
                          vpParseArgv::ARGV_NO_LEFTOVERS |
                          vpParseArgv::ARGV_NO_ABBREV |
                          vpParseArgv::ARGV_NO_DEFAULTS)) {
      return (false);
    }

    std::cout << "Used options: " << std::endl;
#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
    opt_curves = (opt_no_display == 0) ? 1 : 0;
    std::cout << " - no display: " << opt_no_display << std::endl;
    std::cout << " - curves    : " << opt_curves << std::endl;
#endif
    std::cout << " - normalized: " << opt_normalized << std::endl;

    vpCameraParameters cam(640.,480.,320.,240.);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
    vpDisplay *display = NULL;
    if (!opt_no_display) {
#if defined(VISP_HAVE_X11)
      display = new vpDisplayX;
#elif defined VISP_HAVE_GDI
      display = new vpDisplayGDI;
#endif
    }
#endif
    vpImage<unsigned char> I(480,640,0);

#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
    if (!opt_no_display)
      display->init(I);
#endif

    vpHomogeneousMatrix cMo (-0.5, 0.5, 4., vpMath::rad(10), vpMath::rad(20), vpMath::rad(90));
    vpHomogeneousMatrix cdMo(0., 0., 1., vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));

    vpPoint P[4]; // 4 points in the object frame
    P[0].setWorldCoordinates( .1,  .1, 0.);
    P[1].setWorldCoordinates(-.1,  .1, 0.);
    P[2].setWorldCoordinates(-.1, -.1, 0.);
    P[3].setWorldCoordinates( .1, -.1, 0.);

    vpPoint Pd[4]; // 4 points in the desired camera frame
    for (int i=0; i<4; i++) {
      Pd[i] = P[i];
      Pd[i].project(cdMo);
    }
    vpPoint Pc[4]; // 4 points in the current camera frame
    for (int i=0; i<4; i++) {
      Pc[i] = P[i];
      Pc[i].project(cMo);
    }

    vpFeatureSegment seg_cur[2], seg_des[2]; // Current and desired features
    for (int i=0; i <2; i++)
    {
      if (opt_normalized) {
        seg_cur[i].setNormalized(true);
        seg_des[i].setNormalized(true);
      }
      else {
        seg_cur[i].setNormalized(false);
        seg_des[i].setNormalized(false);
      }
      vpFeatureBuilder::create(seg_cur[i], Pc[i*2], Pc[i*2+1]);
      vpFeatureBuilder::create(seg_des[i], Pd[i*2], Pd[i*2+1]);
      seg_cur[i].print();
      seg_des[i].print();
    }

    //define visual servoing task
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(1) ;

    for (int i=0; i <2; i++)
      task.addFeature(seg_cur[i], seg_des[i]);

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
    if (!opt_no_display) {
      vpDisplay::display(I);
      for (int i=0; i <2; i++) {
        seg_cur[i].display(cam, I, vpColor::red);
        seg_des[i].display(cam, I, vpColor::green);
        vpDisplay::flush(I);
      }
    }
#endif

#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
    vpPlot *graph = NULL;
    if (opt_curves)
    {
      //Create a window (700 by 700) at position (100, 200) with two graphics
      graph = new vpPlot(2, 500, 500, 700, 10, "Curves...");

      //The first graphic contains 3 curve and the second graphic contains 3 curves
      graph->initGraph(0,6);
      graph->initGraph(1,8);
      //     graph->setTitle(0, "Velocities");
      //     graph->setTitle(1, "Error s-s*");
    }
#endif

    //param robot
    vpRobotCamera robot ;
    float sampling_time = 0.010f ; // Sampling period in seconds
    robot.setSamplingTime(sampling_time) ;
    robot.setPosition(cMo) ;
    int iter=0;

    do{
      double t = vpTime::measureTimeMs();
      robot.getPosition(cMo);
      for (int i=0; i <4; i++)
        Pc[i].project(cMo);

      for (int i=0; i <2; i++)
        vpFeatureBuilder::create(seg_cur[i], Pc[i*2], Pc[i*2+1]);

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
      if (!opt_no_display) {
        vpDisplay::display(I);
        for (int i=0; i <2; i++) {
          seg_cur[i].display(cam, I, vpColor::red);
          seg_des[i].display(cam, I, vpColor::green);
          vpDisplay::flush(I);
        }
      }
#endif

      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
      if (opt_curves)
      {
        graph->plot(0, iter, v); // plot velocities applied to the robot
        graph->plot(1, iter, task.getError()); // plot error vector
      }
#endif

      vpTime::wait(t, sampling_time * 1000); // Wait 10 ms
      iter ++;

    } while(( task.getError() ).sumSquare() > 0.0005);

    // A call to kill() is requested here to destroy properly the current
    // and desired feature lists.
    task.kill();

#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
    if (graph != NULL)
      delete graph;
#endif
#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
    if (!opt_no_display && display != NULL)
      delete display;
#endif

    std::cout << "final error=" << ( task.getError() ).sumSquare() << std::endl;
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
