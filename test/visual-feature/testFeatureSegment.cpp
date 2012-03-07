/****************************************************************************
 *
 * $Id: testFeature.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpFeatureSegment.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h> //visual servoing task
#include <visp/vpRobotCamera.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <numeric>

/*!

  \example testFeatureSegment.cpp

  Shows how to build a task with a segment visual feature.

*/
int main(int argc, const char **argv)
{  
  bool opt_display = true;
  if (argc == 2)
    opt_display = false;

  vpCameraParameters cam(640.,480.,320.,240.);
#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
#endif
  vpImage<unsigned char> I(480,640,0);

#if (defined (VISP_HAVE_X11) || defined (VISP_HAVE_GDI))
  if (opt_display)
    display.init(I);
#endif

  vpHomogeneousMatrix cMo(-.5, -.5, 2., vpMath::rad(50), vpMath::rad(60), vpMath::rad(70));
  vpHomogeneousMatrix cdMo(0., 0., 1., vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
  vpPoint p1_cur, p2_cur, p1_dst, p2_dst;
  
  p1_cur.setWorldCoordinates(.1, .1, 0.);
  p2_cur.setWorldCoordinates(.3, .2, 0.);

  p1_dst.setWorldCoordinates(.1, .1, 0.);
  p2_dst.setWorldCoordinates(.3, .2, 0.);
  
  p1_cur.project(cMo);
  p2_cur.project(cMo);

  p1_dst.project(cdMo);
  p2_dst.project(cdMo);
  
  vpFeatureSegment seg_cur(p2_cur, p1_cur);
  vpFeatureSegment seg_dst(p2_dst, p1_dst);
  seg_cur.print();
  seg_dst.print();
  
  //define visual servoing task
  vpServo task;
  task.setServo(vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType(vpServo::CURRENT);
  task.setLambda(1) ;

  task.addFeature(seg_cur,seg_dst);  
  
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
  if (opt_display) {
    seg_cur.display(cam,I, vpColor::red);
    seg_dst.display(cam,I, vpColor::green);
    vpDisplay::flush(I);
  }
#endif
  
  //param robot
  vpRobotCamera robot ;
  float sampling_time = 0.010f ; // Sampling period in seconds
  robot.setSamplingTime(sampling_time) ;
  robot.setPosition(cMo) ;
  float iter=0.;

  do{
    robot.getPosition(cMo);
    p1_cur.project(cMo);
    p2_cur.project(cMo);

    seg_cur.buildFrom(p2_cur, p1_cur);
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
    if (opt_display) {
      vpDisplay::display(I);
      seg_cur.display(cam, I, vpColor::red);
      seg_dst.display(cam, I, vpColor::green);
      vpDisplay::flush(I);
    }
#endif

    vpColVector v = task.computeControlLaw();
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
    
    double t = vpTime::measureTimeMs();
    vpTime::wait(t, sampling_time * 1000); // Wait 10 ms    
    iter+=sampling_time;
    
  } while(task.error.sumSquare() > 0.00005);
  
  // A call to kill() is requested here to destroy properly the current
  // and desired feature lists.
  task.kill();

  std::cout << "final error=" << task.error.sumSquare() << std::endl;  
}
