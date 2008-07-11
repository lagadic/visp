/****************************************************************************
 *
 * $Id: manServo4PointsDisplay.cpp,v 1.1 2008-07-11 13:25:57 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Simulation of a visual servoing with display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file manServo4PointsDisplay
  \brief Visual servoing experiment on 4 points with a display.
*/

/*!
  \example manServo4PointsDisplay
  Visual servoing experiment on 4 points with a display.
*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_GTK

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDisplayGTK.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPose.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpIoTools.h>

int main()
{
//////////////////////////////////////////
// sets the initial camera location 
  vpHomogeneousMatrix cMo(0.3,0.2,3,
    vpMath::rad(0),vpMath::rad(0),vpMath::rad(40))  ;

///////////////////////////////////
// initialize the robot
  vpRobotCamera robot ;
  robot.setSamplingTime(0.04); // 40ms 
  robot.setPosition(cMo) ;

//initialize the camera parameters
  vpCameraParameters cam(800,800,240,180); 

//Image definition
  unsigned int height = 360 ;
  unsigned int width  = 480 ;
  vpImage<unsigned char> I(height,width);

//Display initialization
  vpDisplayGTK disp;
  disp.init(I,100,100,"Simulation display");

////////////////////////////////////////
// Desired visual features initialization

// sets the points coordinates in the object frame (in meter)
  vpPoint point[4] ;
  point[0].setWorldCoordinates(-0.1,-0.1,0) ;
  point[1].setWorldCoordinates(0.1,-0.1,0) ;
  point[2].setWorldCoordinates(0.1,0.1,0) ;
  point[3].setWorldCoordinates(-0.1,0.1,0) ;

// sets the desired camera location
  vpHomogeneousMatrix cMo_d(0,0,1,0,0,0) ;

// computes the 3D point coordinates in the camera frame and its 2D coordinates
  for (int i = 0 ; i < 4 ; i++)
    point[i].project(cMo_d) ;

// creates the associated features
  vpFeaturePoint pd[4] ;
  for (int i = 0 ; i < 4 ; i++)
    vpFeatureBuilder::create(pd[i],point[i]) ;


///////////////////////////////////////
// Current visual features initialization

// computes the 3D point coordinates in the camera frame and its 2D coordinates
  for (int i = 0 ; i < 4 ; i++)
    point[i].project(cMo) ;

// creates the associated features
 vpFeaturePoint p[4] ;
  for (int i = 0 ; i < 4 ; i++)
    vpFeatureBuilder::create(p[i],point[i])  ;


/////////////////////////////////
// Task defintion 
  vpServo task ;
// we want an eye-in-hand control law ;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;

// Set the position of the camera in the end-effector frame
  vpHomogeneousMatrix cMe ;
  vpTwistMatrix cVe(cMe) ;
  task.set_cVe(cVe) ;
// Set the Jacobian (expressed in the end-effector frame)
  vpMatrix eJe ;
  robot.get_eJe(eJe) ;
  task.set_eJe(eJe) ;

// we want to see a point on a point
  for (int i = 0 ; i < 4 ; i++)
    task.addFeature(p[i],pd[i]) ;
// Set the gain
  task.setLambda(1.0) ;
// Print the current information about the task
  task.print();


////////////////////////////////////////////////
// The control loop
  int k = 0;
  while(k++ < 200){
    double t = vpTime::measureTimeMs();
    
    // Display the image background
    vpDisplay::display(I);
    
    // Update the current features
    for (int i = 0 ; i < 4 ; i++)
    {
      point[i].project(cMo) ;
      vpFeatureBuilder::create(p[i],point[i])  ;
    }
    
    // Display the task features (current and desired)
    vpServoDisplay::display(task,cam,I);
    vpDisplay::flush(I);
    
    // Update the robot Jacobian
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;
    
    // Compute the control law
    vpColVector v = task.computeControlLaw() ;
    
    // Send the computed velocity to the robot and compute the new robot position
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ;
    robot.getPosition(cMo) ;
    
    // Print the current information about the task
      task.print();
    
    // Wait 40 ms
    vpTime::wait(t,40); 
  }  
  task.kill();
  return 0;
}

#endif