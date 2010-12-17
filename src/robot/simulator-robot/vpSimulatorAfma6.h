/****************************************************************************
 *
 * $Id: vpSimulatorAfma6.h 2598 2010-06-02 09:20:22Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Class which provides a simulator for the robot Afma6.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpSimulatorAfma6_HH
#define vpSimulatorAfma6_HH

/*!
  \file vpSimulatorAfma6.h
  \brief Class which provides a simulator for the robot Afma6.
*/

#include <visp/vpConfig.h>
#include <visp/vpRobotSimulator.h>
#include <visp/vpAfma6.h>

#if defined(WIN32) || defined(VISP_HAVE_PTHREAD)

/*!
  \class vpSimulatorAfma6

  \ingroup Afma6 RealRobotSimu


  \brief Simulator of Irisa's gantry robot named Afma6.

  Implementation of the vpRobotSimulator class in order to simulate Irisa's
  Afma6 robot. This robot is a gantry robot with six degrees of
  freedom manufactured in 1992 by the french Afma-Robots company.

  \warning This class uses threading capabilities. Thus on Unix-like
  platforms, the libpthread third-party library need to be
  installed. On Windows, we use the native threading capabilities.

  This class allows to control the Afma6 gantry robot in position
  and velocity:
  - in the joint space (vpRobot::ARTICULAR_FRAME), 
  - in the fixed reference frame (vpRobot::REFERENCE_FRAME), 
  - in the camera frame (vpRobot::CAMERA_FRAME),
  - or in a mixed frame (vpRobot::MIXT_FRAME) where translations are expressed 
  in the reference frame and rotations in the camera frame.

  All the translations are expressed in meters for positions and m/s
  for the velocities. Rotations are expressed in radians for the
  positions, and rad/s for the rotation velocities.

  The direct and inverse kinematics models are implemented in the
  vpAfma6 class.

  To control the robot in position, you may set the controller
  to position control and then send the position to reach in a specific
  frame like here in the joint space:

  \code
#include <visp/vpConfig.h>
#include <visp/vpSimulatorAfma6.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

int main()
{
  vpSimulatorAfma6 robot;
  
  robot.init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithoutDistortion);

  vpColVector q(6);
  // Set a joint position
  q[0] = 0.1;             // Joint 1 position, in meter
  q[1] = 0.2;             // Joint 2 position, in meter
  q[2] = 0.3;             // Joint 3 position, in meter
  q[3] = M_PI/8;          // Joint 4 position, in rad
  q[4] = M_PI/4;          // Joint 5 position, in rad
  q[5] = M_PI;            // Joint 6 position, in rad

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Moves the robot in the joint space
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
  
  return 0;
}
  \endcode

  To control the robot in velocity, you may set the controller to
  velocity control and then send the velocities. To end the velocity
  control and stop the robot you have to set the controller to the
  stop state. Here is an example of a velocity control in the joint
  space:

  \code
#include <visp/vpConfig.h>
#include <visp/vpSimulatorAfma6.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

int main()
{
  vpSimulatorAfma6 robot;
  
  robot.init(vpAfma6::TOOL_GRIPPER, vpCameraParameters::perspectiveProjWithoutDistortion);

  vpColVector qvel(6);
  // Set a joint velocity
  qvel[0] = 0.1;             // Joint 1 velocity in m/s
  qvel[1] = 0.1;             // Joint 2 velocity in m/s
  qvel[2] = 0.1;             // Joint 3 velocity in m/s
  qvel[3] = M_PI/8;          // Joint 4 velocity in rad/s
  qvel[4] = 0;               // Joint 5 velocity in rad/s
  qvel[5] = 0;               // Joint 6 velocity in rad/s

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  while (1) {
    // Apply a velocity in the joint space
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qvel);

    // Compute new velocities qvel...
  }

  // Stop the robot
  robot.setRobotState(vpRobot::STATE_STOP);
  
  return 0;
}
  \endcode

  It is also possible to measure the robot current position with
  getPosition() method and the robot current velocities with the getVelocity()
  method.

  For convenience, there is also the ability to read/write joint
  positions from a position file with readPosFile() and savePosFile()
  methods.
*/


class VISP_EXPORT vpSimulatorAfma6 : public vpRobotSimulator, public vpAfma6
{
  public:
    static const double defaultPositioningVelocity;
    
  private:
    
    
    vpColVector q_prev_getdis;
    bool first_time_getdis;
    
    double positioningVelocity;
    
    vpColVector zeroPos;
    vpColVector reposPos;
    
    bool toolCustom;
    
  public:
    vpSimulatorAfma6();
    vpSimulatorAfma6(bool display);
    virtual ~vpSimulatorAfma6();
    
    void init (vpAfma6::vpAfma6ToolType tool, vpCameraParameters::vpCameraParametersProjType projModel=vpCameraParameters::perspectiveProjWithoutDistortion);
    
    vpRobot::vpRobotStateType setRobotState (const vpRobot::vpRobotStateType newState);
    
    void getCameraParameters(vpCameraParameters &cam,
			   const unsigned int &image_width,
			   const unsigned int &image_height);
    void getCameraParameters(vpCameraParameters &cam,
			   const vpImage<unsigned char> &I);
    void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I);
    void setCameraParameters(const vpCameraParameters cam) ;
    
    void setVelocity (const vpRobot::vpControlFrameType frame, const vpColVector & velocity);
    void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q);
    vpColVector getVelocity (const vpRobot::vpControlFrameType frame);
    
    void setPositioningVelocity (const double velocity) {positioningVelocity = velocity;}
    void setPosition(const vpRobot::vpControlFrameType frame,const vpColVector &q);
    void setPosition (const vpRobot::vpControlFrameType frame,
				   const double pos1,
				   const double pos2,
				   const double pos3,
				   const double pos4,
				   const double pos5,
				   const double pos6);
    void setPosition(const char *filename);
    void setPosition(const vpHomogeneousMatrix &cdMo, vpImage<unsigned char> *Iint=NULL);
    
    void setJointLimit(vpColVector limitMin, vpColVector limitMax);
    
    double getPositioningVelocity (void){return positioningVelocity;}
    
    void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);
    void getPosition (const vpRobot::vpControlFrameType frame,
		    vpPoseVector &position);

    void getCameraDisplacement(vpColVector &displacement);
    void getArticularDisplacement(vpColVector &displacement);
    void getDisplacement(const vpRobot::vpControlFrameType frame, 
		       vpColVector &displacement);
    
    static bool readPosFile(const char *filename, vpColVector &q);
    static bool savePosFile(const char *filename, const vpColVector &q);
    void move(const char *filename) ;
    
    void get_cMe(vpHomogeneousMatrix &cMe);
    void get_cVe(vpVelocityTwistMatrix &cVe);
    void get_eJe(vpMatrix &eJe);
    void get_fJe(vpMatrix &fJe);
    
    void stopMotion();
    
    void initialiseCameraRelativeToObject(vpHomogeneousMatrix cMo);
    void initialiseObjectRelativeToCamera(vpHomogeneousMatrix cMo);
    
  protected:
    void updateArticularPosition();
    void computeArticularVelocity();
    void findHighestPositioningSpeed(vpColVector &q);
    void init();
    bool singularityTest(const vpColVector q, vpMatrix &J);
    int isInJointLimit ();
    
    void initDisplay();
    void initArms();
    void getExternalImage(vpImage<vpRGBa> &I);
    
    inline void get_fMi(vpHomogeneousMatrix *fMit) {
    #if defined(WIN32)
	  WaitForSingleObject(mutex_fMi,INFINITE);
      for (int i = 0; i < 8; i++)
        fMit[i] = fMi[i];
      ReleaseMutex(mutex_fMi);
    #elif defined(VISP_HAVE_PTHREAD)
      pthread_mutex_lock (&mutex_fMi);
      for (int i = 0; i < 8; i++)
        fMit[i] = fMi[i];
      pthread_mutex_unlock (&mutex_fMi);
    #endif
    }
      
    void compute_fMi();
};

#endif

#endif
