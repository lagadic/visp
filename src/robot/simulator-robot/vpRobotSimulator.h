/****************************************************************************
 *
 * $Id$
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
 * Basic class used to make robot simulators.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpRobotSimulator_HH
#define vpRobotSimulator_HH

/*!
  \file vpRobotSimulator.h
  \brief Basic class used to make robot simulators.
*/
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#include <visp/vpConfig.h>


#if defined(WIN32) || defined(VISP_HAVE_PTHREAD)

#if defined(WIN32)
#  include <windows.h>
#elif defined(VISP_HAVE_PTHREAD)
#  include <pthread.h>
#endif

#include <visp/vpWireFrameSimulator.h>
#include <visp/vpRobot.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>

/*!
  \class vpRobotSimulator

  \ingroup VisuRobotSimu
  
  \brief This class aims to be a basis used to create all the
  simulators of robots.
  
  Thus in this class you will find all the parameters and methods
  which are necessary to create a simulator. Several methods are pure
  virtual. In this case it means that they are specific to the each
  robot, for example the computation of the geometrical model.

  \warning This class uses threading capabilities. Thus on Unix-like
  platforms, the libpthread third-party library need to be
  installed. On Windows, we use the native threading capabilities.
*/
class VISP_EXPORT vpRobotSimulator : protected vpWireFrameSimulator, public vpRobot
{
  public:
    vpImage<vpRGBa> I;
    
    typedef enum  
    {
      MODEL_3D,
      MODEL_DH
    } vpDisplayRobotType;
    
    
  protected:
    /*! sampling time for the thread which manages the movement of the robot */
    float samplingTime;
    /*! cpu time at the begining of the robot's movement*/
    double tcur;
    /*! cpu time at the end of the last robot's movement*/
    double tprev;
    
    /*! Contains the 3D model of the robot's arms*/
    Bound_scene* robotArms;
    
    /*! Size of the fMi table*/
    int size_fMi;
    /*! Table containing all the homogeneous matrices between the reference frame of the robot and the frames you used to compute the Denavit-Hartenberg representation
    
    If you use a camera at the end of the effector, the last homogeneous matrix has to be the one between the reference frame and the camera frame (fMc)*/
    vpHomogeneousMatrix* fMi;
    
    /*! The articular coordinates*/
    vpColVector artCoord;
    /*! The articular velocity*/
    vpColVector artVel;
    /*! The velocity in the current frame (articular, camera or reference)*/
    vpColVector velocity;

	#if defined(WIN32)
    HANDLE  hThread;
	HANDLE mutex_fMi;
    HANDLE mutex_artVel;
    HANDLE mutex_artCoord;
    HANDLE mutex_velocity;
    HANDLE mutex_display;
    #elif defined(VISP_HAVE_PTHREAD)
    pthread_t thread;
    pthread_attr_t attr;
    pthread_mutex_t mutex_fMi;
    pthread_mutex_t mutex_artVel;
    pthread_mutex_t mutex_artCoord;
    pthread_mutex_t mutex_velocity;
    pthread_mutex_t mutex_display;
    #endif
    
    bool displayBusy;

    /*! True if the robot has to be stopped*/
    bool robotStop;
    /*! True if one of the joint reach the limit*/
    bool jointLimit;
    /*! Index of the joint which is in limit*/
    unsigned int jointLimitArt;
    /*! True if the singularity are automatically managed */
    bool singularityManagement;
    
    /*! External camera parameters*/
    vpCameraParameters cameraParam;
    
    #if defined VISP_HAVE_X11
    vpDisplayX display;
	#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
    #elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
    #elif defined VISP_HAVE_D3D9
    vpDisplayD3D display;
    #elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
    #endif
    
    vpDisplayRobotType displayType;
    
    bool displayAllowed;
    
  public:
    vpRobotSimulator();
    vpRobotSimulator(bool display);
     virtual ~vpRobotSimulator();
    
    /* Robot functions */
    /*! Set the velocity of the robot. The velocity is given relative to a frame you have to choose. */
    virtual void setVelocity (const vpRobot::vpControlFrameType /*frame*/, const vpColVector & /*velocity*/) = 0;
    /*! Get the velocity of the robot. The velocity is given relative to a frame you have to choose. */
    virtual void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q) = 0;
    
    /* vpRobot virtual methods*/
    /*! Get the robot Jacobian expressed in the end-effector frame */
    virtual void get_eJe(vpMatrix & _eJe) = 0;
    /*! Get the robot Jacobian expressed in the robot reference frame */
    virtual void get_fJe(vpMatrix &_fJe) = 0;
    /*! Get the displacement expressed in the camera frame */
    virtual void getCameraDisplacement(vpColVector &v) = 0;
    /*! Get the displacement expressed in the articular frame */
    virtual void getArticularDisplacement(vpColVector  &qdot) = 0;
    /*! Get the displacement expressed in a frame you have to specified */
    virtual void getDisplacement(const vpRobot::vpControlFrameType frame,vpColVector &q) = 0;
    /*! Set the parameter which enable or disable the singularity mangement */
    void setSingularityManagement (const bool sm) {singularityManagement = sm;}
    
    
    /* Display functions */
    void initScene(vpSceneObject obj, vpSceneDesiredObject desiredObject);
    void initScene(const char* obj, const char* desiredObject);
    void initScene (vpSceneObject obj);
    void initScene(const char* obj);
    
    /*!
      Set the desired position of the robot's camera relative to the object.
      
      \param cdMo : The desired pose of the camera.
    */
    void setDesiredCameraPosition(const vpHomogeneousMatrix cdMo)
    {
      this->vpWireFrameSimulator::setDesiredCameraPosition(cdMo);
    }

    /*!
      Set the external camera point of view.
      
      \param camMf : The pose of the external camera relative to the world reference frame.
    */
    void setExternalCameraPosition(const vpHomogeneousMatrix camMf)
    {
      this->vpWireFrameSimulator::setExternalCameraPosition(camMf);
    }
    
    /*!
      Get the external camera's position relative to the the world reference frame.
      
      \return the main external camera position relative to the the world reference frame.
    */
    vpHomogeneousMatrix getExternalCameraPosition() const
    {
      return this->vpWireFrameSimulator::getExternalCameraPosition();
    }
    
    /*!
      Set the color used to display the camera in the external view.
      
      \param col : The desired color.
    */
    void setCameraColor(const vpColor col) {camColor = col;}
    
    /*!
      Set the color used to display the object at the current position in the robot's camera view.
      
      \param col : The desired color.
    */
    void setCurrentViewColor(const vpColor col) {curColor = col;}
    
    /*!
      Set the color used to display the object at the desired position in the robot's camera view.
      
      \param col : The desired color.
    */
    void setDesiredViewColor(const vpColor col) {desColor = col;}
    
    /*!
      Get the parameters of the virtual external camera.
      
      \return It returns the camera parameters.
    */
    vpCameraParameters getExternalCameraParameters() const {
      //if(px_ext != 1 && py_ext != 1)
      // we assume px_ext and py_ext > 0
      if( (std::fabs(px_ext-1.) > vpMath::maximum(px_ext,1.)*std::numeric_limits<double>::epsilon()) 
	  && (std::fabs(py_ext-1) > vpMath::maximum(py_ext,1.)*std::numeric_limits<double>::epsilon()))
        return vpCameraParameters(px_ext,py_ext,I.getWidth()/2,I.getHeight()/2);
      else
      {
        unsigned int size = vpMath::minimum(I.getWidth(),I.getHeight())/2;
        return vpCameraParameters(size,size,I.getWidth()/2,I.getHeight()/2);
      }
    }
    
    /*!
      Set the pose between the object and the fixed world frame.
      
      \param fMo : The pose between the object and the fixed world frame.
    */
    void set_fMo(const vpHomogeneousMatrix &fMo) {this->fMo = fMo;}
    
    /*!
      Get the pose between the object and the fixed world frame.
      
      \return The pose between the object and the fixed world frame.
    */
    vpHomogeneousMatrix get_fMo() const {return fMo;}
    
    /*!
      Set the way to draw the robot. Depending on what you choose you can display a 3D wireframe model or a set of lines linking the frames used to compute the geometrical model.
      
      \param dispType : Type of display. Can be MODEL_3D or MODEL_DH.
    */
    inline void setDisplayRobotType (const vpDisplayRobotType dispType) {displayType = dispType;}
    
    vpHomogeneousMatrix get_cMo();
    
    void getInternalView(vpImage<vpRGBa> &I);
    void getInternalView(vpImage<unsigned char> &I);

  protected:
    /*!
      Function used to launch the thread which moves the robot.
    */
    #if defined(WIN32)
	static DWORD WINAPI launcher( LPVOID lpParam ) 
	{
		((vpRobotSimulator *)lpParam)->updateArticularPosition();
		return 0;
	}
    #elif defined(VISP_HAVE_PTHREAD)
    static void* launcher(void *arg)
    {
      (reinterpret_cast<vpRobotSimulator *>(arg))->updateArticularPosition();
      pthread_exit((void*) 0);
    }    
    #endif
    
    /* Robot functions */
    void init() {;}
    /*! Method lauched by the thread to compute the position of the robot in the articular frame. */
    virtual void updateArticularPosition() = 0;
    /*! Method used to check if the robot reached a joint limit. */
    virtual int isInJointLimit () = 0;
    /*! Compute the articular velocity relative to the velocity in another frame. */
    virtual void computeArticularVelocity() = 0;

    /* Display functions */
    void initDisplay() {;}
    virtual void initArms() = 0;

	#if defined(WIN32)
    vpColVector get_artCoord() {
      WaitForSingleObject(mutex_artCoord,INFINITE);
      vpColVector artCoordTmp (6);
      artCoordTmp = artCoord;
      ReleaseMutex(mutex_artCoord);
      return artCoordTmp;}
    void set_artCoord(const vpColVector &coord) {
      WaitForSingleObject(mutex_artCoord,INFINITE);
      artCoord = coord;
      ReleaseMutex(mutex_artCoord);}
    
    vpColVector get_artVel() {
      WaitForSingleObject(mutex_artVel,INFINITE);
      vpColVector artVelTmp (artVel);
      ReleaseMutex(mutex_artVel);
      return artVelTmp;}
    void set_artVel(const vpColVector &vel) {
      WaitForSingleObject(mutex_artVel,INFINITE);
      artVel = vel;
      ReleaseMutex(mutex_artVel);}
    
    vpColVector get_velocity() {
      WaitForSingleObject(mutex_velocity,INFINITE);
      vpColVector velocityTmp = velocity;
      ReleaseMutex(mutex_velocity);
      return velocityTmp;}
    void set_velocity(const vpColVector &vel) {
      WaitForSingleObject(mutex_velocity,INFINITE);
      velocity = vel;
      ReleaseMutex(mutex_velocity);}
      
    void set_displayBusy (const bool &status) {
      WaitForSingleObject(mutex_display,INFINITE);
      displayBusy = status;
      ReleaseMutex(mutex_display);}
    bool get_displayBusy () {
      WaitForSingleObject(mutex_display,INFINITE);
      bool status = displayBusy;
      if (!displayBusy) displayBusy = true;
      ReleaseMutex(mutex_display);
      return status;}

    #elif defined(VISP_HAVE_PTHREAD)
    vpColVector get_artCoord() {
      pthread_mutex_lock (&mutex_artCoord);
      vpColVector artCoordTmp (6);
      artCoordTmp = artCoord;
      pthread_mutex_unlock (&mutex_artCoord);
      return artCoordTmp;}
    void set_artCoord(const vpColVector &coord) {
      pthread_mutex_lock (&mutex_artCoord);
      artCoord = coord;
      pthread_mutex_unlock (&mutex_artCoord);}
    
    vpColVector get_artVel() {
      pthread_mutex_lock (&mutex_artVel);
      vpColVector artVelTmp (artVel);
      pthread_mutex_unlock (&mutex_artVel);
      return artVelTmp;}
    void set_artVel(const vpColVector &vel) {
      pthread_mutex_lock (&mutex_artVel);
      artVel = vel;
      pthread_mutex_unlock (&mutex_artVel);}
    
    vpColVector get_velocity() {
      pthread_mutex_lock (&mutex_velocity);
      vpColVector velocityTmp = velocity;
      pthread_mutex_unlock (&mutex_velocity);
      return velocityTmp;}
    void set_velocity(const vpColVector &vel) {
      pthread_mutex_lock (&mutex_velocity);
      velocity = vel;
      pthread_mutex_unlock (&mutex_velocity);}
      
    void set_displayBusy (const bool &status) {
      pthread_mutex_lock (&mutex_display);
      displayBusy = status;
      pthread_mutex_unlock (&mutex_display);}
    bool get_displayBusy () {
      pthread_mutex_lock (&mutex_display);
      bool status = displayBusy;
      if (!displayBusy) displayBusy = true;
      pthread_mutex_unlock (&mutex_display);
      return status;}
    #endif

    /*! Get a table of poses between the reference frame and the frames you used to compute the Denavit-Hartenberg representation */
    virtual void get_fMi(vpHomogeneousMatrix *fMit) = 0;
};

#endif

#endif
