/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Basic class used to make robot simulators.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpRobotWireFrameSimulator_HH
#define vpRobotWireFrameSimulator_HH

/*!
  \file vpRobotWireFrameSimulator.h
  \brief Basic class used to make robot simulators.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_GUI) && ((defined(_WIN32) && !defined(WINRT_8_0)) || defined(VISP_HAVE_PTHREAD))

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#if defined(_WIN32)
// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>
#elif defined(VISP_HAVE_PTHREAD)
#include <pthread.h>
#endif

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotSimulator.h>
#include <visp3/robot/vpWireFrameSimulator.h>

/*!
  \class vpRobotWireFrameSimulator

  \ingroup group_robot_simu_gantry group_robot_simu_arm

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
class VISP_EXPORT vpRobotWireFrameSimulator : protected vpWireFrameSimulator, public vpRobotSimulator
{
public:
  vpImage<vpRGBa> I;

  typedef enum { MODEL_3D, MODEL_DH } vpDisplayRobotType;

protected:
  /*! cpu time at the begining of the robot's movement*/
  double tcur;
  /*! cpu time at the end of the last robot's movement*/
  double tprev;

  /*! Contains the 3D model of the robot's arms*/
  Bound_scene *robotArms;

  /*! Size of the fMi table*/
  unsigned int size_fMi;
  /*! Table containing all the homogeneous matrices between the reference
  frame of the robot and the frames you used to compute the Denavit-Hartenberg
  representation

  If you use a camera at the end of the effector, the last homogeneous matrix
  has to be the one between the reference frame and the camera frame (fMc)*/
  vpHomogeneousMatrix *fMi;

  /*! The articular coordinates*/
  vpColVector artCoord;
  /*! The articular velocity*/
  vpColVector artVel;
  /*! The velocity in the current frame (articular, camera or reference)*/
  vpColVector velocity;

#if defined(_WIN32)
  HANDLE hThread;
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
  //! Flag used to force the sampling time in the thread computing the robot's
  //! displacement to a constant value (\e samplingTime). It may be useful if
  //! the main thread (computing the features) is very time consumming. False
  //! by default.
  bool constantSamplingTimeMode;

  //! Flag used to specify to the thread managing the robot displacements that
  //! the setVelocity() method has been called.
  bool setVelocityCalled;

  bool verbose_;

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //    vpRobotWireFrameSimulator(const vpRobotWireFrameSimulator &)
  //      : vpWireFrameSimulator(), vpRobotSimulator(),
  //        I(), tcur(0), tprev(0), robotArms(NULL), size_fMi(8), fMi(NULL),
  //        artCoord(), artVel(), velocity(),
  //    #if defined(_WIN32)
  //    #elif defined(VISP_HAVE_PTHREAD)
  //        thread(), attr(),
  //    #endif
  //        mutex_fMi(), mutex_artVel(), mutex_artCoord(), mutex_velocity(),
  //        mutex_display(), displayBusy(false), robotStop(false),
  //        jointLimit(false), jointLimitArt(false),
  //        singularityManagement(true), cameraParam(),
  //    #if defined(VISP_HAVE_DISPLAY)
  //        display(),
  //    #endif
  //        displayType(MODEL_3D), displayAllowed(true),
  //        constantSamplingTimeMode(false), setVelocityCalled(false),
  //        verbose_(false)
  //    {
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!");
  //    }
  //    vpRobotWireFrameSimulator &operator=(const vpRobotWireFrameSimulator
  //    &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  vpRobotWireFrameSimulator();
  explicit vpRobotWireFrameSimulator(bool display);
  virtual ~vpRobotWireFrameSimulator();

  /** @name Inherited functionalities from vpRobotWireFrameSimulator */
  //@{
  /*!
    Get the parameters of the virtual external camera.

    \return It returns the camera parameters.
  */
  vpCameraParameters getExternalCameraParameters() const
  {
    // if(px_ext != 1 && py_ext != 1)
    // we assume px_ext and py_ext > 0
    if ((std::fabs(px_ext - 1.) > vpMath::maximum(px_ext, 1.) * std::numeric_limits<double>::epsilon()) &&
        (std::fabs(py_ext - 1) > vpMath::maximum(py_ext, 1.) * std::numeric_limits<double>::epsilon()))
      return vpCameraParameters(px_ext, py_ext, I.getWidth() / 2, I.getHeight() / 2);
    else {
      unsigned int size = vpMath::minimum(I.getWidth(), I.getHeight()) / 2;
      return vpCameraParameters(size, size, I.getWidth() / 2, I.getHeight() / 2);
    }
  }
  /*!
    Get the external camera's position relative to the the world reference
    frame.

    \return the main external camera position relative to the the world
    reference frame.
  */
  vpHomogeneousMatrix getExternalCameraPosition() const
  {
    return this->vpWireFrameSimulator::getExternalCameraPosition();
  }

  void getInternalView(vpImage<vpRGBa> &I);
  void getInternalView(vpImage<unsigned char> &I);

  vpHomogeneousMatrix get_cMo();
  /*!
    Get the pose between the object and the fixed world frame.

    \return The pose between the object and the fixed world frame.
  */
  vpHomogeneousMatrix get_fMo() const { return fMo; }

  /* Display functions */
  void initScene(const vpSceneObject &obj, const vpSceneDesiredObject &desiredObject);
  void initScene(const char *obj, const char *desiredObject);
  void initScene(const vpSceneObject &obj);
  void initScene(const char *obj);

  /*!
    Set the color used to display the camera in the external view.

    \param col : The desired color.
  */
  void setCameraColor(const vpColor &col) { camColor = col; }

  /*!
    Set the flag used to force the sampling time in the thread computing the
    robot's displacement to a constant value; see setSamplingTime(). It may be
    useful if the main thread (computing the features) is very time consuming.
    False by default.

    \param _constantSamplingTimeMode : The new value of the
    constantSamplingTimeMode flag.
  */
  inline void setConstantSamplingTimeMode(const bool _constantSamplingTimeMode)
  {
    constantSamplingTimeMode = _constantSamplingTimeMode;
  }

  /*!
    Set the color used to display the object at the current position in the
    robot's camera view.

    \param col : The desired color.
  */
  void setCurrentViewColor(const vpColor &col) { curColor = col; }

  /*!
    Set the color used to display the object at the desired position in the
    robot's camera view.

    \param col : The desired color.
  */
  void setDesiredViewColor(const vpColor &col) { desColor = col; }

  /*!
    Set the desired position of the robot's camera relative to the object.

    \param cdMo_ : The desired pose of the camera.
  */
  void setDesiredCameraPosition(const vpHomogeneousMatrix &cdMo_)
  {
    this->vpWireFrameSimulator::setDesiredCameraPosition(cdMo_);
  }

  /*!
    Set the way to draw the robot. Depending on what you choose you can
    display a 3D wireframe model or a set of lines linking the frames used to
    compute the geometrical model.

    \param dispType : Type of display. Can be MODEL_3D or MODEL_DH.
  */
  inline void setDisplayRobotType(const vpDisplayRobotType dispType) { displayType = dispType; }
  /*!
    Set the external camera point of view.

    \param camMf_ : The pose of the external camera relative to the world
    reference frame.
  */
  void setExternalCameraPosition(const vpHomogeneousMatrix &camMf_)
  {
    this->vpWireFrameSimulator::setExternalCameraPosition(camMf_);
  }
  /*!
    Specify the thickness of the graphics drawings.
    */
  void setGraphicsThickness(unsigned int thickness) { this->thickness_ = thickness; }

  /*!
    Set the sampling time.

    \param delta_t : Sampling time in second used to compute the robot
    displacement from the velocity applied to the robot during this time.

    Since the wireframe simulator is threaded, the sampling time is set to
    vpTime::getMinTimeForUsleepCall() / 1000 seconds.

  */
  inline void setSamplingTime(const double &delta_t)
  {
    if (delta_t < static_cast<float>(vpTime::getMinTimeForUsleepCall() * 1e-3)) {
      this->delta_t_ = static_cast<float>(vpTime::getMinTimeForUsleepCall() * 1e-3);
    } else {
      this->delta_t_ = delta_t;
    }
  }
  /*! Set the parameter which enable or disable the singularity mangement */
  void setSingularityManagement(const bool sm) { singularityManagement = sm; }

  /*!
    Activates extra printings when the robot reaches joint limits...
    */
  void setVerbose(bool verbose) { this->verbose_ = verbose; }

  /*!
    Set the pose between the object and the fixed world frame.

    \param fMo_ : The pose between the object and the fixed world frame.
  */
  void set_fMo(const vpHomogeneousMatrix &fMo_) { this->fMo = fMo_; }
  //@}

protected:
/** @name Protected Member Functions Inherited from vpRobotWireFrameSimulator
 */
//@{
/*!
  Function used to launch the thread which moves the robot.
*/
#if defined(_WIN32)
  static DWORD WINAPI launcher(LPVOID lpParam)
  {
    (static_cast<vpRobotWireFrameSimulator *>(lpParam))->updateArticularPosition();
    return 0;
  }
#elif defined(VISP_HAVE_PTHREAD)
  static void *launcher(void *arg)
  {
    (reinterpret_cast<vpRobotWireFrameSimulator *>(arg))->updateArticularPosition();
    // pthread_exit((void*) 0);
    return NULL;
  }
#endif

  /* Robot functions */
  void init() { ; }
  /*! Method lauched by the thread to compute the position of the robot in the
   * articular frame. */
  virtual void updateArticularPosition() = 0;
  /*! Method used to check if the robot reached a joint limit. */
  virtual int isInJointLimit() = 0;
  /*! Compute the articular velocity relative to the velocity in another
   * frame. */
  virtual void computeArticularVelocity() = 0;

  /* Display functions */
  void initDisplay() { ; }
  virtual void initArms() = 0;

#if defined(_WIN32)
  vpColVector get_artCoord() const
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_artCoord, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_artCoord, INFINITE);
#endif
    vpColVector artCoordTmp(6);
    artCoordTmp = artCoord;
    ReleaseMutex(mutex_artCoord);
    return artCoordTmp;
  }
  void set_artCoord(const vpColVector &coord)
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_artCoord, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_artCoord, INFINITE);
#endif
    artCoord = coord;
    ReleaseMutex(mutex_artCoord);
  }

  vpColVector get_artVel() const
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_artVel, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_artVel, INFINITE);
#endif
    vpColVector artVelTmp(artVel);
    ReleaseMutex(mutex_artVel);
    return artVelTmp;
  }
  void set_artVel(const vpColVector &vel)
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_artVel, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_artVel, INFINITE);
#endif
    artVel = vel;
    ReleaseMutex(mutex_artVel);
  }

  vpColVector get_velocity()
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_velocity, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_velocity, INFINITE);
#endif
    vpColVector velocityTmp = velocity;
    ReleaseMutex(mutex_velocity);
    return velocityTmp;
  }
  void set_velocity(const vpColVector &vel)
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_velocity, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_velocity, INFINITE);
#endif
    velocity = vel;
    ReleaseMutex(mutex_velocity);
  }

  void set_displayBusy(const bool &status)
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_display, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_display, INFINITE);
#endif
    displayBusy = status;
    ReleaseMutex(mutex_display);
  }
  bool get_displayBusy()
  {
#if defined(WINRT_8_1)
    WaitForSingleObjectEx(mutex_display, INFINITE, FALSE);
#else // pure win32
    WaitForSingleObject(mutex_display, INFINITE);
#endif
    bool status = displayBusy;
    if (!displayBusy)
      displayBusy = true;
    ReleaseMutex(mutex_display);
    return status;
  }

#elif defined(VISP_HAVE_PTHREAD)
  vpColVector get_artCoord()
  {
    pthread_mutex_lock(&mutex_artCoord);
    vpColVector artCoordTmp(6);
    artCoordTmp = artCoord;
    pthread_mutex_unlock(&mutex_artCoord);
    return artCoordTmp;
  }
  void set_artCoord(const vpColVector &coord)
  {
    pthread_mutex_lock(&mutex_artCoord);
    artCoord = coord;
    pthread_mutex_unlock(&mutex_artCoord);
  }

  vpColVector get_artVel()
  {
    pthread_mutex_lock(&mutex_artVel);
    vpColVector artVelTmp(artVel);
    pthread_mutex_unlock(&mutex_artVel);
    return artVelTmp;
  }
  void set_artVel(const vpColVector &vel)
  {
    pthread_mutex_lock(&mutex_artVel);
    artVel = vel;
    pthread_mutex_unlock(&mutex_artVel);
  }

  vpColVector get_velocity()
  {
    pthread_mutex_lock(&mutex_velocity);
    vpColVector velocityTmp = velocity;
    pthread_mutex_unlock(&mutex_velocity);
    return velocityTmp;
  }
  void set_velocity(const vpColVector &vel)
  {
    pthread_mutex_lock(&mutex_velocity);
    velocity = vel;
    pthread_mutex_unlock(&mutex_velocity);
  }

  void set_displayBusy(const bool &status)
  {
    pthread_mutex_lock(&mutex_display);
    displayBusy = status;
    pthread_mutex_unlock(&mutex_display);
  }
  bool get_displayBusy()
  {
    pthread_mutex_lock(&mutex_display);
    bool status = displayBusy;
    if (!displayBusy)
      displayBusy = true;
    pthread_mutex_unlock(&mutex_display);
    return status;
  }
#endif

  /*! Get a table of poses between the reference frame and the frames you used
   * to compute the Denavit-Hartenberg representation */
  virtual void get_fMi(vpHomogeneousMatrix *fMit) = 0;
  //@}
};

#endif
#endif
