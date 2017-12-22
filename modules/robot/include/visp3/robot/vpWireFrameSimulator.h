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
 * Wire frame simulator
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpWireFrameSimulator_HH
#define vpWireFrameSimulator_HH

/*!
  \file vpWireFrameSimulator.h
  \brief Implementation of a wire frame simulator.
*/
#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits
#include <list>
#include <stdio.h>
#include <string>
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#include <visp3/core/vpList.h>
#endif
#include <visp3/core/vpImagePoint.h>
#include <visp3/robot/vpImageSimulator.h>
#include <visp3/robot/vpWireFrameSimulatorTypes.h>

/*!
  \class vpWireFrameSimulator

  \ingroup group_robot_simu_gantry group_robot_simu_arm

  \brief Implementation of a wire frame simulator. Compared to the vpSimulator
  class, it does not require third party libraries to be used.

  The simulator uses several frames to display the scene in the image. There
  are:

  - The world frame : This is a fixed frame used to compute the position of
  the other frames against each other. By default this frame is positionned at
  the same location as the initial object frame position.

  - The object frame : It is the frame linked to the object.

  - The camera frame : It is the frame linked to the main camera.

  - The external camera frame : It is the frame which corresponds to one
  external point of view.

  The most used homogeneous matrices which describes the displacement between
  two frames are :

  - fMo which is the displacement between the world frame and the object
  frame.

  - cMo which is the displacement between the main camera frame and the object
  frame.

  - cextMf which is the displacement between one external camera frame and the
  world frame.

  The following picture summarize all the previous information.

  \image html vpWireFrameSimulator.jpeg
  \image latex vpWireFrameSimulator.ps

  The simulator uses .bnd or .wrl files as 3D scene descriptors. Several
  scenes can be found in the data folder which is in the directory where you
  build ViSP.

  You can move the main external view while clicking in the image. The left
  click enables to turn, the middle button enables to zoom and the left to
  translate along x and y.

  The simulator is able to take into account to camera parameters. You can set
  the internal and external cameras parameters thanks to a vpCameraParameters.

  The following example shows how it is easy to use.

  \code

  #include <visp3/core/vpImage.h>
  #include <visp3/robot/vpWireFrameSimulator.h>

  int main()
  {
    vpWireFrameSimulator sim;

    vpImage<vpRGBa> Iint(480,640,255);
    vpImage<vpRGBa> Iext(480,640,255);

    //Set the type of scene to use
    sim.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD);

    //Set the initial pose of the camera
    sim.setCameraPositionRelObj(vpHomogeneousMatrix(0,0,0.5,vpMath::rad(0),vpMath::rad(10),0));
    //Set the desired pose of the camera (for the internal view)
    sim.setDesiredCameraPosition(vpHomogeneousMatrix(0.0,0,0.5,0,0,0));
    //Set the pose of the reference frame (for the external view)
    sim.setExternalCameraPosition(vpHomogeneousMatrix(0.1,0,0.2,0,0,0));

    //Set the camera parameters
    vpCameraParameters camera(1000,1000,320,240);
    sim.setInternalCameraParameters(camera);
    sim.setExternalCameraParameters(camera);

    //Get the internal view
    sim.getInternalImage(Iint);

    //Get the external view
    sim.getExternalImage(Iext);

    return 0;
  }
  \endcode
*/

class VISP_EXPORT vpWireFrameSimulator
{
public:
  /*!
      Type of scene used to display the object at the current position.
    */
  typedef enum {
    THREE_PTS,   //!< A 40cm by 40cm plate with 3 points at coordinates (0,0,0),
                 //!< (0.1,0,0), (0,0.1,0). Each point is represented by a
                 //!< circle with 2cm radius.
    CUBE,        //!< A 12.5cm size cube.
    PLATE,       //!< A 40cm by 40cm plate with 4 points at coordinates
                 //!< (-0.1,-0.1,0), (0.1,-0.1,0), (0.1,0.1,0), (0.1,0.1,0). Each
                 //!< point is represented by a circle with 2cm radius.
    SMALL_PLATE, //!< 4 points at coordinates (-0.03,-0.03,0), (0.03,-0.03,0),
                 //!< (0.03,0.03,0), (0.03,0.03,0). Each point is represented
                 //!< by a circle with 1cm radius.
    RECTANGLE,   //!< A 40cm by 40cm plate with 4 points at coordinates
                 //!< (-0.07,-0.05,0), (0.07,0.05,0), (0.07,-0.05,0),
                 //!< (-0.07,-0.05,0). Each point is represented by a circle
                 //!< with 2cm radius.
    SQUARE_10CM, //!< A 40cm by 40cm plate with 4 points at coordinates
                 //!< (-0.05,0.05,0), (0.05,0.05,0), (0.05,-0.05,0),
                 //!< (-0.05,-0.05,0). Each point is represented by a circle
                 //!< with 2cm radius.
    DIAMOND,     //!< A 40cm by 40cm plate with 4 points at coordinates
                 //!< (0,-0.1,0), (0.1,0,0), (0,0.1,0), (-0.1,0,0). Each point is
                 //!< represented by a circle with 2cm radius.
    TRAPEZOID,   //!< A 40cm by 40cm plate with 4 points at coordinates
                 //!< (-0.025,-0.05,0), (-0.075,0.05,0), (0.075,0.05,0),
                 //!< (0.025,-0.05,0). Each point is represented by a circle
                 //!< with 2cm radius.
    THREE_LINES, //!< Three parallel lines with equation y=-5, y=0, y=5.
    ROAD,        //!< Three parallel lines representing a road.
    TIRE,        //!< A tire represented by 2 circles with radius 10cm and 15cm.
    PIPE,        //!< A pipe represented by a cylinder of 25 cm length and 15cm
                 //!< radius.
    CIRCLE,      //!< A 10cm radius circle.
    SPHERE,      //!< A 15cm radius sphere.
    CYLINDER,    //!< A cylinder of 80cm length and 10cm radius.
    PLAN,        //!< A plane represented by a 56cm by 56cm plate with a grid of 49
                 //!< squares inside.
    POINT_CLOUD, //!< A plate with 8 points at coordinates (0.05,0,0),
                 //!< (0.15,0.05,0), (0.2,0.2,0), (-0.05,0.2,0),
                 //!< (-0.15,-0.1,0), (-0.1,-0.1,0), (-0.05,0.05,0) and
                 //!< (0.5,0,0). ach point is represented by a circle with 2cm
                 //!< radius.
  } vpSceneObject;

  /*!
      Type of scene used to display the object at the desired pose (in the
     internal view).

      - D_STANDARD will use the vpSceneObject used to be the object at the
     current position.
      - D_OUTIL will display a tool which is attached to the camera.
    */
  typedef enum {
    D_STANDARD, //!<  The object displayed at the desired position is the same
                //!<  than the scene object defined in vpSceneObject.
    D_CIRCLE,   //!<  The object displayed at the desired position is a circle.
    D_TOOL      //!< A cylindrical tool is attached to the camera.
  } vpSceneDesiredObject;

  typedef enum { CT_LINE, CT_POINT } vpCameraTrajectoryDisplayType;

protected:
  Bound_scene scene;
  Bound_scene desiredScene;
  Bound_scene camera;
  std::list<vpImageSimulator> objectImage;

  vpHomogeneousMatrix fMo;
  vpHomogeneousMatrix fMc;
  vpHomogeneousMatrix camMf;
  vpHomogeneousMatrix refMo;
  vpHomogeneousMatrix cMo;
  vpHomogeneousMatrix cdMo;

  vpSceneObject object;
  vpSceneDesiredObject desiredObject;

  vpColor camColor;
  vpColor camTrajColor;
  vpColor curColor;
  vpColor desColor;

  bool sceneInitialized;

  bool displayCameraTrajectory;
  std::list<vpImagePoint> cameraTrajectory;
  std::list<vpHomogeneousMatrix> poseList;
  std::list<vpHomogeneousMatrix> fMoList;
  unsigned int nbrPtLimit;

  vpImagePoint old_iPr;
  vpImagePoint old_iPz;
  vpImagePoint old_iPt;
  bool blockedr;
  bool blockedz;
  bool blockedt;
  bool blocked;

  vpHomogeneousMatrix camMf2;
  vpHomogeneousMatrix f2Mf;

  double px_int;
  double py_int;
  double px_ext;
  double py_ext;

  bool displayObject;
  bool displayDesiredObject;
  bool displayCamera;
  bool displayImageSimulator;

  float cameraFactor;

  vpCameraTrajectoryDisplayType camTrajType;

  bool extCamChanged;

  vpHomogeneousMatrix rotz;

  unsigned int thickness_;

private:
  std::string scene_dir;

public:
  vpWireFrameSimulator();
  virtual ~vpWireFrameSimulator();

  /** @name Inherited functionalities from vpWireFrameSimulator */
  //@{
  /*!
      Delete the history of the main camera position which are displayed in
     the external views.
    */
  inline void deleteCameraPositionHistory()
  {
    cameraTrajectory.clear();
    poseList.clear();
    fMoList.clear();
  }

  void displayTrajectory(const vpImage<unsigned char> &I, const std::list<vpHomogeneousMatrix> &list_cMo,
                         const std::list<vpHomogeneousMatrix> &list_fMo, const vpHomogeneousMatrix &camMf);
  void displayTrajectory(const vpImage<vpRGBa> &I, const std::list<vpHomogeneousMatrix> &list_cMo,
                         const std::list<vpHomogeneousMatrix> &list_fMo, const vpHomogeneousMatrix &camMf);

  /*!
      Get the parameters of the virtual external camera.

      \param I : The image used to display the view of the camera.

      \return It returns the camera parameters.
    */
  vpCameraParameters getExternalCameraParameters(const vpImage<unsigned char> &I) const
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
     Get the parameters of the virtual external camera.

     \param I : The image used to display the view of the camera.

     \return It returns the camera parameters.
   */
  vpCameraParameters getExternalCameraParameters(const vpImage<vpRGBa> &I) const
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
     Get the main external camera's position relative to the the world
     reference frame.

     \return the main external camera position relative to the the world
     reference frame.
   */
  inline vpHomogeneousMatrix getExternalCameraPosition() const { return rotz * camMf; }

  void getExternalImage(vpImage<unsigned char> &I);
  void getExternalImage(vpImage<unsigned char> &I, const vpHomogeneousMatrix &camMf);
  void getExternalImage(vpImage<vpRGBa> &I);
  void getExternalImage(vpImage<vpRGBa> &I, const vpHomogeneousMatrix &camMf);

  /*!
      Get the parameters of the virtual internal camera.

      \param I : The image used to display the view of the camera.

      \return It returns the camera parameters.
    */
  vpCameraParameters getInternalCameraParameters(const vpImage<unsigned char> &I) const
  {
    // if(px_int != 1 && py_int != 1)
    // we assume px_int and py_int > 0
    if ((std::fabs(px_int - 1.) > vpMath::maximum(px_int, 1.) * std::numeric_limits<double>::epsilon()) &&
        (std::fabs(py_int - 1) > vpMath::maximum(py_int, 1.) * std::numeric_limits<double>::epsilon()))
      return vpCameraParameters(px_int, py_int, I.getWidth() / 2, I.getHeight() / 2);
    else {
      unsigned int size = vpMath::minimum(I.getWidth(), I.getHeight()) / 2;
      return vpCameraParameters(size, size, I.getWidth() / 2, I.getHeight() / 2);
    }
  }
  /*!
      Get the parameters of the virtual internal camera.

      \param I : The image used to display the view of the camera.

      \return It returns the camera parameters.
    */
  vpCameraParameters getInternalCameraParameters(const vpImage<vpRGBa> &I) const
  {
    // if(px_int != 1 && py_int != 1)
    // we assume px_int and py_int > 0
    if ((std::fabs(px_int - 1.) > vpMath::maximum(px_int, 1.) * std::numeric_limits<double>::epsilon()) &&
        (std::fabs(py_int - 1) > vpMath::maximum(py_int, 1.) * std::numeric_limits<double>::epsilon()))
      return vpCameraParameters(px_int, py_int, I.getWidth() / 2, I.getHeight() / 2);
    else {
      unsigned int size = vpMath::minimum(I.getWidth(), I.getHeight()) / 2;
      return vpCameraParameters(size, size, I.getWidth() / 2, I.getHeight() / 2);
    }
  }

  void getInternalImage(vpImage<unsigned char> &I);
  void getInternalImage(vpImage<vpRGBa> &I);

  /*!
      Get the pose between the object and the camera.

      \return The pose between between the object and the camera.
    */
  vpHomogeneousMatrix get_cMo() const { return rotz * cMo; }

  /*!
      Get the homogeneous matrices cMo stored to display the camera
     trajectory.

      \param cMo_history : The list of the homogeneous matrices cMo.
    */
  void get_cMo_History(std::list<vpHomogeneousMatrix> &cMo_history)
  {
    cMo_history.clear();
    for (std::list<vpHomogeneousMatrix>::const_iterator it = poseList.begin(); it != poseList.end(); ++it) {
      cMo_history.push_back(rotz * (*it));
    }
  }

  /*!
      Get the pose between the object and the fixed world frame.

      \return The pose between the object and the fixed world frame.
    */
  vpHomogeneousMatrix get_fMo() const { return fMo; }

  /*!
      Get the homogeneous matrices fMo stored to display the camera
     trajectory.

      \param fMo_history : The list of the homogeneous matrices fMo.
    */
  void get_fMo_History(std::list<vpHomogeneousMatrix> &fMo_history) { fMo_history = fMoList; }

  void initScene(const vpSceneObject &obj, const vpSceneDesiredObject &desiredObject);
  void initScene(const char *obj, const char *desiredObject);
  void initScene(const vpSceneObject &obj);
  void initScene(const char *obj);

  void initScene(const vpSceneObject &obj, const vpSceneDesiredObject &desiredObject,
                 const std::list<vpImageSimulator> &imObj);
  void initScene(const char *obj, const char *desiredObject, const std::list<vpImageSimulator> &imObj);
  void initScene(const vpSceneObject &obj, const std::list<vpImageSimulator> &imObj);
  void initScene(const char *obj, const std::list<vpImageSimulator> &imObj);

  /*!
      Set the color used to display the camera in the external view.

      \param col : The desired color.
    */
  void setCameraColor(const vpColor &col) { camColor = col; }
  /*!
      Set the position of the camera relative to the object.

      \param cMo_ : The pose of the camera.
    */
  void setCameraPositionRelObj(const vpHomogeneousMatrix &cMo_)
  {
    this->cMo = rotz * cMo_;
    fMc = fMo * this->cMo.inverse();
  }

  /*!
      Set the position of the the world reference frame relative to the
     camera.

      \param fMc_ : The pose of the camera.
    */
  void setCameraPositionRelWorld(const vpHomogeneousMatrix &fMc_)
  {
    this->fMc = fMc_ * rotz;
    cMo = this->fMc.inverse() * fMo;
  }

  /*!
      Set the parameter which enables to choose the size of the main camera in
     the external camera views. By default this parameter is set to 1.

      \param factor : The ration for the camera size.
    */
  inline void setCameraSizeFactor(const float factor) { cameraFactor = factor; }

  /*!
      Set the color used to display the camera trajectory in the external
     view.

      \param col : The desired color.
    */
  void setCameraTrajectoryColor(const vpColor &col) { camTrajColor = col; }

  /*!
      Set the way to display the history of the main camera trajectory in the
     main external view. The choice is given between displaying lines and
     points.

      \param camTraj_type : The chosen way to display the camera trajectory.
    */
  inline void setCameraTrajectoryDisplayType(const vpCameraTrajectoryDisplayType &camTraj_type)
  {
    this->camTrajType = camTraj_type;
  }

  /*!
      Set the color used to display the object at the current position.

      \param col : The desired color.
    */
  void setCurrentViewColor(const vpColor &col) { curColor = col; }
  /*!
      Set the desired position of the camera relative to the object.

      \param cdMo_ : The desired pose of the camera.
    */
  void setDesiredCameraPosition(const vpHomogeneousMatrix &cdMo_) { this->cdMo = rotz * cdMo_; }
  /*!
      Set the color used to display the object at the desired position.

      \param col : The desired color.
    */
  void setDesiredViewColor(const vpColor &col) { desColor = col; }
  /*!
      Enable or disable the displaying of the camera trajectory in the main
     external camera view.

      By default the trajectory is displayed.

      \param do_display : Set to true to display the camera trajectory.
    */
  void setDisplayCameraTrajectory(const bool &do_display) { this->displayCameraTrajectory = do_display; }

  /*!
      Set the internal camera parameters.

      \param cam : The desired camera parameters.
    */
  inline void setExternalCameraParameters(const vpCameraParameters &cam)
  {
    px_ext = cam.get_px();
    py_ext = cam.get_py();
  }
  /*!
      Set the external camera point of view.

      \param cam_Mf : The pose of the external camera relative to the world
     reference frame.
    */
  void setExternalCameraPosition(const vpHomogeneousMatrix &cam_Mf)
  {
    this->camMf = rotz * cam_Mf;
    vpTranslationVector T;
    this->camMf.extract(T);
    this->camMf2.buildFrom(0, 0, T[2], 0, 0, 0);
    f2Mf = camMf2.inverse() * this->camMf;
    extCamChanged = true;
  }

  /*!
    Specify the thickness of the graphics drawings.
    */
  void setGraphicsThickness(unsigned int thickness) { this->thickness_ = thickness; }

  /*!
      Set the internal camera parameters.

      \param cam : The desired camera parameters.
    */
  inline void setInternalCameraParameters(const vpCameraParameters &cam)
  {
    px_int = cam.get_px();
    py_int = cam.get_py();
  }

  /*!
      Set the maximum number of main camera's positions which are stored.
     Those position can be displayed in the external camera field of view. By
     default this parameter is set to 1000.

      \param nbPt : The desired number of position which are saved.
    */
  inline void setNbPtTrajectory(const unsigned int nbPt) { nbrPtLimit = nbPt; }

  /*!
      Set the pose between the object and the fixed world frame.

      \param fMo_ : The pose between the object and the fixed world frame.
    */
  void set_fMo(const vpHomogeneousMatrix &fMo_) { this->fMo = fMo_; /*this->cMo = fMc.inverse()*fMo;*/ }
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpWireFrameSimulator */
  //@{
  void display_scene(Matrix mat, Bound_scene &sc, const vpImage<vpRGBa> &I, const vpColor &color);
  void display_scene(Matrix mat, Bound_scene &sc, const vpImage<unsigned char> &I, const vpColor &color);
  vpHomogeneousMatrix navigation(const vpImage<vpRGBa> &I, bool &changed);
  vpHomogeneousMatrix navigation(const vpImage<unsigned char> &I, bool &changed);
  vpImagePoint projectCameraTrajectory(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                       const vpHomogeneousMatrix &fMo);
  vpImagePoint projectCameraTrajectory(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                       const vpHomogeneousMatrix &fMo);
  vpImagePoint projectCameraTrajectory(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                       const vpHomogeneousMatrix &fMo, const vpHomogeneousMatrix &cMf);
  vpImagePoint projectCameraTrajectory(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                       const vpHomogeneousMatrix &fMo, const vpHomogeneousMatrix &cMf);
  //@}
};

#endif
