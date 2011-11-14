/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Description:
 * Generic model based tracker. This class declares the methods to implement in 
 * order to have a model based tracker. 
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/

/*!
  \file vpMbTracker.h
  \brief Generic model based tracker. 
*/
#ifndef vpMbTracker_hh
#define vpMbTracker_hh

#include <visp/vpConfig.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPoint.h>

#ifdef VISP_HAVE_COIN
//Inventor includes
# include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
# include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#endif

#include <vector>
#include <string>

/*!
  \class vpMbTracker
  \ingroup ModelBasedTracking
  \brief Main methods for a model-based tracker. 

  This class provides the main methods for a model based tracker. This pure
  virtual class must be used in inheritance for a tracker that compute the
  interaction matrix and the residu vector using a defined information (edge,
  points of interest, patch, ...)

  This class intends to define a common basis for object tracking. This is
  realised by implementing the main functions:
  - init() : Initialisation of the tracker (it includes re-initialisation). This
    method is called at the end of the initClick() method.
  - initFaceFromCorners() : Initialisation of a face using its corners. 
  - track() : Tracking on the current image
  - testTracking() : Test the tracking. This method throws exception if the 
    tracking failed. 
  - display() : Display the model and eventually other informations.

  In addition, two flags are declared in this class and may have to be 
  initialised in the child class : 
  - modelInitialised : flag to ensure that the model has been loaded.
  - cameraInitialised : flag to ensure that camera parameters have been 
  initialised (either by loading them from a configuration file or by setting 
  them with the setCameraParameters() method).
*/
class VISP_EXPORT vpMbTracker
{
protected:
  //! The camera parameters.
  vpCameraParameters cam;
  //! The current pose.
  vpHomogeneousMatrix cMo;
  //! The name of the file containing the model (it is used to create a file name.0.pos used to store the compute pose in the initClick method).
  std::string modelFileName;
  //! Flag used to ensure that the CAD model is loaded before the initialisation.
  bool modelInitialised;
  //! Flag used to ensure that the camera is set before the initialisation.
  bool cameraInitialised;    
  //! Flag used to specify that the Coin library has been loaded in order to load a vrml model (used to free the memory).
  bool coinUsed;
  //! Filename used to save the initial pose computed using the initClick() method. It is also used to read a previous pose in the same method. 
  std::string poseSavingFilename;

public:
  vpMbTracker();
  virtual ~vpMbTracker();
  virtual void initClick(const vpImage<unsigned char>& _I, const std::string& _initFile, const bool _displayHelp = false);

  /* PURE VIRTUAL METHODS */

  /*!
    Initialise the tracking using the known pose cMo. 

    \param _I : Input image.
    \param _c0Mo : Initial pose between the camera and the object. 
  */
  virtual void init(const vpImage<unsigned char>& _I, const vpHomogeneousMatrix& _c0Mo)=0;

  /*!
    Test the quality of the tracking.

    \throw vpException if the test fail.
  */
  virtual void testTracking() = 0;

  /*!
    Load a config file to parameterise the behavior of the tracker.
    
    Pure virtual method to adapt to each tracker.
    
    \param _configFile : the (xml) config file to parse
  */
  virtual void loadConfigFile(const std::string& _configFile)=0;

  /*!
    Track the object in the given image

    \param _I : The current image.
  */
  virtual void track(const vpImage<unsigned char>& _I)=0;

  /* GENERIC METHODS */

  virtual void loadModel(const std::string& _modelFile);

  /*!
    Display the 3D model at a given position using the given camera parameters 
    on a grey level image.

    \param _I : The image.
    \param _cMo : Pose used to project the 3D model into the image.
    \param _cam : The camera parameters.
    \param _col : The desired color.
    \param _l : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible surfaces).
  */
  virtual void display(const vpImage<unsigned char>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &_cam, const vpColor& _col , const unsigned int _l=1, const bool displayFullModel = false)=0;

  /*  ACCESSORS */

  /*!
    Set the camera parameters.

    \param _cam : the new camera parameters
  */
  virtual void setCameraParameters(const vpCameraParameters& _cam) {this->cam = _cam; cameraInitialised = true;}

  /*!
    Get the camera parameters.

    \param _cam : copy of the camera parameters used by the tracker.
  */
  virtual void getCameraParameters(vpCameraParameters& _cam) const { _cam = this->cam;}

  /*!
    Get the current pose between the object and the camera.
    cMo is the matrix which can be used to express 
    coordinates from the object frame to camera frame.

    \param _cMo : the pose
  */
  inline void getPose(vpHomogeneousMatrix& _cMo) const {_cMo = this->cMo;}
  
  /*!
    Set the filename used to save the initial pose computed using the 
    initClick() method. It is also used to read a previous pose in the same method. 
    If the file is not set then, the initClick() method will create a .0.pos 
    file in the root directory. This directory is the path to the file given to 
    the method initClick() used to know the coordinates in the object frame.
    
    \param filename : The new filename.
  */
  inline void setPoseSavingFilename(const std::string& filename){
    poseSavingFilename = filename;
  }

protected:
  virtual void loadVRMLModel(const std::string& _modelFile);
  virtual void loadCAOModel(const std::string& _modelFile);

  void computeJTR(const vpMatrix& _J, const vpColVector& _R, vpMatrix& _JTR);
 
#ifdef VISP_HAVE_COIN
  virtual void extractFaces(SoVRMLIndexedFaceSet* _face_set);
  virtual void extractLines(SoVRMLIndexedLineSet* _line_set);
  virtual void extractCylinders(SoVRMLIndexedFaceSet* _face_set);
#endif
  vpPoint getGravityCenter(const std::vector<vpPoint>& _pts);

  /*!
    Add a face to track from its corners (in the object frame). This method is
    called from the loadModel() one to add a face of the object to track. 
    The initialisation of the face depends on the primitive to track.
    
    \param _corners : The vector of corners representing the face.
    \param _indexFace : The index of the face.
  */
  virtual void initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace = -1)=0;

  /*!
    Add a cylinder to track from two points on the axis (defining the length of
    the cylinder) and its radius.

    \param _p1 : First point on the axis.
    \param _p2 : Second point on the axis.
    \param _radius : Radius of the cylinder.
    \param _indexCylinder : Index of the cylinder.
  */
  virtual void initCylinder(const vpPoint& _p1, const vpPoint _p2, const double _radius, const unsigned int _indexCylinder=0)=0;
  
};


#endif

