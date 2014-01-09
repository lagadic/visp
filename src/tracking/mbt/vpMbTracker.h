/****************************************************************************
 *
 * $Id$
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
 * Description:
 * Generic model based tracker. This class declares the methods to implement in 
 * order to have a model based tracker. 
 *
 * Authors:
 * Romain Tallonneau
 * Aurélien Yol
 *
 *****************************************************************************/

/*!
  \file vpMbTracker.h
  \brief Generic model based tracker. 
*/
#ifndef vpMbTracker_hh
#define vpMbTracker_hh

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPoint.h>
#include <visp/vpMbtPolygon.h>
#include <visp/vpMbHiddenFaces.h>

#ifdef VISP_HAVE_COIN
//Work around to avoid type redefinition int8_t with Coin
// #if defined(WIN32) && defined(VISP_HAVE_OGRE) && (_MSC_VER >= 1600) // Visual Studio 2010
//   #define HAVE_INT8_T 1
// #endif

//Inventor includes
# include <Inventor/VRMLnodes/SoVRMLGroup.h>
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
  - display() : Display the model and eventually other information.

  In addition, two flags are declared in this class and may have to be 
  initialised in the child class : 
  - modelInitialised : flag to ensure that the model has been loaded.
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
  //! Filename used to save the initial pose computed using the initClick() method. It is also used to read a previous pose in the same method.
  std::string poseSavingFilename;
  //! Flag used to specify if the covariance matrix has to be computed or not.
  bool computeCovariance;
  //! Covariance matrix
  vpMatrix covarianceMatrix;
  //! If true, the features are displayed. 
  bool displayFeatures;

public:
  vpMbTracker();
  virtual ~vpMbTracker();
  
  /*!
    Display the 3D model at a given position using the given camera parameters 
    on a grey level image.

    \param I : The image.
    \param cMo : Pose used to project the 3D model into the image.
    \param cam : The camera parameters.
    \param col : The desired color.
    \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible surfaces).
  */
  virtual void display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false)=0;
  /*!
    Display the 3D model at a given position using the given camera parameters 
    on a color (RGBa) image.

    \param I : The image.
    \param cMo : Pose used to project the 3D model into the image.
    \param cam : The camera parameters.
    \param col : The desired color.
    \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible surfaces).
  */
  virtual void display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor& col , const unsigned int thickness=1, const bool displayFullModel = false)=0;

  /*!
    Get the camera parameters.

    \param cam : copy of the camera parameters used by the tracker.
  */
  virtual void getCameraParameters(vpCameraParameters& cam) const { cam = this->cam;}
  
  /*!
    Get the covariance matrix.
  */
  virtual vpMatrix getCovarianceMatrix() const { 
    if(!computeCovariance)
      vpTRACE("Warning : The covariance matrix has not been computed. See setCovarianceComputation() to do it.");
    
    return covarianceMatrix; 
  }

  /*!
    Get the current pose between the object and the camera.
    cMo is the matrix which can be used to express 
    coordinates from the object frame to camera frame.

    \param cMo : the pose
  */
  inline void getPose(vpHomogeneousMatrix& cMo) const {cMo = this->cMo;}
  
  /*!
    Get the current pose between the object and the camera.
    cMo is the matrix which can be used to express 
    coordinates from the object frame to camera frame.

    \return the current pose
  */
  inline vpHomogeneousMatrix getPose() const {return this->cMo;}

  /* PURE VIRTUAL METHODS */

  /*!
    Initialise the tracking.

    \param I : Input image.
  */
  virtual void init(const vpImage<unsigned char>& I)=0;

  // Intializer

  virtual void initClick( const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp = false );
  virtual void initClick( const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
                          const std::string &displayFile = "" );

  virtual void initFromPoints( const vpImage<unsigned char>& I, const std::string& initFile );
  virtual void initFromPoints( const vpImage<unsigned char>& I, const std::vector<vpImagePoint> &points2D_list, const std::vector<vpPoint> &points3D_list );

  virtual void initFromPose(const vpImage<unsigned char>& I, const std::string &initFile);
  virtual void initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo);
  virtual void initFromPose(const vpImage<unsigned char>& I, const vpPoseVector &cPo);

  /*!
    Load a config file to parameterise the behavior of the tracker.
    
    Pure virtual method to adapt to each tracker.
    
    \param configFile : the (xml) config file to parse
  */
  virtual void loadConfigFile(const std::string& configFile)=0;

  virtual void loadModel(const std::string& modelFile);

  /*!
    Reset the tracker.
  */
  virtual void resetTracker() = 0;

  void savePose(const std::string &filename);
  
  /*!
    Set the camera parameters.

    \param cam : the new camera parameters
  */
  virtual void setCameraParameters(const vpCameraParameters& cam) {this->cam = cam;}
  
  /*!
    Set if the covaraince matrix has to be computed.

    \param flag : True if the covariance has to be computed, false otherwise
  */
  virtual void setCovarianceComputation(const bool& flag) { computeCovariance = flag; }

  /*!
    Enable to display the features.
    
    \param displayF : set it to true to display the features.
  */
  void setDisplayFeatures(const bool displayF) {displayFeatures = displayF;}
  
  /*!
    Set the pose to be used in entry of the next call to the track() function.
    This pose will be just used once.
    
    \warning This function has to be called after the initialisation of the tracker.
    
    \param I : image corresponding to the desired pose.
    \param cdMo : Pose to affect.
  */
  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo) = 0;
  
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

  /*!
    Test the quality of the tracking.

    \throw vpException if the test fail.
  */
  virtual void testTracking() = 0;
  
  /*!
    Track the object in the given image

    \param I : The current image.
  */
  virtual void track(const vpImage<unsigned char>& I)=0;

protected:
  void computeJTR(const vpMatrix& J, const vpColVector& R, vpMatrix& JTR);
  
#ifdef VISP_HAVE_COIN
  virtual void extractGroup(SoVRMLGroup *sceneGraphVRML2, vpHomogeneousMatrix &transform, unsigned int &indexFace);
  virtual void extractFaces(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, unsigned int &indexFace);
  virtual void extractLines(SoVRMLIndexedLineSet* line_set);
  virtual void extractCylinders(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform);
#endif
  
  vpPoint getGravityCenter(const std::vector<vpPoint>& _pts);
  
  /*!
    Add a cylinder to track from two points on the axis (defining the length of
    the cylinder) and its radius.

    \param p1 : First point on the axis.
    \param p2 : Second point on the axis.
    \param radius : Radius of the cylinder.
    \param indexCylinder : Index of the cylinder.
  */
  virtual void initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const unsigned int indexCylinder=0)=0;

  /*!
    Add a face to track from its corners (in the object frame). This method is
    called from the loadModel() one to add a face of the object to track. 
    The initialisation of the face depends on the primitive to track.
    
    \param corners : The vector of corners representing the face.
    \param indexFace : The index of the face.
  */
  virtual void initFaceFromCorners(const std::vector<vpPoint>& corners, const unsigned int indexFace = -1)=0;
  
  virtual void loadVRMLModel(const std::string& modelFile);
  virtual void loadCAOModel(const std::string& modelFile);
};


#endif

