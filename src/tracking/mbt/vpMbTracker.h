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
// #if defined(_WIN32) && defined(VISP_HAVE_OGRE) && (_MSC_VER >= 1600) // Visual Studio 2010
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
  - initFaceFromCorners() : Initialisation of the lines that has to be tracked.
  - track() : Tracking on the current image
  - testTracking() : Test the tracking. This method throws exception if the 
    tracking failed. 
  - display() : Display the model and eventually other information.

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
  //! Weights used in the robust scheme
  vpColVector m_w;
  //! Error s-s*
  vpColVector m_error;

  //! Set of faces describing the object.
  vpMbHiddenFaces<vpMbtPolygon> faces;
  //! Angle used to detect a face appearance
  double angleAppears;
  //! Angle used to detect a face disappearance
  double angleDisappears;
  //! Distance for near clipping
  double distNearClip;
  //! Distance for near clipping
  double distFarClip;
  //! Flags specifying which clipping to used
  unsigned int clippingFlag;
  //! Use Ogre3d for visibility tests
  bool useOgre;

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

  /*! Return the angle used to test polygons appearance. */
  virtual inline  double  getAngleAppear() const { return angleAppears; }

  /*! Return the angle used to test polygons disappearance. */
  virtual inline  double  getAngleDisappear() const { return angleDisappears; }

  /*!
    Get the camera parameters.

    \param camera : copy of the camera parameters used by the tracker.
  */
  virtual void getCameraParameters(vpCameraParameters& camera) const { camera = this->cam;}
  
  /*!
    Get the clipping used.

    \sa vpMbtPolygonClipping

    \return Clipping flags.
  */
  virtual inline  unsigned int getClipping() const { return clippingFlag; }

  /*!
    Get the covariance matrix.
  */
  virtual vpMatrix getCovarianceMatrix() const { 
    if(!computeCovariance)
      vpTRACE("Warning : The covariance matrix has not been computed. See setCovarianceComputation() to do it.");
    
    return covarianceMatrix; 
  }

  /*!
    Return the error vector \f$(s-s^*)\f$ reached after the virtual visual servoing process used to estimate the pose.

    The following example shows how to use this function:
    \code
      tracker.track(I);
      std::cout << "Error: " << sqrt( (tracker.getError()).sumSquare()) << std::endl;
    \endcode

    \sa getRobustWeights()
   */
  virtual vpColVector getError() {
    return m_error;
  }

  /*! Return a reference to the faces structure. */
  inline  vpMbHiddenFaces<vpMbtPolygon>& getFaces() { return faces;}

  /*!
    Get the far distance for clipping.

    \return Far clipping value.
  */
  virtual inline  double  getFarClippingDistance() const { return distFarClip; }

  /*!
    Return the weights vector \f$w_i\f$ computed by the robust scheme.

    The following example shows how to use this function:
    \code
      tracker.track(I);
      vpColVector w = tracker.getRobustWeights();
      vpColVector e = tracker.getError();
      vpColVector we(w.size());
      for(unsigned int i=0; i<w.size(); i++)
        we[i] = w[i]*e[i];

      std::cout << "Error         : " << sqrt( (e ).sumSquare() ) << std::endl;
      std::cout << "Weighted error: " << sqrt( (we).sumSquare() ) << std::endl;
    \endcode

    \sa getError()
   */
  virtual vpColVector getRobustWeights() {
    return m_w;
  }

  /*!
    Get the near distance for clipping.

    \return Near clipping value.
  */
  virtual inline double   getNearClippingDistance() const { return distNearClip; }

  /*!
    Get the current pose between the object and the camera.
    cMo is the matrix which can be used to express 
    coordinates from the object frame to camera frame.

    \param cMo_ : the pose
  */
  inline void getPose(vpHomogeneousMatrix& cMo_) const {cMo_ = this->cMo;}
  
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
    
    \param configFile : An xml config file to parse.
  */
  virtual void loadConfigFile(const std::string& configFile)=0;

  virtual void loadModel(const char *modelFile);
  virtual void loadModel(const std::string &modelFile);

  /*!
    Reset the tracker.
  */
  virtual void resetTracker() = 0;

  void savePose(const std::string &filename);

  /*!
    Set the angle used to test polygons appearance.
    If the angle between the normal of the polygon and the line going
    from the camera to the polygon center has a value lower than
    this parameter, the polygon is considered as appearing.
    The polygon will then be tracked.

    \param a : new angle in radian.
  */
  virtual inline  void    setAngleAppear(const double &a) { angleAppears = a; }

  /*!
    Set the angle used to test polygons disappearance.
    If the angle between the normal of the polygon and the line going
    from the camera to the polygon center has a value greater than
    this parameter, the polygon is considered as disappearing.
    The tracking of the polygon will then be stopped.

    \param a : new angle in radian.
  */
  virtual inline  void    setAngleDisappear(const double &a) { angleDisappears = a; }
  
  /*!
    Set the camera parameters.

    \param camera : the new camera parameters
  */
  virtual void setCameraParameters(const vpCameraParameters& camera) {this->cam = camera;}

  virtual void setClipping(const unsigned int &flags);
  
  /*!
    Set if the covaraince matrix has to be computed.

    \param flag : True if the covariance has to be computed, false otherwise
  */
  virtual void setCovarianceComputation(const bool& flag) { computeCovariance = flag; }

  /*!
    Enable to display the features. By features, we meant the moving edges (ME) and the klt points if used.

    Note that if present, the moving edges can be displayed with different colors:
    - If green : The ME is a good point.
    - If blue : The ME is removed because of a contrast problem during the tracking phase.
    - If purple : The ME is removed because of a threshold problem during the tracking phase.
    - If red : The ME is removed because it is rejected by the robust approach in the virtual visual servoing scheme.

    \param displayF : set it to true to display the features.
  */
  void setDisplayFeatures(const bool displayF) {displayFeatures = displayF;}

  virtual void setFarClippingDistance(const double &dist);

  virtual void setNearClippingDistance(const double &dist);
  
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

  virtual void setOgreVisibilityTest(const bool &v);

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
  void addPolygon(const std::vector<vpPoint>& corners, const unsigned int idFace = -1);
  void addPolygon(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius, const unsigned int idFace=0);

  void computeJTR(const vpMatrix& J, const vpColVector& R, vpMatrix& JTR);
  
#ifdef VISP_HAVE_COIN
  virtual void extractGroup(SoVRMLGroup *sceneGraphVRML2, vpHomogeneousMatrix &transform, unsigned int &indexFace);
  virtual void extractFaces(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, unsigned int &idFace);
  virtual void extractLines(SoVRMLIndexedLineSet* line_set, unsigned int &idFace);
  virtual void extractCylinders(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform);
#endif
  
  vpPoint getGravityCenter(const std::vector<vpPoint>& _pts);
  
  /*!
    Add a circle to track from its center, 3 points (including the center) defining the plane that contain
    the circle and its radius.

    \param p1 : Center of the circle.
    \param p2,p3 : Two points on the plane containing the circle. With the center of the circle we have 3 points
    defining the plane that contains the circle.
    \param radius : Radius of the circle.
    \param idFace : Id of the face associated to the circle.
  */
  virtual void initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius, const unsigned int idFace=0)=0;
  /*!
    Add a cylinder to track from two points on the axis (defining the length of
    the cylinder) and its radius.

    \param p1 : First point on the axis.
    \param p2 : Second point on the axis.
    \param radius : Radius of the cylinder.
    \param idFace : Id of the face associated to the cylinder.
  */
  virtual void initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const unsigned int idFace=0)=0;

  /*!
    Add the lines to track from the polygon description. If the polygon has only
    two points, it defines a single line that is always visible. If it has three or
    more corners, it defines a face. In that case the visibility of the face is computed
    in order to track the corresponding lines only if the face is visible.

    The id of the polygon is supposed to be set prior calling this function.

    \param polygon : The polygon describing the set of lines that has to be tracked.
    \param idFace : Id of the face associated to the polygon.
  */
  virtual void initFaceFromCorners(const vpMbtPolygon *polygon, const unsigned int idFace=0)=0;
  
  virtual void loadVRMLModel(const std::string& modelFile);
  virtual void loadCAOModel(const std::string& modelFile, std::vector<std::string>& vectorOfModelFilename, int& startIdFace);


  void removeComment(std::ifstream& fileId);
};


#endif

