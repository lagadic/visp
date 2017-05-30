/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Generic model based tracker. This class declares the methods to implement in
 * order to have a model based tracker.
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \file vpMbTracker.h
  \brief Generic model based tracker.
*/
#ifndef vpMbTracker_hh
#define vpMbTracker_hh

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <algorithm>
#include <cctype>
#include <locale>
#include <functional>   // std::not1


#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPoint.h>
#include <visp3/mbt/vpMbtPolygon.h>
#include <visp3/mbt/vpMbHiddenFaces.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpRobust.h>

#ifdef VISP_HAVE_COIN3D
//Work around to avoid type redefinition int8_t with Coin
// #if defined(_WIN32) && defined(VISP_HAVE_OGRE) && (_MSC_VER >= 1600) // Visual Studio 2010
//   #define HAVE_INT8_T 1
// #endif

//Inventor includes
# include <Inventor/VRMLnodes/SoVRMLGroup.h>
# include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
# include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#endif

/*!
  \class vpMbTracker
  \ingroup group_mbt_trackers
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
public:
  typedef enum {
    GAUSS_NEWTON_OPT = 0,
    LEVENBERG_MARQUARDT_OPT = 1
  } vpMbtOptimizationMethod;

protected:
  //! The camera parameters.
  vpCameraParameters cam;
  //! The current pose.
  vpHomogeneousMatrix cMo;
  //! The Degrees of Freedom to estimate
  vpMatrix oJo;
  //! Boolean to know if oJo is identity (for fast computation)
  bool isoJoIdentity;
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
  //! Flag used to specify if the gradient error criteria has to be computed or not.
  bool computeProjError;
  //! Error angle between the gradient direction of the model features projected at the resulting pose and their normal.
  double projectionError;
  //! If true, the features are displayed.
  bool displayFeatures;
  //! Optimization method used
  vpMbtOptimizationMethod m_optimizationMethod;

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
  bool ogreShowConfigDialog;
  //! Use Scanline for visibility tests
  bool useScanLine;
  //! Number of points in CAO model
  unsigned int nbPoints;
  //! Number of lines in CAO model
  unsigned int nbLines;
  //! Number of polygon lines in CAO model
  unsigned int nbPolygonLines;
  //! Number of polygon points in CAO model
  unsigned int nbPolygonPoints;
  //! Number of cylinders in CAO model
  unsigned int nbCylinders;
  //! Number of circles in CAO model
  unsigned int nbCircles;
  //! True if LOD mode is enabled
  bool useLodGeneral;
  //! True if the CAO model is loaded before the call to loadConfigFile, (deduced by the number of polygons)
  bool applyLodSettingInConfig;
  //! Minimum line length threshold for LOD mode (general setting)
  double minLineLengthThresholdGeneral;
  //! Minimum polygon area threshold for LOD mode (general setting)
  double minPolygonAreaThresholdGeneral;
  //! Map with [map.first]=parameter_names and [map.second]=type (string, number or boolean)
  std::map<std::string, std::string> mapOfParameterNames;
  //! If true, compute the interaction matrix at each iteration of the minimization. Otherwise, compute it only on the first iteration
  bool m_computeInteraction;
  //! Gain of the virtual visual servoing stage
  double m_lambda;
  //! Maximum number of iterations of the virtual visual servoing stage
  unsigned int m_maxIter;
  //! Epsilon threshold to stop the VVS optimization loop
  double m_stopCriteriaEpsilon;
  //! Initial Mu for Levenberg Marquardt optimization loop
  double m_initialMu;

public:
  vpMbTracker();
  virtual ~vpMbTracker();

  /** @name Inherited functionalities from vpMbTracker */
  //@{

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
    Get the clipping used and defined in vpPolygon3D::vpMbtPolygonClippingType.

    \return Clipping flags.
  */
  virtual inline  unsigned int getClipping() const { return clippingFlag; }

  /*!
    Get the covariance matrix. This matrix is only computed if setCovarianceComputation() is turned on.

    \sa setCovarianceComputation()
  */

  virtual vpMatrix getCovarianceMatrix() const {
    if(!computeCovariance) {
//      vpTRACE("Warning : The covariance matrix has not been computed. See setCovarianceComputation() to do it.");
      std::cerr << "Warning : The covariance matrix has not been computed. See setCovarianceComputation() to do it." << std::endl;
    }

    return covarianceMatrix;
  }

  /*!
    Get the initial value of mu used in the Levenberg Marquardt optimization loop.

    \return the initial mu value.
  */
  virtual inline double getInitialMu() const { return m_initialMu; }

  /*!
    Get the value of the gain used to compute the control law.

    \return the value for the gain.
  */
  virtual inline double getLambda() const {return m_lambda;}

  /*!
    Get the maximum number of iterations of the virtual visual servoing stage.

    \return the number of iteration
   */
  virtual inline unsigned int getMaxIter() const {return m_maxIter;}

  /*!
    Get the error angle between the gradient direction of the model features projected at the resulting pose and their normal.
    The error is expressed in degree between 0 and 90. This value is computed if setProjectionErrorComputation() is turned on.

    \return the value for the error.

    \sa setProjectionErrorComputation()
  */
  virtual double getProjectionError() const { return projectionError; }

  virtual vpColVector getEstimatedDoF() const;

  /*!
    Return the error vector \f$(s-s^*)\f$ reached after the virtual visual servoing process used to estimate the pose.

    The following example shows how to use this function to compute the norm of the residual and the norm of the residual
    normalized by the number of features that are tracked:
    \code
      tracker.track(I);
      std::cout << "Residual: " << sqrt( (tracker.getError()).sumSquare()) << std::endl;
      std::cout << "Residual normalized: " << sqrt( (tracker.getError()).sumSquare())/tracker.getError().size() << std::endl;
    \endcode

    \sa getRobustWeights()
   */
  virtual vpColVector getError() const = 0;

  /*! Return a reference to the faces structure. */
  virtual inline vpMbHiddenFaces<vpMbtPolygon>& getFaces() { return faces;}

  /*!
    Get the far distance for clipping.

    \return Far clipping value.
  */
  virtual inline  double  getFarClippingDistance() const { return distFarClip; }

  /*!
    Return the weights vector \f$w_i\f$ computed by the robust scheme.

    The following example shows how to use this function to compute the norm of the weighted residual
    and the norm of the weighted residual normalized by the sum of the weights associated to the
    features that are tracked:
    \code
      tracker.track(I);
      vpColVector w = tracker.getRobustWeights();
      vpColVector e = tracker.getError();
      vpColVector we(w.size());
      for(unsigned int i=0; i<w.size(); i++)
        we[i] = w[i]*e[i];

      std::cout << "Weighted residual: " << sqrt( (we).sumSquare() ) << std::endl;
      std::cout << "Weighted residual normalized: " << sqrt( (we).sumSquare() ) / w.sum() << std::endl;
    \endcode

    \sa getError()
   */
  virtual vpColVector getRobustWeights() const = 0;

  /*!
    Get the number of polygons (faces) representing the object to track.

    \return Number of polygons.
  */
  virtual inline unsigned int getNbPolygon() const {
    return static_cast<unsigned int>(faces.size());
  }

  /*!
    Get the near distance for clipping.

    \return Near clipping value.
  */
  virtual inline double   getNearClippingDistance() const { return distNearClip; }

  /*!
    Get the optimization method used during the tracking.
    0 = Gauss-Newton approach.
    1 = Levenberg-Marquardt approach.

    \return Optimization method.
  */
  virtual inline vpMbtOptimizationMethod getOptimizationMethod() const { return m_optimizationMethod; }

  /*!
    Return the polygon (face) "index".

    \exception vpException::dimensionError if index does not represent a good
    polygon.

    \param index : Index of the polygon to return.
    \return Pointer to the polygon index.
  */
  virtual inline vpMbtPolygon* getPolygon(const unsigned int index) {
    if(index >= static_cast<unsigned int>(faces.size()) ){
      throw vpException(vpException::dimensionError, "index out of range");
    }

    return faces[index];
  }

  virtual std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > getPolygonFaces(const bool orderPolygons=true,
      const bool useVisibility=true, const bool clipPolygon=false);

  /*!
    Get the current pose between the object and the camera.
    cMo is the matrix which can be used to express
    coordinates from the object frame to camera frame.

    \param cMo_ : the pose
  */
  virtual inline void getPose(vpHomogeneousMatrix& cMo_) const {cMo_ = this->cMo;}

  /*!
    Get the current pose between the object and the camera.
    cMo is the matrix which can be used to express
    coordinates from the object frame to camera frame.

    \return the current pose
  */
  virtual inline vpHomogeneousMatrix getPose() const {return this->cMo;}

  virtual inline double getStopCriteriaEpsilon() const { return m_stopCriteriaEpsilon; }

  // Intializer

#ifdef VISP_HAVE_MODULE_GUI
  virtual void initClick( const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp = false );
  virtual void initClick( const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
                          const std::string &displayFile = "" );
#endif

  virtual void initFromPoints( const vpImage<unsigned char>& I, const std::string& initFile );
  virtual void initFromPoints( const vpImage<unsigned char>& I, const std::vector<vpImagePoint> &points2D_list, const std::vector<vpPoint> &points3D_list );

  virtual void initFromPose(const vpImage<unsigned char>& I, const std::string &initFile);
  virtual void initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo);
  virtual void initFromPose(const vpImage<unsigned char>& I, const vpPoseVector &cPo);

  virtual void loadModel(const char *modelFile, const bool verbose=false);
  virtual void loadModel(const std::string &modelFile, const bool verbose=false);

  /*!
    Set the angle used to test polygons appearance.
    If the angle between the normal of the polygon and the line going
    from the camera to the polygon center has a value lower than
    this parameter, the polygon is considered as appearing.
    The polygon will then be tracked.

    \param a : new angle in radian.
  */
  virtual inline void setAngleAppear(const double &a) { angleAppears = a; }

  /*!
    Set the angle used to test polygons disappearance.
    If the angle between the normal of the polygon and the line going
    from the camera to the polygon center has a value greater than
    this parameter, the polygon is considered as disappearing.
    The tracking of the polygon will then be stopped.

    \param a : new angle in radian.
  */
  virtual inline void setAngleDisappear(const double &a) { angleDisappears = a; }

  /*!
    Set the camera parameters.

    \param camera : the new camera parameters
  */
  virtual void setCameraParameters(const vpCameraParameters& camera) {this->cam = camera;}

  virtual void setClipping(const unsigned int &flags);

  /*!
    Set if the covariance matrix has to be computed.

    \param flag : True if the covariance has to be computed, false otherwise.
    If computed its value is available with getCovarianceMatrix()

    \sa getCovarianceMatrix()
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
  virtual void setDisplayFeatures(const bool displayF) {displayFeatures = displayF;}

  virtual void setEstimatedDoF(const vpColVector& v);

  virtual void setFarClippingDistance(const double &dist);

  /*!
    Set the initial value of mu for the Levenberg Marquardt optimization loop.

    \param mu : initial mu.
  */
  virtual inline void setInitialMu(const double mu) { m_initialMu = mu; }

  /*!
    Set the value of the gain used to compute the control law.

    \param gain : the desired value for the gain.
  */
  virtual inline void setLambda(const double gain) {m_lambda = gain;}

  virtual void setLod(const bool useLod, const std::string &name="");

  /*!
    Set the maximum iteration of the virtual visual servoing stage.

    \param max : the desired number of iteration
   */
  virtual inline void setMaxIter(const unsigned int max) {m_maxIter = max;}

  virtual void setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name="");

  virtual void setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name="");

  virtual void setNearClippingDistance(const double &dist);

  /*!
    Set the optimization method used during the tracking.

    \param opt : Optimization method to use.
  */
  virtual inline void setOptimizationMethod(const vpMbtOptimizationMethod &opt) { m_optimizationMethod = opt; }

  /*!
    Set the minimal error (previous / current estimation) to determine if there is convergence or not.

    \param eps : Epsilon threshold.
  */
  virtual inline void setStopCriteriaEpsilon(const double eps) { m_stopCriteriaEpsilon = eps; }

  /*!
    Set if the projection error criteria has to be computed. This criteria could be used to
    detect the quality of the tracking. It computes an angle between 0 and 90 degrees that is
    available with getProjectionError(). Closer to 0 is the value, better is the tracking.

    \param flag : True if the projection error criteria has to be computed, false otherwise.

    \sa getProjectionError()
  */
  virtual void setProjectionErrorComputation(const bool &flag) { computeProjError = flag; }

  virtual void setScanLineVisibilityTest(const bool &v){ useScanLine = v; }

  virtual void setOgreVisibilityTest(const bool &v);

  void savePose(const std::string &filename) const;

#ifdef VISP_HAVE_OGRE
  /*!
    Set the ratio of visibility attempts that has to be successful to consider a polygon as visible.

    \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

    \param ratio : Ratio of succesful attempts that has to be considered. Value has to be between 0.0 (0%) and 1.0 (100%).
  */
  void setGoodNbRayCastingAttemptsRatio(const double &ratio) {
    faces.setGoodNbRayCastingAttemptsRatio(ratio);
  }
  /*!
    Set the number of rays that will be sent toward each polygon for visibility test.
    Each ray will go from the optic center of the camera to a random point inside the considered polygon.

    \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

    \param attempts Number of rays to be sent.
  */
  void setNbRayCastingAttemptsForVisibility(const unsigned int &attempts) {
    faces.setNbRayCastingAttemptsForVisibility(attempts);
  }
#endif

  /*!
    Enable/Disable the appearance of Ogre config dialog on startup.

    \warning This method has only effect when Ogre is used and Ogre visibility test is
    enabled using setOgreVisibilityTest() with true parameter.

    \param showConfigDialog : if true, shows Ogre dialog window (used to set Ogre
    rendering options) when Ogre visibility is enabled. By default, this functionality
    is turned off.
  */
  inline virtual void setOgreShowConfigDialog(const bool showConfigDialog){
    ogreShowConfigDialog = showConfigDialog;
  }

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

  /* PURE VIRTUAL METHODS */

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
    Initialise the tracking.

    \param I : Input image.
  */
  virtual void init(const vpImage<unsigned char>& I)=0;

  /*!
    Load a config file to parameterise the behavior of the tracker.

    Pure virtual method to adapt to each tracker.

    \param configFile : An xml config file to parse.
  */
  virtual void loadConfigFile(const std::string& configFile)=0;

  /*!
    Reset the tracker.
  */
  virtual void resetTracker() = 0;

  /*!
    Set the pose to be used in entry of the next call to the track() function.
    This pose will be just used once.

    \warning This function has to be called after the initialisation of the tracker.

    \param I : image corresponding to the desired pose.
    \param cdMo : Pose to affect.
  */
  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo) = 0;

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
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpMbTracker */
  //@{
  void addPolygon(const std::vector<vpPoint>& corners, const int idFace=-1, const std::string &polygonName="",
      const bool useLod=false, const double minPolygonAreaThreshold=2500.0, const double minLineLengthThreshold=50.0);
  void addPolygon(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius, const int idFace=-1,
      const std::string &polygonName="", const bool useLod=false, const double minPolygonAreaThreshold=2500.0);
  void addPolygon(const vpPoint& p1, const vpPoint &p2, const int idFace=-1, const std::string &polygonName="",
      const bool useLod=false, const double minLineLengthThreshold=50);
  void addPolygon(const std::vector<std::vector<vpPoint> > &listFaces, const int idFace=-1, const std::string &polygonName="",
      const bool useLod=false, const double minLineLengthThreshold=50);

  void createCylinderBBox(const vpPoint& p1, const vpPoint &p2, const double &radius, std::vector<std::vector<vpPoint> > &listFaces);

  virtual void computeCovarianceMatrixVVS(const bool isoJoIdentity_, const vpColVector &w_true, const vpHomogeneousMatrix &cMoPrev,
                                          const vpMatrix &L_true, const vpMatrix &LVJ_true, const vpColVector &error);

  void computeJTR(const vpMatrix& J, const vpColVector& R, vpColVector& JTR) const;

  virtual void computeVVSCheckLevenbergMarquardt(const unsigned int iter, vpColVector &error, const vpColVector &m_error_prev, const vpHomogeneousMatrix &cMoPrev,
                                                 double &mu, bool &reStartFromLastIncrement, vpColVector * const w=NULL, const vpColVector * const m_w_prev=NULL);
  virtual void computeVVSInit()=0;
  virtual void computeVVSInteractionMatrixAndResidu()=0;
  virtual void computeVVSPoseEstimation(const bool isoJoIdentity_, const unsigned int iter, vpMatrix &L, vpMatrix &LTL, vpColVector &R, const vpColVector &error,
                                        vpColVector &error_prev, vpColVector &LTR, double &mu, vpColVector &v, const vpColVector * const w=NULL, vpColVector * const m_w_prev=NULL);
  virtual void computeVVSWeights(vpRobust &robust, const vpColVector &error, vpColVector &w);

#ifdef VISP_HAVE_COIN3D
  virtual void extractGroup(SoVRMLGroup *sceneGraphVRML2, vpHomogeneousMatrix &transform, int &idFace);
  virtual void extractFaces(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, int &idFace, const std::string &polygonName="");
  virtual void extractLines(SoVRMLIndexedLineSet* line_set, int &idFace, const std::string &polygonName="");
  virtual void extractCylinders(SoVRMLIndexedFaceSet* face_set, vpHomogeneousMatrix &transform, int &idFace, const std::string &polygonName="");
#endif

  vpPoint getGravityCenter(const std::vector<vpPoint>& _pts) const;

  /*!
    Add a circle to track from its center, 3 points (including the center) defining the plane that contain
    the circle and its radius.

    \param p1 : Center of the circle.
    \param p2 : A point on the plane containing the circle.
    \param p3 : An other point on the plane containing the circle. With the center of the circle \e p1,
    \e p2 and \e p3 we have 3 points defining the plane that contains the circle.
    \param radius : Radius of the circle.
    \param idFace : Id of the face associated to the circle.
    \param name : Name of the circle.
  */
  virtual void initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
      const int idFace=0, const std::string &name="")=0;
  /*!
    Add a cylinder to track from two points on the axis (defining the length of
    the cylinder) and its radius.

    \param p1 : First point on the axis.
    \param p2 : Second point on the axis.
    \param radius : Radius of the cylinder.
    \param idFace : Id of the face associated to the cylinder.
    \param name : Name of the cylinder.
  */
  virtual void initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace=0,
      const std::string &name="")=0;

  /*!
    Add the lines to track from the polygon description. If the polygon has only
    two points, it defines a single line that is always visible. If it has three or
    more corners, it defines a face. In that case the visibility of the face is computed
    in order to track the corresponding lines only if the face is visible.

    The id of the polygon is supposed to be set prior calling this function.

    \param polygon : The polygon describing the set of lines that has to be tracked.
  */
  virtual void initFaceFromCorners(vpMbtPolygon &polygon)=0;
  virtual void initFaceFromLines(vpMbtPolygon &polygon)=0;

  virtual void loadVRMLModel(const std::string& modelFile);
  virtual void loadCAOModel(const std::string& modelFile, std::vector<std::string>& vectorOfModelFilename, int& startIdFace,
                            const bool verbose=false, const bool parent=true);

  void removeComment(std::ifstream& fileId);

  inline bool parseBoolean(std::string &input) {
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);
    std::istringstream is(input);
    bool b;
    //Parse string to boolean either in the textual representation (True/False)
    //or in numeric representation (1/0)
    is >> (input.size() > 1 ? std::boolalpha : std::noboolalpha) >> b;
    return b;
  }

  std::map<std::string, std::string> parseParameters(std::string& endLine);

  inline std::string &ltrim(std::string &s) const {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
  }

  inline std::string &rtrim(std::string &s) const {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
  }

  inline std::string &trim(std::string &s) const {
    return ltrim(rtrim(s));
  }
  //@}
};


#endif

