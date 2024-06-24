/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Pose computation.
 */

/*!
 * \file vpPose.h
 * \brief Tools for pose computation (pose from point only).
 */

#ifndef VP_POSE_H
#define VP_POSE_H

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRGBa.h>

#include <list>
#include <math.h>
#include <vector>

// Check if std:c++17 or higher.
// Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) in the declaration of the class
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
#include <map>
#include <optional>
#endif

#include <visp3/core/vpUniRand.h>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpPose
 * \ingroup group_vision_pose
 * \brief Class used for pose computation from N points (pose from point only).
 * Some of the algorithms implemented in this class are described in
 * \cite Marchand16a.
 *
 * \note It is also possible to estimate a pose from other features using
 * vpPoseFeatures class.
 *
 * To see how to use this class you can follow the \ref tutorial-pose-estimation.
*/
class VISP_EXPORT vpPose
{
public:
  //! Methods that could be used to estimate the pose from points.
  typedef enum
  {
    LAGRANGE,             /*!< Linear Lagrange approach (doesn't need an initialization) */
    DEMENTHON,            /*!< Linear Dementhon aproach (doesn't need an initialization) */
    LOWE,                 /*!< Lowe aproach based on a Levenberg Marquartd non linear
                             minimization scheme that needs an initialization from Lagrange or
                             Dementhon aproach */
    RANSAC,               /*!< Robust Ransac aproach (doesn't need an initialization) */
    LAGRANGE_LOWE,        /*!< Non linear Lowe aproach initialized by Lagrange
                             approach */
    DEMENTHON_LOWE,       /*!< Non linear Lowe aproach initialized by Dementhon
                             approach */
    VIRTUAL_VS,           /*!< Non linear virtual visual servoing approach that needs an
                             initialization from Lagrange or Dementhon aproach */
    DEMENTHON_VIRTUAL_VS, /*!< Non linear virtual visual servoing approach
                             initialized by Dementhon approach */
    LAGRANGE_VIRTUAL_VS,  /*!< Non linear virtual visual servoing approach
                             initialized by Lagrange approach */
    DEMENTHON_LAGRANGE_VIRTUAL_VS, /*!< Non linear virtual visual servoing approach
                             initialized by either Dementhon or Lagrange approach,
                             depending on which method has the smallest residual. */
  } vpPoseMethodType;

  /*!
   * Filter applied in Ransac
   */
  enum RANSAC_FILTER_FLAGS
  {
    NO_FILTER, //!< No filter is applied
    PREFILTER_DEGENERATE_POINTS, /*!< Remove degenerate points (same 3D or 2D coordinates) before the RANSAC. */
    CHECK_DEGENERATE_POINTS      /*!< Check for degenerate points during the RANSAC. */
  };

  unsigned int npt;         //!< Number of point used in pose computation
  std::list<vpPoint> listP; //!< Array of point (use here class vpPoint)

  double residual; //!< Residual in meter

public:
  // Typedef a function that checks if a pose is valid.
  typedef bool (*funcCheckValidityPose)(const vpHomogeneousMatrix &);

  /*!
   * Default constructor.
   */
  vpPose();

  /*!
   * Constructor from a vector of points.
   */
  VP_EXPLICIT vpPose(const std::vector<vpPoint> &lP);

  /*!
   * Destructor that deletes the array of point (freed the memory).
   */
  virtual ~vpPose();

  /*!
   * Add a new point in the array of points.
   * \param P : Point to add in the array of point.
   * \warning Considering a point from the class vpPoint, oX, oY, and oZ will
   * represent the 3D coordinates of the point in the object frame and x and y
   * its 2D coordinates in the image plane. These 5 fields must be initialized
   * to be used within this function.
   */
  void addPoint(const vpPoint &P);

  /*!
   * Add (append) a list of points in the array of points.
   * \param  lP : List of points to add (append).
   * \warning Considering a point from the class vpPoint, oX, oY, and oZ will
   * represent the 3D coordinates of the point in the object frame and x and y
   * its 2D coordinates in the image plane. These 5 fields must be initialized
   * to be used within this function.
   */
  void addPoints(const std::vector<vpPoint> &lP);

  /*!
   * Delete the array of point
   */
  void clearPoint();

  /*!
   * Compute the pose according to the desired method which are:
   * - vpPose::LAGRANGE: Linear Lagrange approach (test is done to switch between
   *   planar and non planar algorithm)
   * - vpPose::DEMENTHON: Linear Dementhon approach (test is done to switch
   *   between planar and non planar algorithm)
   * - vpPose::LOWE: Lowe aproach based on a Levenberg Marquartd non linear
   *   minimization scheme that needs an initialization from Lagrange or Dementhon
   *   aproach
   * - vpPose::LAGRANGE_LOWE: Non linear Lowe aproach initialized by Lagrange
   *   approach
   * - vpPose::DEMENTHON_LOWE: Non linear Lowe aproach initialized by Dementhon
   *   approach
   * - vpPose::VIRTUAL_VS: Non linear virtual visual servoing approach that needs
   *   an initialization from Lagrange or Dementhon aproach
   * - vpPose::DEMENTHON_VIRTUAL_VS: Non linear virtual visual servoing approach
   *   initialized by Dementhon approach
   * - vpPose::LAGRANGE_VIRTUAL_VS: Non linear virtual visual servoing approach
   *   initialized by Lagrange approach
   * - vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS: Non linear virtual visual servoing approach
   *   initialized by either Dementhon or Lagrange approach, depending on which method
   *   has the smallest residual.
   * - vpPose::RANSAC: Robust Ransac aproach (doesn't need an initialization)
   */
  bool computePose(vpPoseMethodType method, vpHomogeneousMatrix &cMo, funcCheckValidityPose func = nullptr);

  /*!
   * @brief Method that first computes the pose \b cMo using the linear approaches of Dementhon and Lagrange
   * and then uses the non-linear Virtual Visual Servoing approach to affine the pose which
   * had the lowest residual.
   *
   * @param cMo the pose of the object with regard to the camera.
   * @return true the pose computation was successful.
   * @return false an error occurred during the pose computation.
   */
  bool computePoseDementhonLagrangeVVS(vpHomogeneousMatrix &cMo);

  /*!
   * \brief Compute and return the sum of squared residuals expressed in meter^2 for
   * the pose matrix \e cMo.
   *
   * \param cMo : Input pose. The matrix that defines the pose to be tested.
   *
   * \return The value of the sum of squared residuals in meter^2.
   *
   * \note There is also the possibility to compute the residual expressed in pixel^2
   * using one of the following methods:
   * - vpPose::computeResidual(const vpHomogeneousMatrix &, const vpCameraParameters &)
   * - vpPose::computeResidual(const vpHomogeneousMatrix &, const vpCameraParameters &am, vpColVector &)
   */
  double computeResidual(const vpHomogeneousMatrix &cMo) const;

  /*!
   * \brief Compute and return the sum of squared residuals expressed in pixel^2 for
   * the pose matrix \e cMo.
   *
   * \param cMo : Input pose. The matrix that defines the pose to be tested.
   * \param cam : Camera parameters used to observe the points.
   *
   * \return The value of the sum of squared residuals in pixel^2.
   *
   * \note There is also the possibility to compute the residual expressed in meter^2 using
   * vpPose::computeResidual(const vpHomogeneousMatrix &)
   */
  double computeResidual(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam) const;

  /*!
   * \brief Compute and return the sum of squared residuals expressed in pixel^2 for
   * the pose matrix \e cMo.
   *
   * \param cMo : Input pose. The matrix that defines the pose to be tested.
   * \param cam : Camera parameters used to observe the points.
   * \param squaredResidual: Input/output vector that will be resized and will contain the squared residuals
   * expressed in pixel^2 of each point.
   *
   * \return The value of the sum of squared residuals in pixel^2.
   *
   * \note There is also the possibility to compute the residual expressed in meter^2 using
   * vpPose::computeResidual(const vpHomogeneousMatrix &)
   */
  double computeResidual(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, vpColVector &squaredResidual) const;

  /*!
   * Test the coplanarity of the set of points
   *
   * \param coplanar_plane_type:
   * 1: if plane x=cst
   * 2: if plane y=cst
   * 3: if plane z=cst
   * 4: if the points are collinear.
   * 0: any other plane
   * \param p_a: if different from null, it will be set to equal the a coefficient of the potential plan.
   * \param p_b: if different from null, it will be set to equal the b coefficient of the potential plan.
   * \param p_c: if different from null, it will be set to equal the c coefficient of the potential plan.
   * \param p_d: if different from null, it will be set to equal the d coefficient of the potential plan.
   * \return true if points are coplanar false otherwise.
   */
  bool coplanar(int &coplanar_plane_type, double *p_a = nullptr, double *p_b = nullptr, double *p_c = nullptr, double *p_d = nullptr);

  /*!
   * Display the coordinates of the points in the image plane that are used to
   * compute the pose in image I.
   */
  void displayModel(vpImage<unsigned char> &I, vpCameraParameters &cam, vpColor col = vpColor::none);

  /*!
   * Display the coordinates of the points in the image plane that are used to
   * compute the pose in image I.
   */
  void displayModel(vpImage<vpRGBa> &I, vpCameraParameters &cam, vpColor col = vpColor::none);

  /*!
   * Compute the pose using Dementhon approach for planar objects
   * this is a direct implementation of the algorithm proposed by
   * Dementhon in his PhD.
   */
  void poseDementhonPlan(vpHomogeneousMatrix &cMo);

  /*!
   * Compute the pose using Dementhon approach for non planar objects.
   * This is a direct implementation of the algorithm proposed by
   * Dementhon and Davis in their 1995 paper \cite Dementhon95.
   */
  void poseDementhonNonPlan(vpHomogeneousMatrix &cMo);

  /*!
   * Compute the pose of a planar object using Lagrange approach.
   *
   * \param cMo : Estimated pose. No initialisation is requested to estimate cMo.
   * \param p_isPlan : if different from nullptr, indicates if the object is planar or not.
   * \param p_a : if different from nullptr, the a coefficient of the plan formed by the points.
   * \param p_b : if different from nullptr, the b coefficient of the plan formed by the points.
   * \param p_c : if different from nullptr, the c coefficient of the plan formed by the points.
   * \param p_d : if different from nullptr, the d coefficient of the plan formed by the points.
   */
  void poseLagrangePlan(vpHomogeneousMatrix &cMo, bool *p_isPlan = nullptr, double *p_a = nullptr, double *p_b = nullptr,
                        double *p_c = nullptr, double *p_d = nullptr);

  /*!
   * Compute the pose of a non planar object using Lagrange approach.
   *
   * \param cMo : Estimated pose. No initialisation is requested to estimate cMo.
   */
  void poseLagrangeNonPlan(vpHomogeneousMatrix &cMo);

  /*!
   * \brief  Compute the pose using the Lowe non linear approach
   * it consider the minimization of a residual using
   * the levenberg marquartd approach.
   *
   * The approach has been proposed by D.G Lowe in 1992 paper \cite Lowe92a.
   */
  void poseLowe(vpHomogeneousMatrix &cMo);

  /*!
   * Compute the pose using the Ransac approach.
   *
   * \param cMo : Computed pose
   * \param func : Pointer to a function that takes in parameter a
   * vpHomogeneousMatrix and returns true if the pose check is OK or false
   * otherwise
   * \return True if we found at least 4 points with a reprojection
   * error below ransacThreshold.
   * \note You can enable a multithreaded version if you have C++11 enabled using setUseParallelRansac().
   * The number of threads used can then be set with setNbParallelRansacThreads().
   * Filter flag can be used  with setRansacFilterFlag().
   */
  bool poseRansac(vpHomogeneousMatrix &cMo, funcCheckValidityPose func = nullptr);

  /*!
   * Compute the pose using virtual visual servoing approach and
   * a robust control law.
   *
   * This approach is described in \cite Comport06b.
   */
  void poseVirtualVSrobust(vpHomogeneousMatrix &cMo);

  /*!
   * Compute the pose using virtual visual servoing approach.
   *
   * This approach is described in \cite Marchand02c.
   */
  void poseVirtualVS(vpHomogeneousMatrix &cMo);

  /*!
   * Print to std::cout points used as input.
   */
  void printPoint();

  /*!
   * Set singular value threshold in Dementhon pose estimation method.
   */
  void setDementhonSvThreshold(const double &svThresh);

  /*!
   * Set distance threshold to consider that when a point belongs to a plane.
   */
  void setDistToPlaneForCoplanTest(double d);

  /*!
   * Set virtual visual servoing gain.
   */
  void setLambda(double lambda) { m_lambda = lambda; }

  /*!
   * Set virtual visual servoing epsilon value used in the pseudo-inverse.
   */
  void setVvsEpsilon(const double eps)
  {
    if (eps >= 0) {
      vvsEpsilon = eps;
    }
    else {
      throw vpException(vpException::badValue, "Epsilon value must be >= 0.");
    }
  }

  /*!
   * Set virtual visual servoing pose estimator maximum number od iterations.
   */
  void setVvsIterMax(int nb) { vvsIterMax = nb; }

  /*!
   * Set Ransac requested number of inliers to reach consensus.
   */
  void setRansacNbInliersToReachConsensus(const unsigned int &nbC) { ransacNbInlierConsensus = nbC; }

  /*!
   * Set Ransac threshold.
   */
  void setRansacThreshold(const double &t)
  {
    // Test whether or not t is > 0
    if (t > std::numeric_limits<double>::epsilon()) {
      ransacThreshold = t;
    }
    else {
      throw vpException(vpException::badValue, "The Ransac threshold must be positive as we deal with distance.");
    }
  }

  /*!
   * Set Ransac number of trials.
   */
  void setRansacMaxTrials(const int &rM) { ransacMaxTrials = rM; }

  /*!
   * Get the number of inliers.
   */
  unsigned int getRansacNbInliers() const { return static_cast<unsigned int>(ransacInliers.size()); }

  /**
   * Get the vector of indexes corresponding to inliers.
   */
  std::vector<unsigned int> getRansacInlierIndex() const { return ransacInlierIndex; }

  /*!
   * Get the vector of inliers.
   */
  std::vector<vpPoint> getRansacInliers() const { return ransacInliers; }

  /*!
   * Set if the covariance matrix has to be computed in the Virtual Visual
   * Servoing approach.
   *
   * \param flag : True if the covariance has to be computed, false otherwise.
   */
  void setCovarianceComputation(const bool &flag) { computeCovariance = flag; }

  /*!
   * Get the covariance matrix computed in the Virtual Visual Servoing
   * approach.
   *
   * \warning The compute covariance flag has to be true if you want to compute
   * the covariance matrix.
   *
   * \sa setCovarianceComputation
   */
  vpMatrix getCovarianceMatrix() const
  {
    if (!computeCovariance) {
      std::cout << "Warning: The covariance matrix has not been computed. See setCovarianceComputation() to do it." << std::endl;
    }
    return covarianceMatrix;
  }

  /*!
   * Set RANSAC filter flag.
   *
   * \param flag : RANSAC flag to use to prefilter or perform degenerate configuration check.
   * \sa RANSAC_FILTER_FLAGS
   * \warning Prefilter degenerate points consists to not add subsequent degenerate points. This means that
   * it is possible to discard a valid point and keep an invalid point if the invalid point
   * is added first. It is faster to prefilter for duplicate points instead of checking for degenerate
   * configuration at each time.
   * \note By default the flag is set to NO_FILTER.
   */
  inline void setRansacFilterFlag(const RANSAC_FILTER_FLAGS &flag) { ransacFlag = flag; }

  /*!
   * Get the number of threads for the parallel RANSAC implementation.
   *
   * \sa setNbParallelRansacThreads
   */
  inline int getNbParallelRansacThreads() const { return nbParallelRansacThreads; }

  /*!
   * Set the number of threads for the parallel RANSAC implementation.
   *
   * \note You have to enable the parallel version with setUseParallelRansac().
   * If the number of threads is 0, the number of threads to use is
   * automatically determined with C++11.
   * \sa setUseParallelRansac
   */
  inline void setNbParallelRansacThreads(int nb) { nbParallelRansacThreads = nb; }

  /*!
   * \return True if the parallel RANSAC version should be used (depends also to C++11 availability).
   *
   * \sa setUseParallelRansac
   */
  inline bool getUseParallelRansac() const { return useParallelRansac; }

  /*!
   * Set if parallel RANSAC version should be used or not (only if C++11).
   *
   * \note Need C++11 or higher.
   */
  inline void setUseParallelRansac(bool use) { useParallelRansac = use; }

  /*!
   * Get the vector of points.
   *
   * \return The vector of points.
   */
  std::vector<vpPoint> getPoints() const
  {
    std::vector<vpPoint> vectorOfPoints(listP.begin(), listP.end());
    return vectorOfPoints;
  }

  /*!
   * Compute the pose of a planar object from corresponding 2D-3D point coordinates and depth map.
   * Depth map is here used to estimate the 3D plane of the object.
   *
   * \param[in] depthMap : Depth map aligned to the color image from where \e corners are extracted.
   * \param[in] corners : Vector of 2D pixel coordinates of the object in an image.
   * \param[in] colorIntrinsics : Camera parameters used to convert \e corners from pixel to meters.
   * \param[in] point3d : Vector of 3D points corresponding to the model of the planar object.
   * \param[out] cMo : Computed pose.
   * \param[out] confidence_index : Confidence index in range [0, 1]. When values are close to 1, it means
   * that pose estimation confidence is high. Values close to 0 indicate that pose is not well estimated.
   * This confidence index corresponds to the product between the normalized number of depth data covering the tag
   * and the normalized M-estimator weights returned by the robust estimation of the tag 3D plane.
   *
   * The following code snippet implemented in tutorial-apriltag-detector-live-rgbd-realsense.cpp shows how
   * to use this function to estimate the pose of an AprilTag using this method:
   * \snippet tutorial-apriltag-detector-live-rgbd-realsense.cpp Pose from depth map
   *
   * \return true if pose estimation succeed, false otherwise.
   */
  static bool computePlanarObjectPoseFromRGBD(const vpImage<float> &depthMap, const std::vector<vpImagePoint> &corners,
                                              const vpCameraParameters &colorIntrinsics,
                                              const std::vector<vpPoint> &point3d, vpHomogeneousMatrix &cMo,
                                              double *confidence_index = nullptr);

  /*!
   * Compute the pose of multiple planar object from corresponding 2D-3D point coordinates and depth map.
   * Depth map is here used to estimate the 3D plane of each planar object.
   *
   * This implementation is reserved for the case where multiple planar objects are considered and where an
   * object like robot arm obstruct the view and interfere with plane equation estimation for each single object.
   * Therefore this function considers only the 3D point inside the visible tags.
   *
   * \param[in] depthMap : Depth map aligned to the color image from where \e corners are extracted.
   *
   * \param[in] corners : Vector where each element is a vector containing 2D pixel coordinates of the 2D polygon that
   * defines the object edges in an image.
   *
   * \param[in] colorIntrinsics : Camera parameters used to convert \e corners from pixel to
   * meters.
   *
   * \param[in] point3d : Vector where each element is a vector containing 3D points coordinates of the 3D polygon that
   * defines the model of the planar object.
   *
   * \param[out] cMo : Computed pose.
   *
   * \param[out] confidence_index : Confidence index in range [0, 1]. When values are close to 1, it means that pose
   * estimation confidence is high. Values close to 0 indicate that pose is not well estimated. This confidence index
   * corresponds to the product between the normalized number of depth data covering the tag and the normalized
   * M-estimator weights returned by the robust estimation of the tag 3D plane.
   *
   * \param[in] coplanar_points : There are cases where all the planar objects are not in the same plane. In order to
   * differentiate these cases, this parameter will be used to compute the common plane for all objects if its value is
   * true and compute the plane individually for each object otherwise.
   *
   * \return true if pose estimation succeed, false otherwise.
   */
  static bool computePlanarObjectPoseFromRGBD(const vpImage<float> &depthMap,
                                              const std::vector<std::vector<vpImagePoint> > &corners,
                                              const vpCameraParameters &colorIntrinsics,
                                              const std::vector<std::vector<vpPoint> > &point3d,
                                              vpHomogeneousMatrix &cMo, double *confidence_index = nullptr,
                                              bool coplanar_points = true);

  /*!
   * Compute the number of RANSAC iterations to ensure with a probability \e p
   * that at least one of the random samples of \e s points is free from
   * outliers.
   * \note See: Hartley and Zisserman, Multiple View Geometry in
   * Computer Vision, p119 (2. How many samples?).
   *
   * \param probability : Probability that at least one of the random samples is
   * free from outliers (typically p=0.99).
   * \param epsilon : Probability that a
   * selected point is an outlier (between 0 and 1).
   * \param sampleSize : Minimum
   * number of points to estimate the model (4 for a pose estimation).
   * \param maxIterations : Upper bound on the number of iterations or -1 for INT_MAX.
   * \return The number of RANSAC iterations to ensure with a probability \e p
   * that at least one of the random samples of \e s points is free from outliers
   * or \p maxIterations if it exceeds the desired upper bound or \e INT_MAX if
   * maxIterations=-1.
   */
  static int computeRansacIterations(double probability, double epsilon, const int sampleSize = 4,
                                     int maxIterations = 2000);

  /*!
   * Display in the image \e I the pose represented by its homogenous
   * transformation \e cMo as a 3 axis frame.
   * \param I: Image where the pose is displayed in overlay.
   * \param cMo: Considered pose to display.
   * \param cam: Camera parameters associated to image \e I.
   * \param size: length in meter of the axis that will be displayed.
   * \param col: Color used to display the 3 axis. If vpColor::none, red, green and blue will represent x-axis, y-axis
   * and z-axis respectively.
   */
  static void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                      vpColor col = vpColor::none);

  /*!
   * Display in the image \e I the pose represented by its homogenous
   * transformation \e cMo as a 3 axis frame.
   * \param I: Image where the pose is displayed in overlay.
   * \param cMo: Considered pose to display.
   * \param cam: Camera parameters associated to image \e I.
   * \param size: length in meter of the axis that will be displayed.
   * \param col: Color used to display the 3 axis. If vpColor::none, red, green and blue will represent x-axis, y-axis
   * and z-axis respectively.
   */
  static void display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                      vpColor col = vpColor::none);

  /*!
   * Match a vector p2D of  2D point (x,y)  and  a vector p3D of 3D points
   * (X,Y,Z) using the Ransac algorithm.
   *
   * At least numberOfInlierToReachAConsensus of true correspondence are required
   * to validate the pose
   *
   * The inliers are given in a vector of vpPoint listInliers.
   *
   * The pose is returned in cMo.
   *
   * \param p2D : Vector of 2d points (x and y attributes are used).
   * \param p3D : Vector of 3d points (oX, oY and oZ attributes are used).
   * \param numberOfInlierToReachAConsensus : The minimum number of inlier to
   * have to consider a trial as correct.
   * \param threshold : The maximum error
   * allowed between the 2d points and the reprojection of its associated 3d
   * points by the current pose (in meter).
   * \param ninliers : Number of inliers found for the best solution.
   * \param listInliers : Vector of points (2d and
   * 3d) that are inliers for the best solution.
   * \param cMo : The computed pose (best solution).
   * \param maxNbTrials : Maximum number of trials before
   * considering a solution fitting the required \e
   * numberOfInlierToReachAConsensus and \e threshold cannot be found.
   * \param useParallelRansac : If true, use parallel RANSAC version (if C++11 is available).
   * \param nthreads : Number of threads to use, if 0 the number of CPU threads will be determined.
   * \param func : Pointer to a function that takes in parameter a vpHomogeneousMatrix and returns
   * true if the pose check is OK or false otherwise
   */
  static void findMatch(std::vector<vpPoint> &p2D, std::vector<vpPoint> &p3D,
                        const unsigned int &numberOfInlierToReachAConsensus, const double &threshold,
                        unsigned int &ninliers, std::vector<vpPoint> &listInliers, vpHomogeneousMatrix &cMo,
                        const int &maxNbTrials = 10000, bool useParallelRansac = true, unsigned int nthreads = 0,
                        funcCheckValidityPose func = nullptr);

#ifdef VISP_HAVE_HOMOGRAPHY
  /*!
   * Carries out the camera pose the image of a rectangle and
   * the intrinsic parameters, the length on x axis is known but the
   * proportion of the rectangle are unknown.
   *
   * This method is taken from "Markerless Tracking using Planar Structures
   * in the Scene" by Gilles Simon. The idea is to compute the homography H
   * giving the image point of the rectangle by associating them with the
   * coordinates (0,0)(1,0)(1,1/s)(0,1/s) (the rectangle is on the Z=0 plane).
   * If K is the intrinsic parameters matrix, we have  s = ||Kh1||/ ||Kh2||. s
   * gives us the proportion of the rectangle
   *
   * \param p1,p2,p3,p4: the image of the corners of the rectangle
   * (respectively the image of  (0,0),(lx,0),(lx,lx/s) and (0,lx/s)) (input)
   * \param cam: the camera used (input)
   * \param lx: the rectangle size on the x axis (input)
   * \param cMo: the camera pose (output)
   * \return int : OK if no pb occurs
   */
  static double poseFromRectangle(vpPoint &p1, vpPoint &p2, vpPoint &p3, vpPoint &p4, double lx,
                                  vpCameraParameters &cam, vpHomogeneousMatrix &cMo);
#endif

  // Check if std:c++17 or higher.
  // Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) in the declaration of the class
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

  /*!
   * Compute the pose of a planar object from corresponding 2D-3D point coordinates and plane equation.
   * Here at least 3 points are required.
   *
   * \param[in] plane_in_camera_frame : Plane in camera frame.
   * \param[in] pts : Object points.
   * \param[in] ips : Points in the image.
   * \param[in] camera_intrinsics : Camera parameters.
   * \param[in] cMo_init : Camera to object frame transformation used as initialization. When set to `std::nullopt`,
   * this transformation is computed internally.
   * \param[in] enable_vvs : When true, refine estimated pose using a virtual visual servoing scheme.
   * \return Homogeneous matrix \f${^c}{\bf M}_o\f$ between camera frame and object frame when estimation succeed,
   * nullopt otherwise.
   */
  template <typename DataId>
  static std::optional<vpHomogeneousMatrix> computePlanarObjectPoseWithAtLeast3Points(
    const vpPlane &plane_in_camera_frame, const std::map<DataId, vpPoint> &pts,
    const std::map<DataId, vpImagePoint> &ips, const vpCameraParameters &camera_intrinsics,
    std::optional<vpHomogeneousMatrix> cMo_init = std::nullopt, bool enable_vvs = true)
  {
    if (cMo_init && (!enable_vvs)) {
      throw(vpException(
        vpException::fatalError,
        "It doesn't make sense to use an initialized pose without enabling VVS to compute the pose from 4 points"));
    }

    // Check if detection and model fit
    // - The next line produces an internal compiler error with Visual Studio 2017:
    //   modules\vision\include\visp3/vision/vpPose.h(404): fatal error C1001: An internal error has occurred in the
    //   compiler.
    //    To work around this problem, try simplifying or changing the program near the locations listed above.
    //   Please choose the Technical Support command on the Visual C++
    //    Help menu, or open the Technical Support help file for more information
    // - Note that the next line builds with Visual Studio 2022.
    // for ([[maybe_unused]] const auto &[ip_id, _] : ips) {
    for (const auto &[ip_id, ip_unused] : ips) {
      (void)ip_unused;
      if (pts.find(ip_id) == end(pts)) {
        throw(vpException(vpException::fatalError,
                          "Cannot compute pose with points and image points which do not have the same IDs"));
      }
    }

    std::vector<vpPoint> P {}, Q {};
    // The next line in C++17 produces a build error with Visual Studio 2017, that's why we
    // use rather C++11 to loop through std::map
    // for (auto [pt_id, pt] : pts) {
    for (const auto &pt_map : pts) {
      if (ips.find(pt_map.first) != end(ips)) {
        double x = 0, y = 0;
        vpPoint pt = pt_map.second;
        vpPixelMeterConversion::convertPoint(camera_intrinsics, ips.at(pt_map.first), x, y);
        const auto Z = plane_in_camera_frame.computeZ(x, y);

        pt.set_x(x);
        pt.set_y(y);
        pt.set_Z(Z);

        Q.push_back(pt);
        P.emplace_back(x * Z, y * Z, Z);
      }
    }

    if (Q.size() < 3) {
      return std::nullopt;
    }

    auto cMo = cMo_init.value_or(vpHomogeneousMatrix::compute3d3dTransformation(P, Q));
    if (!cMo.isValid()) {
      return std::nullopt;
    }

    return enable_vvs ? vpPose::poseVirtualVSWithDepth(Q, cMo).value_or(cMo) : cMo;
  }

  /*!
   * Compute the pose by virtual visual servoing using x,y and Z point coordinates as visual features.
   * We recall that x,y are the coordinates of a point in the image plane which are obtained by perspective
   * projection, while Z is the 3D coordinate of the point along the camera frame Z-axis.
   *
   * \param[in] cMo : Pose initial value used to initialize the non linear pose estimation algorithm.
   * \param[in] points : A vector of points with [x,y,Z] values used as visual features.
   * \return Estimated pose when the minimization converged, of std::nullopt when it failed.
   */
  static std::optional<vpHomogeneousMatrix> poseVirtualVSWithDepth(const std::vector<vpPoint> &points,
                                                                   const vpHomogeneousMatrix &cMo);
#endif

protected:
  double m_lambda; //!< Parameters use for the virtual visual servoing approach
  double m_dementhonSvThresh; //!< SVD threshold use for the pseudo-inverse computation in poseDementhonPlan

  /*!
   * Compute and return the residual corresponding to the sum of squared residuals
   * in meter^2 for the pose matrix \e cMo.
   *
   * \param cMo : the matrix that defines the pose to be tested.
   *
   * \return the value of the sum of squared residuals in meter^2.
   */
  double computeResidualDementhon(const vpHomogeneousMatrix &cMo);

  /*!
   * Method used in poseDementhonPlan()
   * Return 0 if success, -1 if failure.
   */
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo);

private:
  void callLagrangePose(vpHomogeneousMatrix &cMo);

  //! Define the maximum number of iteration in VVS
  int vvsIterMax;
  //! Variable used in the Dementhon approach
  std::vector<vpPoint> c3d;
  //! Flag used to specify if the covariance matrix has to be computed or not.
  bool computeCovariance;
  //! Covariance matrix
  vpMatrix covarianceMatrix;
  //! Found a solution when there are at least a minimum number of points in
  //! the consensus set
  unsigned int ransacNbInlierConsensus;
  //! Maximum number of iterations for the RANSAC
  int ransacMaxTrials;
  //! List of inlier points
  std::vector<vpPoint> ransacInliers;
  //! List of inlier point indexes (from the input list)
  std::vector<unsigned int> ransacInlierIndex;
  //! RANSAC threshold to consider a sample inlier or not
  double ransacThreshold;
  //! Minimal distance point to plane to consider if the point belongs or not
  //! to the plane
  double distToPlaneForCoplanarityTest;
  //! RANSAC flag to remove or not degenerate points
  RANSAC_FILTER_FLAGS ransacFlag;
  //! List of points used for the RANSAC (std::vector is contiguous whereas
  //! std::list is a linked list)
  std::vector<vpPoint> listOfPoints;
  //! If true, use a parallel RANSAC implementation
  bool useParallelRansac;
  //! Number of threads to spawn for the parallel RANSAC implementation
  int nbParallelRansacThreads;
  //! Stop the optimization loop when the residual change (|r-r_prec|) <=
  //! epsilon
  double vvsEpsilon;

  /*!
   * Class dedicated to parallelize RANSAC.
   */
  class vpRansacFunctor
  {
  public:
    /*!
     * Constructor.
     */
    vpRansacFunctor(const vpHomogeneousMatrix &cMo_, unsigned int ransacNbInlierConsensus_, const int ransacMaxTrials_,
      double ransacThreshold_, unsigned int initial_seed_, bool checkDegeneratePoints_,
      const std::vector<vpPoint> &listOfUniquePoints_, funcCheckValidityPose func_)
      : m_best_consensus(), m_checkDegeneratePoints(checkDegeneratePoints_), m_cMo(cMo_), m_foundSolution(false),
      m_func(func_), m_listOfUniquePoints(listOfUniquePoints_), m_nbInliers(0), m_ransacMaxTrials(ransacMaxTrials_),
      m_ransacNbInlierConsensus(ransacNbInlierConsensus_), m_ransacThreshold(ransacThreshold_),
      m_uniRand(initial_seed_)
    { }

    /*!
     * Operator() that calls Ransac.
     */
    void operator()() { m_foundSolution = poseRansacImpl(); }

    /*!
     * Access the return value.
     */
    bool getResult() const { return m_foundSolution; }

    /*!
     * Get Ransac best consensus.
     */
    std::vector<unsigned int> getBestConsensus() const { return m_best_consensus; }

    /*!
     * Get Ransac estimated pose.
     */
    vpHomogeneousMatrix getEstimatedPose() const { return m_cMo; }

    /*!
     * Get Ransac number of inliers.
     */
    unsigned int getNbInliers() const { return m_nbInliers; }

  private:
    std::vector<unsigned int> m_best_consensus; //!< Best consensus
    bool m_checkDegeneratePoints; //!< Flag to check for degenerate points
    vpHomogeneousMatrix m_cMo; //!< Estimated pose
    bool m_foundSolution; //!< Solution found
    funcCheckValidityPose m_func; //!< Pointer to ransac function
    std::vector<vpPoint> m_listOfUniquePoints; //!< List of unique points
    unsigned int m_nbInliers; //!< Number of inliers
    int m_ransacMaxTrials; //!< Ransac max trial number
    unsigned int m_ransacNbInlierConsensus; //!< Number of inliers to check for a consensus
    double m_ransacThreshold; //!< Residual threshold
    vpUniRand m_uniRand; //!< Uniform random generator

    /*!
     * Ransac implementation.
     * \return true when a solution is found.
     */
    bool poseRansacImpl();
  };
};

END_VISP_NAMESPACE

#endif
