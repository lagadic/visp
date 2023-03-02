/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 * Aurelien Yol
 * Fabien Spindler
 * Julien Dufour
 *
 *****************************************************************************/

/*!
  \file vpPose.h
  \brief Tools for pose computation (pose from point only).
*/

#ifndef _vpPose_h_
#define _vpPose_h_

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/vision/vpHomography.h>
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#include <visp3/core/vpList.h>
#endif
#include <visp3/core/vpThread.h>

#include <list>
#include <math.h>
#include <vector>
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <atomic>
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) &&                                                                     \
    (!defined(_MSC_VER) || ((VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && (_MSC_VER >= 1911)))
#include <map>
#include <optional>
#endif

#include <visp3/core/vpUniRand.h>

/*!
  \class vpPose
  \ingroup group_vision_pose
  \brief Class used for pose computation from N points (pose from point only).
  Some of the algorithms implemented in this class are described in
  \cite Marchand16a.

  \note It is also possible to estimate a pose from other features using
  vpPoseFeatures class.

  To see how to use this class you can follow the \ref tutorial-pose-estimation.
*/

class VISP_EXPORT vpPose
{
public:
  //! Methods that could be used to estimate the pose from points.
  typedef enum {
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
    LAGRANGE_VIRTUAL_VS   /*!< Non linear virtual visual servoing approach
                             initialized by Lagrange approach */
  } vpPoseMethodType;

  enum RANSAC_FILTER_FLAGS {
    NO_FILTER,
    PREFILTER_DEGENERATE_POINTS, /*!< Remove degenerate points (same 3D or 2D coordinates) before the RANSAC. */
    CHECK_DEGENERATE_POINTS      /*!< Check for degenerate points during the RANSAC. */
  };

  unsigned int npt;         //!< Number of point used in pose computation
  std::list<vpPoint> listP; //!< Array of point (use here class vpPoint)

  double residual; //!< Residual in meter

protected:
  double lambda; //!< parameters use for the virtual visual servoing approach

private:
  //! define the maximum number of iteration in VVS
  int vvsIterMax;
  //! variable used in the Dementhon approach
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
  double distanceToPlaneForCoplanarityTest;
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

  // For parallel RANSAC
  class RansacFunctor
  {
  public:
    RansacFunctor(const vpHomogeneousMatrix &cMo_, unsigned int ransacNbInlierConsensus_, const int ransacMaxTrials_,
                  double ransacThreshold_, unsigned int initial_seed_, bool checkDegeneratePoints_,
                  const std::vector<vpPoint> &listOfUniquePoints_, bool (*func_)(const vpHomogeneousMatrix &))
      : m_best_consensus(), m_checkDegeneratePoints(checkDegeneratePoints_), m_cMo(cMo_), m_foundSolution(false),
        m_func(func_), m_listOfUniquePoints(listOfUniquePoints_), m_nbInliers(0), m_ransacMaxTrials(ransacMaxTrials_),
        m_ransacNbInlierConsensus(ransacNbInlierConsensus_), m_ransacThreshold(ransacThreshold_),
        m_uniRand(initial_seed_)
    {
    }

    void operator()() { m_foundSolution = poseRansacImpl(); }

    // Access the return value.
    bool getResult() const { return m_foundSolution; }

    std::vector<unsigned int> getBestConsensus() const { return m_best_consensus; }

    vpHomogeneousMatrix getEstimatedPose() const { return m_cMo; }

    unsigned int getNbInliers() const { return m_nbInliers; }

  private:
    std::vector<unsigned int> m_best_consensus;
    bool m_checkDegeneratePoints;
    vpHomogeneousMatrix m_cMo;
    bool m_foundSolution;
    bool (*m_func)(const vpHomogeneousMatrix &);
    std::vector<vpPoint> m_listOfUniquePoints;
    unsigned int m_nbInliers;
    int m_ransacMaxTrials;
    unsigned int m_ransacNbInlierConsensus;
    double m_ransacThreshold;
    vpUniRand m_uniRand;

    bool poseRansacImpl();
  };

protected:
  double computeResidualDementhon(const vpHomogeneousMatrix &cMo);

  // method used in poseDementhonPlan()
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo);

public:
  vpPose();
  vpPose(const std::vector<vpPoint> &lP);
  virtual ~vpPose();
  void addPoint(const vpPoint &P);
  void addPoints(const std::vector<vpPoint> &lP);
  void clearPoint();

  bool computePose(vpPoseMethodType method, vpHomogeneousMatrix &cMo, bool (*func)(const vpHomogeneousMatrix &) = NULL);
  double computeResidual(const vpHomogeneousMatrix &cMo) const;
  bool coplanar(int &coplanar_plane_type);
  void displayModel(vpImage<unsigned char> &I, vpCameraParameters &cam, vpColor col = vpColor::none);
  void displayModel(vpImage<vpRGBa> &I, vpCameraParameters &cam, vpColor col = vpColor::none);

  void poseDementhonPlan(vpHomogeneousMatrix &cMo);
  void poseDementhonNonPlan(vpHomogeneousMatrix &cMo);
  void poseLagrangePlan(vpHomogeneousMatrix &cMo);
  void poseLagrangeNonPlan(vpHomogeneousMatrix &cMo);
  void poseLowe(vpHomogeneousMatrix &cMo);
  bool poseRansac(vpHomogeneousMatrix &cMo, bool (*func)(const vpHomogeneousMatrix &) = NULL);
  void poseVirtualVSrobust(vpHomogeneousMatrix &cMo);
  void poseVirtualVS(vpHomogeneousMatrix &cMo);
  void printPoint();
  void setDistanceToPlaneForCoplanarityTest(double d);
  void setLambda(double a) { lambda = a; }
  void setVvsEpsilon(const double eps)
  {
    if (eps >= 0) {
      vvsEpsilon = eps;
    } else {
      throw vpException(vpException::badValue, "Epsilon value must be >= 0.");
    }
  }
  void setVvsIterMax(int nb) { vvsIterMax = nb; }

  void setRansacNbInliersToReachConsensus(const unsigned int &nbC) { ransacNbInlierConsensus = nbC; }
  void setRansacThreshold(const double &t)
  {
    // Test whether or not t is > 0
    if (t > std::numeric_limits<double>::epsilon()) {
      ransacThreshold = t;
    } else {
      throw vpException(vpException::badValue, "The Ransac threshold must be positive as we deal with distance.");
    }
  }
  void setRansacMaxTrials(const int &rM) { ransacMaxTrials = rM; }
  unsigned int getRansacNbInliers() const { return (unsigned int)ransacInliers.size(); }
  std::vector<unsigned int> getRansacInlierIndex() const { return ransacInlierIndex; }
  std::vector<vpPoint> getRansacInliers() const { return ransacInliers; }

  /*!
    Set if the covariance matrix has to be computed in the Virtual Visual
    Servoing approach.

    \param flag : True if the covariance has to be computed, false otherwise.
  */
  void setCovarianceComputation(const bool &flag) { computeCovariance = flag; }

  /*!
    Get the covariance matrix computed in the Virtual Visual Servoing
    approach.

    \warning The compute covariance flag has to be true if you want to compute
    the covariance matrix.

    \sa setCovarianceComputation
  */
  vpMatrix getCovarianceMatrix() const
  {
    if (!computeCovariance)
      vpTRACE("Warning : The covariance matrix has not been computed. See "
              "setCovarianceComputation() to do it.");

    return covarianceMatrix;
  }

  /*!
    Set RANSAC filter flag.

    \param flag : RANSAC flag to use to prefilter or perform degenerate configuration check.
    \sa RANSAC_FILTER_FLAGS
    \warning Prefilter degenerate points consists to not add subsequent degenerate points. This means that
    it is possible to discard a valid point and keep an invalid point if the invalid point
    is added first. It is faster to prefilter for duplicate points instead of checking for degenerate
    configuration at each time.
    \note By default the flag is set to NO_FILTER.
  */
  inline void setRansacFilterFlag(const RANSAC_FILTER_FLAGS &flag) { ransacFlag = flag; }

  /*!
    Get the number of threads for the parallel RANSAC implementation.

    \sa setNbParallelRansacThreads
  */
  inline int getNbParallelRansacThreads() const { return nbParallelRansacThreads; }

  /*!
    Set the number of threads for the parallel RANSAC implementation.

    \note You have to enable the parallel version with setUseParallelRansac().
    If the number of threads is 0, the number of threads to use is
    automatically determined with C++11.
    \sa setUseParallelRansac
  */
  inline void setNbParallelRansacThreads(int nb) { nbParallelRansacThreads = nb; }

  /*!
    \return True if the parallel RANSAC version should be used (depends also to C++11 availability).

    \sa setUseParallelRansac
  */
  inline bool getUseParallelRansac() const { return useParallelRansac; }

  /*!
    Set if parallel RANSAC version should be used or not (only if C++11).

    \note Need C++11 or higher.
  */
  inline void setUseParallelRansac(bool use) { useParallelRansac = use; }

  /*!
    Get the vector of points.

    \return The vector of points.
  */
  std::vector<vpPoint> getPoints() const
  {
    std::vector<vpPoint> vectorOfPoints(listP.begin(), listP.end());
    return vectorOfPoints;
  }

  static bool computePlanarObjectPoseFromRGBD(const vpImage<float> &depthMap, const std::vector<vpImagePoint> &corners,
                                              const vpCameraParameters &colorIntrinsics,
                                              const std::vector<vpPoint> &point3d, vpHomogeneousMatrix &cMo,
                                              double *confidence_index = NULL);

  static bool computePlanarObjectPoseFromRGBD(const vpImage<float> &depthMap,
                                              const std::vector<std::vector<vpImagePoint> > &corners,
                                              const vpCameraParameters &colorIntrinsics,
                                              const std::vector<std::vector<vpPoint> > &point3d,
                                              vpHomogeneousMatrix &cMo, double *confidence_index = NULL,
                                              bool coplanar_points = true);
  static int computeRansacIterations(double probability, double epsilon, const int sampleSize = 4,
                                     int maxIterations = 2000);

  static void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                      vpColor col = vpColor::none);
  static void display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                      vpColor col = vpColor::none);

  static void findMatch(std::vector<vpPoint> &p2D, std::vector<vpPoint> &p3D,
                        const unsigned int &numberOfInlierToReachAConsensus, const double &threshold,
                        unsigned int &ninliers, std::vector<vpPoint> &listInliers, vpHomogeneousMatrix &cMo,
                        const int &maxNbTrials = 10000, bool useParallelRansac = true, unsigned int nthreads = 0,
                        bool (*func)(const vpHomogeneousMatrix &) = NULL);

  static double poseFromRectangle(vpPoint &p1, vpPoint &p2, vpPoint &p3, vpPoint &p4, double lx,
                                  vpCameraParameters &cam, vpHomogeneousMatrix &cMo);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) &&                                                                     \
    (!defined(_MSC_VER) || ((VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && (_MSC_VER >= 1911)))

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
    if (cMo_init && !enable_vvs) {
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

    std::vector<vpPoint> P{}, Q{};
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

  static std::optional<vpHomogeneousMatrix> poseVirtualVSWithDepth(const std::vector<vpPoint> &points,
                                                                   const vpHomogeneousMatrix &cMo);
#endif

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  vp_deprecated void init();
  //@}
#endif
};

#endif
