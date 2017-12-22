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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \file vpPose.h
  \brief Tools for pose computation (pose from point only).

  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)
  \date   April, 6 1999 (first issue)
*/

#ifndef vpPOSE_HH
#define vpPOSE_HH

#include <visp3/core/vpHomogeneousMatrix.h>
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

/*!
  \class vpPose
  \ingroup group_vision_pose
  \brief Class used for pose computation from N points (pose from point only).
  Some of the algorithms implemented in this class are described in
  \cite Marchand16a.

  \note It is also possible to estimate a pose from other features using
  vpPoseFeatures class.

  To see how to use this class you can follow the \ref
  tutorial-pose-estimation.
*/

class VISP_EXPORT vpPose
{
public:
  //! Methods that could be used to estimate the pose from points.
  typedef enum {
    LAGRANGE,             /*!< Linear Lagrange approach (does't need an initialization) */
    DEMENTHON,            /*!< Linear Dementhon aproach (does't need an initialization)
                           */
    LOWE,                 /*!< Lowe aproach based on a Levenberg Marquartd non linear
                             minimization scheme that needs an initialization from Lagrange or
                             Dementhon aproach */
    RANSAC,               /*!< Robust Ransac aproach (does't need an initialization) */
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

  enum FILTERING_RANSAC_FLAGS {
    PREFILTER_DUPLICATE_POINTS = 0x1,        /*!< Remove duplicate points before the RANSAC. */
    PREFILTER_ALMOST_DUPLICATE_POINTS = 0x2, /*!< Remove almost duplicate
                                                points (up to a tolerance)
                                                before the RANSAC. */
    PREFILTER_DEGENERATE_POINTS = 0x4,       /*!< Remove degenerate points (same 3D
                                                or 2D coordinates) before the
                                                RANSAC. */
    CHECK_DEGENERATE_POINTS = 0x8            /*!< Check for degenerate points during the RANSAC. */
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
  //! RANSAC flags to remove or not degenerate points
  int ransacFlags;
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
    RansacFunctor(const vpHomogeneousMatrix &cMo_, const unsigned int ransacNbInlierConsensus_,
                  const int ransacMaxTrials_, const double ransacThreshold_, const unsigned int initial_seed_,
                  const bool checkDegeneratePoints_, const std::vector<vpPoint> &listOfUniquePoints_,
                  bool (*func_)(vpHomogeneousMatrix *))
      : m_best_consensus(), m_checkDegeneratePoints(checkDegeneratePoints_), m_cMo(cMo_), m_foundSolution(false),
        m_func(func_), m_initial_seed(initial_seed_), m_listOfUniquePoints(listOfUniquePoints_), m_nbInliers(0),
        m_ransacMaxTrials(ransacMaxTrials_), m_ransacNbInlierConsensus(ransacNbInlierConsensus_),
        m_ransacThreshold(ransacThreshold_)
    {
    }

    RansacFunctor()
      : m_best_consensus(), m_checkDegeneratePoints(false), m_cMo(), m_foundSolution(false), m_func(NULL),
        m_initial_seed(0), m_listOfUniquePoints(), m_nbInliers(0), m_ransacMaxTrials(), m_ransacNbInlierConsensus(),
        m_ransacThreshold()
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
    bool (*m_func)(vpHomogeneousMatrix *);
    unsigned int m_initial_seed;
    std::vector<vpPoint> m_listOfUniquePoints;
    unsigned int m_nbInliers;
    int m_ransacMaxTrials;
    unsigned int m_ransacNbInlierConsensus;
    double m_ransacThreshold;

    bool poseRansacImpl();
  };

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
  static vpThread::Return poseRansacImplThread(vpThread::Args arg);
#endif

protected:
  double computeResidualDementhon(const vpHomogeneousMatrix &cMo);

  // method used in poseDementhonPlan()
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo);

public:
  vpPose();
  virtual ~vpPose();
  void addPoint(const vpPoint &P);
  void addPoints(const std::vector<vpPoint> &lP);
  void clearPoint();

  bool computePose(vpPoseMethodType method, vpHomogeneousMatrix &cMo, bool (*func)(vpHomogeneousMatrix *) = NULL);
  double computeResidual(const vpHomogeneousMatrix &cMo) const;
  bool coplanar(int &coplanar_plane_type);
  void displayModel(vpImage<unsigned char> &I, vpCameraParameters &cam, vpColor col = vpColor::none);
  void displayModel(vpImage<vpRGBa> &I, vpCameraParameters &cam, vpColor col = vpColor::none);
  void init();
  void poseDementhonPlan(vpHomogeneousMatrix &cMo);
  void poseDementhonNonPlan(vpHomogeneousMatrix &cMo);
  void poseLagrangePlan(vpHomogeneousMatrix &cMo, const int coplanar_plane_type = 0);
  void poseLagrangeNonPlan(vpHomogeneousMatrix &cMo);
  void poseLowe(vpHomogeneousMatrix &cMo);
  bool poseRansac(vpHomogeneousMatrix &cMo, bool (*func)(vpHomogeneousMatrix *) = NULL);
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
    Set RANSAC filtering flags.

    \param flags : Flags to use, e.g. \e
    setRansacFilterFlags(PREFILTER_DUPLICATE_POINTS +
    CHECK_DEGENERATE_POINTS). \sa FILTERING_RANSAC_FLAGS
  */
  inline void setRansacFilterFlags(const int flags) { ransacFlags = flags; }

  /*!
    Get the number of threads for the parallel RANSAC implementation.

    \sa setNbParallelRansacThreads
  */
  inline int getNbParallelRansacThreads() const { return nbParallelRansacThreads; }

  /*!
    Set the number of threads for the parallel RANSAC implementation.

    \note You have to enable the parallel version with setUseParallelRansac().
    If the number of threads is 0, the number of threads to use is
    automatically determined with OpenMP. \sa setUseParallelRansac
  */
  inline void setNbParallelRansacThreads(const int nb) { nbParallelRansacThreads = nb; }

  /*!
    \return True if the parallel RANSAC version should be used.

    \sa setUseParallelRansac
  */
  inline bool getUseParallelRansac() const { return useParallelRansac; }

  /*!
    Set if parallel RANSAC version should be used or not.

    \note Need Pthread.
  */
  inline void setUseParallelRansac(const bool use) { useParallelRansac = use; }

  /*!
    Get the vector of points.

    \return The vector of points.
  */
  std::vector<vpPoint> getPoints() const
  {
    std::vector<vpPoint> vectorOfPoints(listP.begin(), listP.end());
    return vectorOfPoints;
  }

  static void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                      vpColor col = vpColor::none);
  static void display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                      vpColor col = vpColor::none);
  static double poseFromRectangle(vpPoint &p1, vpPoint &p2, vpPoint &p3, vpPoint &p4, double lx,
                                  vpCameraParameters &cam, vpHomogeneousMatrix &cMo);

  static int computeRansacIterations(double probability, double epsilon, const int sampleSize = 4,
                                     int maxIterations = 2000);

  static void findMatch(std::vector<vpPoint> &p2D, std::vector<vpPoint> &p3D,
                        const unsigned int &numberOfInlierToReachAConsensus, const double &threshold,
                        unsigned int &ninliers, std::vector<vpPoint> &listInliers, vpHomogeneousMatrix &cMo,
                        const int &maxNbTrials = 10000);
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
