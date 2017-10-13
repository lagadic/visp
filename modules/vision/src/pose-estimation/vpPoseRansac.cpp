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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 * Aurelien Yol
 * Souriya Trinh
 *
 *****************************************************************************/


/*!
  \file vpPoseRansac.cpp
  \brief function used to estimate a pose using the Ransac algorithm
*/

#include <iostream>
#include <cmath>        // std::fabs
#include <limits>       // numeric_limits
#include <stdlib.h>
#include <algorithm>    // std::count
#include <float.h>      // DBL_MAX
#include <map>

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRansac.h>
#include <visp3/vision/vpPoseException.h>
#include <visp3/core/vpMath.h>

#if defined (VISP_HAVE_CPP11_COMPATIBILITY)
#  include <unordered_map>
#endif

#if defined(VISP_HAVE_OPENMP)
#  include <omp.h>
#endif

#define eps 1e-6


namespace {
//For std::map<vpPoint>
struct ComparePointDuplicate {
  bool operator()( const vpPoint &point1, const vpPoint &point2 ) const {
    if (point1.oP[0] < point2.oP[0]) return true;
    if (point1.oP[0] > point2.oP[0]) return false;

    if (point1.oP[1] < point2.oP[1]) return true;
    if (point1.oP[1] > point2.oP[1]) return false;

    if (point1.oP[2] < point2.oP[2]) return true;
    if (point1.oP[2] > point2.oP[2]) return false;

    if (point1.p[0] < point2.p[0]) return true;
    if (point1.p[0] > point2.p[0]) return false;

    if (point1.p[1] < point2.p[1]) return true;
    if (point1.p[1] > point2.p[1]) return false;

    return false;
  }
};

//For std::map<vpPoint>
struct ComparePointAlmostDuplicate {
  bool operator()( const vpPoint &point1, const vpPoint &point2 ) const {
    if (point1.oP[0] - point2.oP[0] < -eps) return true;
    if (point1.oP[0] - point2.oP[0] > eps) return false;

    if (point1.oP[1] - point2.oP[1] < -eps) return true;
    if (point1.oP[1] - point2.oP[1] > eps) return false;

    if (point1.oP[2] - point2.oP[2] < -eps) return true;
    if (point1.oP[2] - point2.oP[2] > eps) return false;

    if (point1.p[0] - point2.p[0] < -eps) return true;
    if (point1.p[0] - point2.p[0] > eps) return false;

    if (point1.p[1] - point2.p[1] < -eps) return true;
    if (point1.p[1] - point2.p[1] > eps) return false;

    return false;
  }
};

//For std::map<vpPoint>
struct CompareObjectPointDegenerate {
  bool operator()( const vpPoint &point1, const vpPoint &point2 ) const {
    if (point1.oP[0] - point2.oP[0] < -eps) return true;
    if (point1.oP[0] - point2.oP[0] > eps) return false;

    if (point1.oP[1] - point2.oP[1] < -eps) return true;
    if (point1.oP[1] - point2.oP[1] > eps) return false;

    if (point1.oP[2] - point2.oP[2] < -eps) return true;
    if (point1.oP[2] - point2.oP[2] > eps) return false;

    return false;
  }
};

//For std::map<vpPoint>
struct CompareImagePointDegenerate {
  bool operator()( const vpPoint &point1, const vpPoint &point2 ) const {
    if (point1.p[0] - point2.p[0] < -eps) return true;
    if (point1.p[0] - point2.p[0] > eps) return false;

    if (point1.p[1] - point2.p[1] < -eps) return true;
    if (point1.p[1] - point2.p[1] > eps) return false;

    return false;
  }
};

//std::find_if
struct FindDegeneratePoint {
  explicit FindDegeneratePoint( const vpPoint &pt ) : m_pt(pt) { }

  bool operator() (const vpPoint &pt) {
    return ( (std::fabs(m_pt.oP[0] - pt.oP[0]) < eps &&
        std::fabs(m_pt.oP[1] - pt.oP[1]) < eps &&
        std::fabs(m_pt.oP[2] - pt.oP[2]) < eps) ||
        (std::fabs(m_pt.p[0] - pt.p[0]) < eps &&
        std::fabs(m_pt.p[1] - pt.p[1]) < eps) );
  }

  vpPoint m_pt;
};

#if defined (VISP_HAVE_CPP11_COMPATIBILITY)
//For unordered_map<vpPoint>
struct HashDuplicate {
  std::size_t operator()(const vpPoint &point) const {
    using std::size_t;
    using std::hash;

    size_t res = 17;
    res = res * 31 + hash<double>()( point.oP[0] );
    res = res * 31 + hash<double>()( point.oP[1] );
    res = res * 31 + hash<double>()( point.oP[2] );
    res = res * 31 + hash<double>()( point.p[0] );
    res = res * 31 + hash<double>()( point.p[1] );

    return res;
  }
};

//For unordered_map<vpPoint>
struct ComparePointDuplicateUnorderedMap {
  bool operator()( const vpPoint &point1, const vpPoint &point2 ) const {
    return (
          std::fabs(point1.oP[0] - point2.oP[0]) < std::numeric_limits<double>::epsilon() &&
        std::fabs(point1.oP[1] - point2.oP[1]) < std::numeric_limits<double>::epsilon() &&
        std::fabs(point1.oP[2] - point2.oP[2]) < std::numeric_limits<double>::epsilon() &&
        std::fabs(point1.p[0] - point2.p[0]) < std::numeric_limits<double>::epsilon() &&
        std::fabs(point1.p[1] - point2.p[1]) < std::numeric_limits<double>::epsilon()
        );
  }
};
#endif
}

bool vpPose::RansacFunctor::poseRansacImpl() {
  unsigned int size = (unsigned int) m_listOfUniquePoints.size();
  int nbTrials = 0;
  unsigned int nbMinRandom = 4;

#if defined(_WIN32) && (defined(_MSC_VER) || defined(__MINGW32__))
  srand(m_initial_seed);
#endif

  vpPoint p; //Point used to project using the estimated pose

  bool foundSolution = false;
  while (nbTrials < m_ransacMaxTrials && m_nbInliers < (unsigned int) m_ransacNbInlierConsensus)
  {
    //Hold the list of the index of the inliers (points in the consensus set)
    std::vector<unsigned int> cur_consensus;
    //Hold the list of the index of the outliers
    std::vector<unsigned int> cur_outliers;
    //Hold the list of the index of the points randomly picked
    std::vector<unsigned int> cur_randoms;
    //Hold the list of the current inliers points to avoid to add a degenerate point if the flag is set
    std::vector<vpPoint> cur_inliers;

    vpHomogeneousMatrix cMo_lagrange, cMo_dementhon;
    //Use a temporary variable because if not, the cMo passed in parameters will be modified when
    // we compute the pose for the minimal sample sets but if the pose is not correct when we pass
    // a function pointer we do not want to modify the cMo passed in parameters
    vpHomogeneousMatrix cMo_tmp;

    //Vector of used points, initialized at false for all points
    std::vector<bool> usedPt(size, false);

    vpPose poseMin;
    for(unsigned int i = 0; i < nbMinRandom;)
    {
      if((size_t) std::count(usedPt.begin(), usedPt.end(), true) == usedPt.size()) {
        //All points was picked once, break otherwise we stay in an infinite loop
        break;
      }

      //Pick a point randomly
#if defined(_WIN32) && (defined(_MSC_VER) || defined(__MINGW32__))
      unsigned int r_ = (unsigned int) rand() % size;
#else
      unsigned int r_ = (unsigned int) rand_r(&m_initial_seed) % size;
#endif

      while(usedPt[r_]) {
        //If already picked, pick another point randomly
#if defined(_WIN32) && (defined(_MSC_VER) || defined(__MINGW32__))
        r_ = (unsigned int) rand() % size;
#else
        r_ = (unsigned int) rand_r(&m_initial_seed) % size;
#endif
      }
      //Mark this point as already picked
      usedPt[r_] = true;
      vpPoint pt = m_listOfUniquePoints[r_];

      bool degenerate = false;
      if (m_checkDegeneratePoints) {
        if ( std::find_if(poseMin.listOfPoints.begin(), poseMin.listOfPoints.end(), FindDegeneratePoint(pt)) !=  poseMin.listOfPoints.end()) {
          degenerate = true;
        }
      }

      if (!degenerate) {
        poseMin.addPoint(pt);
        cur_randoms.push_back(r_);
        //Increment the number of points picked
        i++;
      }
    }

    if(poseMin.npt < nbMinRandom) {
      nbTrials++;
      continue;
    }

    //Flags set if pose computation is OK
    bool is_valid_lagrange = false;
    bool is_valid_dementhon = false;

    //Set maximum value for residuals
    double r_lagrange = DBL_MAX;
    double r_dementhon = DBL_MAX;

    try {
      poseMin.computePose(vpPose::LAGRANGE, cMo_lagrange);
      r_lagrange = poseMin.computeResidual(cMo_lagrange);
      is_valid_lagrange = true;
    } catch(...) { }

    try {
      poseMin.computePose(vpPose::DEMENTHON, cMo_dementhon);
      r_dementhon = poseMin.computeResidual(cMo_dementhon);
      is_valid_dementhon = true;
    } catch(...) { }

    //If residual returned is not a number (NAN), set valid to false
    if(vpMath::isNaN(r_lagrange)) {
      is_valid_lagrange = false;
      r_lagrange = DBL_MAX;
    }

    if(vpMath::isNaN(r_dementhon)) {
      is_valid_dementhon = false;
      r_dementhon = DBL_MAX;
    }

    //If at least one pose computation is OK,
    //we can continue, otherwise pick another random set
    if(is_valid_lagrange || is_valid_dementhon) {
      double r;
      if (r_lagrange < r_dementhon) {
        r = r_lagrange;
        cMo_tmp = cMo_lagrange;
      }
      else {
        r = r_dementhon;
        cMo_tmp = cMo_dementhon;
      }
      r = sqrt(r) / (double) nbMinRandom;

      //Filter the pose using some criterion (orientation angles, translations, etc.)
      bool isPoseValid = true;
      if(m_func != NULL) {
        isPoseValid = m_func(&cMo_tmp);
        if(isPoseValid) {
          m_cMo = cMo_tmp;
        }
      } else {
        //No post filtering on pose, so copy cMo_temp to cMo
        m_cMo = cMo_tmp;
      }

      if (isPoseValid && r < m_ransacThreshold)
      {
        unsigned int nbInliersCur = 0;
        unsigned int iter = 0;
        for (std::vector<vpPoint>::const_iterator it = m_listOfUniquePoints.begin(); it != m_listOfUniquePoints.end(); ++it, iter++)
        {
          p.setWorldCoordinates(it->get_oX(), it->get_oY(), it->get_oZ());
          p.track(m_cMo);

          double d = vpMath::sqr(p.get_x() - it->get_x()) + vpMath::sqr(p.get_y() - it->get_y());
          double error = sqrt(d);
          if(error < m_ransacThreshold) {
            bool degenerate = false;
            if (m_checkDegeneratePoints) {
              if ( std::find_if(cur_inliers.begin(), cur_inliers.end(), FindDegeneratePoint(*it)) != cur_inliers.end() ) {
                degenerate = true;
              }
            }

            if (!degenerate) {
              // the point is considered as inlier if the error is below the threshold
              nbInliersCur++;
              cur_consensus.push_back(iter);
              cur_inliers.push_back(*it);
            } else {
              cur_outliers.push_back(iter);
            }
          }
          else {
            cur_outliers.push_back(iter);
          }
        }

        if(nbInliersCur > m_nbInliers)
        {
          foundSolution = true;
          m_best_consensus = cur_consensus;
          m_nbInliers = nbInliersCur;
        }

        nbTrials++;

        if(nbTrials >= m_ransacMaxTrials) {
          foundSolution = true;
        }
      }
      else {
        nbTrials++;
      }
    } else {
      nbTrials++;
    }
  }

  return foundSolution;
}

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
vpThread::Return vpPose::poseRansacImplThread(vpThread::Args arg) {
  vpPose::RansacFunctor* f = reinterpret_cast<vpPose::RansacFunctor*>(arg);
  (*f)();
  return 0;
}
#endif

/*!
  Compute the pose using the Ransac approach.

  \param cMo : Computed pose
  \param func : Pointer to a function that takes in parameter a vpHomogeneousMatrix
  and returns true if the pose check is OK or false otherwise
  \return True if we found at least 4 points with a reprojection error below ransacThreshold.
*/
bool vpPose::poseRansac(vpHomogeneousMatrix & cMo, bool (*func)(vpHomogeneousMatrix *))
{
  //Check only for adding / removing problem
  //Do not take into account problem with element modification here
  if (listP.size() != listOfPoints.size()) {
    std::cerr << "You should not modify vpPose::listP!" << std::endl;
    listOfPoints = std::vector<vpPoint>(listP.begin(), listP.end());
  }

  ransacInliers.clear();
  ransacInlierIndex.clear();

  std::vector<unsigned int> best_consensus;
  unsigned int nbInliers = 0;

  vpHomogeneousMatrix cMo_lagrange, cMo_dementhon;

  if (listOfPoints.size() < 4) {
    //vpERROR_TRACE("Not enough point to compute the pose");
    throw(vpPoseException(vpPoseException::notInitializedError,
                          "Not enough point to compute the pose")) ;
  }

  std::vector<vpPoint> listOfUniquePoints;
  std::map<size_t, size_t> mapOfUniquePointIndex;

  //Get RANSAC flags
  bool prefilterDuplicatePoints = (ransacFlags & PREFILTER_DUPLICATE_POINTS) != 0;
  bool prefilterAlmostDuplicatePoints = (ransacFlags & PREFILTER_ALMOST_DUPLICATE_POINTS) != 0;
  bool prefilterDegeneratePoints = (ransacFlags & PREFILTER_DEGENERATE_POINTS) != 0;
  bool checkDegeneratePoints = (ransacFlags & CHECK_DEGENERATE_POINTS) != 0;

  if (prefilterDuplicatePoints || prefilterAlmostDuplicatePoints || prefilterDegeneratePoints) {
    //Prefiltering
    if (prefilterDuplicatePoints) {
#if defined (VISP_HAVE_CPP11_COMPATIBILITY)
      std::unordered_map<vpPoint, size_t, HashDuplicate, ComparePointDuplicateUnorderedMap> filterMap;
      size_t index_pt = 0;
      for (std::vector<vpPoint>::const_iterator it_pt = listOfPoints.begin(); it_pt != listOfPoints.end(); ++it_pt, index_pt++) {
        if (filterMap.find(*it_pt) == filterMap.end()) {
          filterMap[*it_pt] = index_pt;

          listOfUniquePoints.push_back(*it_pt);
          mapOfUniquePointIndex[listOfUniquePoints.size()-1] = index_pt;
        }
      }
#else
      std::map<vpPoint, size_t, ComparePointDuplicate> filterMap;
      size_t index_pt = 0;
      for (std::vector<vpPoint>::const_iterator it_pt = listOfPoints.begin(); it_pt != listOfPoints.end(); ++it_pt, index_pt++) {
        if (filterMap.find(*it_pt) == filterMap.end()) {
          filterMap[*it_pt] = index_pt;

          listOfUniquePoints.push_back(*it_pt);
          mapOfUniquePointIndex[listOfUniquePoints.size()-1] = index_pt;
        }
      }
#endif
    } else if (prefilterAlmostDuplicatePoints) {
      std::map<vpPoint, size_t, ComparePointAlmostDuplicate> filterMap;
      size_t index_pt = 0;
      for (std::vector<vpPoint>::const_iterator it_pt = listOfPoints.begin(); it_pt != listOfPoints.end(); ++it_pt, index_pt++) {
        if (filterMap.find(*it_pt) == filterMap.end()) {
          filterMap[*it_pt] = index_pt;

          listOfUniquePoints.push_back(*it_pt);
          mapOfUniquePointIndex[listOfUniquePoints.size()-1] = index_pt;
        }
      }
    } else {
      //Remove other degenerate object points
      std::map<vpPoint, size_t, CompareObjectPointDegenerate> filterObjectPointMap;
      size_t index_pt = 0;
      for (std::vector<vpPoint>::const_iterator it_pt = listOfPoints.begin(); it_pt != listOfPoints.end(); ++it_pt, index_pt++) {
        if (filterObjectPointMap.find(*it_pt) == filterObjectPointMap.end()) {
          filterObjectPointMap[*it_pt] = index_pt;
        }
      }

      std::map<vpPoint, size_t, CompareImagePointDegenerate> filterImagePointMap;
      for (std::map<vpPoint, size_t>::const_iterator it = filterObjectPointMap.begin(); it != filterObjectPointMap.end(); ++it) {
        if (filterImagePointMap.find(it->first) == filterImagePointMap.end()) {
          filterImagePointMap[it->first] = it->second;

          listOfUniquePoints.push_back(it->first);
          mapOfUniquePointIndex[listOfUniquePoints.size()-1] = it->second;
        }
      }
    }
  } else {
    //No prefiltering
    listOfUniquePoints = listOfPoints;

    size_t index_pt = 0;
    for (std::vector<vpPoint>::const_iterator it_pt = listOfPoints.begin(); it_pt != listOfPoints.end(); ++it_pt, index_pt++) {
      mapOfUniquePointIndex[index_pt] = index_pt;
    }
  }

  unsigned int size = (unsigned int) listOfUniquePoints.size();
  if (size < 4) {
    throw(vpPoseException(vpPoseException::notInitializedError, "Not enough point to compute the pose")) ;
  }


  bool executeParallelVersion = useParallelRansac;

#if defined (VISP_HAVE_PTHREAD) || (defined (_WIN32) && !defined(WINRT_8_0))
#  define VP_THREAD_OK
  int nbThreads = 1;
#endif

  if (executeParallelVersion) {
#if !defined (VP_THREAD_OK) && !defined (VISP_HAVE_OPENMP)
    executeParallelVersion = false;
    std::cerr << "Pthread or WIN32 API or OpenMP is needed to use the parallel RANSAC version." << std::endl;
#elif !defined (VP_THREAD_OK)
    //Use OpenMP
#define PARALLEL_RANSAC_OPEN_MP
#elif !defined (VISP_HAVE_OPENMP)
    if(nbParallelRansacThreads <= 0) {
      //Cannot get the number of CPU threads so use the sequential mode
      executeParallelVersion = false;
      std::cerr << "OpenMP is needed to get the number of CPU threads so use the sequential mode instead." << std::endl;
    } else {
      nbThreads = nbParallelRansacThreads;
      if (nbThreads == 1) {
        executeParallelVersion = false;
      }
    }
#elif defined (VP_THREAD_OK) && defined (VISP_HAVE_OPENMP)
    if (nbParallelRansacThreads <= 0) {
      //Use OpenMP to get the number of CPU threads
      nbThreads = omp_get_max_threads();
      if(nbThreads <= 1) {
        nbThreads = 1;
        executeParallelVersion = false;
      }
    }
#endif
  }

  bool foundSolution = false;

  if(executeParallelVersion) {
#if defined (PARALLEL_RANSAC_OPEN_MP)
    //List of points picked randomly (minimal sample set, MSS)
    //std::vector<unsigned int> best_randoms; // never used

    //Section of code run in parallel
    //All variables declared before the parallel keyword are shared between the team of threads (if private keyword is not used)
    //Code in parallel section are duplicated between the team of threads
#pragma omp parallel
    {
#if defined(VISP_HAVE_OPENMP)
      //Set different seeds for each thread in the team, rand() is not thread safe
      unsigned int initial_seed = (unsigned int) omp_get_thread_num(); //same seed each time
      //(unsigned int) (int(time(NULL)) ^ omp_get_thread_num());
      if(omp_get_num_threads() == 1) {
        initial_seed = 0; //Same seed at each run
      }
#else
      unsigned int initial_seed = 0;
#endif

#if defined(_WIN32)  && (defined(_MSC_VER) || defined(__MINGW32__))
      srand(initial_seed);
#endif

      unsigned int nbMinRandom = 4;

      //Numbers of points in the best consensus set
      unsigned int nb_best_inliers = 0;

      //True if we found a consensus set with a size > ransacNbInlierConsensus
      bool foundSolutionWithConsensus = false;

      vpPoint p; //Point used to project using the estimated pose

#pragma omp for
      for(int nbTrials = 0; nbTrials < ransacMaxTrials; nbTrials++) {
        //Flag to check if a solution has been founded, used to "cancel" the threads
        if(!foundSolutionWithConsensus) {
          //Hold the list of the index of the inliers (points in the consensus set)
          std::vector<unsigned int> cur_consensus;
          //Hold the list of the index of the outliers
          std::vector<unsigned int> cur_outliers;
          //Hold the list of the index of the points randomly picked
          std::vector<unsigned int> cur_randoms;
          //Hold the list of the current inliers points to avoid to add a degenerate point if the flag is set
          std::vector<vpPoint> cur_inliers;

          vpHomogeneousMatrix cMo_lagrange, cMo_dementhon;
          vpHomogeneousMatrix cMo_tmp;

          //Vector of used points, initialized at false for all points
          std::vector<bool> usedPt(size, false);

          vpPose poseMin;

          for(unsigned int i = 0; i < nbMinRandom;) {
            if((size_t) std::count(usedPt.begin(), usedPt.end(), true) == usedPt.size()) {
              //All points was picked once, break otherwise we stay in an infinite loop
              break;
            }

            //Pick a point randomly
#if defined(_WIN32) && (defined(_MSC_VER) || defined(__MINGW32__))
            unsigned int r_ = (unsigned int) rand() % size;
#else
            unsigned int r_ = (unsigned int) rand_r(&initial_seed) % size;
#endif

            while(usedPt[r_]) {
              //If already picked, pick another point randomly
#if defined(_WIN32) && (defined(_MSC_VER) || defined(__MINGW32__))
              r_ = (unsigned int) rand() % size;
#else
              r_ = (unsigned int) rand_r(&initial_seed) % size;
#endif
            }
            //Mark this point as already picked
            usedPt[r_] = true;
            vpPoint pt = listOfUniquePoints[r_];

            bool degenerate = false;
            if (checkDegeneratePoints) {
              if ( std::find_if(poseMin.listOfPoints.begin(), poseMin.listOfPoints.end(), FindDegeneratePoint(pt)) !=  poseMin.listOfPoints.end()) {
                degenerate = true;
              }
            }

            if (!degenerate) {
              poseMin.addPoint(pt);
              cur_randoms.push_back(r_);
              //Increment the number of points picked
              i++;
            }
          }

          if(poseMin.npt >= nbMinRandom) {
            //Flags set if pose computation is OK
            bool is_valid_lagrange = false;
            bool is_valid_dementhon = false;

            //Set maximum value for residuals
            double r = DBL_MAX;
            double r_lagrange = DBL_MAX;
            double r_dementhon = DBL_MAX;

            try {
              poseMin.computePose(vpPose::LAGRANGE, cMo_lagrange);
              r_lagrange = poseMin.computeResidual(cMo_lagrange);
              is_valid_lagrange = true;
            } catch(...) { }

            try {
              poseMin.computePose(vpPose::DEMENTHON, cMo_dementhon);
              r_dementhon = poseMin.computeResidual(cMo_dementhon);
              is_valid_dementhon = true;
            } catch(...) { }

            //If residual returned is not a number (NAN), set valid to false
            if(vpMath::isNaN(r_lagrange)) {
              is_valid_lagrange = false;
              r_lagrange = DBL_MAX;
            }

            if(vpMath::isNaN(r_dementhon)) {
              is_valid_dementhon = false;
              r_dementhon = DBL_MAX;
            }

            //If at least one pose computation is OK,
            //we can continue, otherwise pick another random set
            if(is_valid_lagrange || is_valid_dementhon) {
              if (r_lagrange < r_dementhon) {
                r = r_lagrange;
                cMo_tmp = cMo_lagrange;
              }
              else {
                r = r_dementhon;
                cMo_tmp = cMo_dementhon;
              }
              r = sqrt(r) / (double) nbMinRandom;

              //Filter the pose using some criterion (orientation angles, translations, etc.)
              bool isPoseValid = true;
              if(func != NULL) {
                isPoseValid = func(&cMo_tmp);
              }

              if (isPoseValid && r < ransacThreshold) {
                unsigned int nbInliersCur = 0;
                unsigned int iter = 0;
                for (std::vector<vpPoint>::const_iterator it = listOfUniquePoints.begin(); it != listOfUniquePoints.end(); ++it, iter++) {
                  p.setWorldCoordinates(it->get_oX(), it->get_oY(), it->get_oZ());
                  p.track(cMo_tmp);

                  double d = vpMath::sqr(p.get_x() - it->get_x()) + vpMath::sqr(p.get_y() - it->get_y()) ;
                  double error = sqrt(d) ;
                  if(error < ransacThreshold) {
                    bool degenerate = false;
                    if (checkDegeneratePoints) {
                      if ( std::find_if(cur_inliers.begin(), cur_inliers.end(), FindDegeneratePoint(*it)) != cur_inliers.end() ) {
                        degenerate = true;
                      }
                    }

                    if (!degenerate) {
                      // the point is considered as inlier if the error is below the threshold
                      nbInliersCur++;
                      cur_consensus.push_back(iter);
                      cur_inliers.push_back(*it);
                    } else {
                      cur_outliers.push_back(iter);
                    }
                  } else {
                    cur_outliers.push_back(iter);
                  }
                }

#pragma omp critical (update_best_consensus_set)
                {
                  if(nbInliersCur > nb_best_inliers) {
                    foundSolution = true;
                    best_consensus = cur_consensus;
                    //best_randoms = cur_randoms; // never used
                    nb_best_inliers = nbInliersCur;

                    if(nbInliersCur >= ransacNbInlierConsensus) {
                      foundSolutionWithConsensus = true;
                    }
                  }
                }
              }
            }
          }
        }
      }

      nbInliers = best_consensus.size();
    }
#elif defined(VP_THREAD_OK)
    std::vector<vpThread *> threads((size_t) nbThreads);
    std::vector<RansacFunctor> ransac_func((size_t) nbThreads);

    int splitTrials = ransacMaxTrials / nbThreads;
    for(size_t i = 0; i < (size_t) nbThreads; i++) {
      unsigned int initial_seed = (unsigned int) i; //((unsigned int) time(NULL) ^ i);
      if(i < (size_t) nbThreads-1) {
        ransac_func[i] = RansacFunctor(cMo, ransacNbInlierConsensus, splitTrials, ransacThreshold,
                                       initial_seed, checkDegeneratePoints, listOfUniquePoints, func);
      } else {
        int maxTrialsRemainder = ransacMaxTrials - splitTrials * (nbThreads-1);
        ransac_func[i] = RansacFunctor(cMo, ransacNbInlierConsensus, maxTrialsRemainder, ransacThreshold,
                                       initial_seed, checkDegeneratePoints, listOfUniquePoints, func);
      }

      threads[(size_t) i] = new vpThread((vpThread::Fn) poseRansacImplThread, (vpThread::Args) &ransac_func[ i]);
    }

    //Get the best pose between the threads
    vpPose final_pose;
    for(std::vector<vpPoint>::const_iterator it = listOfPoints.begin(); it != listOfPoints.end(); ++it) {
      final_pose.addPoint(*it);
    }

    for(size_t i = 0; i < (size_t) nbThreads; i++) {
      threads[i]->join();
      delete threads[i];
    }

    bool successRansac = false;
    size_t best_consensus_size = 0;
    for(size_t i = 0; i < (size_t) nbThreads; i++) {
      if(ransac_func[i].getResult()) {
        successRansac = true;

        if(ransac_func[i].getBestConsensus().size() > best_consensus_size) {
          nbInliers = ransac_func[i].getNbInliers();
          best_consensus = ransac_func[i].getBestConsensus();
          best_consensus_size = ransac_func[i].getBestConsensus().size();
        }
      }
    }

    foundSolution = successRansac;
#endif
  } else {
    //Sequential RANSAC
    RansacFunctor sequentialRansac(cMo, ransacNbInlierConsensus, ransacMaxTrials, ransacThreshold,
                                   0, checkDegeneratePoints, listOfUniquePoints, func);
    sequentialRansac();
    foundSolution = sequentialRansac.getResult();

    if (foundSolution) {
      nbInliers = sequentialRansac.getNbInliers();
      best_consensus = sequentialRansac.getBestConsensus();
    }
  }

  if(foundSolution) {
    unsigned int nbMinRandom = 4;
    //    std::cout << "Nombre d'inliers " << nbInliers << std::endl ;

    //Display the random picked points
    /*
    std::cout << "Randoms : ";
    for(unsigned int i = 0 ; i < cur_randoms.size() ; i++)
      std::cout << cur_randoms[i] << " ";
    std::cout << std::endl;
    */

    //Display the outliers
    /*
    std::cout << "Outliers : ";
    for(unsigned int i = 0 ; i < cur_outliers.size() ; i++)
      std::cout << cur_outliers[i] << " ";
    std::cout << std::endl;
    */

    //Even if the cardinality of the best consensus set is inferior to ransacNbInlierConsensus,
    //we want to refine the solution with data in best_consensus and return this pose.
    //This is an approach used for example in p118 in Multiple View Geometry in Computer Vision, Hartley, R.~I. and Zisserman, A.
    if(nbInliers >= nbMinRandom) //if(nbInliers >= (unsigned)ransacNbInlierConsensus)
    {
      //Refine the solution using all the points in the consensus set and with VVS pose estimation
      vpPose pose;
      for(size_t i = 0 ; i < best_consensus.size(); i++)
      {
        vpPoint pt = listOfUniquePoints[best_consensus[i]];

        pose.addPoint(pt) ;
        ransacInliers.push_back(pt);
      }

      //Update the list of inlier index
      for(std::vector<unsigned int>::const_iterator it_index = best_consensus.begin();
          it_index != best_consensus.end(); ++it_index) {
        ransacInlierIndex.push_back((unsigned int) mapOfUniquePointIndex[*it_index]);
      }

      //Flags set if pose computation is OK
      bool is_valid_lagrange = false;
      bool is_valid_dementhon = false;

      //Set maximum value for residuals
      double r_lagrange = DBL_MAX;
      double r_dementhon = DBL_MAX;

      try {
        pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
        r_lagrange = pose.computeResidual(cMo_lagrange);
        is_valid_lagrange = true;
      } catch(...) { }

      try {
        pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
        r_dementhon = pose.computeResidual(cMo_dementhon);
        is_valid_dementhon = true;
      } catch(...) { }

      //If residual returned is not a number (NAN), set valid to false
      if(vpMath::isNaN(r_lagrange)) {
        is_valid_lagrange = false;
        r_lagrange = DBL_MAX;
      }

      if(vpMath::isNaN(r_dementhon)) {
        is_valid_dementhon = false;
        r_dementhon = DBL_MAX;
      }

      if(is_valid_lagrange || is_valid_dementhon) {
        if (r_lagrange < r_dementhon) {
          cMo = cMo_lagrange;
        }
        else {
          cMo = cMo_dementhon;
        }

        pose.setCovarianceComputation(computeCovariance);
        pose.computePose(vpPose::VIRTUAL_VS, cMo);

        //In some rare cases, the final pose could not respect the pose criterion even
        //if the 4 minimal points picked respect the pose criterion.
        if(func != NULL && !func(&cMo)) {
          return false;
        }

        if(computeCovariance) {
          covarianceMatrix = pose.covarianceMatrix;
        }
      }
    } else {
      return false;
    }
  }

  return foundSolution;
}

/*!
  Compute the number of RANSAC iterations to ensure with a probability \e p
  that at least one of the random samples of \e s points is free from outliers.
  \note See: Hartley and Zisserman, Multiple View Geometry in Computer Vision, p119 (2. How many samples?).

  \param probability : Probability that at least one of the random samples is free from outliers (typically p=0.99).
  \param epsilon : Probability that a selected point is an outlier (between 0 and 1).
  \param sampleSize : Minimum number of points to estimate the model (4 for a pose estimation).
  \param maxIterations : Upper bound on the number of iterations or -1 for INT_MAX.
  \return The number of RANSAC iterations to ensure with a probability \e p
  that at least one of the random samples of \e s points is free from outliers or \p maxIterations if it exceeds
  the desired upper bound or \e INT_MAX if maxIterations=-1.
*/
int vpPose::computeRansacIterations(double probability, double epsilon, const int sampleSize, int maxIterations) {
  probability = (std::max)(probability, 0.0);
  probability = (std::min)(probability, 1.0);
  epsilon = (std::max)(epsilon, 0.0);
  epsilon = (std::min)(epsilon, 1.0);

  if (vpMath::nul(epsilon)) {
    //no outliers
    return 1;
  }

  if (maxIterations <= 0) {
    maxIterations = std::numeric_limits<int>::max();
  }

  double logarg, logval, N;
  logarg = -std::pow(1.0 - epsilon, sampleSize);
#ifdef VISP_HAVE_FUNC_LOG1P
  logval = log1p(logarg);
#else
  logval = log(1.0 + logarg);
#endif
  if (vpMath::nul(logval, std::numeric_limits<double>::epsilon())) {
    std::cerr << "vpMath::nul(log(1.0 - std::pow(1.0 - epsilon, sampleSize)), std::numeric_limits<double>::epsilon())"
              << std::endl;
    return 0;
  }

  N = log((std::max)(1.0 - probability, std::numeric_limits<double>::epsilon())) / logval;
  if (logval < 0.0 && N < maxIterations) {
    return (int) ceil(N);
  }

  return maxIterations;
}

/*!
  Match a vector p2D of  2D point (x,y)  and  a vector p3D of 3D points
  (X,Y,Z) using the Ransac algorithm.

  At least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  The inliers are given in a vector of vpPoint listInliers.

  The pose is returned in cMo.

  \param p2D : Vector of 2d points (x and y attributes are used).
  \param p3D : Vector of 3d points (oX, oY and oZ attributes are used).
  \param numberOfInlierToReachAConsensus : The minimum number of inlier to have
  to consider a trial as correct.
  \param threshold : The maximum error allowed between the 2d points and the
  reprojection of its associated 3d points by the current pose (in meter).
  \param ninliers : Number of inliers found for the best solution.
  \param listInliers : Vector of points (2d and 3d) that are inliers for the best solution.
  \param cMo : The computed pose (best solution).
  \param maxNbTrials : Maximum number of trials before considering a solution
  fitting the required \e numberOfInlierToReachAConsensus and \e threshold
  cannot be found.
*/
void vpPose::findMatch(std::vector<vpPoint> &p2D,
                       std::vector<vpPoint> &p3D,
                       const unsigned int &numberOfInlierToReachAConsensus,
                       const double &threshold,
                       unsigned int &ninliers,
                       std::vector<vpPoint> &listInliers,
                       vpHomogeneousMatrix &cMo,
                       const int &maxNbTrials )
{
  vpPose pose;

  int nbPts = 0;
  for(unsigned int i = 0 ; i < p2D.size() ; i++)
  {
    for(unsigned int j = 0 ; j < p3D.size() ; j++)
    {
      vpPoint pt(p3D[j].getWorldCoordinates());
      pt.set_x(p2D[i].get_x());
      pt.set_y(p2D[i].get_y());
      pose.addPoint(pt);
      nbPts++;
    }
  }

  if (pose.listP.size() < 4)
  {
    vpERROR_TRACE("Ransac method cannot be used in that case ") ;
    vpERROR_TRACE("(at least 4 points are required)") ;
    vpERROR_TRACE("Not enough point (%d) to compute the pose  ",pose.listP.size()) ;
    throw(vpPoseException(vpPoseException::notEnoughPointError,
                          "Not enough point (%d) to compute the pose by ransac",pose.listP.size())) ;
  }
  else
  {
    pose.setRansacMaxTrials(maxNbTrials);
    pose.setRansacNbInliersToReachConsensus(numberOfInlierToReachAConsensus);
    pose.setRansacThreshold(threshold);
    pose.computePose(vpPose::RANSAC, cMo);
    ninliers = pose.getRansacNbInliers();
    listInliers = pose.getRansacInliers();
  }
}
