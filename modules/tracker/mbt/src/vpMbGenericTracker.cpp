/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Generic model-based tracker.
 *
 *****************************************************************************/

#include <visp3/mbt/vpMbGenericTracker.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/mbt/vpMbtXmlGenericParser.h>

vpMbGenericTracker::vpMbGenericTracker()
  : m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(),
    m_percentageGdPt(0.4), m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  m_mapOfTrackers["Camera"] = new TrackerWrapper(EDGE_TRACKER);

  // Add default camera transformation matrix
  m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();

  // Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif

  m_mapOfFeatureFactors[DEPTH_NORMAL_TRACKER] = 1.0;
  m_mapOfFeatureFactors[DEPTH_DENSE_TRACKER] = 1.0;
}

vpMbGenericTracker::vpMbGenericTracker(const unsigned int nbCameras, const int trackerType)
  : m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(),
    m_percentageGdPt(0.4), m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  if (nbCameras == 0) {
    throw vpException(vpTrackingException::fatalError, "Cannot use no camera!");
  } else if (nbCameras == 1) {
    m_mapOfTrackers["Camera"] = new TrackerWrapper(trackerType);

    // Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else {
    for (unsigned int i = 1; i <= nbCameras; i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfTrackers[ss.str()] = new TrackerWrapper(trackerType);

      // Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    // Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfTrackers.begin()->first;
  }

  // Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif

  m_mapOfFeatureFactors[DEPTH_NORMAL_TRACKER] = 1.0;
  m_mapOfFeatureFactors[DEPTH_DENSE_TRACKER] = 1.0;
}

vpMbGenericTracker::vpMbGenericTracker(const std::vector<int> &trackerTypes)
  : m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(),
    m_percentageGdPt(0.4), m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  if (trackerTypes.empty()) {
    throw vpException(vpException::badValue, "There is no camera!");
  }

  if (trackerTypes.size() == 1) {
    m_mapOfTrackers["Camera"] = new TrackerWrapper(trackerTypes[0]);

    // Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else {
    for (size_t i = 1; i <= trackerTypes.size(); i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfTrackers[ss.str()] = new TrackerWrapper(trackerTypes[i - 1]);

      // Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    // Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfTrackers.begin()->first;
  }

  // Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif

  m_mapOfFeatureFactors[DEPTH_NORMAL_TRACKER] = 1.0;
  m_mapOfFeatureFactors[DEPTH_DENSE_TRACKER] = 1.0;
}

vpMbGenericTracker::vpMbGenericTracker(const std::vector<std::string> &cameraNames,
                                       const std::vector<int> &trackerTypes)
  : m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(),
    m_percentageGdPt(0.4), m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  if (cameraNames.size() != trackerTypes.size() || cameraNames.empty()) {
    throw vpException(vpTrackingException::badValue,
                      "cameraNames.size() != trackerTypes.size() || cameraNames.empty()");
  }

  for (size_t i = 0; i < cameraNames.size(); i++) {
    m_mapOfTrackers[cameraNames[i]] = new TrackerWrapper(trackerTypes[i]);

    // Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix[cameraNames[i]] = vpHomogeneousMatrix();
  }

  // Set by default the reference camera to the first one
  m_referenceCameraName = m_mapOfTrackers.begin()->first;

  // Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif

  m_mapOfFeatureFactors[DEPTH_NORMAL_TRACKER] = 1.0;
  m_mapOfFeatureFactors[DEPTH_DENSE_TRACKER] = 1.0;
}

vpMbGenericTracker::~vpMbGenericTracker()
{
  for (std::map<std::string, TrackerWrapper *>::iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end();
       ++it) {
    delete it->second;
    it->second = NULL;
  }
}

/*!
  Compute projection error given an input image and camera pose, parameters.
  This projection error uses locations sampled exactly where the model is projected using the camera pose
  and intrinsic parameters.
  You may want to use \sa setProjectionErrorComputation \sa getProjectionError

  to get a projection error computed at the ME locations after a call to track().
  It works similarly to vpMbTracker::getProjectionError function:
  <blockquote>
  Get the error angle between the gradient direction of the model features projected at the resulting pose and their normal.
  The error is expressed in degree between 0 and 90.
  </blockquote>

  \param I : Input grayscale image.
  \param _cMo : Camera pose.
  \param _cam : Camera parameters.
*/
double vpMbGenericTracker::computeCurrentProjectionError(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo,
                                                         const vpCameraParameters &_cam)
{
  double rawTotalProjectionError = 0.0;
  unsigned int nbTotalFeaturesUsed = 0;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    unsigned int nbFeaturesUsed = 0;
    double curProjError = tracker->computeProjectionErrorImpl(I, _cMo, _cam, nbFeaturesUsed);

    if (nbFeaturesUsed > 0) {
      nbTotalFeaturesUsed += nbFeaturesUsed;
      rawTotalProjectionError += curProjError;
    }
  }

  if (nbTotalFeaturesUsed > 0) {
    return vpMath::deg(rawTotalProjectionError / (double)nbTotalFeaturesUsed);
  }

  return 90.0;
}

/*!
  Compute projection error given an input image and camera pose, parameters.
  This projection error uses locations sampled exactly where the model is projected using the camera pose
  and intrinsic parameters.
  You may want to use \sa setProjectionErrorComputation \sa getProjectionError

  to get a projection error computed at the ME locations after a call to track().
  It works similarly to vpMbTracker::getProjectionError function:
  <blockquote>
  Get the error angle between the gradient direction of the model features projected at the resulting pose and their normal.
  The error is expressed in degree between 0 and 90.
  </blockquote>

  \param I_color : Input color image.
  \param _cMo : Camera pose.
  \param _cam : Camera parameters.
*/
double vpMbGenericTracker::computeCurrentProjectionError(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &_cMo,
                                                         const vpCameraParameters &_cam)
{
  vpImage<unsigned char> I;
  vpImageConvert::convert(I_color, I); // FS: Shoudn't we use here m_I that was converted in track() ?

  return computeCurrentProjectionError(I, _cMo, _cam);
}

void vpMbGenericTracker::computeProjectionError()
{
  if (computeProjError) {
    double rawTotalProjectionError = 0.0;
    unsigned int nbTotalFeaturesUsed = 0;

    for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
         it != m_mapOfTrackers.end(); ++it) {
      TrackerWrapper *tracker = it->second;

      double curProjError = tracker->getProjectionError();
      unsigned int nbFeaturesUsed = tracker->nbFeaturesForProjErrorComputation;

      if (nbFeaturesUsed > 0) {
        nbTotalFeaturesUsed += nbFeaturesUsed;
        rawTotalProjectionError += (vpMath::rad(curProjError) * nbFeaturesUsed);
      }
    }

    if (nbTotalFeaturesUsed > 0) {
      projectionError = vpMath::deg(rawTotalProjectionError / (double)nbTotalFeaturesUsed);
    } else {
      projectionError = 90.0;
    }
  } else {
    projectionError = 90.0;
  }
}

void vpMbGenericTracker::computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages)
{
  computeVVSInit(mapOfImages);

  if (m_error.getRows() < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  }

  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  vpMatrix LTL;
  vpColVector LTR, v;
  vpColVector error_prev;

  double mu = m_initialMu;
  vpHomogeneousMatrix cMo_prev;

  bool isoJoIdentity_ = true;

  // Covariance
  vpColVector W_true(m_error.getRows());
  vpMatrix L_true, LVJ_true;

  // Create the map of VelocityTwistMatrices
  std::map<std::string, vpVelocityTwistMatrix> mapOfVelocityTwist;
  for (std::map<std::string, vpHomogeneousMatrix>::const_iterator it = m_mapOfCameraTransformationMatrix.begin();
       it != m_mapOfCameraTransformationMatrix.end(); ++it) {
    vpVelocityTwistMatrix cVo;
    cVo.buildFrom(it->second);
    mapOfVelocityTwist[it->first] = cVo;
  }

  double factorEdge = m_mapOfFeatureFactors[EDGE_TRACKER];
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  double factorKlt = m_mapOfFeatureFactors[KLT_TRACKER];
#endif
  double factorDepth = m_mapOfFeatureFactors[DEPTH_NORMAL_TRACKER];
  double factorDepthDense = m_mapOfFeatureFactors[DEPTH_DENSE_TRACKER];

  while (std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter)) {
    computeVVSInteractionMatrixAndResidu(mapOfImages, mapOfVelocityTwist);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error, error_prev, cMo_prev, mu, reStartFromLastIncrement);
    if (reStartFromLastIncrement) {
      for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
           it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;

        tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo_prev;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
        vpHomogeneousMatrix c_curr_tTc_curr0 =
            m_mapOfCameraTransformationMatrix[it->first] * cMo_prev * tracker->c0Mo.inverse();
        tracker->ctTc0 = c_curr_tTc_curr0;
#endif
      }
    }

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      if (computeCovariance) {
        L_true = m_L;
        if (!isoJoIdentity_) {
          vpVelocityTwistMatrix cVo;
          cVo.buildFrom(cMo);
          LVJ_true = (m_L * (cVo * oJo));
        }
      }

      vpVelocityTwistMatrix cVo;
      if (iter == 0) {
        isoJoIdentity_ = true;
        oJo.eye();

        // If all the 6 dof should be estimated, we check if the interaction
        // matrix is full rank. If not we remove automatically the dof that
        // cannot be estimated This is particularly useful when consering
        // circles (rank 5) and cylinders (rank 4)
        if (isoJoIdentity_) {
          cVo.buildFrom(cMo);

          vpMatrix K; // kernel
          unsigned int rank = (m_L * cVo).kernel(K);
          if (rank == 0) {
            throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
          }

          if (rank != 6) {
            vpMatrix I; // Identity
            I.eye(6);
            oJo = I - K.AtA();

            isoJoIdentity_ = false;
          }
        }
      }

      // Weighting
      double num = 0;
      double den = 0;

      unsigned int start_index = 0;
      for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
           it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;

        if (tracker->m_trackerType & EDGE_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_edge.getRows(); i++) {
            double wi = tracker->m_w_edge[i] * tracker->m_factor[i] * factorEdge;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi * vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_edge.getRows();
        }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
        if (tracker->m_trackerType & KLT_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_klt.getRows(); i++) {
            double wi = tracker->m_w_klt[i] * factorKlt;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi * vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_klt.getRows();
        }
#endif

        if (tracker->m_trackerType & DEPTH_NORMAL_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_depthNormal.getRows(); i++) {
            double wi = tracker->m_w_depthNormal[i] * factorDepth;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi * vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_depthNormal.getRows();
        }

        if (tracker->m_trackerType & DEPTH_DENSE_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_depthDense.getRows(); i++) {
            double wi = tracker->m_w_depthDense[i] * factorDepthDense;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi * vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_depthDense.getRows();
        }
      }

      normRes_1 = normRes;
      normRes = sqrt(num / den);

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L, LTL, m_weightedError, m_error, error_prev, LTR, mu, v);

      cMo_prev = cMo;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
           it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;

        vpHomogeneousMatrix c_curr_tTc_curr0 =
            m_mapOfCameraTransformationMatrix[it->first] * cMo * tracker->c0Mo.inverse();
        tracker->ctTc0 = c_curr_tTc_curr0;
      }
#endif

      // Update cMo
      for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
           it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;
        tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
      }
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMo_prev, L_true, LVJ_true, m_error);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER) {
      tracker->updateMovingEdgeWeights();
    }
  }
}

void vpMbGenericTracker::computeVVSInit()
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::computeVVSInit() should not be called!");
}

void vpMbGenericTracker::computeVVSInit(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages)
{
  unsigned int nbFeatures = 0;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->computeVVSInit(mapOfImages[it->first]);

    nbFeatures += tracker->m_error.getRows();
  }

  m_L.resize(nbFeatures, 6, false, false);
  m_error.resize(nbFeatures, false);

  m_weightedError.resize(nbFeatures, false);
  m_w.resize(nbFeatures, false);
  m_w = 1;
}

void vpMbGenericTracker::computeVVSInteractionMatrixAndResidu()
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::"
                                             "computeVVSInteractionMatrixAndR"
                                             "esidu() should not be called");
}

void vpMbGenericTracker::computeVVSInteractionMatrixAndResidu(
    std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist)
{
  unsigned int start_index = 0;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * tracker->c0Mo.inverse();
    tracker->ctTc0 = c_curr_tTc_curr0;
#endif

    tracker->computeVVSInteractionMatrixAndResidu(mapOfImages[it->first]);

    m_L.insert(tracker->m_L * mapOfVelocityTwist[it->first], start_index, 0);
    m_error.insert(start_index, tracker->m_error);

    start_index += tracker->m_error.getRows();
  }
}

void vpMbGenericTracker::computeVVSWeights()
{
  unsigned int start_index = 0;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->computeVVSWeights();

    m_w.insert(start_index, tracker->m_w);
    start_index += tracker->m_w.getRows();
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The grayscale image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param cam_ : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).

  \note This function will display the model only for the reference camera.
*/
void vpMbGenericTracker::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_,
                                 const vpCameraParameters &cam_, const vpColor &col, const unsigned int thickness,
                                 const bool displayFullModel)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    tracker->display(I, cMo_, cam_, col, thickness, displayFullModel);
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The color image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param cam_ : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).

  \note This function will display the model only for the reference camera.
*/
void vpMbGenericTracker::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                                 const vpCameraParameters &cam_, const vpColor &col, const unsigned int thickness,
                                 const bool displayFullModel)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    tracker->display(I, cMo_, cam_, col, thickness, displayFullModel);
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image.
  \param c1Mo : Pose used to project the 3D model into the first image.
  \param c2Mo : Pose used to project the 3D model into the second image.
  \param cam1 : The first camera parameters.
  \param cam2 : The second camera parameters.
  \param color : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::display(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                 const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                                 const vpCameraParameters &cam1, const vpCameraParameters &cam2, const vpColor &color,
                                 const unsigned int thickness, const bool displayFullModel)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I1 : The first color image.
  \param I2 : The second color image.
  \param c1Mo : Pose used to project the 3D model into the first image.
  \param c2Mo : Pose used to project the 3D model into the second image.
  \param cam1 : The first camera parameters.
  \param cam2 : The second camera parameters.
  \param color : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::display(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, const vpHomogeneousMatrix &c1Mo,
                                 const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
                                 const vpCameraParameters &cam2, const vpColor &color, const unsigned int thickness,
                                 const bool displayFullModel)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param mapOfImages : Map of grayscale images.
  \param mapOfCameraPoses : Map of camera poses.
  \param mapOfCameraParameters : Map of camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).
*/
void vpMbGenericTracker::display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                 const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                                 const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                                 const vpColor &col, const unsigned int thickness, const bool displayFullModel)
{
  // Display only for the given images
  for (std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.begin();
       it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if (it_tracker != m_mapOfTrackers.end() && it_camPose != mapOfCameraPoses.end() &&
        it_cam != mapOfCameraParameters.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements for image:" << it_img->first << "!" << std::endl;
    }
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param mapOfImages : Map of color images.
  \param mapOfCameraPoses : Map of camera poses.
  \param mapOfCameraParameters : Map of camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).
*/
void vpMbGenericTracker::display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
                                 const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                                 const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                                 const vpColor &col, const unsigned int thickness, const bool displayFullModel)
{
  // Display only for the given images
  for (std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img = mapOfImages.begin();
       it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if (it_tracker != m_mapOfTrackers.end() && it_camPose != mapOfCameraPoses.end() &&
        it_cam != mapOfCameraParameters.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements for image:" << it_img->first << "!" << std::endl;
    }
  }
}

/*!
  Get the camera names.

  \return The vector of camera names.
*/
std::vector<std::string> vpMbGenericTracker::getCameraNames() const
{
  std::vector<std::string> cameraNames;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();
       it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    cameraNames.push_back(it_tracker->first);
  }

  return cameraNames;
}

/*!
  Get all the camera parameters.

  \param cam1 : Copy of the camera parameters for the first camera.
  \param cam2 : Copy of the camera parameters for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getCameraParameters(cam1);
    ++it;

    it->second->getCameraParameters(cam2);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Get all the camera parameters.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbGenericTracker::getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const
{
  // Clear the input map
  mapOfCameraParameters.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    vpCameraParameters cam_;
    it->second->getCameraParameters(cam_);
    mapOfCameraParameters[it->first] = cam_;
  }
}

/*!
  Get the camera tracker types.

  \return The map of camera tracker types.
  \sa vpTrackerType
*/
std::map<std::string, int> vpMbGenericTracker::getCameraTrackerTypes() const
{
  std::map<std::string, int> trackingTypes;

  TrackerWrapper *traker;
  for (std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();
       it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    traker = it_tracker->second;
    trackingTypes[it_tracker->first] = traker->getTrackerType();
  }

  return trackingTypes;
}

/*!
  Get the clipping used and defined in vpPolygon3D::vpMbtPolygonClippingType.

  \param clippingFlag1 : Clipping flags for the first camera.
  \param clippingFlag2 : Clipping flags for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::getClipping(unsigned int &clippingFlag1, unsigned int &clippingFlag2) const
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    clippingFlag1 = it->second->getClipping();
    ++it;

    clippingFlag2 = it->second->getClipping();
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Get the clipping used and defined in vpPolygon3D::vpMbtPolygonClippingType.

  \param mapOfClippingFlags : Map of clipping flags.
*/
void vpMbGenericTracker::getClipping(std::map<std::string, unsigned int> &mapOfClippingFlags) const
{
  mapOfClippingFlags.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfClippingFlags[it->first] = tracker->getClipping();
  }
}

/*!
  Return a reference to the faces structure.

  \return Reference to the face structure.
*/
vpMbHiddenFaces<vpMbtPolygon> &vpMbGenericTracker::getFaces()
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    return it->second->getFaces();
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " cannot be found!" << std::endl;
  return faces;
}

/*!
  Return a reference to the faces structure for the given camera name.

  \return Reference to the face structure.
*/
vpMbHiddenFaces<vpMbtPolygon> &vpMbGenericTracker::getFaces(const std::string &cameraName)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    return it->second->getFaces();
  }

  std::cerr << "The camera: " << cameraName << " cannot be found!" << std::endl;
  return faces;
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Return the address of the circle feature list for the reference camera.
*/
std::list<vpMbtDistanceCircle *> &vpMbGenericTracker::getFeaturesCircle()
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getFeaturesCircle();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera:  %s!",
                      m_referenceCameraName.c_str());
  }
}

/*!
  Return the address of the cylinder feature list for the reference camera.
*/
std::list<vpMbtDistanceKltCylinder *> &vpMbGenericTracker::getFeaturesKltCylinder()
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getFeaturesKltCylinder();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera:  %s!",
                      m_referenceCameraName.c_str());
  }
}

/*!
  Return the address of the Klt feature list for the reference camera.
*/
std::list<vpMbtDistanceKltPoints *> &vpMbGenericTracker::getFeaturesKlt()
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getFeaturesKlt();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera:  %s!",
                      m_referenceCameraName.c_str());
  }
}
#endif

/*!
  Return a list of features parameters.
  - ME parameters are: `<feature id (here 0 for ME)>`, `<pt.i()>`, `<pt.j()>`, `<state>`
  - KLT parameters are: `<feature id (here 1 for KLT)>`, `<pt.i()>`, `<pt.j()>`,
  `<klt_id.i()>`, `<klt_id.j()>`, `<klt_id.id>`

  It can be used to display the 3D model with a render engine of your choice.

  \note It returns the model for the reference camera.
*/
std::vector<std::vector<double> > vpMbGenericTracker::getFeaturesForDisplay()
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

  if (it != m_mapOfTrackers.end()) {
    return it->second->getFeaturesForDisplay();
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }

  return std::vector<std::vector<double> >();
}

/*!
  Get a list of features parameters.
  - ME parameters are: `<feature id (here 0 for ME)>`, `<pt.i()>`, `<pt.j()>`, `<state>`
  - KLT parameters are: `<feature id (here 1 for KLT)>`, `<pt.i()>`, `<pt.j()>`,
  `<klt_id.i()>`, `<klt_id.j()>`, `<klt_id.id>`
  It can be used to display the 3D model with a render engine of your choice.
*/
void vpMbGenericTracker::getFeaturesForDisplay(std::map<std::string, std::vector<std::vector<double> > > &mapOfFeatures)
{
  // Clear the input map
  mapOfFeatures.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    mapOfFeatures[it->first] = it->second->getFeaturesForDisplay();
  }
}

/*!
   \return The threshold value between 0 and 1 over good moving edges ratio.
   It allows to decide if the tracker has enough valid moving edges to compute
   a pose. 1 means that all moving edges should be considered as good to have
   a valid pose, while 0.1 means that 10% of the moving edge are enough to
   declare a pose valid.

   \sa setGoodMovingEdgesRatioThreshold()
*/
double vpMbGenericTracker::getGoodMovingEdgesRatioThreshold() const { return m_percentageGdPt; }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Get the current list of KLT points for the reference camera.

  \warning This function convert and copy the OpenCV KLT points into
  vpImagePoints.

  \return the list of KLT points through vpKltOpencv.
*/
std::vector<vpImagePoint> vpMbGenericTracker::getKltImagePoints() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltImagePoints();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return std::vector<vpImagePoint>();
}

/*!
  Get the current list of KLT points and their id for the reference camera.

  \warning This function convert and copy the openCV KLT points into
  vpImagePoints.

  \return the list of KLT points and their id through vpKltOpencv.
*/
std::map<int, vpImagePoint> vpMbGenericTracker::getKltImagePointsWithId() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltImagePointsWithId();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return std::map<int, vpImagePoint>();
}

/*!
  Get the erosion of the mask used on the Model faces.

  \return The erosion for the reference camera.
*/
unsigned int vpMbGenericTracker::getKltMaskBorder() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltMaskBorder();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return 0;
}

/*!
  Get number of KLT points for the reference camera.

  \return Number of KLT points for the reference camera.
*/
int vpMbGenericTracker::getKltNbPoints() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltNbPoints();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return 0;
}

/*!
  Get the klt tracker at the current state for the reference camera.

  \return klt tracker.
*/
vpKltOpencv vpMbGenericTracker::getKltOpencv() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker;
    tracker = it_tracker->second;
    return tracker->getKltOpencv();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return vpKltOpencv();
}

/*!
  Get the klt tracker at the current state.

  \param klt1 : Klt tracker for the first camera.
  \param klt2 : Klt tracker for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::getKltOpencv(vpKltOpencv &klt1, vpKltOpencv &klt2) const
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    klt1 = it->second->getKltOpencv();
    ++it;

    klt2 = it->second->getKltOpencv();
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Get the klt tracker at the current state.

  \param mapOfKlts : Map if klt trackers.
*/
void vpMbGenericTracker::getKltOpencv(std::map<std::string, vpKltOpencv> &mapOfKlts) const
{
  mapOfKlts.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfKlts[it->first] = tracker->getKltOpencv();
  }
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
/*!
  Get the current list of KLT points for the reference camera.

   \return the list of KLT points through vpKltOpencv.
*/
std::vector<cv::Point2f> vpMbGenericTracker::getKltPoints() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltPoints();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return std::vector<cv::Point2f>();
}
#endif

/*!
  Get the threshold for the acceptation of a point.

  \return threshold_outlier : Threshold for the weight below which a point is
  rejected.
*/
double vpMbGenericTracker::getKltThresholdAcceptation() const { return m_thresholdOutlier; }
#endif

/*!
  Get the list of the circles tracked for the specified level. Each circle
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param circlesList : The list of the circles of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLcircle(std::list<vpMbtDistanceCircle *> &circlesList,
                                    const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLcircle(circlesList, level);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the circles tracked for the specified level. Each circle
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceCircle.
  \param circlesList : The list of the circles of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLcircle(const std::string &cameraName, std::list<vpMbtDistanceCircle *> &circlesList,
                                    const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLcircle(circlesList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param cylindersList : The list of the cylinders of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLcylinder(std::list<vpMbtDistanceCylinder *> &cylindersList,
                                      const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLcylinder(cylindersList, level);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceCylinder.
  \param cylindersList : The list of the cylinders of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLcylinder(const std::string &cameraName, std::list<vpMbtDistanceCylinder *> &cylindersList,
                                      const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLcylinder(cylindersList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the lines tracked for the specified level. Each line
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param linesList : The list of the lines of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLline(std::list<vpMbtDistanceLine *> &linesList,
                                  const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

  if (it != m_mapOfTrackers.end()) {
    it->second->getLline(linesList, level);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the lines tracked for the specified level. Each line
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceLine.
  \param linesList : The list of the lines of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLline(const std::string &cameraName, std::list<vpMbtDistanceLine *> &linesList,
                                  const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLline(linesList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  }
}

/*!
  Return a list of primitives parameters to display the model at a given pose and camera parameters.
  - Line parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`.
  - Ellipse parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<mu20>`, `<mu11>`, `<mu02>`.

  It can be used to display the 3D model with a render engine of your choice.

  \param width : Image width.
  \param height : Image height.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not

  \note It returns the model for the reference camera.
*/
std::vector<std::vector<double> > vpMbGenericTracker::getModelForDisplay(unsigned int width, unsigned int height,
                                                                         const vpHomogeneousMatrix &cMo,
                                                                         const vpCameraParameters &cam,
                                                                         const bool displayFullModel)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

  if (it != m_mapOfTrackers.end()) {
    return it->second->getModelForDisplay(width, height, cMo, cam, displayFullModel);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }

  return std::vector<std::vector<double> >();
}

/*!
  Get a list of primitives parameters to display the model at a given pose and camera parameters.
  - Line parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`.
  - Ellipse parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<mu20>`, `<mu11>`, `<mu02>`.

  It can be used to display the 3D model with a render engine of your choice.

  \param mapOfModels : Map of models.
  \param width : Image width.
  \param height : Image height.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
void vpMbGenericTracker::getModelForDisplay(std::map<std::string, std::vector<std::vector<double> > > &mapOfModels,
                                            unsigned int width, unsigned int height,
                                            const vpHomogeneousMatrix &cMo,
                                            const vpCameraParameters &cam,
                                            const bool displayFullModel)
{
  // Clear the input map
  mapOfModels.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    mapOfModels[it->first] = it->second->getModelForDisplay(width, height, cMo, cam, displayFullModel);
  }
}

/*!
  Get the moving edge parameters for the reference camera.

  \return an instance of the moving edge parameters used by the tracker.
*/
vpMe vpMbGenericTracker::getMovingEdge() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

  if (it != m_mapOfTrackers.end()) {
    return it->second->getMovingEdge();
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }

  return vpMe();
}

/*!
  Get the moving edge parameters.

  \param me1 : Moving edge parameters for the first camera.
  \param me2 : Moving edge parameters for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::getMovingEdge(vpMe &me1, vpMe &me2) const
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getMovingEdge(me1);
    ++it;

    it->second->getMovingEdge(me2);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Get the moving edge parameters for all the cameras

  \param mapOfMovingEdges : Map of moving edge parameters for all the cameras.
*/
void vpMbGenericTracker::getMovingEdge(std::map<std::string, vpMe> &mapOfMovingEdges) const
{
  mapOfMovingEdges.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfMovingEdges[it->first] = tracker->getMovingEdge();
  }
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a
  vpMeSite with its flag "state" equal to 0. Only these points are used
  during the virtual visual servoing stage.

  \param level : Pyramid level to consider.

  \exception vpException::dimensionError if level does not represent a used
  level.

  \return the number of good points for the reference camera.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must
  be used.
*/
unsigned int vpMbGenericTracker::getNbPoints(const unsigned int level) const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

  unsigned int nbGoodPoints = 0;
  if (it != m_mapOfTrackers.end()) {

    nbGoodPoints = it->second->getNbPoints(level);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  }

  return nbGoodPoints;
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a
  vpMeSite with its flag "state" equal to 0. Only these points are used
  during the virtual visual servoing stage.

  \param mapOfNbPoints : Map of number of good points (vpMeSite) tracked for
  all the cameras. \param level : Pyramid level to consider.

  \exception vpException::dimensionError if level does not represent a used
  level.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must
  be used.
*/
void vpMbGenericTracker::getNbPoints(std::map<std::string, unsigned int> &mapOfNbPoints, const unsigned int level) const
{
  mapOfNbPoints.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfNbPoints[it->first] = tracker->getNbPoints(level);
  }
}

/*!
  Get the number of polygons (faces) representing the object to track.

  \return Number of polygons for the reference camera.
*/
unsigned int vpMbGenericTracker::getNbPolygon() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    return it->second->getNbPolygon();
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  return 0;
}

/*!
  Get the number of polygons (faces) representing the object to track.

  \return Number of polygons for all the cameras.
*/
void vpMbGenericTracker::getNbPolygon(std::map<std::string, unsigned int> &mapOfNbPolygons) const
{
  mapOfNbPolygons.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfNbPolygons[it->first] = tracker->getNbPolygon();
  }
}

/*!
  Return the polygon (face) "index" for the reference camera.

  \exception vpException::dimensionError if index does not represent a good
  polygon.

  \param index : Index of the polygon to return.
  \return Pointer to the polygon index for the reference camera or NULL in
  case of problem.
*/
vpMbtPolygon *vpMbGenericTracker::getPolygon(const unsigned int index)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    return it->second->getPolygon(index);
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist!" << std::endl;
  return NULL;
}

/*!
  Return the polygon (face) "index" for the specified camera.

  \exception vpException::dimensionError if index does not represent a good
  polygon.

  \param cameraName : Name of the camera to return the polygon.
  \param index : Index of the polygon to return.
  \return Pointer to the polygon index for the specified camera or NULL in
  case of problem.
*/
vpMbtPolygon *vpMbGenericTracker::getPolygon(const std::string &cameraName, const unsigned int index)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    return it->second->getPolygon(index);
  }

  std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  return NULL;
}

/*!
  Get the list of polygons faces (a vpPolygon representing the projection of
  the face in the image and a list of face corners in 3D), with the
  possibility to order by distance to the camera or to use the visibility
  check to consider if the polygon face must be retrieved or not.

  \param orderPolygons : If true, the resulting list is ordered from the
  nearest polygon faces to the farther. \param useVisibility : If true, only
  visible faces will be retrieved. \param clipPolygon : If true, the polygons
  will be clipped according to the clipping flags set in vpMbTracker. \return
  A pair object containing the list of vpPolygon and the list of face corners.

  \note This function will return the 2D polygons faces and 3D face points
  only for the reference camera.
*/
std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > >
vpMbGenericTracker::getPolygonFaces(const bool orderPolygons, const bool useVisibility, const bool clipPolygon)
{
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > polygonFaces;

  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    polygonFaces = tracker->getPolygonFaces(orderPolygons, useVisibility, clipPolygon);
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return polygonFaces;
}

/*!
  Get the list of polygons faces (a vpPolygon representing the projection of
  the face in the image and a list of face corners in 3D), with the
  possibility to order by distance to the camera or to use the visibility
  check to consider if the polygon face must be retrieved or not.

  \param mapOfPolygons : Map of 2D polygon faces.
  \param mapOfPoints : Map of face 3D points.
  \param orderPolygons : If true, the resulting list is ordered from the
  nearest polygon faces to the farther. \param useVisibility : If true, only
  visible faces will be retrieved. \param clipPolygon : If true, the polygons
  will be clipped according to the clipping flags set in vpMbTracker. \return
  A pair object containing the list of vpPolygon and the list of face corners.

  \note This function will return the 2D polygons faces and 3D face points
  only for all the cameras.
*/
void vpMbGenericTracker::getPolygonFaces(std::map<std::string, std::vector<vpPolygon> > &mapOfPolygons,
                                         std::map<std::string, std::vector<std::vector<vpPoint> > > &mapOfPoints,
                                         const bool orderPolygons, const bool useVisibility, const bool clipPolygon)
{
  mapOfPolygons.clear();
  mapOfPoints.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > polygonFaces =
        tracker->getPolygonFaces(orderPolygons, useVisibility, clipPolygon);

    mapOfPolygons[it->first] = polygonFaces.first;
    mapOfPoints[it->first] = polygonFaces.second;
  }
}

/*!
  Get the current pose between the object and the cameras.

  \param c1Mo : The camera pose for the first camera.
  \param c2Mo : The camera pose for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getPose(c1Mo);
    ++it;

    it->second->getPose(c2Mo);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are " << m_mapOfTrackers.size() << " cameras!"
              << std::endl;
  }
}

/*!
  Get the current pose between the object and the cameras.

  \param mapOfCameraPoses : The map of camera poses for all the cameras.
*/
void vpMbGenericTracker::getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const
{
  // Clear the map
  mapOfCameraPoses.clear();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->getPose(mapOfCameraPoses[it->first]);
  }
}

/*!
  The tracker type for the reference camera.
*/
int vpMbGenericTracker::getTrackerType() const
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getTrackerType();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera: %s!",
                      m_referenceCameraName.c_str());
  }
}

void vpMbGenericTracker::init(const vpImage<unsigned char> &I)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
    tracker->init(I);
  }
}

void vpMbGenericTracker::initCircle(const vpPoint & /*p1*/, const vpPoint & /*p2*/, const vpPoint & /*p3*/,
                                    const double /*radius*/, const int /*idFace*/, const std::string & /*name*/)
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initCircle() should not be called!");
}

#ifdef VISP_HAVE_MODULE_GUI

/*!
  Initialise the tracker by clicking in the reference image on the pixels that
  correspond to the 3D points whose coordinates are extracted from a file. In
  this file, comments starting with # character are allowed. Notice that 3D
  point coordinates are expressed in meter in the object frame with their X, Y
  and Z values.

  The structure of this file is the following:

  \code
  # 3D point coordinates
  4                 # Number of points in the file (minimum is four)
  0.01 0.01 0.01    # \
  ...               #  | 3D coordinates in the object frame (X, Y, Z)
  0.01 -0.01 -0.01  # /
  \endcode

  \param I1 : Input grayscale image for the first camera.
  \param I2 : Input grayscale image for the second camera.
  \param initFile1 : File containing the coordinates of at least 4 3D points
  the user has to click in the image acquired by the first camera. This file
  should have .init extension (ie teabox.init).
  \param initFile2 : File
  containing the coordinates of at least 4 3D points the user has to click in
  the image acquired by the second camera. This file should have .init
  extension.
  \param displayHelp : Optionnal display of an image that should
  have the same generic name as the init file (ie teabox.ppm). This image may
  be used to show where to click. This functionality is only available if
  visp_io module is used.
  \param T1 : optional transformation matrix to transform 3D points in \a initFile1
  expressed in the original object frame to the desired object frame.
  \param T2 : optional transformation matrix to transform 3D points in \a initFile2
  expressed in the original object frame to the desired object frame
  (T2==T1 if the init points are expressed in the same object frame which should be the case
  most of the time).

  \exception vpException::ioError : The file specified in \e initFile doesn't
  exist.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                   const std::string &initFile1, const std::string &initFile2, const bool displayHelp,
                                   const vpHomogeneousMatrix &T1, const vpHomogeneousMatrix &T2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initClick(I1, initFile1, displayHelp, T1);

    ++it;

    tracker = it->second;
    tracker->initClick(I2, initFile2, displayHelp, T2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      // Set the reference cMo
      tracker->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "Cannot initClick()! Require two cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracker by clicking in the reference image on the pixels that
  correspond to the 3D points whose coordinates are extracted from a file. In
  this file, comments starting with # character are allowed. Notice that 3D
  point coordinates are expressed in meter in the object frame with their X, Y
  and Z values.

  The structure of this file is the following:

  \code
  # 3D point coordinates
  4                 # Number of points in the file (minimum is four)
  0.01 0.01 0.01    # \
  ...               #  | 3D coordinates in the object frame (X, Y, Z)
  0.01 -0.01 -0.01  # /
  \endcode

  \param I_color1 : Input color image for the first camera.
  \param I_color2 : Input color image for the second camera.
  \param initFile1 : File containing the coordinates of at least 4 3D points
  the user has to click in the image acquired by the first camera. This file
  should have .init extension (ie teabox.init).
  \param initFile2 : File
  containing the coordinates of at least 4 3D points the user has to click in
  the image acquired by the second camera. This file should have .init
  extension.
  \param displayHelp : Optionnal display of an image that should
  have the same generic name as the init file (ie teabox.ppm). This image may
  be used to show where to click. This functionality is only available if
  visp_io module is used.
  \param T1 : optional transformation matrix to transform 3D points in \a initFile1
  expressed in the original object frame to the desired object frame.
  \param T2 : optional transformation matrix to transform 3D points in \a initFile2
  expressed in the original object frame to the desired object frame
  (T2==T1 if the init points are expressed in the same object frame which should be the case
  most of the time).

  \exception vpException::ioError : The file specified in \e initFile doesn't
  exist.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initClick(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
                                   const std::string &initFile1, const std::string &initFile2, const bool displayHelp,
                                   const vpHomogeneousMatrix &T1, const vpHomogeneousMatrix &T2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initClick(I_color1, initFile1, displayHelp, T1);

    ++it;

    tracker = it->second;
    tracker->initClick(I_color2, initFile2, displayHelp, T2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      // Set the reference cMo
      tracker->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "Cannot initClick()! Require two cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracker by clicking in the reference image on the pixels that
  correspond to the 3D points whose coordinates are extracted from a file. In
  this file, comments starting with # character are allowed. Notice that 3D
  point coordinates are expressed in meter in the object frame with their X, Y
  and Z values.

  The structure of this file is the following:

  \code
  # 3D point coordinates
  4                 # Number of points in the file (minimum is four)
  0.01 0.01 0.01    # \
  ...               #  | 3D coordinates in the object frame (X, Y, Z)
  0.01 -0.01 -0.01  # /
  \endcode

  The cameras that have not an init file will be automatically initialized but
  the camera transformation matrices have to be set before.

  \param mapOfImages : Map of grayscale images.
  \param mapOfInitFiles : Map of files containing the points where to click
  for each camera.
  \param displayHelp : Optionnal display of an image that
  should have the same generic name as the init file (ie teabox.ppm). This
  image may be used to show where to click. This functionality is only
  available if visp_io module is used.
  \param mapOfT : optional map of transformation matrices to transform
  3D points in \a mapOfInitFiles expressed in the original object frame to the
  desired object frame (if the init points are expressed in the same object frame
  which should be the case most of the time, all the transformation matrices are identical).

  \exception vpException::ioError : The file specified in \e initFile doesn't
  exist.

  \note Image and init file must be supplied for the reference camera. The
  images for all the cameras must be supplied to correctly initialize the
  trackers but some init files can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                   const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp,
                                   const std::map<std::string, vpHomogeneousMatrix> &mapOfT)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img =
      mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initFile = mapOfInitFiles.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_T = mapOfT.find(it_tracker->first);
    if (it_T != mapOfT.end())
      tracker->initClick(*it_img->second, it_initFile->second, displayHelp, it_T->second);
    else
      tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick for the reference camera!");
  }

  // Vector of missing initFile for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_initFile = mapOfInitFiles.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
        // InitClick for the current camera
        TrackerWrapper *tracker = it_tracker->second;
        tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  // Init for cameras that do not have an initFile
  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->cMo = cCurrentMo;
      m_mapOfTrackers[*it]->init(*it_img->second);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot set the pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Initialise the tracker by clicking in the reference image on the pixels that
  correspond to the 3D points whose coordinates are extracted from a file. In
  this file, comments starting with # character are allowed. Notice that 3D
  point coordinates are expressed in meter in the object frame with their X, Y
  and Z values.

  The structure of this file is the following:

  \code
  # 3D point coordinates
  4                 # Number of points in the file (minimum is four)
  0.01 0.01 0.01    # \
  ...               #  | 3D coordinates in the object frame (X, Y, Z)
  0.01 -0.01 -0.01  # /
  \endcode

  The cameras that have not an init file will be automatically initialized but
  the camera transformation matrices have to be set before.

  \param mapOfColorImages : Map of color images.
  \param mapOfInitFiles : Map of files containing the points where to click
  for each camera.
  \param displayHelp : Optionnal display of an image that
  should have the same generic name as the init file (ie teabox.ppm). This
  image may be used to show where to click. This functionality is only
  available if visp_io module is used.
  \param mapOfT : optional map of transformation matrices to transform
  3D points in \a mapOfInitFiles expressed in the original object frame to the
  desired object frame (if the init points are expressed in the same object frame
  which should be the case most of the time, all the transformation matrices are identical).

  \exception vpException::ioError : The file specified in \e initFile doesn't
  exist.

  \note Image and init file must be supplied for the reference camera. The
  images for all the cameras must be supplied to correctly initialize the
  trackers but some init files can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initClick(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                                   const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp,
                                   const std::map<std::string, vpHomogeneousMatrix> &mapOfT)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img =
      mapOfColorImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initFile = mapOfInitFiles.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfColorImages.end() && it_initFile != mapOfInitFiles.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_T = mapOfT.find(it_tracker->first);
    if (it_T != mapOfT.end())
      tracker->initClick(*it_img->second, it_initFile->second, displayHelp, it_T->second);
    else
      tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick for the reference camera!");
  }

  // Vector of missing initFile for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfColorImages.find(it_tracker->first);
      it_initFile = mapOfInitFiles.find(it_tracker->first);

      if (it_img != mapOfColorImages.end() && it_initFile != mapOfInitFiles.end()) {
        // InitClick for the current camera
        TrackerWrapper *tracker = it_tracker->second;
        tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  // Init for cameras that do not have an initFile
  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfColorImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfColorImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->cMo = cCurrentMo;
      vpImageConvert::convert(*it_img->second, m_mapOfTrackers[*it]->m_I);
      m_mapOfTrackers[*it]->init(m_mapOfTrackers[*it]->m_I);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot set the pose for camera: %s!",
                        it->c_str());
    }
  }
}
#endif

void vpMbGenericTracker::initCylinder(const vpPoint & /*p1*/, const vpPoint & /*p2*/, const double /*radius*/,
                                      const int /*idFace*/, const std::string & /*name*/)
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initCylinder() should not be called!");
}

void vpMbGenericTracker::initFaceFromCorners(vpMbtPolygon & /*polygon*/)
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initFaceFromCorners() should not be called!");
}

void vpMbGenericTracker::initFaceFromLines(vpMbtPolygon & /*polygon*/)
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initFaceFromLines() should not be called!");
}

/*!
  Initialise the tracker by reading 3D point coordinates and the corresponding
  2D image point coordinates from a file. Comments starting with # character
  are allowed. 3D point coordinates are expressed in meter in the object frame
  with X, Y and Z values. 2D point coordinates are expressied in pixel
  coordinates, with first the line and then the column of the pixel in the
  image. The structure of this file is the following.
  \code
 # 3D point coordinates
 4                 # Number of 3D points in the file (minimum is four)
 0.01 0.01 0.01    #  \
 ...               #  | 3D coordinates in meters in the object frame
 0.01 -0.01 -0.01  # /
 # corresponding 2D point coordinates
 4                 # Number of image points in the file (has to be the same as the number of 3D points)
 100 200           #  \
 ...               #  | 2D coordinates in pixel in the image
 50 10  		        #  /
  \endcode

  \param I1 : Input grayscale image for the first camera.
  \param I2 : Input grayscale image for the second camera.
  \param initFile1 : Path to the file containing all the points for the first
  camera.
  \param initFile2 : Path to the file containing all the points for
  the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPoints(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                        const std::string &initFile1, const std::string &initFile2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initFromPoints(I1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPoints(I2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      // Set the reference cMo
      tracker->getPose(cMo);

      // Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "Cannot initFromPoints()! Require two cameras but "
                      "there are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracker by reading 3D point coordinates and the corresponding
  2D image point coordinates from a file. Comments starting with # character
  are allowed. 3D point coordinates are expressed in meter in the object frame
  with X, Y and Z values. 2D point coordinates are expressied in pixel
  coordinates, with first the line and then the column of the pixel in the
  image. The structure of this file is the following.
  \code
 # 3D point coordinates
 4                 # Number of 3D points in the file (minimum is four)
 0.01 0.01 0.01    #  \
 ...               #  | 3D coordinates in meters in the object frame
 0.01 -0.01 -0.01  # /
 # corresponding 2D point coordinates
 4                 # Number of image points in the file (has to be the same as the number of 3D points)
 100 200           #  \
 ...               #  | 2D coordinates in pixel in the image
 50 10  		        #  /
  \endcode

  \param I_color1 : Input color image for the first camera.
  \param I_color2 : Input color image for the second camera.
  \param initFile1 : Path to the file containing all the points for the first
  camera.
  \param initFile2 : Path to the file containing all the points for
  the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPoints(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
                                        const std::string &initFile1, const std::string &initFile2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initFromPoints(I_color1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPoints(I_color2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      // Set the reference cMo
      tracker->getPose(cMo);

      // Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "Cannot initFromPoints()! Require two cameras but "
                      "there are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

void vpMbGenericTracker::initFromPoints(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                        const std::map<std::string, std::string> &mapOfInitPoints)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img =
      mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPoints = mapOfInitPoints.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initPoints != mapOfInitPoints.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPoints(*it_img->second, it_initPoints->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPoints() for the reference camera!");
  }

  // Vector of missing initPoints for cameras
  std::vector<std::string> vectorOfMissingCameraPoints;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_initPoints = mapOfInitPoints.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_initPoints != mapOfInitPoints.end()) {
      // Set pose
      it_tracker->second->initFromPoints(*it_img->second, it_initPoints->second);
    } else {
      vectorOfMissingCameraPoints.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoints.begin();
       it != vectorOfMissingCameraPoints.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot init the pose for camera: %s!",
                        it->c_str());
    }
  }
}

void vpMbGenericTracker::initFromPoints(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                                        const std::map<std::string, std::string> &mapOfInitPoints)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img =
      mapOfColorImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPoints = mapOfInitPoints.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfColorImages.end() && it_initPoints != mapOfInitPoints.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPoints(*it_img->second, it_initPoints->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPoints() for the reference camera!");
  }

  // Vector of missing initPoints for cameras
  std::vector<std::string> vectorOfMissingCameraPoints;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfColorImages.find(it_tracker->first);
    it_initPoints = mapOfInitPoints.find(it_tracker->first);

    if (it_img != mapOfColorImages.end() && it_initPoints != mapOfInitPoints.end()) {
      // Set pose
      it_tracker->second->initFromPoints(*it_img->second, it_initPoints->second);
    } else {
      vectorOfMissingCameraPoints.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoints.begin();
       it != vectorOfMissingCameraPoints.end(); ++it) {
    it_img = mapOfColorImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfColorImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot init the pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read
  in the file initFile.

  \param I1 : Input grayscale image for the first camera.
  \param I2 : Input grayscale image for the second camera.
  \param initFile1 : Init pose file for the first camera.
  \param initFile2 : Init pose file for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                      const std::string &initFile1, const std::string &initFile2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initFromPose(I1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPose(I2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      // Set the reference cMo
      tracker->getPose(cMo);

      // Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "Cannot initFromPose()! Require two cameras but there "
                      "are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read
  in the file initFile.

  \param I_color1 : Input color image for the first camera.
  \param I_color2 : Input color image for the second camera.
  \param initFile1 : Init pose file for the first camera.
  \param initFile2 : Init pose file for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPose(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
                                      const std::string &initFile1, const std::string &initFile2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initFromPose(I_color1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPose(I_color2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      // Set the reference cMo
      tracker->getPose(cMo);

      // Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "Cannot initFromPose()! Require two cameras but there "
                      "are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read
  in the file initFile.

  \param mapOfImages : Map of grayscale images.
  \param mapOfInitPoses : Map of init pose files.

  \note Image and init pose file must be supplied for the reference camera.
  The images for all the cameras must be supplied to correctly initialize the
  trackers but some init pose files can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                      const std::map<std::string, std::string> &mapOfInitPoses)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img =
      mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPose = mapOfInitPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initPose != mapOfInitPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_initPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPose() for the reference camera!");
  }

  // Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_initPose = mapOfInitPoses.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_initPose != mapOfInitPoses.end()) {
      // Set pose
      it_tracker->second->initFromPose(*it_img->second, it_initPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot init the pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read
  in the file initFile.

  \param mapOfColorImages : Map of color images.
  \param mapOfInitPoses : Map of init pose files.

  \note Image and init pose file must be supplied for the reference camera.
  The images for all the cameras must be supplied to correctly initialize the
  trackers but some init pose files can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                                      const std::map<std::string, std::string> &mapOfInitPoses)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img =
      mapOfColorImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPose = mapOfInitPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfColorImages.end() && it_initPose != mapOfInitPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_initPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPose() for the reference camera!");
  }

  // Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfColorImages.find(it_tracker->first);
    it_initPose = mapOfInitPoses.find(it_tracker->first);

    if (it_img != mapOfColorImages.end() && it_initPose != mapOfInitPoses.end()) {
      // Set pose
      it_tracker->second->initFromPose(*it_img->second, it_initPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfColorImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfColorImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot init the pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param I1 : Input grayscale image for the first camera.
  \param I2 : Input grayscale image for the second camera.
  \param c1Mo : Pose matrix for the first camera.
  \param c2Mo : Pose matrix for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                      const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->initFromPose(I1, c1Mo);

    ++it;

    it->second->initFromPose(I2, c2Mo);

    this->cMo = c1Mo;
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "This method requires 2 cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param I_color1 : Input color image for the first camera.
  \param I_color2 : Input color image for the second camera.
  \param c1Mo : Pose matrix for the first camera.
  \param c2Mo : Pose matrix for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPose(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
                                      const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->initFromPose(I_color1, c1Mo);

    ++it;

    it->second->initFromPose(I_color2, c2Mo);

    this->cMo = c1Mo;
  } else {
    throw vpException(vpTrackingException::initializationError,
                      "This method requires 2 cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param mapOfImages : Map of grayscale images.
  \param mapOfCameraPoses : Map of pose matrix.

  \note Image and camera pose must be supplied for the reference camera. The
  images for all the cameras must be supplied to correctly initialize the
  trackers but some camera poses can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img =
      mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot set pose for the reference camera!");
  }

  // Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_camPose = mapOfCameraPoses.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
      // Set pose
      it_tracker->second->initFromPose(*it_img->second, it_camPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot set the pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param mapOfColorImages : Map of color images.
  \param mapOfCameraPoses : Map of pose matrix.

  \note Image and camera pose must be supplied for the reference camera. The
  images for all the cameras must be supplied to correctly initialize the
  trackers but some camera poses can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                                      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img =
      mapOfColorImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfColorImages.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot set pose for the reference camera!");
  }

  // Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfColorImages.find(it_tracker->first);
    it_camPose = mapOfCameraPoses.find(it_tracker->first);

    if (it_img != mapOfColorImages.end() && it_camPose != mapOfCameraPoses.end()) {
      // Set pose
      it_tracker->second->initFromPose(*it_img->second, it_camPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfColorImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfColorImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot set the pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the
  objects: tracking parameters, camera intrinsic parameters.

  \warning To clean up memory allocated by the xml library, the user has to
  call vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file
  not found or wrong format for the data).

  \param configFile : full name of the xml file.

  \sa vpXmlParser::cleanup()
*/
void vpMbGenericTracker::loadConfigFile(const std::string &configFile)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->loadConfigFile(configFile);
  }

  if (m_mapOfTrackers.find(m_referenceCameraName) == m_mapOfTrackers.end()) {
    throw vpException(vpException::fatalError, "Cannot find the reference camera:  %s!", m_referenceCameraName.c_str());
  }

  m_mapOfTrackers[m_referenceCameraName]->getCameraParameters(this->cam);
  this->angleAppears = m_mapOfTrackers[m_referenceCameraName]->getAngleAppear();
  this->angleDisappears = m_mapOfTrackers[m_referenceCameraName]->getAngleDisappear();
  this->clippingFlag = m_mapOfTrackers[m_referenceCameraName]->getClipping();
}

/*!
  Load the xml configuration files.
  From the configuration file initialize the parameters corresponding to the
  objects: tracking parameters, camera intrinsic parameters.

  \warning To clean up memory allocated by the xml library, the user has to
  call vpXmlParser::cleanup() before the exit().

  \param configFile1 : Full name of the xml file for the first camera.
  \param configFile2 : Full name of the xml file for the second camera.

  \sa vpXmlParser::cleanup()

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::loadConfigFile(const std::string &configFile1, const std::string &configFile2)
{
  if (m_mapOfTrackers.size() != 2) {
    throw vpException(vpException::fatalError, "The tracker is not set in a stereo configuration!");
  }

  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();
  TrackerWrapper *tracker = it_tracker->second;
  tracker->loadConfigFile(configFile1);

  ++it_tracker;
  tracker = it_tracker->second;
  tracker->loadConfigFile(configFile2);

  if (m_mapOfTrackers.find(m_referenceCameraName) == m_mapOfTrackers.end()) {
    throw vpException(vpException::fatalError, "Cannot find the reference camera:  %s!", m_referenceCameraName.c_str());
  }

  m_mapOfTrackers[m_referenceCameraName]->getCameraParameters(this->cam);
  this->angleAppears = m_mapOfTrackers[m_referenceCameraName]->getAngleAppear();
  this->angleDisappears = m_mapOfTrackers[m_referenceCameraName]->getAngleDisappear();
  this->clippingFlag = m_mapOfTrackers[m_referenceCameraName]->getClipping();
}

/*!
  Load the xml configuration files.
  From the configuration file initialize the parameters corresponding to the
  objects: tracking parameters, camera intrinsic parameters.

  \warning To clean up memory allocated by the xml library, the user has to
  call vpXmlParser::cleanup() before the exit().

  \param mapOfConfigFiles : Map of xml files.

  \sa loadConfigFile(const std::string &), vpXmlParser::cleanup()

  \note Configuration files must be supplied for all the cameras.
*/
void vpMbGenericTracker::loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();
       it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    TrackerWrapper *tracker = it_tracker->second;

    std::map<std::string, std::string>::const_iterator it_config = mapOfConfigFiles.find(it_tracker->first);
    if (it_config != mapOfConfigFiles.end()) {
      tracker->loadConfigFile(it_config->second);
    } else {
      throw vpException(vpTrackingException::initializationError, "Missing configuration file for camera: %s!",
                        it_tracker->first.c_str());
    }
  }

  // Set the reference camera parameters
  std::map<std::string, TrackerWrapper *>::iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    tracker->getCameraParameters(cam);

    // Set clipping
    this->clippingFlag = tracker->getClipping();
    this->angleAppears = tracker->getAngleAppear();
    this->angleDisappears = tracker->getAngleDisappear();
  } else {
    throw vpException(vpTrackingException::initializationError, "The reference camera: %s does not exist!",
                      m_referenceCameraName.c_str());
  }
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the
  loadCAOModel() method.

  \warning When this class is called to load a vrml model, remember that you
  have to call Call SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION >= 2)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension
is not wrl or cao.

  \param modelFile : the file containing the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading
CAO model files which include other CAO model files.
  \param T : optional transformation matrix (currently only for .cao) to transform
  3D points expressed in the original object frame to the desired object frame.

  \note All the trackers will use the same model in case of stereo / multiple
cameras configuration.
*/
void vpMbGenericTracker::loadModel(const std::string &modelFile, const bool verbose, const vpHomogeneousMatrix &T)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->loadModel(modelFile, verbose, T);
  }
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the
  loadCAOModel() method.

  \warning When this class is called to load a vrml model, remember that you
  have to call Call SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION >= 2)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension
is not wrl or cao.

  \param modelFile1 : the file containing the 3D model description for the
first camera. The extension of this file is either .wrl or .cao.
  \param modelFile2 : the file containing the the 3D model description for the second
camera. The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files
which include other CAO model files.
  \param T1 : optional transformation matrix (currently only for .cao) to transform
  3D points in \a modelFile1 expressed in the original object frame to the desired object frame.
  \param T2 : optional transformation matrix (currently only for .cao) to transform
  3D points in \a modelFile2 expressed in the original object frame to the desired object frame (
  T2==T1 if the two models have the same object frame which should be the case most of the time).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::loadModel(const std::string &modelFile1, const std::string &modelFile2, const bool verbose,
                                   const vpHomogeneousMatrix &T1, const vpHomogeneousMatrix &T2)
{
  if (m_mapOfTrackers.size() != 2) {
    throw vpException(vpException::fatalError, "The tracker is not set in a stereo configuration!");
  }

  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();
  TrackerWrapper *tracker = it_tracker->second;
  tracker->loadModel(modelFile1, verbose, T1);

  ++it_tracker;
  tracker = it_tracker->second;
  tracker->loadModel(modelFile2, verbose, T2);
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the
  loadCAOModel() method.

  \warning When this class is called to load a vrml model, remember that you
  have to call Call SoDD::finish() before ending the program.
  \code
int main()
{
    ...
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION >= 2)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension
is not wrl or cao.

  \param mapOfModelFiles : map of files containing the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading
CAO model files which include other CAO model files.
  \param mapOfT : optional map of transformation matrices (currently only for .cao)
  to transform 3D points in \a mapOfModelFiles expressed in the original object frame to
  the desired object frame (if the models have the same object frame which should be the
  case most of the time, all the transformation matrices are identical).

  \note Each camera must have a model file.
*/
void vpMbGenericTracker::loadModel(const std::map<std::string, std::string> &mapOfModelFiles, const bool verbose,
                                   const std::map<std::string, vpHomogeneousMatrix> &mapOfT)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();
       it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    std::map<std::string, std::string>::const_iterator it_model = mapOfModelFiles.find(it_tracker->first);

    if (it_model != mapOfModelFiles.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      std::map<std::string, vpHomogeneousMatrix>::const_iterator it_T = mapOfT.find(it_tracker->first);

      if (it_T != mapOfT.end())
        tracker->loadModel(it_model->second, verbose, it_T->second);
      else
        tracker->loadModel(it_model->second, verbose);
    } else {
      throw vpException(vpTrackingException::initializationError, "Cannot load model for camera: %s",
                        it_tracker->first.c_str());
    }
  }
}

#ifdef VISP_HAVE_PCL
void vpMbGenericTracker::preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                     std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->preTracking(mapOfImages[it->first], mapOfPointClouds[it->first]);
  }
}
#endif

void vpMbGenericTracker::preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                     std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
                                     std::map<std::string, unsigned int> &mapOfPointCloudWidths,
                                     std::map<std::string, unsigned int> &mapOfPointCloudHeights)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->preTracking(mapOfImages[it->first], mapOfPointClouds[it->first], mapOfPointCloudWidths[it->first],
                         mapOfPointCloudHeights[it->first]);
  }
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The grayscale image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new
  model.
  \param verbose : verbose option to print additional information when
  loading CAO model files which include other CAO model files.
  \param T : optional transformation matrix (currently only for .cao).
*/
void vpMbGenericTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                                     const vpHomogeneousMatrix &cMo_, const bool verbose,
                                     const vpHomogeneousMatrix &T)
{
  if (m_mapOfTrackers.size() != 1) {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly one camera, there are %d cameras!",
                      m_mapOfTrackers.size());
  }

  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  if (it_tracker != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->reInitModel(I, cad_name, cMo_, verbose, T);

    // Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() the reference camera!");
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param I_color : The color image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new
  model.
  \param verbose : verbose option to print additional information when
  loading CAO model files which include other CAO model files.
  \param T : optional transformation matrix (currently only for .cao).
*/
void vpMbGenericTracker::reInitModel(const vpImage<vpRGBa> &I_color, const std::string &cad_name,
                                     const vpHomogeneousMatrix &cMo_, const bool verbose,
                                     const vpHomogeneousMatrix &T)
{
  if (m_mapOfTrackers.size() != 1) {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly one camera, there are %d cameras!",
                      m_mapOfTrackers.size());
  }

  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  if (it_tracker != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->reInitModel(I_color, cad_name, cMo_, verbose, T);

    // Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() the reference camera!");
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param I1 : The grayscale image containing the object to initialize for the first
  camera.
  \param I2 : The grayscale image containing the object to initialize for the second camera.
  \param cad_name1 : Path to the file containing the 3D model description for the first camera.
  \param cad_name2 : Path to the file containing the 3D model description for the second camera.
  \param c1Mo : The new vpHomogeneousMatrix between the first camera and the new model.
  \param c2Mo : The new vpHomogeneousMatrix between the second camera and the new model.
  \param verbose : verbose option to print additional information when
  loading CAO model files which include other CAO model files.
  \param T1 : optional transformation matrix (currently only for .cao) to transform
  3D points in \a cad_name1 expressed in the original object frame to the desired object frame.
  \param T2 : optional transformation matrix (currently only for .cao) to transform
  3D points in \a cad_name2 expressed in the original object frame to the desired object frame (
  T2==T1 if the two models have the same object frame which should be the case most of the time).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                     const std::string &cad_name1, const std::string &cad_name2,
                                     const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                                     const bool verbose,
                                     const vpHomogeneousMatrix &T1, const vpHomogeneousMatrix &T2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();

    it_tracker->second->reInitModel(I1, cad_name1, c1Mo, verbose, T1);

    ++it_tracker;

    it_tracker->second->reInitModel(I2, cad_name2, c2Mo, verbose, T2);

    it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
    if (it_tracker != m_mapOfTrackers.end()) {
      // Set reference pose
      it_tracker->second->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly two cameras!");
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param I_color1 : The color image containing the object to initialize for the first
  camera.
  \param I_color2 : The color image containing the object to initialize for the second camera.
  \param cad_name1 : Path to the file containing the 3D model description for the first camera.
  \param cad_name2 : Path to the file containing the 3D model description for the second camera.
  \param c1Mo : The new vpHomogeneousMatrix between the first camera and the new model.
  \param c2Mo : The new vpHomogeneousMatrix between the second camera and the new model.
  \param verbose : verbose option to print additional information when
  loading CAO model files which include other CAO model files.
  \param T1 : optional transformation matrix (currently only for .cao) to transform
  3D points in \a cad_name1 expressed in the original object frame to the desired object frame.
  \param T2 : optional transformation matrix (currently only for .cao) to transform
  3D points in \a cad_name2 expressed in the original object frame to the desired object frame (
  T2==T1 if the two models have the same object frame which should be the case most of the time).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::reInitModel(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
                                     const std::string &cad_name1, const std::string &cad_name2,
                                     const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                                     const bool verbose,
                                     const vpHomogeneousMatrix &T1, const vpHomogeneousMatrix &T2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.begin();

    it_tracker->second->reInitModel(I_color1, cad_name1, c1Mo, verbose, T1);

    ++it_tracker;

    it_tracker->second->reInitModel(I_color2, cad_name2, c2Mo, verbose, T2);

    it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
    if (it_tracker != m_mapOfTrackers.end()) {
      // Set reference pose
      it_tracker->second->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly two cameras!");
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param mapOfImages : Map of grayscale images.
  \param mapOfModelFiles : Map of model files.
  \param mapOfCameraPoses : The new vpHomogeneousMatrix between the cameras
  and the current object position.
  \param verbose : Verbose option to print additional information when loading CAO model
  files which include other CAO model files.
  \param mapOfT : optional map of transformation matrices (currently only for .cao) to transform
  3D points in \a mapOfModelFiles expressed in the original object frame to the desired object frame
  (if the models have the same object frame which should be the case most of the time,
  all the transformation matrices are identical).
*/
void vpMbGenericTracker::reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                     const std::map<std::string, std::string> &mapOfModelFiles,
                                     const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                                     const bool verbose,
                                     const std::map<std::string, vpHomogeneousMatrix> &mapOfT)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img =
      mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_model = mapOfModelFiles.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_model != mapOfModelFiles.end() &&
      it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_T = mapOfT.find(it_tracker->first);
    if (it_T != mapOfT.end())
      tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose, it_T->second);
    else
      tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose);

    // Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() for reference camera!");
  }

  std::vector<std::string> vectorOfMissingCameras;
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_model = mapOfModelFiles.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_model != mapOfModelFiles.end() && it_camPose != mapOfCameraPoses.end()) {
        TrackerWrapper *tracker = it_tracker->second;
        tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose);
      } else {
        vectorOfMissingCameras.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameras.begin(); it != vectorOfMissingCameras.end();
       ++it) {
    it_img = mapOfImages.find(*it);
    it_model = mapOfModelFiles.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_model != mapOfModelFiles.end() &&
        it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->reInitModel(*it_img->second, it_model->second, cCurrentMo, verbose);
    }
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param mapOfColorImages : Map of color images.
  \param mapOfModelFiles : Map of model files.
  \param mapOfCameraPoses : The new vpHomogeneousMatrix between the cameras
  and the current object position.
  \param verbose : Verbose option to print additional information when loading CAO model
  files which include other CAO model files.
  \param mapOfT : optional map of transformation matrices (currently only for .cao) to transform
  3D points in \a mapOfModelFiles expressed in the original object frame to the desired object frame
  (if the models have the same object frame which should be the case most of the time,
  all the transformation matrices are identical).
*/
void vpMbGenericTracker::reInitModel(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                                     const std::map<std::string, std::string> &mapOfModelFiles,
                                     const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                                     const bool verbose,
                                     const std::map<std::string, vpHomogeneousMatrix> &mapOfT)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img =
      mapOfColorImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_model = mapOfModelFiles.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfColorImages.end() && it_model != mapOfModelFiles.end() &&
      it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_T = mapOfT.find(it_tracker->first);
    if (it_T != mapOfT.end())
      tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose, it_T->second);
    else
      tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose);

    // Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() for reference camera!");
  }

  std::vector<std::string> vectorOfMissingCameras;
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfColorImages.find(it_tracker->first);
      it_model = mapOfModelFiles.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfColorImages.end() && it_model != mapOfModelFiles.end() && it_camPose != mapOfCameraPoses.end()) {
        TrackerWrapper *tracker = it_tracker->second;
        tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose);
      } else {
        vectorOfMissingCameras.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameras.begin(); it != vectorOfMissingCameras.end();
       ++it) {
    it_img = mapOfColorImages.find(*it);
    it_model = mapOfModelFiles.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfColorImages.end() && it_model != mapOfModelFiles.end() &&
        it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->reInitModel(*it_img->second, it_model->second, cCurrentMo, verbose);
    }
  }

  modelInitialised = true;
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void vpMbGenericTracker::resetTracker()
{
  cMo.eye();

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif

  m_computeInteraction = true;
  m_lambda = 1.0;

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);
  clippingFlag = vpPolygon3D::NO_CLIPPING;
  distNearClip = 0.001;
  distFarClip = 100;

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;
  m_maxIter = 30;
  m_stopCriteriaEpsilon = 1e-8;
  m_initialMu = 0.01;

  // Only for Edge
  m_percentageGdPt = 0.4;

  // Only for KLT
  m_thresholdOutlier = 0.5;

  // Reset default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif

  m_mapOfFeatureFactors[DEPTH_NORMAL_TRACKER] = 1.0;
  m_mapOfFeatureFactors[DEPTH_DENSE_TRACKER] = 1.0;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->resetTracker();
  }
}

/*!
  Set the angle used to test polygons appearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value lower than
  this parameter, the polygon is considered as appearing.
  The polygon will then be tracked.

  \param a : new angle in radian.
*/
void vpMbGenericTracker::setAngleAppear(const double &a)
{
  vpMbTracker::setAngleAppear(a);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setAngleAppear(a);
  }
}

/*!
  Set the angle used to test polygons appearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value lower than
  this parameter, the polygon is considered as appearing.
  The polygon will then be tracked.

  \param a1 : new angle in radian for the first camera.
  \param a2 : new angle in radian for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setAngleAppear(const double &a1, const double &a2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setAngleAppear(a1);

    ++it;
    it->second->setAngleAppear(a2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      angleAppears = it->second->getAngleAppear();
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the angle used to test polygons appearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value lower than
  this parameter, the polygon is considered as appearing.
  The polygon will then be tracked.

  \param mapOfAngles : Map of new angles in radian.
*/
void vpMbGenericTracker::setAngleAppear(const std::map<std::string, double> &mapOfAngles)
{
  for (std::map<std::string, double>::const_iterator it = mapOfAngles.begin(); it != mapOfAngles.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setAngleAppear(it->second);

      if (it->first == m_referenceCameraName) {
        angleAppears = it->second;
      }
    }
  }
}

/*!
  Set the angle used to test polygons disappearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value greater than
  this parameter, the polygon is considered as disappearing.
  The tracking of the polygon will then be stopped.

  \param a : new angle in radian.
*/
void vpMbGenericTracker::setAngleDisappear(const double &a)
{
  vpMbTracker::setAngleDisappear(a);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setAngleDisappear(a);
  }
}

/*!
  Set the angle used to test polygons disappearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value greater than
  this parameter, the polygon is considered as disappearing.
  The tracking of the polygon will then be stopped.

  \param a1 : new angle in radian for the first camera.
  \param a2 : new angle in radian for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setAngleDisappear(const double &a1, const double &a2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setAngleDisappear(a1);

    ++it;
    it->second->setAngleDisappear(a2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      angleDisappears = it->second->getAngleDisappear();
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the angle used to test polygons disappearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value greater than
  this parameter, the polygon is considered as disappearing.
  The tracking of the polygon will then be stopped.

  \param mapOfAngles : Map of new angles in radian.
*/
void vpMbGenericTracker::setAngleDisappear(const std::map<std::string, double> &mapOfAngles)
{
  for (std::map<std::string, double>::const_iterator it = mapOfAngles.begin(); it != mapOfAngles.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setAngleDisappear(it->second);

      if (it->first == m_referenceCameraName) {
        angleDisappears = it->second;
      }
    }
  }
}

/*!
  Set the camera parameters.

  \param camera : the new camera parameters.
*/
void vpMbGenericTracker::setCameraParameters(const vpCameraParameters &camera)
{
  vpMbTracker::setCameraParameters(camera);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setCameraParameters(camera);
  }
}

/*!
  Set the camera parameters.

  \param camera1 : the new camera parameters for the first camera.
  \param camera2 : the new camera parameters for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setCameraParameters(const vpCameraParameters &camera1, const vpCameraParameters &camera2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setCameraParameters(camera1);

    ++it;
    it->second->setCameraParameters(camera2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      it->second->getCameraParameters(cam);
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the camera parameters.

  \param mapOfCameraParameters : map of new camera parameters.

  \note This function will set the camera parameters only for the supplied
  camera names.
*/
void vpMbGenericTracker::setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters)
{
  for (std::map<std::string, vpCameraParameters>::const_iterator it = mapOfCameraParameters.begin();
       it != mapOfCameraParameters.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setCameraParameters(it->second);

      if (it->first == m_referenceCameraName) {
        cam = it->second;
      }
    }
  }
}

/*!
  Set the camera transformation matrix for the specified camera (\f$
  _{}^{c_{current}}\textrm{M}_{c_{reference}} \f$).

  \param cameraName : Camera name.
  \param cameraTransformationMatrix : Camera transformation matrix between the
  current and the reference camera.
*/
void vpMbGenericTracker::setCameraTransformationMatrix(const std::string &cameraName,
                                                       const vpHomogeneousMatrix &cameraTransformationMatrix)
{
  std::map<std::string, vpHomogeneousMatrix>::iterator it = m_mapOfCameraTransformationMatrix.find(cameraName);

  if (it != m_mapOfCameraTransformationMatrix.end()) {
    it->second = cameraTransformationMatrix;
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot find camera: %s!", cameraName.c_str());
  }
}

/*!
  Set the map of camera transformation matrices
  (\f$ _{}^{c_1}\textrm{M}_{c_1}, _{}^{c_2}\textrm{M}_{c_1},
  _{}^{c_3}\textrm{M}_{c_1}, \cdots, _{}^{c_n}\textrm{M}_{c_1} \f$).

  \param mapOfTransformationMatrix : map of camera transformation matrices.
*/
void vpMbGenericTracker::setCameraTransformationMatrix(
    const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix)
{
  for (std::map<std::string, vpHomogeneousMatrix>::const_iterator it = mapOfTransformationMatrix.begin();
       it != mapOfTransformationMatrix.end(); ++it) {
    std::map<std::string, vpHomogeneousMatrix>::iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(it->first);

    if (it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      it_camTrans->second = it->second;
    }
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags : New clipping flags.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setClipping(const unsigned int &flags)
{
  vpMbTracker::setClipping(flags);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setClipping(flags);
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags1 : New clipping flags for the first camera.
  \param flags2 : New clipping flags for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setClipping(const unsigned int &flags1, const unsigned int &flags2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setClipping(flags1);

    ++it;
    it->second->setClipping(flags2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      clippingFlag = it->second->getClipping();
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
    }
  } else {
    std::stringstream ss;
    ss << "Require two cameras! There are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param mapOfClippingFlags : Map of new clipping flags.
*/
void vpMbGenericTracker::setClipping(const std::map<std::string, unsigned int> &mapOfClippingFlags)
{
  for (std::map<std::string, unsigned int>::const_iterator it = mapOfClippingFlags.begin();
       it != mapOfClippingFlags.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setClipping(it->second);

      if (it->first == m_referenceCameraName) {
        clippingFlag = it->second;
      }
    }
  }
}

/*!
  Set maximum distance to consider a face.
  You should use the maximum depth range of the sensor used.

  \param maxDistance : Maximum distance to the face.

  \sa setDepthDenseFilteringMethod
  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthDenseFilteringMaxDistance(const double maxDistance)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthDenseFilteringMaxDistance(maxDistance);
  }
}

/*!
  Set method to discard a face, e.g.if outside of the depth range.

  \param method : Depth dense filtering method.

  \sa vpMbtFaceDepthDense::vpDepthDenseFilteringType
  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthDenseFilteringMethod(const int method)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthDenseFilteringMethod(method);
  }
}

/*!
  Set minimum distance to consider a face.
  You should use the minimum depth range of the sensor used.

  \param minDistance : Minimum distance to the face.

  \sa setDepthDenseFilteringMethod
  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthDenseFilteringMinDistance(const double minDistance)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthDenseFilteringMinDistance(minDistance);
  }
}

/*!
  Set depth occupancy ratio to consider a face, used to discard faces where
  the depth map is not well reconstructed.

  \param occupancyRatio : Occupancy ratio, between [0 ; 1].

  \sa setDepthDenseFilteringMethod
  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthDenseFilteringOccupancyRatio(const double occupancyRatio)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthDenseFilteringOccupancyRatio(occupancyRatio);
  }
}

/*!
  Set depth dense sampling step.

  \param stepX : Sampling step in x-direction.
  \param stepY : Sampling step in y-direction.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthDenseSamplingStep(const unsigned int stepX, const unsigned int stepY)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthDenseSamplingStep(stepX, stepY);
  }
}

/*!
  Set method to compute the centroid for display for depth tracker.

  \param method : Centroid computation method.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthNormalFaceCentroidMethod(const vpMbtFaceDepthNormal::vpFaceCentroidType &method)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthNormalFaceCentroidMethod(method);
  }
}

/*!
  Set depth feature estimation method.

  \param method : Depth feature estimation method.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthNormalFeatureEstimationMethod(
    const vpMbtFaceDepthNormal::vpFeatureEstimationType &method)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthNormalFeatureEstimationMethod(method);
  }
}

/*!
  Set depth PCL plane estimation method.

  \param method : Depth PCL plane estimation method.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthNormalPclPlaneEstimationMethod(const int method)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthNormalPclPlaneEstimationMethod(method);
  }
}

/*!
  Set depth PCL RANSAC maximum number of iterations.

  \param maxIter : Depth PCL RANSAC maximum number of iterations.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthNormalPclPlaneEstimationRansacMaxIter(const int maxIter)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthNormalPclPlaneEstimationRansacMaxIter(maxIter);
  }
}

/*!
  Set depth PCL RANSAC threshold.

  \param thresold : Depth PCL RANSAC threshold.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthNormalPclPlaneEstimationRansacThreshold(const double thresold)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthNormalPclPlaneEstimationRansacThreshold(thresold);
  }
}

/*!
  Set depth sampling step.

  \param stepX : Sampling step in x-direction.
  \param stepY : Sampling step in y-direction.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDepthNormalSamplingStep(const unsigned int stepX, const unsigned int stepY)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDepthNormalSamplingStep(stepX, stepY);
  }
}

/*!
  Enable to display the features. By features, we meant the moving edges (ME)
  and the klt points if used.

  Note that if present, the moving edges can be displayed with different
  colors:
  - If green : The ME is a good point.
  - If blue : The ME is removed because of a contrast problem during the
  tracking phase.
  - If purple : The ME is removed because of a threshold problem during the
  tracking phase.
  - If red : The ME is removed because it is rejected by the robust approach
  in the virtual visual servoing scheme.

  \param displayF : set it to true to display the features.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDisplayFeatures(const bool displayF)
{
  vpMbTracker::setDisplayFeatures(displayF);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDisplayFeatures(displayF);
  }
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setFarClippingDistance(const double &dist)
{
  vpMbTracker::setFarClippingDistance(dist);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setFarClippingDistance(dist);
  }
}

/*!
  Set the far distance for clipping.

  \param dist1 : Far clipping value for the first camera.
  \param dist2 : Far clipping value for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setFarClippingDistance(const double &dist1, const double &dist2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setFarClippingDistance(dist1);

    ++it;
    it->second->setFarClippingDistance(dist2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      distFarClip = it->second->getFarClippingDistance();
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the far distance for clipping.

  \param mapOfClippingDists : Map of far clipping values.
*/
void vpMbGenericTracker::setFarClippingDistance(const std::map<std::string, double> &mapOfClippingDists)
{
  for (std::map<std::string, double>::const_iterator it = mapOfClippingDists.begin(); it != mapOfClippingDists.end();
       ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setFarClippingDistance(it->second);

      if (it->first == m_referenceCameraName) {
        distFarClip = it->second;
      }
    }
  }
}

/*!
  Set the feature factors used in the VVS stage (ponderation between the
  feature types).

  \param mapOfFeatureFactors : Map of feature factors.
*/
void vpMbGenericTracker::setFeatureFactors(const std::map<vpTrackerType, double> &mapOfFeatureFactors)
{
  for (std::map<vpTrackerType, double>::iterator it = m_mapOfFeatureFactors.begin(); it != m_mapOfFeatureFactors.end();
       ++it) {
    std::map<vpTrackerType, double>::const_iterator it_factor = mapOfFeatureFactors.find(it->first);
    if (it_factor != mapOfFeatureFactors.end()) {
      it->second = it_factor->second;
    }
  }
}

/*!
   Set the threshold value between 0 and 1 over good moving edges ratio. It
  allows to decide if the tracker has enough valid moving edges to compute a
  pose. 1 means that all moving edges should be considered as good to have a
  valid pose, while 0.1 means that 10% of the moving edge are enough to
  declare a pose valid.

   \param threshold : Value between 0 and 1 that corresponds to the ratio of
  good moving edges that is necessary to consider that the estimated pose is
  valid. Default value is 0.4.

   \sa getGoodMovingEdgesRatioThreshold()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setGoodMovingEdgesRatioThreshold(const double threshold)
{
  m_percentageGdPt = threshold;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setGoodMovingEdgesRatioThreshold(threshold);
  }
}

#ifdef VISP_HAVE_OGRE
/*!
  Set the ratio of visibility attempts that has to be successful to consider a
  polygon as visible.

  \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

  \param ratio : Ratio of succesful attempts that has to be considered. Value
  has to be between 0.0 (0%) and 1.0 (100%).

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setGoodNbRayCastingAttemptsRatio(const double &ratio)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setGoodNbRayCastingAttemptsRatio(ratio);
  }
}

/*!
  Set the number of rays that will be sent toward each polygon for visibility
  test. Each ray will go from the optic center of the camera to a random point
  inside the considered polygon.

  \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

  \param attempts Number of rays to be sent.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setNbRayCastingAttemptsForVisibility(const unsigned int &attempts)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setNbRayCastingAttemptsForVisibility(attempts);
  }
}
#endif

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Set the new value of the klt tracker.

  \param t : Klt tracker containing the new values.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setKltOpencv(const vpKltOpencv &t)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setKltOpencv(t);
  }
}

/*!
  Set the new value of the klt tracker.

  \param t1 : Klt tracker containing the new values for the first camera.
  \param t2 : Klt tracker containing the new values for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setKltOpencv(const vpKltOpencv &t1, const vpKltOpencv &t2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setKltOpencv(t1);

    ++it;
    it->second->setKltOpencv(t2);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the new value of the klt tracker.

  \param mapOfKlts : Map of klt tracker containing the new values.
*/
void vpMbGenericTracker::setKltOpencv(const std::map<std::string, vpKltOpencv> &mapOfKlts)
{
  for (std::map<std::string, vpKltOpencv>::const_iterator it = mapOfKlts.begin(); it != mapOfKlts.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setKltOpencv(it->second);
    }
  }
}

/*!
  Set the threshold for the acceptation of a point.

  \param th : Threshold for the weight below which a point is rejected.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setKltThresholdAcceptation(const double th)
{
  m_thresholdOutlier = th;

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setKltThresholdAcceptation(th);
  }
}
#endif

/*!
  Set the flag to consider if the level of detail (LOD) is used.

  \param useLod : true if the level of detail must be used, false otherwise.
  When true, two parameters can be set, see setMinLineLengthThresh() and
  setMinPolygonAreaThresh(). \param name : name of the face we want to modify
  the LOD parameter.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setLod(const bool useLod, const std::string &name)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setLod(useLod, name);
  }
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Set the erosion of the mask used on the Model faces.

  \param e : The desired erosion.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setKltMaskBorder(const unsigned int &e)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setKltMaskBorder(e);
  }
}

/*!
  Set the erosion of the mask used on the Model faces.

  \param e1 : The desired erosion for the first camera.
  \param e2 : The desired erosion for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setKltMaskBorder(const unsigned int &e1, const unsigned int &e2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setKltMaskBorder(e1);

    ++it;

    it->second->setKltMaskBorder(e2);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the erosion of the mask used on the Model faces.

  \param mapOfErosions : Map of desired erosions.
*/
void vpMbGenericTracker::setKltMaskBorder(const std::map<std::string, unsigned int> &mapOfErosions)
{
  for (std::map<std::string, unsigned int>::const_iterator it = mapOfErosions.begin(); it != mapOfErosions.end();
       ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setKltMaskBorder(it->second);
    }
  }
}
#endif

/*!
  Set the visibility mask.

  \param mask : visibility mask.
*/
void vpMbGenericTracker::setMask(const vpImage<bool> &mask)
{
  vpMbTracker::setMask(mask);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMask(mask);
  }
}


/*!
  Set the threshold for the minimum line length to be considered as visible in
  the LOD case.

  \param minLineLengthThresh : threshold for the minimum line length in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinPolygonAreaThresh()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMinLineLengthThresh(minLineLengthThresh, name);
  }
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in
  pixel. \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinLineLengthThresh()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMinPolygonAreaThresh(minPolygonAreaThresh, name);
  }
}

/*!
  Set the moving edge parameters.

  \param me : an instance of vpMe containing all the desired parameters.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setMovingEdge(const vpMe &me)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMovingEdge(me);
  }
}

/*!
  Set the moving edge parameters.

  \param me1 : an instance of vpMe containing all the desired parameters for
  the first camera. \param me2 : an instance of vpMe containing all the
  desired parameters for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setMovingEdge(const vpMe &me1, const vpMe &me2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setMovingEdge(me1);

    ++it;

    it->second->setMovingEdge(me2);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the moving edge parameters.

  \param mapOfMe : Map of vpMe containing all the desired parameters.
*/
void vpMbGenericTracker::setMovingEdge(const std::map<std::string, vpMe> &mapOfMe)
{
  for (std::map<std::string, vpMe>::const_iterator it = mapOfMe.begin(); it != mapOfMe.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setMovingEdge(it->second);
    }
  }
}

/*!
  Set the near distance for clipping.

  \param dist : Near clipping value.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setNearClippingDistance(const double &dist)
{
  vpMbTracker::setNearClippingDistance(dist);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setNearClippingDistance(dist);
  }
}

/*!
  Set the near distance for clipping.

  \param dist1 : Near clipping value for the first camera.
  \param dist2 : Near clipping value for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setNearClippingDistance(const double &dist1, const double &dist2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setNearClippingDistance(dist1);

    ++it;

    it->second->setNearClippingDistance(dist2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      distNearClip = it->second->getNearClippingDistance();
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the near distance for clipping.

  \param mapOfDists : Map of near clipping values.
*/
void vpMbGenericTracker::setNearClippingDistance(const std::map<std::string, double> &mapOfDists)
{
  for (std::map<std::string, double>::const_iterator it = mapOfDists.begin(); it != mapOfDists.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setNearClippingDistance(it->second);

      if (it->first == m_referenceCameraName) {
        distNearClip = it->second;
      }
    }
  }
}

/*!
  Enable/Disable the appearance of Ogre config dialog on startup.

  \warning This method has only effect when Ogre is used and Ogre visibility
  test is enabled using setOgreVisibilityTest() with true parameter.

  \param showConfigDialog : if true, shows Ogre dialog window (used to set
  Ogre rendering options) when Ogre visibility is enabled. By default, this
  functionality is turned off.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setOgreShowConfigDialog(const bool showConfigDialog)
{
  vpMbTracker::setOgreShowConfigDialog(showConfigDialog);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setOgreShowConfigDialog(showConfigDialog);
  }
}

/*!
  Use Ogre3D for visibility tests

  \warning This function has to be called before the initialization of the
  tracker.

  \param v : True to use it, False otherwise

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setOgreVisibilityTest(const bool &v)
{
  vpMbTracker::setOgreVisibilityTest(v);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->faces.getOgreContext()->setWindowName("Multi Generic MBT (" + it->first + ")");
  }
#endif
}

/*!
  Set the optimization method used during the tracking.

  \param opt : Optimization method to use (see vpMbtOptimizationMethod).

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setOptimizationMethod(const vpMbtOptimizationMethod &opt)
{
  vpMbTracker::setOptimizationMethod(opt);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setOptimizationMethod(opt);
  }
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track()
  function. This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders with
  the KLT tracking.

  \param I : grayscale image corresponding to the desired pose.
  \param cdMo : Pose to affect.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo)
{
  if (m_mapOfTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "The function setPose() requires the generic tracker "
                                                                "to be configured with only one camera!");
  }

  cMo = cdMo;

  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    tracker->setPose(I, cdMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "The reference camera: %s does not exist!",
                      m_referenceCameraName.c_str());
  }
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track()
  function. This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders with
  the KLT tracking.

  \param I_color : color image corresponding to the desired pose.
  \param cdMo : Pose to affect.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo)
{
  if (m_mapOfTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "The function setPose() requires the generic tracker "
                                                                "to be configured with only one camera!");
  }

  cMo = cdMo;

  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    vpImageConvert::convert(I_color, m_I);
    tracker->setPose(m_I, cdMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "The reference camera: %s does not exist!",
                      m_referenceCameraName.c_str());
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I1 : First grayscale image corresponding to the desired pose.
  \param I2 : Second grayscale image corresponding to the desired pose.
  \param c1Mo : First pose to affect.
  \param c2Mo : Second pose to affect.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                 const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setPose(I1, c1Mo);

    ++it;

    it->second->setPose(I2, c2Mo);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      // Set reference pose
      it->second->getPose(cMo);
    } else {
      throw vpException(vpTrackingException::fatalError, "The reference camera: %s does not exist!",
                        m_referenceCameraName.c_str());
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I_color1 : First color image corresponding to the desired pose.
  \param I_color2 : Second color image corresponding to the desired pose.
  \param c1Mo : First pose to affect.
  \param c2Mo : Second pose to affect.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setPose(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
                                 const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setPose(I_color1, c1Mo);

    ++it;

    it->second->setPose(I_color2, c2Mo);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      // Set reference pose
      it->second->getPose(cMo);
    } else {
      throw vpException(vpTrackingException::fatalError, "The reference camera: %s does not exist!",
                        m_referenceCameraName.c_str());
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!",
                      m_mapOfTrackers.size());
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  The camera transformation matrices have to be set before.

  \param mapOfImages : Map of grayscale images.
  \param mapOfCameraPoses : Map of pose to affect to the cameras.

  \note Image and camera pose must be supplied for the reference camera. The
  images for all the cameras must be supplied to correctly initialize the
  trackers but some camera poses can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                 const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img =
      mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->setPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot set pose for the reference camera!");
  }

  // Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        // Set pose
        TrackerWrapper *tracker = it_tracker->second;
        tracker->setPose(*it_img->second, it_camPose->second);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->setPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::fatalError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot set pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  The camera transformation matrices have to be set before.

  \param mapOfColorImages : Map of color images.
  \param mapOfCameraPoses : Map of pose to affect to the cameras.

  \note Image and camera pose must be supplied for the reference camera. The
  images for all the cameras must be supplied to correctly initialize the
  trackers but some camera poses can be omitted. In this case, they will be
  initialized using the pose computed from the reference camera pose and using
  the known geometric transformation between each camera (see
  setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::setPose(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                                 const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses)
{
  // Set the reference cMo
  std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img =
      mapOfColorImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfColorImages.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->setPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot set pose for the reference camera!");
  }

  // Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  // Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfColorImages.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfColorImages.end() && it_camPose != mapOfCameraPoses.end()) {
        // Set pose
        TrackerWrapper *tracker = it_tracker->second;
        tracker->setPose(*it_img->second, it_camPose->second);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin();
       it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfColorImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
        m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfColorImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->setPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::fatalError,
                        "Missing image or missing camera transformation "
                        "matrix! Cannot set pose for camera: %s!",
                        it->c_str());
    }
  }
}

/*!
  Set if the projection error criteria has to be computed. This criteria could
  be used to detect the quality of the tracking. It computes an angle between
  0 and 90 degrees that is available with getProjectionError(). Closer to 0 is
  the value, better is the tracking.

  \param flag : True if the projection error criteria has to be computed,
  false otherwise.

  \sa getProjectionError()

  \note Available only if the edge features are used (e.g. Edge tracking or
  Edge + KLT tracking). Otherwise, the value of 90 degrees will be returned.
*/
void vpMbGenericTracker::setProjectionErrorComputation(const bool &flag)
{
  vpMbTracker::setProjectionErrorComputation(flag);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setProjectionErrorComputation(flag);
  }
}

/*!
  Display or not gradient and model orientation when computing the projection error.
*/
void vpMbGenericTracker::setProjectionErrorDisplay(const bool display)
{
  vpMbTracker::setProjectionErrorDisplay(display);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setProjectionErrorDisplay(display);
  }
}

/*!
  Arrow length used to display gradient and model orientation for projection error computation.
*/
void vpMbGenericTracker::setProjectionErrorDisplayArrowLength(const unsigned int length)
{
  vpMbTracker::setProjectionErrorDisplayArrowLength(length);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setProjectionErrorDisplayArrowLength(length);
  }
}

void vpMbGenericTracker::setProjectionErrorDisplayArrowThickness(const unsigned int thickness)
{
  vpMbTracker::setProjectionErrorDisplayArrowThickness(thickness);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setProjectionErrorDisplayArrowThickness(thickness);
  }
}

/*!
  Set the reference camera name.

  \param referenceCameraName : Name of the reference camera.
*/
void vpMbGenericTracker::setReferenceCameraName(const std::string &referenceCameraName)
{
  std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.find(referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    m_referenceCameraName = referenceCameraName;
  } else {
    std::cerr << "The reference camera: " << referenceCameraName << " does not exist!";
  }
}

void vpMbGenericTracker::setScanLineVisibilityTest(const bool &v)
{
  vpMbTracker::setScanLineVisibilityTest(v);

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setScanLineVisibilityTest(v);
  }
}

/*!
  Set the tracker type.

  \param type : Type of features to used, see vpTrackerType (e.g.
  vpMbGenericTracker::EDGE_TRACKER or vpMbGenericTracker::EDGE_TRACKER |
  vpMbGenericTracker::KLT_TRACKER).

  \note This function will set the new parameter for all the cameras.

  \warning This function has to be called before the loading of the CAD model.
*/
void vpMbGenericTracker::setTrackerType(const int type)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setTrackerType(type);
  }
}

/*!
  Set the tracker types.

  \param mapOfTrackerTypes : Map of feature types to used, see vpTrackerType
  (e.g. vpMbGenericTracker::EDGE_TRACKER or vpMbGenericTracker::EDGE_TRACKER |
  vpMbGenericTracker::KLT_TRACKER).

  \warning This function has to be called before the loading of the CAD model.
*/
void vpMbGenericTracker::setTrackerType(const std::map<std::string, int> &mapOfTrackerTypes)
{
  for (std::map<std::string, int>::const_iterator it = mapOfTrackerTypes.begin(); it != mapOfTrackerTypes.end(); ++it) {
    std::map<std::string, TrackerWrapper *>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);
    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setTrackerType(it->second);
    }
  }
}

/*!
  Set if the polygon that has the given name has to be considered during
  the tracking phase.

  \param name : name of the polygon.
  \param useDepthDenseTracking : True if it has to be considered, False otherwise.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setUseDepthDenseTracking(const std::string &name, const bool &useDepthDenseTracking)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setUseDepthDenseTracking(name, useDepthDenseTracking);
  }
}

/*!
  Set if the polygon that has the given name has to be considered during
  the tracking phase.

  \param name : name of the polygon.
  \param useDepthNormalTracking : True if it has to be considered, False otherwise.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setUseDepthNormalTracking(const std::string &name, const bool &useDepthNormalTracking)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setUseDepthNormalTracking(name, useDepthNormalTracking);
  }
}

/*!
  Set if the polygon that has the given name has to be considered during
  the tracking phase.

  \param name : name of the polygon.
  \param useEdgeTracking : True if it has to be considered, False otherwise.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setUseEdgeTracking(name, useEdgeTracking);
  }
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Set if the polygon that has the given name has to be considered during
  the tracking phase.

  \param name : name of the polygon.
  \param useKltTracking : True if it has to be considered, False otherwise.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setUseKltTracking(const std::string &name, const bool &useKltTracking)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setUseKltTracking(name, useKltTracking);
  }
}
#endif

void vpMbGenericTracker::testTracking()
{
  // Test tracking fails only if all testTracking have failed
  bool isOneTestTrackingOk = false;
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    try {
      tracker->testTracking();
      isOneTestTrackingOk = true;
    } catch (...) {
    }
  }

  if (!isOneTestTrackingOk) {
    std::ostringstream oss;
    oss << "Not enough moving edges to track the object. Try to reduce the "
           "threshold="
        << m_percentageGdPt << " using setGoodMovingEdgesRatioThreshold()";
    throw vpTrackingException(vpTrackingException::fatalError, oss.str());
  }
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param I : The current grayscale image.

  \note This function will track only for the reference camera.
*/
void vpMbGenericTracker::track(const vpImage<unsigned char> &I)
{
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  mapOfImages[m_referenceCameraName] = &I;

  std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;

  track(mapOfImages, mapOfPointClouds, mapOfWidths, mapOfHeights);
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param I_color : The current color image.

  \note This function will track only for the reference camera.
*/
void vpMbGenericTracker::track(const vpImage<vpRGBa> &I_color)
{
  std::map<std::string, const vpImage<vpRGBa> *> mapOfColorImages;
  mapOfColorImages[m_referenceCameraName] = &I_color;

  std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;

  track(mapOfColorImages, mapOfPointClouds, mapOfWidths, mapOfHeights);
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::track(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
    mapOfImages[it->first] = &I1;
    ++it;

    mapOfImages[it->first] = &I2;

    std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
    std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;

    track(mapOfImages, mapOfPointClouds, mapOfWidths, mapOfHeights);
  } else {
    std::stringstream ss;
    ss << "Require two cameras! There are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param I_color1 : The first color image.
  \param _colorI2 : The second color image.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::track(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &_colorI2)
{
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
    std::map<std::string, const vpImage<vpRGBa> *> mapOfImages;
    mapOfImages[it->first] = &I_color1;
    ++it;

    mapOfImages[it->first] = &_colorI2;

    std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
    std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;

    track(mapOfImages, mapOfPointClouds, mapOfWidths, mapOfHeights);
  } else {
    std::stringstream ss;
    ss << "Require two cameras! There are " << m_mapOfTrackers.size() << " cameras!";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfImages : Map of images.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages)
{
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;

  track(mapOfImages, mapOfPointClouds, mapOfWidths, mapOfHeights);
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfColorImages : Map of color images.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages)
{
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;

  track(mapOfColorImages, mapOfPointClouds, mapOfWidths, mapOfHeights);
}

#ifdef VISP_HAVE_PCL
/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfImages : Map of images.
  \param mapOfPointClouds : Map of PCL pointclouds.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                               std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if ((tracker->m_trackerType & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                   KLT_TRACKER |
#endif
                                   DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
      throw vpException(vpException::fatalError, "Bad tracker type: %d", tracker->m_trackerType);
    }

    if (tracker->m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
#endif
                                  ) &&
        mapOfImages[it->first] == NULL) {
      throw vpException(vpException::fatalError, "Image pointer is NULL!");
    }

    if (tracker->m_trackerType & (DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER) &&
        mapOfPointClouds[it->first] == nullptr) {
      throw vpException(vpException::fatalError, "Pointcloud smart pointer is NULL!");
    }
  }

  preTracking(mapOfImages, mapOfPointClouds);

  try {
    computeVVS(mapOfImages);
  } catch (...) {
    covarianceMatrix = -1;
    throw; // throw the original exception
  }

  testTracking();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER && displayFeatures) {
      tracker->m_featuresToBeDisplayedEdge = tracker->getFeaturesForDisplayEdge();
    }

    tracker->postTracking(mapOfImages[it->first], mapOfPointClouds[it->first]);

    if (displayFeatures) {
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (tracker->m_trackerType & KLT_TRACKER) {
        tracker->m_featuresToBeDisplayedKlt = tracker->getFeaturesForDisplayKlt();
      }
#endif

      if (tracker->m_trackerType & DEPTH_NORMAL_TRACKER) {
        tracker->m_featuresToBeDisplayedDepthNormal = tracker->getFeaturesForDisplayDepthNormal();
      }
    }
  }

  computeProjectionError();
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfColorImages : Map of color images.
  \param mapOfPointClouds : Map of PCL pointclouds.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                               std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds)
{
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if ((tracker->m_trackerType & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                   KLT_TRACKER |
#endif
                                   DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
      throw vpException(vpException::fatalError, "Bad tracker type: %d", tracker->m_trackerType);
    }

    if (tracker->m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
#endif
                                  ) && mapOfImages[it->first] == NULL) {
      throw vpException(vpException::fatalError, "Image pointer is NULL!");
    } else if (tracker->m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
#endif
                                  ) && mapOfImages[it->first] != NULL) {
      vpImageConvert::convert(*mapOfColorImages[it->first], tracker->m_I);
      mapOfImages[it->first] = &tracker->m_I; //update grayscale image buffer
    }

    if (tracker->m_trackerType & (DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER) &&
        mapOfPointClouds[it->first] == nullptr) {
      throw vpException(vpException::fatalError, "Pointcloud smart pointer is NULL!");
    }
  }

  preTracking(mapOfImages, mapOfPointClouds);

  try {
    computeVVS(mapOfImages);
  } catch (...) {
    covarianceMatrix = -1;
    throw; // throw the original exception
  }

  testTracking();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER && displayFeatures) {
      tracker->m_featuresToBeDisplayedEdge = tracker->getFeaturesForDisplayEdge();
    }

    tracker->postTracking(mapOfImages[it->first], mapOfPointClouds[it->first]);

    if (displayFeatures) {
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (tracker->m_trackerType & KLT_TRACKER) {
        tracker->m_featuresToBeDisplayedKlt = tracker->getFeaturesForDisplayKlt();
      }
#endif

      if (tracker->m_trackerType & DEPTH_NORMAL_TRACKER) {
        tracker->m_featuresToBeDisplayedDepthNormal = tracker->getFeaturesForDisplayDepthNormal();
      }
    }
  }

  computeProjectionError();
}
#endif

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfImages : Map of images.
  \param mapOfPointClouds : Map of pointclouds.
  \param mapOfPointCloudWidths : Map of pointcloud widths.
  \param mapOfPointCloudHeights : Map of pointcloud heights.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                               std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
                               std::map<std::string, unsigned int> &mapOfPointCloudWidths,
                               std::map<std::string, unsigned int> &mapOfPointCloudHeights)
{
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if ((tracker->m_trackerType & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                   KLT_TRACKER |
#endif
                                   DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
      throw vpException(vpException::fatalError, "Bad tracker type: %d", tracker->m_trackerType);
    }

    if (tracker->m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
#endif
                                  ) &&
        mapOfImages[it->first] == NULL) {
      throw vpException(vpException::fatalError, "Image pointer is NULL!");
    }

    if (tracker->m_trackerType & (DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER) &&
        (mapOfPointClouds[it->first] == NULL)) {
      throw vpException(vpException::fatalError, "Pointcloud is NULL!");
    }
  }

  preTracking(mapOfImages, mapOfPointClouds, mapOfPointCloudWidths, mapOfPointCloudHeights);

  try {
    computeVVS(mapOfImages);
  } catch (...) {
    covarianceMatrix = -1;
    throw; // throw the original exception
  }

  testTracking();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER && displayFeatures) {
      tracker->m_featuresToBeDisplayedEdge = tracker->getFeaturesForDisplayEdge();
    }

    tracker->postTracking(mapOfImages[it->first], mapOfPointCloudWidths[it->first], mapOfPointCloudHeights[it->first]);

    if (displayFeatures) {
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (tracker->m_trackerType & KLT_TRACKER) {
        tracker->m_featuresToBeDisplayedKlt = tracker->getFeaturesForDisplayKlt();
      }
#endif

      if (tracker->m_trackerType & DEPTH_NORMAL_TRACKER) {
        tracker->m_featuresToBeDisplayedDepthNormal = tracker->getFeaturesForDisplayDepthNormal();
      }
    }
  }

  computeProjectionError();
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfColorImages : Map of images.
  \param mapOfPointClouds : Map of pointclouds.
  \param mapOfPointCloudWidths : Map of pointcloud widths.
  \param mapOfPointCloudHeights : Map of pointcloud heights.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
                               std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
                               std::map<std::string, unsigned int> &mapOfPointCloudWidths,
                               std::map<std::string, unsigned int> &mapOfPointCloudHeights)
{
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if ((tracker->m_trackerType & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                   KLT_TRACKER |
#endif
                                   DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
      throw vpException(vpException::fatalError, "Bad tracker type: %d", tracker->m_trackerType);
    }

    if (tracker->m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
#endif
                                  ) && mapOfColorImages[it->first] == NULL) {
      throw vpException(vpException::fatalError, "Image pointer is NULL!");
    } else if (tracker->m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
#endif
                                  ) && mapOfColorImages[it->first] != NULL) {
      vpImageConvert::convert(*mapOfColorImages[it->first], tracker->m_I);
      mapOfImages[it->first] = &tracker->m_I; //update grayscale image buffer
    }

    if (tracker->m_trackerType & (DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER) &&
        (mapOfPointClouds[it->first] == NULL)) {
      throw vpException(vpException::fatalError, "Pointcloud is NULL!");
    }
  }

  preTracking(mapOfImages, mapOfPointClouds, mapOfPointCloudWidths, mapOfPointCloudHeights);

  try {
    computeVVS(mapOfImages);
  } catch (...) {
    covarianceMatrix = -1;
    throw; // throw the original exception
  }

  testTracking();

  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin();
       it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER && displayFeatures) {
      tracker->m_featuresToBeDisplayedEdge = tracker->getFeaturesForDisplayEdge();
    }

    tracker->postTracking(mapOfImages[it->first], mapOfPointCloudWidths[it->first], mapOfPointCloudHeights[it->first]);

    if (displayFeatures) {
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (tracker->m_trackerType & KLT_TRACKER) {
        tracker->m_featuresToBeDisplayedKlt = tracker->getFeaturesForDisplayKlt();
      }
#endif

      if (tracker->m_trackerType & DEPTH_NORMAL_TRACKER) {
        tracker->m_featuresToBeDisplayedDepthNormal = tracker->getFeaturesForDisplayDepthNormal();
      }
    }
  }

  computeProjectionError();
}

/** TrackerWrapper **/
vpMbGenericTracker::TrackerWrapper::TrackerWrapper()
  : m_error(), m_L(), m_trackerType(EDGE_TRACKER), m_w(), m_weightedError()
{
  m_lambda = 1.0;
  m_maxIter = 30;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT TrackerWrapper");

  m_projectionErrorFaces.getOgreContext()->setWindowName("MBT TrackerWrapper (projection error)");
#endif
}

vpMbGenericTracker::TrackerWrapper::TrackerWrapper(const int trackerType)
  : m_error(), m_L(), m_trackerType(trackerType), m_w(), m_weightedError()
{
  if ((m_trackerType & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                        KLT_TRACKER |
#endif
                        DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
    throw vpException(vpTrackingException::badValue, "Bad value for tracker type: %d!", m_trackerType);
  }

  m_lambda = 1.0;
  m_maxIter = 30;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT TrackerWrapper");

  m_projectionErrorFaces.getOgreContext()->setWindowName("MBT TrackerWrapper (projection error)");
#endif
}

vpMbGenericTracker::TrackerWrapper::~TrackerWrapper() { }

// Implemented only for debugging purposes: use TrackerWrapper as a standalone tracker
void vpMbGenericTracker::TrackerWrapper::computeVVS(const vpImage<unsigned char> *const ptr_I)
{
  computeVVSInit(ptr_I);

  if (m_error.getRows() < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  }

  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  double factorEdge = 1.0;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  double factorKlt = 1.0;
#endif
  double factorDepth = 1.0;
  double factorDepthDense = 1.0;

  vpMatrix LTL;
  vpColVector LTR, v;
  vpColVector error_prev;

  double mu = m_initialMu;
  vpHomogeneousMatrix cMo_prev;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpHomogeneousMatrix ctTc0_Prev; // Only for KLT
#endif
  bool isoJoIdentity_ = true;

  // Covariance
  vpColVector W_true(m_error.getRows());
  vpMatrix L_true, LVJ_true;

  unsigned int nb_edge_features = m_error_edge.getRows();
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  unsigned int nb_klt_features = m_error_klt.getRows();
#endif
  unsigned int nb_depth_features = m_error_depthNormal.getRows();
  unsigned int nb_depth_dense_features = m_error_depthDense.getRows();

  while (std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter)) {
    computeVVSInteractionMatrixAndResidu(ptr_I);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error, error_prev, cMo_prev, mu, reStartFromLastIncrement);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    if (reStartFromLastIncrement) {
      if (m_trackerType & KLT_TRACKER) {
        ctTc0 = ctTc0_Prev;
      }
    }
#endif

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      if (computeCovariance) {
        L_true = m_L;
        if (!isoJoIdentity_) {
          vpVelocityTwistMatrix cVo;
          cVo.buildFrom(cMo);
          LVJ_true = (m_L * cVo * oJo);
        }
      }

      vpVelocityTwistMatrix cVo;
      if (iter == 0) {
        isoJoIdentity_ = true;
        oJo.eye();

        // If all the 6 dof should be estimated, we check if the interaction
        // matrix is full rank. If not we remove automatically the dof that
        // cannot be estimated This is particularly useful when consering
        // circles (rank 5) and cylinders (rank 4)
        if (isoJoIdentity_) {
          cVo.buildFrom(cMo);

          vpMatrix K; // kernel
          unsigned int rank = (m_L * cVo).kernel(K);
          if (rank == 0) {
            throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
          }

          if (rank != 6) {
            vpMatrix I; // Identity
            I.eye(6);
            oJo = I - K.AtA();

            isoJoIdentity_ = false;
          }
        }
      }

      // Weighting
      double num = 0;
      double den = 0;

      unsigned int start_index = 0;
      if (m_trackerType & EDGE_TRACKER) {
        for (unsigned int i = 0; i < nb_edge_features; i++) {
          double wi = m_w_edge[i] * m_factor[i] * factorEdge;
          W_true[i] = wi;
          m_weightedError[i] = wi * m_error[i];

          num += wi * vpMath::sqr(m_error[i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[i][j] *= wi;
          }
        }

        start_index += nb_edge_features;
      }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (m_trackerType & KLT_TRACKER) {
        for (unsigned int i = 0; i < nb_klt_features; i++) {
          double wi = m_w_klt[i] * factorKlt;
          W_true[start_index + i] = wi;
          m_weightedError[start_index + i] = wi * m_error_klt[i];

          num += wi * vpMath::sqr(m_error[start_index + i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[start_index + i][j] *= wi;
          }
        }

        start_index += nb_klt_features;
      }
#endif

      if (m_trackerType & DEPTH_NORMAL_TRACKER) {
        for (unsigned int i = 0; i < nb_depth_features; i++) {
          double wi = m_w_depthNormal[i] * factorDepth;
          m_w[start_index + i] = m_w_depthNormal[i];
          m_weightedError[start_index + i] = wi * m_error[start_index + i];

          num += wi * vpMath::sqr(m_error[start_index + i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[start_index + i][j] *= wi;
          }
        }

        start_index += nb_depth_features;
      }

      if (m_trackerType & DEPTH_DENSE_TRACKER) {
        for (unsigned int i = 0; i < nb_depth_dense_features; i++) {
          double wi = m_w_depthDense[i] * factorDepthDense;
          m_w[start_index + i] = m_w_depthDense[i];
          m_weightedError[start_index + i] = wi * m_error[start_index + i];

          num += wi * vpMath::sqr(m_error[start_index + i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[start_index + i][j] *= wi;
          }
        }

        //        start_index += nb_depth_dense_features;
      }

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L, LTL, m_weightedError, m_error, error_prev, LTR, mu, v);

      cMo_prev = cMo;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (m_trackerType & KLT_TRACKER) {
        ctTc0_Prev = ctTc0;
      }
#endif

      cMo = vpExponentialMap::direct(v).inverse() * cMo;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      if (m_trackerType & KLT_TRACKER) {
        ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
      }
#endif
      normRes_1 = normRes;

      normRes = sqrt(num / den);
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMo_prev, L_true, LVJ_true, m_error);

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdgeWeights();
  }
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInit()
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::"
                                             "TrackerWrapper::computeVVSInit("
                                             ") should not be called!");
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInit(const vpImage<unsigned char> *const ptr_I)
{
  initMbtTracking(ptr_I);

  unsigned int nbFeatures = 0;

  if (m_trackerType & EDGE_TRACKER) {
    nbFeatures += m_error_edge.getRows();
  } else {
    m_error_edge.clear();
    m_weightedError_edge.clear();
    m_L_edge.clear();
    m_w_edge.clear();
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    vpMbKltTracker::computeVVSInit();
    nbFeatures += m_error_klt.getRows();
  } else {
    m_error_klt.clear();
    m_weightedError_klt.clear();
    m_L_klt.clear();
    m_w_klt.clear();
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    vpMbDepthNormalTracker::computeVVSInit();
    nbFeatures += m_error_depthNormal.getRows();
  } else {
    m_error_depthNormal.clear();
    m_weightedError_depthNormal.clear();
    m_L_depthNormal.clear();
    m_w_depthNormal.clear();
  }

  if (m_trackerType & DEPTH_DENSE_TRACKER) {
    vpMbDepthDenseTracker::computeVVSInit();
    nbFeatures += m_error_depthDense.getRows();
  } else {
    m_error_depthDense.clear();
    m_weightedError_depthDense.clear();
    m_L_depthDense.clear();
    m_w_depthDense.clear();
  }

  m_L.resize(nbFeatures, 6, false, false);
  m_error.resize(nbFeatures, false);

  m_weightedError.resize(nbFeatures, false);
  m_w.resize(nbFeatures, false);
  m_w = 1;
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu()
{
  throw vpException(vpException::fatalError, "vpMbGenericTracker::"
                                             "TrackerWrapper::"
                                             "computeVVSInteractionMatrixAndR"
                                             "esidu() should not be called!");
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> *const ptr_I)
{
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu(*ptr_I);
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    vpMbKltTracker::computeVVSInteractionMatrixAndResidu();
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    vpMbDepthNormalTracker::computeVVSInteractionMatrixAndResidu();
  }

  if (m_trackerType & DEPTH_DENSE_TRACKER) {
    vpMbDepthDenseTracker::computeVVSInteractionMatrixAndResidu();
  }

  unsigned int start_index = 0;
  if (m_trackerType & EDGE_TRACKER) {
    m_L.insert(m_L_edge, start_index, 0);
    m_error.insert(start_index, m_error_edge);

    start_index += m_error_edge.getRows();
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    m_L.insert(m_L_klt, start_index, 0);
    m_error.insert(start_index, m_error_klt);

    start_index += m_error_klt.getRows();
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    m_L.insert(m_L_depthNormal, start_index, 0);
    m_error.insert(start_index, m_error_depthNormal);

    start_index += m_error_depthNormal.getRows();
  }

  if (m_trackerType & DEPTH_DENSE_TRACKER) {
    m_L.insert(m_L_depthDense, start_index, 0);
    m_error.insert(start_index, m_error_depthDense);

    //    start_index += m_error_depthDense.getRows();
  }
}

void vpMbGenericTracker::TrackerWrapper::computeVVSWeights()
{
  unsigned int start_index = 0;

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSWeights();
    m_w.insert(start_index, m_w_edge);

    start_index += m_w_edge.getRows();
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    vpMbTracker::computeVVSWeights(m_robust_klt, m_error_klt, m_w_klt);
    m_w.insert(start_index, m_w_klt);

    start_index += m_w_klt.getRows();
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    if (m_depthNormalUseRobust) {
      vpMbTracker::computeVVSWeights(m_robust_depthNormal, m_error_depthNormal, m_w_depthNormal);
      m_w.insert(start_index, m_w_depthNormal);
    }

    start_index += m_w_depthNormal.getRows();
  }

  if (m_trackerType & DEPTH_DENSE_TRACKER) {
    vpMbDepthDenseTracker::computeVVSWeights();
    m_w.insert(start_index, m_w_depthDense);

    //    start_index += m_w_depthDense.getRows();
  }
}

void vpMbGenericTracker::TrackerWrapper::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_,
                                                 const vpCameraParameters &camera, const vpColor &col,
                                                 const unsigned int thickness, const bool displayFullModel)
{
  if (displayFeatures) {
    std::vector<std::vector<double> > features = getFeaturesForDisplay();
    for (size_t i = 0; i < features.size(); i++) {
      if (vpMath::equal(features[i][0], 0)) {
        vpImagePoint ip(features[i][1], features[i][2]);
        int state = static_cast<int>(features[i][3]);

        switch (state) {
        case vpMeSite::NO_SUPPRESSION:
          vpDisplay::displayCross(I, ip, 3, vpColor::green, 1);
          break;

        case vpMeSite::CONSTRAST:
          vpDisplay::displayCross(I, ip, 3, vpColor::blue, 1);
          break;

        case vpMeSite::THRESHOLD:
          vpDisplay::displayCross(I, ip, 3, vpColor::purple, 1);
          break;

        case vpMeSite::M_ESTIMATOR:
          vpDisplay::displayCross(I, ip, 3, vpColor::red, 1);
          break;

        case vpMeSite::TOO_NEAR:
          vpDisplay::displayCross(I, ip, 3, vpColor::cyan, 1);
          break;

        default:
          vpDisplay::displayCross(I, ip, 3, vpColor::yellow, 1);
        }
      } else if (vpMath::equal(features[i][0], 1)) {
        vpImagePoint ip1(features[i][1], features[i][2]);
        vpDisplay::displayCross(I, ip1, 10, vpColor::red);

        vpImagePoint ip2(features[i][3], features[i][4]);
        double id = features[i][5];
        std::stringstream ss;
        ss << id;
        vpDisplay::displayText(I, ip2, ss.str(), vpColor::red);
      } else if (vpMath::equal(features[i][0], 2)) {
        vpImagePoint im_centroid(features[i][1], features[i][2]);
        vpImagePoint im_extremity(features[i][3], features[i][4]);
        bool desired = vpMath::equal(features[i][0], 2);
        vpDisplay::displayArrow(I, im_centroid, im_extremity, desired ? vpColor::blue : vpColor::red, 4, 2, thickness);
      }
    }
  }

  std::vector<std::vector<double> > models = getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, camera, displayFullModel);
  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    } else if (vpMath::equal(models[i][0], 1)) {
      vpImagePoint center(models[i][1], models[i][2]);
      double mu20 = models[i][3];
      double mu11 = models[i][4];
      double mu02 = models[i][5];
      vpDisplay::displayEllipse(I, center, mu20, mu11, mu02, true, col, thickness);
    }
  }

#ifdef VISP_HAVE_OGRE
  if ((m_trackerType & EDGE_TRACKER)
    #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      || (m_trackerType & KLT_TRACKER)
    #endif
      ) {
    if (useOgre)
      faces.displayOgre(cMo_);
  }
#endif
}

void vpMbGenericTracker::TrackerWrapper::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                                                 const vpCameraParameters &camera, const vpColor &col,
                                                 const unsigned int thickness, const bool displayFullModel)
{
  if (displayFeatures) {
    std::vector<std::vector<double> > features = getFeaturesForDisplay();
    for (size_t i = 0; i < features.size(); i++) {
      if (vpMath::equal(features[i][0], 0)) {
        vpImagePoint ip(features[i][1], features[i][2]);
        int state = static_cast<int>(features[i][3]);

        switch (state) {
        case vpMeSite::NO_SUPPRESSION:
          vpDisplay::displayCross(I, ip, 3, vpColor::green, 1);
          break;

        case vpMeSite::CONSTRAST:
          vpDisplay::displayCross(I, ip, 3, vpColor::blue, 1);
          break;

        case vpMeSite::THRESHOLD:
          vpDisplay::displayCross(I, ip, 3, vpColor::purple, 1);
          break;

        case vpMeSite::M_ESTIMATOR:
          vpDisplay::displayCross(I, ip, 3, vpColor::red, 1);
          break;

        case vpMeSite::TOO_NEAR:
          vpDisplay::displayCross(I, ip, 3, vpColor::cyan, 1);
          break;

        default:
          vpDisplay::displayCross(I, ip, 3, vpColor::yellow, 1);
        }
      } else if (vpMath::equal(features[i][0], 1)) {
        vpImagePoint ip1(features[i][1], features[i][2]);
        vpDisplay::displayCross(I, ip1, 10, vpColor::red);

        vpImagePoint ip2(features[i][3], features[i][4]);
        double id = features[i][5];
        std::stringstream ss;
        ss << id;
        vpDisplay::displayText(I, ip2, ss.str(), vpColor::red);
      } else if (vpMath::equal(features[i][0], 2)) {
        vpImagePoint im_centroid(features[i][1], features[i][2]);
        vpImagePoint im_extremity(features[i][3], features[i][4]);
        bool desired = vpMath::equal(features[i][0], 2);
        vpDisplay::displayArrow(I, im_centroid, im_extremity, desired ? vpColor::blue : vpColor::red, 4, 2, thickness);
      }
    }
  }

  std::vector<std::vector<double> > models = getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, camera, displayFullModel);
  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    } else if (vpMath::equal(models[i][0], 1)) {
      vpImagePoint center(models[i][1], models[i][2]);
      double mu20 = models[i][3];
      double mu11 = models[i][4];
      double mu02 = models[i][5];
      vpDisplay::displayEllipse(I, center, mu20, mu11, mu02, true, col, thickness);
    }
  }

#ifdef VISP_HAVE_OGRE
  if ((m_trackerType & EDGE_TRACKER)
    #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      || (m_trackerType & KLT_TRACKER)
    #endif
      ) {
    if (useOgre)
      faces.displayOgre(cMo_);
  }
#endif
}

std::vector<std::vector<double> > vpMbGenericTracker::TrackerWrapper::getFeaturesForDisplay()
{
  std::vector<std::vector<double> > features;

  if (m_trackerType & EDGE_TRACKER) {
    //m_featuresToBeDisplayedEdge updated after computeVVS()
    features.insert(features.end(), m_featuresToBeDisplayedEdge.begin(), m_featuresToBeDisplayedEdge.end());
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    //m_featuresToBeDisplayedKlt updated after postTracking()
    features.insert(features.end(), m_featuresToBeDisplayedKlt.begin(), m_featuresToBeDisplayedKlt.end());
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    //m_featuresToBeDisplayedDepthNormal updated after postTracking()
    features.insert(features.end(), m_featuresToBeDisplayedDepthNormal.begin(), m_featuresToBeDisplayedDepthNormal.end());
  }

  return features;
}

std::vector<std::vector<double> > vpMbGenericTracker::TrackerWrapper::getModelForDisplay(unsigned int width, unsigned int height,
                                                                                         const vpHomogeneousMatrix &cMo_,
                                                                                         const vpCameraParameters &camera,
                                                                                         const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  //Do not add multiple times the same models
  if (m_trackerType == EDGE_TRACKER) {
    models = vpMbEdgeTracker::getModelForDisplay(width, height, cMo_, camera, displayFullModel);
  }
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  else if (m_trackerType == KLT_TRACKER) {
    models = vpMbKltTracker::getModelForDisplay(width, height, cMo_, camera, displayFullModel);
  }
#endif
  else if (m_trackerType == DEPTH_NORMAL_TRACKER) {
    models = vpMbDepthNormalTracker::getModelForDisplay(width, height, cMo_, camera, displayFullModel);
  } else if (m_trackerType == DEPTH_DENSE_TRACKER) {
    models = vpMbDepthDenseTracker::getModelForDisplay(width, height, cMo_, camera, displayFullModel);
  } else {
    //Edge and KLT trackers use the same primitives
    if (m_trackerType & EDGE_TRACKER) {
      std::vector<std::vector<double> > edgeModels = vpMbEdgeTracker::getModelForDisplay(width, height, cMo_, camera, displayFullModel);
      models.insert(models.end(), edgeModels.begin(), edgeModels.end());
    }

    //Depth dense and depth normal trackers use the same primitives
    if (m_trackerType & DEPTH_DENSE_TRACKER) {
      std::vector<std::vector<double> > depthDenseModels = vpMbDepthDenseTracker::getModelForDisplay(width, height, cMo_, camera, displayFullModel);
      models.insert(models.end(), depthDenseModels.begin(), depthDenseModels.end());
    }
  }

  return models;
}

void vpMbGenericTracker::TrackerWrapper::init(const vpImage<unsigned char> &I)
{
  if (!modelInitialised) {
    throw vpException(vpException::fatalError, "model not initialized");
  }

  if (useScanLine || clippingFlag > 3)
    cam.computeFov(I.getWidth(), I.getHeight());

  bool reInitialisation = false;
  if (!useOgre) {
    faces.setVisible(I.getWidth(), I.getHeight(), cam, cMo, angleAppears, angleDisappears, reInitialisation);
  } else {
#ifdef VISP_HAVE_OGRE
    if (!faces.isOgreInitialised()) {
      faces.setBackgroundSizeOgre(I.getHeight(), I.getWidth());

      faces.setOgreShowConfigDialog(ogreShowConfigDialog);
      faces.initOgre(cam);
      // Turn off Ogre config dialog display for the next call to this
      // function since settings are saved in the ogre.cfg file and used
      // during the next call
      ogreShowConfigDialog = false;
    }

    faces.setVisibleOgre(I.getWidth(), I.getHeight(), cam, cMo, angleAppears, angleDisappears, reInitialisation);
#else
    faces.setVisible(I.getWidth(), I.getHeight(), cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
  }

  if (useScanLine) {
    faces.computeClippedPolygons(cMo, cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::reinit(I);
#endif

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::resetMovingEdge();

    bool a = false;
    vpMbEdgeTracker::visibleFace(I, cMo, a); // should be useless, but keep it for nbvisiblepolygone

    initMovingEdge(I, cMo);
  }

  if (m_trackerType & DEPTH_NORMAL_TRACKER)
    vpMbDepthNormalTracker::computeVisibility(I.getWidth(), I.getHeight()); // vpMbDepthNormalTracker::init(I);

  if (m_trackerType & DEPTH_DENSE_TRACKER)
    vpMbDepthDenseTracker::computeVisibility(I.getWidth(), I.getHeight()); // vpMbDepthDenseTracker::init(I);
}

void vpMbGenericTracker::TrackerWrapper::initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3,
                                                    const double radius, const int idFace, const std::string &name)
{
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initCircle(p1, p2, p3, radius, idFace, name);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initCircle(p1, p2, p3, radius, idFace, name);
#endif
}

void vpMbGenericTracker::TrackerWrapper::initCylinder(const vpPoint &p1, const vpPoint &p2, const double radius,
                                                      const int idFace, const std::string &name)
{
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initCylinder(p1, p2, radius, idFace, name);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initCylinder(p1, p2, radius, idFace, name);
#endif
}

void vpMbGenericTracker::TrackerWrapper::initFaceFromCorners(vpMbtPolygon &polygon)
{
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initFaceFromCorners(polygon);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initFaceFromCorners(polygon);
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER)
    vpMbDepthNormalTracker::initFaceFromCorners(polygon);

  if (m_trackerType & DEPTH_DENSE_TRACKER)
    vpMbDepthDenseTracker::initFaceFromCorners(polygon);
}

void vpMbGenericTracker::TrackerWrapper::initFaceFromLines(vpMbtPolygon &polygon)
{
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initFaceFromLines(polygon);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initFaceFromLines(polygon);
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER)
    vpMbDepthNormalTracker::initFaceFromLines(polygon);

  if (m_trackerType & DEPTH_DENSE_TRACKER)
    vpMbDepthDenseTracker::initFaceFromLines(polygon);
}

void vpMbGenericTracker::TrackerWrapper::initMbtTracking(const vpImage<unsigned char> *const ptr_I)
{
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSInit();
    vpMbEdgeTracker::computeVVSFirstPhaseFactor(*ptr_I, 0);
  }
}

void vpMbGenericTracker::TrackerWrapper::loadConfigFile(const std::string &configFile)
{
  // Load projection error config
  vpMbTracker::loadConfigFile(configFile);

#ifdef VISP_HAVE_XML2
  vpMbtXmlGenericParser xmlp((vpMbtXmlGenericParser::vpParserType)m_trackerType);

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  // Edge
  xmlp.setEdgeMe(me);

  // KLT
  xmlp.setKltMaxFeatures(10000);
  xmlp.setKltWindowSize(5);
  xmlp.setKltQuality(0.01);
  xmlp.setKltMinDistance(5);
  xmlp.setKltHarrisParam(0.01);
  xmlp.setKltBlockSize(3);
  xmlp.setKltPyramidLevels(3);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  xmlp.setKltMaskBorder(maskBorder);
#endif

  // Depth normal
  xmlp.setDepthNormalFeatureEstimationMethod(m_depthNormalFeatureEstimationMethod);
  xmlp.setDepthNormalPclPlaneEstimationMethod(m_depthNormalPclPlaneEstimationMethod);
  xmlp.setDepthNormalPclPlaneEstimationRansacMaxIter(m_depthNormalPclPlaneEstimationRansacMaxIter);
  xmlp.setDepthNormalPclPlaneEstimationRansacThreshold(m_depthNormalPclPlaneEstimationRansacThreshold);
  xmlp.setDepthNormalSamplingStepX(m_depthNormalSamplingStepX);
  xmlp.setDepthNormalSamplingStepY(m_depthNormalSamplingStepY);

  // Depth dense
  xmlp.setDepthDenseSamplingStepX(m_depthDenseSamplingStepX);
  xmlp.setDepthDenseSamplingStepY(m_depthDenseSamplingStepY);

  try {

    std::cout << " *********** Parsing XML for";

    std::vector<std::string> tracker_names;
    if (m_trackerType & EDGE_TRACKER)
      tracker_names.push_back("Edge");
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    if (m_trackerType & KLT_TRACKER)
      tracker_names.push_back("Klt");
#endif
    if (m_trackerType & DEPTH_NORMAL_TRACKER)
      tracker_names.push_back("Depth Normal");
    if (m_trackerType & DEPTH_DENSE_TRACKER)
      tracker_names.push_back("Depth Dense");

    for (size_t i = 0; i < tracker_names.size(); i++) {
      std::cout << " " << tracker_names[i];
      if (i == tracker_names.size() - 1) {
        std::cout << " ";
      }
    }

    std::cout << "Model-Based Tracker ************ " << std::endl;
    xmlp.parse(configFile);
  } catch (...) {
    throw vpException(vpException::ioError, "Can't open XML file \"%s\"\n ", configFile.c_str());
  }

  vpCameraParameters camera;
  xmlp.getCameraParameters(camera);
  setCameraParameters(camera);

  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  if (xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());

  if (xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());

  if (xmlp.getFovClipping()) {
    setClipping(vpMbEdgeTracker::clippingFlag | vpPolygon3D::FOV_CLIPPING);
  }

  useLodGeneral = xmlp.getLodState();
  minLineLengthThresholdGeneral = xmlp.getLodMinLineLengthThreshold();
  minPolygonAreaThresholdGeneral = xmlp.getLodMinPolygonAreaThreshold();

  applyLodSettingInConfig = false;
  if (this->getNbPolygon() > 0) {
    applyLodSettingInConfig = true;
    setLod(useLodGeneral);
    setMinLineLengthThresh(minLineLengthThresholdGeneral);
    setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);
  }

  // Edge
  vpMe meParser;
  xmlp.getEdgeMe(meParser);
  vpMbEdgeTracker::setMovingEdge(meParser);

// KLT
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  tracker.setMaxFeatures((int)xmlp.getKltMaxFeatures());
  tracker.setWindowSize((int)xmlp.getKltWindowSize());
  tracker.setQuality(xmlp.getKltQuality());
  tracker.setMinDistance(xmlp.getKltMinDistance());
  tracker.setHarrisFreeParameter(xmlp.getKltHarrisParam());
  tracker.setBlockSize((int)xmlp.getKltBlockSize());
  tracker.setPyramidLevels((int)xmlp.getKltPyramidLevels());
  maskBorder = xmlp.getKltMaskBorder();

  // if(useScanLine)
  faces.getMbScanLineRenderer().setMaskBorder(maskBorder);
#endif

  // Depth normal
  setDepthNormalFeatureEstimationMethod(xmlp.getDepthNormalFeatureEstimationMethod());
  setDepthNormalPclPlaneEstimationMethod(xmlp.getDepthNormalPclPlaneEstimationMethod());
  setDepthNormalPclPlaneEstimationRansacMaxIter(xmlp.getDepthNormalPclPlaneEstimationRansacMaxIter());
  setDepthNormalPclPlaneEstimationRansacThreshold(xmlp.getDepthNormalPclPlaneEstimationRansacThreshold());
  setDepthNormalSamplingStep(xmlp.getDepthNormalSamplingStepX(), xmlp.getDepthNormalSamplingStepY());

  // Depth dense
  setDepthDenseSamplingStep(xmlp.getDepthDenseSamplingStepX(), xmlp.getDepthDenseSamplingStepY());
#else
  std::cerr << "You need the libXML2 to read the config file: " << configFile << std::endl;
#endif
}

#ifdef VISP_HAVE_PCL
void vpMbGenericTracker::TrackerWrapper::postTracking(const vpImage<unsigned char> *const ptr_I,
                                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  // KLT
  if (m_trackerType & KLT_TRACKER) {
    if (vpMbKltTracker::postTracking(*ptr_I, m_w_klt)) {
      vpMbKltTracker::reinit(*ptr_I);
    }
  }
#endif

  // Looking for new visible face
  if (m_trackerType & EDGE_TRACKER) {
    bool newvisibleface = false;
    vpMbEdgeTracker::visibleFace(*ptr_I, cMo, newvisibleface);

    if (useScanLine) {
      faces.computeClippedPolygons(cMo, cam);
      faces.computeScanLineRender(cam, ptr_I->getWidth(), ptr_I->getHeight());
    }
  }

  // Depth normal
  if (m_trackerType & DEPTH_NORMAL_TRACKER)
    vpMbDepthNormalTracker::computeVisibility(point_cloud->width, point_cloud->height);

  // Depth dense
  if (m_trackerType & DEPTH_DENSE_TRACKER)
    vpMbDepthDenseTracker::computeVisibility(point_cloud->width, point_cloud->height);

  // Edge
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdge(*ptr_I);

    vpMbEdgeTracker::initMovingEdge(*ptr_I, cMo);
    // Reinit the moving edge for the lines which need it.
    vpMbEdgeTracker::reinitMovingEdge(*ptr_I, cMo);

    if (computeProjError) {
      vpMbEdgeTracker::computeProjectionError(*ptr_I);
    }
  }
}

void vpMbGenericTracker::TrackerWrapper::preTracking(const vpImage<unsigned char> *const ptr_I,
                                                     const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
  if (m_trackerType & EDGE_TRACKER) {
    try {
      vpMbEdgeTracker::trackMovingEdge(*ptr_I);
    } catch (...) {
      std::cerr << "Error in moving edge tracking" << std::endl;
      throw;
    }
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    try {
      vpMbKltTracker::preTracking(*ptr_I);
    } catch (const vpException &e) {
      std::cerr << "Error in KLT tracking: " << e.what() << std::endl;
      throw;
    }
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    try {
      vpMbDepthNormalTracker::segmentPointCloud(point_cloud);
    } catch (...) {
      std::cerr << "Error in Depth normal tracking" << std::endl;
      throw;
    }
  }

  if (m_trackerType & DEPTH_DENSE_TRACKER) {
    try {
      vpMbDepthDenseTracker::segmentPointCloud(point_cloud);
    } catch (...) {
      std::cerr << "Error in Depth dense tracking" << std::endl;
      throw;
    }
  }
}
#endif

void vpMbGenericTracker::TrackerWrapper::postTracking(const vpImage<unsigned char> *const ptr_I,
                                                      const unsigned int pointcloud_width,
                                                      const unsigned int pointcloud_height)
{
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  // KLT
  if (m_trackerType & KLT_TRACKER) {
    if (vpMbKltTracker::postTracking(*ptr_I, m_w_klt)) {
      vpMbKltTracker::reinit(*ptr_I);
    }
  }
#endif

  // Looking for new visible face
  if (m_trackerType & EDGE_TRACKER) {
    bool newvisibleface = false;
    vpMbEdgeTracker::visibleFace(*ptr_I, cMo, newvisibleface);

    if (useScanLine) {
      faces.computeClippedPolygons(cMo, cam);
      faces.computeScanLineRender(cam, ptr_I->getWidth(), ptr_I->getHeight());
    }
  }

  // Depth normal
  if (m_trackerType & DEPTH_NORMAL_TRACKER)
    vpMbDepthNormalTracker::computeVisibility(pointcloud_width, pointcloud_height);

  // Depth dense
  if (m_trackerType & DEPTH_DENSE_TRACKER)
    vpMbDepthDenseTracker::computeVisibility(pointcloud_width, pointcloud_height);

  // Edge
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdge(*ptr_I);

    vpMbEdgeTracker::initMovingEdge(*ptr_I, cMo);
    // Reinit the moving edge for the lines which need it.
    vpMbEdgeTracker::reinitMovingEdge(*ptr_I, cMo);

    if (computeProjError) {
      vpMbEdgeTracker::computeProjectionError(*ptr_I);
    }
  }
}

void vpMbGenericTracker::TrackerWrapper::preTracking(const vpImage<unsigned char> * const ptr_I,
                                                     const std::vector<vpColVector> * const point_cloud,
                                                     const unsigned int pointcloud_width,
                                                     const unsigned int pointcloud_height)
{
  if (m_trackerType & EDGE_TRACKER) {
    try {
      vpMbEdgeTracker::trackMovingEdge(*ptr_I);
    } catch (...) {
      std::cerr << "Error in moving edge tracking" << std::endl;
      throw;
    }
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    try {
      vpMbKltTracker::preTracking(*ptr_I);
    } catch (const vpException &e) {
      std::cerr << "Error in KLT tracking: " << e.what() << std::endl;
      throw;
    }
  }
#endif

  if (m_trackerType & DEPTH_NORMAL_TRACKER) {
    try {
      vpMbDepthNormalTracker::segmentPointCloud(*point_cloud, pointcloud_width, pointcloud_height);
    } catch (...) {
      std::cerr << "Error in Depth tracking" << std::endl;
      throw;
    }
  }

  if (m_trackerType & DEPTH_DENSE_TRACKER) {
    try {
      vpMbDepthDenseTracker::segmentPointCloud(*point_cloud, pointcloud_width, pointcloud_height);
    } catch (...) {
      std::cerr << "Error in Depth dense tracking" << std::endl;
      throw;
    }
  }
}

void vpMbGenericTracker::TrackerWrapper::reInitModel(const vpImage<unsigned char> * const I, const vpImage<vpRGBa> * const I_color,
                                                     const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose,
                                                     const vpHomogeneousMatrix &T)
{
  cMo.eye();

  // Edge
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i++) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (l != NULL)
          delete l;
        l = NULL;
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        cy = *it;
        if (cy != NULL)
          delete cy;
        cy = NULL;
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (ci != NULL)
          delete ci;
        ci = NULL;
      }

      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }

  nline = 0;
  ncylinder = 0;
  ncircle = 0;
  nbvisiblepolygone = 0;

// KLT
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
#if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if (cur != NULL) {
    cvReleaseImage(&cur);
    cur = NULL;
  }
#endif

  // delete the Klt Polygon features
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly != NULL) {
      delete kltpoly;
    }
    kltpoly = NULL;
  }
  kltPolygons.clear();

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    if (kltPolyCylinder != NULL) {
      delete kltPolyCylinder;
    }
    kltPolyCylinder = NULL;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    ci = *it;
    if (ci != NULL) {
      delete ci;
    }
    ci = NULL;
  }
  circles_disp.clear();

  firstInitialisation = true;

#endif

  // Depth normal
  for (size_t i = 0; i < m_depthNormalFaces.size(); i++) {
    delete m_depthNormalFaces[i];
    m_depthNormalFaces[i] = NULL;
  }
  m_depthNormalFaces.clear();

  // Depth dense
  for (size_t i = 0; i < m_depthDenseFaces.size(); i++) {
    delete m_depthDenseFaces[i];
    m_depthDenseFaces[i] = NULL;
  }
  m_depthDenseFaces.clear();

  faces.reset();

  loadModel(cad_name, verbose, T);
  if (I) {
    initFromPose(*I, cMo_);
  } else {
    initFromPose(*I_color, cMo_);
  }
}

void vpMbGenericTracker::TrackerWrapper::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                                                     const vpHomogeneousMatrix &cMo_, const bool verbose,
                                                     const vpHomogeneousMatrix &T)
{
  reInitModel(&I, NULL, cad_name, cMo_, verbose, T);
}

void vpMbGenericTracker::TrackerWrapper::reInitModel(const vpImage<vpRGBa> &I_color, const std::string &cad_name,
                                                     const vpHomogeneousMatrix &cMo_, const bool verbose,
                                                     const vpHomogeneousMatrix &T)
{
  reInitModel(NULL, &I_color, cad_name, cMo_, verbose, T);
}

void vpMbGenericTracker::TrackerWrapper::resetTracker()
{
  vpMbEdgeTracker::resetTracker();
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::resetTracker();
#endif
  vpMbDepthNormalTracker::resetTracker();
  vpMbDepthDenseTracker::resetTracker();
}

void vpMbGenericTracker::TrackerWrapper::setCameraParameters(const vpCameraParameters &camera)
{
  this->cam = camera;

  vpMbEdgeTracker::setCameraParameters(cam);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::setCameraParameters(cam);
#endif
  vpMbDepthNormalTracker::setCameraParameters(cam);
  vpMbDepthDenseTracker::setCameraParameters(cam);
}

void vpMbGenericTracker::TrackerWrapper::setClipping(const unsigned int &flags)
{
  vpMbEdgeTracker::setClipping(flags);
}

void vpMbGenericTracker::TrackerWrapper::setFarClippingDistance(const double &dist)
{
  vpMbEdgeTracker::setFarClippingDistance(dist);
}

void vpMbGenericTracker::TrackerWrapper::setNearClippingDistance(const double &dist)
{
  vpMbEdgeTracker::setNearClippingDistance(dist);
}

void vpMbGenericTracker::TrackerWrapper::setOgreVisibilityTest(const bool &v)
{
  vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("TrackerWrapper");
#endif
}

void vpMbGenericTracker::TrackerWrapper::setPose(const vpImage<unsigned char> * const I, const vpImage<vpRGBa> * const I_color,
                                                 const vpHomogeneousMatrix &cdMo)
{
  bool performKltSetPose = false;
  if (I_color) {
    vpImageConvert::convert(*I_color, m_I);
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    performKltSetPose = true;

    if (useScanLine || clippingFlag > 3) {
      cam.computeFov(I ? I->getWidth() : m_I.getWidth(), I ? I->getHeight() : m_I.getHeight());
    }

    vpMbKltTracker::setPose(I ? *I : m_I, cdMo);
  }
#endif

  if (!performKltSetPose) {
    cMo = cdMo;
    init(I ? *I : m_I);
    return;
  }

  if (m_trackerType & EDGE_TRACKER)
    resetMovingEdge();

  if (useScanLine) {
    faces.computeClippedPolygons(cMo, cam);
    faces.computeScanLineRender(cam, I ? I->getWidth() : m_I.getWidth(), I ? I->getHeight() : m_I.getHeight());
  }

#if 0
  if (m_trackerType & EDGE_TRACKER) {
    initPyramid(I, Ipyramid);

    unsigned int i = (unsigned int) scales.size();
    do {
      i--;
      if(scales[i]){
        downScale(i);
        initMovingEdge(*Ipyramid[i], cMo);
        upScale(i);
      }
    } while(i != 0);

    cleanPyramid(Ipyramid);
  }
#else
  if (m_trackerType & EDGE_TRACKER)
    initMovingEdge(I ? *I : m_I, cMo);
#endif

  // Depth normal
  vpMbDepthNormalTracker::computeVisibility(I ? I->getWidth() : m_I.getWidth(), I ? I->getHeight() : m_I.getHeight());

  // Depth dense
  vpMbDepthDenseTracker::computeVisibility(I ? I->getWidth() : m_I.getWidth(), I ? I->getHeight() : m_I.getHeight());
}

void vpMbGenericTracker::TrackerWrapper::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo)
{
  setPose(&I, NULL, cdMo);
}

void vpMbGenericTracker::TrackerWrapper::setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo)
{
  setPose(NULL, &I_color, cdMo);
}

void vpMbGenericTracker::TrackerWrapper::setProjectionErrorComputation(const bool &flag)
{
  vpMbEdgeTracker::setProjectionErrorComputation(flag);
}

void vpMbGenericTracker::TrackerWrapper::setScanLineVisibilityTest(const bool &v)
{
  vpMbEdgeTracker::setScanLineVisibilityTest(v);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::setScanLineVisibilityTest(v);
#endif
  vpMbDepthNormalTracker::setScanLineVisibilityTest(v);
  vpMbDepthDenseTracker::setScanLineVisibilityTest(v);
}

void vpMbGenericTracker::TrackerWrapper::setTrackerType(const int type)
{
  if ((type & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
               KLT_TRACKER |
#endif
               DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
    throw vpException(vpTrackingException::badValue, "bad value for tracker type: !", type);
  }

  m_trackerType = type;
}

void vpMbGenericTracker::TrackerWrapper::testTracking()
{
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::testTracking();
  }
}

void vpMbGenericTracker::TrackerWrapper::track(const vpImage<unsigned char> &
#ifdef VISP_HAVE_PCL
                                                   I
#endif
)
{
  if ((m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                        | KLT_TRACKER
#endif
                        )) == 0) {
    std::cerr << "Bad tracker type: " << m_trackerType << std::endl;
    return;
  }

#ifdef VISP_HAVE_PCL
  track(&I, nullptr);
#endif
}

void vpMbGenericTracker::TrackerWrapper::track(const vpImage<vpRGBa> &)
{
  //not exposed to the public API, only for debug
}

#ifdef VISP_HAVE_PCL
void vpMbGenericTracker::TrackerWrapper::track(const vpImage<unsigned char> *const ptr_I,
                                               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
  if ((m_trackerType & (EDGE_TRACKER |
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                        KLT_TRACKER |
#endif
                        DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER)) == 0) {
    std::cerr << "Bad tracker type: " << m_trackerType << std::endl;
    return;
  }

  if (m_trackerType & (EDGE_TRACKER
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                       | KLT_TRACKER
#endif
                       ) &&
      ptr_I == NULL) {
    throw vpException(vpException::fatalError, "Image pointer is NULL!");
  }

  if (m_trackerType & (DEPTH_NORMAL_TRACKER | DEPTH_DENSE_TRACKER) && point_cloud == nullptr) {
    throw vpException(vpException::fatalError, "Pointcloud smart pointer is NULL!");
  }

  // Back-up cMo in case of exception
  vpHomogeneousMatrix cMo_1 = cMo;
  try {
    preTracking(ptr_I, point_cloud);

    try {
      computeVVS(ptr_I);
    } catch (...) {
      covarianceMatrix = -1;
      throw; // throw the original exception
    }

    if (m_trackerType == EDGE_TRACKER)
      testTracking();

    postTracking(ptr_I, point_cloud);

  } catch (const vpException &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    cMo = cMo_1;
    throw; // rethrowing the original exception
  }
}
#endif
