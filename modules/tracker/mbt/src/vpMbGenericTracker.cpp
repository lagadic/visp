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
 * Generic model-based tracker.
 *
 *****************************************************************************/

#include <visp3/mbt/vpMbGenericTracker.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/mbt/vpMbtEdgeKltXmlParser.h>


vpMbGenericTracker::vpMbGenericTracker() :
  m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(), m_percentageGdPt(0.4),
  m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  m_mapOfTrackers["Camera"] = new TrackerWrapper(EDGE_TRACKER);

  //Add default camera transformation matrix
  m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();

  //Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif
}

vpMbGenericTracker::vpMbGenericTracker(const unsigned int nbCameras, const int trackerType) :
  m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(), m_percentageGdPt(0.4),
  m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  if (nbCameras == 0) {
    throw vpException(vpTrackingException::fatalError, "Cannot use no camera!");
  } else if(nbCameras == 1) {
    m_mapOfTrackers["Camera"] = new TrackerWrapper(trackerType);

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else {
    for(unsigned int i = 1; i <= nbCameras; i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfTrackers[ss.str()] = new TrackerWrapper(trackerType);

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfTrackers.begin()->first;
  }

  //Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif
}

vpMbGenericTracker::vpMbGenericTracker(const std::vector<int> &trackerTypes) :
  m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(), m_percentageGdPt(0.4),
  m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  if (trackerTypes.empty()) {
    throw vpException(vpException::badValue, "There is no camera!");
  }

  if (trackerTypes.size() == 1) {
    m_mapOfTrackers["Camera"] = new TrackerWrapper(trackerTypes[0]);

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else {
    for(size_t i = 1; i <= trackerTypes.size(); i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfTrackers[ss.str()] = new TrackerWrapper(trackerTypes[i-1]);

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfTrackers.begin()->first;
  }

  //Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif
}

vpMbGenericTracker::vpMbGenericTracker(const std::vector<std::string> &cameraNames, const std::vector<int> &trackerTypes) :
  m_error(), m_L(), m_mapOfCameraTransformationMatrix(), m_mapOfFeatureFactors(), m_mapOfTrackers(), m_percentageGdPt(0.4),
  m_referenceCameraName("Camera"), m_thresholdOutlier(0.5), m_w(), m_weightedError()
{
  if (cameraNames.size() != trackerTypes.size() || cameraNames.empty()) {
    throw vpException(vpTrackingException::badValue, "cameraNames.size() != trackerTypes.size() || cameraNames.empty()");
  }

  for (size_t i = 0; i < cameraNames.size(); i++) {
    m_mapOfTrackers[cameraNames[i]] = new TrackerWrapper(trackerTypes[i]);

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix[cameraNames[i]] = vpHomogeneousMatrix();
  }

  //Set by default the reference camera to the first one
  m_referenceCameraName = m_mapOfTrackers.begin()->first;

  //Add default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif
}

vpMbGenericTracker::~vpMbGenericTracker() {
  for (std::map<std::string, TrackerWrapper*>::iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    delete it->second;
    it->second = NULL;
  }
}

void vpMbGenericTracker::computeProjectionError() {
  if (computeProjError) {
    double rawTotalProjectionError = 0.0;
    unsigned int nbTotalFeaturesUsed = 0;

    for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
      TrackerWrapper *tracker = it->second;

      double curProjError = tracker->getProjectionError();
      unsigned int nbFeaturesUsed = tracker->nbFeaturesForProjErrorComputation;

      if (nbFeaturesUsed > 0) {
        nbTotalFeaturesUsed += nbFeaturesUsed;
        rawTotalProjectionError += ( vpMath::rad(curProjError)*nbFeaturesUsed );
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

void vpMbGenericTracker::computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
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

  //Covariance
  vpColVector W_true(m_error.getRows());
  vpMatrix L_true, LVJ_true;

  //Create the map of VelocityTwistMatrices
  std::map<std::string, vpVelocityTwistMatrix> mapOfVelocityTwist;
  for(std::map<std::string, vpHomogeneousMatrix>::const_iterator it = m_mapOfCameraTransformationMatrix.begin(); it != m_mapOfCameraTransformationMatrix.end(); ++it) {
    vpVelocityTwistMatrix cVo;
    cVo.buildFrom(it->second);
    mapOfVelocityTwist[it->first] = cVo;
  }

  double factorEdge = m_mapOfFeatureFactors[EDGE_TRACKER];
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  double factorKlt = m_mapOfFeatureFactors[KLT_TRACKER];
#endif

  while( std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter) ) {
    computeVVSInteractionMatrixAndResidu(mapOfImages, mapOfVelocityTwist);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error, error_prev, cMo_prev, mu, reStartFromLastIncrement);
    if (reStartFromLastIncrement) {
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;

        tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo_prev;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
        vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo_prev * tracker->c0Mo.inverse();
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
          LVJ_true = (m_L*cVo*oJo);
        }
      }

      vpVelocityTwistMatrix cVo;
      if (iter == 0) {
        isoJoIdentity_ = true;
        oJo.eye();

        // If all the 6 dof should be estimated, we check if the interaction matrix is full rank.
        // If not we remove automatically the dof that cannot be estimated
        // This is particularly useful when consering circles (rank 5) and cylinders (rank 4)
        if (isoJoIdentity_) {
          cVo.buildFrom(cMo);

          vpMatrix K; // kernel
          unsigned int rank = (m_L*cVo).kernel(K);
          if (rank == 0) {
            throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
          }

          if (rank != 6) {
            vpMatrix I; // Identity
            I.eye(6);
            oJo = I-K.AtA();

            isoJoIdentity_ = false;
          }
        }
      }

      //Weighting
      double wi;
      double num = 0;
      double den = 0;

      unsigned int start_index = 0;
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;

        if (tracker->m_trackerType & EDGE_TRACKER) {
          for (unsigned int i = 0; i < tracker->m_error_edge.getRows(); i++) {
            wi = tracker->m_w_edge[i] * tracker->m_factor[i] * factorEdge;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi*vpMath::sqr(m_error[start_index + i]);
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
            wi = tracker->m_w_klt[i] * factorKlt;
            W_true[start_index + i] = wi;
            m_weightedError[start_index + i] = wi * m_error[start_index + i];

            num += wi*vpMath::sqr(m_error[start_index + i]);
            den += wi;

            for (unsigned int j = 0; j < m_L.getCols(); j++) {
              m_L[start_index + i][j] *= wi;
            }
          }

          start_index += tracker->m_error_klt.getRows();
        }
#endif
      }

      normRes_1 = normRes;
      normRes = sqrt(num/den);

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L, LTL, m_weightedError, m_error, error_prev, LTR, mu, v);

      cMo_prev = cMo;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;

        vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * tracker->c0Mo.inverse();
        tracker->ctTc0 = c_curr_tTc_curr0;
      }
#endif

      //Update cMo
      for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
        TrackerWrapper *tracker = it->second;
        tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
      }
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMo_prev, L_true, LVJ_true, m_error);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if (tracker->m_trackerType & EDGE_TRACKER) {
      tracker->updateMovingEdgeWeights();
    }
  }
}

void vpMbGenericTracker::computeVVSInit() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::computeVVSInit() should not be called!");
}

void vpMbGenericTracker::computeVVSInit(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  unsigned int nbFeatures = 0;

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->computeVVSInit(*mapOfImages[it->first]);

    nbFeatures += tracker->m_error.getRows();
  }

  m_L.resize(nbFeatures, 6, false);
  m_error.resize(nbFeatures, false);

  m_weightedError.resize(nbFeatures, false);
  m_w.resize(nbFeatures, false);
  m_w = 1;
}

void vpMbGenericTracker::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::computeVVSInteractionMatrixAndResidu() should not be called");
}

void vpMbGenericTracker::computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                             std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist) {
  unsigned int start_index = 0;

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    tracker->cMo = m_mapOfCameraTransformationMatrix[it->first]*cMo;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * tracker->c0Mo.inverse();
    tracker->ctTc0 = c_curr_tTc_curr0;
#endif
    tracker->computeVVSInteractionMatrixAndResidu(*mapOfImages[it->first]);

    m_L.insert(tracker->m_L*mapOfVelocityTwist[it->first], start_index, 0);
    m_error.insert(start_index, tracker->m_error);

    start_index += tracker->m_error.getRows();
  }
}

void vpMbGenericTracker::computeVVSWeights() {
  unsigned int start_index = 0;

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).

  \note This function will display the model only for the reference camera.
*/
void vpMbGenericTracker::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &cam_,
                                const vpColor &col , const unsigned int thickness, const bool displayFullModel) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
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
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).

  \note This function will display the model only for the reference camera.
*/
void vpMbGenericTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &cam_,
                                const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
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
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::display(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                               const vpCameraParameters &cam1, const vpCameraParameters &cam2, const vpColor &color, const unsigned int thickness, const bool displayFullModel) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
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
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::display(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                                 const vpCameraParameters &cam1, const vpCameraParameters &cam2, const vpColor &color, const unsigned int thickness, const bool displayFullModel) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param mapOfImages : Map of grayscale images.
  \param mapOfCameraPoses : Map of camera poses.
  \param mapOfCameraParameters : Map of camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbGenericTracker::display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                 const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                                 const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                                 const vpColor& col, const unsigned int thickness, const bool displayFullModel) {
  //Display only for the given images
  for (std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.begin(); it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if (it_tracker != m_mapOfTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
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
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbGenericTracker::display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
                                 const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                                 const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
                                 const vpColor& col, const unsigned int thickness, const bool displayFullModel) {
  //Display only for the given images
  for (std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img = mapOfImages.begin(); it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if (it_tracker != m_mapOfTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
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
std::vector<std::string> vpMbGenericTracker::getCameraNames() const {
  std::vector<std::string> cameraNames;

  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
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
void vpMbGenericTracker::getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getCameraParameters(cam1);
    ++it;

    it->second->getCameraParameters(cam2);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

/*!
  Get all the camera parameters.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbGenericTracker::getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const {
  //Clear the input map
  mapOfCameraParameters.clear();

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
std::map<std::string, int> vpMbGenericTracker::getCameraTrackerTypes() const {
  std::map<std::string, int> trackingTypes;

  TrackerWrapper *traker;
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
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
void vpMbGenericTracker::getClipping(unsigned int &clippingFlag1, unsigned int &clippingFlag2) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    clippingFlag1 = it->second->getClipping();
    ++it;

    clippingFlag2 = it->second->getClipping();
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

/*!
  Get the clipping used and defined in vpPolygon3D::vpMbtPolygonClippingType.

  \param mapOfClippingFlags : Map of clipping flags.
*/
void vpMbGenericTracker::getClipping(std::map<std::string, unsigned int> &mapOfClippingFlags) const {
  mapOfClippingFlags.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfClippingFlags[it->first] = tracker->getClipping();
  }
}

/*!
  Return a reference to the faces structure.

  \return Reference to the face structure.
*/
vpMbHiddenFaces<vpMbtPolygon>& vpMbGenericTracker::getFaces() {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
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
vpMbHiddenFaces<vpMbtPolygon>& vpMbGenericTracker::getFaces(const std::string &cameraName) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(cameraName);
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
std::list<vpMbtDistanceCircle*>& vpMbGenericTracker::getFeaturesCircle() {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getFeaturesCircle();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera:  %s!", m_referenceCameraName);
  }
}

/*!
  Return the address of the cylinder feature list for the reference camera.
*/
std::list<vpMbtDistanceKltCylinder*>& vpMbGenericTracker::getFeaturesKltCylinder() {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getFeaturesKltCylinder();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera:  %s!", m_referenceCameraName);
  }
}

/*!
  Return the address of the Klt feature list for the reference camera.
*/
std::list<vpMbtDistanceKltPoints*>& vpMbGenericTracker::getFeaturesKlt() {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getFeaturesKlt();
  } else {
    throw vpException(vpTrackingException::badValue, "Cannot find the reference camera:  %s!", m_referenceCameraName);
  }
}
#endif

/*!
   \return The threshold value between 0 and 1 over good moving edges ratio. It allows to
   decide if the tracker has enough valid moving edges to compute a pose. 1 means that all
   moving edges should be considered as good to have a valid pose, while 0.1 means that
   10% of the moving edge are enough to declare a pose valid.

   \sa setGoodMovingEdgesRatioThreshold()
*/
double vpMbGenericTracker::getGoodMovingEdgesRatioThreshold() const {
  return m_percentageGdPt;
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Get the klt tracker at the current state for the reference camera.

  \return klt tracker.
*/
vpKltOpencv vpMbGenericTracker::getKltOpencv() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);

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
void vpMbGenericTracker::getKltOpencv(vpKltOpencv &klt1, vpKltOpencv &klt2) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    klt1 = it->second->getKltOpencv();
    ++it;

    klt2 = it->second->getKltOpencv();
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

/*!
  Get the klt tracker at the current state.

  \param mapOfKlts : Map if klt trackers.
*/
void vpMbGenericTracker::getKltOpencv(std::map<std::string, vpKltOpencv> &mapOfKlts) const {
  mapOfKlts.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfKlts[it->first] = tracker->getKltOpencv();
  }
}

/*!
  Get the current list of KLT points for the reference camera.

   \return the list of KLT points through vpKltOpencv.
*/
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
std::vector<cv::Point2f> vpMbGenericTracker::getKltPoints() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltPoints();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return std::vector<cv::Point2f>();
}

/*!
  Get the threshold for the acceptation of a point.

  \return threshold_outlier : Threshold for the weight below which a point is rejected.
*/
double vpMbGenericTracker::getKltThresholdAcceptation() const {
  return m_thresholdOutlier;
}
#endif

/*!
  Get the current list of KLT points for the reference camera.

  \warning This function convert and copy the OpenCV KLT points into vpImagePoints.

  \return the list of KLT points through vpKltOpencv.
*/
std::vector<vpImagePoint> vpMbGenericTracker::getKltImagePoints() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
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

  \warning This function convert and copy the openCV KLT points into vpImagePoints.

  \return the list of KLT points and their id through vpKltOpencv.
*/
std::map<int, vpImagePoint> vpMbGenericTracker::getKltImagePointsWithId() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltImagePointsWithId();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return std::map<int, vpImagePoint>();
}
#endif

/*!
  Get the list of the circles tracked for the specified level. Each circle
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceCircle.
  \param circlesList : The list of the circles of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLcircle(const std::string &cameraName, std::list<vpMbtDistanceCircle *>& circlesList, const unsigned int level) const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLcircle(circlesList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceCylinder.
  \param cylindersList : The list of the cylinders of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLcylinder(const std::string &cameraName, std::list<vpMbtDistanceCylinder *>& cylindersList, const unsigned int level) const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLcylinder(cylindersList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  }
}

/*!
  Get the list of the lines tracked for the specified level. Each line contains
  the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceLine.
  \param linesList : The list of the lines of the model.
  \param level : Level corresponding to the list to return.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getLline(const std::string &cameraName, std::list<vpMbtDistanceLine *>& linesList, const unsigned int level) const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    it->second->getLline(linesList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  }
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Get the erosion of the mask used on the Model faces.

  \return The erosion for the reference camera.
*/
unsigned int vpMbGenericTracker::getKltMaskBorder() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltMaskBorder();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return 0;
}
#endif

/*!
  Get the moving edge parameters for the reference camera.

  \return an instance of the moving edge parameters used by the tracker.
*/
vpMe vpMbGenericTracker::getMovingEdge() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

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
void vpMbGenericTracker::getMovingEdge(vpMe &me1, vpMe &me2) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getMovingEdge(me1);
    ++it;

    it->second->getMovingEdge(me2);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
int vpMbGenericTracker::getKltNbPoints() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    return tracker->getKltNbPoints();
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return 0;
}
#endif

/*!
  Get the moving edge parameters for all the cameras

  \param mapOfMovingEdges : Map of moving edge parameters for all the cameras.
*/
void vpMbGenericTracker::getMovingEdge(std::map<std::string, vpMe> &mapOfMovingEdges) const {
  mapOfMovingEdges.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
unsigned int vpMbGenericTracker::getNbPoints(const unsigned int level) const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);

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

  \param mapOfNbPoints : Map of number of good points (vpMeSite) tracked for all the cameras.
  \param level : Pyramid level to consider.

  \exception vpException::dimensionError if level does not represent a used
  level.

  \note Multi-scale moving edge tracking is not possible, scale level=0 must be used.
*/
void vpMbGenericTracker::getNbPoints(std::map<std::string, unsigned int> &mapOfNbPoints, const unsigned int level) const {
  mapOfNbPoints.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfNbPoints[it->first] = tracker->getNbPoints(level);
  }
}

/*!
  Get the number of polygons (faces) representing the object to track.

  \return Number of polygons for the reference camera.
*/
unsigned int vpMbGenericTracker::getNbPolygon() const {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
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
void vpMbGenericTracker::getNbPolygon(std::map<std::string, unsigned int> &mapOfNbPolygons) const {
  mapOfNbPolygons.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    mapOfNbPolygons[it->first] = tracker->getNbPolygon();
  }
}

/*!
  Return the polygon (face) "index" for the reference camera.

  \exception vpException::dimensionError if index does not represent a good
  polygon.

  \param index : Index of the polygon to return.
  \return Pointer to the polygon index for the reference camera or NULL in case of problem.
*/
vpMbtPolygon* vpMbGenericTracker::getPolygon(const unsigned int index) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
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
  \return Pointer to the polygon index for the specified camera or NULL in case of problem.
*/
vpMbtPolygon* vpMbGenericTracker::getPolygon(const std::string &cameraName, const unsigned int index) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(cameraName);
  if (it != m_mapOfTrackers.end()) {
    return it->second->getPolygon(index);
  }

  std::cerr << "The camera: " << cameraName << " does not exist!" << std::endl;
  return NULL;
}

/*!
  Get the list of polygons faces (a vpPolygon representing the projection of the face in the image and a list of face corners
  in 3D), with the possibility to order by distance to the camera or to use the visibility check to consider if the polygon
  face must be retrieved or not.

  \param orderPolygons : If true, the resulting list is ordered from the nearest polygon faces to the farther.
  \param useVisibility : If true, only visible faces will be retrieved.
  \param clipPolygon : If true, the polygons will be clipped according to the clipping flags set in vpMbTracker.
  \return A pair object containing the list of vpPolygon and the list of face corners.

  \note This function will return the 2D polygons faces and 3D face points only for the reference camera.
*/
std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > vpMbGenericTracker::getPolygonFaces(const bool orderPolygons, const bool useVisibility, const bool clipPolygon) {
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > polygonFaces;

  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    polygonFaces = tracker->getPolygonFaces(orderPolygons, useVisibility, clipPolygon);
  } else {
    std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << "!" << std::endl;
  }

  return polygonFaces;
}

/*!
  Get the list of polygons faces (a vpPolygon representing the projection of the face in the image and a list of face corners
  in 3D), with the possibility to order by distance to the camera or to use the visibility check to consider if the polygon
  face must be retrieved or not.

  \param mapOfPolygons : Map of 2D polygon faces.
  \param mapOfPoints : Map of face 3D points.
  \param orderPolygons : If true, the resulting list is ordered from the nearest polygon faces to the farther.
  \param useVisibility : If true, only visible faces will be retrieved.
  \param clipPolygon : If true, the polygons will be clipped according to the clipping flags set in vpMbTracker.
  \return A pair object containing the list of vpPolygon and the list of face corners.

  \note This function will return the 2D polygons faces and 3D face points only for all the cameras.
*/
void vpMbGenericTracker::getPolygonFaces(std::map<std::string, std::vector<vpPolygon> > &mapOfPolygons, std::map<std::string, std::vector<std::vector<vpPoint> > > &mapOfPoints,
                                         const bool orderPolygons, const bool useVisibility, const bool clipPolygon) {
  mapOfPolygons.clear();
  mapOfPoints.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > polygonFaces = tracker->getPolygonFaces(orderPolygons, useVisibility, clipPolygon);

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
void vpMbGenericTracker::getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->getPose(c1Mo);
    ++it;

    it->second->getPose(c2Mo);
  } else {
    std::cerr << "The tracker is not set as a stereo configuration! There are "
              << m_mapOfTrackers.size() << " cameras!" << std::endl;
  }
}

/*!
  Get the current pose between the object and the cameras.

  \param mapOfCameraPoses : The map of camera poses for all the cameras.
*/
void vpMbGenericTracker::getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const {
  //Clear the map
  mapOfCameraPoses.clear();

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->getPose(mapOfCameraPoses[it->first]);
  }
}

void vpMbGenericTracker::init(const vpImage<unsigned char>& I) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
    tracker->init(I);
  }
}

void vpMbGenericTracker::initCircle(const vpPoint& /*p1*/, const vpPoint &/*p2*/, const vpPoint &/*p3*/, const double /*radius*/, const int /*idFace*/, const std::string &/*name*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initCircle() should not be called!");
}

#ifdef VISP_HAVE_MODULE_GUI

/*!
  Initialise the tracker by clicking in the reference image on the pixels that correspond to the
  3D points whose coordinates are extracted from a file. In this file, comments starting
  with # character are allowed. Notice that 3D point coordinates are expressed in meter
  in the object frame with their X, Y and Z values.

  The structure of this file is the following:

  \code
  # 3D point coordinates
  4                 # Number of points in the file (minimum is four)
  0.01 0.01 0.01    # \
  ...               #  | 3D coordinates in the object frame (X, Y, Z)
  0.01 -0.01 -0.01  # /
  \endcode

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param initFile1 : File containing the coordinates of at least 4 3D points the user has
  to click in the image acquired by the first camera. This file should have .init extension (ie teabox.init).
  \param initFile2 : File containing the coordinates of at least 4 3D points the user has
  to click in the image acquired by the second camera. This file should have .init extension.
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                   const std::string &initFile1, const std::string &initFile2, const bool displayHelp) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initClick(I1, initFile1, displayHelp);

    ++it;

    tracker = it->second;
    tracker->initClick(I2, initFile2, displayHelp);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      //Set the reference cMo
      tracker->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick()! Require two cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracker by clicking in the reference image on the pixels that correspond to the
  3D points whose coordinates are extracted from a file. In this file, comments starting
  with # character are allowed. Notice that 3D point coordinates are expressed in meter
  in the object frame with their X, Y and Z values.

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

  \param mapOfImages : Map of images.
  \param mapOfInitFiles : Map of files containing the points where to click for each camera.
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.

  \note Image and init file must be supplied for the reference camera. The images for all the cameras must
  be supplied to correctly initialize the trackers but some init files can be omitted. In this case,
  they will be initialized using the pose computed from the reference camera pose and using the known geometric
  transformation between each camera (see setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                   const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp) {
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initFile = mapOfInitFiles.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick for the reference camera!");
  }

  //Vector of missing initFile for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_initFile = mapOfInitFiles.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
        //InitClick for the current camera
        TrackerWrapper *tracker = it_tracker->second;
        tracker->initClick(*it_img->second, it_initFile->second, displayHelp);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  //Init for cameras that do not have an initFile
  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->cMo = cCurrentMo;
      m_mapOfTrackers[*it]->init(*it_img->second);
    } else {
      throw vpException(vpTrackingException::initializationError, "Missing image or missing camera transformation matrix! Cannot set the pose for camera: %s!", *it);
    }
  }
}
#endif

void vpMbGenericTracker::initCylinder(const vpPoint& /*p1*/, const vpPoint &/*p2*/, const double /*radius*/, const int /*idFace*/, const std::string &/*name*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initCylinder() should not be called!");
}

void vpMbGenericTracker::initFaceFromCorners(vpMbtPolygon &/*polygon*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initFaceFromCorners() should not be called!");
}

void vpMbGenericTracker::initFaceFromLines(vpMbtPolygon &/*polygon*/) {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::initFaceFromLines() should not be called!");
}

/*!
  Initialise the tracker by reading 3D point coordinates and the corresponding 2D image point coordinates
  from a file. Comments starting with # character are allowed.
  3D point coordinates are expressed in meter in the object frame with X, Y and Z values.
  2D point coordinates are expressied in pixel coordinates, with first the line and then the column of the pixel in the image.
  The structure of this file is the following.
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

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param initFile1 : Path to the file containing all the points for the first camera.
  \param initFile2 : Path to the file containing all the points for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPoints(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const std::string &initFile1, const std::string &initFile2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initFromPoints(I1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPoints(I2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      //Set the reference cMo
      tracker->getPose(cMo);

      //Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPoints()! Require two cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

void vpMbGenericTracker::initFromPoints(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, std::string> &mapOfInitPoints) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPoints = mapOfInitPoints.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initPoints != mapOfInitPoints.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPoints(*it_img->second, it_initPoints->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPoints() for the reference camera!");
  }

  //Vector of missing initPoints for cameras
  std::vector<std::string> vectorOfMissingCameraPoints;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_initPoints = mapOfInitPoints.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_initPoints != mapOfInitPoints.end()) {
      //Set pose
      it_tracker->second->initFromPoints(*it_img->second, it_initPoints->second);
    } else {
      vectorOfMissingCameraPoints.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoints.begin(); it != vectorOfMissingCameraPoints.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError, "Missing image or missing camera transformation matrix! Cannot init the pose for camera: %s!", *it);
    }
  }
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read in the file initFile.

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param initFile1 : Init pose file for the first camera.
  \param initFile2 : Init pose file for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const std::string &initFile1, const std::string &initFile2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    TrackerWrapper *tracker = it->second;
    tracker->initFromPose(I1, initFile1);

    ++it;

    tracker = it->second;
    tracker->initFromPose(I2, initFile2);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      tracker = it->second;

      //Set the reference cMo
      tracker->getPose(cMo);

      //Set the reference camera parameters
      tracker->getCameraParameters(cam);
    }
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPose()! Require two cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read in the file initFile.

  \param mapOfImages : Map of images.
  \param mapOfInitPoses : Map of init pose files.

  \note Image and init pose file must be supplied for the reference camera. The images for all the cameras must
  be supplied to correctly initialize the trackers but some init pose files can be omitted. In this case,
  they will be initialized using the pose computed from the reference camera pose and using the known geometric
  transformation between each camera (see setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, std::string> &mapOfInitPoses) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initPose = mapOfInitPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_initPose != mapOfInitPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_initPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initFromPose() for the reference camera!");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_initPose = mapOfInitPoses.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_initPose != mapOfInitPoses.end()) {
      //Set pose
      it_tracker->second->initFromPose(*it_img->second, it_initPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError, "Missing image or missing camera transformation matrix! Cannot init the pose for camera: %s!", *it);
    }
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param c1Mo : Pose matrix for the first camera.
  \param c2Mo : Pose matrix for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->initFromPose(I1, c1Mo);

    ++it;

    it->second->initFromPose(I2, c2Mo);

    this->cMo = c1Mo;
  } else {
    throw vpException(vpTrackingException::initializationError, "This method requires 2 cameras but there are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param mapOfImages : Map of images.
  \param mapOfCameraPoses : Map of pose matrix.

  \note Image and camera pose must be supplied for the reference camera. The images for all the cameras must
  be supplied to correctly initialize the trackers but some camera poses can be omitted. In this case,
  they will be initialized using the pose computed from the reference camera pose and using the known geometric
  transformation between each camera (see setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->initFromPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot set pose for the reference camera!");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    it_img = mapOfImages.find(it_tracker->first);
    it_camPose = mapOfCameraPoses.find(it_tracker->first);

    if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
      //Set pose
      it_tracker->second->initFromPose(*it_img->second, it_camPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_tracker->first);
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::initializationError, "Missing image or missing camera transformation matrix! Cannot set the pose for camera: %s!", *it);
    }
  }
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the objects: tracking parameters, camera intrinsic parameters.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data).

  \param configFile : full name of the xml file.

  \sa vpXmlParser::cleanup()
*/
void vpMbGenericTracker::loadConfigFile(const std::string& configFile) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->loadConfigFile(configFile);
  }

  if (m_mapOfTrackers.find(m_referenceCameraName) == m_mapOfTrackers.end()) {
    throw vpException(vpException::fatalError, "Cannot find the reference camera:  %s!", m_referenceCameraName);
  }

  m_mapOfTrackers[m_referenceCameraName]->getCameraParameters(this->cam);
  this->angleAppears = m_mapOfTrackers[m_referenceCameraName]->getAngleAppear();
  this->angleDisappears = m_mapOfTrackers[m_referenceCameraName]->getAngleDisappear();
  this->clippingFlag = m_mapOfTrackers[m_referenceCameraName]->getClipping();
}

/*!
  Load the xml configuration files.
  From the configuration file initialize the parameters corresponding to the objects: tracking parameters, camera intrinsic parameters.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param configFile1 : Full name of the xml file for the first camera.
  \param configFile2 : Full name of the xml file for the second camera.

  \sa vpXmlParser::cleanup()

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::loadConfigFile(const std::string& configFile1, const std::string& configFile2) {
  if (m_mapOfTrackers.size() != 2) {
    throw vpException(vpException::fatalError, "The tracker is not set in a stereo configuration!");
  }

  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin();
  TrackerWrapper *tracker = it_tracker->second;
  tracker->loadConfigFile(configFile1);

  ++it_tracker;
  tracker = it_tracker->second;
  tracker->loadConfigFile(configFile2);

  if (m_mapOfTrackers.find(m_referenceCameraName) == m_mapOfTrackers.end()) {
    throw vpException(vpException::fatalError, "Cannot find the reference camera:  %s!", m_referenceCameraName);
  }

  m_mapOfTrackers[m_referenceCameraName]->getCameraParameters(this->cam);
  this->angleAppears = m_mapOfTrackers[m_referenceCameraName]->getAngleAppear();
  this->angleDisappears = m_mapOfTrackers[m_referenceCameraName]->getAngleDisappear();
  this->clippingFlag = m_mapOfTrackers[m_referenceCameraName]->getClipping();
}

/*!
  Load the xml configuration files.
  From the configuration file initialize the parameters corresponding to the objects: tracking parameters, camera intrinsic parameters.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param mapOfConfigFiles : Map of xml files.

  \sa loadConfigFile(const std::string &), vpXmlParser::cleanup()

  \note Configuration files must be supplied for all the cameras.
*/
void vpMbGenericTracker::loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    TrackerWrapper *tracker = it_tracker->second;

    std::map<std::string, std::string>::const_iterator it_config = mapOfConfigFiles.find(it_tracker->first);
    if (it_config != mapOfConfigFiles.end()) {
      tracker->loadConfigFile(it_config->second);
    } else {
      throw vpException(vpTrackingException::initializationError, "Missing configuration file for camera: %s!", it_tracker->first);
    }
  }

  //Set the reference camera parameters
  std::map<std::string, TrackerWrapper*>::iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    tracker->getCameraParameters(cam);

    //Set clipping
    this->clippingFlag = tracker->getClipping();
    this->angleAppears = tracker->getAngleAppear();
    this->angleDisappears = tracker->getAngleDisappear();
  } else {
    throw vpException(vpTrackingException::initializationError, "The reference camera: %s does not exist!", m_referenceCameraName);
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
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao.

  \param modelFile : the file containing the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.

  \note All the trackers will use the same model in case of stereo / multiple cameras configuration.
*/
void vpMbGenericTracker::loadModel(const std::string &modelFile, const bool verbose) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->loadModel(modelFile, verbose);
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
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao.

  \param modelFile1 : the file containing the 3D model description for the first camera.
  The extension of this file is either .wrl or .cao.
  \param modelFile2 : the file containing the the 3D model description for the second camera.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::loadModel(const std::string &modelFile1, const std::string &modelFile2, const bool verbose) {
  if (m_mapOfTrackers.size() != 2) {
    throw vpException(vpException::fatalError, "The tracker is not set in a stereo configuration!");
  }

  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin();
  TrackerWrapper *tracker = it_tracker->second;
  tracker->loadModel(modelFile1, verbose);

  ++it_tracker;
  tracker = it_tracker->second;
  tracker->loadModel(modelFile2, verbose);
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
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
  SoDB::finish();
#endif
}
  \endcode

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao.

  \param mapOfModelFiles : map of files containing the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.

  \note Each camera must have a model file.
*/
void vpMbGenericTracker::loadModel(const std::map<std::string, std::string> &mapOfModelFiles, const bool verbose) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    std::map<std::string, std::string>::const_iterator it_model = mapOfModelFiles.find(it_tracker->first);

    if (it_model != mapOfModelFiles.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->loadModel(it_model->second, verbose);
    } else {
      throw vpException(vpTrackingException::initializationError, "Cannot load model for camera: %s", it_tracker->first);
    }
  }
}

void vpMbGenericTracker::preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->preTracking(*mapOfImages[it->first]);
  }
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new model.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbGenericTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose) {
  if (m_mapOfTrackers.size() != 1) {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly one camera, there are %d cameras!", m_mapOfTrackers.size());
  }

  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  if (it_tracker != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->reInitModel(I, cad_name, cMo_, verbose);

    //Set reference pose
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel() the reference camera!");
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param I1 : The image containing the object to initialize for the first camera.
  \param I2 : The image containing the object to initialize for the second camera.
  \param cad_name1 : Path to the file containing the 3D model description for the first camera.
  \param cad_name2 : Path to the file containing the 3D model description for the second camera.
  \param c1Mo : The new vpHomogeneousMatrix between the first camera and the new model.
  \param c2Mo : The new vpHomogeneousMatrix between the second camera and the new model.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const std::string &cad_name1, const std::string &cad_name2,
                                   const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const bool verbose) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin();

    it_tracker->second->reInitModel(I1, cad_name1, c1Mo, verbose);

    ++it_tracker;

    it_tracker->second->reInitModel(I2, cad_name2, c2Mo, verbose);

    it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
    if (it_tracker != m_mapOfTrackers.end()) {
      //Set reference pose
      it_tracker->second->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly two cameras!");
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param mapOfImages : Map of images.
  \param mapOfModelFiles : Map of model files.
  \param mapOfCameraPoses : The new vpHomogeneousMatrix between the cameras and the current object position.
  \param verbose : Verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbGenericTracker::reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, std::string> &mapOfModelFiles,
                                     const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses, const bool verbose) {
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_model = mapOfModelFiles.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_model != mapOfModelFiles.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->reInitModel(*it_img->second, it_model->second, it_camPose->second, verbose);

    //Set reference pose
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

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameras.begin(); it != vectorOfMissingCameras.end(); ++it) {
    it_img = mapOfImages.find(*it);
    it_model = mapOfModelFiles.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_model != mapOfModelFiles.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
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
void vpMbGenericTracker::resetTracker() {
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

  //Only for Edge
  m_percentageGdPt = 0.4;

  //Only for KLT
  m_thresholdOutlier = 0.5;

  //Reset default ponderation between each feature type
  m_mapOfFeatureFactors[EDGE_TRACKER] = 1.0;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  m_mapOfFeatureFactors[KLT_TRACKER] = 1.0;
#endif

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setAngleAppear(const double &a) {
  vpMbTracker::setAngleAppear(a);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setAngleAppear(const double &a1, const double &a2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
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
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
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
void vpMbGenericTracker::setAngleAppear(const std::map<std::string, double> &mapOfAngles) {
  for (std::map<std::string, double>::const_iterator it = mapOfAngles.begin(); it != mapOfAngles.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
void vpMbGenericTracker::setAngleDisappear(const double &a) {
  vpMbTracker::setAngleDisappear(a);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setAngleDisappear(const double &a1, const double &a2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
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
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
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
void vpMbGenericTracker::setAngleDisappear(const std::map<std::string, double> &mapOfAngles) {
  for (std::map<std::string, double>::const_iterator it = mapOfAngles.begin(); it != mapOfAngles.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
void vpMbGenericTracker::setCameraParameters(const vpCameraParameters &camera) {
  vpMbTracker::setCameraParameters(camera);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setCameraParameters(const vpCameraParameters &camera1, const vpCameraParameters &camera2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
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
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the camera parameters.

  \param mapOfCameraParameters : map of new camera parameters.

  \note This function will set the camera parameters only for the supplied camera names.
*/
void vpMbGenericTracker::setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters) {
  for (std::map<std::string, vpCameraParameters>::const_iterator it = mapOfCameraParameters.begin(); it != mapOfCameraParameters.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
  Set the camera transformation matrix for the specified camera (\f$ _{}^{c_{current}}\textrm{M}_{c_{reference}} \f$).

  \param cameraName : Camera name.
  \param cameraTransformationMatrix : Camera transformation matrix between the current and the reference camera.
*/
void vpMbGenericTracker::setCameraTransformationMatrix(const std::string &cameraName, const vpHomogeneousMatrix &cameraTransformationMatrix) {
  std::map<std::string, vpHomogeneousMatrix>::iterator it = m_mapOfCameraTransformationMatrix.find(cameraName);

  if (it != m_mapOfCameraTransformationMatrix.end()) {
    it->second = cameraTransformationMatrix;
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot find camera: %s!", cameraName);
  }
}

/*!
  Set the map of camera transformation matrices
  (\f$ _{}^{c_1}\textrm{M}_{c_1}, _{}^{c_2}\textrm{M}_{c_1}, _{}^{c_3}\textrm{M}_{c_1}, \cdots, _{}^{c_n}\textrm{M}_{c_1} \f$).

  \param mapOfTransformationMatrix : map of camera transformation matrices.

  \note The transformation matrices for all the cameras are required.
*/
void vpMbGenericTracker::setCameraTransformationMatrix(const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix) {
  //Check if all the cameras have a transformation matrix
  for (std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = mapOfTransformationMatrix.find(it_tracker->first);

    if (it_camTrans == mapOfTransformationMatrix.end()) {
      throw vpException(vpTrackingException::initializationError, "Missing transformation matrix for camera: %s", it_tracker->first);
    }
  }

  m_mapOfCameraTransformationMatrix = mapOfTransformationMatrix;
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags : New clipping flags.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setClipping(const unsigned int &flags) {
  vpMbTracker::setClipping(flags);

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setClipping(const unsigned int &flags1, const unsigned int &flags2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
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
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param mapOfClippingFlags : Map of new clipping flags.
*/
void vpMbGenericTracker::setClipping(const std::map<std::string, unsigned int> &mapOfClippingFlags) {
  for (std::map<std::string, unsigned int>::const_iterator it = mapOfClippingFlags.begin(); it != mapOfClippingFlags.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
  Enable to display the features. By features, we meant the moving edges (ME) and the klt points if used.

  Note that if present, the moving edges can be displayed with different colors:
  - If green : The ME is a good point.
  - If blue : The ME is removed because of a contrast problem during the tracking phase.
  - If purple : The ME is removed because of a threshold problem during the tracking phase.
  - If red : The ME is removed because it is rejected by the robust approach in the virtual visual servoing scheme.

  \param displayF : set it to true to display the features.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setDisplayFeatures(const bool displayF) {
  vpMbTracker::setDisplayFeatures(displayF);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setDisplayFeatures(displayF);
  }
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setFarClippingDistance(const double &dist) {
  vpMbTracker::setFarClippingDistance(dist);

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setFarClippingDistance(const double &dist1, const double &dist2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
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
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the far distance for clipping.

  \param mapOfClippingDists : Map of far clipping values.
*/
void vpMbGenericTracker::setFarClippingDistance(const std::map<std::string, double> &mapOfClippingDists) {
  for (std::map<std::string, double>::const_iterator it = mapOfClippingDists.begin(); it != mapOfClippingDists.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
  Set the feature factors used in the VVS stage (ponderation between the feature types).

  \param mapOfFeatureFactors : Map of feature factors.
*/
void vpMbGenericTracker::setFeatureFactors(const std::map<vpTrackerType, double> &mapOfFeatureFactors) {
  for (std::map<vpTrackerType, double>::iterator it = m_mapOfFeatureFactors.begin(); it != m_mapOfFeatureFactors.end(); ++it) {
    std::map<vpTrackerType, double>::const_iterator it_factor = mapOfFeatureFactors.find(it->first);
    if (it_factor != mapOfFeatureFactors.end()) {
      it->second = it_factor->second;
    }
  }
}

/*!
   Set the threshold value between 0 and 1 over good moving edges ratio. It allows to
   decide if the tracker has enough valid moving edges to compute a pose. 1 means that all
   moving edges should be considered as good to have a valid pose, while 0.1 means that
   10% of the moving edge are enough to declare a pose valid.

   \param threshold : Value between 0 and 1 that corresponds to the ratio of good
   moving edges that is necessary to consider that the estimated pose is valid.
   Default value is 0.4.

   \sa getGoodMovingEdgesRatioThreshold()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setGoodMovingEdgesRatioThreshold(const double threshold) {
  m_percentageGdPt = threshold;

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setGoodMovingEdgesRatioThreshold(threshold);
  }
}

#ifdef VISP_HAVE_OGRE
/*!
  Set the ratio of visibility attempts that has to be successful to consider a polygon as visible.

  \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

  \param ratio : Ratio of succesful attempts that has to be considered. Value has to be between 0.0 (0%) and 1.0 (100%).

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setGoodNbRayCastingAttemptsRatio(const double &ratio) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setGoodNbRayCastingAttemptsRatio(ratio);
  }
}

/*!
  Set the number of rays that will be sent toward each polygon for visibility test.
  Each ray will go from the optic center of the camera to a random point inside the considered polygon.

  \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

  \param attempts Number of rays to be sent.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setNbRayCastingAttemptsForVisibility(const unsigned int &attempts) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setKltOpencv(const vpKltOpencv &t) {
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setKltOpencv(const vpKltOpencv &t1, const vpKltOpencv &t2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setKltOpencv(t1);

    ++it;
    it->second->setKltOpencv(t2);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the new value of the klt tracker.

  \param mapOfKlts : Map of klt tracker containing the new values.
*/
void vpMbGenericTracker::setKltOpencv(const std::map<std::string, vpKltOpencv> &mapOfKlts) {
  for (std::map<std::string, vpKltOpencv>::const_iterator it = mapOfKlts.begin(); it != mapOfKlts.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
void vpMbGenericTracker::setKltThresholdAcceptation(const double th) {
  m_thresholdOutlier = th;

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setKltThresholdAcceptation(th);
  }
}
#endif

/*!
  Set the flag to consider if the level of detail (LOD) is used.

  \param useLod : true if the level of detail must be used, false otherwise. When true,
  two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
  \param name : name of the face we want to modify the LOD parameter.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setLod(const bool useLod, const std::string &name) {
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setKltMaskBorder(const unsigned int &e) {
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setKltMaskBorder(const unsigned int &e1, const unsigned int &e2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setKltMaskBorder(e1);

    ++it;

    it->second->setKltMaskBorder(e2);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the erosion of the mask used on the Model faces.

  \param mapOfErosions : Map of desired erosions.
*/
void vpMbGenericTracker::setKltMaskBorder(const std::map<std::string, unsigned int> &mapOfErosions) {
  for (std::map<std::string, unsigned int>::const_iterator it = mapOfErosions.begin(); it != mapOfErosions.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

    if (it_tracker != m_mapOfTrackers.end()) {
      TrackerWrapper *tracker = it_tracker->second;
      tracker->setKltMaskBorder(it->second);
    }
  }
}
#endif

/*!
  Set the threshold for the minimum line length to be considered as visible in the LOD case.

  \param minLineLengthThresh : threshold for the minimum line length in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinPolygonAreaThresh()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name) {
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMinLineLengthThresh(minLineLengthThresh, name);
  }
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinLineLengthThresh()

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name) {
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMinPolygonAreaThresh(minPolygonAreaThresh, name);
  }
}

/*!
  Set the moving edge parameters.

  \param me : an instance of vpMe containing all the desired parameters.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setMovingEdge(const vpMe &me) {
  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setMovingEdge(me);
  }
}

/*!
  Set the moving edge parameters.

  \param me1 : an instance of vpMe containing all the desired parameters for the first camera.
  \param me2 : an instance of vpMe containing all the desired parameters for the second camera.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setMovingEdge(const vpMe &me1, const vpMe &me2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setMovingEdge(me1);

    ++it;

    it->second->setMovingEdge(me2);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the moving edge parameters.

  \param mapOfMe : Map of vpMe containing all the desired parameters.
*/
void vpMbGenericTracker::setMovingEdge(const std::map<std::string, vpMe> &mapOfMe) {
  for (std::map<std::string, vpMe>::const_iterator it = mapOfMe.begin(); it != mapOfMe.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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
void vpMbGenericTracker::setNearClippingDistance(const double &dist) {
  vpMbTracker::setNearClippingDistance(dist);

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setNearClippingDistance(const double &dist1, const double &dist2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
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
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the near distance for clipping.

  \param mapOfDists : Map of near clipping values.
*/
void vpMbGenericTracker::setNearClippingDistance(const std::map<std::string, double> &mapOfDists) {
  for (std::map<std::string, double>::const_iterator it = mapOfDists.begin(); it != mapOfDists.end(); ++it) {
    std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(it->first);

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

  \warning This method has only effect when Ogre is used and Ogre visibility test is
  enabled using setOgreVisibilityTest() with true parameter.

  \param showConfigDialog : if true, shows Ogre dialog window (used to set Ogre
  rendering options) when Ogre visibility is enabled. By default, this functionality
  is turned off.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setOgreShowConfigDialog(const bool showConfigDialog) {
  vpMbTracker::setOgreShowConfigDialog(showConfigDialog);

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setOgreShowConfigDialog(showConfigDialog);
  }
}

/*!
  Use Ogre3D for visibility tests

  \warning This function has to be called before the initialization of the tracker.

  \param v : True to use it, False otherwise

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setOgreVisibilityTest(const bool &v) {
  vpMbTracker::setOgreVisibilityTest(v);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for (std::map<std::string, TrackerWrapper *>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
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
void vpMbGenericTracker::setOptimizationMethod(const vpMbtOptimizationMethod &opt) {
  vpMbTracker::setOptimizationMethod(opt);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setOptimizationMethod(opt);
  }
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track() function.
  This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders with the KLT tracking.

  \param I : image corresponding to the desired pose.
  \param cdMo : Pose to affect.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) {
  if (m_mapOfTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "The function setPose() requires the generic tracker to be configured with only one camera!");
  }

  cMo = cdMo;

  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(m_referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    TrackerWrapper *tracker = it->second;
    tracker->setPose(I, cdMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "The reference camera: %s does not exist!", m_referenceCameraName);
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I1 : First image corresponding to the desired pose.
  \param I2 : Second image corresponding to the desired pose.
  \param c1Mo : First pose to affect.
  \param c2Mo : Second pose to affect.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix c2Mo) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    it->second->setPose(I1, c1Mo);

    ++it;

    it->second->setPose(I2, c2Mo);

    it = m_mapOfTrackers.find(m_referenceCameraName);
    if (it != m_mapOfTrackers.end()) {
      //Set reference pose
      it->second->getPose(cMo);
    } else {
      throw vpException(vpTrackingException::fatalError, "The reference camera: %s does not exist!", m_referenceCameraName);
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images.
  \param mapOfCameraPoses : Map of pose to affect to the cameras.

  \note Image and camera pose must be supplied for the reference camera. The images for all the cameras must
  be supplied to correctly initialize the trackers but some camera poses can be omitted. In this case,
  they will be initialized using the pose computed from the reference camera pose and using the known geometric
  transformation between each camera (see setCameraTransformationMatrix()).
*/
void vpMbGenericTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, TrackerWrapper*>::const_iterator it_tracker = m_mapOfTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if (it_tracker != m_mapOfTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    TrackerWrapper *tracker = it_tracker->second;
    tracker->setPose(*it_img->second, it_camPose->second);
    tracker->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot set pose for the reference camera!");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for (it_tracker = m_mapOfTrackers.begin(); it_tracker != m_mapOfTrackers.end(); ++it_tracker) {
    if (it_tracker->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_tracker->first);
      it_camPose = mapOfCameraPoses.find(it_tracker->first);

      if (it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        //Set pose
        TrackerWrapper *tracker = it_tracker->second;
        tracker->setPose(*it_img->second, it_camPose->second);
      } else {
        vectorOfMissingCameraPoses.push_back(it_tracker->first);
      }
    }
  }

  for (std::vector<std::string>::const_iterator it = vectorOfMissingCameraPoses.begin(); it != vectorOfMissingCameraPoses.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if (it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfTrackers[*it]->setPose(*it_img->second, cCurrentMo);
    } else {
      throw vpException(vpTrackingException::fatalError, "Missing image or missing camera transformation matrix! Cannot set pose for camera: %s!", *it);
    }
  }
}

/*!
  Set if the projection error criteria has to be computed. This criteria could be used to
  detect the quality of the tracking. It computes an angle between 0 and 90 degrees that is
  available with getProjectionError(). Closer to 0 is the value, better is the tracking.

  \param flag : True if the projection error criteria has to be computed, false otherwise.

  \sa getProjectionError()

  \note Available only if the edge features are used (e.g. Edge tracking or Edge + KLT tracking).
  Otherwise, the value of 90 degrees will be returned.
*/
void vpMbGenericTracker::setProjectionErrorComputation(const bool &flag) {
  vpMbTracker::setProjectionErrorComputation(flag);

  for(std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setProjectionErrorComputation(flag);
  }
}

/*!
  Set the reference camera name.

  \param referenceCameraName : Name of the reference camera.
*/
void vpMbGenericTracker::setReferenceCameraName(const std::string &referenceCameraName) {
  std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.find(referenceCameraName);
  if (it != m_mapOfTrackers.end()) {
    m_referenceCameraName = referenceCameraName;
  } else {
    std::cerr << "The reference camera: " << referenceCameraName << " does not exist!";
  }
}

void vpMbGenericTracker::setScanLineVisibilityTest(const bool &v) {
  vpMbTracker::setScanLineVisibilityTest(v);

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setScanLineVisibilityTest(v);
  }
}

/*!
  Set the tracker type.

  \param type : Type of features to used, see vpTrackerType (e.g. vpMbGenericTracker::EDGE_TRACKER or vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER).

  \note This function will set the new parameter for all the cameras.

  \warning This function has to be called before the loading of the CAD model.
*/
void vpMbGenericTracker::setTrackerType(const int type) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setTrackerType(type);
  }
}

/*!
  Set if the polygons that have the given name have to be considered during the tracking phase.

  \param name : name of the polygon(s).
  \param useEdgeTracking : True if it has to be considered, False otherwise.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setUseEdgeTracking(name, useEdgeTracking);
  }
}

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
/*!
  Set if the polygons that have the given name have to be considered during the tracking phase.

  \param name : name of the polygon(s).
  \param useKltTracking : True if it has to be considered, False otherwise.

  \note This function will set the new parameter for all the cameras.
*/
void vpMbGenericTracker::setUseKltTracking(const std::string &name, const bool &useKltTracking) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;
    tracker->setUseKltTracking(name, useKltTracking);
  }
}
#endif

void vpMbGenericTracker::testTracking() {

}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param I : The current image.

  \note This function will track only for the reference camera.
*/
void vpMbGenericTracker::track(const vpImage<unsigned char> &I) {
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  mapOfImages[m_referenceCameraName] = &I;

  track(mapOfImages);
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param I1 : The first image.
  \param I2 : The second image.

  \note This function assumes a stereo configuration of the generic tracker.
*/
void vpMbGenericTracker::track(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2) {
  if (m_mapOfTrackers.size() == 2) {
    std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin();
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
    mapOfImages[it->first] = &I1;
    ++it;

    mapOfImages[it->first] = &I2;
    track(mapOfImages);
  } else {
    throw vpException(vpTrackingException::fatalError, "Require two cameras! There are %d cameras!", m_mapOfTrackers.size());
  }
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfImages : Map of images.
*/
void vpMbGenericTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    if ( (tracker->m_trackerType & (EDGE_TRACKER
                                #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                    | KLT_TRACKER
                                #endif
                                    )) == 0 ) {

      throw vpException(vpException::fatalError, "Bad tracker type: %d", tracker->m_trackerType);
    }

    if (tracker->m_trackerType & (EDGE_TRACKER
                              #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                                  | KLT_TRACKER
                              #endif
                                  ) && mapOfImages[it->first] == NULL) {
      throw vpException(vpException::fatalError, "Image pointer is NULL!");
    }
  }

  preTracking(mapOfImages);

  try {
    computeVVS(mapOfImages);
  } catch (...) {
    covarianceMatrix = -1;
    throw; // throw the original exception
  }

  //TODO: testTracking somewhere/needed?

  for (std::map<std::string, TrackerWrapper*>::const_iterator it = m_mapOfTrackers.begin(); it != m_mapOfTrackers.end(); ++it) {
    TrackerWrapper *tracker = it->second;

    tracker->postTracking(*mapOfImages[it->first]);
  }

  computeProjectionError();
}


/** TrackerWrapper **/
vpMbGenericTracker::TrackerWrapper::TrackerWrapper() :
  m_error(), m_L(), m_trackerType(EDGE_TRACKER), m_w(), m_weightedError()
{
  m_lambda = 1.0;
  m_maxIter = 30;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT TrackerWrapper");
#endif
}

vpMbGenericTracker::TrackerWrapper::TrackerWrapper(const int trackerType) :
  m_error(), m_L(), m_trackerType(trackerType), m_w(), m_weightedError()
{
  if ( (m_trackerType & (EDGE_TRACKER
                       #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                         | KLT_TRACKER
                       #endif
                         )) == 0 ) {
    throw vpException(vpTrackingException::badValue, "Bad value for tracker type: %d!", m_trackerType);
  }

  m_lambda = 1.0;
  m_maxIter = 30;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT TrackerWrapper");
#endif
}

vpMbGenericTracker::TrackerWrapper::~TrackerWrapper() { }

// Implemented only for debugging purposes: use TrackerWrapper as a standalone tracker
void vpMbGenericTracker::TrackerWrapper::computeVVS(const vpImage<unsigned char> &I) {
  computeVVSInit(I);

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

  vpMatrix LTL;
  vpColVector LTR, v;
  vpColVector error_prev;

  double mu = m_initialMu;
  vpHomogeneousMatrix cMo_prev;
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpHomogeneousMatrix ctTc0_Prev; //Only for KLT
#endif
  bool isoJoIdentity_ = true;

  //Covariance
  vpColVector W_true(m_error.getRows());
  vpMatrix L_true, LVJ_true;

  unsigned int nb_edge_features = m_error_edge.getRows();
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  unsigned int nb_klt_features = m_error_klt.getRows();
#endif

  while( std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter) ) {
    computeVVSInteractionMatrixAndResidu(I);

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
          LVJ_true = (m_L*cVo*oJo);
        }
      }

      vpVelocityTwistMatrix cVo;
      if (iter == 0) {
        isoJoIdentity_ = true;
        oJo.eye();

        // If all the 6 dof should be estimated, we check if the interaction matrix is full rank.
        // If not we remove automatically the dof that cannot be estimated
        // This is particularly useful when consering circles (rank 5) and cylinders (rank 4)
        if (isoJoIdentity_) {
          cVo.buildFrom(cMo);

          vpMatrix K; // kernel
          unsigned int rank = (m_L*cVo).kernel(K);
          if (rank == 0) {
            throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
          }

          if (rank != 6) {
            vpMatrix I; // Identity
            I.eye(6);
            oJo = I-K.AtA();

            isoJoIdentity_ = false;
          }
        }
      }

      //Weighting
      double wi;
      double num = 0;
      double den = 0;

      unsigned int start_index = 0;
      if (m_trackerType & EDGE_TRACKER) {
        for (unsigned int i = 0; i < nb_edge_features; i++) {
          wi = m_w_edge[i] * m_factor[i] * factorEdge;
          W_true[i] = wi;
          m_weightedError[i] = wi*m_error[i];

          num += wi*vpMath::sqr(m_error[i]);
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
          wi = m_w_klt[i] * factorKlt;
          W_true[start_index + i] = wi;
          m_weightedError[start_index + i] = wi * m_error_klt[i];

          num += wi*vpMath::sqr(m_error[start_index + i]);
          den += wi;

          for (unsigned int j = 0; j < m_L.getCols(); j++) {
            m_L[start_index + i][j] *= wi;
          }
        }

        start_index += nb_klt_features;
      }
#endif

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

      normRes = sqrt(num/den);
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMo_prev, L_true, LVJ_true, m_error);

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdgeWeights();
  }
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInit() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::TrackerWrapper::computeVVSInit() should not be called!");
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInit(const vpImage<unsigned char> &I) {
  initMbtTracking(I);

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

  m_L.resize(nbFeatures, 6, false);
  m_error.resize(nbFeatures, false);

  m_weightedError.resize(nbFeatures, false);
  m_w.resize(nbFeatures, false);
  m_w = 1;
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu() should not be called!");
}

void vpMbGenericTracker::TrackerWrapper::computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> &I) {
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu(I);
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    vpMbKltTracker::computeVVSInteractionMatrixAndResidu();
  }
#endif

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
}

void vpMbGenericTracker::TrackerWrapper::computeVVSWeights() {
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
}

void vpMbGenericTracker::TrackerWrapper::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &camera,
                             const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  if ( m_trackerType == EDGE_TRACKER ) {
    vpMbEdgeTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  } else if ( m_trackerType == KLT_TRACKER) {
    vpMbKltTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
#endif
  } else {
    if (m_trackerType & EDGE_TRACKER) {
      for (unsigned int i = 0; i < scales.size(); i += 1){
        if(scales[i]){
          for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
            (*it)->display(I,cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[scaleLevel].begin(); it!=circles[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          break; //displaying model on one scale only
        }
      }
    }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    if (m_trackerType & KLT_TRACKER) {
      vpMbtDistanceKltPoints *kltpoly;
      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
        kltpoly = *it;
        if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
            kltpoly->displayPrimitive(I);
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
        kltPolyCylinder = *it;
        if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
          kltPolyCylinder->displayPrimitive(I);
      }
    }
#endif

  #ifdef VISP_HAVE_OGRE
    if(useOgre)
      faces.displayOgre(cMo_);
  #endif
  }
}

void vpMbGenericTracker::TrackerWrapper::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &camera,
                             const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  if ( m_trackerType == EDGE_TRACKER ) {
    vpMbEdgeTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
  }
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  else if ( m_trackerType == KLT_TRACKER ) {
    vpMbKltTracker::display(I, cMo_, camera, col, thickness, displayFullModel);
  }
#endif
  else {
    if (m_trackerType & EDGE_TRACKER) {
      for (unsigned int i = 0; i < scales.size(); i += 1){
        if(scales[i]){
          for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
            (*it)->display(I,cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          for(std::list<vpMbtDistanceCircle*>::const_iterator it=circles[scaleLevel].begin(); it!=circles[scaleLevel].end(); ++it){
            (*it)->display(I, cMo_, camera, col, thickness, displayFullModel);
          }

          break; //displaying model on one scale only
        }
      }
    }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    if (m_trackerType & KLT_TRACKER) {
      vpMbtDistanceKltPoints *kltpoly;
      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
        kltpoly = *it;
        if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
            kltpoly->displayPrimitive(I);
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it=kltCylinders.begin(); it!=kltCylinders.end(); ++it){
        kltPolyCylinder = *it;
        if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
          kltPolyCylinder->displayPrimitive(I);
      }
    }
#endif

#ifdef VISP_HAVE_OGRE
    if(useOgre)
      faces.displayOgre(cMo_);
#endif
  }
}

void vpMbGenericTracker::TrackerWrapper::init(const vpImage<unsigned char>& I) {
  if(!modelInitialised){
    throw vpException(vpException::fatalError, "model not initialized");
  }

  if (clippingFlag > 2)
    cam.computeFov(I.getWidth(), I.getHeight());

  bool reInitialisation = false;
  if (!useOgre) {
    faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
  } else {
#ifdef VISP_HAVE_OGRE
    if(!faces.isOgreInitialised()){
      faces.setBackgroundSizeOgre(I.getHeight(), I.getWidth());

      faces.setOgreShowConfigDialog(ogreShowConfigDialog);
      faces.initOgre(cam);
      // Turn off Ogre config dialog display for the next call to this function
      // since settings are saved in the ogre.cfg file and used during the next
      // call
      ogreShowConfigDialog = false;
    }

    faces.setVisibleOgre(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#else
    faces.setVisible(I, cam, cMo, angleAppears, angleDisappears, reInitialisation);
#endif
  }

  if (useScanLine) {
    if (clippingFlag <= 2)
      cam.computeFov(I.getWidth(), I.getHeight());

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
    vpMbEdgeTracker::visibleFace(I, cMo, a); //should be useless, but keep it for nbvisiblepolygone

    initMovingEdge(I, cMo);
  }
}

void vpMbGenericTracker::TrackerWrapper::initCircle(const vpPoint& p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                                const int idFace, const std::string &name) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initCircle(p1, p2, p3, radius, idFace, name);
}

void vpMbGenericTracker::TrackerWrapper::initCylinder(const vpPoint& p1, const vpPoint &p2, const double radius, const int idFace,
                                  const std::string &name) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initCylinder(p1, p2, radius, idFace, name);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initCylinder(p1, p2, radius, idFace, name);
#endif
}

void vpMbGenericTracker::TrackerWrapper::initFaceFromCorners(vpMbtPolygon &polygon) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initFaceFromCorners(polygon);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initFaceFromCorners(polygon);
#endif
}

void vpMbGenericTracker::TrackerWrapper::initFaceFromLines(vpMbtPolygon &polygon) {
  if (m_trackerType & EDGE_TRACKER)
    vpMbEdgeTracker::initFaceFromLines(polygon);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER)
    vpMbKltTracker::initFaceFromLines(polygon);
#endif
}

void vpMbGenericTracker::TrackerWrapper::initMbtTracking(const vpImage<unsigned char> &I) {
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::computeVVSInit();
    vpMbEdgeTracker::computeVVSFirstPhaseFactor(I, 0);
  }
}

void vpMbGenericTracker::TrackerWrapper::loadConfigFile(const std::string& configFile) {
#ifdef VISP_HAVE_XML2
  vpMbtEdgeKltXmlParser xmlp;

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  xmlp.setMovingEdge(me);

  xmlp.setMaxFeatures(10000);
  xmlp.setWindowSize(5);
  xmlp.setQuality(0.01);
  xmlp.setMinDistance(5);
  xmlp.setHarrisParam(0.01);
  xmlp.setBlockSize(3);
  xmlp.setPyramidLevels(3);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  xmlp.setMaskBorder(maskBorder);
#endif

  try{
    std::cout << " *********** Parsing XML for Mb Edge Tracker ************ " << std::endl;
    xmlp.parse(configFile.c_str());
  }
  catch(...){
    throw vpException(vpException::ioError, "Can't open XML file \"%s\"\n ", configFile.c_str());
  }

  vpCameraParameters camera;
  xmlp.getCameraParameters(camera);
  setCameraParameters(camera);

  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  if(xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());

  if(xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());

  if(xmlp.getFovClipping()){
    setClipping(vpMbEdgeTracker::clippingFlag | vpPolygon3D::FOV_CLIPPING);
  }

  useLodGeneral = xmlp.getLodState();
  minLineLengthThresholdGeneral = xmlp.getMinLineLengthThreshold();
  minPolygonAreaThresholdGeneral = xmlp.getMinPolygonAreaThreshold();

  applyLodSettingInConfig = false;
  if(this->getNbPolygon() > 0) {
    applyLodSettingInConfig = true;
    setLod(useLodGeneral);
    setMinLineLengthThresh(minLineLengthThresholdGeneral);
    setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);
  }

  vpMe meParser;
  xmlp.getMe(meParser);
  vpMbEdgeTracker::setMovingEdge(meParser);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  tracker.setMaxFeatures((int)xmlp.getMaxFeatures());
  tracker.setWindowSize((int)xmlp.getWindowSize());
  tracker.setQuality(xmlp.getQuality());
  tracker.setMinDistance(xmlp.getMinDistance());
  tracker.setHarrisFreeParameter(xmlp.getHarrisParam());
  tracker.setBlockSize((int)xmlp.getBlockSize());
  tracker.setPyramidLevels((int)xmlp.getPyramidLevels());
  maskBorder = xmlp.getMaskBorder();

  //if(useScanLine)
  faces.getMbScanLineRenderer().setMaskBorder(maskBorder);
#endif

#else
  vpTRACE("You need the libXML2 to read the config file %s", configFile);
#endif
}

void vpMbGenericTracker::TrackerWrapper::preTracking(const vpImage<unsigned char> &I) {
  if (m_trackerType & EDGE_TRACKER) {
    try {
      vpMbEdgeTracker::trackMovingEdge(I);
    } catch (...) {
      std::cerr << "Error in moving edge tracking" << std::endl;
      throw;
    }
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  if (m_trackerType & KLT_TRACKER) {
    try {
      vpMbKltTracker::preTracking(I);
    } catch (...) {
      std::cerr << "Error in KLT tracking" << std::endl;
      throw;
    }
  }
#endif
}

void vpMbGenericTracker::TrackerWrapper::postTracking(const vpImage<unsigned char> &I) {
  if (displayFeatures) {
    if (m_trackerType & EDGE_TRACKER) {
      vpMbEdgeTracker::displayFeaturesOnImage(I, 0);
    }
  }

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  //KLT
  if (m_trackerType & KLT_TRACKER) {
    if (vpMbKltTracker::postTracking(I, m_w_klt)) {
      vpMbKltTracker::reinit(I);
    }
  }
#endif

  // Looking for new visible face
  if (m_trackerType & EDGE_TRACKER) {
    bool newvisibleface = false ;
    vpMbEdgeTracker::visibleFace(I, cMo, newvisibleface);

    if (useScanLine) {
      faces.computeClippedPolygons(cMo, cam);
      faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
    }
  }

  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::updateMovingEdge(I);

    vpMbEdgeTracker::initMovingEdge(I, cMo);
    // Reinit the moving edge for the lines which need it.
    vpMbEdgeTracker::reinitMovingEdge(I, cMo);

    if (computeProjError) {
      vpMbEdgeTracker::computeProjectionError(I);
    }
  }
}

void vpMbGenericTracker::TrackerWrapper::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_, const bool verbose) {
  cMo.eye();


  //Edge
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i++) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine*>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (l != NULL) delete l;
        l = NULL;
      }

      for (std::list<vpMbtDistanceCylinder*>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end(); ++it) {
        cy = *it;
        if (cy != NULL) delete cy;
        cy = NULL;
      }

      for (std::list<vpMbtDistanceCircle*>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (ci != NULL) delete ci;
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


#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  //KLT
#  if (VISP_HAVE_OPENCV_VERSION < 0x020408)
  if(cur != NULL){
    cvReleaseImage(&cur);
    cur = NULL;
  }
#  endif

  // delete the Klt Polygon features
  vpMbtDistanceKltPoints *kltpoly;
  for (std::list<vpMbtDistanceKltPoints*>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    kltpoly = *it;
    if (kltpoly != NULL) {
      delete kltpoly;
    }
    kltpoly = NULL;
  }
  kltPolygons.clear();

  vpMbtDistanceKltCylinder *kltPolyCylinder;
  for (std::list<vpMbtDistanceKltCylinder*>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end(); ++it) {
    kltPolyCylinder = *it;
    if (kltPolyCylinder!=NULL) {
      delete kltPolyCylinder;
    }
    kltPolyCylinder = NULL;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  for (std::list<vpMbtDistanceCircle*>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    ci = *it;
    if (ci!=NULL) {
      delete ci;
    }
    ci = NULL;
  }
  circles_disp.clear();

  firstInitialisation = true;
#endif

  faces.reset();

  loadModel(cad_name, verbose);
  initFromPose(I, cMo_);
}

void vpMbGenericTracker::TrackerWrapper::resetTracker() {
  vpMbEdgeTracker::resetTracker();
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::resetTracker();
#endif
}

void vpMbGenericTracker::TrackerWrapper::setCameraParameters(const vpCameraParameters &camera) {
  this->cam = camera;

  vpMbEdgeTracker::setCameraParameters(cam);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::setCameraParameters(cam);
#endif
}

void vpMbGenericTracker::TrackerWrapper::setOgreVisibilityTest(const bool &v) {
  vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("TrackerWrapper");
#endif
}

void vpMbGenericTracker::TrackerWrapper::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo) {
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::setPose(I, cdMo);
#else
  cMo = cdMo;
  init(I);
  return;
#endif

  if (m_trackerType & EDGE_TRACKER)
    resetMovingEdge();

  if (useScanLine) {
    cam.computeFov(I.getWidth(), I.getHeight());
    faces.computeClippedPolygons(cMo, cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
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
    initMovingEdge(I, cMo);
#endif
}

void vpMbGenericTracker::TrackerWrapper::setProjectionErrorComputation(const bool &flag) {
  vpMbEdgeTracker::setProjectionErrorComputation(flag);
}

void vpMbGenericTracker::TrackerWrapper::setScanLineVisibilityTest(const bool &v) {
  vpMbEdgeTracker::setScanLineVisibilityTest(v);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  vpMbKltTracker::setScanLineVisibilityTest(v);
#endif
}

void vpMbGenericTracker::TrackerWrapper::setTrackerType(const int type) {
  if ( (type & (EDGE_TRACKER
              #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                | KLT_TRACKER
              #endif
                )) == 0 ) {
    throw vpException(vpTrackingException::badValue, "bad value for tracker type: !", type);
  }

  m_trackerType = type;
}

void vpMbGenericTracker::TrackerWrapper::testTracking() {
  if (m_trackerType & EDGE_TRACKER) {
    vpMbEdgeTracker::testTracking();
  }
}

void vpMbGenericTracker::TrackerWrapper::track(const vpImage<unsigned char> &I) {
  if ( (m_trackerType & (EDGE_TRACKER
                       #if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                         | KLT_TRACKER
                       #endif
                         )) == 0 ) {
    std::cerr << "Bad tracker type: " << m_trackerType << std::endl;
    return;
  }

  //Back-up cMo in case of exception
  vpHomogeneousMatrix cMo_1 = cMo;
  try {
    preTracking(I);

    try {
      computeVVS(I);
    } catch (...) {
      covarianceMatrix = -1;
      throw; // throw the original exception
    }

    if (m_trackerType == EDGE_TRACKER)
      testTracking();

    postTracking(I);

  } catch (vpException &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    cMo = cMo_1;
    throw e;
  }
}
