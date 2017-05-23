/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2016 by INRIA. All rights reserved.
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
 *
 * Description:
 * Model-based edge klt tracker with multiple cameras.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
 \file vpMbEdgeKltMultiTracker.cpp
 \brief Model-based edge klt tracker with multiple cameras.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/mbt/vpMbEdgeKltMultiTracker.h>


/*!
  Basic constructor
*/
vpMbEdgeKltMultiTracker::vpMbEdgeKltMultiTracker()
    : vpMbEdgeMultiTracker(), vpMbKltMultiTracker(),
      m_factorKLT(0.65), m_factorMBT(0.35),
      thresholdKLT(2.), thresholdMBT(2.),
      m_mapOfCameraTransformationMatrix(), m_referenceCameraName("Camera"),
      m_nbrow(0), m_L_hybridMulti(), m_error_hybridMulti(), m_w_hybridMulti(), m_weightedError_hybridMulti()
{
  //Add default camera transformation matrix
  m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();

  m_lambda = 0.8;
  m_maxIter = 200;
}

/*!
  Construct a vpMbEdgeKltMultiTracker with the specified number of cameras.

  \param nbCameras : Number of cameras to use.
*/
vpMbEdgeKltMultiTracker::vpMbEdgeKltMultiTracker(const unsigned int nbCameras)
    : vpMbEdgeMultiTracker(nbCameras), vpMbKltMultiTracker(nbCameras),
      m_factorKLT(0.65), m_factorMBT(0.35),
      thresholdKLT(2.), thresholdMBT(2.),
      m_mapOfCameraTransformationMatrix(), m_referenceCameraName("Camera"),
      m_nbrow(0), m_L_hybridMulti(), m_error_hybridMulti(), m_w_hybridMulti(), m_weightedError_hybridMulti()
{

  if(nbCameras == 0) {
    throw vpException(vpTrackingException::fatalError, "Cannot construct a vpMbkltMultiTracker with no camera !");
  } else if(nbCameras == 1) {
    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else if(nbCameras == 2) {
    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera1"] = vpHomogeneousMatrix();
    m_mapOfCameraTransformationMatrix["Camera2"] = vpHomogeneousMatrix();

    //Set by default the reference camera
    m_referenceCameraName = "Camera1";
  } else {
    for(unsigned int i = 1; i <= nbCameras; i++) {
      std::stringstream ss;
      ss << "Camera" << i;

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfKltTrackers.begin()->first;
  }

  m_lambda = 0.8;
  m_maxIter = 200;
}

/*!
  Construct a vpMbEdgeKltMultiTracker with the specified list of camera names.

  \param cameraNames : List of camera names.
*/
vpMbEdgeKltMultiTracker::vpMbEdgeKltMultiTracker(const std::vector<std::string> &cameraNames)
    : vpMbEdgeMultiTracker(cameraNames), vpMbKltMultiTracker(cameraNames),
      m_factorKLT(0.65), m_factorMBT(0.35),
      thresholdKLT(2.), thresholdMBT(2.),
      m_mapOfCameraTransformationMatrix(), m_referenceCameraName("Camera"),
      m_nbrow(0), m_L_hybridMulti(), m_error_hybridMulti(), m_w_hybridMulti(), m_weightedError_hybridMulti()
{
  //Set by default the reference camera
  m_referenceCameraName = cameraNames.front();

  m_lambda = 0.8;
  m_maxIter = 200;
}

vpMbEdgeKltMultiTracker::~vpMbEdgeKltMultiTracker() {
}

void vpMbEdgeKltMultiTracker::computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const unsigned int lvl) {

  m_nbrow = initMbtTracking(mapOfImages, lvl);

  if(m_nbInfos < 4 && m_nbrow < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  } else if(m_nbrow < 4) {
    m_nbrow = 0;
  }

  double factorMBT = m_factorMBT;
  double factorKLT = m_factorKLT;
  if (m_nbrow < 4) {
    factorKLT = 1.;
    std::cerr << "There are less than 4 KLT features, set factorKLT = 1. !" << std::endl;
  }

  if (m_nbInfos < 4) {
    factorMBT = 1.;
    std::cerr << "There are less than 4 moving edges, set factorMBT = 1. !" << std::endl;
    m_nbInfos = 0;
  }

  computeVVSInit();

  vpHomogeneousMatrix cMoPrev;
  vpHomogeneousMatrix ctTc0_Prev;
  //Error vector for MBT + KLT for the previous iteration
  vpColVector m_error_prev;
  //Weighting vector for MBT + KLT for the previous iteration
  vpColVector m_w_prev;
  double mu = m_initialMu;

  //Create the map of VelocityTwistMatrices
  std::map<std::string, vpVelocityTwistMatrix> mapOfVelocityTwist;
  for(std::map<std::string, vpHomogeneousMatrix>::const_iterator it = m_mapOfCameraTransformationMatrix.begin(); it != m_mapOfCameraTransformationMatrix.end(); ++it) {
    vpVelocityTwistMatrix cVo;
    cVo.buildFrom(it->second);
    mapOfVelocityTwist[it->first] = cVo;
  }

  //Variables used in the minimization process
  double residu = 0;
  double residu_1 = -1;
  unsigned int iter = 0;

  vpMatrix L_true;
  vpMatrix LVJ_true;

  vpColVector v;

  vpMatrix LTL;
  vpColVector LTR;


  while ( ((int)((residu - residu_1)*1e8) !=0 )  && (iter < m_maxIter) ) {
    computeVVSInteractionMatrixAndResidu(mapOfImages, mapOfVelocityTwist);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_hybridMulti, m_error_prev, cMoPrev, mu, reStartFromLastIncrement, &m_w_prev);
    if (reStartFromLastIncrement) {
      ctTc0 = ctTc0_Prev;
    }

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      //Set weight for m_w with the good weighting between MBT and KLT
      for (unsigned int cpt = 0; cpt < m_nbrow + 2*m_nbInfos; cpt++) {
        if (cpt < m_nbrow) {
          m_w_hybridMulti[cpt] = ( m_w_hybridMulti[cpt] * m_factor[cpt]) * factorMBT;
        } else {
          m_w_hybridMulti[cpt] *= factorKLT;
        }
      }

      if (computeCovariance) {
        L_true = m_L_hybridMulti;
        if (!isoJoIdentity) {
           vpVelocityTwistMatrix cVo;
           cVo.buildFrom(cMo);
           LVJ_true = ( m_L_hybridMulti*cVo*oJo );
        }
      }

      residu_1 = residu;
      residu = 0;
      double num = 0;
      double den = 0;

      for (unsigned int i = 0; i < m_weightedError_hybridMulti.getRows(); i++) {
        num += m_w_hybridMulti[i]*vpMath::sqr(m_error_hybridMulti[i]);
        den += m_w_hybridMulti[i];

        m_weightedError_hybridMulti[i] = m_error_hybridMulti[i] * m_w_hybridMulti[i];
        if (m_computeInteraction) {
          for (unsigned int j = 0; j < 6; j++) {
            m_L_hybridMulti[i][j] *= m_w_hybridMulti[i];
          }
        }
      }

      residu = sqrt(num/den);

      computeVVSPoseEstimation(isoJoIdentity, iter, m_L_hybridMulti, LTL, m_weightedError_hybridMulti, m_error_hybridMulti, m_error_prev, LTR, mu, v, &m_w_hybridMulti, &m_w_prev);

      cMoPrev = cMo;
      ctTc0_Prev = ctTc0;
      ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
      cMo = ctTc0 * c0Mo;
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity, m_w_hybridMulti, cMoPrev, L_true, LVJ_true, m_error_hybridMulti);
}

void vpMbEdgeKltMultiTracker::computeVVSInit() {
  unsigned int totalNbRows = 2*m_nbInfos + m_nbrow;

  m_L_hybridMulti.resize(totalNbRows, 6, false);
  m_error_hybridMulti.resize(totalNbRows, false);

  m_weightedError_hybridMulti.resize(totalNbRows, false);
  m_w_hybridMulti.resize(totalNbRows, false);
  m_w_hybridMulti = 1;

  vpMbKltTracker *klt;
  for (std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;
    klt->computeVVSInit();
  }
}

void vpMbEdgeKltMultiTracker::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbEdgeKltMultiTracker::computeVVSInteractionMatrixAndResidu() should not be called!");
}

void vpMbEdgeKltMultiTracker::computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                                   std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist) {
  unsigned int startIdx = 0;

  if (m_nbrow >= 4) {
    vpMbEdgeTracker *edge;

    for (std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
      edge = it->second;

      //Set the corresponding cMo for the current camera
      it->second->cMo = m_mapOfCameraTransformationMatrix[it->first]*cMo;

      edge->computeVVSInteractionMatrixAndResidu(*mapOfImages[it->first]);

      //Stack interaction matrix for MBT
      m_L_hybridMulti.insert(edge->m_L_edge*mapOfVelocityTwist[it->first], startIdx, 0);
      //Stack residual for MBT
      m_error_hybridMulti.insert(startIdx, edge->m_error_edge);

      startIdx += edge->m_error_edge.getRows();
    }
  }

  vpMbKltTracker *klt;
  for (std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;

    if (klt->m_nbInfos > 0) {

      //Use the ctTc0 variable instead of the formula in the monocular case
      //to ensure that we have the same result than vpMbKltTracker
      //as some slight differences can occur due to numerical imprecision
      if(m_mapOfKltTrackers.size() == 1) {
        klt->ctTc0 = ctTc0;
        klt->computeVVSInteractionMatrixAndResidu();
      } else {
        vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * klt->c0Mo.inverse();
        klt->ctTc0 = c_curr_tTc_curr0;
        klt->computeVVSInteractionMatrixAndResidu();
      }

      //Stack residual and interaction matrix
      m_error_hybridMulti.insert(startIdx, klt->m_error_klt);
      m_L_hybridMulti.insert(klt->m_L_klt*mapOfVelocityTwist[it->first], startIdx, 0);

      startIdx += 2*klt->m_nbInfos;
    }
  }
}

void vpMbEdgeKltMultiTracker::computeVVSWeights() {
  unsigned int startIdx = 0;

  vpMbEdgeTracker *edge = NULL;
  for (std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;

    //Compute weights
    edge->computeVVSWeights();

    m_w_hybridMulti.insert(startIdx, edge->m_w_edge);
    startIdx += edge->m_w_edge.getRows();
  }

  vpMbKltTracker *klt;
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;

    //Compute weights
    klt->computeVVSWeights(klt->m_robust_klt, klt->m_error_klt, klt->m_w_klt);

    m_w_hybridMulti.insert(startIdx, klt->m_w_klt);
    startIdx += klt->m_w_klt.getRows();
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
*/
void vpMbEdgeKltMultiTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_,
    const vpCameraParameters &cam_, const vpColor& col, const unsigned int thickness,
    const bool displayFullModel) {
  vpMbEdgeMultiTracker::display(I, cMo_, cam_, col, thickness, displayFullModel);

  //Display only features for KLT trackers
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {

    vpMbtDistanceKltPoints *kltpoly;
    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it_pts = it_klt->second->kltPolygons.begin();
        it_pts != it_klt->second->kltPolygons.end(); ++it_pts){
      kltpoly = *it_pts;
      if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
          kltpoly->displayPrimitive(I);
      }
    }

    vpMbtDistanceKltCylinder *kltPolyCylinder;
    for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it_cyl = it_klt->second->kltCylinders.begin();
        it_cyl != it_klt->second->kltCylinders.end(); ++it_cyl){
      kltPolyCylinder = *it_cyl;
      if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
        kltPolyCylinder->displayPrimitive(I);
    }
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The color image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param cam_ : The camera parameters.
  \param color : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbEdgeKltMultiTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_,
    const vpCameraParameters &cam_, const vpColor& color , const unsigned int thickness,
    const bool displayFullModel) {
  vpMbEdgeMultiTracker::display(I, cMo_, cam_, color, thickness, displayFullModel);
//  vpMbKltMultiTracker::display(I, cMo_, cam_, color, thickness, displayFullModel);

  //Display only features for KLT trackers
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {

    vpMbtDistanceKltPoints *kltpoly;
    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it_pts = it_klt->second->kltPolygons.begin();
        it_pts != it_klt->second->kltPolygons.end(); ++it_pts){
      kltpoly = *it_pts;
      if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
          kltpoly->displayPrimitive(I);
      }
    }

    vpMbtDistanceKltCylinder *kltPolyCylinder;
    for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it_cyl = it_klt->second->kltCylinders.begin();
        it_cyl != it_klt->second->kltCylinders.end(); ++it_cyl){
      kltPolyCylinder = *it_cyl;
      if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
        kltPolyCylinder->displayPrimitive(I);
    }
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
*/
void vpMbEdgeKltMultiTracker::display(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor& color, const unsigned int thickness,
    const bool displayFullModel) {
  vpMbEdgeMultiTracker::display(I1, I2, c1Mo, c2Mo, cam1, cam2, color, thickness, displayFullModel);
//  vpMbKltMultiTracker::display(I1, I2, c1Mo, c2Mo, cam1, cam2, color, thickness, displayFullModel);

  //Display only features for KLT trackers
  bool first = true;
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    vpMbtDistanceKltPoints *kltpoly;
    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it_pts = it_klt->second->kltPolygons.begin();
        it_pts != it_klt->second->kltPolygons.end(); ++it_pts){
      kltpoly = *it_pts;
      if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
        if(first) {
          kltpoly->displayPrimitive(I1);
        } else {
          kltpoly->displayPrimitive(I2);
        }
      }
    }

    vpMbtDistanceKltCylinder *kltPolyCylinder;
    for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it_cyl = it_klt->second->kltCylinders.begin();
        it_cyl != it_klt->second->kltCylinders.end(); ++it_cyl){
      kltPolyCylinder = *it_cyl;
      if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints()) {
        if(first) {
          kltPolyCylinder->displayPrimitive(I1);
        } else {
          kltPolyCylinder->displayPrimitive(I2);
        }
      }
    }

    first = false;
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
*/
void vpMbEdgeKltMultiTracker::display(const vpImage<vpRGBa>& I1, const vpImage<vpRGBa>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor& color, const unsigned int thickness,
    const bool displayFullModel) {
  vpMbEdgeMultiTracker::display(I1, I2, c1Mo, c2Mo, cam1, cam2, color, thickness, displayFullModel);
//  vpMbKltMultiTracker::display(I1, I2, c1Mo, c2Mo, cam1, cam2, color, thickness, displayFullModel);

  //Display only features for KLT trackers (not the model)
  bool first = true;
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    vpMbtDistanceKltPoints *kltpoly;
    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it_pts = it_klt->second->kltPolygons.begin();
        it_pts != it_klt->second->kltPolygons.end(); ++it_pts){
      kltpoly = *it_pts;
      if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
        if(first) {
          kltpoly->displayPrimitive(I1);
        } else {
          kltpoly->displayPrimitive(I2);
        }
      }
    }

    vpMbtDistanceKltCylinder *kltPolyCylinder;
    for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it_cyl = it_klt->second->kltCylinders.begin();
        it_cyl != it_klt->second->kltCylinders.end(); ++it_cyl){
      kltPolyCylinder = *it_cyl;
      if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints()) {
        if(first) {
          kltPolyCylinder->displayPrimitive(I1);
        } else {
          kltPolyCylinder->displayPrimitive(I2);
        }
      }
    }

    first = false;
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
void vpMbEdgeKltMultiTracker::display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
      const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
      const vpColor& col, const unsigned int thickness, const bool displayFullModel) {
  vpMbEdgeMultiTracker::display(mapOfImages, mapOfCameraPoses, mapOfCameraParameters, col, thickness, displayFullModel);
//  vpMbKltMultiTracker::display(mapOfImages, mapOfCameraPoses, mapOfCameraParameters, col, thickness, displayFullModel);

  //Display only features for KLT trackers (not the model)
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {

    std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(it_klt->first);
    if(it_img != mapOfImages.end()) {
      vpMbtDistanceKltPoints *kltpoly;
      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it_pts = it_klt->second->kltPolygons.begin();
          it_pts != it_klt->second->kltPolygons.end(); ++it_pts) {
        kltpoly = *it_pts;
        if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
            kltpoly->displayPrimitive( *(it_img->second) );
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it_cyl = it_klt->second->kltCylinders.begin();
          it_cyl != it_klt->second->kltCylinders.end(); ++it_cyl) {
        kltPolyCylinder = *it_cyl;
        if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
          kltPolyCylinder->displayPrimitive( *(it_img->second) );
      }
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
void vpMbEdgeKltMultiTracker::display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
      const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
      const vpColor& col, const unsigned int thickness, const bool displayFullModel) {
  vpMbEdgeMultiTracker::display(mapOfImages, mapOfCameraPoses, mapOfCameraParameters, col, thickness, displayFullModel);
//  vpMbKltMultiTracker::display(mapOfImages, mapOfCameraPoses, mapOfCameraParameters, col, thickness, displayFullModel);

  //Display only features for KLT trackers  (not the model)
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {

    std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img = mapOfImages.find(it_klt->first);
    if(it_img != mapOfImages.end()) {
      vpMbtDistanceKltPoints *kltpoly;
      for(std::list<vpMbtDistanceKltPoints*>::const_iterator it_pts = it_klt->second->kltPolygons.begin();
          it_pts != it_klt->second->kltPolygons.end(); ++it_pts){
        kltpoly = *it_pts;
        if(displayFeatures && kltpoly->hasEnoughPoints() && kltpoly->isTracked() && kltpoly->polygon->isVisible()) {
            kltpoly->displayPrimitive( *(it_img->second) );
        }
      }

      vpMbtDistanceKltCylinder *kltPolyCylinder;
      for(std::list<vpMbtDistanceKltCylinder*>::const_iterator it_cyl = it_klt->second->kltCylinders.begin();
          it_cyl != it_klt->second->kltCylinders.end(); ++it_cyl){
        kltPolyCylinder = *it_cyl;
        if(displayFeatures && kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
          kltPolyCylinder->displayPrimitive( *(it_img->second) );
      }
    }
  }
}

/*!
  Get the camera names

  \return The vector of camera names.
*/
std::vector<std::string> vpMbEdgeKltMultiTracker::getCameraNames() const {
  return vpMbEdgeMultiTracker::getCameraNames();
}

/*!
  Get the camera parameters for the reference camera.

  \param camera : Copy of the camera parameters used by the tracker.
*/
void vpMbEdgeKltMultiTracker::getCameraParameters(vpCameraParameters &camera) const {
  camera = this->cam;
}

/*!
  Get the camera parameters for the stereo cameras case.

  \param cam1 : Copy of the camera parameters for the first camera.
  \param cam2 : Copy of the camera parameters for the second camera.
*/
void vpMbEdgeKltMultiTracker::getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const {
  //We could use either the vpMbEdgeMultiTracker or vpMbKltMultiTracker class
  vpMbEdgeMultiTracker::getCameraParameters(cam1, cam2);
}

/*!
  Get the camera parameters specified by its name.

  \param cameraName : Name of the camera.
  \param camera : Copy of the camera parameters.
*/
void vpMbEdgeKltMultiTracker::getCameraParameters(const std::string &cameraName, vpCameraParameters &camera) const {
  //We could use either the vpMbEdgeMultiTracker or vpMbKltMultiTracker class
  vpMbEdgeMultiTracker::getCameraParameters(cameraName, camera);
}

/*!
  Get all the camera parameters.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbEdgeKltMultiTracker::getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const {
  //Clear the input map
  mapOfCameraParameters.clear();

  //We could use either the vpMbEdgeMultiTracker or vpMbKltMultiTracker class
  vpMbEdgeMultiTracker::getCameraParameters(mapOfCameraParameters);
}

/*!
  Get the clipping used and defined in vpPolygon3D::vpMbtPolygonClippingType for the given camera name.

  \param cameraName : Name of the desired camera.
  \return Clipping flags.
*/
unsigned int vpMbEdgeKltMultiTracker::getClipping(const std::string &cameraName) const {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    //Return the clipping flags for m_mapOfKltTrackers as it should be the same for the
    //same camera in m_mapOfEdgeTrackers
    return it->second->getClipping();
  } else {
    std::cerr << "Cannot find camera: " << cameraName << std::endl;
  }

  return vpMbTracker::getClipping();
}

vpMbHiddenFaces<vpMbtPolygon>& vpMbEdgeKltMultiTracker::getFaces() {
  std::cerr << "Return the wrong faces reference !" << std::endl;
  std::cerr << "Use vpMbEdgeKltMultiTracker::getEdgeFaces or "
      "vpMbEdgeKltMultiTracker::getKltFaces instead !" << std::endl;

  return faces;
}

/*!
  Return a reference to the faces structure for the given camera name for edge trackers.

  \return Reference to the face structure.
 */
vpMbHiddenFaces<vpMbtPolygon>& vpMbEdgeKltMultiTracker::getEdgeFaces(const std::string &cameraName) {
  return vpMbEdgeMultiTracker::getFaces(cameraName);
}

/*!
  Return a map of faces structure for each camera for the edge trackers.

  \return Reference a map of the face structure for each camera.
 */
std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > vpMbEdgeKltMultiTracker::getEdgeFaces() const {
  return vpMbEdgeMultiTracker::getFaces();
}

/*!
  Return a reference to the faces structure for the given camera name for KLT trackers.

  \return Reference to the face structure.
 */
vpMbHiddenFaces<vpMbtPolygon>& vpMbEdgeKltMultiTracker::getKltFaces(const std::string &cameraName) {
  return vpMbKltMultiTracker::getFaces(cameraName);
}

/*!
  Return a map of faces structure for each camera for the KLT trackers.

  \return Reference a map of the face structure for each camera.
 */
std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > vpMbEdgeKltMultiTracker::getKltFaces() const {
  return vpMbKltMultiTracker::getFaces();
}

unsigned int vpMbEdgeKltMultiTracker::getNbPolygon() const {
  std::cerr << "Use vpMbEdgeKltMultiTracker::getEdgeMultiNbPolygon or "
      "vpMbEdgeKltMultiTracker::getKltMultiNbPolygon instead !" << std::endl;
  return 0;
}

/*!
  Get the number of polygons (faces) representing the object to track for the edge trackers.

  \return Number of polygons.
*/
std::map<std::string, unsigned int> vpMbEdgeKltMultiTracker::getEdgeMultiNbPolygon() const {
  return vpMbEdgeMultiTracker::getMultiNbPolygon();
}

/*!
  Get the number of polygons (faces) representing the object to track for the KLT trackers.

  \return Number of polygons.
*/
std::map<std::string, unsigned int> vpMbEdgeKltMultiTracker::getKltMultiNbPolygon() const {
  return vpMbKltMultiTracker::getMultiNbPolygon();
}

/*!
  Get the current pose between the object and the cameras.

  \param c1Mo : The camera pose for the first camera.
  \param c2Mo : The camera pose for the second camera.
*/
void vpMbEdgeKltMultiTracker::getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const {
  //We could use either the vpMbEdgeMultiTracker or vpMbKltMultiTracker class
  vpMbEdgeMultiTracker::getPose(c1Mo, c2Mo);
}

/*!
  Get the current pose between the object and the camera.
  cMo is the matrix which can be used to express
  coordinates from the object frame to camera frame.

  \param cameraName : The name of the camera.
  \param cMo_ : The camera pose for the specified camera.
*/
void vpMbEdgeKltMultiTracker::getPose(const std::string &cameraName, vpHomogeneousMatrix &cMo_) const {
  //We could use either the vpMbEdgeMultiTracker or vpMbKltMultiTracker class
  vpMbEdgeMultiTracker::getPose(cameraName, cMo_);
}

/*!
  Get the current pose between the object and the cameras.

  \param mapOfCameraPoses : The map of camera poses for all the cameras.
*/
void vpMbEdgeKltMultiTracker::getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const {
  //Clear the map
  mapOfCameraPoses.clear();

  //We could use either the vpMbEdgeMultiTracker or vpMbKltMultiTracker class
  vpMbEdgeMultiTracker::getPose(mapOfCameraPoses);
}

void vpMbEdgeKltMultiTracker::init(const vpImage<unsigned char>& /*I*/) {
  if(!modelInitialised){
    throw vpException(vpTrackingException::initializationError, "model not initialized");
  }
}

#ifdef VISP_HAVE_MODULE_GUI
/*!
  Initialise the tracker by clicking in the image on the pixels that correspond to the
  3D points whose coordinates are given in \e points3D_list.

  \param I : Input image where the user has to click.
  \param points3D_list : List of at least 4 3D points with coordinates expressed in meters in the object frame.
  \param displayFile : Path to the image used to display the help. This image may be used to show where to click.
  This functionality is only available if visp_io module is used.
*/
void vpMbEdgeKltMultiTracker::initClick(const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
                       const std::string &displayFile) {
  //Cannot use directly set pose for KLT as it is different than for the edge case
  //It moves the KLT points instead of detecting new KLT points
  vpMbKltMultiTracker::initClick(I, points3D_list, displayFile);

  //Set pose for Edge (setPose or initFromPose is equivalent with vpMbEdgeTracker)
  //And it avoids to click a second time
  vpMbEdgeMultiTracker::setPose(I, cMo);
}

/*!
  Initialise the tracker by clicking in the image on the pixels that correspond to the
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

  \param I : Input image where the user has to click.
  \param initFile : File containing the coordinates of at least 4 3D points the user has
  to click in the image. This file should have .init extension (ie teabox.init).
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.
*/
void vpMbEdgeKltMultiTracker::initClick(const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp) {
  //Cannot use directly set pose for KLT as it is different than for the edge case
  //It moves the KLT points instead of detecting new KLT points
  vpMbKltMultiTracker::initClick(I, initFile, displayHelp);

  //Set pose for Edge (setPose or initFromPose is equivalent with vpMbEdgeTracker)
  //And it avoids to click a second time
  vpMbEdgeMultiTracker::setPose(I, cMo);
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

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param initFile1 : File containing the coordinates of at least 4 3D points the user has
  to click in the image acquired by the first camera. This file should have .init extension (ie teabox.init).
  \param initFile2 : File containing the coordinates of at least 4 3D points the user has
  to click in the image acquired by the second camera. This file should have .init extension.
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \param firstCameraIsReference : If true, the first camera is the reference, otherwise it is the second one.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.
*/
void vpMbEdgeKltMultiTracker::initClick(const vpImage<unsigned char>& I1, const vpImage<unsigned char> &I2,
    const std::string& initFile1, const std::string& initFile2, const bool displayHelp, const bool firstCameraIsReference) {
  vpMbKltMultiTracker::initClick(I1, I2, initFile1, initFile2, displayHelp, firstCameraIsReference);

  //Get c2Mo
  vpHomogeneousMatrix c2Mo;
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.begin();
  if(firstCameraIsReference) {
    ++it_klt;
    it_klt->second->getPose(c2Mo);
  } else {
    it_klt->second->getPose(c2Mo);
  }

  //Set pose for Edge (setPose or initFromPose is equivalent with vpMbEdgeTracker)
  vpMbEdgeMultiTracker::setPose(I1, I2, cMo, c2Mo, firstCameraIsReference);
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

  \param mapOfImages : Map of images.
  \param initFile : File containing the points where to click for the reference camera.
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.

*/
void vpMbEdgeKltMultiTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::string &initFile, const bool displayHelp) {
  vpMbKltMultiTracker::initClick(mapOfImages, initFile, displayHelp);

  //Get pose for all the cameras
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;
  vpMbKltMultiTracker::getPose(mapOfCameraPoses);

  //Set pose for Edge for all the cameras (setPose or initFromPose is equivalent with vpMbEdgeTracker)
  //And it avoids to click a second time
  vpMbEdgeMultiTracker::setPose(mapOfImages, mapOfCameraPoses);
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
  \param mapOfInitFiles : map of files containing the points where to click for each camera.
  \param displayHelp : Optional display of an image (ie teabox.ppm). This
  image may be used to show where to click.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.
*/
void vpMbEdgeKltMultiTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp) {
  vpMbKltMultiTracker::initClick(mapOfImages, mapOfInitFiles, displayHelp);

  //Get pose for all the cameras
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;
  vpMbKltMultiTracker::getPose(mapOfCameraPoses);

  //Set pose for Edge for all the cameras (setPose or initFromPose is equivalent with vpMbEdgeTracker)
  //And it avoids to click a second time
  vpMbEdgeMultiTracker::setPose(mapOfImages, mapOfCameraPoses);
}
#endif //#ifdef VISP_HAVE_MODULE_GUI

void vpMbEdgeKltMultiTracker::initCircle(const vpPoint&, const vpPoint &, const vpPoint &, const double, const int,
    const std::string &) {
  std::cerr << "The method initCircle is not used in vpMbEdgeKltMultiTracker !" << std::endl;
}

void vpMbEdgeKltMultiTracker::initCylinder(const vpPoint&, const vpPoint &, const double, const int,
    const std::string &) {
  std::cerr << "The method initCylinder is not used in vpMbEdgeKltMultiTracker !" << std::endl;
}

void vpMbEdgeKltMultiTracker::initFaceFromCorners(vpMbtPolygon &) {
  std::cerr << "The method initFaceFromCorners is not used in vpMbEdgeKltMultiTracker !" << std::endl;
}

void vpMbEdgeKltMultiTracker::initFaceFromLines(vpMbtPolygon &) {
  std::cerr << "The method initFaceFromLines is not used in vpMbEdgeKltMultiTracker !" << std::endl;
}

/*!
  Initialise the tracking thanks to the pose in vpPoseVector format, and read in the file initFile.
  The structure of this file is (without the comments):
  \code
  // The six value of the pose vector
  0.0000    //  \
  0.0000    //  |
  1.0000    //  | Exemple of value for the pose vector where Z = 1 meter
  0.0000    //  |
  0.0000    //  |
  0.0000    //  /
  \endcode

  Where the three firsts lines refer to the translation and the three last to the rotation in thetaU
  parametrisation (see vpThetaUVector).
  \param I : Input image
  \param initFile : Path to the file containing the pose.
*/
void vpMbEdgeKltMultiTracker::initFromPose(const vpImage<unsigned char>& I, const std::string &initFile) {
  //Monocular case only !
  if(m_mapOfKltTrackers.size() > 1) {
    throw vpException(vpTrackingException::fatalError, "This function can only be used for the monocular case !");
  }

  char s[FILENAME_MAX];
  std::fstream finit ;
  vpPoseVector init_pos;

  std::string ext = ".pos";
  size_t pos =  initFile.rfind(ext);

  if( pos == initFile.size()-ext.size() && pos != 0)
    sprintf(s,"%s", initFile.c_str());
  else
    sprintf(s,"%s.pos", initFile.c_str());

  finit.open(s,std::ios::in) ;
  if (finit.fail()){
    std::cerr << "cannot read " << s << std::endl;
    throw vpException(vpException::ioError, "cannot read init file");
  }

  for (unsigned int i = 0; i < 6; i += 1){
    finit >> init_pos[i];
  }

  cMo.buildFrom(init_pos);
  vpMbEdgeMultiTracker::initFromPose(I, cMo);
  vpMbKltMultiTracker::initFromPose(I, cMo);
}

/*!
  Initialise the tracking thanks to the pose.

  \param I : Input image
  \param cMo_ : Pose matrix.
*/
void vpMbEdgeKltMultiTracker::initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_) {
  //Monocular case only !
  if(m_mapOfKltTrackers.size() > 1) {
    throw vpException(vpTrackingException::fatalError, "This function can only be used for the monocular case !");
  }

  this->cMo = cMo_;
  vpMbEdgeMultiTracker::initFromPose(I, cMo);
  vpMbKltMultiTracker::initFromPose(I, cMo);
}

/*!
  Initialise the tracking thanks to the pose vector.

  \param I : Input image
  \param cPo : Pose vector.
*/
void vpMbEdgeKltMultiTracker::initFromPose (const vpImage<unsigned char>& I, const vpPoseVector &cPo) {
  vpHomogeneousMatrix _cMo(cPo);
  initFromPose(I, _cMo);
}

/*!
  Initialize the tracking thanks to the pose for stereo cameras configuration.

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param c1Mo : Pose matrix for the first camera.
  \param c2Mo : Pose matrix for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference camera, otherwise it is the second one.
*/
void vpMbEdgeKltMultiTracker::initFromPose(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const bool firstCameraIsReference) {
  vpMbEdgeMultiTracker::initFromPose(I1, I2, c1Mo, c2Mo, firstCameraIsReference);
  vpMbKltMultiTracker::initFromPose(I1, I2, c1Mo, c2Mo, firstCameraIsReference);
}

/*!
  Initialize the tracking thanks to the pose. The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images.
  \param cMo_ : Pose matrix for the reference camera.
*/
void vpMbEdgeKltMultiTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const vpHomogeneousMatrix &cMo_) {
  vpMbEdgeMultiTracker::initFromPose(mapOfImages, cMo_);
  vpMbKltMultiTracker::initFromPose(mapOfImages, cMo_);
}

/*!
  Initialize the tracking thanks to the pose.

  \param mapOfImages : Map of images.
  \param mapOfCameraPoses : Map of pose matrix.
*/
void vpMbEdgeKltMultiTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  vpMbEdgeMultiTracker::initFromPose(mapOfImages, mapOfCameraPoses);
  vpMbKltMultiTracker::initFromPose(mapOfImages, mapOfCameraPoses);
}

unsigned int vpMbEdgeKltMultiTracker::initMbtTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, unsigned int lvl) {
  vpMbEdgeTracker *edge = NULL;
  unsigned int nbrows = 0;

  m_factor.resize(0, false);
  for (std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;

    try {
      edge->computeVVSInit();

      unsigned int nbrow = edge->m_error_edge.getRows();
      nbrows += nbrow;

      //Set the corresponding cMo for each camera
      //Used in computeVVSFirstPhaseFactor with computeInteractionMatrixError
      edge->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;

      edge->computeVVSFirstPhaseFactor(*mapOfImages[it->first], lvl);
      m_factor.stack(edge->m_factor);
    } catch (...) {
      edge->m_L_edge.resize(0, 6, false);
      edge->m_error_edge.resize(0, false);

      edge->m_weightedError_edge.resize(0, false);
      edge->m_w_edge.resize(0, false);
      edge->m_factor.resize(0, false);

      edge->m_robustLines.resize(0);
      edge->m_robustCylinders.resize(0);
      edge->m_robustCircles.resize(0);

      edge->m_wLines.resize(0, false);
      edge->m_wCylinders.resize(0, false);
      edge->m_wCircles.resize(0, false);

      edge->m_errorLines.resize(0, false);
      edge->m_errorCylinders.resize(0, false);
      edge->m_errorCircles.resize(0, false);
    }
  }

  return nbrows;
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the objects: KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data).

  \param configFile : full name of the xml file.

  The XML configuration file has the following form:
  \code
<?xml version="1.0"?>
<conf>
  <camera>
    <width>640</width>
    <height>480</height>
    <u0>320</u0>
    <v0>240</v0>
    <px>686.24</px>
    <py>686.24</py>
  </camera>
  <face>
    <angle_appear>65</angle_appear>
    <angle_disappear>85</angle_disappear>
    <near_clipping>0.01</near_clipping>
    <far_clipping>0.90</far_clipping>
    <fov_clipping>1</fov_clipping>
  </face>
  <klt>
    <mask_border>10</mask_border>
    <max_features>10000</max_features>
    <window_size>5</window_size>
    <quality>0.02</quality>
    <min_distance>10</min_distance>
    <harris>0.02</harris>
    <size_block>3</size_block>
    <pyramid_lvl>3</pyramid_lvl>
  </klt>
</conf>
  \endcode

  \sa vpXmlParser::cleanup()
*/
void vpMbEdgeKltMultiTracker::loadConfigFile(const std::string &configFile) {
  vpMbEdgeMultiTracker::loadConfigFile(configFile);
  vpMbKltMultiTracker::loadConfigFile(configFile);
}

/*!
  Load the xml configuration files for the stereo cameras case. An example of such a file is provided in
  loadConfigFile(const std::string &) documentation.
  From the configuration file initialize the parameters corresponding to the objects: KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param configFile1 : Full name of the xml file for the first camera.
  \param configFile2 : Full name of the xml file for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference, otherwise it is the second one.

  \sa loadConfigFile(const std::string &), vpXmlParser::cleanup()
*/
void vpMbEdgeKltMultiTracker::loadConfigFile(const std::string& configFile1, const std::string& configFile2,
    const bool firstCameraIsReference) {
  vpMbEdgeMultiTracker::loadConfigFile(configFile1, configFile2, firstCameraIsReference);
  vpMbKltMultiTracker::loadConfigFile(configFile1, configFile2, firstCameraIsReference);
}

/*!
  Load the xml configuration files for all the cameras. An example of such a file is provided in
  loadConfigFile(const std::string &) documentation.
  From the configuration file initialize the parameters corresponding to the objects: KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param mapOfConfigFiles : Map of xml files.

  \sa loadConfigFile(const std::string &), vpXmlParser::cleanup()
*/
void vpMbEdgeKltMultiTracker::loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles) {
  vpMbEdgeMultiTracker::loadConfigFile(mapOfConfigFiles);
  vpMbKltMultiTracker::loadConfigFile(mapOfConfigFiles);
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

  \param modelFile : the file containing the the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbEdgeKltMultiTracker::loadModel(const std::string &modelFile, const bool verbose) {
  vpMbEdgeMultiTracker::loadModel(modelFile, verbose);
  vpMbKltMultiTracker::loadModel(modelFile, verbose);

  modelInitialised = true;
}

void vpMbEdgeKltMultiTracker::postTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const unsigned int lvl) {
  //MBT
  vpMbEdgeTracker *edge = NULL;
  for (std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;

    edge->updateMovingEdgeWeights();

    if (displayFeatures) {
      edge->displayFeaturesOnImage(*mapOfImages[it->first], lvl);
    }
  }

  //KLT
  vpMbKltMultiTracker::postTracking(mapOfImages);

  // Looking for new visible face
  bool newvisibleface = false;
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;
    edge->visibleFace(*mapOfImages[it->first], it->second->cMo, newvisibleface);
  }

  if(useScanLine) {
    for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
      edge = it->second;

      edge->faces.computeClippedPolygons(it->second->cMo, it->second->cam);
      edge->faces.computeScanLineRender(it->second->cam, mapOfImages[it->first]->getWidth(), mapOfImages[it->first]->getHeight());
    }
  }

  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;
    edge->updateMovingEdge(*mapOfImages[it->first]);
  }

  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;
    edge->initMovingEdge(*mapOfImages[it->first], it->second->cMo);

    // Reinit the moving edge for the lines which need it.
    edge->reinitMovingEdge(*mapOfImages[it->first], it->second->cMo);

    if(computeProjError) {
      edge->computeProjectionError(*mapOfImages[it->first]);
    }
  }
}

void vpMbEdgeKltMultiTracker::reinit(/*const vpImage<unsigned char>& I */) {
//  vpMbEdgeMultiTracker::reinit();
  vpMbKltMultiTracker::reinit();
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new model
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbEdgeKltMultiTracker::reInitModel(const vpImage<unsigned char>& I, const std::string &cad_name,
    const vpHomogeneousMatrix& cMo_, const bool verbose) {
  vpMbEdgeMultiTracker::reInitModel(I, cad_name, cMo_, verbose);
  vpMbKltMultiTracker::reInitModel(I, cad_name, cMo_, verbose);
}

/*!
  Re-initialize the model used by the tracker.

  \param I1 : The image containing the object to initialize for the first camera.
  \param I2 : The image containing the object to initialize for the second camera.
  \param cad_name : Path to the file containing the 3D model description.
  \param c1Mo : The new vpHomogeneousMatrix between the first camera and the new model.
  \param c2Mo : The new vpHomogeneousMatrix between the second camera and the new model.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
  \param firstCameraIsReference : If true, the first camera is the reference camera, otherwise it is the second one.
*/
void vpMbEdgeKltMultiTracker::reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &cad_name, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
    const bool verbose, const bool firstCameraIsReference) {
  vpMbEdgeMultiTracker::reInitModel(I1, I2, cad_name, c1Mo, c2Mo, verbose, firstCameraIsReference);
  vpMbKltMultiTracker::reInitModel(I1, I2, cad_name, c1Mo, c2Mo, verbose, firstCameraIsReference);
}

/*!
  Re-initialize the model used by the tracker.

  \param mapOfImages : Map of images.
  \param cad_name : Path to the file containing the 3D model description.
  \param mapOfCameraPoses : The new vpHomogeneousMatrix between the cameras and the current object position.
  \param verbose : Verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbEdgeKltMultiTracker::reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::string &cad_name, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
    const bool verbose) {
  vpMbEdgeMultiTracker::reInitModel(mapOfImages, cad_name, mapOfCameraPoses, verbose);
  vpMbKltMultiTracker::reInitModel(mapOfImages, cad_name, mapOfCameraPoses, verbose);
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void vpMbEdgeKltMultiTracker::resetTracker() {
  vpMbEdgeMultiTracker::resetTracker();
  vpMbKltMultiTracker::resetTracker();
}

/*!
  Set the angle used to test polygons appearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value lower than
  this parameter, the polygon is considered as appearing.
  The polygon will then be tracked.

  \param a : new angle in radian.
*/
void vpMbEdgeKltMultiTracker::setAngleAppear(const double &a) {
  vpMbEdgeMultiTracker::setAngleAppear(a);
  vpMbKltMultiTracker::setAngleAppear(a);
}

/*!
  Set the angle used to test polygons disappearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value greater than
  this parameter, the polygon is considered as disappearing.
  The tracking of the polygon will then be stopped.

  \param a : new angle in radian.
*/
void vpMbEdgeKltMultiTracker::setAngleDisappear(const double &a) {
  vpMbEdgeMultiTracker::setAngleDisappear(a);
  vpMbKltMultiTracker::setAngleDisappear(a);
}

/*!
  Set the camera parameters for the monocular case.

  \param camera : The new camera parameters.
*/
void vpMbEdgeKltMultiTracker::setCameraParameters(const vpCameraParameters& camera) {
  vpMbEdgeMultiTracker::setCameraParameters(camera);
  vpMbKltMultiTracker::setCameraParameters(camera);

  this->cam = camera;
}

/*!
  Set the camera parameters for the stereo cameras case.

  \param camera1 : The new camera parameters for the first camera.
  \param camera2 : The new camera parameters for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference, otherwise it is the second one.
*/
void vpMbEdgeKltMultiTracker::setCameraParameters(const vpCameraParameters& camera1, const vpCameraParameters& camera2,
    const bool firstCameraIsReference) {
  vpMbEdgeMultiTracker::setCameraParameters(camera1, camera2, firstCameraIsReference);
  vpMbKltMultiTracker::setCameraParameters(camera1, camera2, firstCameraIsReference);

  if(firstCameraIsReference) {
    this->cam = camera1;
  } else {
    this->cam = camera2;
  }
}

/*!
  Set the camera parameters for the specified camera.

  \param cameraName : Camera name.
  \param camera : The new camera parameters.
*/
void vpMbEdgeKltMultiTracker::setCameraParameters(const std::string &cameraName, const vpCameraParameters& camera) {
  vpMbEdgeMultiTracker::setCameraParameters(cameraName, camera);
  vpMbKltMultiTracker::setCameraParameters(cameraName, camera);

  if(cameraName == m_referenceCameraName) {
    this->cam = camera;
  }
}

/*!
  Set the camera parameters for all the cameras.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbEdgeKltMultiTracker::setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters) {
  vpMbEdgeMultiTracker::setCameraParameters(mapOfCameraParameters);
  vpMbKltMultiTracker::setCameraParameters(mapOfCameraParameters);

  for(std::map<std::string, vpCameraParameters>::const_iterator it = mapOfCameraParameters.begin();
      it != mapOfCameraParameters.end(); ++it) {
    if(it->first == m_referenceCameraName) {
      this->cam = it->second;
    }
  }
}

/*!
  Set the camera transformation matrix for the specified camera (\f$ _{}^{c_{current}}\textrm{M}_{c_{reference}} \f$).

  \param cameraName : Camera name.
  \param cameraTransformationMatrix : Camera transformation matrix between the current and the reference camera.
*/
void vpMbEdgeKltMultiTracker::setCameraTransformationMatrix(const std::string &cameraName,
    const vpHomogeneousMatrix &cameraTransformationMatrix) {
  vpMbEdgeMultiTracker::setCameraTransformationMatrix(cameraName, cameraTransformationMatrix);
  vpMbKltMultiTracker::setCameraTransformationMatrix(cameraName, cameraTransformationMatrix);

  std::map<std::string, vpHomogeneousMatrix>::iterator it = m_mapOfCameraTransformationMatrix.find(cameraName);
  if(it != m_mapOfCameraTransformationMatrix.end()) {
    it->second = cameraTransformationMatrix;
  } else {
    std::cerr << "Cannot find camera: " << cameraName << " !" << std::endl;
  }
}

/*!
  Set the map of camera transformation matrices
  (\f$ _{}^{c_1}\textrm{M}_{c_1}, _{}^{c_2}\textrm{M}_{c_1}, _{}^{c_3}\textrm{M}_{c_1}, \cdots, _{}^{c_n}\textrm{M}_{c_1} \f$).

  \param mapOfTransformationMatrix : map of camera transformation matrices.
*/
void vpMbEdgeKltMultiTracker::setCameraTransformationMatrix(
    const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix) {
  vpMbEdgeMultiTracker::setCameraTransformationMatrix(mapOfTransformationMatrix);
  vpMbKltMultiTracker::setCameraTransformationMatrix(mapOfTransformationMatrix);

  m_mapOfCameraTransformationMatrix = mapOfTransformationMatrix;
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags : New clipping flags.
*/
void vpMbEdgeKltMultiTracker::setClipping(const unsigned int &flags) {
  vpMbEdgeMultiTracker::setClipping(flags);
  vpMbKltMultiTracker::setClipping(flags);
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param cameraName : Camera to set the clipping.
  \param flags : New clipping flags.
*/
void vpMbEdgeKltMultiTracker::setClipping(const std::string &cameraName, const unsigned int &flags) {
  //Here, we do not change the general clipping flag
  vpMbEdgeMultiTracker::setClipping(cameraName, flags);
  vpMbKltMultiTracker::setClipping(cameraName, flags);
}

/*!
  Set if the covariance matrix has to be computed.

  \param flag : True if the covariance has to be computed, false otherwise
*/
void vpMbEdgeKltMultiTracker::setCovarianceComputation(const bool& flag) {
  vpMbEdgeMultiTracker::setCovarianceComputation(flag);
  vpMbKltMultiTracker::setCovarianceComputation(flag);
}

/*!
  Enable to display the moving edges (ME) and the klt features.

  Note that if present, the moving edges can be displayed with different colors:
  - If green : The ME is a good point.
  - If blue : The ME is removed because of a contrast problem during the tracking phase.
  - If purple : The ME is removed because of a threshold problem during the tracking phase.
  - If red : The ME is removed because it is rejected by the robust approach in the virtual visual servoing scheme.

  \param displayF : set it to true to display the features.
*/
void vpMbEdgeKltMultiTracker::setDisplayFeatures(const bool displayF) {
  vpMbEdgeMultiTracker::setDisplayFeatures(displayF);
  vpMbKltMultiTracker::setDisplayFeatures(displayF);
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.
*/
void vpMbEdgeKltMultiTracker::setFarClippingDistance(const double &dist) {
  vpMbEdgeMultiTracker::setFarClippingDistance(dist);
  vpMbKltMultiTracker::setFarClippingDistance(dist);
}

/*!
  Set the far distance for clipping for the specified camera.

  \param cameraName : Camera to set the far clipping.
  \param dist : Far clipping value.
*/
void vpMbEdgeKltMultiTracker::setFarClippingDistance(const std::string &cameraName, const double &dist) {
  vpMbEdgeMultiTracker::setFarClippingDistance(cameraName, dist);
  vpMbKltMultiTracker::setFarClippingDistance(cameraName, dist);
}

#ifdef VISP_HAVE_OGRE
/*!
  Set the ratio of visibility attempts that has to be successful to consider a polygon as visible.

  \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

  \param ratio : Ratio of succesful attempts that has to be considered. Value has to be between 0.0 (0%) and 1.0 (100%).
*/
  void vpMbEdgeKltMultiTracker::setGoodNbRayCastingAttemptsRatio(const double &ratio) {
    vpMbEdgeMultiTracker::setGoodNbRayCastingAttemptsRatio(ratio);
    vpMbKltMultiTracker::setGoodNbRayCastingAttemptsRatio(ratio);
  }

  /*!
    Set the number of rays that will be sent toward each polygon for visibility test.
    Each ray will go from the optic center of the camera to a random point inside the considered polygon.

    \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

    \param attempts Number of rays to be sent.
  */
  void vpMbEdgeKltMultiTracker::setNbRayCastingAttemptsForVisibility(const unsigned int &attempts) {
    vpMbEdgeMultiTracker::setNbRayCastingAttemptsForVisibility(attempts);
    vpMbKltMultiTracker::setNbRayCastingAttemptsForVisibility(attempts);
  }
#endif

/*!
  Set the flag to consider if the level of detail (LOD) is used for all the cameras.

  \param useLod : true if the level of detail must be used, false otherwise. When true,
  two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
  \param name : name of the face we want to modify the LOD parameter.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
 */
void vpMbEdgeKltMultiTracker::setLod(const bool useLod, const std::string &name) {
  vpMbEdgeMultiTracker::setLod(useLod, name);
  vpMbKltMultiTracker::setLod(useLod, name);
}

/*!
  Set the flag to consider if the level of detail (LOD) is used for all the cameras.

  \param useLod : true if the level of detail must be used, false otherwise. When true,
  two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
  \param cameraName : Name of the camera we want to set the LOD.
  \param name : name of the face we want to modify the LOD parameter, if empty all the faces are considered.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
 */
void vpMbEdgeKltMultiTracker::setLod(const bool useLod, const std::string &cameraName, const std::string &name) {
  vpMbEdgeMultiTracker::setLod(useLod, cameraName, name);
  vpMbKltMultiTracker::setLod(useLod, cameraName, name);
}

/*!
  Set the threshold for the minimum line length to be considered as visible in the LOD case.

  \param minLineLengthThresh : threshold for the minimum line length in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinPolygonAreaThresh()
 */
void vpMbEdgeKltMultiTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name) {
  vpMbEdgeMultiTracker::setMinLineLengthThresh(minLineLengthThresh, name);
}

/*!
  Set the threshold for the minimum line length to be considered as visible in the LOD case.

  \param minLineLengthThresh : threshold for the minimum line length in pixel.
  \param cameraName : name of the camera to consider.
  \param name : name of the face we want to modify the LOD threshold, if empty all the faces are considered.

  \sa setLod(), setMinPolygonAreaThresh()
 */
void vpMbEdgeKltMultiTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &cameraName,
    const std::string &name) {
  vpMbEdgeMultiTracker::setMinLineLengthThresh(minLineLengthThresh, cameraName, name);
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinLineLengthThresh()
 */
void vpMbEdgeKltMultiTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name) {
  vpMbEdgeMultiTracker::setMinPolygonAreaThresh(minPolygonAreaThresh, name);
  vpMbKltMultiTracker::setMinPolygonAreaThresh(minPolygonAreaThresh, name);
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param cameraName : name of the camera to consider.
  \param name : name of the face we want to modify the LOD threshold, if empty all the faces are considered.

  \sa setLod(), setMinLineLengthThresh()
 */
void vpMbEdgeKltMultiTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &cameraName,
    const std::string &name) {
  vpMbEdgeMultiTracker::setMinPolygonAreaThresh(minPolygonAreaThresh, cameraName, name);
  vpMbKltMultiTracker::setMinPolygonAreaThresh(minPolygonAreaThresh, cameraName, name);
}

/*!
  Set the near distance for clipping.

  \param dist : Near clipping value.
*/
void vpMbEdgeKltMultiTracker::setNearClippingDistance(const double &dist) {
  vpMbEdgeMultiTracker::setNearClippingDistance(dist);
  vpMbKltMultiTracker::setNearClippingDistance(dist);
}

/*!
  Set the near distance for clipping for the specified camera.

  \param cameraName : Camera name to set the near clipping distance.
  \param dist : Near clipping value.
*/
void vpMbEdgeKltMultiTracker::setNearClippingDistance(const std::string &cameraName, const double &dist) {
  vpMbEdgeMultiTracker::setNearClippingDistance(cameraName, dist);
  vpMbKltMultiTracker::setNearClippingDistance(cameraName, dist);
}

/*!
  Enable/Disable the appearance of Ogre config dialog on startup.

  \warning This method has only effect when Ogre is used and Ogre visibility test is
  enabled using setOgreVisibilityTest() with true parameter.

  \param showConfigDialog : if true, shows Ogre dialog window (used to set Ogre
  rendering options) when Ogre visibility is enabled. By default, this functionality
  is turned off.
*/
void vpMbEdgeKltMultiTracker::setOgreShowConfigDialog(const bool showConfigDialog) {
  vpMbEdgeMultiTracker::setOgreShowConfigDialog(showConfigDialog);
  vpMbKltMultiTracker::setOgreShowConfigDialog(showConfigDialog);
}

/*!
  Use Ogre3D for visibility tests

  \warning This function has to be called before the initialization of the tracker.

  \param v : True to use it, False otherwise
*/
void vpMbEdgeKltMultiTracker::setOgreVisibilityTest(const bool &v) {
  //Edge
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->faces.getOgreContext()->setWindowName("Multi Edge MBT Hybrid (" + it->first + ")");
  }
#endif


  //KLT
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->faces.getOgreContext()->setWindowName("Multi KLT MBT Hybrid (" + it->first + ")");
  }
#endif

  useOgre = v;
}

/*!
  Set the optimization method used during the tracking.

  \param opt : Optimization method to use.
*/
void vpMbEdgeKltMultiTracker::setOptimizationMethod(const vpMbtOptimizationMethod &opt) {
  vpMbEdgeMultiTracker::setOptimizationMethod(opt);
  vpMbKltMultiTracker::setOptimizationMethod(opt);
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I : image corresponding to the desired pose.
  \param cMo_ : Pose to affect.
*/
void vpMbEdgeKltMultiTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_) {
  if(m_mapOfEdgeTrackers.size() != 1 || m_mapOfKltTrackers.size() != 1) {
    std::cerr << "This method requires only 1 camera !" << std::endl;
  } else {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
    std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
    if(it_edge != m_mapOfEdgeTrackers.end() && it_klt != m_mapOfKltTrackers.end()) {
      it_edge->second->setPose(I, cMo_);
      it_klt->second->setPose(I, cMo_);

      this->cMo = cMo_;
      c0Mo = this->cMo;
      ctTc0.eye();
    } else {
      std::cerr << "Cannot find the reference camera: " << m_referenceCameraName << " !" << std::endl;
    }
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I1 : First image corresponding to the desired pose.
  \param I2 : Second image corresponding to the desired pose.
  \param c1Mo : First pose to affect.
  \param c2Mo : Second pose to affect.
  \param firstCameraIsReference : if true, the first camera is the reference, otherwise it is the second one.
*/
void vpMbEdgeKltMultiTracker::setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix c2Mo, const bool firstCameraIsReference) {
  vpMbEdgeMultiTracker::setPose(I1, I2, c1Mo, c2Mo, firstCameraIsReference);
  vpMbKltMultiTracker::setPose(I1, I2, c1Mo, c2Mo, firstCameraIsReference);
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images.
  \param cMo_ : Pose to affect to the reference camera.
*/
void vpMbEdgeKltMultiTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const vpHomogeneousMatrix &cMo_) {
  vpMbEdgeMultiTracker::setPose(mapOfImages, cMo_);
  vpMbKltMultiTracker::setPose(mapOfImages, cMo_);
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  Cameras that do not have pose will be automatically handled but the pose for the reference has to be passed in parameter.
  The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images corresponding to the desired pose.
  \param mapOfCameraPoses : Map of poses to affect.
*/
void vpMbEdgeKltMultiTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  vpMbEdgeMultiTracker::setPose(mapOfImages, mapOfCameraPoses);
  vpMbKltMultiTracker::setPose(mapOfImages, mapOfCameraPoses);
}

/*!
  Set if the projection error criteria has to be computed.

  \param flag : True if the projection error criteria has to be computed, false otherwise
*/
void vpMbEdgeKltMultiTracker::setProjectionErrorComputation(const bool &flag) {
  //Set the general flag for the current class
  vpMbTracker::setProjectionErrorComputation(flag);

  //Set the flag for each camera
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setProjectionErrorComputation(flag);
  }
}

/*!
  Set the reference camera name

  \param referenceCameraName : Name of the reference camera.
 */
void vpMbEdgeKltMultiTracker::setReferenceCameraName(const std::string &referenceCameraName) {
  vpMbEdgeMultiTracker::setReferenceCameraName(referenceCameraName);
  vpMbKltMultiTracker::setReferenceCameraName(referenceCameraName);
  m_referenceCameraName = referenceCameraName;
}

/*!
  Use Scanline algorithm for visibility tests

  \param v : True to use it, False otherwise
*/
void vpMbEdgeKltMultiTracker::setScanLineVisibilityTest(const bool &v) {
  vpMbEdgeMultiTracker::setScanLineVisibilityTest(v);
  vpMbKltMultiTracker::setScanLineVisibilityTest(v);
}

/*!
  Set the threshold for the acceptation of a point.

  \param th : Threshold for the weight below which a point is rejected.
*/
void vpMbEdgeKltMultiTracker::setThresholdAcceptation(const double th) {
  vpMbKltMultiTracker::setThresholdAcceptation(th);
}

void vpMbEdgeKltMultiTracker::testTracking() {
  std::cerr << "The method vpMbEdgeKltMultiTracker::testTracking is not used !" << std::endl;
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I : the input image
*/
void vpMbEdgeKltMultiTracker::track(const vpImage<unsigned char> &I) {
  //Track only with reference camera
  //Get the reference camera parameters
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_mbt = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);

  if(it_mbt != m_mapOfEdgeTrackers.end() && it_klt != m_mapOfKltTrackers.end()) {
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
    mapOfImages[m_referenceCameraName] = &I;
    track(mapOfImages);
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }

  //Set the projection error from the single camera
  if(computeProjError) {
    projectionError = it_mbt->second->getProjectionError();
  }
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I1 : The first image.
  \param I2 : The second image.
*/
void vpMbEdgeKltMultiTracker::track(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2) {
  if(m_mapOfEdgeTrackers.size() == 2 && m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
    //Assume that the first image is the first name in alphabetic order
    mapOfImages[it->first] = &I1;
    ++it;

    mapOfImages[it->first] = &I2;
    track(mapOfImages);
  } else {
    std::stringstream ss;
    ss << "Require two cameras ! There are " << m_mapOfKltTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param mapOfImages : Map of images.
*/
void vpMbEdgeKltMultiTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  //Reset the projectionError
  projectionError = 90.0;

  //Check if there is an image for each camera
  //mbt
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();
      it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(it_edge->first);

    if(it_img == mapOfImages.end()) {
      throw vpException(vpTrackingException::fatalError, "Missing images for edge trackers !");
    }
  }

  //klt
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(it_klt->first);

    if(it_img == mapOfImages.end()) {
      throw vpException(vpTrackingException::fatalError, "Missing images for KLT trackers !");
    }
  }

  try {
    vpMbKltMultiTracker::preTracking(mapOfImages);
  } catch(...) { }

  //MBT: track moving edges
  trackMovingEdges(mapOfImages);

  computeVVS(mapOfImages);

  postTracking(mapOfImages, 0);

  if(computeProjError) {
    vpMbEdgeMultiTracker::computeProjectionError();
  }
}

void vpMbEdgeKltMultiTracker::trackMovingEdges(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  vpMbEdgeTracker *edge;
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;
    //Track moving edges
    try {
      edge->trackMovingEdge(*mapOfImages[it->first]);
    } catch(...) {
      std::cerr << "Error in moving edge tracking" << std::endl;
      throw;
    }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(dummy_vpMbEdgeKltMultiTracker.cpp.o) has no symbols
void dummy_vpMbEdgeKltMultiTracker() {};
#endif //VISP_HAVE_OPENCV
