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
 * Model-based edge tracker with multiple cameras.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpMbEdgeMultiTracker.cpp
  \brief Model-based edge tracker with multiple cameras.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/mbt/vpMbEdgeMultiTracker.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>


/*!
  Basic constructor
*/
vpMbEdgeMultiTracker::vpMbEdgeMultiTracker() :
    m_mapOfCameraTransformationMatrix(), m_mapOfEdgeTrackers(), m_mapOfPyramidalImages(), m_referenceCameraName("Camera"),
    m_L_edgeMulti(), m_error_edgeMulti(), m_w_edgeMulti(), m_weightedError_edgeMulti(), m_factor()
{
  m_mapOfEdgeTrackers["Camera"] = new vpMbEdgeTracker();

  //Add default camera transformation matrix
  m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
}

/*!
  Construct a vpMbEdgeMultiTracker with the specified number of cameras.

  \param nbCameras : Number of cameras to use.
*/
vpMbEdgeMultiTracker::vpMbEdgeMultiTracker(const unsigned int nbCameras) :
    m_mapOfCameraTransformationMatrix(), m_mapOfEdgeTrackers(), m_mapOfPyramidalImages(), m_referenceCameraName("Camera"),
    m_L_edgeMulti(), m_error_edgeMulti(), m_w_edgeMulti(), m_weightedError_edgeMulti(), m_factor()
{

  if(nbCameras == 0) {
    throw vpException(vpTrackingException::fatalError, "Cannot construct a vpMbEdgeMultiTracker with no camera !");
  } else if (nbCameras == 1) {
    m_mapOfEdgeTrackers["Camera"] = new vpMbEdgeTracker();

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else if(nbCameras == 2) {
    m_mapOfEdgeTrackers["Camera1"] = new vpMbEdgeTracker();
    m_mapOfEdgeTrackers["Camera2"] = new vpMbEdgeTracker();

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera1"] = vpHomogeneousMatrix();
    m_mapOfCameraTransformationMatrix["Camera2"] = vpHomogeneousMatrix();

    //Set by default the reference camera to Camera1
    m_referenceCameraName = "Camera1";
  } else {
    for(unsigned int i = 1; i <= nbCameras; i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfEdgeTrackers[ss.str()] = new vpMbEdgeTracker();

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfEdgeTrackers.begin()->first;
  }
}

/*!
  Construct a vpMbEdgeMultiTracker with the specified list of camera names.
  By default, the first camera name by alphabetic order is the reference camera.

  \param cameraNames : List of camera names.
*/
vpMbEdgeMultiTracker::vpMbEdgeMultiTracker(const std::vector<std::string> &cameraNames) :
    m_mapOfCameraTransformationMatrix(), m_mapOfEdgeTrackers(), m_mapOfPyramidalImages(), m_referenceCameraName("Camera"),
    m_L_edgeMulti(), m_error_edgeMulti(), m_w_edgeMulti(), m_weightedError_edgeMulti(), m_factor()
{

  if(cameraNames.empty()) {
    throw vpException(vpTrackingException::fatalError, "Cannot construct a vpMbEdgeMultiTracker with no camera !");
  }

  for(std::vector<std::string>::const_iterator it = cameraNames.begin(); it != cameraNames.end(); ++it) {
    m_mapOfEdgeTrackers[*it] = new vpMbEdgeTracker();
  }

  //Set by default the reference camera
  m_referenceCameraName = cameraNames.front();
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbEdgeMultiTracker::~vpMbEdgeMultiTracker() {
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    delete it->second;
  }

  m_mapOfEdgeTrackers.clear();

  cleanPyramid(m_mapOfPyramidalImages);
}

void vpMbEdgeMultiTracker::cleanPyramid(std::map<std::string, std::vector<const vpImage<unsigned char>* > >& pyramid) {
  for(std::map<std::string, std::vector<const vpImage<unsigned char>* > >::iterator it1 = pyramid.begin();
      it1 != pyramid.end(); ++it1) {
    if(it1->second.size() > 0){
      it1->second[0] = NULL;
      for(size_t i = 1; i < it1->second.size(); i++) {
        if(it1->second[i] != NULL) {
          delete it1->second[i];
          it1->second[i] = NULL;
        }
      }
      it1->second.clear();
    }
  }
}

void vpMbEdgeMultiTracker::computeProjectionError() {
  if(computeProjError) {
    double rawTotalProjectionError = 0.0;
    unsigned int nbTotalFeaturesUsed = 0;
    for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
        it != m_mapOfEdgeTrackers.end(); ++it) {
      double curProjError = it->second->getProjectionError();
      unsigned int nbFeaturesUsed = it->second->nbFeaturesForProjErrorComputation;

      if(nbFeaturesUsed > 0) {
        nbTotalFeaturesUsed += nbFeaturesUsed;
        rawTotalProjectionError += ( vpMath::rad(curProjError)*nbFeaturesUsed );
      }
    }

    if(nbTotalFeaturesUsed > 0) {
      nbFeaturesForProjErrorComputation = nbTotalFeaturesUsed;
      projectionError = vpMath::deg(rawTotalProjectionError / (double)nbTotalFeaturesUsed);
    } else {
      nbFeaturesForProjErrorComputation = 0;
      projectionError = 90.0;
    }
  }
}

void vpMbEdgeMultiTracker::computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const unsigned int lvl) {
  computeVVSInit();
  unsigned int nbrow = m_error_edgeMulti.getRows();

  unsigned int iter = 0;
  //Parametre pour la premiere phase d'asservissement
  bool reloop = true;

  bool isoJoIdentity_ = isoJoIdentity; // Backup since it can be modified if L is not full rank

  std::map<std::string, vpVelocityTwistMatrix> mapOfVelocityTwist;
  for(std::map<std::string, vpHomogeneousMatrix>::const_iterator it = m_mapOfCameraTransformationMatrix.begin();
      it != m_mapOfCameraTransformationMatrix.end(); ++it) {
    vpVelocityTwistMatrix cVo;
    cVo.buildFrom(it->second);
    mapOfVelocityTwist[it->first] = cVo;
  }

//  std::cout << "\n\n\ncMo used before the first phase=\n" << cMo << std::endl;

  /*** First phase ***/

  vpMbEdgeTracker *edge;
  while(reloop == true && iter < 10) {
    for (std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
        it != m_mapOfEdgeTrackers.end(); ++it) {
      it->second->cMo = m_mapOfCameraTransformationMatrix[it->first] * cMo;
    }

    double count = 0;
    reloop = false;

    unsigned int start_idx = 0;
    for (std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
      double count_tmp = 0.0;
      edge = it->second;
      edge->computeVVSFirstPhase(*mapOfImages[it->first], iter, count_tmp, lvl);
      count += count_tmp;

      m_L_edgeMulti.insert(edge->m_L_edge*mapOfVelocityTwist[it->first], start_idx, 0);
      m_factor.insert(start_idx, edge->m_factor);
      m_w_edgeMulti.insert(start_idx, edge->m_w_edge);
      m_error_edgeMulti.insert(start_idx, edge->m_error_edge);

      start_idx += edge->m_error_edge.getRows();
    }

    count = count / (double) nbrow;
    if (count < 0.85) {
      reloop = true;
    }

    computeVVSFirstPhasePoseEstimation(iter, isoJoIdentity_);

    iter++;
  }

//  std::cout << "\n\t First minimization in " << iter << " iteration give as initial cMo: \n" << cMo << std::endl;
//  std::cout << "Residual=" << m_error.sum() / m_error.size() << std::endl;


  /*** Second phase ***/

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;
    edge->m_w_edge = 1;
  }

  iter = 0;

  vpHomogeneousMatrix cMoPrev;
  vpColVector W_true(nbrow);
  vpMatrix L_true;
  vpMatrix LVJ_true;

  double mu = m_initialMu;
  vpColVector m_error_prev;
  vpColVector m_w_prev;

  double residu_1 = 1e3;
  double r =1e3-1;


  //For computeVVSPoseEstimation
  vpColVector LTR;
  vpColVector v;
  vpMatrix LTL;


  //while ( ((int)((residu_1 - r)*1e8) != 0 )  && (iter<30))
  while (std::fabs((residu_1 - r)*1e8) > std::numeric_limits<double>::epsilon() && (iter < m_maxIter)) {
    computeVVSInteractionMatrixAndResidu(mapOfImages, mapOfVelocityTwist);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_edgeMulti, m_error_prev, cMoPrev, mu, reStartFromLastIncrement, &m_w_prev);

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      L_true = m_L_edgeMulti;
      vpVelocityTwistMatrix cVo;

      if (computeCovariance) {
          L_true = m_L_edgeMulti;
         if (!isoJoIdentity_) {
           cVo.buildFrom(cMo);
           LVJ_true = (m_L_edgeMulti*cVo*oJo);
         }
      }

      double wi = 0.0, eri = 0.0;
      double num = 0.0, den = 0.0;
      if ((iter==0) || m_computeInteraction) {
        for (unsigned int i = 0; i < nbrow; i++) {
          wi = m_w_edgeMulti[i]*m_factor[i];
          W_true[i] = wi;
          eri = m_error_edgeMulti[i];
          num += wi*vpMath::sqr(eri);
          den += wi;

          m_weightedError_edgeMulti[i] =  wi*eri ;

          for (unsigned int j = 0; j < 6; j++) {
            m_L_edgeMulti[i][j] = wi*m_L_edgeMulti[i][j];
          }
        }
      } else {
        for (unsigned int i = 0; i < nbrow; i++) {
          wi = m_w_edgeMulti[i]*m_factor[i];
          W_true[i] = wi;
          eri = m_error_edgeMulti[i];
          num += wi*vpMath::sqr(eri);
          den += wi;

          m_weightedError_edgeMulti[i] =  wi*eri ;
        }
      }

      residu_1 = r;
      r = sqrt(num/den); //Le critere d'arret prend en compte le poids

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L_edgeMulti, LTL, m_weightedError_edgeMulti, m_error_edgeMulti, m_error_prev, LTR, mu, v, &m_w_edgeMulti, &m_w_prev);


      cMoPrev = cMo;
      cMo =  vpExponentialMap::direct(v).inverse() * cMo;
    }

    iter++;
  }

// std::cout << "VVS estimate pose cMo:\n" << cMo << std::endl;

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMoPrev, L_true, LVJ_true, m_error_edgeMulti);

  for (std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    edge = it->second;
    edge->updateMovingEdgeWeights();
  }
}

void vpMbEdgeMultiTracker::computeVVSFirstPhasePoseEstimation(const unsigned int iter, bool &isoJoIdentity_) {
  unsigned int nerror = m_weightedError_edgeMulti.getRows();

  double wi, eri;
  if((iter==0) || m_computeInteraction) {
    for (unsigned int i = 0; i < nerror; i++) {
      wi = m_w_edgeMulti[i]*m_factor[i];
      eri = m_error_edgeMulti[i];

      m_weightedError_edgeMulti[i] =  wi*eri;

      for (unsigned int j = 0; j < 6; j++) {
        m_L_edgeMulti[i][j] = wi*m_L_edgeMulti[i][j];
      }
    }
  } else {
    for(unsigned int i = 0; i < nerror; i++) {
      wi = m_w_edgeMulti[i]*m_factor[i];
      eri = m_error_edgeMulti[i];

      m_weightedError_edgeMulti[i] =  wi*eri;
    }
  }

  vpVelocityTwistMatrix cVo;

  // If all the 6 dof should be estimated, we check if the interaction matrix is full rank.
  // If not we remove automatically the dof that cannot be estimated
  // This is particularly useful when consering circles (rank 5) and cylinders (rank 4)
  if (isoJoIdentity_) {
    cVo.buildFrom(cMo);

    vpMatrix K; // kernel
    unsigned int rank = (m_L_edgeMulti*cVo).kernel(K);
    if(rank == 0) {
      throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
    }
    if (rank != 6) {
      vpMatrix I; // Identity
      I.eye(6);
      oJo = I-K.AtA();

      isoJoIdentity_ = false;
    }
  }

  vpColVector v;
  vpMatrix LTL;
  vpColVector LTR;

  if(isoJoIdentity_){
      LTL = m_L_edgeMulti.AtA();
      computeJTR(m_L_edgeMulti, m_weightedError_edgeMulti, LTR);
      v = -0.7*LTL.pseudoInverse(LTL.getRows()*std::numeric_limits<double>::epsilon())*LTR;
  }
  else{
      cVo.buildFrom(cMo);
      vpMatrix LVJ = (m_L_edgeMulti*cVo*oJo);
      vpMatrix LVJTLVJ = (LVJ).AtA();
      vpColVector LVJTR;
      computeJTR(LVJ, m_weightedError_edgeMulti, LVJTR);
      v = -0.7*LVJTLVJ.pseudoInverse(LVJTLVJ.getRows()*std::numeric_limits<double>::epsilon())*LVJTR;
      v = cVo * v;
  }

  cMo =  vpExponentialMap::direct(v).inverse() * cMo;
}

void vpMbEdgeMultiTracker::computeVVSInit() {
  unsigned int nbrow = 0;

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    vpMbEdgeTracker *edge = it->second;

    try {
      edge->computeVVSInit();
      nbrow += edge->m_error_edge.getRows();
    } catch (...) {  }
  }

  if (nbrow < 4) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "No data found to compute the interaction matrix...");
  }

  //Initialize with correct size
  m_L_edgeMulti.resize(nbrow, 6, false);
  m_w_edgeMulti.resize(nbrow, false);
  m_error_edgeMulti.resize(nbrow, false);
  m_weightedError_edgeMulti.resize(nbrow, false);
  m_factor.resize(nbrow, false);
}

void vpMbEdgeMultiTracker::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbEdgeMultiTracker::computeVVSInteractionMatrixAndResidu() should not be called!");
}

void vpMbEdgeMultiTracker::computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                                std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist) {
  unsigned int start_idx = 0;

  for (std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    vpMbEdgeTracker *edge = it->second;
    edge->cMo = m_mapOfCameraTransformationMatrix[it->first]*cMo;

    edge->computeVVSInteractionMatrixAndResidu(*mapOfImages[it->first]);

    m_L_edgeMulti.insert(edge->m_L_edge*mapOfVelocityTwist[it->first], start_idx, 0);
    m_error_edgeMulti.insert(start_idx, edge->m_error_edge);

    start_idx += edge->m_error_edge.getRows();
  }
}

void vpMbEdgeMultiTracker::computeVVSWeights() {
  unsigned int start_idx = 0;

  for (std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin(); it != m_mapOfEdgeTrackers.end(); ++it) {
    vpMbEdgeTracker *edge = it->second;

    //Compute weights
    edge->computeVVSWeights();

    m_w_edgeMulti.insert(start_idx, edge->m_w_edge);
    start_idx += edge->m_w_edge.getRows();
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
void vpMbEdgeMultiTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_,
    const vpCameraParameters &cam_, const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->display(I, cMo_, cam_, col, thickness, displayFullModel);
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
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
*/
void vpMbEdgeMultiTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &cam_,
    const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->display(I, cMo_, cam_, col, thickness, displayFullModel);
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera (stereo cameras configuration).

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image.
  \param c1Mo : Pose used to project the 3D model into the first image.
  \param c2Mo : Pose used to project the 3D model into the second image.
  \param cam1 : The first camera parameters.
  \param cam2 : The second camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbEdgeMultiTracker::display(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->display(I1, c1Mo, cam1, col, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, col, thickness, displayFullModel);
  } else {
    std::cerr << "This display is only for the stereo case ! There are "
        << m_mapOfEdgeTrackers.size() << " camera !" << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera (stereo cameras configuration).

  \param I1 : The first color image.
  \param I2 : The second color image.
  \param c1Mo : Pose used to project the 3D model into the first image.
  \param c2Mo : Pose used to project the 3D model into the second image.
  \param cam1 : The first camera parameters.
  \param cam2 : The second camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbEdgeMultiTracker::display(const vpImage<vpRGBa>& I1, const vpImage<vpRGBa>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->display(I1, c1Mo, cam1, col, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, col, thickness, displayFullModel);
  } else {
    std::cerr << "This display is only for the stereo case ! There are "
        << m_mapOfEdgeTrackers.size() << " cameras !" << std::endl;
  }
}

/*!
  Display the 3D model from a given position of the camera (multiple cameras configuration).

  \param mapOfImages : Map of grayscale images.
  \param mapOfCameraPoses : Map of camera poses.
  \param mapOfCameraParameters : Map of camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbEdgeMultiTracker::display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
      const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
      const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  //Display only for the given images
  for(std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.begin();
      it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if(it_edge != m_mapOfEdgeTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
      it_edge->second->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements !" << std::endl;
    }
  }
}

/*!
  Display the 3D model from a given position of the camera (multiple cameras configuration).

  \param mapOfImages : Map of color images.
  \param mapOfCameraPoses : Map of camera poses.
  \param mapOfCameraParameters : Map of camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible faces).
*/
void vpMbEdgeMultiTracker::display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
      const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
      const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  //Display only for the given images
  for(std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img = mapOfImages.begin();
      it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if(it_edge != m_mapOfEdgeTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
      it_edge->second->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements !" << std::endl;
    }
  }
}

/*!
  Get the camera names.

  \return The vector of camera names.
*/
std::vector<std::string> vpMbEdgeMultiTracker::getCameraNames() const {
  std::vector<std::string> cameraNames;

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();
      it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    cameraNames.push_back(it_edge->first);
  }

  return cameraNames;
}

/*!
  Get the camera parameters for the mono/reference camera.

  \param camera : Copy of the camera parameters used by the tracker.
*/
void vpMbEdgeMultiTracker::getCameraParameters(vpCameraParameters &camera) const {
  //Get the reference camera parameters
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getCameraParameters(camera);
  } else {
    std::cerr << "The reference camera name: " << m_referenceCameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the camera parameters for the stereo cameras case.

  \param cam1 : Copy of the camera parameters for the first camera.
  \param cam2 : Copy of the camera parameters for the second camera.
*/
void vpMbEdgeMultiTracker::getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const {
  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->getCameraParameters(cam1);
    ++it;

    it->second->getCameraParameters(cam2);
  } else {
    std::cerr << "Problem with the number of cameras ! There are "
        << m_mapOfEdgeTrackers.size() << " cameras !" << std::endl;
  }
}

/*!
  Get the camera parameters specified by its name.

  \param cameraName : Name of the camera.
  \param camera : Copy of the camera parameters.
*/
void vpMbEdgeMultiTracker::getCameraParameters(const std::string &cameraName, vpCameraParameters &camera) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getCameraParameters(camera);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get all the camera parameters.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbEdgeMultiTracker::getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const {
  //Clear the input map
  mapOfCameraParameters.clear();

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    vpCameraParameters cam_;
    it->second->getCameraParameters(cam_);
    mapOfCameraParameters[it->first] = cam_;
  }
}

/*!
  Get the clipping used and defined in vpPolygon3D::vpMbtPolygonClippingType for the given camera name.

  \param cameraName : Name of the desired camera.
  \return Clipping flags.
*/
unsigned int vpMbEdgeMultiTracker::getClipping(const std::string &cameraName) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    return it->second->getClipping();
  } else {
    std::cerr << "Cannot find camera: " << cameraName << std::endl;
  }

  return vpMbTracker::getClipping();
}

/*!
  Return a reference to the faces structure.

  \return Reference to the face structure.
 */
vpMbHiddenFaces<vpMbtPolygon>& vpMbEdgeMultiTracker::getFaces() {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    return it->second->getFaces();
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " cannot be found !" << std::endl;
  return faces;
}

/*!
  Return a reference to the faces structure for the given camera name.

  \return Reference to the face structure.
 */
vpMbHiddenFaces<vpMbtPolygon>& vpMbEdgeMultiTracker::getFaces(const std::string &cameraName) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    return it->second->getFaces();
  }

  std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  return faces;
}

/*!
  Return a map of faces structure for each camera.

  \return Reference a map of the face structure for each camera.
 */
std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > vpMbEdgeMultiTracker::getFaces() const {
  std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > mapOfFaces;
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    mapOfFaces[it->first] = it->second->faces;
  }

  return mapOfFaces;
}

/*!
  Get the list of the circles tracked for the specified level. Each circle
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param level : Level corresponding to the list to return.
  \param circlesList : The list of the circles of the model.
*/
void vpMbEdgeMultiTracker::getLcircle(std::list<vpMbtDistanceCircle *>& circlesList, const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);

  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->getLcircle(circlesList, level);
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }
}

/*!
  Get the list of the circles tracked for the specified level. Each circle
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceCircle.
  \param level : Level corresponding to the list to return.
  \param circlesList : The list of the circles of the model.
*/
void vpMbEdgeMultiTracker::getLcircle(const std::string &cameraName, std::list<vpMbtDistanceCircle *>& circlesList,
    const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getLcircle(circlesList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param level : Level corresponding to the list to return.
  \param cylindersList : The list of the cylinders of the model.
*/
void vpMbEdgeMultiTracker::getLcylinder(std::list<vpMbtDistanceCylinder *>& cylindersList, const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);

  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->getLcylinder(cylindersList, level);
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }
}

/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceCylinder.
  \param level : Level corresponding to the list to return.
  \param cylindersList : The list of the cylinders of the model.
*/
void vpMbEdgeMultiTracker::getLcylinder(const std::string &cameraName, std::list<vpMbtDistanceCylinder *>& cylindersList,
    const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getLcylinder(cylindersList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the list of the lines tracked for the specified level. Each line contains
  the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param level : Level corresponding to the list to return.
  \param linesList : The list of the lines of the model.
*/
void vpMbEdgeMultiTracker::getLline(std::list<vpMbtDistanceLine *>& linesList, const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);

  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->getLline(linesList, level);
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }
}

/*!
  Get the list of the lines tracked for the specified level. Each line contains
  the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param cameraName : Camera name for which we want to get the list of vpMbtDistanceLine.
  \param level : Level corresponding to the list to return.
  \param linesList : The list of the lines of the model.
*/
void vpMbEdgeMultiTracker::getLline(const std::string &cameraName, std::list<vpMbtDistanceLine *>& linesList,
    const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getLline(linesList, level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the moving edge parameters for the reference camera.

  \param p_me : Moving edge parameters for the reference camera.

*/
void vpMbEdgeMultiTracker::getMovingEdge(vpMe &p_me) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->getMovingEdge(p_me);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the moving edge parameters for the reference camera.

  \return an instance of the moving edge parameters used by the tracker.
*/
vpMe vpMbEdgeMultiTracker::getMovingEdge() const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  vpMe me_tmp;
  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->getMovingEdge(me_tmp);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist !" << std::endl;
  }

  return me_tmp;
}

/*!
  Get the moving edge parameters for the specified camera name.

  \param cameraName : Name of the camera.
  \param p_me : Moving edge parameters for the specified camera name.

*/
void vpMbEdgeMultiTracker::getMovingEdge(const std::string &cameraName, vpMe &p_me) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getMovingEdge(p_me);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the moving edge parameters for the specified camera name.

  \param cameraName : Name of the camera.

  \return an instance of the moving edge parameters used by the tracker.
*/
vpMe vpMbEdgeMultiTracker::getMovingEdge(const std::string &cameraName) const {
  vpMe me_tmp;
  getMovingEdge(cameraName, me_tmp);
  return me_tmp;
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a
  vpMeSite with its flag "state" equal to 0. Only these points are used
  during the virtual visual servoing stage.

  \param level : Pyramid level to consider.

  \exception vpException::dimensionError if level does not represent a used
  level.

  \return the number of good points for the reference camera.
*/
unsigned int vpMbEdgeMultiTracker::getNbPoints(const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);

  unsigned int nbGoodPoints = 0;
  if (it_edge != m_mapOfEdgeTrackers.end()) {

    nbGoodPoints += it_edge->second->getNbPoints(level);
  } else {
    std::cerr << "The reference camera: " << m_referenceCameraName << " does not exist !" << std::endl;
  }

  return nbGoodPoints;
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a
  vpMeSite with its flag "state" equal to 0. Only these points are used
  during the virtual visual servoing stage.

  \param cameraName : Camera name.
  \param level : Pyramid level to consider.

  \exception vpException::dimensionError if level does not represent a used
  level.

  \return the number of good points for the specified camera name.
*/
unsigned int vpMbEdgeMultiTracker::getNbPoints(const std::string &cameraName, const unsigned int level) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it =
      m_mapOfEdgeTrackers.find(cameraName);
  if (it != m_mapOfEdgeTrackers.end()) {
    return it->second->getNbPoints(level);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !"
        << std::endl;
  }

  return 0;
}

/*!
  Get the number of polygons (faces) representing the object to track.

  \return Number of polygons.
*/
unsigned int vpMbEdgeMultiTracker::getNbPolygon() const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    return it->second->getNbPolygon();
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " cannot be found !" << std::endl;
  return 0;
}

/*!
  Get the number of polygons (faces) representing the object to track for the specified camera.

  \return Number of polygons for the specified camera.
*/
unsigned int vpMbEdgeMultiTracker::getNbPolygon(const std::string &cameraName) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    return it->second->getNbPolygon();
  }

  std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  return 0;
}

/*!
  Get the number of polygons (faces) representing the object to track for all the cameras.

  \return Number of polygons for the specified camera.
*/
std::map<std::string, unsigned int> vpMbEdgeMultiTracker::getMultiNbPolygon() const {
  std::map<std::string, unsigned int> mapOfNbPolygons;
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    mapOfNbPolygons[it->first] = it->second->getNbPolygon();
  }

  return mapOfNbPolygons;
}

/*!
  Get the current pose between the object and the cameras.

  \param c1Mo : The camera pose for the first camera.
  \param c2Mo : The camera pose for the second camera.
*/
void vpMbEdgeMultiTracker::getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const {
  if (m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it =
        m_mapOfEdgeTrackers.begin();
    it->second->getPose(c1Mo);
    ++it;

    it->second->getPose(c2Mo);
  } else {
    std::cerr << "Require two cameras ! There are "
        << m_mapOfEdgeTrackers.size() << " cameras !" << std::endl;
  }
}

/*!
  Get the current pose between the object and the camera.
  cMo is the matrix which can be used to express
  coordinates from the object frame to camera frame.

  \param cameraName : The name of the camera.
  \param cMo_ : The camera pose for the specified camera.
*/
void vpMbEdgeMultiTracker::getPose(const std::string &cameraName, vpHomogeneousMatrix &cMo_) const {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getPose(cMo_);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the current pose between the object and the cameras.

  \param mapOfCameraPoses : The map of camera poses for all the cameras.
*/
void vpMbEdgeMultiTracker::getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const {
  //Clear the map
  mapOfCameraPoses.clear();

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    vpHomogeneousMatrix cMo_;
    it->second->getPose(cMo_);
    mapOfCameraPoses[it->first] = cMo_;
  }
}

void vpMbEdgeMultiTracker::init(const vpImage<unsigned char>& /*I*/) {
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
void vpMbEdgeMultiTracker::initClick(const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
    const std::string &displayFile) {
  if(m_mapOfEdgeTrackers.empty()) {
    throw vpException(vpTrackingException::initializationError, "There is no camera !");
  } else if(m_mapOfEdgeTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "There is more than one camera !");
  } else {
    //Get the vpMbEdgeTracker object for the reference camera name
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
    if(it != m_mapOfEdgeTrackers.end()) {
      it->second->initClick(I, points3D_list, displayFile);
      it->second->getPose(cMo);
    } else {
      std::stringstream ss;
      ss << "Cannot initClick as the reference camera: " << m_referenceCameraName << " does not exist !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
  }
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
void vpMbEdgeMultiTracker::initClick(const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp) {
  if(m_mapOfEdgeTrackers.empty()) {
    throw vpException(vpTrackingException::initializationError, "There is no camera !");
  } else if(m_mapOfEdgeTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "There is more than one camera !");
  } else {
    //Get the vpMbEdgeTracker object for the reference camera name
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
    if(it != m_mapOfEdgeTrackers.end()) {
      it->second->initClick(I, initFile, displayHelp);
      it->second->getPose(cMo);
    } else {
      std::stringstream ss;
      ss << "Cannot initClick as the reference camera: " << m_referenceCameraName << " does not exist !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
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
void vpMbEdgeMultiTracker::initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string& initFile1, const std::string& initFile2, const bool displayHelp, const bool firstCameraIsReference) {
  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->initClick(I1, initFile1, displayHelp);

    if(firstCameraIsReference) {
      //Set the reference cMo
      it->second->getPose(cMo);

      //Set the reference camera parameters
      it->second->getCameraParameters(this->cam);
    }

    ++it;

    it->second->initClick(I2, initFile2, displayHelp);

    if(!firstCameraIsReference) {
      //Set the reference cMo
      it->second->getPose(cMo);

      //Set the reference camera parameters
      it->second->getCameraParameters(this->cam);
    }
  } else {
    std::stringstream ss;
    ss << "Cannot init click ! Require two cameras but there are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str());
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

  \param mapOfImages : Map of images.
  \param initFile : File containing the points where to click for the reference camera.
  \param displayHelp : Optionnal display of an image that should have the same generic name
  as the init file (ie teabox.ppm). This image may be used to show where to click. This
  functionality is only available if visp_io module is used.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.

*/
void vpMbEdgeMultiTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::string &initFile, const bool displayHelp) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it_edge != m_mapOfEdgeTrackers.end()) {
    std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);

    if(it_img != mapOfImages.end()) {
      //Init click on reference camera
      it_edge->second->initClick(*it_img->second, initFile, displayHelp);

      //Set the reference cMo
      it_edge->second->getPose(cMo);

      //Set the pose for the others cameras
      for(it_edge = m_mapOfEdgeTrackers.begin(); it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
        if(it_edge->first != m_referenceCameraName) {
          it_img = mapOfImages.find(it_edge->first);
          std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
              m_mapOfCameraTransformationMatrix.find(it_edge->first);

          if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
            vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
            it_edge->second->setPose(*it_img->second, cCurrentMo);
          } else {
            std::stringstream ss;
            ss << "Cannot init click ! Missing image for camera: " << m_referenceCameraName << " !";
            throw vpException(vpTrackingException::initializationError, ss.str());
          }
        }
      }
    } else {
      std::stringstream ss;
      ss << "Cannot init click ! Missing image for camera: " << m_referenceCameraName << " !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
  } else {
    std::stringstream ss;
    ss << "Cannot init click ! The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str());
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
  \param mapOfInitFiles : map of files containing the points where to click for each camera.
  \param displayHelp : Optional display of an image (ie teabox.ppm). This
  image may be used to show where to click.

  \exception vpException::ioError : The file specified in \e initFile doesn't exist.
*/
void vpMbEdgeMultiTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initFile = mapOfInitFiles.find(m_referenceCameraName);

  if(it_edge != m_mapOfEdgeTrackers.end() && it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
    //InitClick for the reference camera
    it_edge->second->initClick(*it_img->second, it_initFile->second, displayHelp);

    //Get reference camera pose
    it_edge->second->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick for the reference camera !");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //InitClick for all the cameras that have an initFile
  for(it_edge = m_mapOfEdgeTrackers.begin(); it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    if(it_edge->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_edge->first);
      it_initFile = mapOfInitFiles.find(it_edge->first);

      if(it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
        it_edge->second->initClick(*it_img->second, it_initFile->second, displayHelp);
      } else {
        vectorOfMissingCameraPoses.push_back(it_edge->first);
      }
    }
  }

  //SetPose for cameras that do not have an initFile
  for(std::vector<std::string>::const_iterator it1 = vectorOfMissingCameraPoses.begin();
      it1 != vectorOfMissingCameraPoses.end(); ++it1) {
    it_img = mapOfImages.find(*it1);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it1);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfEdgeTrackers[*it1]->setPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix ! Cannot set the pose for camera: " << (*it1) << " !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
  }
}
#endif //#ifdef VISP_HAVE_MODULE_GUI

/*!
  Initialize the tracking thanks to the pose in vpPoseVector format, and read in the file initFile.
  The structure of this file is (without the comments):
  \code
  // The six value of the pose vector
  0.0000    //  \
  0.0000    //  |
  1.0000    //  | Example of value for the pose vector where Z = 1 meter
  0.0000    //  |
  0.0000    //  |
  0.0000    //  /
  \endcode

  Where the three firsts lines refer to the translation and the three last to the rotation in thetaU
  parameterization (see vpThetaUVector).
  \param I : Input image
  \param initFile : Path to the file containing the pose.
*/
void vpMbEdgeMultiTracker::initFromPose(const vpImage<unsigned char>& I, const std::string &initFile) {
  //Monocular case only !
  if(m_mapOfEdgeTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "This method can only be used for the monocular case !");
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

  //Set the new pose for the reference camera
  cMo.buildFrom(init_pos);

  //Init for the reference camera
  std::map<std::string, vpMbEdgeTracker *>::iterator it_ref = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it_ref == m_mapOfEdgeTrackers.end()) {
    throw vpException(vpTrackingException::initializationError, "Cannot find the reference camera !");
  }

  it_ref->second->cMo = cMo;
  it_ref->second->init(I);
}

/*!
  Initialize the tracking thanks to the pose.

  \param I : Input image
  \param cMo_ : Pose matrix.
*/
void vpMbEdgeMultiTracker::initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_) {
  if(m_mapOfEdgeTrackers.size() != 1) {
    std::stringstream ss;
    ss << "This method requires exactly one camera, there are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }

  this->cMo = cMo_;

  //Init for the reference camera
  std::map<std::string, vpMbEdgeTracker *>::iterator it_ref = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it_ref == m_mapOfEdgeTrackers.end()) {
    throw vpException(vpTrackingException::initializationError, "Cannot find the reference camera !");
  }

  it_ref->second->cMo = cMo;
  it_ref->second->init(I);
}

/*!
  Initialize the tracking thanks to the pose vector.

  \param I : Input image
  \param cPo : Pose vector.
*/
void vpMbEdgeMultiTracker::initFromPose (const vpImage<unsigned char>& I, const vpPoseVector &cPo) {
  vpHomogeneousMatrix _cMo(cPo);
  vpMbEdgeMultiTracker::initFromPose(I, _cMo);
}

/*!
  Initialize the tracking thanks to the pose for stereo cameras configuration.

  \param I1 : Input image for the first camera.
  \param I2 : Input image for the second camera.
  \param c1Mo : Pose matrix for the first camera.
  \param c2Mo : Pose matrix for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference camera, otherwise it is the second one.
*/
void vpMbEdgeMultiTracker::initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const bool firstCameraIsReference) {
  //For Edge, initFromPose has the same behavior than setPose
  //So, for convenience we call setPose
  vpMbEdgeMultiTracker::setPose(I1, I2, c1Mo, c2Mo, firstCameraIsReference);
}

/*!
  Initialize the tracking thanks to the pose. The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images.
  \param cMo_ : Pose matrix for the reference camera.
*/
void vpMbEdgeMultiTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const vpHomogeneousMatrix &cMo_) {
  //For Edge, initFromPose has the same behavior than setPose
  //So, for convenience we call setPose
  vpMbEdgeMultiTracker::setPose(mapOfImages, cMo_);
}

/*!
  Initialize the tracking thanks to the pose.

  \param mapOfImages : Map of images.
  \param mapOfCameraPoses : Map of pose matrix.
*/
void vpMbEdgeMultiTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //For Edge, initFromPose has the same behavior than setPose
  //So, for convenience we call setPose
  vpMbEdgeMultiTracker::setPose(mapOfImages, mapOfCameraPoses);
}

void vpMbEdgeMultiTracker::initPyramid(const std::map<std::string, const vpImage<unsigned char> * >& mapOfImages,
    std::map<std::string, std::vector<const vpImage<unsigned char>* > >& pyramid)
{
  for(std::map<std::string, const vpImage<unsigned char> * >::const_iterator it = mapOfImages.begin();
      it != mapOfImages.end(); ++it) {
    pyramid[it->first].resize(scales.size());

    vpMbEdgeTracker::initPyramid(*it->second, pyramid[it->first]);
  }
}

/*!
  Load the xml configuration file.
  From the configuration file, initialize the parameters corresponding to the objects: moving-edges,
  camera and visibility angles.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data).

  \param configFile : full name of the xml file.

  The XML configuration file has the following form:
  \code
<?xml version="1.0"?>
<conf>
  <ecm>
    <mask>
      <size>5</size>
      <nb_mask>180</nb_mask>
    </mask>
    <range>
      <tracking>7</tracking>
    </range>
    <contrast>
      <edge_threshold>5000</edge_threshold>
      <mu1>0.5</mu1>
      <mu2>0.5</mu2>
    </contrast>
    <sample>
      <step>4</step>
    </sample>
  </ecm>
  <face>
    <near_clipping>0.01</near_clipping>
    <far_clipping>0.90</far_clipping>
    <fov_clipping>1</fov_clipping>
  </face>
  <camera>
    <u0>320</u0>
    <v0>240</v0>
    <px>686.24</px>
    <py>686.24</py>
  </camera>
</conf>
  \endcode

  \sa vpXmlParser::cleanup()
*/
void vpMbEdgeMultiTracker::loadConfigFile(const std::string &configFile) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    //Load ConfigFile for reference camera
    it->second->loadConfigFile(configFile);
    it->second->getCameraParameters(cam);

    //Set Moving Edge parameters
    this->me = it->second->getMovingEdge();

    //Set clipping
    this->clippingFlag = it->second->getClipping();
    this->angleAppears = it->second->getAngleAppear();
    this->angleDisappears = it->second->getAngleDisappear();
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Load the xml configuration files for the stereo cameras case. An example of such a file is provided in
  loadConfigFile(const std::string &) documentation.
  From the configuration file initialize the parameters corresponding to the objects: moving-edges,
  camera and visibility angles.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param configFile1 : Full name of the xml file for the first camera.
  \param configFile2 : Full name of the xml file for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference, otherwise it is the second one.

  \sa loadConfigFile(const std::string &), vpXmlParser::cleanup()
*/
void vpMbEdgeMultiTracker::loadConfigFile(const std::string &configFile1, const std::string &configFile2,
    const bool firstCameraIsReference) {
  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->loadConfigFile(configFile1);

    if(firstCameraIsReference) {
      it->second->getCameraParameters(cam);

      //Set Moving Edge parameters
      this->me = it->second->getMovingEdge();

      //Set clipping
      this->clippingFlag = it->second->getClipping();
      this->angleAppears = it->second->getAngleAppear();
      this->angleDisappears = it->second->getAngleDisappear();
    }
    ++it;

    it->second->loadConfigFile(configFile2);

    if(!firstCameraIsReference) {
      it->second->getCameraParameters(cam);

      //Set Moving Edge parameters
      this->me = it->second->getMovingEdge();

      //Set clipping
      this->clippingFlag = it->second->getClipping();
      this->angleAppears = it->second->getAngleAppear();
      this->angleDisappears = it->second->getAngleDisappear();
    }
  } else {
    std::stringstream ss;
    ss << "Cannot loadConfigFile. Require two cameras ! There are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Load the xml configuration files for all the cameras. An example of such a file is provided in
  loadConfigFile(const std::string &) documentation.
  From the configuration file initialize the parameters corresponding to the objects: moving-edges,
  camera and visibility angles.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param mapOfConfigFiles : Map of xml files.

  \sa loadConfigFile(const std::string &), vpXmlParser::cleanup()
*/
void vpMbEdgeMultiTracker::loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles) {
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();
      it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    std::map<std::string, std::string>::const_iterator it_config = mapOfConfigFiles.find(it_edge->first);
    if(it_config != mapOfConfigFiles.end()) {
      it_edge->second->loadConfigFile(it_config->second);
    } else {
      std::stringstream ss;
      ss << "Missing configuration file for camera: " << it_edge->first << " !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
  }

  //Set the reference camera parameters
  std::map<std::string, vpMbEdgeTracker *>::iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->getCameraParameters(cam);

    //Set Moving Edge parameters
    this->me = it->second->getMovingEdge();

    //Set clipping
    this->clippingFlag = it->second->getClipping();
    this->angleAppears = it->second->getAngleAppear();
    this->angleDisappears = it->second->getAngleDisappear();
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str());
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

  \param modelFile : the file containing the the 3D model description.
  The extension of this file is either .wrl or .cao.
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbEdgeMultiTracker::loadModel(const std::string &modelFile, const bool verbose) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->loadModel(modelFile, verbose);
  }

  modelInitialised = true;
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new model
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbEdgeMultiTracker::reInitModel(const vpImage<unsigned char>& I, const std::string &cad_name,
    const vpHomogeneousMatrix& cMo_, const bool verbose) {
  if(m_mapOfEdgeTrackers.size() != 1) {
    std::stringstream ss;
    ss << "This method requires exactly one camera, there are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }

  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->reInitModel(I, cad_name, cMo_, verbose);

    //Set reference pose
    it_edge->second->getPose(cMo);

    modelInitialised = true;
  }
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
void vpMbEdgeMultiTracker::reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &cad_name, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
    const bool verbose, const bool firstCameraIsReference) {
  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();

    it_edge->second->reInitModel(I1, cad_name, c1Mo, verbose);

    if(firstCameraIsReference) {
      //Set reference pose
      it_edge->second->getPose(cMo);
    }

    ++it_edge;

    it_edge->second->reInitModel(I2, cad_name, c2Mo, verbose);

    if(!firstCameraIsReference) {
      //Set reference pose
      it_edge->second->getPose(cMo);
    }
  } else {
    throw vpException(vpTrackingException::fatalError, "This method requires exactly two cameras !");
  }
}

/*!
  Re-initialize the model used by the tracker.

  \param mapOfImages : Map of images.
  \param cad_name : Path to the file containing the 3D model description.
  \param mapOfCameraPoses : The new vpHomogeneousMatrix between the cameras and the current object position.
  \param verbose : Verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbEdgeMultiTracker::reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::string &cad_name, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
    const bool verbose) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if(it_edge != m_mapOfEdgeTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    it_edge->second->reInitModel(*it_img->second, cad_name, it_camPose->second, verbose);
    modelInitialised = true;

    //Set reference pose
    it_edge->second->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel for reference camera !");
  }

  std::vector<std::string> vectorOfMissingCameras;
  for(it_edge = m_mapOfEdgeTrackers.begin(); it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    if(it_edge->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_edge->first);
      it_camPose = mapOfCameraPoses.find(it_edge->first);

      if(it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        it_edge->second->reInitModel(*it_img->second, cad_name, it_camPose->second, verbose);
      } else {
        vectorOfMissingCameras.push_back(it_edge->first);
      }
    }
  }

  for(std::vector<std::string>::const_iterator it = vectorOfMissingCameras.begin();
      it != vectorOfMissingCameras.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfEdgeTrackers[*it]->reInitModel(*it_img->second, cad_name, cCurrentMo, verbose);
    }
  }
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void vpMbEdgeMultiTracker::resetTracker() {
  this->cMo.eye();

  //Reset all internal trackers
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->resetTracker();
  }

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif

  m_computeInteraction = true;
//  nline = 0; //Not used in vpMbEdgeMultiTracker class
//  ncylinder = 0; //Not used in vpMbEdgeMultiTracker class
  m_lambda = 1.0;
//  nbvisiblepolygone = 0; //Not used in vpMbEdgeMultiTracker class
  percentageGdPt = 0.4;

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);
  clippingFlag = vpPolygon3D::NO_CLIPPING;

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  // reinitialization of the scales.
  this->setScales(scales);
}

/*!
  Set the angle used to test polygons appearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value lower than
  this parameter, the polygon is considered as appearing.
  The polygon will then be tracked.

  \param a : new angle in radian.
*/
void vpMbEdgeMultiTracker::setAngleAppear(const double &a) {
  vpMbTracker::setAngleAppear(a);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setAngleAppear(a);
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
void vpMbEdgeMultiTracker::setAngleDisappear(const double &a) {
  vpMbTracker::setAngleDisappear(a);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setAngleDisappear(a);
  }
}

/*!
  Set the camera parameters for the monocular case.

  \param camera : The new camera parameters.
*/
void vpMbEdgeMultiTracker::setCameraParameters(const vpCameraParameters& camera) {
  if(m_mapOfEdgeTrackers.empty()) {
    throw vpException(vpTrackingException::fatalError, "There is no camera !");
  } else if(m_mapOfEdgeTrackers.size() > 1) {
    throw vpException(vpTrackingException::fatalError, "There is more than one camera !");
  } else {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
    if(it != m_mapOfEdgeTrackers.end()) {
      it->second->setCameraParameters(camera);

      //Set reference camera parameters
      this->cam = camera;
    } else {
      std::stringstream ss;
      ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
  }
}

/*!
  Set the camera parameters for the stereo cameras case.

  \param camera1 : The new camera parameters for the first camera.
  \param camera2 : The new camera parameters for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference, otherwise it is the second one.
*/
void vpMbEdgeMultiTracker::setCameraParameters(const vpCameraParameters& camera1, const vpCameraParameters& camera2,
    const bool firstCameraIsReference) {
  if(m_mapOfEdgeTrackers.empty()) {
    throw vpException(vpTrackingException::fatalError, "There is no camera !");
  } else if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->setCameraParameters(camera1);

    ++it;
    it->second->setCameraParameters(camera2);

    if(firstCameraIsReference) {
      this->cam = camera1;
    } else {
      this->cam = camera2;
    }
  } else {
    std::stringstream ss;
    ss << "Require two cameras ! There are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Set the camera parameters for the specified camera.

  \param cameraName : Camera name.
  \param camera : The new camera parameters.
*/
void vpMbEdgeMultiTracker::setCameraParameters(const std::string &cameraName, const vpCameraParameters& camera) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->setCameraParameters(camera);

    if(it->first == m_referenceCameraName) {
      this->cam = camera;
    }
  } else {
    std::stringstream ss;
    ss << "The camera: " << cameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Set the camera parameters for all the cameras.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbEdgeMultiTracker::setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters) {
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();
      it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_edge->first);
    if(it_cam != mapOfCameraParameters.end()) {
      it_edge->second->setCameraParameters(it_cam->second);

      if(it_edge->first == m_referenceCameraName) {
        this->cam = it_cam->second;
      }
    } else {
      std::stringstream ss;
      ss << "Missing camera parameters for camera: " << it_edge->first << " !";
      throw vpException(vpTrackingException::initializationError, ss.str());
    }
  }
}

/*!
  Set the camera transformation matrix for the specified camera (\f$ _{}^{c_{current}}\textrm{M}_{c_{reference}} \f$).

  \param cameraName : Camera name.
  \param cameraTransformationMatrix : Camera transformation matrix between the current and the reference camera.
*/
void vpMbEdgeMultiTracker::setCameraTransformationMatrix(const std::string &cameraName,
    const vpHomogeneousMatrix &cameraTransformationMatrix) {
  std::map<std::string, vpHomogeneousMatrix>::iterator it = m_mapOfCameraTransformationMatrix.find(cameraName);
  if(it != m_mapOfCameraTransformationMatrix.end()) {
    it->second = cameraTransformationMatrix;
  } else {
    std::stringstream ss;
    ss << "Cannot find camera: " << cameraName << " !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Set the map of camera transformation matrices
  (\f$ _{}^{c_1}\textrm{M}_{c_1},  _{}^{c_2}\textrm{M}_{c_1}, _{}^{c_3}\textrm{M}_{c_1}, \cdots, _{}^{c_n}\textrm{M}_{c_1} \f$).

  \param mapOfTransformationMatrix : map of camera transformation matrices.
*/
void vpMbEdgeMultiTracker::setCameraTransformationMatrix(
    const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix) {
  //Check if all cameras have a transformation matrix
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();
      it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = mapOfTransformationMatrix.find(it_edge->first);

    if(it_camTrans == mapOfTransformationMatrix.end()) {
      throw vpException(vpTrackingException::initializationError, "Missing camera transformation matrix !");
    }
  }

  m_mapOfCameraTransformationMatrix = mapOfTransformationMatrix;
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags : New clipping flags.
*/
void vpMbEdgeMultiTracker::setClipping(const unsigned int &flags) {
  //Set clipping for vpMbEdgeMultiTracker class
  vpMbTracker::setClipping(flags);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setClipping(flags);
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param cameraName : Camera to set the clipping.
  \param flags : New clipping flags.
*/
void vpMbEdgeMultiTracker::setClipping(const std::string &cameraName, const unsigned int &flags) {
  //Set clipping for the given camera, do not change the clipping for vpMbEdgeMultiTracker class
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->setClipping(flags);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Set if the covariance matrix has to be computed.

  \param flag : True if the covariance has to be computed, false otherwise
*/
void vpMbEdgeMultiTracker::setCovarianceComputation(const bool& flag) {
  vpMbTracker::setCovarianceComputation(flag);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setCovarianceComputation(flag);
  }
}

/*!
  Enable to display the features. By features, we mean the moving edges (ME) and the klt points if used.

  Note that if present, the moving edges can be displayed with different colors:
  - If green : The ME is a good point.
  - If blue : The ME is removed because of a contrast problem during the tracking phase.
  - If purple : The ME is removed because of a threshold problem during the tracking phase.
  - If red : The ME is removed because it is rejected by the robust approach in the virtual visual servoing scheme.

  \param displayF : set it to true to display the features.
*/
void vpMbEdgeMultiTracker::setDisplayFeatures(const bool displayF) {
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setDisplayFeatures(displayF);
  }

  displayFeatures = displayF;
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.
*/
void vpMbEdgeMultiTracker::setFarClippingDistance(const double &dist) {
  vpMbTracker::setFarClippingDistance(dist);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setFarClippingDistance(dist);
  }
}

/*!
  Set the far distance for clipping for the specified camera.

  \param cameraName : Camera to set the far clipping.
  \param dist : Far clipping value.
*/
void vpMbEdgeMultiTracker::setFarClippingDistance(const std::string &cameraName, const double &dist) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->setFarClippingDistance(dist);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
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
 */
void vpMbEdgeMultiTracker::setGoodMovingEdgesRatioThreshold(const double threshold) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setGoodMovingEdgesRatioThreshold(threshold);
  }

  percentageGdPt = threshold;
}

#ifdef VISP_HAVE_OGRE
/*!
  Set the ratio of visibility attempts that has to be successful to consider a polygon as visible.

  \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

  \param ratio : Ratio of succesful attempts that has to be considered. Value has to be between 0.0 (0%) and 1.0 (100%).
*/
  void vpMbEdgeMultiTracker::setGoodNbRayCastingAttemptsRatio(const double &ratio) {
    for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
        it != m_mapOfEdgeTrackers.end(); ++it) {
      it->second->setGoodNbRayCastingAttemptsRatio(ratio);
    }
  }

  /*!
    Set the number of rays that will be sent toward each polygon for visibility test.
    Each ray will go from the optic center of the camera to a random point inside the considered polygon.

    \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

    \param attempts Number of rays to be sent.
  */
  void vpMbEdgeMultiTracker::setNbRayCastingAttemptsForVisibility(const unsigned int &attempts) {
    for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
        it != m_mapOfEdgeTrackers.end(); ++it) {
      it->second->setNbRayCastingAttemptsForVisibility(attempts);
    }
  }
#endif

  /*!
    Set the flag to consider if the level of detail (LOD) is used for all the cameras.

    \param useLod : true if the level of detail must be used, false otherwise. When true,
    two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
    \param name : name of the face we want to modify the LOD parameter.

    \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
   */
void vpMbEdgeMultiTracker::setLod(const bool useLod, const std::string &name) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setLod(useLod, name);
  }
}

/*!
  Set the flag to consider if the level of detail (LOD) is used for all the cameras.

  \param useLod : true if the level of detail must be used, false otherwise. When true,
  two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
  \param cameraName : Name of the camera we want to set the LOD.
  \param name : name of the face we want to modify the LOD parameter, if empty all the faces are considered.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
 */
void vpMbEdgeMultiTracker::setLod(const bool useLod, const std::string &cameraName, const std::string &name) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(cameraName);

  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->setLod(useLod, name);
  } else {
    std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  }
}

/*!
  Set the threshold for the minimum line length to be considered as visible in the LOD case.

  \param minLineLengthThresh : threshold for the minimum line length in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinPolygonAreaThresh()
 */
void vpMbEdgeMultiTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setMinLineLengthThresh(minLineLengthThresh, name);
  }
}

/*!
  Set the threshold for the minimum line length to be considered as visible in the LOD case.

  \param minLineLengthThresh : threshold for the minimum line length in pixel.
  \param cameraName : name of the camera to consider.
  \param name : name of the face we want to modify the LOD threshold, if empty all the faces are considered.

  \sa setLod(), setMinPolygonAreaThresh()
 */
void vpMbEdgeMultiTracker::setMinLineLengthThresh(const double minLineLengthThresh, const std::string &cameraName,
    const std::string &name) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(cameraName);

  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->setMinLineLengthThresh(minLineLengthThresh, name);
  } else {
    std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  }
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinLineLengthThresh()
 */
void vpMbEdgeMultiTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setMinPolygonAreaThresh(minPolygonAreaThresh, name);
  }
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param cameraName : name of the camera to consider.
  \param name : name of the face we want to modify the LOD threshold, if empty all the faces are considered.

  \sa setLod(), setMinLineLengthThresh()
 */
void vpMbEdgeMultiTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &cameraName,
    const std::string &name) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(cameraName);

  if(it_edge != m_mapOfEdgeTrackers.end()) {
    it_edge->second->setMinPolygonAreaThresh(minPolygonAreaThresh, name);
  } else {
    std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  }
}

/*!
  Set the moving edge parameters.

  \param me : an instance of vpMe containing all the desired parameters.
*/
void vpMbEdgeMultiTracker::setMovingEdge(const vpMe &me) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setMovingEdge(me);
  }
}

/*!
  Set the moving edge parameters for the specified camera.

  \param cameraName : Camera name to set the moving edge parameters.
  \param me : An instance of vpMe containing all the desired parameters.
*/
void vpMbEdgeMultiTracker::setMovingEdge(const std::string &cameraName, const vpMe &me) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->setMovingEdge(me);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Set the near distance for clipping.

  \param dist : Near clipping value.
*/
void vpMbEdgeMultiTracker::setNearClippingDistance(const double &dist) {
  vpMbTracker::setNearClippingDistance(dist);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setNearClippingDistance(dist);
  }
}

/*!
  Set the near distance for clipping for the specified camera.

  \param cameraName : Camera name to set the near clipping distance.
  \param dist : Near clipping value.
*/
void vpMbEdgeMultiTracker::setNearClippingDistance(const std::string &cameraName, const double &dist) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(cameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->setNearClippingDistance(dist);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Enable/Disable the appearance of Ogre config dialog on startup.

  \warning This method has only effect when Ogre is used and Ogre visibility test is
  enabled using setOgreVisibilityTest() with true parameter.

  \param showConfigDialog : if true, shows Ogre dialog window (used to set Ogre
  rendering options) when Ogre visibility is enabled. By default, this functionality
  is turned off.
*/
void vpMbEdgeMultiTracker::setOgreShowConfigDialog(const bool showConfigDialog) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setOgreShowConfigDialog(showConfigDialog);
  }
}

/*!
  Use Ogre3D for visibility tests

  \warning This function has to be called before the initialization of the tracker.

  \param v : True to use it, False otherwise
*/
void vpMbEdgeMultiTracker::setOgreVisibilityTest(const bool &v) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->faces.getOgreContext()->setWindowName("Multi MBT Edge (" + it->first + ")");
  }
#endif

  useOgre = v;
}

/*!
  Set the optimization method used during the tracking.

  \param opt : Optimization method to use.
*/
void vpMbEdgeMultiTracker::setOptimizationMethod(const vpMbtOptimizationMethod &opt) {
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setOptimizationMethod(opt);
  }

  m_optimizationMethod = opt;
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I : image corresponding to the desired pose.
  \param cMo_ : Pose to affect.
*/
void vpMbEdgeMultiTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_) {
  if(m_mapOfEdgeTrackers.size() == 1) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);
    if(it != m_mapOfEdgeTrackers.end()) {
      it->second->setPose(I, cMo_);
      this->cMo = cMo_;
    } else {
      std::stringstream ss;
      ss << "Cannot find the reference camera: " << m_referenceCameraName << " !";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  } else {
    std::stringstream ss;
    ss << "You are trying to set the pose with only one image and cMo but there are multiple cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str());
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
void vpMbEdgeMultiTracker::setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix c2Mo, const bool firstCameraIsReference) {
  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    it->second->setPose(I1, c1Mo);

    ++it;

    it->second->setPose(I2, c2Mo);

    if(firstCameraIsReference) {
      this->cMo = c1Mo;
    } else {
      this->cMo = c2Mo;
    }
  } else {
    std::stringstream ss;
    ss << "This method requires 2 cameras but there are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images.
  \param cMo_ : Pose to affect to the reference camera.
*/
void vpMbEdgeMultiTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const vpHomogeneousMatrix &cMo_) {
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  if(it_edge != m_mapOfEdgeTrackers.end()) {
    std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);

    if(it_img != mapOfImages.end()) {
      //Set pose on reference camera
      it_edge->second->setPose(*it_img->second, cMo_);

      //Set the reference cMo
      cMo = cMo_;

      //Set the pose for the others cameras
      for(it_edge = m_mapOfEdgeTrackers.begin(); it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
        if(it_edge->first != m_referenceCameraName) {
          it_img = mapOfImages.find(it_edge->first);
          std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
              m_mapOfCameraTransformationMatrix.find(it_edge->first);

          if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
            vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
            it_edge->second->setPose(*it_img->second, cCurrentMo);
          } else {
            throw vpException(vpTrackingException::fatalError, "Missing image or camera transformation matrix !");
          }
        }
      }
    } else {
      std::stringstream ss;
      ss << "Missing image for camera: " << m_referenceCameraName << " !";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  Cameras that do not have pose will be automatically handled but the pose for the reference has to be passed in parameter.
  The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images corresponding to the desired pose.
  \param mapOfCameraPoses : Map of poses to affect.
*/
void vpMbEdgeMultiTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it_edge = m_mapOfEdgeTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if(it_edge != m_mapOfEdgeTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    it_edge->second->setPose(*it_img->second, it_camPose->second);
    it_edge->second->getPose(cMo);
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot set pose for the reference camera !");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for(it_edge = m_mapOfEdgeTrackers.begin(); it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    if(it_edge->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_edge->first);
      it_camPose = mapOfCameraPoses.find(it_edge->first);

      if(it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        //Set pose
        it_edge->second->setPose(*it_img->second, it_camPose->second);
      } else {
        vectorOfMissingCameraPoses.push_back(it_edge->first);
      }
    }
  }

  for(std::vector<std::string>::const_iterator it1 = vectorOfMissingCameraPoses.begin();
      it1 != vectorOfMissingCameraPoses.end(); ++it1) {
    it_img = mapOfImages.find(*it1);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it1);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfEdgeTrackers[*it1]->setPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix ! Cannot set the pose for camera: " << (*it1) << " !";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  }
}

/*!
  Set if the projection error criteria has to be computed.

  \param flag : True if the projection error criteria has to be computed, false otherwise
*/
void vpMbEdgeMultiTracker::setProjectionErrorComputation(const bool &flag) {
  //Set the flag in the current class
  vpMbTracker::setProjectionErrorComputation(flag);

  //Set the flag for each camera
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setProjectionErrorComputation(flag);
  }
}

/*!
  Set the reference camera name.

  \param referenceCameraName : Name of the reference camera.
 */
void vpMbEdgeMultiTracker::setReferenceCameraName(const std::string &referenceCameraName) {
  std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.find(referenceCameraName);
  if(it != m_mapOfEdgeTrackers.end()) {
    m_referenceCameraName = referenceCameraName;
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Set the scales to use to realize the tracking. The vector of boolean activates
  or not the scales to set for the object tracking. The first element of the list
  correspond to the tracking on the full image, the second element corresponds
  to the tracking on an image subsampled by two.

  Using multi scale tracking allows to track the object with greater moves. It
  requires the computation of a pyramid of images, but the total tracking can be
  faster than a tracking based only on the full scale. The pose is computed from
  the smallest image to the biggest. This may be dangerous if the object to
  track is small in the image, because the subsampled scale(s) will have only
  few points to compute the pose (it could result in a loss of precision).

  \warning This method must be used before the tracker has been initialized (
  before the call of the loadConfigFile() or loadModel() methods).

  \warning Not working for the moment. At least one level must be activated.

  \param scale : The vector describing the levels to use.
*/
void vpMbEdgeMultiTracker::setScales(const std::vector<bool>& scale) {
  vpMbEdgeTracker::setScales(scale);
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setScales(scale);
  }
}

/*!
  Use Scanline algorithm for visibility tests

  \param v : True to use it, False otherwise
*/
void vpMbEdgeMultiTracker::setScanLineVisibilityTest(const bool &v) {
  //Set general setScanLineVisibilityTest
  vpMbTracker::setScanLineVisibilityTest(v);

  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setScanLineVisibilityTest(v);
  }
}

/*!
  Set if the polygons that have the given name have to be considered during the tracking phase.

  \param name : name of the polygon(s).
  \param useEdgeTracking : True if it has to be considered, false otherwise.
*/
void vpMbEdgeMultiTracker::setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking) {
  for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
      it != m_mapOfEdgeTrackers.end(); ++it) {
    it->second->setUseEdgeTracking(name, useEdgeTracking);
  }
}

/*!
  Compute each state of the tracking procedure for all the feature sets.

  If the tracking is considered as failed an exception is thrown.

  \param I : The image.
 */
void vpMbEdgeMultiTracker::track(const vpImage<unsigned char> &I) {
  //Track only with reference camera
  //Get the reference camera parameters
  std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.find(m_referenceCameraName);

  if(it != m_mapOfEdgeTrackers.end()) {
    it->second->track(I);
    it->second->getPose(cMo);
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }

  if(computeProjError) {
    //Use the projection error computed by the single vpMbEdgeTracker in m_mapOfEdgeTrackers
    projectionError = it->second->getProjectionError();
  }
}

/*!
  Compute each state of the tracking procedure for all the feature sets.

  If the tracking is considered as failed an exception is thrown.

  \param I1 : The first image.
  \param I2 : The second image.
 */
void vpMbEdgeMultiTracker::track(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2) {
  //Track with the special case of stereo cameras
  if(m_mapOfEdgeTrackers.size() == 2) {
    std::map<std::string, vpMbEdgeTracker *>::const_iterator it = m_mapOfEdgeTrackers.begin();
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
    mapOfImages[it->first] = &I1;
    ++it;

    mapOfImages[it->first] = &I2;
    track(mapOfImages);
  } else {
    std::stringstream ss;
    ss << "Require two cameras ! There are " << m_mapOfEdgeTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Compute each state of the tracking procedure for all the feature sets.

  If the tracking is considered as failed an exception is thrown.

  \param mapOfImages : Map of images.
 */
void vpMbEdgeMultiTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  //Check if there is an image for each camera
  for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it_edge = m_mapOfEdgeTrackers.begin();
      it_edge != m_mapOfEdgeTrackers.end(); ++it_edge) {
    std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(it_edge->first);

    if(it_img == mapOfImages.end()) {
      throw vpException(vpTrackingException::fatalError, "Missing images !");
    }
  }


  initPyramid(mapOfImages, m_mapOfPyramidalImages);

  unsigned int lvl = (unsigned int) scales.size();
  do {
    lvl--;

    projectionError = 90.0;

    if(scales[lvl]) {
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);
        for(std::map<std::string, vpMbEdgeTracker *>::const_iterator it1 = m_mapOfEdgeTrackers.begin();
            it1 != m_mapOfEdgeTrackers.end(); ++it1) {
          //Downscale for each camera
          it1->second->downScale(lvl);

          //Track moving edges
          try {
            it1->second->trackMovingEdge(*m_mapOfPyramidalImages[it1->first][lvl]);
          } catch(...) {
            vpTRACE("Error in moving edge tracking") ;
            throw ;
          }
        }

        try {
          std::map<std::string, const vpImage<unsigned char> *> mapOfPyramidImages;
          for(std::map<std::string, std::vector<const vpImage<unsigned char>* > >::const_iterator
              it = m_mapOfPyramidalImages.begin(); it != m_mapOfPyramidalImages.end(); ++it) {
            mapOfPyramidImages[it->first] = it->second[lvl];
          }

          computeVVS(mapOfPyramidImages, lvl);
        } catch(...) {
          covarianceMatrix = -1;
          throw; // throw the original exception
        }


        //Test tracking failed only if all testTracking failed
        bool isOneTestTrackingOk = false;
        for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
            it != m_mapOfEdgeTrackers.end(); ++it) {
          //Set the camera pose
          it->second->cMo = m_mapOfCameraTransformationMatrix[it->first]*cMo;

          try {
            it->second->testTracking();
            isOneTestTrackingOk = true;
          } catch(/*vpException &e*/...) {
      //      throw e;
          }
        }

        if(!isOneTestTrackingOk) {
          std::ostringstream oss;
          oss << "Not enough moving edges to track the object. Try to reduce the threshold="
              << percentageGdPt << " using setGoodMovingEdgesRatioThreshold()";
          throw vpTrackingException(vpTrackingException::fatalError, oss.str());
        }


        if(displayFeatures) {
          for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
                      it != m_mapOfEdgeTrackers.end(); ++it) {
            it->second->displayFeaturesOnImage(*mapOfImages[it->first], lvl);
          }
        }

        // Looking for new visible face
        bool newvisibleface = false;
        for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
            it != m_mapOfEdgeTrackers.end(); ++it) {
          it->second->visibleFace(*mapOfImages[it->first], it->second->cMo, newvisibleface);
        }

        if(useScanLine) {
          for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
                      it != m_mapOfEdgeTrackers.end(); ++it) {
            it->second->faces.computeClippedPolygons(it->second->cMo, it->second->cam);
            it->second->faces.computeScanLineRender(it->second->cam, mapOfImages[it->first]->getWidth(),
                mapOfImages[it->first]->getHeight());
          }
        }

        try {
          for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
                      it != m_mapOfEdgeTrackers.end(); ++it) {
            it->second->updateMovingEdge(*mapOfImages[it->first]);
          }
        } catch(...) {
          throw; // throw the original exception
        }

        for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
            it != m_mapOfEdgeTrackers.end(); ++it) {
          it->second->initMovingEdge(*mapOfImages[it->first], it->second->cMo);

          // Reinit the moving edge for the lines which need it.
          it->second->reinitMovingEdge(*mapOfImages[it->first], it->second->cMo);

          if(computeProjError) {
            //Compute the projection error
            it->second->computeProjectionError(*mapOfImages[it->first]);
          }
        }

        computeProjectionError();

        upScale(lvl);
        for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
            it != m_mapOfEdgeTrackers.end(); ++it) {
          it->second->upScale(lvl);
        }
      }
      catch(vpException &e)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);

          for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
              it != m_mapOfEdgeTrackers.end(); ++it) {
            it->second->cMo = cMo_1;
            it->second->reInitLevel(lvl);
            it->second->upScale(lvl);
          }
        }
        else{
          upScale(lvl);
          for(std::map<std::string, vpMbEdgeTracker*>::const_iterator it = m_mapOfEdgeTrackers.begin();
              it != m_mapOfEdgeTrackers.end(); ++it) {
            it->second->upScale(lvl);
          }
          throw(e) ;
        }
      }
    }
  } while(lvl != 0);

  cleanPyramid(m_mapOfPyramidalImages);
}
