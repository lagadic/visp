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
 * Model-based klt tracker with multiple cameras.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
 \file vpMbKltMultiTracker.cpp
 \brief Model-based klt tracker with multiple cameras.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/mbt/vpMbKltMultiTracker.h>


/*!
  Basic constructor
*/
vpMbKltMultiTracker::vpMbKltMultiTracker() :
    m_mapOfCameraTransformationMatrix(), m_mapOfKltTrackers(), m_referenceCameraName("Camera"),
    m_L_kltMulti(), m_error_kltMulti(), m_w_kltMulti(), m_weightedError_kltMulti()
{
  m_mapOfKltTrackers["Camera"] = new vpMbKltTracker();

  //Add default camera transformation matrix
  m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
}

/*!
  Construct a vpMbKltMultiTracker with the specified number of cameras.

  \param nbCameras : Number of cameras to use.
*/
vpMbKltMultiTracker::vpMbKltMultiTracker(const unsigned int nbCameras) :
    m_mapOfCameraTransformationMatrix(), m_mapOfKltTrackers(), m_referenceCameraName("Camera"),
    m_L_kltMulti(), m_error_kltMulti(), m_w_kltMulti(), m_weightedError_kltMulti()
{

  if(nbCameras == 0) {
    throw vpException(vpTrackingException::fatalError, "Cannot construct a vpMbkltMultiTracker with no camera !");
  } else if(nbCameras == 1) {
    m_mapOfKltTrackers["Camera"] = new vpMbKltTracker();

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera"] = vpHomogeneousMatrix();
  } else if(nbCameras == 2) {
    m_mapOfKltTrackers["Camera1"] = new vpMbKltTracker();
    m_mapOfKltTrackers["Camera2"] = new vpMbKltTracker();

    //Add default camera transformation matrix
    m_mapOfCameraTransformationMatrix["Camera1"] = vpHomogeneousMatrix();
    m_mapOfCameraTransformationMatrix["Camera2"] = vpHomogeneousMatrix();

    //Set by default the reference camera
    m_referenceCameraName = "Camera1";
  } else {
    for(unsigned int i = 1; i <= nbCameras; i++) {
      std::stringstream ss;
      ss << "Camera" << i;
      m_mapOfKltTrackers[ss.str()] = new vpMbKltTracker();

      //Add default camera transformation matrix
      m_mapOfCameraTransformationMatrix[ss.str()] = vpHomogeneousMatrix();
    }

    //Set by default the reference camera to the first one
    m_referenceCameraName = m_mapOfKltTrackers.begin()->first;
  }
}

/*!
  Construct a vpMbKltMultiTracker with the specified list of camera names.

  \param cameraNames : List of camera names.
*/
vpMbKltMultiTracker::vpMbKltMultiTracker(const std::vector<std::string> &cameraNames) :
    m_mapOfCameraTransformationMatrix(), m_mapOfKltTrackers(), m_referenceCameraName("Camera"),
    m_L_kltMulti(), m_error_kltMulti(), m_w_kltMulti(), m_weightedError_kltMulti()
{
  if(cameraNames.empty()) {
    throw vpException(vpTrackingException::fatalError, "Cannot construct a vpMbKltMultiTracker with no camera !");
  }

  for(std::vector<std::string>::const_iterator it = cameraNames.begin(); it != cameraNames.end(); ++it) {
    m_mapOfKltTrackers[*it] = new vpMbKltTracker();
  }

  //Set by default the reference camera
  m_referenceCameraName = cameraNames.front();
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbKltMultiTracker::~vpMbKltMultiTracker() {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    delete it->second;
  }
}

/*!
  Add a circle to the list of circles.

  \param P1 : Center of the circle.
  \param P2,P3 : Two points on the plane containing the circle. With the center of the circle we have 3 points
  defining the plane that contains the circle.
  \param r : Radius of the circle.
  \param name : Name of the circle.
*/
void
vpMbKltMultiTracker::addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, const double r, const std::string &name)
{
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->addCircle(P1, P2, P3, r, name);
  }
}

void vpMbKltMultiTracker::computeVVS() {
  vpMatrix L_true;
  vpMatrix LVJ_true;
  vpColVector v;

  vpMatrix LTL;
  vpColVector LTR;
  vpHomogeneousMatrix cMoPrev;
  vpHomogeneousMatrix ctTc0_Prev;
  vpColVector error_prev;
  double mu = m_initialMu;

  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  computeVVSInit();
  vpMbKltTracker *klt;
  for (std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;
    klt->computeVVSInit();
  }

  std::map<std::string, vpVelocityTwistMatrix> mapOfVelocityTwist;
  for (std::map<std::string, vpHomogeneousMatrix>::const_iterator it = m_mapOfCameraTransformationMatrix.begin();
      it != m_mapOfCameraTransformationMatrix.end(); ++it) {
    vpVelocityTwistMatrix cVo;
    cVo.buildFrom(it->second);
    mapOfVelocityTwist[it->first] = cVo;
  }

  while ( ((int)((normRes - normRes_1)*1e8) != 0 )  && (iter < m_maxIter) ) {
    computeVVSInteractionMatrixAndResidu(mapOfVelocityTwist);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_kltMulti, error_prev, cMoPrev, mu, reStartFromLastIncrement);
    if (reStartFromLastIncrement) {
      ctTc0 = ctTc0_Prev;
    }

    if(!reStartFromLastIncrement) {
      vpMbKltMultiTracker::computeVVSWeights();

      if (computeCovariance) {
        L_true = m_L_kltMulti;
        if (!isoJoIdentity) {
           vpVelocityTwistMatrix cVo;
           cVo.buildFrom(cMo);
           LVJ_true = (m_L_kltMulti*cVo*oJo);
        }
      }

      normRes_1 = normRes;
      normRes = 0.0;

      for (unsigned int i = 0; i < m_weightedError_kltMulti.getRows(); i ++) {
        m_weightedError_kltMulti[i] = m_error_kltMulti[i] * m_w_kltMulti[i];
        normRes += m_weightedError_kltMulti[i];
      }

      if ((iter == 0) || m_computeInteraction) {
        for (unsigned int i = 0; i < m_L_kltMulti.getRows(); i++) {
          for (unsigned int j = 0; j < 6; j++) {
            m_L_kltMulti[i][j] *= m_w_kltMulti[i];
          }
        }
      }

      computeVVSPoseEstimation(isoJoIdentity, iter, m_L_kltMulti, LTL, m_weightedError_kltMulti, m_error_kltMulti, error_prev, LTR, mu, v);

      cMoPrev = cMo;
      ctTc0_Prev = ctTc0;
      ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
      cMo = ctTc0 * c0Mo;
    } // endif(!reStartFromLastIncrement)

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity, m_w_kltMulti, cMoPrev, L_true, LVJ_true, m_error_kltMulti);
}

void vpMbKltMultiTracker::computeVVSInit() {
  unsigned int nbFeatures = 2*m_nbInfos;

  m_L_kltMulti.resize(nbFeatures, 6, false);
  m_w_kltMulti.resize(nbFeatures, false);
  m_error_kltMulti.resize(nbFeatures, false);
  m_weightedError_kltMulti.resize(nbFeatures, false);
}

void vpMbKltMultiTracker::computeVVSInteractionMatrixAndResidu() {
  throw vpException(vpException::fatalError, "vpMbKltMultiTracker::computeVVSInteractionMatrixAndResidu() should not be called!");
}

void vpMbKltMultiTracker::computeVVSInteractionMatrixAndResidu(std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist) {
  unsigned int startIdx = 0;

  vpMbKltTracker *klt;
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;

    //Use the ctTc0 variable instead of the formula in the monocular case
    //to ensure that we have the same result than vpMbKltTracker
    //as some slight differences can occur due to numerical imprecision
    if(m_mapOfKltTrackers.size() == 1) {
      klt->ctTc0 = ctTc0;
      klt->computeVVSInteractionMatrixAndResidu();
    } else {
      vpHomogeneousMatrix c_curr_tTc_curr0 = m_mapOfCameraTransformationMatrix[it->first] * cMo * it->second->c0Mo.inverse();
      klt->ctTc0 = c_curr_tTc_curr0;
      klt->computeVVSInteractionMatrixAndResidu();
    }

    m_error_kltMulti.insert(startIdx, klt->m_error_klt);
    m_L_kltMulti.insert(klt->m_L_klt*mapOfVelocityTwist[it->first], startIdx, 0);

    startIdx += klt->m_error_klt.getRows();
  }
}

void vpMbKltMultiTracker::computeVVSWeights() {
  vpMbKltTracker *klt;
  unsigned int startIdx = 0;
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;
    klt->computeVVSWeights(klt->m_robust_klt, klt->m_error_klt, klt->m_w_klt);

    m_w_kltMulti.insert(startIdx, klt->m_w_klt);
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
void vpMbKltMultiTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_,
    const vpCameraParameters &cam_, const vpColor& col , const unsigned int thickness, const bool displayFullModel) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->display(I, cMo_, cam_, col, thickness, displayFullModel);
  } else {
    std::cerr << "Cannot find reference camera:" << m_referenceCameraName << " !" << std::endl;
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
void vpMbKltMultiTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &cam_,
     const vpColor& color , const unsigned int thickness, const bool displayFullModel) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->display(I, cMo_, cam_, color, thickness, displayFullModel);
  } else {
    std::cerr << "Cannot find reference camera:" << m_referenceCameraName << " !" << std::endl;
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
void vpMbKltMultiTracker::display(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor& color, const unsigned int thickness, const bool displayFullModel) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "This display is only for the stereo case ! There are "
        << m_mapOfKltTrackers.size() << " cameras !" << std::endl;
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
void vpMbKltMultiTracker::display(const vpImage<vpRGBa>& I1, const vpImage<vpRGBa>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor& color, const unsigned int thickness, const bool displayFullModel) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->display(I1, c1Mo, cam1, color, thickness, displayFullModel);
    ++it;

    it->second->display(I2, c2Mo, cam2, color, thickness, displayFullModel);
  } else {
    std::cerr << "This display is only for the stereo case ! There are "
        << m_mapOfKltTrackers.size() << " cameras !" << std::endl;
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
void vpMbKltMultiTracker::display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
      const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
      const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  //Display only for the given images
  for(std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.begin();
      it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if(it_klt != m_mapOfKltTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
      it_klt->second->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements ! " << std::endl;
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
void vpMbKltMultiTracker::display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
      const std::map<std::string, vpCameraParameters> &mapOfCameraParameters,
      const vpColor& col, const unsigned int thickness, const bool displayFullModel) {

  //Display only for the given images
  for(std::map<std::string, const vpImage<vpRGBa> *>::const_iterator it_img = mapOfImages.begin();
      it_img != mapOfImages.end(); ++it_img) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(it_img->first);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(it_img->first);
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_img->first);

    if(it_klt != m_mapOfKltTrackers.end() && it_camPose != mapOfCameraPoses.end() && it_cam != mapOfCameraParameters.end()) {
      it_klt->second->display(*it_img->second, it_camPose->second, it_cam->second, col, thickness, displayFullModel);
    } else {
      std::cerr << "Missing elements ! " << std::endl;
    }
  }
}

/*!
  Get the camera names

  \return The vector of camera names.
*/
std::vector<std::string> vpMbKltMultiTracker::getCameraNames() const {
  std::vector<std::string> cameraNames;

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    cameraNames.push_back(it_klt->first);
  }

  return cameraNames;
}

/*!
  Get the camera parameters for the reference camera.

  \param camera : Copy of the camera parameters used by the tracker.
*/
void vpMbKltMultiTracker::getCameraParameters(vpCameraParameters &camera) const {
  //Get the reference camera parameters
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
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
void vpMbKltMultiTracker::getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->getCameraParameters(cam1);
    ++it;

    it->second->getCameraParameters(cam2);
  } else {
    std::cerr << "Problem with the number of cameras ! There are "
        << m_mapOfKltTrackers.size() << " cameras !" << std::endl;
  }
}

/*!
  Get the camera parameters specified by its name.

  \param cameraName : Name of the camera.
  \param camera : Copy of the camera parameters.
*/
void vpMbKltMultiTracker::getCameraParameters(const std::string &cameraName, vpCameraParameters &camera) const {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->getCameraParameters(camera);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get all the camera parameters.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbKltMultiTracker::getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const {
  //Clear the input map
  mapOfCameraParameters.clear();

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
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
unsigned int vpMbKltMultiTracker::getClipping(const std::string &cameraName) const {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
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
vpMbHiddenFaces<vpMbtPolygon>& vpMbKltMultiTracker::getFaces() {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    return it->second->getFaces();
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " cannot be found !" << std::endl;
  return faces;
}

/*!
  Return a reference to the faces structure for the given camera name.

  \return Reference to the face structure.
 */
vpMbHiddenFaces<vpMbtPolygon>& vpMbKltMultiTracker::getFaces(const std::string &cameraName) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    return it->second->getFaces();
  }

  std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  return faces;
}

/*!
  Return a map of faces structure for each camera.

  \return Reference a map of the face structure for each camera.
 */
std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > vpMbKltMultiTracker::getFaces() const {
  std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > mapOfFaces;
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfFaces[it->first] = it->second->faces;
  }

  return mapOfFaces;
}

/*!
  Return the address of the circle feature list.

  \return The address of the circle feature list.
*/
std::list<vpMbtDistanceCircle*>& vpMbKltMultiTracker::getFeaturesCircle() {
  std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end()) {
    return it_klt->second->getFeaturesCircle();
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }

  return circles_disp;
}

/*!
  Return the address of the circle feature list for the given camera.

  \param cameraName : Camera name.
  \return The address of the circle feature list.
*/
std::list<vpMbtDistanceCircle*>& vpMbKltMultiTracker::getFeaturesCircle(const std::string &cameraName) {
  std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.find(cameraName);

  if(it != m_mapOfKltTrackers.end()) {
    return it->second->getFeaturesCircle();
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !";
  }

  return circles_disp;
}

/*!
  Return the address of the Klt feature list.

  \return The address of the Klt feature list.

*/
std::list<vpMbtDistanceKltPoints*>& vpMbKltMultiTracker::getFeaturesKlt() {
  std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end()) {
    return it_klt->second->getFeaturesKlt();
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }

  return kltPolygons;
}

/*!
  Return the address of the Klt feature list for the given camera.

  \param cameraName : Camera name.
  \return The address of the Klt feature list.

*/
std::list<vpMbtDistanceKltPoints*>& vpMbKltMultiTracker::getFeaturesKlt(const std::string &cameraName) {
  std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.find(cameraName);

  if(it != m_mapOfKltTrackers.end()) {
    return it->second->getFeaturesKlt();
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !";
  }

  return kltPolygons;
}

/*!
  Return the address of the cylinder feature list.

  \return The address of the cylinder feature list.
*/
std::list<vpMbtDistanceKltCylinder*>& vpMbKltMultiTracker::getFeaturesKltCylinder() {
  std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end()) {
    return it_klt->second->getFeaturesKltCylinder();
  } else {
    std::cerr << "Cannot find reference camera: " << m_referenceCameraName << " !" << std::endl;
  }

  return kltCylinders;
}

/*!
  Return the address of the cylinder feature list for the given camera.

  \param cameraName : Camera name.
  \return The address of the cylinder feature list.
*/
std::list<vpMbtDistanceKltCylinder*>& vpMbKltMultiTracker::getFeaturesKltCylinder(const std::string &cameraName) {
  std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.find(cameraName);

  if(it != m_mapOfKltTrackers.end()) {
    return it->second->getFeaturesKltCylinder();
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !";
  }

  return kltCylinders;
}

/*!
  Get the current list of KLT points for each camera.

  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f.
  This function convert and copy the openCV KLT points into vpImagePoints.

  \return the list of KLT points through vpKltOpencv for each camera.
*/
std::map<std::string, std::vector<vpImagePoint> > vpMbKltMultiTracker::getKltImagePoints() const {
  std::map<std::string, std::vector<vpImagePoint> > mapOfFeatures;

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfFeatures[it->first] = it->second->getKltImagePoints();
  }

  return mapOfFeatures;
}

/*!
  Get the current list of KLT points and their id for each camera.

  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f.
  This function convert and copy the openCV KLT points into vpImagePoints.

  \return the list of KLT points and their id through vpKltOpencv for each camera.
*/
std::map<std::string, std::map<int, vpImagePoint> > vpMbKltMultiTracker::getKltImagePointsWithId() const {
  std::map<std::string, std::map<int, vpImagePoint> > mapOfFeatures;

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfFeatures[it->first] = it->second->getKltImagePointsWithId();
  }

  return mapOfFeatures;
}

/*!
  Get the klt tracker at the current state for each camera.

  \return klt tracker.
*/
std::map<std::string, vpKltOpencv> vpMbKltMultiTracker::getKltOpencv() const {
  std::map<std::string, vpKltOpencv> mapOfKltOpenCVTracker;

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfKltOpenCVTracker[it->first] = it->second->getKltOpencv();
  }

  return mapOfKltOpenCVTracker;
}

/*!
  Get the current list of KLT points.

  \return The list of KLT points through vpKltOpencv.
*/
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
std::map<std::string, std::vector<cv::Point2f> > vpMbKltMultiTracker::getKltPoints() const {
  std::map<std::string, std::vector<cv::Point2f> > mapOfFeatures;

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfFeatures[it->first] = it->second->getKltPoints();
  }

  return mapOfFeatures;
}
#else
std::map<std::string, CvPoint2D32f*> vpMbKltMultiTracker::getKltPoints() {
  std::map<std::string, CvPoint2D32f*> mapOfFeatures;

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfFeatures[it->first] = it->second->getKltPoints();
  }

  return mapOfFeatures;
}
#endif

/*!
  Get the current number of klt points for each camera.

  \return The number of features
*/
std::map<std::string, int> vpMbKltMultiTracker::getKltNbPoints() const {
  std::map<std::string, int> mapOfFeatures;

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfFeatures[it->first] = it->second->getKltNbPoints();
  }

  return mapOfFeatures;
}

/*!
  Get the number of polygons (faces) representing the object to track.

  \return Number of polygons.
*/
unsigned int vpMbKltMultiTracker::getNbPolygon() const {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    return it->second->getNbPolygon();
  }

  std::cerr << "The reference camera: " << m_referenceCameraName << " cannot be found !" << std::endl;
  return 0;
}

/*!
  Get the number of polygons (faces) representing the object to track for all the cameras.

  \return Number of polygons for the specified camera.
*/
std::map<std::string, unsigned int> vpMbKltMultiTracker::getMultiNbPolygon() const {
  std::map<std::string, unsigned int> mapOfNbPolygons;
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    mapOfNbPolygons[it->first] = it->second->getNbPolygon();
  }

  return mapOfNbPolygons;
}

/*!
  Get the current pose between the object and the cameras.

  \param c1Mo : The camera pose for the first camera.
  \param c2Mo : The camera pose for the second camera.
*/
void vpMbKltMultiTracker::getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->getPose(c1Mo);
    ++it;

    it->second->getPose(c2Mo);
  } else {
    std::cerr << "Require two cameras ! There are "
        << m_mapOfKltTrackers.size() << " cameras !" << std::endl;
  }
}

/*!
  Get the current pose between the object and the camera.
  cMo is the matrix which can be used to express
  coordinates from the object frame to camera frame.

  \param cameraName : The name of the camera.
  \param cMo_ : The camera pose for the specified camera.
*/
void vpMbKltMultiTracker::getPose(const std::string &cameraName, vpHomogeneousMatrix &cMo_) const {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->getPose(cMo_);
  } else {
    std::cerr << "The camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Get the current pose between the object and the cameras.

  \param mapOfCameraPoses : The map of camera poses for all the cameras.
*/
void vpMbKltMultiTracker::getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const {
  //Clear the map
  mapOfCameraPoses.clear();

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    vpHomogeneousMatrix cMo_;
    it->second->getPose(cMo_);
    mapOfCameraPoses[it->first] = cMo_;
  }
}

void vpMbKltMultiTracker::init(const vpImage<unsigned char>& /*I*/) {
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
void vpMbKltMultiTracker::initClick(const vpImage<unsigned char>& I, const std::vector<vpPoint> &points3D_list,
    const std::string &displayFile) {
  if(m_mapOfKltTrackers.empty()) {
    throw vpException(vpTrackingException::initializationError, "There is no camera !");
  } else if(m_mapOfKltTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "There is more than one camera !");
  } else {
    //Get the vpMbKltTracker object for the reference camera name
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
    if(it != m_mapOfKltTrackers.end()) {
      it->second->initClick(I, points3D_list, displayFile);
      it->second->getPose(cMo);

      //Init c0Mo
      c0Mo = cMo;
      ctTc0.eye();
    } else {
      std::stringstream ss;
      ss << "Cannot initClick as the reference camera: " << m_referenceCameraName << " does not exist !";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
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
void vpMbKltMultiTracker::initClick(const vpImage<unsigned char>& I, const std::string& initFile, const bool displayHelp) {
  if(m_mapOfKltTrackers.empty()) {
    throw vpException(vpTrackingException::initializationError, "There is no camera !");
  } else if(m_mapOfKltTrackers.size() > 1) {
    throw vpException(vpTrackingException::initializationError, "There is more than one camera !");
  } else {
    //Get the vpMbKltTracker object for the reference camera name
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
    if(it != m_mapOfKltTrackers.end()) {
      it->second->initClick(I, initFile, displayHelp);
      it->second->getPose(cMo);

      //Init c0Mo
      c0Mo = cMo;
      ctTc0.eye();
    } else {
      std::stringstream ss;
      ss << "Cannot initClick as the reference camera: " << m_referenceCameraName << " does not exist !";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
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
void vpMbKltMultiTracker::initClick(const vpImage<unsigned char>& I1, const vpImage<unsigned char> &I2,
    const std::string& initFile1, const std::string& initFile2, const bool displayHelp, const bool firstCameraIsReference) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->initClick(I1, initFile1, displayHelp);

    if(firstCameraIsReference) {
      //Set the reference cMo
      it->second->getPose(cMo);

      //Set the reference camera parameters
      it->second->getCameraParameters(this->cam);

      //Init c0Mo and ctTc0
      c0Mo = cMo;
      ctTc0.eye();
    }

    ++it;

    it->second->initClick(I2, initFile2, displayHelp);

    if(!firstCameraIsReference) {
      //Set the reference cMo
      it->second->getPose(cMo);

      //Set the reference camera parameters
      it->second->getCameraParameters(this->cam);

      //Init c0Mo and ctTc0
      c0Mo = cMo;
      ctTc0.eye();
    }
  } else {
    std::stringstream ss;
    ss << "Cannot init click ! Require two cameras but there are " << m_mapOfKltTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str().c_str());
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
void vpMbKltMultiTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::string &initFile, const bool displayHelp) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it_klt != m_mapOfKltTrackers.end()) {
    std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);

    if(it_img != mapOfImages.end()) {
      it_klt->second->initClick(*it_img->second, initFile, displayHelp);

      //Set the reference cMo
      it_klt->second->getPose(cMo);

      //Init c0Mo
      c0Mo = cMo;
      ctTc0.eye();

      //Set the pose for the others cameras
      for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
        if(it_klt->first != m_referenceCameraName) {
          it_img = mapOfImages.find(it_klt->first);
          std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
              m_mapOfCameraTransformationMatrix.find(it_klt->first);

          if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
            vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
            it_klt->second->cMo = cCurrentMo;
            it_klt->second->init(*it_img->second);
          } else {
            std::stringstream ss;
            ss << "Cannot init click ! Missing image for camera: " << m_referenceCameraName << " !";
            throw vpException(vpTrackingException::initializationError, ss.str().c_str());
          }
        }
      }
    } else {
      std::stringstream ss;
      ss << "Cannot init click ! Missing image for camera: " << m_referenceCameraName << " !";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  } else {
    std::stringstream ss;
    ss << "Cannot init click ! The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str().c_str());
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
void vpMbKltMultiTracker::initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, std::string>::const_iterator it_initFile = mapOfInitFiles.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end() && it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
    it_klt->second->initClick(*it_img->second, it_initFile->second, displayHelp);
    it_klt->second->getPose(cMo);

    c0Mo = this->cMo;
    ctTc0.eye();
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot initClick for the reference camera !");
  }

  //Vector of missing initFile for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    if(it_klt->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_klt->first);
      it_initFile = mapOfInitFiles.find(it_klt->first);

      if(it_img != mapOfImages.end() && it_initFile != mapOfInitFiles.end()) {
        //InitClick for the current camera
        it_klt->second->initClick(*it_img->second, it_initFile->second, displayHelp);
      } else {
        vectorOfMissingCameraPoses.push_back(it_klt->first);
      }
    }
  }

  //Set pose for cameras that do not have an initFile
  for(std::vector<std::string>::const_iterator it1 = vectorOfMissingCameraPoses.begin();
      it1 != vectorOfMissingCameraPoses.end(); ++it1) {
    it_img = mapOfImages.find(*it1);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it1);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfKltTrackers[*it1]->cMo = cCurrentMo;
      m_mapOfKltTrackers[*it1]->init(*it_img->second);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix ! Cannot set the pose for camera: " << (*it1) << " !";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }
}
#endif //#ifdef VISP_HAVE_MODULE_GUI

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
void vpMbKltMultiTracker::initFromPose(const vpImage<unsigned char>& I, const std::string &initFile) {
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

  //Set the new pose for the reference camera
  cMo.buildFrom(init_pos);
  c0Mo = cMo;
  ctTc0.eye();

  //Init for the reference camera
  std::map<std::string, vpMbKltTracker *>::iterator it_ref = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it_ref == m_mapOfKltTrackers.end()) {
    throw vpException(vpTrackingException::initializationError, "Cannot find the reference camera !");
  }

  it_ref->second->initFromPose(I, cMo);
}

/*!
  Initialise the tracking thanks to the pose.

  \param I : Input image
  \param cMo_ : Pose matrix.
*/
void vpMbKltMultiTracker::initFromPose(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo_) {
  this->cMo = cMo_;
  c0Mo = cMo;
  ctTc0.eye();

  //Init for the reference camera
  std::map<std::string, vpMbKltTracker *>::iterator it_ref = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it_ref == m_mapOfKltTrackers.end()) {
    throw vpException(vpTrackingException::initializationError, "Cannot find the reference camera !");
  }

  it_ref->second->initFromPose(I, cMo);
}

/*!
  Initialise the tracking thanks to the pose vector.

  \param I : Input image
  \param cPo : Pose vector.
*/
void vpMbKltMultiTracker::initFromPose (const vpImage<unsigned char>& I, const vpPoseVector &cPo) {
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
void vpMbKltMultiTracker::initFromPose(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const bool firstCameraIsReference) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->initFromPose(I1, c1Mo);

    ++it;

    it->second->initFromPose(I2, c2Mo);

    if(firstCameraIsReference) {
      this->cMo = c1Mo;
    } else {
      this->cMo = c2Mo;
    }

    c0Mo = this->cMo;
    ctTc0.eye();
  } else {
    std::stringstream ss;
    ss << "This method requires 2 cameras but there are " << m_mapOfKltTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Initialize the tracking thanks to the pose. The camera transformation matrices have to be set before.

  \param mapOfImages : Map of images.
  \param cMo_ : Pose matrix for the reference camera.
*/
void vpMbKltMultiTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const vpHomogeneousMatrix &cMo_) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end()) {
    std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(it_klt->first);

    if(it_img != mapOfImages.end()) {
      it_klt->second->initFromPose(*it_img->second, cMo_);

      cMo = cMo_;
      c0Mo = this->cMo;
      ctTc0.eye();

      for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
        if(it_klt->first != m_referenceCameraName) {
          std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
              m_mapOfCameraTransformationMatrix.find(it_klt->first);
          it_img = mapOfImages.find(it_klt->first);

          if(it_camTrans != m_mapOfCameraTransformationMatrix.end() && it_img != mapOfImages.end()) {
            vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
            it_klt->second->initFromPose(*it_img->second, cCurrentMo);
          } else {
            throw vpException(vpTrackingException::initializationError, "Cannot find camera transformation matrix or image !");
          }
        }
      }
    } else {
      throw vpException(vpTrackingException::initializationError, "Cannot find image for reference camera !");
    }
  } else {
    std::stringstream ss;
    ss << "Cannot find reference camera: " << m_referenceCameraName << std::endl;
    throw vpException(vpTrackingException::initializationError, ss.str());
  }
}

/*!
  Initialize the tracking thanks to the pose.

  \param mapOfImages : Map of images.
  \param mapOfCameraPoses : Map of pose matrix.
*/
void vpMbKltMultiTracker::initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    it_klt->second->initFromPose(*it_img->second, it_camPose->second);
    it_klt->second->getPose(cMo);

    c0Mo = this->cMo;
    ctTc0.eye();
  } else {
    throw vpException(vpTrackingException::initializationError, "Cannot set pose for the reference camera !");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    it_img = mapOfImages.find(it_klt->first);
    it_camPose = mapOfCameraPoses.find(it_klt->first);

    if(it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
      //Set pose
      it_klt->second->initFromPose(*it_img->second, it_camPose->second);
    } else {
      vectorOfMissingCameraPoses.push_back(it_klt->first);
    }
  }

  for(std::vector<std::string>::const_iterator it1 = vectorOfMissingCameraPoses.begin();
      it1 != vectorOfMissingCameraPoses.end(); ++it1) {
    it_img = mapOfImages.find(*it1);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it1);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfKltTrackers[*it1]->initFromPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix ! Cannot set the pose for camera: " << (*it1) << " !";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }
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
void vpMbKltMultiTracker::loadConfigFile(const std::string &configFile) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    //Load ConfigFile for reference camera
    it->second->loadConfigFile(configFile);
    it->second->getCameraParameters(cam);

    //Set clipping
    this->clippingFlag = it->second->getClipping();
    this->angleAppears = it->second->getAngleAppear();
    this->angleDisappears = it->second->getAngleDisappear();
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str().c_str());
  }
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
void vpMbKltMultiTracker::loadConfigFile(const std::string& configFile1, const std::string& configFile2,
    const bool firstCameraIsReference) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->loadConfigFile(configFile1);

    if(firstCameraIsReference) {
      it->second->getCameraParameters(cam);

      //Set clipping
      this->clippingFlag = it->second->getClipping();
      this->angleAppears = it->second->getAngleAppear();
      this->angleDisappears = it->second->getAngleDisappear();
    }
    ++it;

    it->second->loadConfigFile(configFile2);

    if(!firstCameraIsReference) {
      it->second->getCameraParameters(cam);

      //Set clipping
      this->clippingFlag = it->second->getClipping();
      this->angleAppears = it->second->getAngleAppear();
      this->angleDisappears = it->second->getAngleDisappear();
    }
  } else {
    std::stringstream ss;
    ss << "Cannot loadConfigFile. Require two cameras ! There are " << m_mapOfKltTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::initializationError, ss.str().c_str());
  }
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
void vpMbKltMultiTracker::loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles) {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    std::map<std::string, std::string>::const_iterator it_config = mapOfConfigFiles.find(it_klt->first);
    if(it_config != mapOfConfigFiles.end()) {
      it_klt->second->loadConfigFile(it_config->second);
    } else {
      std::stringstream ss;
      ss << "Missing configuration file for camera: " << it_klt->first << " !";
      throw vpException(vpTrackingException::initializationError, ss.str().c_str());
    }
  }

  //Set the reference camera parameters
  std::map<std::string, vpMbKltTracker *>::iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->getCameraParameters(cam);

    //Set clipping
    this->clippingFlag = it->second->getClipping();
    this->angleAppears = it->second->getAngleAppear();
    this->angleDisappears = it->second->getAngleDisappear();
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::initializationError, ss.str().c_str());
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
void vpMbKltMultiTracker::loadModel(const std::string &modelFile, const bool verbose) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->loadModel(modelFile, verbose);
  }

  modelInitialised = true;
}

void vpMbKltMultiTracker::preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  m_nbInfos = 0;
  m_nbFaceUsed = 0;

  vpMbKltTracker *klt;
  for (std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;

    try {
      klt->preTracking(*mapOfImages[it->first]);
      m_nbInfos += klt->m_nbInfos;
      m_nbFaceUsed += klt->m_nbFaceUsed;
    } catch (...) {  }
  }
}

void vpMbKltMultiTracker::postTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  vpMbKltTracker *klt;

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin(); it != m_mapOfKltTrackers.end(); ++it) {
    klt = it->second;

    //Set the camera pose
    it->second->cMo = m_mapOfCameraTransformationMatrix[it->first]*cMo;

    if (klt->m_nbInfos > 0 && klt->postTracking(*mapOfImages[it->first], klt->m_w_klt)) {
      klt->reinit(*mapOfImages[it->first]);

      //set ctTc0 to identity
      if(it->first == m_referenceCameraName) {
        reinit(/*mapOfImages[it->first]*/);
      }
    }
  }
}

/*!
 The parameter is not used.
 */
void vpMbKltMultiTracker::reinit(/* const vpImage<unsigned char>& I*/) {
  c0Mo = cMo;
  ctTc0.eye();
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new model
  \param verbose : verbose option to print additional information when loading CAO model files which include other
  CAO model files.
*/
void vpMbKltMultiTracker::reInitModel(const vpImage<unsigned char>& I, const std::string &cad_name,
    const vpHomogeneousMatrix& cMo_, const bool verbose) {
  if(m_mapOfKltTrackers.size() != 1) {
    std::stringstream ss;
    ss << "This method requires exactly one camera, there are " << m_mapOfKltTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }

  firstInitialisation = true;
  modelInitialised = true;

  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it_klt != m_mapOfKltTrackers.end()) {
    it_klt->second->reInitModel(I, cad_name, cMo_, verbose);

    //Set reference pose
    it_klt->second->getPose(cMo);

    c0Mo = cMo;
    ctTc0.eye();
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel the reference camera !");
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
void vpMbKltMultiTracker::reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &cad_name, const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
    const bool verbose, const bool firstCameraIsReference) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it_edge = m_mapOfKltTrackers.begin();

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

    c0Mo = cMo;
    ctTc0.eye();
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
void vpMbKltMultiTracker::reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::string &cad_name, const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
    const bool verbose) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    it_klt->second->reInitModel(*it_img->second, cad_name, it_camPose->second, verbose);
    modelInitialised = true;

    //Set reference pose
    it_klt->second->getPose(cMo);

    c0Mo = cMo;
    ctTc0.eye();
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot reInitModel for reference camera !");
  }

  std::vector<std::string> vectorOfMissingCameras;
  for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    if(it_klt->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_klt->first);
      it_camPose = mapOfCameraPoses.find(it_klt->first);

      if(it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        it_klt->second->reInitModel(*it_img->second, cad_name, it_camPose->second, verbose);
      } else {
        vectorOfMissingCameras.push_back(it_klt->first);
      }
    }
  }

  for(std::vector<std::string>::const_iterator it = vectorOfMissingCameras.begin();
      it != vectorOfMissingCameras.end(); ++it) {
    it_img = mapOfImages.find(*it);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfKltTrackers[*it]->reInitModel(*it_img->second, cad_name, cCurrentMo, verbose);
    }
  }
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void vpMbKltMultiTracker::resetTracker() {
  cMo.eye();
  ctTc0.eye();

  //Call resetTracker for all the cameras
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->resetTracker();
  }

  m_computeInteraction = true;
  firstInitialisation = true;
  computeCovariance = false;

  angleAppears = vpMath::rad(65);
  angleDisappears = vpMath::rad(75);

  clippingFlag = vpPolygon3D::NO_CLIPPING;

  maskBorder = 5;
  threshold_outlier = 0.5;
  percentGood = 0.7;

  m_lambda = 0.8;
  m_maxIter = 200;

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif
}

/*!
  Set the angle used to test polygons appearance.
  If the angle between the normal of the polygon and the line going
  from the camera to the polygon center has a value lower than
  this parameter, the polygon is considered as appearing.
  The polygon will then be tracked.

  \param a : new angle in radian.
*/
void vpMbKltMultiTracker::setAngleAppear(const double &a) {
  vpMbTracker::setAngleAppear(a);

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
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
void vpMbKltMultiTracker::setAngleDisappear(const double &a) {
  vpMbTracker::setAngleDisappear(a);

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setAngleDisappear(a);
  }
}

/*!
  Set the camera parameters for the monocular case.

  \param camera : The new camera parameters.
*/
void vpMbKltMultiTracker::setCameraParameters(const vpCameraParameters& camera) {
  if(m_mapOfKltTrackers.empty()) {
    throw vpException(vpTrackingException::fatalError, "There is no camera !");
  } else if(m_mapOfKltTrackers.size() > 1) {
    throw vpException(vpTrackingException::fatalError, "There is more than one camera !");
  } else {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
    if(it != m_mapOfKltTrackers.end()) {
      it->second->setCameraParameters(camera);

      //Set reference camera parameters
      this->cam = camera;
    } else {
      std::stringstream ss;
      ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  }
}

/*!
  Set the camera parameters for the stereo cameras case.

  \param camera1 : The new camera parameters for the first camera.
  \param camera2 : The new camera parameters for the second camera.
  \param firstCameraIsReference : If true, the first camera is the reference, otherwise it is the second one.
*/
void vpMbKltMultiTracker::setCameraParameters(const vpCameraParameters& camera1, const vpCameraParameters& camera2,
    const bool firstCameraIsReference) {
  if(m_mapOfKltTrackers.empty()) {
    throw vpException(vpTrackingException::fatalError, "There is no camera !");
  } else if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
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
    ss << "Require two cameras ! There are " << m_mapOfKltTrackers.size() << " cameras !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Set the camera parameters for the specified camera.

  \param cameraName : Camera name.
  \param camera : The new camera parameters.
*/
void vpMbKltMultiTracker::setCameraParameters(const std::string &cameraName, const vpCameraParameters& camera) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->setCameraParameters(camera);

    if(it->first == m_referenceCameraName) {
      this->cam = camera;
    }
  } else {
    std::stringstream ss;
    ss << "The camera: " << cameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Set the camera parameters for all the cameras.

  \param mapOfCameraParameters : Map of camera parameters.
*/
void vpMbKltMultiTracker::setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters) {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    std::map<std::string, vpCameraParameters>::const_iterator it_cam = mapOfCameraParameters.find(it_klt->first);
    if(it_cam != mapOfCameraParameters.end()) {
      it_klt->second->setCameraParameters(it_cam->second);

      if(it_klt->first == m_referenceCameraName) {
        this->cam = it_cam->second;
      }
    } else {
      std::stringstream ss;
      ss << "Missing camera parameters for camera: " << it_klt->first << " !";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  }
}

/*!
  Set the camera transformation matrix for the specified camera (\f$ _{}^{c_{current}}\textrm{M}_{c_{reference}} \f$).

  \param cameraName : Camera name.
  \param cameraTransformationMatrix : Camera transformation matrix between the current and the reference camera.
*/
void vpMbKltMultiTracker::setCameraTransformationMatrix(const std::string &cameraName,
    const vpHomogeneousMatrix &cameraTransformationMatrix) {
  std::map<std::string, vpHomogeneousMatrix>::iterator it = m_mapOfCameraTransformationMatrix.find(cameraName);
  if(it != m_mapOfCameraTransformationMatrix.end()) {
    it->second = cameraTransformationMatrix;
  } else {
    std::stringstream ss;
    ss << "Cannot find camera: " << cameraName << " !";
    throw vpException(vpTrackingException::fatalError, ss.str());
  }
}

/*!
  Set the map of camera transformation matrices
  (\f$ _{}^{c_1}\textrm{M}_{c_1}, _{}^{c_2}\textrm{M}_{c_1}, _{}^{c_3}\textrm{M}_{c_1}, \cdots, _{}^{c_n}\textrm{M}_{c_1} \f$).

  \param mapOfTransformationMatrix : map of camera transformation matrices.
*/
void vpMbKltMultiTracker::setCameraTransformationMatrix(
    const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix) {
  //Check if all cameras have a transformation matrix
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = mapOfTransformationMatrix.find(it_klt->first);

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
void vpMbKltMultiTracker::setClipping(const unsigned int &flags) {
  vpMbTracker::setClipping(flags);

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setClipping(flags);
  }
}

/*!
  Specify which clipping to use for the specified camera.

  \sa vpMbtPolygonClipping

  \param cameraName : Camera to set the clipping.
  \param flags : New clipping flags.
*/
void vpMbKltMultiTracker::setClipping(const std::string &cameraName, const unsigned int &flags) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->setClipping(flags);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Set if the covariance matrix has to be computed.

  \param flag : True if the covariance has to be computed, false otherwise
*/
void vpMbKltMultiTracker::setCovarianceComputation(const bool& flag) {
  vpMbTracker::setCovarianceComputation(flag);

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setCovarianceComputation(flag);
  }
}

/*!
  Enable to display the KLT features.

  \param displayF : set it to true to display the features.
*/
void vpMbKltMultiTracker::setDisplayFeatures(const bool displayF) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setDisplayFeatures(displayF);
  }

  displayFeatures = displayF;
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.
*/
void vpMbKltMultiTracker::setFarClippingDistance(const double &dist) {
  vpMbTracker::setFarClippingDistance(dist);

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setFarClippingDistance(dist);
  }
}

/*!
  Set the far distance for clipping for the specified camera.

  \param cameraName : Camera to set the far clipping.
  \param dist : Far clipping value.
*/
void vpMbKltMultiTracker::setFarClippingDistance(const std::string &cameraName, const double &dist) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->setFarClippingDistance(dist);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
  }
}

#ifdef VISP_HAVE_OGRE
/*!
  Set the ratio of visibility attempts that has to be successful to consider a polygon as visible.

  \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

  \param ratio : Ratio of succesful attempts that has to be considered. Value has to be between 0.0 (0%) and 1.0 (100%).
*/
  void vpMbKltMultiTracker::setGoodNbRayCastingAttemptsRatio(const double &ratio) {
    for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
        it != m_mapOfKltTrackers.end(); ++it) {
      it->second->setGoodNbRayCastingAttemptsRatio(ratio);
    }
  }

  /*!
    Set the number of rays that will be sent toward each polygon for visibility test.
    Each ray will go from the optic center of the camera to a random point inside the considered polygon.

    \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

    \param attempts Number of rays to be sent.
  */
  void vpMbKltMultiTracker::setNbRayCastingAttemptsForVisibility(const unsigned int &attempts) {
    for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
        it != m_mapOfKltTrackers.end(); ++it) {
      it->second->setNbRayCastingAttemptsForVisibility(attempts);
    }
  }
#endif

  /*!
    Set the new value of the klt tracker.

    \param t : Klt tracker containing the new values.
  */
void vpMbKltMultiTracker::setKltOpencv(const vpKltOpencv& t) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    it_klt->second->setKltOpencv(t);
  }
}

/*!
  Set the new value of the klt tracker for the specified cameras.

  \param mapOfOpenCVTrackers : Map of Klt trackers containing the new values.
*/
void vpMbKltMultiTracker::setKltOpencv(const std::map<std::string, vpKltOpencv> &mapOfOpenCVTrackers) {
  for(std::map<std::string, vpKltOpencv>::const_iterator it_kltOpenCV = mapOfOpenCVTrackers.begin();
      it_kltOpenCV != mapOfOpenCVTrackers.end(); ++it_kltOpenCV) {
    std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.find(it_kltOpenCV->first);
    if(it_klt != m_mapOfKltTrackers.end()) {
      it_klt->second->setKltOpencv(it_kltOpenCV->second);
    } else {
      std::cerr << "The camera: " << it_kltOpenCV->first << " does not exist !" << std::endl;
    }
  }
}

/*!
  Set the flag to consider if the level of detail (LOD) is used for all the cameras.

  \param useLod : true if the level of detail must be used, false otherwise. When true,
  two parameters can be set, see setMinLineLengthThresh() and setMinPolygonAreaThresh().
  \param name : name of the face we want to modify the LOD parameter.

  \sa setMinLineLengthThresh(), setMinPolygonAreaThresh()
 */
void vpMbKltMultiTracker::setLod(const bool useLod, const std::string &name) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
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
void vpMbKltMultiTracker::setLod(const bool useLod, const std::string &cameraName, const std::string &name) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(cameraName);

  if(it_klt != m_mapOfKltTrackers.end()) {
    it_klt->second->setLod(useLod, name);
  } else {
    std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  }
}

/*!
  Set the erosion of the mask used on the Model faces.

  \param  e : The desired erosion.
*/
void vpMbKltMultiTracker::setKltMaskBorder(const unsigned int &e) {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setKltMaskBorder(e);
  }

  maskBorder = e;
}

/*!
  Useless for KLT tracker.
 */
void vpMbKltMultiTracker::setMinLineLengthThresh(const double /*minLineLengthThresh*/, const std::string &/*name*/) {
  std::cerr << "Useless for KLT tracker !" << std::endl;
}

/*!
  Set the minimum polygon area to be considered as visible in the LOD case.

  \param minPolygonAreaThresh : threshold for the minimum polygon area in pixel.
  \param name : name of the face we want to modify the LOD threshold.

  \sa setLod(), setMinLineLengthThresh()
 */
void vpMbKltMultiTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
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
void vpMbKltMultiTracker::setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &cameraName,
    const std::string &name) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(cameraName);

  if(it_klt != m_mapOfKltTrackers.end()) {
    it_klt->second->setMinPolygonAreaThresh(minPolygonAreaThresh, name);
  } else {
    std::cerr << "The camera: " << cameraName << " cannot be found !" << std::endl;
  }
}

/*!
  Set the near distance for clipping.

  \param dist : Near clipping value.
*/
void vpMbKltMultiTracker::setNearClippingDistance(const double &dist) {
  vpMbTracker::setNearClippingDistance(dist);

  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setNearClippingDistance(dist);
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
void vpMbKltMultiTracker::setOgreShowConfigDialog(const bool showConfigDialog) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setOgreShowConfigDialog(showConfigDialog);
  }
}

/*!
  Use Ogre3D for visibility tests

  \warning This function has to be called before the initialization of the tracker.

  \param v : True to use it, False otherwise
*/
void vpMbKltMultiTracker::setOgreVisibilityTest(const bool &v) {
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setOgreVisibilityTest(v);
  }

#ifdef VISP_HAVE_OGRE
  for(std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->faces.getOgreContext()->setWindowName("Multi MBT Klt (" + it->first + ")");
  }
#endif

  useOgre = v;
}

/*!
  Set the near distance for clipping for the specified camera.

  \param cameraName : Camera name to set the near clipping distance.
  \param dist : Near clipping value.
*/
void vpMbKltMultiTracker::setNearClippingDistance(const std::string &cameraName, const double &dist) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(cameraName);
  if(it != m_mapOfKltTrackers.end()) {
    it->second->setNearClippingDistance(dist);
  } else {
    std::cerr << "Camera: " << cameraName << " does not exist !" << std::endl;
  }
}

/*!
  Set the optimization method used during the tracking.

  \param opt : Optimization method to use.
*/
void vpMbKltMultiTracker::setOptimizationMethod(const vpMbtOptimizationMethod &opt) {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
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
void vpMbKltMultiTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_) {
  if(m_mapOfKltTrackers.size() == 1) {
    std::map<std::string, vpMbKltTracker *>::iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);
    if(it != m_mapOfKltTrackers.end()) {
      it->second->setPose(I, cMo_);
      this->cMo = cMo_;

      c0Mo = this->cMo;
      ctTc0.eye();
    } else {
      std::stringstream ss;
      ss << "Cannot find the reference camera: " << m_referenceCameraName << " !";
      throw vpException(vpTrackingException::fatalError, ss.str());
    }
  } else {
    std::stringstream ss;
    ss << "You are trying to set the pose with only one image and cMo "
        "but there are multiple cameras !";
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
void vpMbKltMultiTracker::setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix c2Mo, const bool firstCameraIsReference) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    it->second->setPose(I1, c1Mo);

    ++it;

    it->second->setPose(I2, c2Mo);

    if(firstCameraIsReference) {
      this->cMo = c1Mo;
    } else {
      this->cMo = c2Mo;
    }

    c0Mo = this->cMo;
    ctTc0.eye();
  } else {
    std::stringstream ss;
    ss << "This method requires 2 cameras but there are " << m_mapOfKltTrackers.size() << " cameras !";
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
void vpMbKltMultiTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const vpHomogeneousMatrix &cMo_) {
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  if(it_klt != m_mapOfKltTrackers.end()) {
    std::map<std::string, const vpImage<unsigned char> *>::const_iterator it_img = mapOfImages.find(m_referenceCameraName);

    if(it_img != mapOfImages.end()) {
      //Set pose on reference camera
      it_klt->second->setPose(*it_img->second, cMo_);

      //Set the reference cMo
      cMo = cMo_;

      c0Mo = this->cMo;
      ctTc0.eye();

      //Set the pose for the others cameras
      for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
        if(it_klt->first != m_referenceCameraName) {
          std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans =
              m_mapOfCameraTransformationMatrix.find(it_klt->first);
          it_img = mapOfImages.find(it_klt->first);

          if(it_camTrans != m_mapOfCameraTransformationMatrix.end() && it_img != mapOfImages.end()) {
            vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
            it_klt->second->setPose(*it_img->second, cCurrentMo);
          } else {
            throw vpException(vpTrackingException::fatalError, "Cannot find camera transformation matrix or image !");
          }
        }
      }
    } else {
      std::stringstream ss;
      ss << "Missing image for reference camera: " << m_referenceCameraName << " !";
      throw vpException(vpTrackingException::fatalError, ss.str().c_str());
    }
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
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
void vpMbKltMultiTracker::setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) {
  //Set the reference cMo
  std::map<std::string, vpMbKltTracker *>::const_iterator it_klt = m_mapOfKltTrackers.find(m_referenceCameraName);
  std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(m_referenceCameraName);
  std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camPose = mapOfCameraPoses.find(m_referenceCameraName);

  if(it_klt != m_mapOfKltTrackers.end() && it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
    it_klt->second->setPose(*it_img->second, it_camPose->second);
    it_klt->second->getPose(cMo);

    c0Mo = this->cMo;
    ctTc0.eye();
  } else {
    throw vpException(vpTrackingException::fatalError, "Cannot set pose for the reference camera !");
  }

  //Vector of missing pose matrices for cameras
  std::vector<std::string> vectorOfMissingCameraPoses;

  //Set pose for the specified cameras
  for(it_klt = m_mapOfKltTrackers.begin(); it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    if(it_klt->first != m_referenceCameraName) {
      it_img = mapOfImages.find(it_klt->first);
      it_camPose = mapOfCameraPoses.find(it_klt->first);

      if(it_img != mapOfImages.end() && it_camPose != mapOfCameraPoses.end()) {
        //Set pose
        it_klt->second->setPose(*it_img->second, it_camPose->second);
      } else {
        vectorOfMissingCameraPoses.push_back(it_klt->first);
      }
    }
  }

  for(std::vector<std::string>::const_iterator it1 = vectorOfMissingCameraPoses.begin();
      it1 != vectorOfMissingCameraPoses.end(); ++it1) {
    it_img = mapOfImages.find(*it1);
    std::map<std::string, vpHomogeneousMatrix>::const_iterator it_camTrans = m_mapOfCameraTransformationMatrix.find(*it1);

    if(it_img != mapOfImages.end() && it_camTrans != m_mapOfCameraTransformationMatrix.end()) {
      vpHomogeneousMatrix cCurrentMo = it_camTrans->second * cMo;
      m_mapOfKltTrackers[*it1]->setPose(*it_img->second, cCurrentMo);
    } else {
      std::stringstream ss;
      ss << "Missing image or missing camera transformation matrix ! Cannot set the pose for camera: " << (*it1) << " !";
      throw vpException(vpTrackingException::fatalError, ss.str().c_str());
    }
  }
}

/*!
  Set the reference camera name

  \param referenceCameraName : Name of the reference camera.
 */
void vpMbKltMultiTracker::setReferenceCameraName(const std::string &referenceCameraName) {
  std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.find(referenceCameraName);
  if(it != m_mapOfKltTrackers.end()) {
    m_referenceCameraName = referenceCameraName;
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }
}

/*!
  Use Scanline algorithm for visibility tests

  \param v : True to use it, False otherwise
*/
void vpMbKltMultiTracker::setScanLineVisibilityTest(const bool &v) {
  //Set general setScanLineVisibilityTest
  vpMbTracker::setScanLineVisibilityTest(v);

  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setScanLineVisibilityTest(v);
  }
}

/*!
  Set the threshold for the acceptation of a point.

  \param th : Threshold for the weight below which a point is rejected.
*/
void vpMbKltMultiTracker::setKltThresholdAcceptation(const double th) {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setKltThresholdAcceptation(th);
  }

  threshold_outlier = th;
}

/*!
  Set if the polygons that have the given name have to be considered during the tracking phase.

  \param name : name of the polygon(s).
  \param useKltTracking : True if it has to be considered, False otherwise.
*/
void vpMbKltMultiTracker::setUseKltTracking(const std::string &name, const bool &useKltTracking) {
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it = m_mapOfKltTrackers.begin();
      it != m_mapOfKltTrackers.end(); ++it) {
    it->second->setUseKltTracking(name, useKltTracking);
  }
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I : the input image
*/
void vpMbKltMultiTracker::track(const vpImage<unsigned char> &I) {
  //Track only with reference camera
  //Get the reference camera parameters
  std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.find(m_referenceCameraName);

  if(it != m_mapOfKltTrackers.end()) {
    it->second->track(I);
    it->second->getPose(cMo);
  } else {
    std::stringstream ss;
    ss << "The reference camera: " << m_referenceCameraName << " does not exist !";
    throw vpException(vpTrackingException::fatalError, ss.str().c_str());
  }
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I1 : The first image.
  \param I2 : The second image.
*/
void vpMbKltMultiTracker::track(const vpImage<unsigned char>& I1, const vpImage<unsigned char>& I2) {
  if(m_mapOfKltTrackers.size() == 2) {
    std::map<std::string, vpMbKltTracker *>::const_iterator it = m_mapOfKltTrackers.begin();
    std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
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
void vpMbKltMultiTracker::track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages) {
  //Check if there is an image for each camera
  for(std::map<std::string, vpMbKltTracker*>::const_iterator it_klt = m_mapOfKltTrackers.begin();
      it_klt != m_mapOfKltTrackers.end(); ++it_klt) {
    std::map<std::string, const vpImage<unsigned char>* >::const_iterator it_img = mapOfImages.find(it_klt->first);

    if(it_img == mapOfImages.end()) {
      throw vpException(vpTrackingException::fatalError, "Missing images !");
    }
  }

  preTracking(mapOfImages);

  if (m_nbInfos < 4 || m_nbFaceUsed == 0) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  }

  computeVVS();

  postTracking(mapOfImages);
}

//////////////// Deprecated ////////////////
/*!
  Get the current number of klt points for each camera.
  \deprecated Use rather getKltNbPoints()

  \return The number of features
*/
std::map<std::string, int> vpMbKltMultiTracker::getNbKltPoints() const {
  return getKltNbPoints();
}

/*!
  Set the erosion of the mask used on the Model faces.
  \deprecated Use rather setKltMaskBorder()

  \param  e : The desired erosion.
*/
void vpMbKltMultiTracker::setMaskBorder(const unsigned int &e) {
  setKltMaskBorder(e);
}

/*!
  Set the threshold for the acceptation of a point.

  \param th : Threshold for the weight below which a point is rejected.
*/
void vpMbKltMultiTracker::setThresholdAcceptation(const double th) {
  setKltThresholdAcceptation(th);
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(dummy_vpMbKltMultiTracker.cpp.o) has no symbols
void dummy_vpMbKltMultiTracker() {};
#endif //VISP_HAVE_OPENCV
