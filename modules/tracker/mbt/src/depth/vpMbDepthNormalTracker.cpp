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
 * Model-based tracker using depth normal features.
 *
 *****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_PCL
#include <pcl/point_cloud.h>
#endif

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/mbt/vpMbDepthNormalTracker.h>
#include <visp3/mbt/vpMbtXmlGenericParser.h>

#if DEBUG_DISPLAY_DEPTH_NORMAL
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#endif

vpMbDepthNormalTracker::vpMbDepthNormalTracker()
  : m_depthNormalFeatureEstimationMethod(vpMbtFaceDepthNormal::ROBUST_FEATURE_ESTIMATION),
    m_depthNormalHiddenFacesDisplay(), m_depthNormalListOfActiveFaces(),
    m_depthNormalListOfDesiredFeatures(), m_depthNormalFaces(), m_depthNormalPclPlaneEstimationMethod(2),
    m_depthNormalPclPlaneEstimationRansacMaxIter(200), m_depthNormalPclPlaneEstimationRansacThreshold(0.001),
    m_depthNormalSamplingStepX(2), m_depthNormalSamplingStepY(2), m_depthNormalUseRobust(false), m_error_depthNormal(),
    m_featuresToBeDisplayedDepthNormal(), m_L_depthNormal(), m_robust_depthNormal(), m_w_depthNormal(), m_weightedError_depthNormal()
#if DEBUG_DISPLAY_DEPTH_NORMAL
    ,
    m_debugDisp_depthNormal(NULL), m_debugImage_depthNormal()
#endif
{
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Depth");
#endif

#if defined(VISP_HAVE_X11) && DEBUG_DISPLAY_DEPTH_NORMAL
  m_debugDisp_depthNormal = new vpDisplayX;
#elif defined(VISP_HAVE_GDI) && DEBUG_DISPLAY_DEPTH_NORMAL
  m_debugDisp_depthNormal = new vpDisplayGDI;
#endif
}

vpMbDepthNormalTracker::~vpMbDepthNormalTracker()
{
  for (size_t i = 0; i < m_depthNormalFaces.size(); i++) {
    delete m_depthNormalFaces[i];
  }
}

void vpMbDepthNormalTracker::addFace(vpMbtPolygon &polygon, const bool alreadyClose)
{
  if (polygon.nbpt < 3) {
    return;
  }

  // Copy hidden faces
  m_depthNormalHiddenFacesDisplay = faces;

  vpMbtFaceDepthNormal *normal_face = new vpMbtFaceDepthNormal;
  normal_face->m_hiddenFace = &faces;
  normal_face->m_polygon = &polygon;
  normal_face->m_cam = cam;
  normal_face->m_useScanLine = useScanLine;
  normal_face->m_clippingFlag = clippingFlag;
  normal_face->m_distNearClip = distNearClip;
  normal_face->m_distFarClip = distFarClip;
  normal_face->setFeatureEstimationMethod(m_depthNormalFeatureEstimationMethod);
  normal_face->setPclPlaneEstimationMethod(m_depthNormalPclPlaneEstimationMethod);
  normal_face->setPclPlaneEstimationRansacMaxIter(m_depthNormalPclPlaneEstimationRansacMaxIter);
  normal_face->setPclPlaneEstimationRansacThreshold(m_depthNormalPclPlaneEstimationRansacThreshold);

  // Add lines that compose the face
  unsigned int nbpt = polygon.getNbPoint();
  if (nbpt > 0) {
    for (unsigned int i = 0; i < nbpt - 1; i++) {
      normal_face->addLine(polygon.p[i], polygon.p[i + 1], &m_depthNormalHiddenFacesDisplay, polygon.getIndex(),
                           polygon.getName());
    }

    if (!alreadyClose) {
      // Add last line that closes the face
      normal_face->addLine(polygon.p[nbpt - 1], polygon.p[0], &m_depthNormalHiddenFacesDisplay, polygon.getIndex(),
                           polygon.getName());
    }
  }

  // Construct a vpPlane in object frame
  vpPoint pts[3];
  pts[0] = polygon.p[0];
  pts[1] = polygon.p[1];
  pts[2] = polygon.p[2];
  normal_face->m_planeObject = vpPlane(pts[0], pts[1], pts[2], vpPlane::object_frame);

  m_depthNormalFaces.push_back(normal_face);
}

void vpMbDepthNormalTracker::computeVisibility(const unsigned int width, const unsigned int height)
{
  bool changed = false;
  faces.setVisible(width, height, cam, cMo, angleAppears, angleDisappears, changed);

  if (useScanLine) {
    //    if (clippingFlag <= 2) {
    //      cam.computeFov(width, height);
    //    }

    faces.computeClippedPolygons(cMo, cam);
    faces.computeScanLineRender(cam, width, height);
  }

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    vpMbtFaceDepthNormal *face_normal = *it;
    face_normal->computeVisibility();
  }
}

void vpMbDepthNormalTracker::computeVVS()
{
  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  computeVVSInit();
  unsigned int nb_features = (unsigned int)(3 * m_depthNormalListOfDesiredFeatures.size());

  vpColVector error_prev(nb_features);
  vpMatrix LTL;
  vpColVector LTR, v;

  double mu = m_initialMu;
  vpHomogeneousMatrix cMo_prev;

  bool isoJoIdentity_ = true;
  vpVelocityTwistMatrix cVo;
  vpMatrix L_true, LVJ_true;

  while (std::fabs(normRes_1 - normRes) > m_stopCriteriaEpsilon && (iter < m_maxIter)) {
    computeVVSInteractionMatrixAndResidu();

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_depthNormal, error_prev, cMo_prev, mu, reStartFromLastIncrement);

    if (!reStartFromLastIncrement) {
      if (m_depthNormalUseRobust)
        computeVVSWeights(m_robust_depthNormal, m_error_depthNormal, m_w_depthNormal);

      if (computeCovariance) {
        L_true = m_L_depthNormal;
        if (!isoJoIdentity_) {
          vpVelocityTwistMatrix cVo;
          cVo.buildFrom(cMo);
          LVJ_true = (m_L_depthNormal * (cVo * oJo));
        }
      }

      // Compute DoF only once
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
          unsigned int rank = (m_L_depthNormal * cVo).kernel(K);
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

      double num = 0.0, den = 0.0;
      for (unsigned int i = 0; i < m_L_depthNormal.getRows(); i++) {
        // Compute weighted errors and stop criteria
        m_weightedError_depthNormal[i] = m_w_depthNormal[i] * m_error_depthNormal[i];
        num += m_w_depthNormal[i] * vpMath::sqr(m_error_depthNormal[i]);
        den += m_w_depthNormal[i];

        // weight interaction matrix
        for (unsigned int j = 0; j < 6; j++) {
          m_L_depthNormal[i][j] *= m_w_depthNormal[i];
        }
      }

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L_depthNormal, LTL, m_weightedError_depthNormal,
                               m_error_depthNormal, error_prev, LTR, mu, v);

      cMo_prev = cMo;
      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      normRes_1 = normRes;
      normRes = sqrt(num / den);
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, m_w_depthNormal, cMo_prev, L_true, LVJ_true, m_error_depthNormal);
}

void vpMbDepthNormalTracker::computeVVSInit()
{
  unsigned int nb_features = (unsigned int)(3 * m_depthNormalListOfDesiredFeatures.size());

  m_L_depthNormal.resize(nb_features, 6, false, false);
  m_error_depthNormal.resize(nb_features, false);
  m_weightedError_depthNormal.resize(nb_features, false);

  m_w_depthNormal.resize(nb_features, false);
  m_w_depthNormal = 1;

  m_robust_depthNormal.resize(nb_features);
  m_robust_depthNormal.setThreshold(1e-3);
}

void vpMbDepthNormalTracker::computeVVSInteractionMatrixAndResidu()
{
  unsigned int cpt = 0;
  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalListOfActiveFaces.begin();
       it != m_depthNormalListOfActiveFaces.end(); ++it) {
    vpMatrix L_face;
    vpColVector features_face;
    (*it)->computeInteractionMatrix(cMo, L_face, features_face);

    vpColVector face_error = features_face - m_depthNormalListOfDesiredFeatures[(size_t)cpt];

    m_error_depthNormal.insert(cpt * 3, face_error);
    m_L_depthNormal.insert(L_face, cpt * 3, 0);

    cpt++;
  }
}

void vpMbDepthNormalTracker::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_,
                                     const vpCameraParameters &cam_, const vpColor &col, const unsigned int thickness,
                                     const bool displayFullModel)
{
  std::vector<std::vector<double> > models = vpMbDepthNormalTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, cam_, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    }
  }

  if (displayFeatures) {
    std::vector<std::vector<double> > features = getFeaturesForDisplayDepthNormal();
    for (size_t i = 0; i < features.size(); i++) {
      vpImagePoint im_centroid(features[i][1], features[i][2]);
      vpImagePoint im_extremity(features[i][3], features[i][4]);
      bool desired = vpMath::equal(features[i][0], 2);
      vpDisplay::displayArrow(I, im_centroid, im_extremity, desired ? vpColor::blue : vpColor::red, 4, 2, thickness);
    }
  }
}

void vpMbDepthNormalTracker::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                                     const vpCameraParameters &cam_, const vpColor &col, const unsigned int thickness,
                                     const bool displayFullModel)
{
  std::vector<std::vector<double> > models = vpMbDepthNormalTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, cam_, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    }
  }

  if (displayFeatures) {
    std::vector<std::vector<double> > features = getFeaturesForDisplayDepthNormal();
    for (size_t i = 0; i < features.size(); i++) {
      vpImagePoint im_centroid(features[i][1], features[i][2]);
      vpImagePoint im_extremity(features[i][3], features[i][4]);
      bool desired = vpMath::equal(features[i][0], 2);
      vpDisplay::displayArrow(I, im_centroid, im_extremity, desired ? vpColor::blue : vpColor::red, 4, 2, thickness);
    }
  }
}

std::vector<std::vector<double> > vpMbDepthNormalTracker::getFeaturesForDisplayDepthNormal() {
  std::vector<std::vector<double> > features;

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    vpMbtFaceDepthNormal *face_normal = *it;
    std::vector<std::vector<double> > currentFeatures = face_normal->getFeaturesForDisplay(cMo, cam);
    features.insert(features.end(), currentFeatures.begin(), currentFeatures.end());
  }

  return features;
}

/*!
  Return a list of primitives parameters to display the model at a given pose and camera parameters.
  - Line parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`.
  - Ellipse parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<mu20>`, `<mu11>`, `<mu02>`.

  \param width : Image width.
  \param height : Image height.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param cam_ : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbDepthNormalTracker::getModelForDisplay(unsigned int width, unsigned int height,
                                                                             const vpHomogeneousMatrix &cMo_,
                                                                             const vpCameraParameters &cam_,
                                                                             const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  vpCameraParameters c = cam_;

  bool changed = false;
  m_depthNormalHiddenFacesDisplay.setVisible(width, height, c, cMo_, angleAppears, angleDisappears, changed);

  if (useScanLine) {
    c.computeFov(width, height);

    m_depthNormalHiddenFacesDisplay.computeClippedPolygons(cMo_, c);
    m_depthNormalHiddenFacesDisplay.computeScanLineRender(c, width, height);
  }

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    vpMbtFaceDepthNormal *face_normal = *it;
    std::vector<std::vector<double> > modelLines = face_normal->getModelForDisplay(width, height, cMo_, cam_, displayFullModel);
    models.insert(models.end(), modelLines.begin(), modelLines.end());
  }

  return models;
}

void vpMbDepthNormalTracker::init(const vpImage<unsigned char> &I)
{
  if (!modelInitialised) {
    throw vpException(vpException::fatalError, "model not initialized");
  }

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

  if (useScanLine || clippingFlag > 3)
    cam.computeFov(I.getWidth(), I.getHeight());

  computeVisibility(I.getWidth(), I.getHeight());
}

void vpMbDepthNormalTracker::loadConfigFile(const std::string &configFile)
{
#ifdef VISP_HAVE_XML2
  vpMbtXmlGenericParser xmlp(vpMbtXmlGenericParser::DEPTH_NORMAL_PARSER);

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  xmlp.setDepthNormalFeatureEstimationMethod(m_depthNormalFeatureEstimationMethod);
  xmlp.setDepthNormalPclPlaneEstimationMethod(m_depthNormalPclPlaneEstimationMethod);
  xmlp.setDepthNormalPclPlaneEstimationRansacMaxIter(m_depthNormalPclPlaneEstimationRansacMaxIter);
  xmlp.setDepthNormalPclPlaneEstimationRansacThreshold(m_depthNormalPclPlaneEstimationRansacThreshold);
  xmlp.setDepthNormalSamplingStepX(m_depthNormalSamplingStepX);
  xmlp.setDepthNormalSamplingStepY(m_depthNormalSamplingStepY);

  try {
    std::cout << " *********** Parsing XML for Mb Depth Tracker ************ " << std::endl;
    xmlp.parse(configFile);
  } catch (const vpException &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    throw vpException(vpException::ioError, "Cannot open XML file \"%s\"", configFile.c_str());
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

  if (xmlp.getFovClipping())
    setClipping(clippingFlag | vpPolygon3D::FOV_CLIPPING);

  setDepthNormalFeatureEstimationMethod(xmlp.getDepthNormalFeatureEstimationMethod());
  setDepthNormalPclPlaneEstimationMethod(xmlp.getDepthNormalPclPlaneEstimationMethod());
  setDepthNormalPclPlaneEstimationRansacMaxIter(xmlp.getDepthNormalPclPlaneEstimationRansacMaxIter());
  setDepthNormalPclPlaneEstimationRansacThreshold(xmlp.getDepthNormalPclPlaneEstimationRansacThreshold());
  setDepthNormalSamplingStep(xmlp.getDepthNormalSamplingStepX(), xmlp.getDepthNormalSamplingStepY());
#else
  std::cerr << "You need the libXML2 to read the config file " << configFile << std::endl;
#endif
}

void vpMbDepthNormalTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                                         const vpHomogeneousMatrix &cMo_, const bool verbose)
{
  cMo.eye();

  for (size_t i = 0; i < m_depthNormalFaces.size(); i++) {
    delete m_depthNormalFaces[i];
    m_depthNormalFaces[i] = NULL;
  }

  m_depthNormalFaces.clear();

  loadModel(cad_name, verbose);
  initFromPose(I, cMo_);
}

#if defined(VISP_HAVE_PCL)
void vpMbDepthNormalTracker::reInitModel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                                         const std::string &cad_name, const vpHomogeneousMatrix &cMo_,
                                         const bool verbose)
{
  vpImage<unsigned char> I_dummy(point_cloud->height, point_cloud->width);
  reInitModel(I_dummy, cad_name, cMo_, verbose);
}

#endif

void vpMbDepthNormalTracker::resetTracker()
{
  cMo.eye();

  for (std::vector<vpMbtFaceDepthNormal *>::iterator it = m_depthNormalFaces.begin(); it != m_depthNormalFaces.end();
       ++it) {
    vpMbtFaceDepthNormal *normal_face = *it;
    delete normal_face;
    normal_face = NULL;
  }

  m_depthNormalFaces.clear();

  m_computeInteraction = true;
  computeCovariance = false;

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);

  clippingFlag = vpPolygon3D::NO_CLIPPING;

  m_lambda = 1.0;

  faces.reset();

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif

  m_depthNormalListOfActiveFaces.clear();
  m_depthNormalListOfDesiredFeatures.clear();
}

void vpMbDepthNormalTracker::setOgreVisibilityTest(const bool &v)
{
  vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Depth");
#endif
}

void vpMbDepthNormalTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo)
{
  cMo = cdMo;
  init(I);
}

void vpMbDepthNormalTracker::setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo)
{
  cMo = cdMo;
  vpImageConvert::convert(I_color, m_I);
  init(m_I);
}

#if defined(VISP_HAVE_PCL)
void vpMbDepthNormalTracker::setPose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                                     const vpHomogeneousMatrix &cdMo)
{
  vpImage<unsigned char> I_dummy(point_cloud->height, point_cloud->width);
  cMo = cdMo;
  init(I_dummy);
}
#endif

void vpMbDepthNormalTracker::setScanLineVisibilityTest(const bool &v)
{
  vpMbTracker::setScanLineVisibilityTest(v);

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setScanLineVisibilityTest(v);
  }
}

void vpMbDepthNormalTracker::setUseDepthNormalTracking(const std::string &name, const bool &useDepthNormalTracking)
{
  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    vpMbtFaceDepthNormal *face = *it;
    if (face->m_polygon->getName() == name) {
      face->setTracked(useDepthNormalTracking);
    }
  }
}

void vpMbDepthNormalTracker::testTracking() {}

#ifdef VISP_HAVE_PCL
void vpMbDepthNormalTracker::segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
  m_depthNormalListOfActiveFaces.clear();
  m_depthNormalListOfDesiredFeatures.clear();

#if DEBUG_DISPLAY_DEPTH_NORMAL
  if (!m_debugDisp_depthNormal->isInitialised()) {
    m_debugImage_depthNormal.resize(point_cloud->height, point_cloud->width);
    m_debugDisp_depthNormal->init(m_debugImage_depthNormal, 50, 0, "Debug display normal depth tracker");
  }

  m_debugImage_depthNormal = 0;
  std::vector<std::vector<vpImagePoint> > roiPts_vec;
#endif

  for (std::vector<vpMbtFaceDepthNormal *>::iterator it = m_depthNormalFaces.begin(); it != m_depthNormalFaces.end();
       ++it) {
    vpMbtFaceDepthNormal *face = *it;

    if (face->isVisible() && face->isTracked()) {
      vpColVector desired_features;

#if DEBUG_DISPLAY_DEPTH_NORMAL
      std::vector<std::vector<vpImagePoint> > roiPts_vec_;
#endif
      if (face->computeDesiredFeatures(cMo, point_cloud->width, point_cloud->height, point_cloud, desired_features,
                                       m_depthNormalSamplingStepX, m_depthNormalSamplingStepY
#if DEBUG_DISPLAY_DEPTH_NORMAL
                                       ,
                                       m_debugImage_depthNormal, roiPts_vec_
#endif
                                       , m_mask
                                       )) {
        m_depthNormalListOfDesiredFeatures.push_back(desired_features);
        m_depthNormalListOfActiveFaces.push_back(face);

#if DEBUG_DISPLAY_DEPTH_NORMAL
        roiPts_vec.insert(roiPts_vec.end(), roiPts_vec_.begin(), roiPts_vec_.end());
#endif
      }
    }
  }

#if DEBUG_DISPLAY_DEPTH_NORMAL
  vpDisplay::display(m_debugImage_depthNormal);

  for (size_t i = 0; i < roiPts_vec.size(); i++) {
    if (roiPts_vec[i].empty())
      continue;

    for (size_t j = 0; j < roiPts_vec[i].size() - 1; j++) {
      vpDisplay::displayLine(m_debugImage_depthNormal, roiPts_vec[i][j], roiPts_vec[i][j + 1], vpColor::red, 2);
    }
    vpDisplay::displayLine(m_debugImage_depthNormal, roiPts_vec[i][0], roiPts_vec[i][roiPts_vec[i].size() - 1],
                           vpColor::red, 2);
  }

  vpDisplay::flush(m_debugImage_depthNormal);
#endif
}
#endif

void vpMbDepthNormalTracker::segmentPointCloud(const std::vector<vpColVector> &point_cloud, const unsigned int width,
                                               const unsigned int height)
{
  m_depthNormalListOfActiveFaces.clear();
  m_depthNormalListOfDesiredFeatures.clear();

#if DEBUG_DISPLAY_DEPTH_NORMAL
  if (!m_debugDisp_depthNormal->isInitialised()) {
    m_debugImage_depthNormal.resize(height, width);
    m_debugDisp_depthNormal->init(m_debugImage_depthNormal, 50, 0, "Debug display normal depth tracker");
  }

  m_debugImage_depthNormal = 0;
  std::vector<std::vector<vpImagePoint> > roiPts_vec;
#endif

  for (std::vector<vpMbtFaceDepthNormal *>::iterator it = m_depthNormalFaces.begin(); it != m_depthNormalFaces.end();
       ++it) {
    vpMbtFaceDepthNormal *face = *it;

    if (face->isVisible() && face->isTracked()) {
      vpColVector desired_features;

#if DEBUG_DISPLAY_DEPTH_NORMAL
      std::vector<std::vector<vpImagePoint> > roiPts_vec_;
#endif

      if (face->computeDesiredFeatures(cMo, width, height, point_cloud, desired_features, m_depthNormalSamplingStepX,
                                       m_depthNormalSamplingStepY
#if DEBUG_DISPLAY_DEPTH_NORMAL
                                       ,
                                       m_debugImage_depthNormal, roiPts_vec_
#endif
                                       , m_mask
                                       )) {
        m_depthNormalListOfDesiredFeatures.push_back(desired_features);
        m_depthNormalListOfActiveFaces.push_back(face);

#if DEBUG_DISPLAY_DEPTH_NORMAL
        roiPts_vec.insert(roiPts_vec.end(), roiPts_vec_.begin(), roiPts_vec_.end());
#endif
      }
    }
  }

#if DEBUG_DISPLAY_DEPTH_NORMAL
  vpDisplay::display(m_debugImage_depthNormal);

  for (size_t i = 0; i < roiPts_vec.size(); i++) {
    if (roiPts_vec[i].empty())
      continue;

    for (size_t j = 0; j < roiPts_vec[i].size() - 1; j++) {
      vpDisplay::displayLine(m_debugImage_depthNormal, roiPts_vec[i][j], roiPts_vec[i][j + 1], vpColor::red, 2);
    }
    vpDisplay::displayLine(m_debugImage_depthNormal, roiPts_vec[i][0], roiPts_vec[i][roiPts_vec[i].size() - 1],
                           vpColor::red, 2);
  }

  vpDisplay::flush(m_debugImage_depthNormal);
#endif
}

void vpMbDepthNormalTracker::setCameraParameters(const vpCameraParameters &camera)
{
  this->cam = camera;

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setCameraParameters(camera);
  }
}

void vpMbDepthNormalTracker::setDepthNormalFaceCentroidMethod(const vpMbtFaceDepthNormal::vpFaceCentroidType &method)
{
  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setFaceCentroidMethod(method);
  }
}

void vpMbDepthNormalTracker::setDepthNormalFeatureEstimationMethod(
    const vpMbtFaceDepthNormal::vpFeatureEstimationType &method)
{
  m_depthNormalFeatureEstimationMethod = method;

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setFeatureEstimationMethod(method);
  }
}

void vpMbDepthNormalTracker::setDepthNormalPclPlaneEstimationMethod(const int method)
{
  m_depthNormalPclPlaneEstimationMethod = method;

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setPclPlaneEstimationMethod(method);
  }
}

void vpMbDepthNormalTracker::setDepthNormalPclPlaneEstimationRansacMaxIter(const int maxIter)
{
  m_depthNormalPclPlaneEstimationRansacMaxIter = maxIter;

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setPclPlaneEstimationRansacMaxIter(maxIter);
  }
}

void vpMbDepthNormalTracker::setDepthNormalPclPlaneEstimationRansacThreshold(const double thresold)
{
  m_depthNormalPclPlaneEstimationRansacThreshold = thresold;

  for (std::vector<vpMbtFaceDepthNormal *>::const_iterator it = m_depthNormalFaces.begin();
       it != m_depthNormalFaces.end(); ++it) {
    (*it)->setPclPlaneEstimationRansacThreshold(thresold);
  }
}

void vpMbDepthNormalTracker::setDepthNormalSamplingStep(const unsigned int stepX, const unsigned int stepY)
{
  if (stepX == 0 || stepY == 0) {
    std::cerr << "stepX and stepY must be greater than zero!" << std::endl;
    return;
  }

  m_depthNormalSamplingStepX = stepX;
  m_depthNormalSamplingStepY = stepY;
}

// void vpMbDepthNormalTracker::setDepthNormalUseRobust(const bool use) {
//  m_depthNormalUseRobust = use;
//}

void vpMbDepthNormalTracker::track(const vpImage<unsigned char> &)
{
  throw vpException(vpException::fatalError, "Cannot track with a grayscale image!");
}

void vpMbDepthNormalTracker::track(const vpImage<vpRGBa> &)
{
  throw vpException(vpException::fatalError, "Cannot track with a color image!");
}

#ifdef VISP_HAVE_PCL
void vpMbDepthNormalTracker::track(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
  segmentPointCloud(point_cloud);

  computeVVS();

  computeVisibility(point_cloud->width, point_cloud->height);
}
#endif

void vpMbDepthNormalTracker::track(const std::vector<vpColVector> &point_cloud, const unsigned int width,
                                   const unsigned int height)
{
  segmentPointCloud(point_cloud, width, height);

  computeVVS();

  computeVisibility(width, height);
}

void vpMbDepthNormalTracker::initCircle(const vpPoint & /*p1*/, const vpPoint & /*p2*/, const vpPoint & /*p3*/,
                                        const double /*radius*/, const int /*idFace*/, const std::string & /*name*/)
{
  throw vpException(vpException::fatalError, "vpMbDepthNormalTracker::initCircle() should not be called!");
}

void vpMbDepthNormalTracker::initCylinder(const vpPoint & /*p1*/, const vpPoint & /*p2*/, const double /*radius*/,
                                          const int /*idFace*/, const std::string & /*name*/)
{
  throw vpException(vpException::fatalError, "vpMbDepthNormalTracker::initCylinder() should not be called!");
}

void vpMbDepthNormalTracker::initFaceFromCorners(vpMbtPolygon &polygon) { addFace(polygon, false); }

void vpMbDepthNormalTracker::initFaceFromLines(vpMbtPolygon &polygon) { addFace(polygon, true); }
