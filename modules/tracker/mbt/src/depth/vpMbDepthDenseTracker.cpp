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
 * Model-based tracker using depth dense features.
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
#include <visp3/mbt/vpMbDepthDenseTracker.h>
#include <visp3/mbt/vpMbtXmlGenericParser.h>

#if DEBUG_DISPLAY_DEPTH_DENSE
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#endif

vpMbDepthDenseTracker::vpMbDepthDenseTracker()
  : m_depthDenseHiddenFacesDisplay(), m_depthDenseListOfActiveFaces(),
    m_denseDepthNbFeatures(0), m_depthDenseFaces(), m_depthDenseSamplingStepX(2), m_depthDenseSamplingStepY(2),
    m_error_depthDense(), m_L_depthDense(), m_robust_depthDense(), m_w_depthDense(), m_weightedError_depthDense()
#if DEBUG_DISPLAY_DEPTH_DENSE
    ,
    m_debugDisp_depthDense(NULL), m_debugImage_depthDense()
#endif
{
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Depth Dense");
#endif

#if defined(VISP_HAVE_X11) && DEBUG_DISPLAY_DEPTH_DENSE
  m_debugDisp_depthDense = new vpDisplayX;
#elif defined(VISP_HAVE_GDI) && DEBUG_DISPLAY_DEPTH_DENSE
  m_debugDisp_depthDense = new vpDisplayGDI;
#endif
}

vpMbDepthDenseTracker::~vpMbDepthDenseTracker()
{
  for (size_t i = 0; i < m_depthDenseFaces.size(); i++) {
    delete m_depthDenseFaces[i];
  }

#if DEBUG_DISPLAY_DEPTH_DENSE
  delete m_debugDisp_depthDense;
#endif
}

void vpMbDepthDenseTracker::addFace(vpMbtPolygon &polygon, const bool alreadyClose)
{
  if (polygon.nbpt < 3) {
    return;
  }

  // Copy hidden faces
  m_depthDenseHiddenFacesDisplay = faces;

  vpMbtFaceDepthDense *normal_face = new vpMbtFaceDepthDense;
  normal_face->m_hiddenFace = &faces;
  normal_face->m_polygon = &polygon;
  normal_face->m_cam = cam;
  normal_face->m_useScanLine = useScanLine;
  normal_face->m_clippingFlag = clippingFlag;
  normal_face->m_distNearClip = distNearClip;
  normal_face->m_distFarClip = distFarClip;

  // Add lines that compose the face
  unsigned int nbpt = polygon.getNbPoint();
  if (nbpt > 0) {
    for (unsigned int i = 0; i < nbpt - 1; i++) {
      normal_face->addLine(polygon.p[i], polygon.p[i + 1], &m_depthDenseHiddenFacesDisplay, polygon.getIndex(),
                           polygon.getName());
    }

    if (!alreadyClose) {
      // Add last line that closes the face
      normal_face->addLine(polygon.p[nbpt - 1], polygon.p[0], &m_depthDenseHiddenFacesDisplay, polygon.getIndex(),
                           polygon.getName());
    }
  }

  // Construct a vpPlane in object frame
  vpPoint pts[3];
  pts[0] = polygon.p[0];
  pts[1] = polygon.p[1];
  pts[2] = polygon.p[2];
  normal_face->m_planeObject = vpPlane(pts[0], pts[1], pts[2], vpPlane::object_frame);

  m_depthDenseFaces.push_back(normal_face);
}

void vpMbDepthDenseTracker::computeVisibility(const unsigned int width, const unsigned int height)
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

  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    vpMbtFaceDepthDense *face_normal = *it;
    face_normal->computeVisibility();
  }
}

void vpMbDepthDenseTracker::computeVVS()
{
  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  computeVVSInit();

  vpColVector error_prev(m_denseDepthNbFeatures);
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
    computeVVSCheckLevenbergMarquardt(iter, m_error_depthDense, error_prev, cMo_prev, mu, reStartFromLastIncrement);

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      if (computeCovariance) {
        L_true = m_L_depthDense;
        if (!isoJoIdentity_) {
          vpVelocityTwistMatrix cVo;
          cVo.buildFrom(cMo);
          LVJ_true = (m_L_depthDense * cVo * oJo);
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
          unsigned int rank = (m_L_depthDense * cVo).kernel(K);
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
      for (unsigned int i = 0; i < m_L_depthDense.getRows(); i++) {
        // Compute weighted errors and stop criteria
        m_weightedError_depthDense[i] = m_w_depthDense[i] * m_error_depthDense[i];
        num += m_w_depthDense[i] * vpMath::sqr(m_error_depthDense[i]);
        den += m_w_depthDense[i];

        // weight interaction matrix
        for (unsigned int j = 0; j < 6; j++) {
          m_L_depthDense[i][j] *= m_w_depthDense[i];
        }
      }

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L_depthDense, LTL, m_weightedError_depthDense,
                               m_error_depthDense, error_prev, LTR, mu, v);

      cMo_prev = cMo;
      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      normRes_1 = normRes;
      normRes = sqrt(num / den);
    }

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, m_w_depthDense, cMo_prev, L_true, LVJ_true, m_error_depthDense);
}

void vpMbDepthDenseTracker::computeVVSInit()
{
  m_denseDepthNbFeatures = 0;

  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseListOfActiveFaces.begin();
       it != m_depthDenseListOfActiveFaces.end(); ++it) {
    vpMbtFaceDepthDense *face = *it;
    m_denseDepthNbFeatures += face->getNbFeatures();
  }

  m_L_depthDense.resize(m_denseDepthNbFeatures, 6, false, false);
  m_error_depthDense.resize(m_denseDepthNbFeatures, false);
  m_weightedError_depthDense.resize(m_denseDepthNbFeatures, false);

  m_w_depthDense.resize(m_denseDepthNbFeatures, false);
  m_w_depthDense = 1;
}

void vpMbDepthDenseTracker::computeVVSInteractionMatrixAndResidu()
{
  unsigned int start_index = 0;
  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseListOfActiveFaces.begin();
       it != m_depthDenseListOfActiveFaces.end(); ++it) {
    vpMbtFaceDepthDense *face = *it;

    vpMatrix L_face;
    vpColVector error;

    face->computeInteractionMatrixAndResidu(cMo, L_face, error);

    m_error_depthDense.insert(start_index, error);
    m_L_depthDense.insert(L_face, start_index, 0);

    start_index += error.getRows();
  }
}

void vpMbDepthDenseTracker::computeVVSWeights()
{
  m_robust_depthDense.MEstimator(m_error_depthDense, m_w_depthDense, 1e-3);
}

void vpMbDepthDenseTracker::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_,
                                    const vpCameraParameters &cam_, const vpColor &col, const unsigned int thickness,
                                    const bool displayFullModel)
{
  std::vector<std::vector<double> > models = vpMbDepthDenseTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, cam_, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    }
  }
}

void vpMbDepthDenseTracker::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                                    const vpCameraParameters &cam_, const vpColor &col, const unsigned int thickness,
                                    const bool displayFullModel)
{
  std::vector<std::vector<double> > models = vpMbDepthDenseTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, cam_, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    }
  }
}

/*!
  Return a list of primitives parameters to display the model at a given pose and camera parameters.
  - Line parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`
  - Ellipse parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<mu20>`, `<mu11>`, `<mu02>`

  \param width : Image width.
  \param height : Image height.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param cam_ : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbDepthDenseTracker::getModelForDisplay(unsigned int width, unsigned int height,
                                                                            const vpHomogeneousMatrix &cMo_,
                                                                            const vpCameraParameters &cam_,
                                                                            const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  vpCameraParameters c = cam_;

  bool changed = false;
  m_depthDenseHiddenFacesDisplay.setVisible(width, height, c, cMo_, angleAppears, angleDisappears, changed);

  if (useScanLine) {
    c.computeFov(width, height);

    m_depthDenseHiddenFacesDisplay.computeClippedPolygons(cMo_, c);
    m_depthDenseHiddenFacesDisplay.computeScanLineRender(c, width, height);
  }

  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    vpMbtFaceDepthDense *face_dense = *it;
    std::vector<std::vector<double> > modelLines = face_dense->getModelForDisplay(width, height, cMo_, cam_, displayFullModel);
    models.insert(models.end(), modelLines.begin(), modelLines.end());
  }

  return models;
}

void vpMbDepthDenseTracker::init(const vpImage<unsigned char> &I)
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

void vpMbDepthDenseTracker::loadConfigFile(const std::string &configFile)
{
#ifdef VISP_HAVE_XML2
  vpMbtXmlGenericParser xmlp(vpMbtXmlGenericParser::DEPTH_DENSE_PARSER);

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  xmlp.setDepthDenseSamplingStepX(m_depthDenseSamplingStepX);
  xmlp.setDepthDenseSamplingStepY(m_depthDenseSamplingStepY);

  try {
    std::cout << " *********** Parsing XML for Mb Depth Dense Tracker ************ " << std::endl;
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

  setDepthDenseSamplingStep(xmlp.getDepthDenseSamplingStepX(), xmlp.getDepthDenseSamplingStepY());
#else
  std::cerr << "You need the libXML2 to read the config file " << configFile << std::endl;
#endif
}

void vpMbDepthDenseTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                                        const vpHomogeneousMatrix &cMo_, const bool verbose)
{
  cMo.eye();

  for (size_t i = 0; i < m_depthDenseFaces.size(); i++) {
    delete m_depthDenseFaces[i];
    m_depthDenseFaces[i] = NULL;
  }

  m_depthDenseFaces.clear();

  loadModel(cad_name, verbose);
  initFromPose(I, cMo_);
}

#if defined(VISP_HAVE_PCL)
void vpMbDepthDenseTracker::reInitModel(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                                        const std::string &cad_name, const vpHomogeneousMatrix &cMo_,
                                        const bool verbose)
{
  vpImage<unsigned char> I_dummy(point_cloud->height, point_cloud->width);
  reInitModel(I_dummy, cad_name, cMo_, verbose);
}

#endif

void vpMbDepthDenseTracker::resetTracker()
{
  cMo.eye();

  for (std::vector<vpMbtFaceDepthDense *>::iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    vpMbtFaceDepthDense *normal_face = *it;
    delete normal_face;
    normal_face = NULL;
  }

  m_depthDenseFaces.clear();

  m_computeInteraction = true;
  computeCovariance = false;

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);

  clippingFlag = vpPolygon3D::NO_CLIPPING;

  m_lambda = 1.0;
  m_maxIter = 30;

  faces.reset();

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif

  m_depthDenseListOfActiveFaces.clear();
}

void vpMbDepthDenseTracker::setCameraParameters(const vpCameraParameters &camera)
{
  this->cam = camera;

  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    (*it)->setCameraParameters(camera);
  }
}

void vpMbDepthDenseTracker::setDepthDenseFilteringMaxDistance(const double maxDistance)
{
  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    (*it)->setDepthDenseFilteringMaxDistance(maxDistance);
  }
}

void vpMbDepthDenseTracker::setDepthDenseFilteringMethod(const int method)
{
  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    (*it)->setDepthDenseFilteringMethod(method);
  }
}

void vpMbDepthDenseTracker::setDepthDenseFilteringMinDistance(const double minDistance)
{
  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    (*it)->setDepthDenseFilteringMinDistance(minDistance);
  }
}

void vpMbDepthDenseTracker::setDepthDenseFilteringOccupancyRatio(const double occupancyRatio)
{
  if (occupancyRatio < 0.0 || occupancyRatio > 1.0) {
    std::cerr << "occupancyRatio < 0.0 || occupancyRatio > 1.0" << std::endl;
    return;
  }

  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    (*it)->setDepthDenseFilteringOccupancyRatio(occupancyRatio);
  }
}

#ifdef VISP_HAVE_PCL
void vpMbDepthDenseTracker::segmentPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
  m_depthDenseListOfActiveFaces.clear();

#if DEBUG_DISPLAY_DEPTH_DENSE
  if (!m_debugDisp_depthDense->isInitialised()) {
    m_debugImage_depthDense.resize(point_cloud->height, point_cloud->width);
    m_debugDisp_depthDense->init(m_debugImage_depthDense, 50, 0, "Debug display dense depth tracker");
  }

  m_debugImage_depthDense = 0;
  std::vector<std::vector<vpImagePoint> > roiPts_vec;
#endif

  for (std::vector<vpMbtFaceDepthDense *>::iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    vpMbtFaceDepthDense *face = *it;

    if (face->isVisible() && face->isTracked()) {
#if DEBUG_DISPLAY_DEPTH_DENSE
      std::vector<std::vector<vpImagePoint> > roiPts_vec_;
#endif
      if (face->computeDesiredFeatures(cMo, point_cloud, m_depthDenseSamplingStepX, m_depthDenseSamplingStepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                                       ,
                                       m_debugImage_depthDense, roiPts_vec_
#endif
                                       , m_mask
                                       )) {
        m_depthDenseListOfActiveFaces.push_back(*it);

#if DEBUG_DISPLAY_DEPTH_DENSE
        roiPts_vec.insert(roiPts_vec.end(), roiPts_vec_.begin(), roiPts_vec_.end());
#endif
      }
    }
  }

#if DEBUG_DISPLAY_DEPTH_DENSE
  vpDisplay::display(m_debugImage_depthDense);

  for (size_t i = 0; i < roiPts_vec.size(); i++) {
    if (roiPts_vec[i].empty())
      continue;

    for (size_t j = 0; j < roiPts_vec[i].size() - 1; j++) {
      vpDisplay::displayLine(m_debugImage_depthDense, roiPts_vec[i][j], roiPts_vec[i][j + 1], vpColor::red, 2);
    }
    vpDisplay::displayLine(m_debugImage_depthDense, roiPts_vec[i][0], roiPts_vec[i][roiPts_vec[i].size() - 1],
                           vpColor::red, 2);
  }

  vpDisplay::flush(m_debugImage_depthDense);
#endif
}
#endif

void vpMbDepthDenseTracker::segmentPointCloud(const std::vector<vpColVector> &point_cloud, const unsigned int width,
                                              const unsigned int height)
{
  m_depthDenseListOfActiveFaces.clear();

#if DEBUG_DISPLAY_DEPTH_DENSE
  if (!m_debugDisp_depthDense->isInitialised()) {
    m_debugImage_depthDense.resize(height, width);
    m_debugDisp_depthDense->init(m_debugImage_depthDense, 50, 0, "Debug display dense depth tracker");
  }

  m_debugImage_depthDense = 0;
  std::vector<std::vector<vpImagePoint> > roiPts_vec;
#endif

  for (std::vector<vpMbtFaceDepthDense *>::iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    vpMbtFaceDepthDense *face = *it;

    if (face->isVisible() && face->isTracked()) {
#if DEBUG_DISPLAY_DEPTH_DENSE
      std::vector<std::vector<vpImagePoint> > roiPts_vec_;
#endif
      if (face->computeDesiredFeatures(cMo, width, height, point_cloud, m_depthDenseSamplingStepX,
                                       m_depthDenseSamplingStepY
#if DEBUG_DISPLAY_DEPTH_DENSE
                                       ,
                                       m_debugImage_depthDense, roiPts_vec_
#endif
                                       , m_mask
                                       )) {
        m_depthDenseListOfActiveFaces.push_back(*it);

#if DEBUG_DISPLAY_DEPTH_DENSE
        roiPts_vec.insert(roiPts_vec.end(), roiPts_vec_.begin(), roiPts_vec_.end());
#endif
      }
    }
  }

#if DEBUG_DISPLAY_DEPTH_DENSE
  vpDisplay::display(m_debugImage_depthDense);

  for (size_t i = 0; i < roiPts_vec.size(); i++) {
    if (roiPts_vec[i].empty())
      continue;

    for (size_t j = 0; j < roiPts_vec[i].size() - 1; j++) {
      vpDisplay::displayLine(m_debugImage_depthDense, roiPts_vec[i][j], roiPts_vec[i][j + 1], vpColor::red, 2);
    }
    vpDisplay::displayLine(m_debugImage_depthDense, roiPts_vec[i][0], roiPts_vec[i][roiPts_vec[i].size() - 1],
                           vpColor::red, 2);
  }

  vpDisplay::flush(m_debugImage_depthDense);
#endif
}

void vpMbDepthDenseTracker::setOgreVisibilityTest(const bool &v)
{
  vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Depth Dense");
#endif
}

void vpMbDepthDenseTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo)
{
  cMo = cdMo;
  init(I);
}

void vpMbDepthDenseTracker::setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo)
{
  cMo = cdMo;
  vpImageConvert::convert(I_color, m_I);
  init(m_I);
}

#if defined(VISP_HAVE_PCL)
void vpMbDepthDenseTracker::setPose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud,
                                    const vpHomogeneousMatrix &cdMo)
{
  m_I.resize(point_cloud->height, point_cloud->width);
  cMo = cdMo;
  init(m_I);
}
#endif

void vpMbDepthDenseTracker::setScanLineVisibilityTest(const bool &v)
{
  vpMbTracker::setScanLineVisibilityTest(v);

  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    (*it)->setScanLineVisibilityTest(v);
  }
}

void vpMbDepthDenseTracker::setUseDepthDenseTracking(const std::string &name, const bool &useDepthDenseTracking)
{
  for (std::vector<vpMbtFaceDepthDense *>::const_iterator it = m_depthDenseFaces.begin();
       it != m_depthDenseFaces.end(); ++it) {
    vpMbtFaceDepthDense *face = *it;
    if (face->m_polygon->getName() == name) {
      face->setTracked(useDepthDenseTracking);
    }
  }
}

void vpMbDepthDenseTracker::testTracking() {}

void vpMbDepthDenseTracker::track(const vpImage<unsigned char> &)
{
  throw vpException(vpException::fatalError, "Cannot track with a grayscale image!");
}

void vpMbDepthDenseTracker::track(const vpImage<vpRGBa> &)
{
  throw vpException(vpException::fatalError, "Cannot track with a color image!");
}

#ifdef VISP_HAVE_PCL
void vpMbDepthDenseTracker::track(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud)
{
  segmentPointCloud(point_cloud);

  computeVVS();

  computeVisibility(point_cloud->width, point_cloud->height);
}
#endif

void vpMbDepthDenseTracker::track(const std::vector<vpColVector> &point_cloud, const unsigned int width,
                                  const unsigned int height)
{
  segmentPointCloud(point_cloud, width, height);

  computeVVS();

  computeVisibility(width, height);
}

void vpMbDepthDenseTracker::initCircle(const vpPoint & /*p1*/, const vpPoint & /*p2*/, const vpPoint & /*p3*/,
                                       const double /*radius*/, const int /*idFace*/, const std::string & /*name*/)
{
  throw vpException(vpException::fatalError, "vpMbDepthDenseTracker::initCircle() should not be called!");
}

void vpMbDepthDenseTracker::initCylinder(const vpPoint & /*p1*/, const vpPoint & /*p2*/, const double /*radius*/,
                                         const int /*idFace*/, const std::string & /*name*/)
{
  throw vpException(vpException::fatalError, "vpMbDepthDenseTracker::initCylinder() should not be called!");
}

void vpMbDepthDenseTracker::initFaceFromCorners(vpMbtPolygon &polygon) { addFace(polygon, false); }

void vpMbDepthDenseTracker::initFaceFromLines(vpMbtPolygon &polygon) { addFace(polygon, true); }
