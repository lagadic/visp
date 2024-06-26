/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Model based tracker using only KLT
 *
*****************************************************************************/

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/mbt/vpMbKltTracker.h>
#include <visp3/mbt/vpMbtXmlGenericParser.h>

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)

#if defined(__APPLE__) && defined(__MACH__) // Apple OSX and iOS (Darwin)
#include <TargetConditionals.h>             // To detect OSX or IOS using TARGET_OS_IPHONE or TARGET_OS_IOS macro
#endif

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace
{
/*!
 * Transform an homography from calibrated domain to pixel space.
 *
 * Given homography \f$\bf H\f$ in the Euclidean space or in the calibrated domain,
 * compute the homography \f$\bf G\f$ corresponding to the collineation matrix in the pixel space using:
 * \f[ {\bf G} = {\bf K} {\bf H} {\bf K}^{-1} \f]
 * \param[in] H : Homography in the calibrated domain.
 * \param[in] cam : Camera parameters used to fill \f${\bf K}\f$ matrix such as
 * \f[{\bf K} =
 * \left[ \begin{array}{ccc}
 * p_x & 0   & u_0  \\
 * 0   & p_y & v_0 \\
 * 0   & 0   & 1
 * \end{array}\right]
 * \f]
 * \return The corresponding collineation matrix \f$\bf G\f$ in the pixel space.
 *
 * \sa vpHomography::homography2collineation()
 */
vpMatrix homography2collineation(const vpMatrix &H, const vpCameraParameters &cam)
{
  vpMatrix G(3, 3);
  double px = cam.get_px();
  double py = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double one_over_px = cam.get_px_inverse();
  double one_over_py = cam.get_py_inverse();
  double h00 = H[0][0], h01 = H[0][1], h02 = H[0][2];
  double h10 = H[1][0], h11 = H[1][1], h12 = H[1][2];
  double h20 = H[2][0], h21 = H[2][1], h22 = H[2][2];

  double A = h00 * px + u0 * h20;
  double B = h01 * px + u0 * h21;
  double C = h02 * px + u0 * h22;
  double D = h10 * py + v0 * h20;
  double E = h11 * py + v0 * h21;
  double F = h12 * py + v0 * h22;

  G[0][0] = A * one_over_px;
  G[1][0] = D * one_over_px;
  G[2][0] = h20 * one_over_px;

  G[0][1] = B * one_over_py;
  G[1][1] = E * one_over_py;
  G[2][1] = h21 * one_over_py;

  double u0_one_over_px = u0 * one_over_px;
  double v0_one_over_py = v0 * one_over_py;

  G[0][2] = -A * u0_one_over_px - B * v0_one_over_py + C;
  G[1][2] = -D * u0_one_over_px - E * v0_one_over_py + F;
  G[2][2] = -h20 * u0_one_over_px - h21 * v0_one_over_py + h22;

  return G;
}
} // namespace

#endif // DOXYGEN_SHOULD_SKIP_THIS

vpMbKltTracker::vpMbKltTracker()
  :
  cur(), c0Mo(), firstInitialisation(true), maskBorder(5), threshold_outlier(0.5), percentGood(0.6), ctTc0(), tracker(),
  kltPolygons(), kltCylinders(), circles_disp(), m_nbInfos(0), m_nbFaceUsed(0), m_L_klt(), m_error_klt(), m_w_klt(),
  m_weightedError_klt(), m_robust_klt(), m_featuresToBeDisplayedKlt()
{
  tracker.setTrackerId(1);
  tracker.setUseHarris(1);
  tracker.setMaxFeatures(10000);
  tracker.setWindowSize(5);
  tracker.setQuality(0.01);
  tracker.setMinDistance(5);
  tracker.setHarrisFreeParameter(0.01);
  tracker.setBlockSize(3);
  tracker.setPyramidLevels(3);

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Klt");
#endif

  m_lambda = 0.8;
  m_maxIter = 200;
}

/*!
  Basic destructor.

*/
vpMbKltTracker::~vpMbKltTracker()
{
  // delete the Klt Polygon features
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly != nullptr) {
      delete kltpoly;
    }
    kltpoly = nullptr;
  }
  kltPolygons.clear();

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    if (kltPolyCylinder != nullptr) {
      delete kltPolyCylinder;
    }
    kltPolyCylinder = nullptr;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    if (ci != nullptr) {
      delete ci;
    }
    ci = nullptr;
  }

  circles_disp.clear();
}

void vpMbKltTracker::init(const vpImage<unsigned char> &I)
{
  if (!modelInitialised) {
    throw vpException(vpException::fatalError, "model not initialized");
  }

  bool reInitialisation = false;
  if (!useOgre)
    faces.setVisible(I.getWidth(), I.getHeight(), m_cam, m_cMo, angleAppears, angleDisappears, reInitialisation);
  else {
#ifdef VISP_HAVE_OGRE
    if (!faces.isOgreInitialised()) {
      faces.setBackgroundSizeOgre(I.getHeight(), I.getWidth());
      faces.setOgreShowConfigDialog(ogreShowConfigDialog);
      faces.initOgre(m_cam);
      // Turn off Ogre config dialog display for the next call to this
      // function since settings are saved in the ogre.cfg file and used
      // during the next call
      ogreShowConfigDialog = false;
    }

    faces.setVisibleOgre(I.getWidth(), I.getHeight(), m_cam, m_cMo, angleAppears, angleDisappears, reInitialisation);

#else
    faces.setVisible(I.getWidth(), I.getHeight(), m_cam, m_cMo, angleAppears, angleDisappears, reInitialisation);
#endif
  }
  reinit(I);
}

void vpMbKltTracker::reinit(const vpImage<unsigned char> &I)
{
  c0Mo = m_cMo;
  ctTc0.eye();

  vpImageConvert::convert(I, cur);

  m_cam.computeFov(I.getWidth(), I.getHeight());

  if (useScanLine) {
    faces.computeClippedPolygons(m_cMo, m_cam);
    faces.computeScanLineRender(m_cam, I.getWidth(), I.getHeight());
  }

  // mask
  cv::Mat mask((int)I.getRows(), (int)I.getCols(), CV_8UC1, cv::Scalar(0));

  vpMbtDistanceKltPoints *kltpoly;
  vpMbtDistanceKltCylinder *kltPolyCylinder;
  if (useScanLine) {
    vpImageConvert::convert(faces.getMbScanLineRenderer().getMask(), mask);
  }
  else {
    unsigned char val = 255 /* - i*15*/;
    for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
      kltpoly = *it;
      if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2) {
        // need to changeFrame when reinit() is called by postTracking
        // other solution is
        kltpoly->polygon->changeFrame(m_cMo);
        kltpoly->polygon->computePolygonClipped(m_cam); // Might not be necessary when scanline is activated
        kltpoly->updateMask(mask, val, maskBorder);
      }
    }

    for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
         ++it) {
      kltPolyCylinder = *it;

      if (kltPolyCylinder->isTracked()) {
        for (unsigned int k = 0; k < kltPolyCylinder->listIndicesCylinderBBox.size(); k++) {
          unsigned int indCylBBox = (unsigned int)kltPolyCylinder->listIndicesCylinderBBox[k];
          if (faces[indCylBBox]->isVisible() && faces[indCylBBox]->getNbPoint() > 2u) {
            faces[indCylBBox]->computePolygonClipped(m_cam); // Might not be necessary when scanline is activated
          }
        }

        kltPolyCylinder->updateMask(mask, val, maskBorder);
      }
    }
  }

  tracker.initTracking(cur, mask);
  //  tracker.track(cur); // AY: Not sure to be usefull but makes sure that
  //  the points are valid for tracking and avoid too fast reinitialisations.
  //  vpCTRACE << "init klt. detected " << tracker.getNbFeatures() << "
  //  points" << std::endl;

  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    kltpoly = *it;
    if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2) {
      kltpoly->init(tracker, m_mask);
    }
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    kltPolyCylinder = *it;

    if (kltPolyCylinder->isTracked())
      kltPolyCylinder->init(tracker, m_cMo);
  }
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.
*/
void vpMbKltTracker::resetTracker()
{
  m_cMo.eye();

  // delete the Klt Polygon features
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly != nullptr) {
      delete kltpoly;
    }
    kltpoly = nullptr;
  }
  kltPolygons.clear();

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    if (kltPolyCylinder != nullptr) {
      delete kltPolyCylinder;
    }
    kltPolyCylinder = nullptr;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    if (ci != nullptr) {
      delete ci;
    }
    ci = nullptr;
  }

  circles_disp.clear();

  m_computeInteraction = true;
  firstInitialisation = true;
  computeCovariance = false;

  tracker.setTrackerId(1);
  tracker.setUseHarris(1);

  tracker.setMaxFeatures(10000);
  tracker.setWindowSize(5);
  tracker.setQuality(0.01);
  tracker.setMinDistance(5);
  tracker.setHarrisFreeParameter(0.01);
  tracker.setBlockSize(3);
  tracker.setPyramidLevels(3);

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);

  clippingFlag = vpPolygon3D::NO_CLIPPING;

  maskBorder = 5;
  threshold_outlier = 0.5;
  percentGood = 0.6;

  m_lambda = 0.8;
  m_maxIter = 200;

  faces.reset();

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif
}

/*!
  Get the current list of KLT points.

  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f.
  This function convert and copy the openCV KLT points into vpImagePoints.

  \return the list of KLT points through vpKltOpencv.
*/
std::vector<vpImagePoint> vpMbKltTracker::getKltImagePoints() const
{
  std::vector<vpImagePoint> kltPoints;
  for (unsigned int i = 0; i < static_cast<unsigned int>(tracker.getNbFeatures()); i++) {
    long id;
    float x_tmp, y_tmp;
    tracker.getFeature((int)i, id, x_tmp, y_tmp);
    kltPoints.push_back(vpImagePoint(y_tmp, x_tmp));
  }

  return kltPoints;
}

/*!
  Get the current list of KLT points and their id.

  \warning Contrary to getKltPoints which returns a pointer on CvPoint2D32f.
  This function convert and copy the openCV KLT points into vpImagePoints.

  \return the list of KLT points and their id through vpKltOpencv.
*/
std::map<int, vpImagePoint> vpMbKltTracker::getKltImagePointsWithId() const
{
  std::map<int, vpImagePoint> kltPoints;
  for (unsigned int i = 0; i < static_cast<unsigned int>(tracker.getNbFeatures()); i++) {
    long id;
    float x_tmp, y_tmp;
    tracker.getFeature((int)i, id, x_tmp, y_tmp);
#ifdef TARGET_OS_IPHONE
    kltPoints[(int)id] = vpImagePoint(y_tmp, x_tmp);
#else
    kltPoints[id] = vpImagePoint(y_tmp, x_tmp);
#endif
  }

  return kltPoints;
}

/*!
  Set the new value of the klt tracker.

  \param t : Klt tracker containing the new values.
*/
void vpMbKltTracker::setKltOpencv(const vpKltOpencv &t)
{
  tracker.setMaxFeatures(t.getMaxFeatures());
  tracker.setWindowSize(t.getWindowSize());
  tracker.setQuality(t.getQuality());
  tracker.setMinDistance(t.getMinDistance());
  tracker.setHarrisFreeParameter(t.getHarrisFreeParameter());
  tracker.setBlockSize(t.getBlockSize());
  tracker.setPyramidLevels(t.getPyramidLevels());
}

/*!
  Set the camera parameters.

  \param cam : the new camera parameters.
*/
void vpMbKltTracker::setCameraParameters(const vpCameraParameters &cam)
{
  //  for (unsigned int i = 0; i < faces.size(); i += 1){
  //    faces[i]->setCameraParameters(camera);
  //  }

  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    kltpoly->setCameraParameters(cam);
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    kltPolyCylinder->setCameraParameters(cam);
  }

  m_cam = cam;
}

void vpMbKltTracker::setPose(const vpImage<unsigned char> *const I, const vpImage<vpRGBa> *const I_color,
                             const vpHomogeneousMatrix &cdMo)
{
  if (I_color) {
    vpImageConvert::convert(*I_color, m_I);
  }

  if (!kltCylinders.empty()) {
    std::cout << "WARNING: Cannot set pose when model contains cylinder(s). "
      "This feature is not implemented yet."
      << std::endl;
    std::cout << "Tracker will be reinitialized with the given pose." << std::endl;
    m_cMo = cdMo;
    if (I) {
      init(*I);
    }
    else {
      init(m_I);
    }
  }
  else {
    vpMbtDistanceKltPoints *kltpoly;

    std::vector<cv::Point2f> init_pts;
    std::vector<long> init_ids;
    std::vector<cv::Point2f> guess_pts;

    vpHomogeneousMatrix cdMc = cdMo * m_cMo.inverse();
    vpHomogeneousMatrix cMcd = cdMc.inverse();

    vpRotationMatrix cdRc;
    vpTranslationVector cdtc;

    cdMc.extract(cdRc);
    cdMc.extract(cdtc);

    // unsigned int nbCur = 0;

    for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
      kltpoly = *it;

      if (kltpoly->polygon->isVisible() && kltpoly->polygon->getNbPoint() > 2) {
        kltpoly->polygon->changeFrame(cdMo);

        // Get the normal to the face at the current state cMo
        vpPlane plan(kltpoly->polygon->p[0], kltpoly->polygon->p[1], kltpoly->polygon->p[2]);
        plan.changeFrame(cMcd);

        vpColVector Nc = plan.getNormal();
        Nc.normalize();

        double invDc = 1.0 / plan.getD();

        // Create the homography
        vpMatrix cdHc;
        vpGEMM(cdtc, Nc, -invDc, cdRc, 1.0, cdHc, VP_GEMM_B_T);
        cdHc /= cdHc[2][2];

        // Compute homography in the pixel space cdGc = K * cdHc * K^{-1}
        vpMatrix cdGc = homography2collineation(cdHc, m_cam);

        // Points displacement
        std::map<int, vpImagePoint>::const_iterator iter = kltpoly->getCurrentPoints().begin();
        // nbCur+= (unsigned int)kltpoly->getCurrentPoints().size();
        for (; iter != kltpoly->getCurrentPoints().end(); ++iter) {
#ifdef TARGET_OS_IPHONE
          if (std::find(init_ids.begin(), init_ids.end(), (long)(kltpoly->getCurrentPointsInd())[(int)iter->first]) !=
              init_ids.end())
#else
          if (std::find(init_ids.begin(), init_ids.end(),
                        (long)(kltpoly->getCurrentPointsInd())[(size_t)iter->first]) != init_ids.end())
#endif
          {
            // KLT point already processed (a KLT point can exist in another
            // vpMbtDistanceKltPoints due to possible overlapping faces)
            continue;
          }

          vpColVector cdp(3);
          cdp[0] = iter->second.get_j();
          cdp[1] = iter->second.get_i();
          cdp[2] = 1.0;

          cv::Point2f p((float)cdp[0], (float)cdp[1]);
          init_pts.push_back(p);
#ifdef TARGET_OS_IPHONE
          init_ids.push_back((size_t)(kltpoly->getCurrentPointsInd())[(int)iter->first]);
#else
          init_ids.push_back((size_t)(kltpoly->getCurrentPointsInd())[(size_t)iter->first]);
#endif

          double p_mu_t_2 = cdp[0] * cdGc[2][0] + cdp[1] * cdGc[2][1] + cdGc[2][2];

          if (fabs(p_mu_t_2) < std::numeric_limits<double>::epsilon()) {
            cdp[0] = 0.0;
            cdp[1] = 0.0;
            throw vpException(vpException::divideByZeroError, "the depth of the point is calculated to zero");
          }

          cdp[0] = (cdp[0] * cdGc[0][0] + cdp[1] * cdGc[0][1] + cdGc[0][2]) / p_mu_t_2;
          cdp[1] = (cdp[0] * cdGc[1][0] + cdp[1] * cdGc[1][1] + cdGc[1][2]) / p_mu_t_2;

          // Set value to the KLT tracker
          cv::Point2f p_guess((float)cdp[0], (float)cdp[1]);
          guess_pts.push_back(p_guess);
        }
      }
    }

    if (I) {
      vpImageConvert::convert(*I, cur);
    }
    else {
      vpImageConvert::convert(m_I, cur);
    }

    tracker.setInitialGuess(init_pts, guess_pts, init_ids);

    bool reInitialisation = false;
    if (!useOgre) {
      if (I) {
        faces.setVisible(I->getWidth(), I->getHeight(), m_cam, cdMo, angleAppears, angleDisappears, reInitialisation);
      }
      else {
        faces.setVisible(m_I.getWidth(), m_I.getHeight(), m_cam, cdMo, angleAppears, angleDisappears, reInitialisation);
      }
    }
    else {
#ifdef VISP_HAVE_OGRE
      if (I) {
        faces.setVisibleOgre(I->getWidth(), I->getHeight(), m_cam, cdMo, angleAppears, angleDisappears,
                             reInitialisation);
      }
      else {
        faces.setVisibleOgre(m_I.getWidth(), m_I.getHeight(), m_cam, cdMo, angleAppears, angleDisappears,
                             reInitialisation);
      }
#else
      if (I) {
        faces.setVisible(I->getWidth(), I->getHeight(), m_cam, cdMo, angleAppears, angleDisappears, reInitialisation);
      }
      else {
        faces.setVisible(m_I.getWidth(), m_I.getHeight(), m_cam, cdMo, angleAppears, angleDisappears, reInitialisation);
      }
#endif
    }

    m_cam.computeFov(I ? I->getWidth() : m_I.getWidth(), I ? I->getHeight() : m_I.getHeight());

    if (useScanLine) {
      faces.computeClippedPolygons(cdMo, m_cam);
      faces.computeScanLineRender(m_cam, I ? I->getWidth() : m_I.getWidth(), I ? I->getHeight() : m_I.getHeight());
    }

    for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
      kltpoly = *it;
      if (kltpoly->polygon->isVisible() && kltpoly->polygon->getNbPoint() > 2) {
        kltpoly->polygon->computePolygonClipped(m_cam);
        kltpoly->init(tracker, m_mask);
      }
    }

    m_cMo = cdMo;
    c0Mo = m_cMo;
    ctTc0.eye();
  }
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track()
  function. This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders.

  \param I : grayscale image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void vpMbKltTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo)
{
  vpMbKltTracker::setPose(&I, nullptr, cdMo);
}

/*!
  Set the pose to be used in entry (as guess) of the next call to the track()
  function. This pose will be just used once.

  \warning This functionnality is not available when tracking cylinders.

  \param I_color : color image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void vpMbKltTracker::setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo)
{
  vpMbKltTracker::setPose(nullptr, &I_color, cdMo);
}

/*!
  Initialise a new face from the coordinates given in parameter.

  \param polygon : The polygon describing the set of lines that has to be
  tracked.
*/
void vpMbKltTracker::initFaceFromCorners(vpMbtPolygon &polygon)
{
  vpMbtDistanceKltPoints *kltPoly = new vpMbtDistanceKltPoints();
  kltPoly->setCameraParameters(m_cam);
  kltPoly->polygon = &polygon;
  kltPoly->hiddenface = &faces;
  kltPoly->useScanLine = useScanLine;
  kltPolygons.push_back(kltPoly);
}
/*!
  Initialise a new face from the coordinates given in parameter.

  \param polygon : The polygon describing the set of lines that has to be
  tracked.
*/
void vpMbKltTracker::initFaceFromLines(vpMbtPolygon &polygon)
{
  vpMbtDistanceKltPoints *kltPoly = new vpMbtDistanceKltPoints();
  kltPoly->setCameraParameters(m_cam);
  kltPoly->polygon = &polygon;
  kltPoly->hiddenface = &faces;
  kltPoly->useScanLine = useScanLine;
  kltPolygons.push_back(kltPoly);
}

/*!
  Achieve the tracking of the KLT features and associate the features to the
  faces.

  \param I : The input image.
*/
void vpMbKltTracker::preTracking(const vpImage<unsigned char> &I)
{
  vpImageConvert::convert(I, cur);
  tracker.track(cur);

  m_nbInfos = 0;
  m_nbFaceUsed = 0;
  //  for (unsigned int i = 0; i < faces.size(); i += 1){
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2) {
      kltpoly->computeNbDetectedCurrent(tracker, m_mask);
      //       faces[i]->ransac();
      if (kltpoly->hasEnoughPoints()) {
        m_nbInfos += kltpoly->getCurrentNumberPoints();
        m_nbFaceUsed++;
      }
    }
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;

    if (kltPolyCylinder->isTracked()) {
      kltPolyCylinder->computeNbDetectedCurrent(tracker);
      if (kltPolyCylinder->hasEnoughPoints()) {
        m_nbInfos += kltPolyCylinder->getCurrentNumberPoints();
        m_nbFaceUsed++;
      }
    }
  }
}

/*!
  Realize the post tracking operations. Mostly visibility tests
*/
bool vpMbKltTracker::postTracking(const vpImage<unsigned char> &I, vpColVector &w)
{
  // # For a better Post Tracking, tracker should reinitialize if so faces
  // don't have enough points but are visible. # Here we are not doing it for
  // more speed performance.
  bool reInitialisation = false;

  unsigned int initialNumber = 0;
  unsigned int currentNumber = 0;
  unsigned int shift = 0;
  //  for (unsigned int i = 0; i < faces.size(); i += 1){
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2) {
      initialNumber += kltpoly->getInitialNumberPoint();
      if (kltpoly->hasEnoughPoints()) {
        vpSubColVector sub_w(w, shift, 2 * kltpoly->getCurrentNumberPoints());
        shift += 2 * kltpoly->getCurrentNumberPoints();
        kltpoly->removeOutliers(sub_w, threshold_outlier);

        currentNumber += kltpoly->getCurrentNumberPoints();
      }
      //       else{
      //         reInitialisation = true;
      //         break;
      //       }
    }
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;

    if (kltPolyCylinder->isTracked()) {
      initialNumber += kltPolyCylinder->getInitialNumberPoint();
      if (kltPolyCylinder->hasEnoughPoints()) {
        vpSubColVector sub_w(w, shift, 2 * kltPolyCylinder->getCurrentNumberPoints());
        shift += 2 * kltPolyCylinder->getCurrentNumberPoints();
        kltPolyCylinder->removeOutliers(sub_w, threshold_outlier);

        currentNumber += kltPolyCylinder->getCurrentNumberPoints();
      }
    }
  }

  //   if(!reInitialisation){
  double value = percentGood * (double)initialNumber;
  if ((double)currentNumber < value) {
    //     std::cout << "Too many point disappear : " << initialNumber << "/"
    //     << currentNumber << std::endl;
    reInitialisation = true;
  }
  else {
    if (!useOgre)
      faces.setVisible(I.getWidth(), I.getHeight(), m_cam, m_cMo, angleAppears, angleDisappears, reInitialisation);
    else {
#ifdef VISP_HAVE_OGRE
      faces.setVisibleOgre(I.getWidth(), I.getHeight(), m_cam, m_cMo, angleAppears, angleDisappears, reInitialisation);
#else
      faces.setVisible(I.getWidth(), I.getHeight(), m_cam, m_cMo, angleAppears, angleDisappears, reInitialisation);
#endif
    }
  }
  //   }

  if (reInitialisation)
    return true;

  return false;
}

/*!
  Realize the VVS loop for the tracking
*/
void vpMbKltTracker::computeVVS()
{
  vpMatrix L_true; // interaction matrix without weighting
  vpMatrix LVJ_true;
  vpColVector v; // "speed" for VVS

  vpMatrix LTL;
  vpColVector LTR;
  vpHomogeneousMatrix cMoPrev;
  vpHomogeneousMatrix ctTc0_Prev;
  vpColVector error_prev;
  double mu = m_initialMu;

  double normRes = 0;
  double normRes_1 = -1;
  unsigned int iter = 0;

  bool isoJoIdentity = m_isoJoIdentity; // Backup since it can be modified if L is not full rank
  if (isoJoIdentity)
    oJo.eye();

  vpMbKltTracker::computeVVSInit();

  while (((int)((normRes - normRes_1) * 1e8) != 0) && (iter < m_maxIter)) {
    vpMbKltTracker::computeVVSInteractionMatrixAndResidu();

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_klt, error_prev, cMoPrev, mu, reStartFromLastIncrement);
    if (reStartFromLastIncrement) {
      ctTc0 = ctTc0_Prev;
    }

    if (!reStartFromLastIncrement) {
      vpMbTracker::computeVVSWeights(m_robust_klt, m_error_klt, m_w_klt);

      if (computeCovariance) {
        L_true = m_L_klt;
        if (!isoJoIdentity) {
          vpVelocityTwistMatrix cVo;
          cVo.build(m_cMo);
          LVJ_true = (m_L_klt * cVo * oJo);
        }
      }

      normRes_1 = normRes;
      normRes = 0.0;

      for (unsigned int i = 0; i < m_error_klt.getRows(); i++) {
        m_weightedError_klt[i] = m_error_klt[i] * m_w_klt[i];
        normRes += m_weightedError_klt[i];
      }

      if ((iter == 0) || m_computeInteraction) {
        for (unsigned int i = 0; i < m_error_klt.getRows(); i++) {
          for (unsigned int j = 0; j < 6; j++) {
            m_L_klt[i][j] *= m_w_klt[i];
          }
        }
      }

      computeVVSPoseEstimation(m_isoJoIdentity, iter, m_L_klt, LTL, m_weightedError_klt, m_error_klt, error_prev, LTR, mu, v);

      cMoPrev = m_cMo;
      ctTc0_Prev = ctTc0;
      ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
      m_cMo = ctTc0 * c0Mo;
    } // endif(!reStartFromLastIncrement)

    iter++;
  }

  computeCovarianceMatrixVVS(m_isoJoIdentity, m_w_klt, cMoPrev, L_true, LVJ_true, m_error_klt);
}

void vpMbKltTracker::computeVVSInit()
{
  unsigned int nbFeatures = 2 * m_nbInfos;

  m_L_klt.resize(nbFeatures, 6, false, false);
  m_error_klt.resize(nbFeatures, false);

  m_weightedError_klt.resize(nbFeatures, false);
  m_w_klt.resize(nbFeatures, false);
  m_w_klt = 1;

  m_robust_klt.setMinMedianAbsoluteDeviation(2 / m_cam.get_px());
}

void vpMbKltTracker::computeVVSInteractionMatrixAndResidu()
{
  unsigned int shift = 0;
  vpHomography H;

  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2 &&
        kltpoly->hasEnoughPoints()) {
      vpSubColVector subR(m_error_klt, shift, 2 * kltpoly->getCurrentNumberPoints());
      vpSubMatrix subL(m_L_klt, shift, 0, 2 * kltpoly->getCurrentNumberPoints(), 6);

      try {
        kltpoly->computeHomography(ctTc0, H);
        kltpoly->computeInteractionMatrixAndResidu(subR, subL);
      }
      catch (...) {
        throw vpTrackingException(vpTrackingException::fatalError, "Cannot compute interaction matrix");
      }

      shift += 2 * kltpoly->getCurrentNumberPoints();
    }
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;

    if (kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints()) {
      vpSubColVector subR(m_error_klt, shift, 2 * kltPolyCylinder->getCurrentNumberPoints());
      vpSubMatrix subL(m_L_klt, shift, 0, 2 * kltPolyCylinder->getCurrentNumberPoints(), 6);

      try {
        kltPolyCylinder->computeInteractionMatrixAndResidu(ctTc0, subR, subL);
      }
      catch (...) {
        throw vpTrackingException(vpTrackingException::fatalError, "Cannot compute interaction matrix");
      }

      shift += 2 * kltPolyCylinder->getCurrentNumberPoints();
    }
  }
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I : the input grayscale image
*/
void vpMbKltTracker::track(const vpImage<unsigned char> &I)
{
  preTracking(I);

  if (m_nbInfos < 4 || m_nbFaceUsed == 0) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  }

  computeVVS();

  if (postTracking(I, m_w_klt))
    reinit(I);

  if (displayFeatures) {
    m_featuresToBeDisplayedKlt = getFeaturesForDisplayKlt();
  }
}

/*!
  Realize the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param I_color : the input color image
*/
void vpMbKltTracker::track(const vpImage<vpRGBa> &I_color)
{
  vpImageConvert::convert(I_color, m_I);
  preTracking(m_I);

  if (m_nbInfos < 4 || m_nbFaceUsed == 0) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "Error: not enough features");
  }

  computeVVS();

  if (postTracking(m_I, m_w_klt))
    reinit(m_I);
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the
  objects: KLT, camera.

  \throw vpException::ioError if the file has not been properly parsed (file
  not found or wrong format for the data).

  \param configFile : full name of the xml file.
  \param verbose : Set true to activate the verbose mode, false otherwise.

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
*/
void vpMbKltTracker::loadConfigFile(const std::string &configFile, bool verbose)
{
#if defined(VISP_HAVE_PUGIXML)
// Load projection error config
  vpMbTracker::loadConfigFile(configFile, verbose);

  vpMbtXmlGenericParser xmlp(vpMbtXmlGenericParser::KLT_PARSER);
  xmlp.setVerbose(verbose);
  xmlp.setKltMaxFeatures(10000);
  xmlp.setKltWindowSize(5);
  xmlp.setKltQuality(0.01);
  xmlp.setKltMinDistance(5);
  xmlp.setKltHarrisParam(0.01);
  xmlp.setKltBlockSize(3);
  xmlp.setKltPyramidLevels(3);
  xmlp.setKltMaskBorder(maskBorder);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));

  try {
    if (verbose) {
      std::cout << " *********** Parsing XML for MBT KLT Tracker ************ " << std::endl;
    }
    xmlp.parse(configFile.c_str());
  }
  catch (...) {
    vpERROR_TRACE("Can't open XML file \"%s\"\n ", configFile.c_str());
    throw vpException(vpException::ioError, "problem to parse configuration file.");
  }

  vpCameraParameters camera;
  xmlp.getCameraParameters(camera);
  setCameraParameters(camera);

  tracker.setMaxFeatures((int)xmlp.getKltMaxFeatures());
  tracker.setWindowSize((int)xmlp.getKltWindowSize());
  tracker.setQuality(xmlp.getKltQuality());
  tracker.setMinDistance(xmlp.getKltMinDistance());
  tracker.setHarrisFreeParameter(xmlp.getKltHarrisParam());
  tracker.setBlockSize((int)xmlp.getKltBlockSize());
  tracker.setPyramidLevels((int)xmlp.getKltPyramidLevels());
  maskBorder = xmlp.getKltMaskBorder();
  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  // if(useScanLine)
  faces.getMbScanLineRenderer().setMaskBorder(maskBorder);

  if (xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());

  if (xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());

  if (xmlp.getFovClipping())
    setClipping(clippingFlag = clippingFlag | vpPolygon3D::FOV_CLIPPING);

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
#else
  (void)configFile;
  (void)verbose;
  throw(vpException(vpException::ioError, "vpMbKltTracker::loadConfigFile() needs pugixml built-in 3rdparty"));
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : Boolean to say if all the model has to be
  displayed, even the faces that are visible.
*/
void vpMbKltTracker::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                             const vpCameraParameters &cam, const vpColor &col, unsigned int thickness,
                             bool displayFullModel)
{
  std::vector<std::vector<double> > models =
    vpMbKltTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo, cam, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    }
    else if (vpMath::equal(models[i][0], 1)) {
      vpImagePoint center(models[i][1], models[i][2]);
      double n20 = models[i][3];
      double n11 = models[i][4];
      double n02 = models[i][5];
      vpDisplay::displayEllipse(I, center, n20, n11, n02, true, col, thickness);
    }
  }

  if (displayFeatures) {
    for (size_t i = 0; i < m_featuresToBeDisplayedKlt.size(); i++) {
      if (vpMath::equal(m_featuresToBeDisplayedKlt[i][0], 1)) {
        vpImagePoint ip1(m_featuresToBeDisplayedKlt[i][1], m_featuresToBeDisplayedKlt[i][2]);
        vpDisplay::displayCross(I, ip1, 10, vpColor::red);

        vpImagePoint ip2(m_featuresToBeDisplayedKlt[i][3], m_featuresToBeDisplayedKlt[i][4]);
        double id = m_featuresToBeDisplayedKlt[i][5];
        std::stringstream ss;
        ss << id;
        vpDisplay::displayText(I, ip2, ss.str(), vpColor::red);
      }
    }
  }

#ifdef VISP_HAVE_OGRE
  if (useOgre)
    faces.displayOgre(cMo);
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The color image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : Boolean to say if all the model has to be
  displayed, even the faces that are not visible.
*/
void vpMbKltTracker::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                             const vpColor &col, unsigned int thickness, bool displayFullModel)
{
  std::vector<std::vector<double> > models =
    vpMbKltTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo, cam, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    }
    else if (vpMath::equal(models[i][0], 1)) {
      vpImagePoint center(models[i][1], models[i][2]);
      double n20 = models[i][3];
      double n11 = models[i][4];
      double n02 = models[i][5];
      vpDisplay::displayEllipse(I, center, n20, n11, n02, true, col, thickness);
    }
  }

  if (displayFeatures) {
    for (size_t i = 0; i < m_featuresToBeDisplayedKlt.size(); i++) {
      if (vpMath::equal(m_featuresToBeDisplayedKlt[i][0], 1)) {
        vpImagePoint ip1(m_featuresToBeDisplayedKlt[i][1], m_featuresToBeDisplayedKlt[i][2]);
        vpDisplay::displayCross(I, ip1, 10, vpColor::red);

        vpImagePoint ip2(m_featuresToBeDisplayedKlt[i][3], m_featuresToBeDisplayedKlt[i][4]);
        double id = m_featuresToBeDisplayedKlt[i][5];
        std::stringstream ss;
        ss << id;
        vpDisplay::displayText(I, ip2, ss.str(), vpColor::red);
      }
    }
  }

#ifdef VISP_HAVE_OGRE
  if (useOgre)
    faces.displayOgre(cMo);
#endif
}

std::vector<std::vector<double> > vpMbKltTracker::getFeaturesForDisplayKlt()
{
  std::vector<std::vector<double> > features;

  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;

    if (kltpoly->hasEnoughPoints() && kltpoly->polygon->isVisible() && kltpoly->isTracked()) {
      std::vector<std::vector<double> > currentFeatures = kltpoly->getFeaturesForDisplay();
      features.insert(features.end(), currentFeatures.begin(), currentFeatures.end());
    }
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;

    if (kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints()) {
      std::vector<std::vector<double> > currentFeatures = kltPolyCylinder->getFeaturesForDisplay();
      features.insert(features.end(), currentFeatures.begin(), currentFeatures.end());
    }
  }

  return features;
}

/*!
  Return a list of primitives parameters to display the model at a given pose and camera parameters.
  - Line parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`
  - Ellipse parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<n_20>`, `<n_11>`, `<n_02>` where `<n_ij>` are the second order centered moments of the ellipse
  normalized by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments and a
  the area).

  \param width : Image width.
  \param height : Image height.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbKltTracker::getModelForDisplay(unsigned int width, unsigned int height,
                                                                     const vpHomogeneousMatrix &cMo,
                                                                     const vpCameraParameters &cam,
                                                                     bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  vpCameraParameters c = cam;

  if (clippingFlag > 3) // Contains at least one FOV constraint
    c.computeFov(width, height);

  //  vpMbtDistanceKltPoints *kltpoly;
  //  vpMbtDistanceKltCylinder *kltPolyCylinder;

  // Previous version 12/08/2015
  //  for(std::list<vpMbtDistanceKltPoints*>::const_iterator
  //  it=kltPolygons.begin(); it!=kltPolygons.end(); ++it){
  //    kltpoly = *it;
  //    kltpoly->polygon->changeFrame(cMo_);
  //    kltpoly->polygon->computePolygonClipped(c);
  //  }
  faces.computeClippedPolygons(cMo, c);

  if (useScanLine && !displayFullModel)
    faces.computeScanLineRender(m_cam, width, height);

  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    std::vector<std::vector<double> > modelLines = kltpoly->getModelForDisplay(cam, displayFullModel);
    models.insert(models.end(), modelLines.begin(), modelLines.end());
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    std::vector<std::vector<double> > modelLines = kltPolyCylinder->getModelForDisplay(cMo, cam);
    models.insert(models.end(), modelLines.begin(), modelLines.end());
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    vpMbtDistanceCircle *displayCircle = *it;
    std::vector<double> paramsCircle = displayCircle->getModelForDisplay(cMo, cam, displayFullModel);
    if (!paramsCircle.empty()) {
      models.push_back(paramsCircle);
    }
  }

  return models;
}

/*!
  Test the quality of the tracking.
  The tracking is supposed to fail if less than 10 points are tracked.

  \throw vpTrackingException::fatalError  if the test fails.
*/
void vpMbKltTracker::testTracking()
{
  unsigned int nbTotalPoints = 0;
  //  for (unsigned int i = 0; i < faces.size(); i += 1){
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly->polygon->isVisible() && kltpoly->isTracked() && kltpoly->polygon->getNbPoint() > 2 &&
        kltpoly->hasEnoughPoints()) {
      nbTotalPoints += kltpoly->getCurrentNumberPoints();
    }
  }

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    if (kltPolyCylinder->isTracked() && kltPolyCylinder->hasEnoughPoints())
      nbTotalPoints += kltPolyCylinder->getCurrentNumberPoints();
  }

  if (nbTotalPoints < 10) {
    std::cerr << "test tracking failed (too few points to realize a good tracking)." << std::endl;
    throw vpTrackingException(vpTrackingException::fatalError,
                              "test tracking failed (too few points to realize a good tracking).");
  }
}

/*!
  Add a cylinder to display (not for tracking) from two points on the axis
  (defining the length of the cylinder) and its radius.

  \param p1 : First point on the axis.
  \param p2 : Second point on the axis.
  \param radius : Radius of the cylinder.
  \param idFace : Identifier of the polygon representing the revolution axis
  of the cylinder. \param name : The optional name of the cylinder.
*/
void vpMbKltTracker::initCylinder(const vpPoint &p1, const vpPoint &p2, double radius, int idFace,
                                  const std::string & /*name*/)
{
  vpMbtDistanceKltCylinder *kltPoly = new vpMbtDistanceKltCylinder();
  kltPoly->setCameraParameters(m_cam);

  kltPoly->buildFrom(p1, p2, radius);

  // Add the Cylinder BBox to the list of polygons
  kltPoly->listIndicesCylinderBBox.push_back(idFace + 1);
  kltPoly->listIndicesCylinderBBox.push_back(idFace + 2);
  kltPoly->listIndicesCylinderBBox.push_back(idFace + 3);
  kltPoly->listIndicesCylinderBBox.push_back(idFace + 4);

  kltPoly->hiddenface = &faces;
  kltPoly->useScanLine = useScanLine;
  kltCylinders.push_back(kltPoly);
}

/*!
  Add a circle to display (not for tracking) from its center, 3 points
  (including the center) defining the plane that contain the circle and its
  radius.

  \param p1 : Center of the circle.
  \param p2,p3 : Two points on the plane containing the circle. With the
  center of the circle we have 3 points defining the plane that contains the
  circle. \param radius : Radius of the circle. \param name : The optional
  name of the circle.
*/
void vpMbKltTracker::initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, double radius, int /*idFace*/,
                                const std::string &name)
{
  addCircle(p1, p2, p3, radius, name);
}

/*!
  Add a circle to the list of circles.

  \param P1 : Center of the circle.
  \param P2,P3 : Two points on the plane containing the circle. With the
  center of the circle we have 3 points defining the plane that contains the
  circle. \param r : Radius of the circle. \param name : Name of the circle.
*/
void vpMbKltTracker::addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, double r,
                               const std::string &name)
{
  bool already_here = false;

  //  for(std::list<vpMbtDistanceCircle*>::const_iterator
  //  it=circles_disp.begin(); it!=circles_disp[i].end(); ++it){
  //    ci = *it;
  //    if((samePoint(*(ci->p1),P1) && samePoint(*(ci->p2),P2) &&
  //    samePoint(*(ci->p3),P3)) ||
  //       (samePoint(*(ci->p1),P1) && samePoint(*(ci->p2),P3) &&
  //       samePoint(*(ci->p3),P2)) ){
  //      already_here = (std::fabs(ci->radius - r) <
  //      std::numeric_limits<double>::epsilon() * vpMath::maximum(ci->radius,
  //      r));
  //    }
  //  }

  if (!already_here) {
    vpMbtDistanceCircle *ci = new vpMbtDistanceCircle;

    ci->setCameraParameters(m_cam);
    ci->setName(name);
    ci->buildFrom(P1, P2, P3, r);
    circles_disp.push_back(ci);
  }
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo : The new vpHomogeneousMatrix between the camera and the new
  model
  \param verbose : verbose option to print additional information when
  loading CAO model files which include other CAO model files.
  \param T : optional transformation matrix (currently only for .cao) to transform
  3D points expressed in the original object frame to the desired object frame.
*/
void vpMbKltTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                                 const vpHomogeneousMatrix &cMo, bool verbose, const vpHomogeneousMatrix &T)
{
  m_cMo.eye();

  firstInitialisation = true;

  // delete the Klt Polygon features
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly != nullptr) {
      delete kltpoly;
    }
    kltpoly = nullptr;
  }
  kltPolygons.clear();

  for (std::list<vpMbtDistanceKltCylinder *>::const_iterator it = kltCylinders.begin(); it != kltCylinders.end();
       ++it) {
    vpMbtDistanceKltCylinder *kltPolyCylinder = *it;
    if (kltPolyCylinder != nullptr) {
      delete kltPolyCylinder;
    }
    kltPolyCylinder = nullptr;
  }
  kltCylinders.clear();

  // delete the structures used to display circles
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles_disp.begin(); it != circles_disp.end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    if (ci != nullptr) {
      delete ci;
    }
    ci = nullptr;
  }

  faces.reset();

  loadModel(cad_name, verbose, T);
  initFromPose(I, cMo);
}

/*!
  Set if the polygons that have the given name have to be considered during
  the tracking phase.

  \param name : name of the polygon(s).
  \param useKltTracking : True if it has to be considered, False otherwise.
*/
void vpMbKltTracker::setUseKltTracking(const std::string &name, const bool &useKltTracking)
{
  for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it) {
    vpMbtDistanceKltPoints *kltpoly = *it;
    if (kltpoly->polygon->getName() == name) {
      kltpoly->setTracked(useKltTracking);
    }
  }
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_mbt.a(vpMbKltTracker.cpp.o) has no symbols
void dummy_vpMbKltTracker() { };
#endif // VISP_HAVE_OPENCV
