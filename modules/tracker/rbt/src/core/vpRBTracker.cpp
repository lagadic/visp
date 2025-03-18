/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 */

#include <visp3/rbt/vpRBTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>

#include <visp3/ar/vpPanda3DRendererSet.h>
#include <visp3/ar/vpPanda3DGeometryRenderer.h>
#include <visp3/ar/vpPanda3DRGBRenderer.h>

#include <visp3/rbt/vpRBFeatureTrackerFactory.h>
#include <visp3/rbt/vpRBDriftDetectorFactory.h>
#include <visp3/rbt/vpObjectMaskFactory.h>
#include <visp3/rbt/vpRBVisualOdometry.h>
#include <visp3/rbt/vpRBInitializationHelper.h>

BEGIN_VISP_NAMESPACE

vpRBTracker::vpRBTracker() :
  m_firstIteration(true), m_trackers(0), m_lambda(1.0), m_vvsIterations(10), m_muInit(0.0), m_muIterFactor(0.5), m_scaleInvariantOptim(false),
  m_renderer(m_rendererSettings), m_imageHeight(480), m_imageWidth(640), m_verbose(false), m_convergenceMetric(1024, 41), m_convergedMetricThreshold(0.0), m_displaySilhouette(false)
{
  m_rendererSettings.setClippingDistance(0.01, 1.0);
  m_renderer.setRenderParameters(m_rendererSettings);

  m_driftDetector = nullptr;
  m_mask = nullptr;
  m_odometry = nullptr;
}

void vpRBTracker::getPose(vpHomogeneousMatrix &cMo) const
{
  cMo = m_cMo;
}

void vpRBTracker::setPose(const vpHomogeneousMatrix &cMo)
{
  m_cMo = cMo;
  m_cMoPrev = cMo;
  m_renderer.setCameraPose(cMo.inverse());
}

vpMatrix vpRBTracker::getCovariance() const
{
  vpMatrix sumInvs(6, 6, 0.0);
  double sumWeights = 0.0;
  for (const std::shared_ptr<vpRBFeatureTracker> &tracker: m_trackers) {
    if (tracker->getNumFeatures() == 0) {
      continue;
    }
    tracker->updateCovariance(m_lambda);
    vpMatrix trackerCov = tracker->getCovariance();
    double trackerWeight = tracker->getVVSTrackerWeight();
    if (trackerCov.getRows() != 6 || trackerCov.getCols() != 6) {
      throw vpException(vpException::dimensionError,
      "Expected tracker pose covariance to have dimensions 6x6, but got %dx%d",
      trackerCov.getRows(), trackerCov.getCols());
    }

    sumInvs += (trackerWeight * trackerCov.pseudoInverse());
    sumWeights += trackerWeight;
  }
  return sumWeights * sumInvs.pseudoInverse();
}

vpCameraParameters vpRBTracker::getCameraParameters() const { return m_cam; }

void vpRBTracker::setCameraParameters(const vpCameraParameters &cam, unsigned h, unsigned w)
{
  if (cam.get_projModel() != vpCameraParameters::vpCameraParametersProjType::perspectiveProjWithoutDistortion) {
    throw vpException(vpException::badValue,
    "Camera model cannot have distortion. Undistort images before tracking and use the undistorted camera model");
  }
  if (h == 0 || w == 0) {
    throw vpException(
      vpException::badValue,
      "Image dimensions must be greater than 0"
    );
  }
  m_cam = cam;
  m_imageHeight = h;
  m_imageWidth = w;
  m_rendererSettings.setCameraIntrinsics(m_cam);
  m_rendererSettings.setImageResolution(m_imageHeight, m_imageWidth);
  m_renderer.setRenderParameters(m_rendererSettings);
}

void vpRBTracker::setSilhouetteExtractionParameters(const vpSilhouettePointsExtractionSettings &settings)
{
  m_depthSilhouetteSettings = settings;
}

void vpRBTracker::reset()
{
  m_firstIteration = true;
}

void vpRBTracker::setModelPath(const std::string &path)
{
  m_modelPath = path;
}

void vpRBTracker::setupRenderer(const std::string &file)
{
  if (!vpIoTools::checkFilename(file)) {
    throw vpException(vpException::badValue, "3D model file %s could not be found", file.c_str());
  }

  const std::shared_ptr<vpPanda3DGeometryRenderer> geometryRenderer = std::make_shared<vpPanda3DGeometryRenderer>(
    vpPanda3DGeometryRenderer::vpRenderType::OBJECT_NORMALS);
  m_renderer.addSubRenderer(geometryRenderer);

  bool requiresSilhouetteShader = false;
  for (std::shared_ptr<vpRBFeatureTracker> &tracker: m_trackers) {
    if (tracker->requiresSilhouetteCandidates()) {
      requiresSilhouetteShader = true;
      break;
    }
  }
  if (requiresSilhouetteShader) {
    m_renderer.addSubRenderer(std::make_shared<vpPanda3DDepthCannyFilter>(
      "depthCanny", geometryRenderer, true, 0.0));
  }
  m_renderer.initFramework();
  m_renderer.addLight(vpPanda3DAmbientLight("ambient", vpRGBf(0.4f)));
  m_renderer.addNodeToScene(m_renderer.loadObject("object", file));
  m_renderer.setFocusedObject("object");
}

void vpRBTracker::track(const vpImage<unsigned char> &I)
{
  for (std::shared_ptr<vpRBFeatureTracker> tracker : m_trackers) {
    if (tracker->requiresDepth() || tracker->requiresRGB()) {
      throw vpException(vpException::badValue, "Some tracked features require RGB or depth features");
    }
  }
  checkDimensionsOrThrow(I, "grayscale");
  vpRBFeatureTrackerInput frameInput;
  frameInput.I = I;
  frameInput.cam = m_cam;
  track(frameInput);
}

void vpRBTracker::track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB)
{
  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    if (tracker->requiresDepth()) {
      throw vpException(vpException::badValue, "Some tracked features require depth features");
    }
  }
  checkDimensionsOrThrow(I, "grayscale");
  checkDimensionsOrThrow(IRGB, "color");
  vpRBFeatureTrackerInput frameInput;
  frameInput.I = I;
  frameInput.IRGB = IRGB;
  frameInput.cam = m_cam;
  track(frameInput);
}

void vpRBTracker::startTracking()
{
  setupRenderer(m_modelPath);
  m_convergenceMetric.sampleObject(m_renderer);
}

void vpRBTracker::track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth)
{
  checkDimensionsOrThrow(I, "grayscale");
  checkDimensionsOrThrow(IRGB, "color");
  checkDimensionsOrThrow(depth, "depth");
  vpRBFeatureTrackerInput frameInput;
  frameInput.I = I;
  frameInput.IRGB = IRGB;
  frameInput.depth = depth;
  frameInput.cam = m_cam;
  track(frameInput);
}

void vpRBTracker::track(vpRBFeatureTrackerInput &input)
{
  m_logger.reset();

  m_logger.startTimer();
  updateRender(input);
  m_logger.setRenderTime(m_logger.endTimer());

  if (m_firstIteration) {
    m_firstIteration = false;
    m_previousFrame.I = input.I;
    m_previousFrame.IRGB = input.IRGB;
  }

  m_logger.startTimer();
  if (m_mask) {
    m_mask->updateMask(input, m_previousFrame, input.mask);
  }
  m_logger.setMaskTime(m_logger.endTimer());

  bool requiresSilhouetteCandidates = false;
  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    if (tracker->requiresSilhouetteCandidates()) {
      requiresSilhouetteCandidates = true;
      break;
    }
  }

  m_logger.startTimer();
  if (requiresSilhouetteCandidates) {
    const vpHomogeneousMatrix cTcp = m_cMo * m_cMoPrev.inverse();
    input.silhouettePoints = extractSilhouettePoints(input.renders.normals, input.renders.depth,
                                                    input.renders.silhouetteCanny, input.renders.isSilhouette, input.cam, cTcp);
    if (input.silhouettePoints.size() == 0) {
      throw vpException(vpException::badValue, "Could not extract silhouette from depth canny: Object may not be in image");
    }
  }
  m_logger.setSilhouetteTime(m_logger.endTimer());

  if (m_odometry) {
    m_logger.startTimer();
    m_odometry->compute(input, m_previousFrame);
    vpHomogeneousMatrix cMo_beforeOdo = m_cMo;
    vpHomogeneousMatrix cnTc = m_odometry->getCameraMotion();
    m_cMo = cnTc * m_cMo;
    bool shouldRerender = true;
    if (m_convergedMetricThreshold > 0.0 && m_cMo != cMo_beforeOdo) {
      double adds = m_convergenceMetric.ADDS(m_cMo, cMo_beforeOdo);
      std::cout << "ADDS = " << adds << std::endl;
      // Multiply by number of vvs iterations: convergence is tested between two optim iterations,
      // while we consider that odometry performs a full optim pass wrt to the environment
      shouldRerender = adds > (m_convergedMetricThreshold * m_vvsIterations);
    }
    if (shouldRerender) {
      std::cout << "Rerendering for odometry" << std::endl;
      updateRender(input);

      if (requiresSilhouetteCandidates) {
        const vpHomogeneousMatrix cTcp = m_cMo * m_cMoPrev.inverse();
        input.silhouettePoints = extractSilhouettePoints(input.renders.normals, input.renders.depth,
                                                        input.renders.silhouetteCanny, input.renders.isSilhouette, input.cam, cTcp);
        if (input.silhouettePoints.size() == 0) {
          throw vpException(vpException::badValue, "Could not extract silhouette from depth canny: Object may not be in image");
        }
      }

    }
    m_logger.setOdometryTime(m_logger.endTimer());
  }



  int id = 0;
  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    m_logger.startTimer();
    tracker->onTrackingIterStart(m_cMo);
    m_logger.setTrackerIterStartTime(id, m_logger.endTimer());
    id += 1;
  }
  id = 0;
  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    m_logger.startTimer();
    try {
      tracker->extractFeatures(input, m_previousFrame, m_cMo);
    }
    catch (vpException &e) {
      std::cerr << "Tracker " << id << " raised an exception in extractFeatures" << std::endl;
      throw e;
    }
    m_logger.setTrackerFeatureExtractionTime(id, m_logger.endTimer());
    id += 1;
  }
  id = 0;
  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    m_logger.startTimer();
    try {
      tracker->trackFeatures(input, m_previousFrame, m_cMo);
    }
    catch (vpException &e) {
      std::cerr << "Tracker " << id << " raised an exception in trackFeatures" << std::endl;
      throw e;
    }
    m_logger.setTrackerFeatureTrackingTime(id, m_logger.endTimer());
    id += 1;
  }



  id = 0;
  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    m_logger.startTimer();
    tracker->initVVS(input, m_previousFrame, m_cMo);
    m_logger.setInitVVSTime(id, m_logger.endTimer());
    //std::cout << "Tracker " << id << " has " << tracker->getNumFeatures() << " features" << std::endl;
    id += 1;
  }

  m_cMoPrev = m_cMo;
  double bestError = std::numeric_limits<double>::max();
  vpHomogeneousMatrix m_cMoPrevIter = m_cMo;
  vpHomogeneousMatrix best_cMo = m_cMo;
  double mu = m_muInit;
  vpColVector firstMotion(6, 0.0);
  for (unsigned int iter = 0; iter < m_vvsIterations; ++iter) {
    id = 0;
    for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
      m_logger.startTimer();
      try {
        tracker->computeVVSIter(input, m_cMo, iter);
      }
      catch (vpException &) {
        std::cerr << "Tracker " << id << " raised an exception in computeVVSIter" << std::endl;
        throw;
      }
      m_logger.addTrackerVVSTime(id, m_logger.endTimer());
      id += 1;
    }

    //! Check if all trackers have converged
    bool converged = true;
    for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
      if (!tracker->vvsHasConverged()) {
        converged = false;
        break;
      }
    }
    if (converged) {
      break;
    }

    vpMatrix LTL(6, 6, 0.0);
    vpColVector LTR(6, 0.0);
    double error = 0.f;
    unsigned int numFeatures = 0;


    for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
      if (tracker->getNumFeatures() > 0) {
        numFeatures += tracker->getNumFeatures();
        const double weight = tracker->getVVSTrackerWeight();
        LTL += weight * tracker->getLTL();
        LTR += weight * tracker->getLTR();
        error += weight * (tracker->getWeightedError()).sumSquare();
        //std::cout << "Error = " << (weight * tracker->getWeightedError()).sumSquare() << std::endl;
      }
    }

    if (numFeatures >= 6) {

      if (error < bestError) {
        bestError = error;
        best_cMo = m_cMo;
      }

      vpMatrix H(6, 6);
      H.eye(6);

      if (m_scaleInvariantOptim) {
        for (unsigned int i = 0; i < 6; ++i) {
          H[i][i] = LTL[i][i];
        }
      }
      try {
        vpColVector v = -m_lambda * ((LTL + mu * H).pseudoInverse(LTL.getRows() * std::numeric_limits<double>::epsilon()) * LTR);
        m_cMo = vpExponentialMap::direct(v).inverse() * m_cMo;
      }
      catch (vpException &) {
        std::cerr << "Could not compute pseudo inverse" << std::endl;
      }
      mu *= m_muIterFactor;

      if (iter > 0 && m_convergedMetricThreshold > 0.0 && m_convergenceMetric.ADDS(m_cMoPrevIter, m_cMo) < m_convergedMetricThreshold) {
        break;
      }

      m_cMoPrevIter = m_cMo;
    }
    else {
      break;
    }
  }

  //m_cMo = best_cMo;

  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    tracker->onTrackingIterEnd(m_cMo);
  }
  //m_cMo = m_kalman.filter(m_cMo, 1.0 / 20.0);
  if (m_currentFrame.I.getSize() == 0) {
    m_currentFrame = input;
    m_previousFrame = input;
  }
  else {
    m_previousFrame = std::move(m_currentFrame);
    m_currentFrame = std::move(input);
  }
  m_logger.startTimer();
  if (m_driftDetector) {
    m_driftDetector->update(m_previousFrame, m_currentFrame, m_cMo, m_cMoPrev);
  }
  m_logger.setDriftDetectionTime(m_logger.endTimer());
  if (m_verbose) {
    std::cout << m_logger << std::endl;
  }
}

void vpRBTracker::updateRender(vpRBFeatureTrackerInput &frame)
{
  m_renderer.setCameraPose(m_cMo.inverse());

  frame.renders.cMo = m_cMo;

  // Update clipping distances
  frame.renders.normals.resize(m_imageHeight, m_imageWidth);
  frame.renders.silhouetteCanny.resize(m_imageHeight, m_imageWidth);
  frame.renders.isSilhouette.resize(m_imageHeight, m_imageWidth);

  float clipNear, clipFar;
  m_renderer.computeClipping(clipNear, clipFar);
  frame.renders.zNear = std::max(0.001f, clipNear);
  frame.renders.zFar = clipFar;
  m_rendererSettings.setClippingDistance(frame.renders.zNear, frame.renders.zFar);
  m_renderer.setRenderParameters(m_rendererSettings);

  bool shouldRenderSilhouette = m_renderer.getRenderer<vpPanda3DDepthCannyFilter>() != nullptr;
  if (shouldRenderSilhouette) {
    // For silhouette extraction, update depth difference threshold
    double thresholdValue = m_depthSilhouetteSettings.getThreshold();
    if (m_depthSilhouetteSettings.thresholdIsRelative()) {
      m_renderer.getRenderer<vpPanda3DDepthCannyFilter>()->setEdgeThreshold((frame.renders.zFar - frame.renders.zNear) * thresholdValue);
    }
    else {
      m_renderer.getRenderer<vpPanda3DDepthCannyFilter>()->setEdgeThreshold(thresholdValue);
    }
  }

  // Call Panda renderer
  m_renderer.renderFrame();

  frame.renders.boundingBox = m_renderer.getBoundingBox();

  // Extract data from Panda textures
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel sections
#endif

  {
#ifdef VISP_HAVE_OPENMP
#pragma omp section
#endif
    {
      m_renderer.getRenderer<vpPanda3DGeometryRenderer>()->getRender(
        frame.renders.normals,
        frame.renders.depth,
        frame.renders.boundingBox,
        m_imageHeight, m_imageWidth);
    }
#ifdef VISP_HAVE_OPENMP
#pragma omp section
#endif
    {
      if (shouldRenderSilhouette) {
        m_renderer.getRenderer<vpPanda3DDepthCannyFilter>()->getRender(
          frame.renders.silhouetteCanny,
          frame.renders.isSilhouette,
          frame.renders.boundingBox,
          m_imageHeight, m_imageWidth);
      }
    }
    // #pragma omp section
    //     {
    //       vpImage<vpRGBa> renders.color;
    //       m_renderer.getRenderer<vpPanda3DRGBRenderer>()->getRender(renders.color);
    //       m_renderer.placeRendernto(renders.color, frame.renders.color, vpRGBa(0));
    //     }
  }

}
std::vector<vpRBSilhouettePoint> vpRBTracker::extractSilhouettePoints(
  const vpImage<vpRGBf> &Inorm, const vpImage<float> &Idepth,
  const vpImage<vpRGBf> &silhouetteCanny, const vpImage<unsigned char> &Ivalid,
  const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp)
{
  std::vector<std::pair<unsigned int, unsigned int>> candidates =
    m_depthSilhouetteSettings.getSilhouetteCandidates(Ivalid, Idepth, cam, cTcp, m_previousFrame.silhouettePoints, 42);

  std::vector<vpRBSilhouettePoint> points;
  vpColVector norm(3);

  for (unsigned int i = 0; i < candidates.size(); ++i) {
    unsigned int n = candidates[i].first, m = candidates[i].second;
    double theta = silhouetteCanny[n][m].B;
    if (std::isnan(theta)) {
      continue;
    }

    norm[0] = Inorm[n][m].R;
    norm[1] = Inorm[n][m].G;
    norm[2] = Inorm[n][m].B;
    const double l = std::sqrt(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]);
    if (l > 1e-1) {
      norm.normalize();
      const double Z = Idepth[n][m];
#if defined(VISP_DEBUG_RB_TRACKER)
      if (fabs(theta) > M_PI + 1e-6) {
        throw vpException(vpException::badValue, "Theta expected to be in -Pi, Pi range but was not");
      }
#endif
      vpRBSilhouettePoint p(n, m, norm, theta, Z);
      p.detectSilhouette(Idepth);
      points.push_back(p);
    }
  }
  return points;
}

void vpRBTracker::addTracker(std::shared_ptr<vpRBFeatureTracker> tracker)
{
  if (tracker == nullptr) {
    throw vpException(vpException::badValue, "Adding tracker: tracker cannot be null");
  }
  m_trackers.push_back(tracker);
}

void vpRBTracker::displayMask(vpImage<unsigned char> &Imask) const
{
  if (m_mask) {
    m_mask->display(m_currentFrame.mask, Imask);
  }
}

void vpRBTracker::display(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth)
{
  if (m_currentFrame.renders.normals.getSize() == 0) {
    return;
  }

  if (m_displaySilhouette && m_currentFrame.silhouettePoints.size() > 0) {
    const vpImage<unsigned char> &Isilhouette = m_currentFrame.renders.isSilhouette;
    const vpRect bb = m_renderer.getBoundingBox();
    for (unsigned int r = std::max(bb.getTop(), 0.); (r < bb.getBottom()) &&(r < IRGB.getRows()); ++r) {
      for (unsigned int c = std::max(bb.getLeft(), 0.); (c < bb.getRight()) && (c < IRGB.getCols()); ++c) {
        if (Isilhouette[r][c] != 0) {
          vpDisplay::displayPoint(IRGB, vpImagePoint(r, c), vpColor::green);
        }
      }
    }
  }

  for (std::shared_ptr<vpRBFeatureTracker> &tracker : m_trackers) {
    if (tracker->featuresShouldBeDisplayed()) {
      tracker->display(m_currentFrame.cam, I, IRGB, depth);
    }
  }
}

vpObjectCentricRenderer &vpRBTracker::getRenderer()
{
  return m_renderer;
}

#if defined(VISP_HAVE_NLOHMANN_JSON)
void vpRBTracker::loadConfigurationFile(const std::string &filename)
{
  std::ifstream jsonFile(filename);
  if (!jsonFile.good()) {
    throw vpException(vpException::ioError, "Could not read from settings file " + filename + " to initialize the RBTracker");
  }
  nlohmann::json settings;
  try {
    settings = nlohmann::json::parse(jsonFile);
  }
  catch (nlohmann::json::parse_error &e) {
    std::stringstream msg;
    msg << "Could not parse JSON file : \n";
    msg << e.what() << std::endl;
    msg << "Byte position of error: " << e.byte;
    throw vpException(vpException::ioError, msg.str());
  }
  loadJsonConfiguration(settings);
  jsonFile.close();
}

void vpRBTracker::loadJsonConfiguration(const nlohmann::json &j)
{
  verify(j);
  m_firstIteration = true;
  const nlohmann::json verboseSettings = j.at("verbose");
  m_verbose = verboseSettings.value("enabled", m_verbose);

  m_displaySilhouette = j.value("displaySilhouette", m_displaySilhouette);

  const nlohmann::json cameraSettings = j.at("camera");
  m_cam = cameraSettings.at("intrinsics");
  m_imageHeight = cameraSettings.value("height", m_imageHeight);
  m_imageWidth = cameraSettings.value("width", m_imageWidth);
  setCameraParameters(m_cam, m_imageHeight, m_imageWidth);

  if (j.contains("model")) {
    setModelPath(j.at("model"));
  }

  const nlohmann::json vvsSettings = j.at("vvs");
  setMaxOptimizationIters(vvsSettings.value("maxIterations", m_vvsIterations));
  setOptimizationGain(vvsSettings.value("gain", m_lambda));
  setOptimizationInitialMu(vvsSettings.value("mu", m_muInit));
  setOptimizationMuIterFactor(vvsSettings.value("muIterFactor", m_muIterFactor));
  setOptimizationMuIterFactor(vvsSettings.value("scaleInvariant", m_scaleInvariantOptim));
  m_convergedMetricThreshold = (vvsSettings.value("convergenceMetricThreshold", m_convergedMetricThreshold));

  m_depthSilhouetteSettings = j.at("silhouetteExtractionSettings");

  m_trackers.clear();
  const nlohmann::json features = j.at("features");
  vpRBFeatureTrackerFactory &featureFactory = vpRBFeatureTrackerFactory::getFactory();
  for (const nlohmann::json &trackerSettings: features) {
    std::shared_ptr<vpRBFeatureTracker> tracker = featureFactory.buildFromJson(trackerSettings);
    if (tracker == nullptr) {
      throw vpException(
        vpException::badValue,
        "Cannot instantiate subtracker with the current settings, make sure that the type is registered. Settings: %s",
        trackerSettings.dump(2).c_str()
      );
    }
    m_trackers.push_back(tracker);
  }

  if (j.contains("mask")) {
    vpObjectMaskFactory &maskFactory = vpObjectMaskFactory::getFactory();
    const nlohmann::json maskSettings = j.at("mask");
    m_mask = maskFactory.buildFromJson(maskSettings);
    if (m_mask == nullptr) {
      throw vpException(
        vpException::badValue,
        "Cannot instantiate object mask with the current settings, make sure that the type is registered. Settings: %s",
        maskSettings.dump(2).c_str());
    }
  }
  if (j.contains("drift")) {
    vpRBDriftDetectorFactory &factory = vpRBDriftDetectorFactory::getFactory();
    const nlohmann::json driftSettings = j.at("drift");
    m_driftDetector = factory.buildFromJson(driftSettings);
    if (m_driftDetector == nullptr) {
      throw vpException(
        vpException::badValue,
       "Cannot instantiate drift detection with the current settings, make sure that the type is registered in the factory"
      );
    }
  }
}


nlohmann::json vpRBTracker::explain() const
{
  std::vector<nlohmann::json> optimParameters = {
    vpRBJsonParsable::parameter(
      "gain", "The gain used during levenberg-marquardt minimization of the feature error", true, 1.0
    ),
    vpRBJsonParsable::parameter(
      "maxIterations", "The maximum number of iterations for the minimization process.", true, 10
    ),
    vpRBJsonParsable::parameter(
      "mu", "The initial value for the regularization parameter of the LM algorithm."
      "This value is multiplied by muIterfactor at every iteration."
      "A low value of mu leads to a Gauss Newton minimizer, while a higher value is closer to a gradient descent solution.", true, 0.1
    ),
    vpRBJsonParsable::parameter(
      "muIterFactor", "The factor with which to multiply the current regularization parameter value.", true, 0.1
    ),
    vpRBJsonParsable::parameter(
      "scaleInvariant", "Whether to use the identity matrix as a regularizer (value of false) or the diagonal of the Hessian approximation (true)", true, false
    ),
    vpRBJsonParsable::parameter(
      "convergenceMetricThreshold", "Minimum value of the convergence metric  to not stop optimization. "
      "If the metric is below this threshold, then optimization is stopped."
      "A value of 0 leads to running optimization for the full number of maxIterations.", false, 0.0
    ),
  };

  std::vector<nlohmann::json> baseParameters = {
    vpRBJsonParsable::parameter("displaySilhouette",
    "Whether to display the object silhouette on top of the color image."
    "This will work only if silhouette is extracted,"
    "and the displayed silhouette will be the one corresponding "
    "to the object pose estimated at the previous timestep", false, false),
    vpRBJsonParsable::parameter("model", "Path to the model of the object to track."
    "This parameter is not required and the model can be specified using a setter."
    "A model should be set before calling startTracking.", false, "path/to/object.obj")
  };


  nlohmann::json baseDict = flipToDict(baseParameters);
  baseDict["vvs"] = flipToDict(optimParameters);
  baseDict["silhouetteExtractionSettings"] = m_depthSilhouetteSettings.explain();


  return baseDict;
}


#endif

#ifdef VISP_HAVE_MODULE_GUI
void vpRBTracker::initClick(const vpImage<unsigned char> &I, const std::string &initFile, bool displayHelp)
{
  vpRBInitializationHelper initializer;
  initializer.setCameraParameters(m_cam);
  initializer.initClick(I, initFile, displayHelp);
  m_cMo = initializer.getPose();
}
#endif

END_VISP_NAMESPACE
