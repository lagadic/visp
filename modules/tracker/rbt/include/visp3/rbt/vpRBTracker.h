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

/*!
  \file vpRBTracker.h
  \brief Render-Based Tracker
*/
#ifndef VP_RB_TRACKER_H
#define VP_RB_TRACKER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/rbt/vpRBSilhouettePointsExtractionSettings.h>
#include <visp3/rbt/vpPanda3DDepthFilters.h>
#include <visp3/rbt/vpObjectCentricRenderer.h>
#include <visp3/rbt/vpRBTrackingResult.h>
#include <visp3/rbt/vpRBADDSMetric.h>
#include <visp3/rbt/vpRBInitializationHelper.h>
#include <visp3/core/vpDisplay.h>

#include <ostream>
#include <type_traits>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE

class vpObjectMask;
class vpRBDriftDetector;
class vpRBVisualOdometry;

/**
 * \brief Class implementing the Render-Based Tracker (RBT).
 *
 * The RBT is an extension and rework of \cite Petit14a and of the Model-Based Tracker.
 *
 * Tracking is framed as a non-linear optimization problem where a visual error (formulated by comparing **renders**
 * with images) should be minimized.
 *
 * The RBT supports a various set of features, from tracked 2D points to dense depth information.
 * For a list of base features, see \ref group_rbt_trackers.
 *
 * The RBT can be extended and you can add your own features and other functionalities to the pipeline.
 *
 * \ingroup group_rbt_core
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> For a detailed description of the tracker, how to use it and extend it, see:</span><br>
 *
 * - \ref tutorial-tracking-rbt
*/
class VISP_EXPORT vpRBTracker
{
public:

  vpRBTracker();

  ~vpRBTracker() = default;

  /**
   * \name Information retrieval
   * @{
   */

  /**
   * \brief Get the estimated pose of the object in the camera frame.
   *
   * You should call vpRBTracker::track to update the retrieved pose
   *
   * \param cMo pose to update
  */
  void getPose(vpHomogeneousMatrix &cMo) const;

  /**
   * \brief Sets the pose of the object in the camera frame.
   * Should be called when initializing the tracker or when reinitializing after tracking failure.
   *
   * \param cMo the new object pose
  */
  void setPose(const vpHomogeneousMatrix &cMo);
  /**
   * \brief Get the renderer used to render the object.
   *
   * \return vpObjectCentricRenderer&
  */
  vpObjectCentricRenderer &getRenderer();
  /**
   * \brief Retrieve the most recent frame that was used when tracking the object.
   * The renders may not correspond to the latest pose that can be retrieved with getPose
   * To retrieve the render pose see vpRBRenderData::cMo
   *
   * \return const vpRBFeatureTrackerInput&
  */
  const vpRBFeatureTrackerInput &getMostRecentFrame() const { return m_currentFrame; }

  vpMatrix getCovariance() const;

  /**
   * @}
   */

  /**
   * \name Settings
   * @{
   */

  /**
   * \brief Add a new feature to track
   *
   * \param tracker the feature to add
   *
   * \throws vpException if the tracker is null
  */
  void addTracker(std::shared_ptr<vpRBFeatureTracker> tracker);

  /**
   * \brief Get the tracked features
   *
   * \return std::vector<std::shared_ptr<vpRBFeatureTracker>>
  */
  inline std::vector<std::shared_ptr<vpRBFeatureTracker>> getFeatureTrackers() const { return m_trackers; }

  /**
   * \brief Get the path to the 3D model to track
  */
  inline std::string getModelPath() const { return m_modelPath; }

  /**
   * \brief Set the path to the 3D model to load
   *
   * \param path a path to an existing file containing a 3D mesh that can be read by Panda3D
  */
  void setModelPath(const std::string &path);

  /**
   * \brief Get the camera intrinsics that are used to render the 3D object and process the tracked frames
   *
   * \return vpCameraParameters
  */
  vpCameraParameters getCameraParameters() const;
  /**
   * \brief Sets the camera intrinsics and image resolution for the images where the object will be tracked.
   *
   * \param cam Camera intrinsics for the color (and potential depth) image. It should follow a model without distortion.
   * \param h Image height
   * \param w Image width
   *
   * \throws vpException if camera intrinsics have distortion
   * \throws vpException if image resolution is incorrect
  */
  void setCameraParameters(const vpCameraParameters &cam, unsigned h, unsigned w);

  inline unsigned int getImageWidth() const { return m_imageWidth; }
  inline unsigned int getImageHeight() const { return m_imageHeight; }

  inline vpSilhouettePointsExtractionSettings getSilhouetteExtractionParameters() const
  {
    return m_depthSilhouetteSettings;
  }

  void setSilhouetteExtractionParameters(const vpSilhouettePointsExtractionSettings &settings);

  double getOptimizationGain() const { return m_lambda; }
  inline void setOptimizationGain(double lambda)
  {
    if (lambda < 0.0) {
      throw vpException(vpException::badValue, "Optimization gain should be greater or equal to zero");
    }
    m_lambda = lambda;
  }
  unsigned int getMaxOptimizationIters() const { return m_vvsIterations; }
  inline void setMaxOptimizationIters(unsigned int iters)
  {
    if (iters == 0) {
      throw vpException(vpException::badValue, "Max number of iterations must be greater than zero");
    }
    m_vvsIterations = iters;
  }

  double getOptimizationInitialMu() const { return m_muInit; }
  inline void setOptimizationInitialMu(double mu)
  {
    if (mu < 0.0) {
      throw vpException(vpException::badValue, "Optimization mu should be greater than or equal to zero");
    }
    m_muInit = mu;
  }

  double getOptimizationMuIterFactor() const { return m_muIterFactor; }
  inline void setOptimizationMuIterFactor(double factor)
  {
    if (factor < 0.0) {
      throw vpException(vpException::badValue, "Optimization mu factor should be greater than or equal to zero");
    }
    m_muIterFactor = factor;
  }

  bool scaleInvariantRegularization() const { return m_scaleInvariantOptim; }
  inline void setScaleInvariantRegularization(bool invariant)
  {
    m_scaleInvariantOptim = invariant;
  }

  /**
   * \see setDriftDetector
   *
  */
  std::shared_ptr<vpRBDriftDetector> getDriftDetector() const { return m_driftDetector; }
  /**
   * \brief Sets the method to perform drift detection and estimate tracking confidence.
   * Set to null to disable this functionality.
   *
   * \param detector the algorithm to use
  */
  inline void setDriftDetector(const std::shared_ptr<vpRBDriftDetector> &detector)
  {
    m_driftDetector = detector;
  }

  /**
   * \see setObjectSegmentationMethod
  */
  std::shared_ptr<vpObjectMask> getObjectSegmentationMethod() const { return m_mask; }
  /**
   * \brief Sets the algorithm to use when performing object segmentation.
   * Set to null to disable the use of segmentation.
   *
   * \param mask the segmentation method
  */
  inline void setObjectSegmentationMethod(const std::shared_ptr<vpObjectMask> &mask)
  {
    m_mask = mask;
  }

  /**
   * \see setOdometryMethod
  */
  std::shared_ptr<vpRBVisualOdometry> getOdometryMethod() const { return m_odometry; }
  /**
   * \brief Set the method to use when performing visual odometry to preestimate the camera motion before tracking.
   * If null, then odometry is disabled.
   *
   * \param odometry
  */
  inline void setOdometryMethod(const std::shared_ptr<vpRBVisualOdometry> &odometry)
  {
    m_odometry = odometry;
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  /**
   * \brief Update tracker settings from a .json file.
   * See the tutorials for a full description of the json format
   *
   * \param filename The path to the .json file to load
   *
   * \throws if the file is ill-formed or settings are incorrect
  */
  void loadConfigurationFile(const std::string &filename);

  /**
   * \brief Update the tracker with a new json configuration
   *
   * \param j the full json configuration
   * \throws if the parameters are incorrect
  */
  void loadConfiguration(const nlohmann::json &j);
#endif

  /**
   * @}
   */

  void reset();

  /**
   * \name Tracking
   * @{
   */


  /**
   * \brief Method that should be called before starting tracking
   *
   * Initializes the renderer, loads the 3D model and ensures that the parameters are correct
   *
  */
  void startTracking();
  /**
   * \brief track and re-estimate the pose of the object in this frame, given only a grayscale image
   * The pose after tracking should be retrieved using getPose.
   * You should use the returned vpRBTrackingResult to check that tracking was successful
   *
   * \param I Grayscale image
   * \return vpRBTrackingResult
   *
   * \throws if Image dimensions are incorrect or if one of the used features requires additional data, such as color or depth information
  */
  vpRBTrackingResult track(const vpImage<unsigned char> &I);
  /**
   * \brief track and re-estimate the pose of the object in this frame, given only a grayscale image
   * The pose after tracking should be retrieved using getPose.
   * You should use the returned vpRBTrackingResult to check that tracking was successful
   *
   * \param I Grayscale image
   * \param IRGB Grayscale image
   *
   * \return vpRBTrackingResult
   *
   * \throws if Image dimensions are incorrect or if one of the used features requires additional data, such as depth information
  */
  vpRBTrackingResult track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB);
  /**
   * \brief track and re-estimate the pose of the object in this frame, given only a grayscale image
   * The pose after tracking should be retrieved using getPose.
   * You should use the returned vpRBTrackingResult to check that tracking was successful
   *
   * \param I Grayscale image
   * \param IRGB Grayscale image
   * \param depth Depth image, in meters. The Depth image should be aligned with the color and grayscale images.
   *
   * \return vpRBTrackingResult
   *
   * \throws if Image dimensions are incorrect
  */
  vpRBTrackingResult track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth);
  /**
   * @}
   */

  /**
   * \brief Using the preconfigured drift detector, score a given pose by comparing the render with the given frame information.
   *
   * \param cMo the pose of the object in the camera frame to be scored
   * \param I Grayscale frame
   * \param IRGB Color image
   * \param depth Depth image
   * \return The tracking confidence score estimated by the drift detector.
   *
   * \throws if no vpRBDriftDetector is specified, then scoring cannot be performed.
  */
  double score(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth);

  /**
   * \name Display
   * @{
   */

  /**
   * \brief Convert the mask output by the object segmentation method to a displayable representation.
   * If no object segmentation method is set, then this method has no effect.
   * \param Imask the image into which to write the display representation of the mask.
   *
   * \see setObjectSegmentationMethod, display
  */
  void displayMask(vpImage<unsigned char> &Imask) const;

  /**
   * \brief Displays tracker information such as current pose, object silhouette and tracked features on display images.
   *
   *
   * \param I Grayscale display image
   * \param IRGB  Color display image
   * \param depth Depth display image
  */
  void display(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth);
  /**
   * @}
   */

#ifdef VISP_HAVE_MODULE_GUI
  /**
   * \brief Perform Pose initialization by asking the user to click on predefined 3D points. This method is similar to the one provided in the MBT.
   *
   * The 3D points (in the object frame) are defined in a .init file which should look like:
   *
   * \verbatim
   * 4 # Number of points
   * 0.0 0.0 0.0 # Point 1 (In object frame coordinates)
   * 0.0 0.0 0.1 # Point 2 (In object frame coordinates)
   * 0.0 0.1 0.1 # Point 3 (In object frame coordinates)
   * 0.1 0.1 0.1 # Point 4 (In object frame coordinates)
   * \endverbatim
   *
   * \tparam ImageType The pixel type for I, either unsigned char or vpRGBa
   * \param I the image where to click. It should be associated to a vpDisplay
   * \param initFile The path to the initialization file.
   * \param displayHelp Whether to display an image to help with initialization.
   * If true and the init file is "path/to/object.init", then the function will look for an image "path/to/object.{png, jpg, jpeg, ppm}".
  */
  template <typename ImageType>
  typename std::enable_if<std::is_same<ImageType, unsigned char>::value || std::is_same<ImageType, vpRGBa>::value, void >::type initClick(const vpImage<ImageType> &I, const std::string &initFile, bool displayHelp)
  {
    vpRBInitializationHelper initializer;
    initializer.setCameraParameters(m_cam);
    initializer.initClick(I, initFile, displayHelp, *this);
    setPose(initializer.getPose());
  }
#endif

  friend vpRBInitializationHelper;
protected:

  vpRBTrackingResult track(vpRBFeatureTrackerInput &input);
  /**
   * \brief Setup the renderer, and load the 3D model
   *
   * \param file path to the 3D model to load and track
  */
  void setupRenderer(const std::string &file);
  /**
   * \brief Update the render data with a render at the last tracked pose
   *
   * \param frame the frame to update
  */
  void updateRender(vpRBFeatureTrackerInput &frame);
  /**
   * \brief Update the frame data with renders at a given pose
   *
   * \param frame the frame to update
   * \param cMo the pose of the object at which to perform rendering
  */
  void updateRender(vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo);

  /**
   * \brief Display the object silhouette of the frame in I
   *
   *
   * \tparam T Pixel type of I
   * \param I The image where to display the object silhouette
   * \param frame Frame containing the rendered data. Does not have to be last pose or associated to the estimated pose of I.
  */
  template <typename T>
  void displaySilhouette(const vpImage<T> &I, const vpRBFeatureTrackerInput &frame)
  {
    if (shouldRenderSilhouette()) {
      const vpImage<unsigned char> &Isilhouette = frame.renders.isSilhouette;
      const vpRect bb = m_renderer.getBoundingBox();
      for (unsigned int r = std::max(bb.getTop(), 0.); (r < bb.getBottom()) &&(r < I.getRows()); ++r) {
        for (unsigned int c = std::max(bb.getLeft(), 0.); (c < bb.getRight()) && (c < I.getCols()); ++c) {
          if (Isilhouette[r][c] != 0) {
            vpDisplay::displayPoint(I, vpImagePoint(r, c), vpColor::green);
          }
        }
      }
    }
  }

  /**
   * \brief Converts from pixelwise object silhouette representation to an actionable list of silhouette points.
   *
   * \param Inorm Image containing normals in object frame
   * \param Idepth Render image containing depth, in meters
   * \param Ior Image containing the silhouette 2D edge orientation data
   * \param Ivalid Image contianing whether a pixel is a silhouette point
   * \param cam Camera intrinsics
   * \param cTcp Pose of the previous camera in the current camera frame.
   * Used when silhouette extraction settings try to reuse the same silhouette points in successive frames.
   * \return std::vector<vpRBSilhouettePoint>
  */
  std::vector<vpRBSilhouettePoint> extractSilhouettePoints(
    const vpImage<vpRGBf> &Inorm, const vpImage<float> &Idepth,
    const vpImage<vpRGBf> &Ior, const vpImage<unsigned char> &Ivalid,
    const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp);

  /**
   * \brief Check that a given image has the correct dimensions, previously specified with setCameraParameters or in the config file.
   *
   * \tparam T Pixel type
   * \param I Image to check
   * \param imgType Helper string to indicat which image type failed.
  */
  template<typename T>
  void checkDimensionsOrThrow(const vpImage<T> &I, const std::string &imgType) const
  {
    if (I.getRows() != m_imageHeight || I.getCols() != m_imageWidth) {
      std::stringstream ss;
      ss << "vpRBTracker: dimension error: expected " << imgType;
      ss << " image to have the following resolution " << m_imageWidth << " x " << m_imageHeight;
      ss << ", but got " << I.getCols() << " x " << I.getRows();
      throw vpException(vpException::dimensionError, ss.str());
    }
  }

  /**
   * \brief Returns whether the renderer should render the silhouette information.
  */
  bool shouldRenderSilhouette()
  {
    return m_renderer.getRenderer<vpPanda3DDepthCannyFilter>() != nullptr;
  }
  //! Whether this is the first iteration
  bool m_firstIteration;
  //! List of feature trackers
  std::vector<std::shared_ptr<vpRBFeatureTracker>> m_trackers;

  vpRBFeatureTrackerInput m_currentFrame;
  vpRBFeatureTrackerInput m_previousFrame;

  //! Location of the 3D model to load
  std::string m_modelPath;

  //! Current pose of the object in the camera frame
  vpHomogeneousMatrix m_cMo;
  //! Previous pose of the object in the camera frame
  vpHomogeneousMatrix m_cMoPrev;
  //! Camera intrinsics
  vpCameraParameters m_cam;

  //! Optimization gain
  double m_lambda;
  //! Maximum number of optimization iterations
  unsigned m_vvsIterations;
  //! Initial mu value for Levenberg-Marquardt
  double m_muInit;
  //! Factor with which to multiply mu at every iteration during optimization.
  double m_muIterFactor;
  //! Whether to use diagonal scaling in Levenberg-Marquardt regularization
  bool m_scaleInvariantOptim;

  //! Settings for silhouette extraction
  vpSilhouettePointsExtractionSettings m_depthSilhouetteSettings;
   //! Camera specific setup for the 3D Panda renderer
  vpPanda3DRenderParameters m_rendererSettings;
  //! 3D renderer
  vpObjectCentricRenderer m_renderer;

  //! Color and render image dimensions
  unsigned m_imageHeight, m_imageWidth;

  std::shared_ptr<vpObjectMask> m_mask;
  std::shared_ptr<vpRBDriftDetector> m_driftDetector;
  std::shared_ptr<vpRBVisualOdometry> m_odometry;

  vpRBADDSMetric m_convergenceMetric; //! Metric used to compare the motion between different poses
  double m_convergedMetricThreshold; //! Metric threshold under which we consider that tracking optimization has converged to a correct pose
  double m_updateRenderThreshold; //! Metric threshold above which we consider that an object should be rerendered at the latest estimated pose.

  bool m_displaySilhouette; //! Whether a call to the display function should draw a silhouette outline

};

END_VISP_NAMESPACE

#endif
#endif
