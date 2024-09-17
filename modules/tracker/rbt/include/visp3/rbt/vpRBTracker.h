/****************************************************************************
 *
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
 *
*****************************************************************************/

/*!
  \file vpRBTracker.h
  \brief Render-Based Tracker
*/
#ifndef VP_RB_TRACKER_H
#define VP_RB_TRACKER_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PANDA3D)

#include <visp3/mbt/vpMbTracker.h>

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/rbt/vpRBSilhouettePointsExtractionSettings.h>
#include <visp3/rbt/vpPanda3DDepthFilters.h>
#include <visp3/rbt/vpObjectCentricRenderer.h>
#include <visp3/rbt/vpRBTrackerLogger.h>

class vpObjectMask;
class vpRBDriftDetector;

#include <ostream>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json_fwd.hpp>
#endif

// #include <visp3/core/vpUnscentedKalmanPose.h>
// class VISP_EXPORT vpRBTrackerFilter
// {
// public:
//   vpRBTrackerFilter() : m_initialized(false)
//   {
//     double opt_stdevP0 = 1e-8;
//     double opt_stdevQ = 1e-10;
//     double opt_stdevR = 1e-6;
//     vpMatrix Id;
//     Id.eye(6);
//     vpMatrix P0 = Id * opt_stdevP0 * opt_stdevP0;
//     vpMatrix Q = Id * opt_stdevQ * opt_stdevQ;
//     vpMatrix R = Id * opt_stdevR * opt_stdevR;
//     double alphaPred = 0.01;
//     vpUnscentedKalmanPose::State X0;
//     Id.eye(3);
//     vpMatrix R_ukfm = Id * opt_stdevR * opt_stdevR;
//     vpUnscentedKalmanPose ukfm(Q, R_ukfm, std::vector<double>(3, alphaPred), X0, P0,
//                                           vpUnscentedKalmanPose::fSE3, vpUnscentedKalmanPose::hSE3, vpUnscentedKalmanPose::phiSE3,
//                                           vpUnscentedKalmanPose::phiinvSE3);
//     m_kalman = std::shared_ptr<vpUnscentedKalmanPose>(new vpUnscentedKalmanPose(ukfm));
//   }

//   void filter(const vpHomogeneousMatrix &cMo, double dt)
//   {
//     if (!m_initialized) {
//       m_kalman->setX0(cMo);
//       m_cMoPrev = cMo;
//       m_initialized = true;
//     }
//     else {
//       std::ios_base::fmtflags f(std::cerr.flags());

//       vpColVector v = vpExponentialMap::inverse(m_cMoPrev * cMo.inverse(), dt);
//       std::cerr << "Kalman v = " << std::setprecision(4) << std::scientific << v.t() << std::endl;
//       std::cerr.flags(f);
//       m_kalman->filter(v, vpUnscentedKalmanPose::asPositionVector(cMo), dt);
//       m_cMoPrev = cMo;
//     }
//   }

//   void reinit(const vpHomogeneousMatrix &cMo)
//   {
//     m_kalman->setX0(cMo);
//     m_cMoPrev = cMo;
//     m_initialized = true;
//   }

//   vpHomogeneousMatrix getFilteredPose()
//   {
//     return m_kalman->getState();
//   }



// private:
//   std::shared_ptr<vpUnscentedKalmanPose> m_kalman;
//   vpHomogeneousMatrix m_cMoPrev;
//   bool m_initialized;

// };





class VISP_EXPORT vpRBTracker : public vpMbTracker
{
public:

  vpRBTracker();

  ~vpRBTracker() = default;


  void getPose(vpHomogeneousMatrix &cMo) const;
  void setPose(const vpHomogeneousMatrix &cMo);

  vpCameraParameters getCameraParameters() const;
  void setCameraParameters(const vpCameraParameters &cam, unsigned h, unsigned w);
  void setSilhouetteExtractionParameters(const vpSilhouettePointsExtractionSettings &settings);

  void reset();

  void loadObjectModel(const std::string &file);

  void track(const vpImage<unsigned char> &I);
  void track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB);
  void track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<float> &depth);

  void addTracker(std::shared_ptr<vpRBFeatureTracker> tracker);

  void displayMask(vpImage<unsigned char> &Imask) const;
  void display(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth, const vpRBFeatureDisplayType type);

  vpObjectCentricRenderer &getRenderer();
  const vpRBFeatureTrackerInput &getMostRecentFrame() const { return m_currentFrame; }

  const std::shared_ptr<const vpRBDriftDetector> getDriftDetector() const { return m_driftDetector; }


  //vpRBTrackerFilter &getFilter() { return m_filter; }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadConfigurationFile(const std::string &filename);
  void loadConfiguration(const nlohmann::json &j);
#endif

  virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, double radius, int idFace = 0,
                          const std::string &name = "") VP_OVERRIDE
  { }
  virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, double radius, int idFace = 0,
                            const std::string &name = "") VP_OVERRIDE
  { }
  virtual void initFaceFromCorners(vpMbtPolygon &polygon) VP_OVERRIDE { }
  virtual void initFaceFromLines(vpMbtPolygon &polygon) VP_OVERRIDE { }
  virtual vpColVector getError() const VP_OVERRIDE { return vpColVector(); }
  virtual vpColVector getRobustWeights() const { return vpColVector(); }
  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE
  { }
  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE
  { }

  virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
                                                               const vpHomogeneousMatrix &cMo,
                                                               const vpCameraParameters &cam,
                                                               bool displayFullModel = false)
  {
    return {};
  }

  virtual void init(const vpImage<unsigned char> &I) VP_OVERRIDE { }
  virtual void resetTracker() VP_OVERRIDE { }
  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE { }
  virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE { }
  virtual void testTracking() VP_OVERRIDE { }
  virtual void track(const vpImage<vpRGBa> &I) VP_OVERRIDE { }
  virtual void computeVVSInit() VP_OVERRIDE { }
  virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE { }
protected:

  void track(vpRBFeatureTrackerInput &input);
  void updateRender(vpRBFeatureTrackerInput &frame);


  std::vector<vpRBSilhouettePoint> extractSilhouettePoints(const vpImage<vpRGBf> &Inorm, const vpImage<float> &Idepth,
   const vpImage<vpRGBf> &Ior, const vpImage<unsigned char> &Ivalid, const vpCameraParameters &cam, const vpHomogeneousMatrix &cTcp);

  vpMatrix getCovariance()
  {
    throw vpException(vpException::notImplementedError);
  }

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


  bool m_firstIteration; //! Whether this is the first iteration

  std::vector<std::shared_ptr<vpRBFeatureTracker>> m_trackers; //! List of trackers

  // vpHomogeneousMatrix m_cMo;
  vpHomogeneousMatrix m_cMoPrev;
  //vpCameraParameters m_cam;

  vpRBFeatureTrackerInput m_currentFrame;
  vpRBFeatureTrackerInput m_previousFrame;


  double m_lambda; //! VVS gain
  unsigned m_vvsIterations; //! Max number of VVS iterations
  double m_muInit; //! Initial mu value for Levenberg-Marquardt
  double m_muIterFactor; //! Factor with which to multiply mu at every iteration during VVS.

  vpSilhouettePointsExtractionSettings m_depthSilhouetteSettings;

  vpPanda3DRenderParameters m_rendererSettings;
  vpObjectCentricRenderer m_renderer;
  //vpRenderer m_renderer;

  unsigned m_imageHeight, m_imageWidth; //! Color and render image dimensions

  vpRBTrackerLogger m_logger;

  std::shared_ptr<vpObjectMask> m_mask;

  std::shared_ptr<vpRBDriftDetector> m_driftDetector;

  // vpRBTrackerFilter m_filter;

  vpRBFeatureTrackerInput m_tempRenders;

};

#endif
#endif
