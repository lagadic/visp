/*
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
 */

/*!
 * \file vpMbKltTracker.h
 * \brief Model based tracker using only KLT
 */

#ifndef _vpMbKltTracker_h_
#define _vpMbKltTracker_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpSubColVector.h>
#include <visp3/core/vpSubMatrix.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtDistanceCircle.h>
#include <visp3/mbt/vpMbtDistanceKltCylinder.h>
#include <visp3/mbt/vpMbtDistanceKltPoints.h>
#include <visp3/vision/vpHomography.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMbKltTracker
 * \ingroup group_mbt_trackers
 * \warning This class is deprecated for user usage. You should rather use the high level
 * vpMbGenericTracker class.
 * \warning This class is only available if OpenCV is installed, and used.
 *
 * \brief Model based tracker using only KLT.
 *
 * The \ref tutorial-tracking-mb-deprecated is a good starting point to use this class.
 *
 * The tracker requires the knowledge of the 3D model that could be provided in
 * a vrml or in a cao file. The cao format is described in loadCAOModel(). It may
 * also use an xml file used to tune the behavior of the tracker and an init file
 * used to compute the pose at the very first image.
 *
 * The following code shows the simplest way to use the tracker. The \ref
 * tutorial-tracking-mb-deprecated is also a good starting point to use this class.
 *
 * \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpHomogeneousMatrix.h>
 * #include <visp3/core/vpImage.h>
 * #include <visp3/gui/vpDisplayX.h>
 * #include <visp3/io/vpImageIo.h>
 * #include <visp3/mbt/vpMbKltTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined VISP_HAVE_OPENCV
 *   vpMbKltTracker tracker; // Create a model based tracker via KLT points.
 *   vpImage<unsigned char> I;
 *   vpHomogeneousMatrix cMo; // Pose computed using the tracker.
 *   vpCameraParameters cam;
 *
 *   // Acquire an image
 *   vpImageIo::read(I, "cube.pgm");
 *
 * #if defined(VISP_HAVE_X11)
 *   vpDisplayX display;
 *   display.init(I,100,100,"Mb Klt Tracker");
 * #endif
 *
 *   tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
 *   tracker.getCameraParameters(cam);   // Get the camera parameters used by the tracker (from the configuration file).
 *   tracker.loadModel("cube.cao");      // Load the 3d model in cao format. No 3rd party library is required
 *   // Initialise manually the pose by clicking on the image points associated to the 3d points contained in the
 *   // cube.init file.
 *   tracker.initClick(I, "cube.init");
 *
 *   while(true){
 *     // Acquire a new image
 *     vpDisplay::display(I);
 *     tracker.track(I);     // Track the object on this image
 *     tracker.getPose(cMo); // Get the pose
 *
 *     tracker.display(I, cMo, cam, vpColor::darkRed, 1); // Display the model at the computed pose.
 *     vpDisplay::flush(I);
 *   }
 *
 *   return 0;
 * #endif
 * }
 * \endcode
 *
 * The tracker can also be used without display, in that case the initial pose
 * must be known (object always at the same initial pose for example) or
 * computed using another method:
 *
 * \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpHomogeneousMatrix.h>
 * #include <visp3/core/vpImage.h>
 * #include <visp3/io/vpImageIo.h>
 * #include <visp3/mbt/vpMbKltTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined VISP_HAVE_OPENCV
 *   vpMbKltTracker tracker; // Create a model based tracker via Klt Points.
 *   vpImage<unsigned char> I;
 *   vpHomogeneousMatrix cMo; // Pose used in entry (has to be defined), then computed using the tracker.
 *
 *   //acquire an image
 *   vpImageIo::read(I, "cube.pgm"); // Example of acquisition
 *
 *   tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
 *   // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.
 *   tracker.loadModel("cube.cao");
 *   tracker.initFromPose(I, cMo); // initialize the tracker with the given pose.
 *
 *   while(true){
 *     // acquire a new image
 *     tracker.track(I); // track the object on this image
 *     tracker.getPose(cMo); // get the pose
 *   }
 *
 *   return 0;
 * #endif
 * }
 * \endcode
 *
 * Finally it can be used not to track an object but just to display a model at
 * a given pose:
 *
 * \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpHomogeneousMatrix.h>
 * #include <visp3/core/vpImage.h>
 * #include <visp3/gui/vpDisplayX.h>
 * #include <visp3/io/vpImageIo.h>
 * #include <visp3/mbt/vpMbKltTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined VISP_HAVE_OPENCV
 *   vpMbKltTracker tracker; // Create a model based tracker via Klt Points.
 *   vpImage<unsigned char> I;
 *   vpHomogeneousMatrix cMo; // Pose used to display the model.
 *   vpCameraParameters cam;
 *
 *   // Acquire an image
 *   vpImageIo::read(I, "cube.pgm");
 *
 * #if defined(VISP_HAVE_X11)
 *   vpDisplayX display;
 *   display.init(I,100,100,"Mb Klt Tracker");
 * #endif
 *
 *   tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
 *   tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
 *   // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.
 *   tracker.loadModel("cube.cao");
 *
 *   while(true){
 *     // acquire a new image
 *     // Get the pose using any method
 *     vpDisplay::display(I);
 *     tracker.display(I, cMo, cam, vpColor::darkRed, 1, true); // Display the model at the computed pose.
 *     vpDisplay::flush(I);
 *   }
 *
 *   return 0;
 * #endif
 * }
 * \endcode
*/
class VISP_EXPORT vpMbKltTracker : public virtual vpMbTracker
{
protected:
  //! Temporary OpenCV image for fast conversion.
  cv::Mat cur;
  //! Initial pose.
  vpHomogeneousMatrix c0Mo;
  //! Flag to specify whether the init method is called the first or not
  //! (specific calls to realize in this case).
  bool firstInitialisation;
  //! Erosion of the mask
  unsigned int maskBorder;
  //! Threshold below which the weight associated to a point to consider this
  //! one as an outlier.
  double threshold_outlier;
  //! Percentage of good points, according to the initial number, that must
  //! have the tracker.
  double percentGood;
  //! The estimated displacement of the pose between the current instant and
  //! the initial position.
  vpHomogeneousMatrix ctTc0;
  //! Points tracker.
  vpKltOpencv tracker;
  //!
  std::list<vpMbtDistanceKltPoints *> kltPolygons;
  //!
  std::list<vpMbtDistanceKltCylinder *> kltCylinders;
  //! Vector of the circles used here only to display the full model.
  std::list<vpMbtDistanceCircle *> circles_disp;
  //!
  unsigned int m_nbInfos;
  //!
  unsigned int m_nbFaceUsed;
  //! Interaction matrix
  vpMatrix m_L_klt;
  //! (s - s*)
  vpColVector m_error_klt;
  //! Robust weights
  vpColVector m_w_klt;
  //! Weighted error
  vpColVector m_weightedError_klt;
  //! Robust
  vpRobust m_robust_klt;
  //! Display features
  std::vector<std::vector<double> > m_featuresToBeDisplayedKlt;

public:
  vpMbKltTracker();
  virtual ~vpMbKltTracker();

  /** @name Inherited functionalities from vpMbKltTracker */
  //@{

  void addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, double r, const std::string &name = "");
  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;
  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  /*! Return the address of the circle feature list. */
  virtual std::list<vpMbtDistanceCircle *> &getFeaturesCircle() { return circles_disp; }
  /*! Return the address of the cylinder feature list. */
  virtual std::list<vpMbtDistanceKltCylinder *> &getFeaturesKltCylinder() { return kltCylinders; }
  /*! Return the address of the Klt feature list. */
  virtual std::list<vpMbtDistanceKltPoints *> &getFeaturesKlt() { return kltPolygons; }

  /*!
   * Get the current list of KLT points.
   *
   * \return the list of KLT points through vpKltOpencv.
   */
  inline std::vector<cv::Point2f> getKltPoints() const { return tracker.getFeatures(); }

  std::vector<vpImagePoint> getKltImagePoints() const;

  std::map<int, vpImagePoint> getKltImagePointsWithId() const;

  /*!
   * Get the klt tracker at the current state.
   *
   * \return klt tracker.
   */
  inline vpKltOpencv getKltOpencv() const { return tracker; }

  /*!
   * Get the erosion of the mask used on the Model faces.
   *
   * \return The erosion.
   */
  inline unsigned int getKltMaskBorder() const { return maskBorder; }

  /*!
   * Get the current number of klt points.
   *
   * \return the number of features
   */
  inline int getKltNbPoints() const { return tracker.getNbFeatures(); }

  /*!
   * Get the threshold for the acceptation of a point.
   *
   * \return threshold_outlier : Threshold for the weight below which a point
   * is rejected.
   */
  inline double getKltThresholdAcceptation() const { return threshold_outlier; }

  virtual inline vpColVector getError() const VP_OVERRIDE { return m_error_klt; }

  virtual inline vpColVector getRobustWeights() const VP_OVERRIDE { return m_w_klt; }

  virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
                                                               const vpHomogeneousMatrix &cMo,
                                                               const vpCameraParameters &cam,
                                                               bool displayFullModel = false) VP_OVERRIDE;

  virtual void loadConfigFile(const std::string &configFile, bool verbose = true) VP_OVERRIDE;

  virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo,
                           bool verbose = false, const vpHomogeneousMatrix &T = vpHomogeneousMatrix());
  void resetTracker() VP_OVERRIDE;

  void setCameraParameters(const vpCameraParameters &cam) VP_OVERRIDE;

  /*!
   * Set the erosion of the mask used on the Model faces.
   *
   * \param  e : The desired erosion.
   */
  inline void setKltMaskBorder(const unsigned int &e)
  {
    maskBorder = e;
    // if(useScanLine)
    faces.getMbScanLineRenderer().setMaskBorder(maskBorder);
  }

  virtual void setKltOpencv(const vpKltOpencv &t);

  /*!
   * Set the threshold for the acceptation of a point.
   *
   * \param th : Threshold for the weight below which a point is rejected.
   */
  inline void setKltThresholdAcceptation(double th) { threshold_outlier = th; }

  /*!
   * Use Ogre3D for visibility tests
   *
   * \warning This function has to be called before the initialization of the
   * tracker.
   *
   * \param v : True to use it, False otherwise
   */
  virtual void setOgreVisibilityTest(const bool &v) VP_OVERRIDE
  {
    vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
    faces.getOgreContext()->setWindowName("MBT Klt");
#endif
  }

  /*!
   * Use Scanline algorithm for visibility tests
   *
   * \param v : True to use it, False otherwise
   */
  virtual void setScanLineVisibilityTest(const bool &v) VP_OVERRIDE
  {
    vpMbTracker::setScanLineVisibilityTest(v);

    for (std::list<vpMbtDistanceKltPoints *>::const_iterator it = kltPolygons.begin(); it != kltPolygons.end(); ++it)
      (*it)->useScanLine = v;
  }

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
  virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;

  /*!
   * Set if the projection error criteria has to be computed.
   *
   * \param flag : True if the projection error criteria has to be computed,
   * false otherwise
   */
  virtual void setProjectionErrorComputation(const bool &flag) VP_OVERRIDE
  {
    if (flag)
      std::cerr << "This option is not yet implemented in vpMbKltTracker, "
      "projection error computation set to false."
      << std::endl;
  }

  void setUseKltTracking(const std::string &name, const bool &useKltTracking);

  virtual void testTracking() VP_OVERRIDE;
  virtual void track(const vpImage<unsigned char> &I) VP_OVERRIDE;
  virtual void track(const vpImage<vpRGBa> &I_color) VP_OVERRIDE;

  /*!
    @name Deprecated functions
  */
  //@{

  /*!
   * Get the erosion of the mask used on the Model faces.
   * \deprecated Use rather getkltMaskBorder()
   *
   * \return The erosion.
   */
  /* VP_DEPRECATED */ inline unsigned int getMaskBorder() const { return maskBorder; }

  /*!
   * Get the current number of klt points.
   * \deprecated Use rather getKltNbPoints()
   *
   * \return the number of features
   */
  /* VP_DEPRECATED */ inline int getNbKltPoints() const { return tracker.getNbFeatures(); }

  /*!
   * Get the threshold for the acceptation of a point.
   * \deprecated Use rather getKltThresholdAcceptation()
   *
   * \return threshold_outlier : Threshold for the weight below which a point
   * is rejected.
   */
  /* VP_DEPRECATED */ inline double getThresholdAcceptation() const { return threshold_outlier; }

  /*!
   * Set the erosion of the mask used on the Model faces.
   *
   * \param  e : The desired erosion.
   */
  /* VP_DEPRECATED */ inline void setMaskBorder(const unsigned int &e)
  {
    maskBorder = e;
    // if(useScanLine)
    faces.getMbScanLineRenderer().setMaskBorder(maskBorder);
  }

  /*!
   * Set the threshold for the acceptation of a point.
   * \deprecated Use rather setKltThresholdAcceptation()
   *
   * \param th : Threshold for the weight below which a point is rejected.
   */
  /* VP_DEPRECATED */ inline void setThresholdAcceptation(double th) { threshold_outlier = th; }

  //@}

protected:
  /** @name Protected Member Functions Inherited from vpMbKltTracker */
  //@{
  void computeVVS();
  virtual void computeVVSInit() VP_OVERRIDE;
  virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE;

  virtual std::vector<std::vector<double> > getFeaturesForDisplayKlt();

  virtual void init(const vpImage<unsigned char> &I) VP_OVERRIDE;
  virtual void initFaceFromCorners(vpMbtPolygon &polygon) VP_OVERRIDE;
  virtual void initFaceFromLines(vpMbtPolygon &polygon) VP_OVERRIDE;
  virtual void initCircle(const vpPoint &, const vpPoint &, const vpPoint &, double, int, const std::string &name = "") VP_OVERRIDE;
  virtual void initCylinder(const vpPoint &, const vpPoint &, double, int, const std::string &name = "") VP_OVERRIDE;

  void preTracking(const vpImage<unsigned char> &I);
  bool postTracking(const vpImage<unsigned char> &I, vpColVector &w);
  virtual void reinit(const vpImage<unsigned char> &I);
  virtual void setPose(const vpImage<unsigned char> *I, const vpImage<vpRGBa> *I_color,
                       const vpHomogeneousMatrix &cdMo);
  //@}
};
END_VISP_NAMESPACE
#endif
#endif // VISP_HAVE_OPENCV
