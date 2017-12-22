/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 \file vpMbEdgeKltTracker.h
 \brief Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
*/

#ifndef vpMbEdgeKltTracker_HH
#define vpMbEdgeKltTracker_HH

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpSubColVector.h>
#include <visp3/core/vpSubMatrix.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbKltTracker.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtEdgeKltXmlParser.h>

/*!
  \class vpMbEdgeKltTracker
  \ingroup group_mbt_trackers
  \warning This class is only available if OpenCV is installed, and used.

  \brief Hybrid tracker based on moving-edges and keypoints tracked using KLT
  tracker.

  The \ref tutorial-tracking-mb is a good starting point to use this class.

  The tracker requires the knowledge of the 3D model that could be provided in
a vrml or in a cao file. The cao format is described in loadCAOModel(). It may
also use an xml file used to tune the behavior of the tracker and an init file
used to compute the pose at the very first image.

  The following code shows the simplest way to use the tracker. The \ref
tutorial-tracking-mb is also a good starting point to use this class.

\code
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpException.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeKltTracker.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbEdgeKltTracker tracker; // Create an hybrid model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose computed using the tracker.
  vpCameraParameters cam;

  // Acquire an image
  vpImageIo::read(I, "cube.pgm");

#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Hybrid Tracker");
#endif

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  // Load the 3d model in cao format. No 3rd party library is required
  tracker.loadModel("cube.cao");
  // Get the camera parameters used by the tracker (from the configuration file).
  tracker.getCameraParameters(cam);
  // Initialise manually the pose by clicking on the image points associated to the 3d points contained in the
  // cube.init file.
  tracker.initClick(I, "cube.init");

  while(true){
    // Acquire a new image
    vpDisplay::display(I);
    tracker.track(I);     // Track the object on this image
    tracker.getPose(cMo); // Get the pose

    tracker.display(I, cMo, cam, vpColor::darkRed, 1); // Display the model at the computed pose.
    vpDisplay::flush(I);
  }

#if defined VISP_HAVE_XML2
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode

  The tracker can also be used without display, in that case the initial pose
  must be known (object always at the same initial pose for example) or
computed using another method:

\code
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeKltTracker.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbEdgeKltTracker tracker; // Create an hybrid model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used in entry (has to be defined), then computed using the tracker.

  //acquire an image
  vpImageIo::read(I, "cube.pgm"); // Example of acquisition

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.
  tracker.loadModel("cube.cao");
  tracker.initFromPose(I, cMo); // initialise the tracker with the given pose.

  while(true){
    // acquire a new image
    tracker.track(I); // track the object on this image
    tracker.getPose(cMo); // get the pose
  }

#if defined VISP_HAVE_XML2
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeKltTracker::loadConfigFile()
  vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode

  Finally it can be used not to track an object but just to display a model at
a given pose:

\code
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplayX.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeKltTracker.h>

int main()
{
#if defined VISP_HAVE_OPENCV
  vpMbEdgeKltTracker tracker; // Create an hybrid model based tracker.
  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo; // Pose used to display the model.
  vpCameraParameters cam;

  // Acquire an image
  vpImageIo::read(I, "cube.pgm");

#if defined VISP_HAVE_X11
  vpDisplayX display;
  display.init(I,100,100,"Mb Hybrid Tracker");
#endif

#if defined VISP_HAVE_XML2
  tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
#endif
  tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
  // load the 3d model, to read .wrl model coin is required, if coin is not installed .cao file can be used.
  tracker.loadModel("cube.cao");

  while(true){
    // acquire a new image
    // Get the pose using any method
    vpDisplay::display(I);
    tracker.display(I, cMo, cam, vpColor::darkRed, 1, true); // Display the model at the computed pose.
    vpDisplay::flush(I);
  }

#endif
  // Cleanup memory allocated by xml library used to parse the xml config file in vpMbEdgeKltTracker::loadConfigFile()
 vpXmlParser::cleanup();
#endif

  return 0;
#endif
}
\endcode
*/
class VISP_EXPORT vpMbEdgeKltTracker : public vpMbKltTracker, public vpMbEdgeTracker
{
protected:
  //! The threshold used in the robust estimation of KLT.
  double thresholdKLT;
  //! The threshold used in the robust estimation of MBT.
  double thresholdMBT;
  //! The maximum iteration of the virtual visual servoing stage.
  unsigned int m_maxIterKlt;
  //! Robust weights for Edge
  vpColVector w_mbt;
  //! Robust weights for KLT
  vpColVector w_klt;
  //! (s - s*)
  vpColVector m_error_hybrid;
  //! Robust weights
  vpColVector m_w_hybrid;

public:
  vpMbEdgeKltTracker();
  virtual ~vpMbEdgeKltTracker();

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual inline vpColVector getError() const { return m_error_hybrid; }

  virtual inline vpColVector getRobustWeights() const { return m_w_hybrid; }

  /*!
    Get the near distance for clipping.

    \return Near clipping value.
   */
  virtual inline double getNearClippingDistance() const { return vpMbKltTracker::getNearClippingDistance(); }

  void loadConfigFile(const char *configFile);
  virtual void loadConfigFile(const std::string &configFile);

  void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo_,
                   const bool verbose = false);
  void reInitModel(const vpImage<unsigned char> &I, const char *cad_name, const vpHomogeneousMatrix &cMo,
                   const bool verbose = false);
  void resetTracker();

  virtual void setCameraParameters(const vpCameraParameters &cam);

  /*!
    Specify which clipping to use.

    \sa vpMbtPolygonClipping

    \param flags : New clipping flags.
   */
  virtual void setClipping(const unsigned int &flags) { vpMbEdgeTracker::setClipping(flags); }

  /*!
    Set the far distance for clipping.

    \param dist : Far clipping value.
   */
  virtual void setFarClippingDistance(const double &dist) { vpMbEdgeTracker::setFarClippingDistance(dist); }

  /*!
    Set the near distance for clipping.

    \param dist : Near clipping value.
   */
  virtual void setNearClippingDistance(const double &dist) { vpMbEdgeTracker::setNearClippingDistance(dist); }

  /*!
    Use Ogre3D for visibility tests

    \warning This function has to be called before the initialization of the
    tracker.

    \param v : True to use it, False otherwise
   */
  virtual void setOgreVisibilityTest(const bool &v)
  {
    vpMbTracker::setOgreVisibilityTest(v);
#ifdef VISP_HAVE_OGRE
    faces.getOgreContext()->setWindowName("MBT Hybrid");
#endif
  }

  /*!
    Use Scanline algorithm for visibility tests

    \param v : True to use it, False otherwise
  */
  virtual void setScanLineVisibilityTest(const bool &v)
  {
    vpMbEdgeTracker::setScanLineVisibilityTest(v);
    vpMbKltTracker::setScanLineVisibilityTest(v);
  }

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo);
  /*!
    Set if the projection error criteria has to be computed.

    \param flag : True if the projection error criteria has to be computed,
    false otherwise
  */
  virtual void setProjectionErrorComputation(const bool &flag) { vpMbEdgeTracker::setProjectionErrorComputation(flag); }

  virtual void testTracking() {}
  virtual void track(const vpImage<unsigned char> &I);

protected:
  virtual void computeVVS(const vpImage<unsigned char> &I, const unsigned int &nbInfos, unsigned int &nbrow,
                          const unsigned int lvl = 0);
  virtual void computeVVSInit();
  virtual void computeVVSInteractionMatrixAndResidu();
  using vpMbTracker::computeCovarianceMatrixVVS;
  using vpMbTracker::computeVVSPoseEstimation;

  virtual void init(const vpImage<unsigned char> &I);
  virtual void initCircle(const vpPoint &, const vpPoint &, const vpPoint &, const double r, const int idFace = 0,
                          const std::string &name = "");
  virtual void initCylinder(const vpPoint &, const vpPoint &, const double r, const int idFace,
                            const std::string &name = "");
  virtual void initFaceFromCorners(vpMbtPolygon &polygon);
  virtual void initFaceFromLines(vpMbtPolygon &polygon);
  unsigned int initMbtTracking(const unsigned int level = 0);

  bool postTracking(const vpImage<unsigned char> &I, vpColVector &w_mbt, vpColVector &w_klt,
                    const unsigned int lvl = 0);
  void postTrackingMbt(vpColVector &w, const unsigned int level = 0);

  unsigned int trackFirstLoop(const vpImage<unsigned char> &I, vpColVector &factor, const unsigned int lvl = 0);
  void trackSecondLoop(const vpImage<unsigned char> &I, vpMatrix &L, vpColVector &_error, vpHomogeneousMatrix &cMo,
                       const unsigned int lvl = 0);
};

#endif

#endif // VISP_HAVE_OPENCV
