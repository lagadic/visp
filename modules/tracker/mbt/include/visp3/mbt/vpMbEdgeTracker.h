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
 * Make the complete tracking of an object by using its CAD model
 */

/*!
 * \file vpMbEdgeTracker.h
 * \brief Make the complete tracking of an object by using its CAD model.
 */

#ifndef vpMbEdgeTracker_HH
#define vpMbEdgeTracker_HH

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPoint.h>
#include <visp3/mbt/vpMbTracker.h>
#include <visp3/mbt/vpMbtDistanceCircle.h>
#include <visp3/mbt/vpMbtDistanceCylinder.h>
#include <visp3/mbt/vpMbtDistanceLine.h>
#include <visp3/mbt/vpMbtMeLine.h>
#include <visp3/me/vpMe.h>

#include <fstream>
#include <iostream>
#include <list>
#include <vector>

#if defined(VISP_HAVE_COIN3D)
// Inventor includes
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/nodes/SoSeparator.h>
#endif

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#endif

BEGIN_VISP_NAMESPACE

/*!
 * \class vpMbEdgeTracker
 * \ingroup group_mbt_trackers
 * \brief Make the complete tracking of an object by using its CAD model.
 * \warning This class is deprecated for user usage. You should rather use the high level
 * vpMbGenericTracker class.
 *
 * This class allows to track an object or a scene given its 3D model. A
 * video can be found on YouTube \e https://www.youtube.com/watch?v=UK10KMMJFCI
 * The \ref tutorial-tracking-mb-deprecated is also a good starting point to use this class.
 *
 * The tracker requires the knowledge of the 3D model that could be provided in
 * a vrml or in a cao file. The cao format is described in loadCAOModel(). It may
 * also use an xml file used to tune the behavior of the tracker and an init file
 * used to compute the pose at the very first image.
 *
 * The following code shows the simplest way to use the tracker.
 *
 * \code
 * #include <visp3/core/vpCameraParameters.h>
 * #include <visp3/core/vpHomogeneousMatrix.h>
 * #include <visp3/core/vpImage.h>
 * #include <visp3/gui/vpDisplayX.h>
 * #include <visp3/io/vpImageIo.h>
 * #include <visp3/mbt/vpMbEdgeTracker.h>
 *
 * int main()
 * {
 *   vpMbEdgeTracker tracker; // Create a model based tracker.
 *   vpImage<unsigned char> I;
 *   vpHomogeneousMatrix cMo; // Pose computed using the tracker.
 *   vpCameraParameters cam;
 *
 *   // Acquire an image
 *   vpImageIo::read(I, "cube.pgm");
 *
 * #if defined(VISP_HAVE_X11)
 *   vpDisplayX display;
 *   display.init(I,100,100,"Mb Edge Tracker");
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
 *   vpDisplay::flush(I);
 *   }
 *
 *   return 0;
 * }
 * \endcode
 *
 * For application with large inter-images displacement, multi-scale tracking
 * is also possible, by setting the number of scales used and by activating (or
 * not) them using a vector of booleans, as presented in the following code:
 *
 * \code
 *   ...
 *   vpHomogeneousMatrix cMo; // Pose computed using the tracker.
 *   vpCameraParameters cam;
 *
 *   std::vector< bool > scales(3); //Three scales used
 *   scales.push_back(true); //First scale : active
 *   scales.push_back(false); //Second scale (/2) : not active
 *   scales.push_back(true); //Third scale (/4) : active
 *   tracker.setScales(scales); // Set active scales for multi-scale tracking
 *
 *   tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
 *   tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
 *   ...
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
 * #include <visp3/mbt/vpMbEdgeTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpMbEdgeTracker tracker; // Create a model based tracker.
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
 * #include <visp3/mbt/vpMbEdgeTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpMbEdgeTracker tracker; // Create a model based tracker.
 *   vpImage<unsigned char> I;
 *   vpHomogeneousMatrix cMo; // Pose used to display the model.
 *   vpCameraParameters cam;
 *
 *   // Acquire an image
 *   vpImageIo::read(I, "cube.pgm");
 *
 * #if defined(VISP_HAVE_X11)
 *   vpDisplayX display;
 *   display.init(I,100,100,"Mb Edge Tracker");
 * #endif
 *
 *   tracker.loadConfigFile("cube.xml"); // Load the configuration of the tracker
 *   tracker.getCameraParameters(cam); // Get the camera parameters used by the tracker (from the configuration file).
 *   // load the 3d model, to read .wrl model coin is required, if coin is not installed
 *   // .cao file can be used.
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
 * }
 $ \endcode
*/
class VISP_EXPORT vpMbEdgeTracker : public virtual vpMbTracker
{
protected:
  //! The moving edges parameters.
  vpMe me;
  //! Vector of list of all the lines tracked (each line is linked to a list
  //! of moving edges). Each element of the vector is for a scale (element 0 =
  //! level 0 = no subsampling).
  std::vector<std::list<vpMbtDistanceLine *> > lines;

  //! Vector of the tracked circles.
  std::vector<std::list<vpMbtDistanceCircle *> > circles;

  //! Vector of the tracked cylinders.
  std::vector<std::list<vpMbtDistanceCylinder *> > cylinders;

  //! Index of the polygon to add, and total number of polygon extracted so
  //! far.
  unsigned int nline;

  //! Index of the circle to add, and total number of circles extracted so
  //! far.
  unsigned int ncircle;

  //! Index of the cylinder to add, and total number of cylinders extracted so
  //! far.
  unsigned int ncylinder;

  //! Number of polygon (face) currently visible.
  unsigned int nbvisiblepolygone;

  //! Percentage of good points over total number of points below which
  //! tracking is supposed to have failed.
  double percentageGdPt;

  //! Vector of scale level to use for the multi-scale tracking.
  std::vector<bool> scales;

  //! Pyramid of image associated to the current image. This pyramid is
  //! computed in the init() and in the track() methods.
  std::vector<const vpImage<unsigned char> *> Ipyramid;

  //! Current scale level used. This attribute must not be modified outside of
  //! the downScale() and upScale() methods, as it used to specify to some
  //! methods which set of distanceLine use.
  unsigned int scaleLevel;

  //! Number of features used in the computation of the projection error
  unsigned int nbFeaturesForProjErrorComputation;

  /// Edge VVS variables
  vpColVector m_factor;
  vpRobust m_robustLines;
  vpRobust m_robustCylinders;
  vpRobust m_robustCircles;
  vpColVector m_wLines;
  vpColVector m_wCylinders;
  vpColVector m_wCircles;
  vpColVector m_errorLines;
  vpColVector m_errorCylinders;
  vpColVector m_errorCircles;
  //! Interaction matrix
  vpMatrix m_L_edge;
  //! (s - s*)
  vpColVector m_error_edge;
  //! Robust weights
  vpColVector m_w_edge;
  //! Weighted error
  vpColVector m_weightedError_edge;
  //! Robust
  vpRobust m_robust_edge;
  //! Display features
  std::vector<std::vector<double> > m_featuresToBeDisplayedEdge;

public:
  vpMbEdgeTracker();
  virtual ~vpMbEdgeTracker() VP_OVERRIDE;

  /** @name Inherited functionalities from vpMbEdgeTracker */
  //@{

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;
  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  void getLline(std::list<vpMbtDistanceLine *> &linesList, unsigned int level = 0) const;
  void getLcircle(std::list<vpMbtDistanceCircle *> &circlesList, unsigned int level = 0) const;
  void getLcylinder(std::list<vpMbtDistanceCylinder *> &cylindersList, unsigned int level = 0) const;

  virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
                                                               const vpHomogeneousMatrix &cMo,
                                                               const vpCameraParameters &cam,
                                                               bool displayFullModel = false) VP_OVERRIDE;

  /*!
   * Get the moving edge parameters.
   *
   * \param p_me [out] : an instance of the moving edge parameters used by the
   * tracker.
   */
  virtual inline void getMovingEdge(vpMe &p_me) const { p_me = this->me; }
  /*!
   * Get the moving edge parameters.
   *
   * \return an instance of the moving edge parameters used by the tracker.
   */
  virtual inline vpMe getMovingEdge() const { return this->me; }

  virtual unsigned int getNbPoints(unsigned int level = 0) const;

  /*!
   * Return the scales levels used for the tracking.
   *
   * \return The scales levels used for the tracking.
   */
  std::vector<bool> getScales() const { return scales; }
  /*!
   *  \return The threshold value between 0 and 1 over good moving edges ratio.
   *  It allows to decide if the tracker has enough valid moving edges to
   *  compute a pose. 1 means that all moving edges should be considered as
   *  good to have a valid pose, while 0.1 means that 10% of the moving edge
   *  are enough to declare a pose valid.
   *
   *  \sa setGoodMovingEdgesRatioThreshold()
   */
  inline double getGoodMovingEdgesRatioThreshold() const { return percentageGdPt; }

  virtual inline vpColVector getError() const VP_OVERRIDE { return m_error_edge; }

  virtual inline vpColVector getRobustWeights() const VP_OVERRIDE { return m_w_edge; }

  virtual void loadConfigFile(const std::string &configFile, bool verbose = true) VP_OVERRIDE;

  virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo,
                           bool verbose = false, const vpHomogeneousMatrix &T = vpHomogeneousMatrix());
  void resetTracker() VP_OVERRIDE;

 /*!
  * Set the camera parameters.
  *
  * \param cam : The new camera parameters.
  */
  virtual void setCameraParameters(const vpCameraParameters &cam) VP_OVERRIDE
  {
    m_cam = cam;

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
          (*it)->setCameraParameters(m_cam);
        }

        for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
             ++it) {
          (*it)->setCameraParameters(m_cam);
        }

        for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
          (*it)->setCameraParameters(m_cam);
        }
      }
    }
  }

  virtual void setClipping(const unsigned int &flags) VP_OVERRIDE;

  virtual void setFarClippingDistance(const double &dist) VP_OVERRIDE;

  virtual void setNearClippingDistance(const double &dist) VP_OVERRIDE;

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
    faces.getOgreContext()->setWindowName("MBT Edge");
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

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
          (*it)->useScanLine = v;
        }
      }
    }
  }

  /*!
   *  Set the threshold value between 0 and 1 over good moving edges ratio. It
   *  allows to decide if the tracker has enough valid moving edges to compute
   *  a pose. 1 means that all moving edges should be considered as good to
   *  have a valid pose, while 0.1 means that 10% of the moving edge are enough
   *  to declare a pose valid.
   *
   *  \param threshold : Value between 0 and 1 that corresponds to the ratio of
   *  good moving edges that is necessary to consider that the estimated pose
   *  is valid. Default value is 0.4.
   *
   *  \sa getGoodMovingEdgesRatioThreshold()
   */
  void setGoodMovingEdgesRatioThreshold(double threshold) { percentageGdPt = threshold; }

  void setMovingEdge(const vpMe &me);

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
  virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;

  void setScales(const std::vector<bool> &_scales);

  void setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking);

  virtual void track(const vpImage<unsigned char> &I) VP_OVERRIDE;
  virtual void track(const vpImage<vpRGBa> &I) VP_OVERRIDE;
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpMbEdgeTracker */
  //@{
  void addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, double r, int idFace = -1,
                 const std::string &name = "");
  void addCylinder(const vpPoint &P1, const vpPoint &P2, double r, int idFace = -1, const std::string &name = "");
  void addLine(vpPoint &p1, vpPoint &p2, int polygon = -1, std::string name = "");
  void addPolygon(vpMbtPolygon &p);

  void cleanPyramid(std::vector<const vpImage<unsigned char> *> &_pyramid);
  void computeProjectionError(const vpImage<unsigned char> &_I);

  void computeVVS(const vpImage<unsigned char> &_I, unsigned int lvl);
  void computeVVSFirstPhase(const vpImage<unsigned char> &I, unsigned int iter, double &count, unsigned int lvl = 0);
  void computeVVSFirstPhaseFactor(const vpImage<unsigned char> &I, unsigned int lvl = 0);
  void computeVVSFirstPhasePoseEstimation(unsigned int iter, bool &isoJoIdentity);
  virtual void computeVVSInit() VP_OVERRIDE;
  virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE;
  virtual void computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> &I);
  virtual void computeVVSWeights();
  using vpMbTracker::computeVVSWeights;

  void displayFeaturesOnImage(const vpImage<unsigned char> &I);
  void displayFeaturesOnImage(const vpImage<vpRGBa> &I);
  void downScale(const unsigned int _scale);
  virtual std::vector<std::vector<double> > getFeaturesForDisplayEdge();
  virtual void init(const vpImage<unsigned char> &I) VP_OVERRIDE;
  virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, double radius, int idFace = 0,
                          const std::string &name = "") VP_OVERRIDE;
  virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, double radius, int idFace = 0,
                            const std::string &name = "") VP_OVERRIDE;
  virtual void initFaceFromCorners(vpMbtPolygon &polygon) VP_OVERRIDE;
  virtual void initFaceFromLines(vpMbtPolygon &polygon) VP_OVERRIDE;
  unsigned int initMbtTracking(unsigned int &nberrors_lines, unsigned int &nberrors_cylinders,
                               unsigned int &nberrors_circles);
  void initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo);
  void initPyramid(const vpImage<unsigned char> &_I, std::vector<const vpImage<unsigned char> *> &_pyramid);
  void reInitLevel(const unsigned int _lvl);
  void reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo);
  void removeCircle(const std::string &name);
  void removeCylinder(const std::string &name);
  void removeLine(const std::string &name);
  void resetMovingEdge();
  virtual void testTracking() VP_OVERRIDE;
  void trackMovingEdge(const vpImage<unsigned char> &I);
  void updateMovingEdge(const vpImage<unsigned char> &I);
  void updateMovingEdgeWeights();
  void upScale(const unsigned int _scale);
  void visibleFace(const vpImage<unsigned char> &_I, const vpHomogeneousMatrix &_cMo, bool &newvisibleline);
  //@}
};
END_VISP_NAMESPACE
#endif
