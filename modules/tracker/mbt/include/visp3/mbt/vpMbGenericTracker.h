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
 * Generic model-based tracker.
 */

/*!
 * \file vpMbGenericTracker.h
 *\brief Generic model-based tracker
 */

#ifndef _vpMbGenericTracker_h_
#define _vpMbGenericTracker_h_

#include <visp3/core/vpConfig.h>
#include <visp3/mbt/vpMbDepthDenseTracker.h>
#include <visp3/mbt/vpMbDepthNormalTracker.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbKltTracker.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json_fwd.hpp>
#include <visp3/core/vpJsonParsing.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMbGenericTracker
 * \ingroup group_mbt_trackers
 * \brief Real-time 6D object pose tracking using its CAD model.
 *
 * The tracker requires the knowledge of the 3D model that could be provided in
 * a vrml or in a cao file. The cao format is described in loadCAOModel(). It may
 * also use an xml file used to tune the behavior of the tracker and an init file
 * used to compute the pose at the very first image.
 *
 * This class allows tracking an object or a scene given its 3D model. More information in \cite Trinh18a.
 * A lot of videos can be found on <a href="https://www.youtube.com/user/VispTeam">YouTube VispTeam</a> channel.
 *
 * \htmlonly
 * <iframe width="280" height="160" src="https://www.youtube.com/embed/UK10KMMJFCI"
 * frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
 * <iframe width="280" height="160" src="https://www.youtube.com/embed/DDdIXja7YpE"
 * frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
 * <iframe width="280" height="160" src="https://www.youtube.com/embed/M3XAxu9QC7Q"
 * frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
 * <iframe width="280" height="160" src="https://www.youtube.com/embed/4FARYLYzNL8"
 * frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
 * \endhtmlonly
 *
 * The \ref tutorial-tracking-mb-generic is a good starting point to use this
 * class. If you want to track an object with a stereo camera refer to
 * \ref tutorial-tracking-mb-generic-stereo. If you want rather use a RGB-D camera and exploit
 * the depth information, you may see \ref tutorial-tracking-mb-generic-rgbd.
 * There is also \ref tutorial-detection-object that shows how to initialize the tracker from
 * an initial pose provided by a detection algorithm.
 *
 * <b>JSON serialization</b>
 *
 * Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpMbGenericTracker.
 * The following sample code shows how to save a model-based tracker settings in a file named `mbt.json`
 * and reload the values from this JSON file.
 * \code
 * #include <visp3/mbt/vpMbGenericTracker.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_NLOHMANN_JSON)
 *   std::string filename = "mbt-generic.json";
 *   {
 *     vpMbGenericTracker mbt;
 *     mbt.saveConfigFile(filename);
 *   }
 *   {
 *     vpMbGenericTracker mbt;
 *     bool verbose = false;
 *     std::cout << "Read model-based tracker settings from " << filename << std::endl;
 *     mbt.loadConfigFile(filename, verbose);
 *   }
 * #endif
 * }
 * \endcode
 * If you build and execute the sample code, it will produce the following output:
 * \code{.unparsed}
 * Read model-based tracker settings from mbt-generic.json
 * \endcode
 *
 * The content of the `mbt.json` file is the following:
 * \code{.unparsed}
 * $ cat mbt-generic.json
 * {
 *   "referenceCameraName": "Camera",
 *   "trackers": {
 *       "Camera": {
 *           "angleAppear": 89.0,
 *           "angleDisappear": 89.0,
 *           "camTref": {
 *               "cols": 4,
 *               "data": [
 *                   1.0,
 *                   0.0,
 *                   0.0,
 *                   0.0,
 *                   0.0,
 *                   1.0,
 *                   0.0,
 *                   0.0,
 *                   0.0,
 *                   0.0,
 *                   1.0,
 *                   0.0,
 *                   0.0,
 *                   0.0,
 *                   0.0,
 *                   1.0
 *               ],
 *               "rows": 4,
 *               "type": "vpHomogeneousMatrix"
 *           },
 *           "camera": {
 *               "model": "perspectiveWithoutDistortion",
 *               "px": 600.0,
 *               "py": 600.0,
 *               "u0": 192.0,
 *               "v0": 144.0
 *           },
 *           "clipping": {
 *               "far": 100.0,
 *               "flags": [
 *                   "none"
 *               ],
 *               "near": 0.001
 *           },
 *           "display": {
 *               "features": false,
 *               "projectionError": false
 *           },
 *           "edge": {
 *               "maskSign": 0,
 *               "maskSize": 5,
 *               "minSampleStep": 4.0,
 *               "mu": [
 *                   0.5,
 *                   0.5
 *               ],
 *               "nMask": 180,
 *               "ntotalSample": 0,
 *               "pointsToTrack": 500,
 *               "range": 4,
 *               "sampleStep": 10.0,
 *               "strip": 2,
*                "thresholdType": "normalized",
 *               "threshold": 20.0
 *           },
 *           "lod": {
 *               "minLineLengthThresholdGeneral": 50.0,
 *               "minPolygonAreaThresholdGeneral": 2500.0,
 *               "useLod": false
 *           },
 *           "type": [
 *               "edge"
 *           ],
 *           "visibilityTest": {
 *               "ogre": false,
 *               "scanline": false
 *           }
 *       }
 *   },
 *   "version": "1.0"
 * }
 * \endcode
*/
class VISP_EXPORT vpMbGenericTracker : public vpMbTracker
{
public:
  enum vpTrackerType
  {
    EDGE_TRACKER = 1 << 0, /*!< Model-based tracking using moving edges features. */
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) &&  defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO))
    KLT_TRACKER = 1 << 1, /*!< Model-based tracking using KLT features. */
#endif
    DEPTH_NORMAL_TRACKER = 1 << 2, /*!< Model-based tracking using depth normal features. */
    DEPTH_DENSE_TRACKER = 1 << 3   /*!< Model-based tracking using depth dense features. */
  };

  vpMbGenericTracker();
  vpMbGenericTracker(unsigned int nbCameras, int trackerType = EDGE_TRACKER);
  VP_EXPLICIT vpMbGenericTracker(const std::vector<int> &trackerTypes);
  vpMbGenericTracker(const std::vector<std::string> &cameraNames, const std::vector<int> &trackerTypes);

  virtual ~vpMbGenericTracker() VP_OVERRIDE;

  virtual double computeCurrentProjectionError(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo,
    const vpCameraParameters &_cam) VP_OVERRIDE;
  virtual double computeCurrentProjectionError(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &_cMo,
    const vpCameraParameters &_cam);

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
    const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;
  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
    const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

  virtual void display(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
    const vpCameraParameters &cam2, const vpColor &color, unsigned int thickness = 1,
    bool displayFullModel = false);
  virtual void display(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, const vpHomogeneousMatrix &c1Mo,
    const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1, const vpCameraParameters &cam2,
    const vpColor &color, unsigned int thickness = 1, bool displayFullModel = false);

  virtual void display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
    const std::map<std::string, vpCameraParameters> &mapOfCameraParameters, const vpColor &col,
    unsigned int thickness = 1, bool displayFullModel = false);
  virtual void display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
    const std::map<std::string, vpCameraParameters> &mapOfCameraParameters, const vpColor &col,
    unsigned int thickness = 1, bool displayFullModel = false);

  virtual std::vector<std::string> getCameraNames() const;

  using vpMbTracker::getCameraParameters;
  virtual void getCameraParameters(vpCameraParameters &camera) const VP_OVERRIDE;
  virtual void getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const;
  virtual void getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const;

  virtual std::map<std::string, int> getCameraTrackerTypes() const;

  using vpMbTracker::getClipping;
  virtual void getClipping(unsigned int &clippingFlag1, unsigned int &clippingFlag2) const;
  virtual void getClipping(std::map<std::string, unsigned int> &mapOfClippingFlags) const;

  virtual inline vpColVector getError() const VP_OVERRIDE { return m_error; }

  virtual vpMbHiddenFaces<vpMbtPolygon> &getFaces() VP_OVERRIDE;
  virtual vpMbHiddenFaces<vpMbtPolygon> &getFaces(const std::string &cameraName);

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  virtual std::list<vpMbtDistanceCircle *> &getFeaturesCircle();
  virtual std::list<vpMbtDistanceKltCylinder *> &getFeaturesKltCylinder();
  virtual std::list<vpMbtDistanceKltPoints *> &getFeaturesKlt();
#endif

  virtual std::vector<std::vector<double> > getFeaturesForDisplay();
  virtual void getFeaturesForDisplay(std::map<std::string, std::vector<std::vector<double> > > &mapOfFeatures);

  virtual double getGoodMovingEdgesRatioThreshold() const;

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  virtual std::vector<vpImagePoint> getKltImagePoints() const;
  virtual std::map<int, vpImagePoint> getKltImagePointsWithId() const;

  virtual unsigned int getKltMaskBorder() const;
  virtual int getKltNbPoints() const;

  virtual vpKltOpencv getKltOpencv() const;
  virtual void getKltOpencv(vpKltOpencv &klt1, vpKltOpencv &klt2) const;
  virtual void getKltOpencv(std::map<std::string, vpKltOpencv> &mapOfKlts) const;

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  virtual std::vector<cv::Point2f> getKltPoints() const;
#endif

  virtual double getKltThresholdAcceptation() const;
#endif

  virtual void getLcircle(std::list<vpMbtDistanceCircle *> &circlesList, unsigned int level = 0) const;
  virtual void getLcircle(const std::string &cameraName, std::list<vpMbtDistanceCircle *> &circlesList,
    unsigned int level = 0) const;
  virtual void getLcylinder(std::list<vpMbtDistanceCylinder *> &cylindersList, unsigned int level = 0) const;
  virtual void getLcylinder(const std::string &cameraName, std::list<vpMbtDistanceCylinder *> &cylindersList,
    unsigned int level = 0) const;
  virtual void getLline(std::list<vpMbtDistanceLine *> &linesList, unsigned int level = 0) const;
  virtual void getLline(const std::string &cameraName, std::list<vpMbtDistanceLine *> &linesList,
    unsigned int level = 0) const;

  virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
    const vpHomogeneousMatrix &cMo,
    const vpCameraParameters &cam,
    bool displayFullModel = false) VP_OVERRIDE;
  virtual void getModelForDisplay(std::map<std::string, std::vector<std::vector<double> > > &mapOfModels,
    const std::map<std::string, unsigned int> &mapOfwidths,
    const std::map<std::string, unsigned int> &mapOfheights,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfcMos,
    const std::map<std::string, vpCameraParameters> &mapOfCams,
    bool displayFullModel = false);

  virtual vpMe getMovingEdge() const;
  virtual void getMovingEdge(vpMe &me1, vpMe &me2) const;
  virtual void getMovingEdge(std::map<std::string, vpMe> &mapOfMovingEdges) const;

  /*!
   * Return the number of depth dense features taken into account in the virtual visual-servoing scheme.
   */
  virtual inline unsigned int getNbFeaturesDepthDense() const { return m_nb_feat_depthDense; }

  /*!
   * Return the number of depth normal features features taken into account in the virtual visual-servoing scheme.
   */
  virtual inline unsigned int getNbFeaturesDepthNormal() const { return m_nb_feat_depthNormal; }

  /*!
   * Return the number of moving-edges features taken into account in the virtual visual-servoing scheme.
   *
   * This function is similar to getNbPoints().
   */
  virtual inline unsigned int getNbFeaturesEdge() const { return m_nb_feat_edge; }

  /*!
   * Return the number of klt keypoints features taken into account in the virtual visual-servoing scheme.
   */
  virtual inline unsigned int getNbFeaturesKlt() const { return m_nb_feat_klt; }

  virtual unsigned int getNbPoints(unsigned int level = 0) const;
  virtual void getNbPoints(std::map<std::string, unsigned int> &mapOfNbPoints, unsigned int level = 0) const;

  virtual unsigned int getNbPolygon() const VP_OVERRIDE;
  virtual void getNbPolygon(std::map<std::string, unsigned int> &mapOfNbPolygons) const;

  virtual vpMbtPolygon *getPolygon(unsigned int index) VP_OVERRIDE;
  virtual vpMbtPolygon *getPolygon(const std::string &cameraName, unsigned int index);

  virtual std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > >
    getPolygonFaces(bool orderPolygons = true, bool useVisibility = true, bool clipPolygon = false) VP_OVERRIDE;
  virtual void getPolygonFaces(std::map<std::string, std::vector<vpPolygon> > &mapOfPolygons,
    std::map<std::string, std::vector<std::vector<vpPoint> > > &mapOfPoints,
    bool orderPolygons = true, bool useVisibility = true, bool clipPolygon = false);

  using vpMbTracker::getPose;
  virtual void getPose(vpHomogeneousMatrix &cMo) const VP_OVERRIDE;
  virtual void getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const;
  virtual void getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const;

  virtual std::string getReferenceCameraName() const;

  virtual inline vpColVector getRobustWeights() const VP_OVERRIDE { return m_w; }

  virtual int getTrackerType() const;

  virtual void init(const vpImage<unsigned char> &I) VP_OVERRIDE;

#ifdef VISP_HAVE_MODULE_GUI
  using vpMbTracker::initClick;
  virtual void initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &initFile1, const std::string &initFile2, bool displayHelp = false,
    const vpHomogeneousMatrix &T1 = vpHomogeneousMatrix(),
    const vpHomogeneousMatrix &T2 = vpHomogeneousMatrix());
  virtual void initClick(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2, const std::string &initFile1,
    const std::string &initFile2, bool displayHelp = false,
    const vpHomogeneousMatrix &T1 = vpHomogeneousMatrix(),
    const vpHomogeneousMatrix &T2 = vpHomogeneousMatrix());

  virtual void
    initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, std::string> &mapOfInitFiles, bool displayHelp = false,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfT = std::map<std::string, vpHomogeneousMatrix>());
  virtual void
    initClick(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
      const std::map<std::string, std::string> &mapOfInitFiles, bool displayHelp = false,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfT = std::map<std::string, vpHomogeneousMatrix>());
#endif

  using vpMbTracker::initFromPoints;
  virtual void initFromPoints(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &initFile1, const std::string &initFile2);
  virtual void initFromPoints(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
    const std::string &initFile1, const std::string &initFile2);

  virtual void initFromPoints(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, std::string> &mapOfInitPoints);
  virtual void initFromPoints(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    const std::map<std::string, std::string> &mapOfInitPoints);

  using vpMbTracker::initFromPose;
  virtual void initFromPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;
  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &initFile1, const std::string &initFile2);
  virtual void initFromPose(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
    const std::string &initFile1, const std::string &initFile2);

  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, std::string> &mapOfInitPoses);
  virtual void initFromPose(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    const std::map<std::string, std::string> &mapOfInitPoses);

  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);
  virtual void initFromPose(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);

  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);
  virtual void initFromPose(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void loadConfigFile(const std::string &configFile, bool verbose = true) VP_OVERRIDE;
  virtual void loadConfigFile(const std::string &configFile1, const std::string &configFile2, bool verbose = true);
  virtual void loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles, bool verbose = true);

#ifdef VISP_HAVE_NLOHMANN_JSON
  virtual void saveConfigFile(const std::string &settingsFile) const;
#endif

  virtual void loadModel(const std::string &modelFile, bool verbose = false,
    const vpHomogeneousMatrix &T = vpHomogeneousMatrix()) VP_OVERRIDE;
  virtual void loadModel(const std::string &modelFile1, const std::string &modelFile2, bool verbose = false,
    const vpHomogeneousMatrix &T1 = vpHomogeneousMatrix(),
    const vpHomogeneousMatrix &T2 = vpHomogeneousMatrix());

  virtual void
    loadModel(const std::map<std::string, std::string> &mapOfModelFiles, bool verbose = false,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfT = std::map<std::string, vpHomogeneousMatrix>());

  virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name, const vpHomogeneousMatrix &cMo,
    bool verbose = false, const vpHomogeneousMatrix &T = vpHomogeneousMatrix());
  virtual void reInitModel(const vpImage<vpRGBa> &I_color, const std::string &cad_name, const vpHomogeneousMatrix &cMo,
    bool verbose = false, const vpHomogeneousMatrix &T = vpHomogeneousMatrix());

  virtual void reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const std::string &cad_name1, const std::string &cad_name2, const vpHomogeneousMatrix &c1Mo,
    const vpHomogeneousMatrix &c2Mo, bool verbose = false,
    const vpHomogeneousMatrix &T1 = vpHomogeneousMatrix(),
    const vpHomogeneousMatrix &T2 = vpHomogeneousMatrix());
  virtual void reInitModel(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
    const std::string &cad_name1, const std::string &cad_name2, const vpHomogeneousMatrix &c1Mo,
    const vpHomogeneousMatrix &c2Mo, bool verbose = false,
    const vpHomogeneousMatrix &T1 = vpHomogeneousMatrix(),
    const vpHomogeneousMatrix &T2 = vpHomogeneousMatrix());

  virtual void
    reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
      const std::map<std::string, std::string> &mapOfModelFiles,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses, bool verbose = false,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfT = std::map<std::string, vpHomogeneousMatrix>());
  virtual void
    reInitModel(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
      const std::map<std::string, std::string> &mapOfModelFiles,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses, bool verbose = false,
      const std::map<std::string, vpHomogeneousMatrix> &mapOfT = std::map<std::string, vpHomogeneousMatrix>());

  virtual void resetTracker() VP_OVERRIDE;

  virtual void setAngleAppear(const double &a) VP_OVERRIDE;
  virtual void setAngleAppear(const double &a1, const double &a2);
  virtual void setAngleAppear(const std::map<std::string, double> &mapOfAngles);

  virtual void setAngleDisappear(const double &a) VP_OVERRIDE;
  virtual void setAngleDisappear(const double &a1, const double &a2);
  virtual void setAngleDisappear(const std::map<std::string, double> &mapOfAngles);

  virtual void setCameraParameters(const vpCameraParameters &camera) VP_OVERRIDE;
  virtual void setCameraParameters(const vpCameraParameters &camera1, const vpCameraParameters &camera2);
  virtual void setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters);

  virtual void setCameraTransformationMatrix(const std::string &cameraName,
    const vpHomogeneousMatrix &cameraTransformationMatrix);
  virtual void
    setCameraTransformationMatrix(const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix);

  virtual void setClipping(const unsigned int &flags) VP_OVERRIDE;
  virtual void setClipping(const unsigned int &flags1, const unsigned int &flags2);
  virtual void setClipping(const std::map<std::string, unsigned int> &mapOfClippingFlags);

  virtual void setDepthDenseFilteringMaxDistance(double maxDistance);
  virtual void setDepthDenseFilteringMethod(int method);
  virtual void setDepthDenseFilteringMinDistance(double minDistance);
  virtual void setDepthDenseFilteringOccupancyRatio(double occupancyRatio);
  virtual void setDepthDenseSamplingStep(unsigned int stepX, unsigned int stepY);

  virtual void setDepthNormalFaceCentroidMethod(const vpMbtFaceDepthNormal::vpFaceCentroidType &method);
  virtual void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method);
  virtual void setDepthNormalPclPlaneEstimationMethod(int method);
  virtual void setDepthNormalPclPlaneEstimationRansacMaxIter(int maxIter);
  virtual void setDepthNormalPclPlaneEstimationRansacThreshold(double threshold);
  virtual void setDepthNormalSamplingStep(unsigned int stepX, unsigned int stepY);

  virtual void setDisplayFeatures(bool displayF) VP_OVERRIDE;

  virtual void setFarClippingDistance(const double &dist) VP_OVERRIDE;
  virtual void setFarClippingDistance(const double &dist1, const double &dist2);
  virtual void setFarClippingDistance(const std::map<std::string, double> &mapOfClippingDists);

  virtual void setFeatureFactors(const std::map<vpTrackerType, double> &mapOfFeatureFactors);

  virtual void setGoodMovingEdgesRatioThreshold(double threshold);

#ifdef VISP_HAVE_OGRE
  virtual void setGoodNbRayCastingAttemptsRatio(const double &ratio);
  virtual void setNbRayCastingAttemptsForVisibility(const unsigned int &attempts);
#endif

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  virtual void setKltMaskBorder(const unsigned int &e);
  virtual void setKltMaskBorder(const unsigned int &e1, const unsigned int &e2);
  virtual void setKltMaskBorder(const std::map<std::string, unsigned int> &mapOfErosions);

  virtual void setKltOpencv(const vpKltOpencv &t);
  virtual void setKltOpencv(const vpKltOpencv &t1, const vpKltOpencv &t2);
  virtual void setKltOpencv(const std::map<std::string, vpKltOpencv> &mapOfKlts);

  virtual void setKltThresholdAcceptation(double th);
#endif

  virtual void setLod(bool useLod, const std::string &name = "") VP_OVERRIDE;

  virtual void setMask(const vpImage<bool> &mask) VP_OVERRIDE;

  virtual void setMinLineLengthThresh(double minLineLengthThresh, const std::string &name = "") VP_OVERRIDE;
  virtual void setMinPolygonAreaThresh(double minPolygonAreaThresh, const std::string &name = "") VP_OVERRIDE;

  virtual void setMovingEdge(const vpMe &me);
  virtual void setMovingEdge(const vpMe &me1, const vpMe &me2);
  virtual void setMovingEdge(const std::map<std::string, vpMe> &mapOfMe);

  virtual void setNearClippingDistance(const double &dist) VP_OVERRIDE;
  virtual void setNearClippingDistance(const double &dist1, const double &dist2);
  virtual void setNearClippingDistance(const std::map<std::string, double> &mapOfDists);

  virtual void setOgreShowConfigDialog(bool showConfigDialog) VP_OVERRIDE;
  virtual void setOgreVisibilityTest(const bool &v) VP_OVERRIDE;

  virtual void setOptimizationMethod(const vpMbtOptimizationMethod &opt) VP_OVERRIDE;

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
  virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;

  virtual void setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);
  virtual void setPose(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2,
    const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);

  virtual void setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);
  virtual void setPose(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void setProjectionErrorComputation(const bool &flag) VP_OVERRIDE;

  virtual void setProjectionErrorDisplay(bool display) VP_OVERRIDE;
  virtual void setProjectionErrorDisplayArrowLength(unsigned int length) VP_OVERRIDE;
  virtual void setProjectionErrorDisplayArrowThickness(unsigned int thickness) VP_OVERRIDE;

  virtual void setReferenceCameraName(const std::string &referenceCameraName);

  virtual void setScanLineVisibilityTest(const bool &v) VP_OVERRIDE;

  virtual void setTrackerType(int type);
  virtual void setTrackerType(const std::map<std::string, int> &mapOfTrackerTypes);

  virtual void setUseDepthDenseTracking(const std::string &name, const bool &useDepthDenseTracking);
  virtual void setUseDepthNormalTracking(const std::string &name, const bool &useDepthNormalTracking);
  virtual void setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking);
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  virtual void setUseKltTracking(const std::string &name, const bool &useKltTracking);
#endif

  virtual void testTracking() VP_OVERRIDE;

  virtual void track(const vpImage<unsigned char> &I) VP_OVERRIDE;
  virtual void track(const vpImage<vpRGBa> &I_color) VP_OVERRIDE;

  virtual void track(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2);
  virtual void track(const vpImage<vpRGBa> &I_color1, const vpImage<vpRGBa> &I_color2);

  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);
  virtual void track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages);

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds);
  virtual void track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds);
#endif

  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
    std::map<std::string, unsigned int> &mapOfPointCloudWidths,
    std::map<std::string, unsigned int> &mapOfPointCloudHeights);
  virtual void track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
    std::map<std::string, unsigned int> &mapOfPointCloudWidths,
    std::map<std::string, unsigned int> &mapOfPointCloudHeights);

  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, const vpMatrix *> &mapOfPointClouds,
    std::map<std::string, unsigned int> &mapOfPointCloudWidths,
    std::map<std::string, unsigned int> &mapOfPointCloudHeights);
  virtual void track(std::map<std::string, const vpImage<vpRGBa> *> &mapOfColorImages,
    std::map<std::string, const vpMatrix *> &mapOfPointClouds,
    std::map<std::string, unsigned int> &mapOfPointCloudWidths,
    std::map<std::string, unsigned int> &mapOfPointCloudHeights);

protected:
  virtual void computeProjectionError();

  virtual void computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);

  virtual void computeVVSInit() VP_OVERRIDE;
  virtual void computeVVSInit(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);
  virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE;
  virtual void computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist);
  using vpMbTracker::computeVVSWeights;
  virtual void computeVVSWeights();

  virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, double radius, int idFace = 0,
    const std::string &name = "") VP_OVERRIDE;

  virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, double radius, int idFace = 0,
    const std::string &name = "") VP_OVERRIDE;

  virtual void initFaceFromCorners(vpMbtPolygon &polygon) VP_OVERRIDE;

  virtual void initFaceFromLines(vpMbtPolygon &polygon) VP_OVERRIDE;

  virtual void loadConfigFileXML(const std::string &configFile, bool verbose = true);
#ifdef VISP_HAVE_NLOHMANN_JSON
  virtual void loadConfigFileJSON(const std::string &configFile, bool verbose = true);
#endif

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  virtual void preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds);
#endif
  virtual void preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
    std::map<std::string, unsigned int> &mapOfPointCloudWidths,
    std::map<std::string, unsigned int> &mapOfPointCloudHeights);
  virtual void preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
    std::map<std::string, const vpMatrix *> &mapOfPointClouds,
    std::map<std::string, unsigned int> &mapOfPointCloudWidths,
    std::map<std::string, unsigned int> &mapOfPointCloudHeights);

private:
  class TrackerWrapper : public vpMbEdgeTracker,
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    public vpMbKltTracker,
#endif
    public vpMbDepthNormalTracker,
    public vpMbDepthDenseTracker
  {
    friend class vpMbGenericTracker;
#ifdef VISP_HAVE_NLOHMANN_JSON
    friend void to_json(nlohmann::json &j, const TrackerWrapper &t);
    friend void from_json(const nlohmann::json &j, TrackerWrapper &t);
#endif

  public:
    //! (s - s*)
    vpColVector m_error;
    //! Interaction matrix
    vpMatrix m_L;
    //! Type of the tracker (a combination of the above)
    int m_trackerType;
    //! Robust weights
    vpColVector m_w;
    //! Weighted error
    vpColVector m_weightedError;

    TrackerWrapper();
    explicit TrackerWrapper(int trackerType);

    virtual inline vpColVector getError() const VP_OVERRIDE { return m_error; }

    virtual inline vpColVector getRobustWeights() const VP_OVERRIDE { return m_w; }

    virtual inline int getTrackerType() const { return m_trackerType; }

    virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
      const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;
    virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
      const vpColor &col, unsigned int thickness = 1, bool displayFullModel = false) VP_OVERRIDE;

    virtual std::vector<std::vector<double> > getFeaturesForDisplay();

    virtual std::vector<std::vector<double> > getModelForDisplay(unsigned int width, unsigned int height,
      const vpHomogeneousMatrix &cMo,
      const vpCameraParameters &cam,
      bool displayFullModel = false) VP_OVERRIDE;

    virtual void init(const vpImage<unsigned char> &I) VP_OVERRIDE;

    virtual void loadConfigFile(const std::string &configFile, bool verbose = true) VP_OVERRIDE;

    virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
      const vpHomogeneousMatrix &cMo, bool verbose = false,
      const vpHomogeneousMatrix &T = vpHomogeneousMatrix()) VP_OVERRIDE;
    virtual void reInitModel(const vpImage<vpRGBa> &I_color, const std::string &cad_name,
      const vpHomogeneousMatrix &cMo, bool verbose = false,
      const vpHomogeneousMatrix &T = vpHomogeneousMatrix());

    virtual void resetTracker() VP_OVERRIDE;

    virtual void setCameraParameters(const vpCameraParameters &camera) VP_OVERRIDE;

    virtual void setClipping(const unsigned int &flags) VP_OVERRIDE;

    virtual void setFarClippingDistance(const double &dist) VP_OVERRIDE;

    virtual void setNearClippingDistance(const double &dist) VP_OVERRIDE;

    virtual void setOgreVisibilityTest(const bool &v) VP_OVERRIDE;

    virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
    virtual void setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;

    virtual void setProjectionErrorComputation(const bool &flag) VP_OVERRIDE;

    virtual void setScanLineVisibilityTest(const bool &v) VP_OVERRIDE;

    virtual void setTrackerType(int type);

    virtual void testTracking() VP_OVERRIDE;

    virtual void track(const vpImage<unsigned char> &I) VP_OVERRIDE;
    virtual void track(const vpImage<vpRGBa> &I_color) VP_OVERRIDE;
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    // Fix error: using declaration ‘using vpMbDepthDenseTracker::setPose’ conflicts with a previous
    // using declaration that occurs with g++ 4.6.3 on Ubuntu 12.04
#if !((__GNUC__ == 4) && (__GNUC_MINOR__ == 6))
    using vpMbDepthNormalTracker::track;
#endif
    using vpMbDepthDenseTracker::track;
    using vpMbEdgeTracker::track;
    virtual void track(const vpImage<unsigned char> *const ptr_I,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif

  protected:
    virtual void computeVVS(const vpImage<unsigned char> *const ptr_I);
    virtual void computeVVSInit() VP_OVERRIDE;
    virtual void computeVVSInit(const vpImage<unsigned char> *const ptr_I);
    virtual void computeVVSInteractionMatrixAndResidu() VP_OVERRIDE;
    using vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu;
    virtual void computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> *const ptr_I);
    using vpMbTracker::computeVVSWeights;
    virtual void computeVVSWeights() VP_OVERRIDE;

    virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, double radius, int idFace = 0,
      const std::string &name = "") VP_OVERRIDE;

    virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, double radius, int idFace = 0,
      const std::string &name = "") VP_OVERRIDE;

    virtual void initFaceFromCorners(vpMbtPolygon &polygon) VP_OVERRIDE;
    virtual void initFaceFromLines(vpMbtPolygon &polygon) VP_OVERRIDE;

    virtual void initMbtTracking(const vpImage<unsigned char> *const ptr_I);

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    virtual void postTracking(const vpImage<unsigned char> *const ptr_I,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
    virtual void preTracking(const vpImage<unsigned char> *const ptr_I,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud);
#endif
    virtual void postTracking(const vpImage<unsigned char> *const ptr_I = nullptr, const unsigned int pointcloud_width = 0,
      const unsigned int pointcloud_height = 0);
    virtual void preTracking(const vpImage<unsigned char> *const ptr_I = nullptr,
      const std::vector<vpColVector> *const point_cloud = nullptr,
      const unsigned int pointcloud_width = 0, const unsigned int pointcloud_height = 0);
    virtual void preTracking(const vpImage<unsigned char> *const ptr_I = nullptr,
      const vpMatrix *const point_cloud = nullptr,
      const unsigned int pointcloud_width = 0, const unsigned int pointcloud_height = 0);

    virtual void reInitModel(const vpImage<unsigned char> *const I, const vpImage<vpRGBa> *const I_color,
      const std::string &cad_name, const vpHomogeneousMatrix &cMo, bool verbose = false,
      const vpHomogeneousMatrix &T = vpHomogeneousMatrix());

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
    // Fix error: using declaration ‘using vpMbDepthDenseTracker::track’ conflicts with a previous
    // using declaration that occurs with g++ 4.6.3 on Ubuntu 12.04
#if !((__GNUC__ == 4) && (__GNUC_MINOR__ == 6))
    using vpMbDepthNormalTracker::setPose;
#endif
    using vpMbDepthDenseTracker::setPose;
#endif
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    virtual void setPose(const vpImage<unsigned char> *I, const vpImage<vpRGBa> *I_color,
      const vpHomogeneousMatrix &cdMo) VP_OVERRIDE;
#else
    virtual void setPose(const vpImage<unsigned char> *I, const vpImage<vpRGBa> *I_color,
      const vpHomogeneousMatrix &cdMo);
#endif
  };
#ifdef VISP_HAVE_NLOHMANN_JSON
  friend void to_json(nlohmann::json &j, const TrackerWrapper &t);
  friend void from_json(const nlohmann::json &j, TrackerWrapper &t);
#endif

protected:
  //! (s - s*)
  vpColVector m_error;
  //! Interaction matrix
  vpMatrix m_L;
  //! Map of camera transformation matrix between the current camera frame to
  //! the reference camera frame (cCurrent_M_cRef)
  std::map<std::string, vpHomogeneousMatrix> m_mapOfCameraTransformationMatrix;
  //! Ponderation between each feature type in the VVS stage
  std::map<vpTrackerType, double> m_mapOfFeatureFactors;
  //! Map of Model-based trackers, key is the name of the camera, value is the
  //! tracker
  std::map<std::string, TrackerWrapper *> m_mapOfTrackers;
  //! Percentage of good points over total number of points below which
  //! tracking is supposed to have failed (only for Edge tracking).
  double m_percentageGdPt;
  //! Name of the reference camera
  std::string m_referenceCameraName;
  //! Threshold below which the weight associated to a point to consider this
  //! one as an outlier (only for KLT tracking).
  double m_thresholdOutlier;
  //! Robust weights
  vpColVector m_w;
  //! Weighted error
  vpColVector m_weightedError;

  //! Number of moving-edges features
  unsigned int m_nb_feat_edge;
  //! Number of klt features
  unsigned int m_nb_feat_klt;
  //! Number of depth normal features
  unsigned int m_nb_feat_depthNormal;
  //! Number of depth dense features
  unsigned int m_nb_feat_depthDense;
};

#ifdef VISP_HAVE_NLOHMANN_JSON

#define MBT_JSON_SETTINGS_VERSION "1.0"

// Serialize tracker type enumeration
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
NLOHMANN_JSON_SERIALIZE_ENUM(vpMbGenericTracker::vpTrackerType, {
    {vpMbGenericTracker::EDGE_TRACKER, "edge"},
    {vpMbGenericTracker::KLT_TRACKER, "klt"},
    {vpMbGenericTracker::DEPTH_DENSE_TRACKER, "depthDense"},
    {vpMbGenericTracker::DEPTH_NORMAL_TRACKER, "depthNormal"}
  });
#else
NLOHMANN_JSON_SERIALIZE_ENUM(vpMbGenericTracker::vpTrackerType, {
    {vpMbGenericTracker::EDGE_TRACKER, "edge"},
    {vpMbGenericTracker::DEPTH_DENSE_TRACKER, "depthDense"},
    {vpMbGenericTracker::DEPTH_NORMAL_TRACKER, "depthNormal"}
});
#endif

/**
* @brief Serialize a tracker wrapper's settings into a JSON representation.
* \sa from_json for more details on what is serialized
* @param j The modified json object.
* @param t The tracker to serialize.
*/
inline void to_json(nlohmann::json &j, const vpMbGenericTracker::TrackerWrapper &t)
{
  // Common tracker attributes
  const static std::vector<vpMbGenericTracker::vpTrackerType> trackerTypes = {
    vpMbGenericTracker::EDGE_TRACKER,
    #if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    vpMbGenericTracker::KLT_TRACKER,
    #endif
    vpMbGenericTracker::DEPTH_DENSE_TRACKER,
    vpMbGenericTracker::DEPTH_NORMAL_TRACKER
  };
  j = nlohmann::json {
    {"camera", t.m_cam},
    {"type", flagsToJSON(t.m_trackerType, trackerTypes)},
    {"angleAppear", vpMath::deg(t.getAngleAppear())},
    {"angleDisappear", vpMath::deg(t.getAngleDisappear())},
    {"lod", {
      {"useLod", t.useLodGeneral},
      {"minLineLengthThresholdGeneral", t.minLineLengthThresholdGeneral},
      {"minPolygonAreaThresholdGeneral", t.minPolygonAreaThresholdGeneral}
    }},
    {"display", {
      {"features", t.displayFeatures},
      {"projectionError", t.m_projectionErrorDisplay}
    }},
    {"visibilityTest", {
      {"ogre", t.useOgre},
      {"scanline", t.useScanLine}
    }},
    {"clipping", {
      {"flags", clippingFlagsToJSON(t.getClipping())},
      {"near", t.getNearClippingDistance()},
      {"far", t.getFarClippingDistance()},
    }}
  };
  //Check tracker type: for each type, add settings to json if the tracker t does use the features
  //Edge tracker settings
  if (t.m_trackerType & vpMbGenericTracker::EDGE_TRACKER) {
    j["edge"] = t.me;
  }
  //KLT tracker settings
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  if (t.m_trackerType & vpMbGenericTracker::KLT_TRACKER) {
    nlohmann::json klt = nlohmann::json {
      {"maxFeatures", t.tracker.getMaxFeatures()},
      {"windowSize", t.tracker.getWindowSize()},
      {"quality", t.tracker.getQuality()},
      {"minDistance", t.tracker.getMinDistance()},
      {"harris", t.tracker.getHarrisFreeParameter()},
      {"blockSize", t.tracker.getBlockSize()},
      {"pyramidLevels", t.tracker.getPyramidLevels()}
    };
    klt["maskBorder"] = t.maskBorder;
    j["klt"] = klt;
  }
#endif
  //Depth normal settings
  if (t.m_trackerType & vpMbGenericTracker::DEPTH_NORMAL_TRACKER) {
    j["normals"] = nlohmann::json {
      {"featureEstimationMethod", t.m_depthNormalFeatureEstimationMethod},
      {"pcl", {
        {"method", t.m_depthNormalPclPlaneEstimationMethod},
        {"ransacMaxIter", t.m_depthNormalPclPlaneEstimationRansacMaxIter},
        {"ransacThreshold", t.m_depthNormalPclPlaneEstimationRansacThreshold}
      }},
      {"sampling", {
        {"x", t.m_depthNormalSamplingStepX},
        {"y", t.m_depthNormalSamplingStepY}
      }}
    };
  }
  //Depth dense settings
  if (t.m_trackerType & vpMbGenericTracker::DEPTH_DENSE_TRACKER) {
    j["dense"] = {
      {"sampling", {
        {"x", t.m_depthDenseSamplingStepX},
        {"y", t.m_depthDenseSamplingStepY}
      }}
    };
  }
}
/**
 * @brief Load configuration settings from a JSON object for a tracker wrapper.
 *
 * The settings must at the minimum contain the camera parameters #vpCameraParameters::from_json and the type of the tracker.
 *
 * The type of the tracker is serialized as a combination of flags of type vpMbGenericTracker::vpTrackerType:
 * \code{.json}
 * "type" : ["edge", "klt"]              // for a tracker that uses edges and KLT point as features
 * "type": ["depthDense", "depthNormal"] // for a tracker that operates on the depth map using normal
 *                                       // and the dense depth map features
 * \endcode
 *
 * Then for each used type of feature that is used, the corresponding settings are deserialized.
 *
 * The settings may also contain settings about clipping, LOD or face tracking.
 *
 * \sa to_json
 * @param j The JSON object containing the settings
 * @param t The tracker wrapper for which to load settings
 */
inline void from_json(const nlohmann::json &j, vpMbGenericTracker::TrackerWrapper &t)
{
  t.setCameraParameters(j.at("camera"));
  t.setTrackerType(flagsFromJSON<vpMbGenericTracker::vpTrackerType>(j.at("type")));
  //Load base settings
  if (j.contains("angleAppear")) {
    t.setAngleAppear(vpMath::rad(static_cast<double>(j.at("angleAppear"))));
  }
  if (j.contains("angleDisappear")) {
    t.setAngleDisappear(vpMath::rad(static_cast<double>(j.at("angleDisappear"))));
  }
  if (j.contains("clipping")) {
    const nlohmann::json clipping = j["clipping"];
    t.setNearClippingDistance(clipping.value("near", t.getNearClippingDistance()));
    t.setFarClippingDistance(clipping.value("far", t.getFarClippingDistance()));
    if (clipping.contains("flags")) {
      t.setClipping(flagsFromJSON<vpPolygon3D::vpPolygon3DClippingType>(clipping.at("flags")));
    }
  }
  if (j.contains("lod")) {
    const nlohmann::json lod = j["lod"];
    t.useLodGeneral = lod.value("useLod", t.useLodGeneral);
    t.minLineLengthThresholdGeneral = lod.value("minLineLengthThresholdGeneral", t.minLineLengthThresholdGeneral);
    t.minPolygonAreaThresholdGeneral = lod.value("minPolygonAreaThresholdGeneral", t.minPolygonAreaThresholdGeneral);
    t.applyLodSettingInConfig = false;
    if (t.getNbPolygon() > 0) {
      t.applyLodSettingInConfig = true;
      t.setLod(t.useLodGeneral);
      t.setMinLineLengthThresh(t.minLineLengthThresholdGeneral);
      t.setMinPolygonAreaThresh(t.minPolygonAreaThresholdGeneral);
    }
  }
  if (j.contains("display")) {
    const nlohmann::json displayJson = j["display"];
    t.setDisplayFeatures(displayJson.value("features", t.displayFeatures));
    t.setProjectionErrorDisplay(displayJson.value("projectionError", t.m_projectionErrorDisplay));
  }
  if (j.contains("visibilityTest")) {
    const nlohmann::json visJson = j["visibilityTest"];
    t.setOgreVisibilityTest(visJson.value("ogre", t.useOgre));
    t.setScanLineVisibilityTest(visJson.value("scanline", t.useScanLine));
  }

  //Check tracker type: for each type, load settings for this specific tracker type
  //Edge tracker settings
  if (t.m_trackerType & vpMbGenericTracker::EDGE_TRACKER) {
    from_json(j.at("edge"), t.me);
    t.setMovingEdge(t.me);
  }
  //KLT tracker settings
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  if (t.m_trackerType & vpMbGenericTracker::KLT_TRACKER) {
    const nlohmann::json klt = j.at("klt");
    auto &ktrack = t.tracker;
    ktrack.setMaxFeatures(klt.value("maxFeatures", 10000));
    ktrack.setWindowSize(klt.value("windowSize", 5));
    ktrack.setQuality(klt.value("quality", 0.01));
    ktrack.setMinDistance(klt.value("minDistance", 5));
    ktrack.setHarrisFreeParameter(klt.value("harris", 0.01));
    ktrack.setBlockSize(klt.value("blockSize", 3));
    ktrack.setPyramidLevels(klt.value("pyramidLevels", 3));
    t.setMaskBorder(klt.value("maskBorder", t.maskBorder));
    t.faces.getMbScanLineRenderer().setMaskBorder(t.maskBorder);
  }
#else
  if (j.contains("klt")) {
    std::cerr << "Trying to load a KLT tracker, but the ViSP dependency requirements are not met. Ignoring." << std::endl;
  }
#endif
  //Depth normal settings
  if (t.m_trackerType & vpMbGenericTracker::DEPTH_NORMAL_TRACKER) {
    const nlohmann::json n = j.at("normals");
    t.setDepthNormalFeatureEstimationMethod(n.at("featureEstimationMethod"));
    if (n.contains("pcl")) {
      const nlohmann::json pcl = n["pcl"];
      t.setDepthNormalPclPlaneEstimationMethod(pcl.at("method"));
      t.setDepthNormalPclPlaneEstimationRansacMaxIter(pcl.at("ransacMaxIter"));
      t.setDepthNormalPclPlaneEstimationRansacThreshold(pcl.at("ransacThreshold"));
    }
    if (n.contains("sampling")) {
      const nlohmann::json sampling = n.at("sampling");
      t.setDepthNormalSamplingStep(sampling.at("x"), sampling.at("y"));
    }
  }
  //Depth Dense settings
  if (t.m_trackerType & vpMbGenericTracker::DEPTH_DENSE_TRACKER) {
    const nlohmann::json dense = j.at("dense");
    if (dense.contains("sampling")) {
      const nlohmann::json sampling = dense.at("sampling");
      t.setDepthDenseSamplingStep(sampling.at("x"), sampling.at("y"));
    }
  }
}

#endif

END_VISP_NAMESPACE

#endif
