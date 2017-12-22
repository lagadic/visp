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
 * Generic model-based tracker.
 *
 *****************************************************************************/
/*!
 \file vpMbGenericTracker.h
 \brief Generic model-based tracker
*/

#ifndef __vpMbGenericTracker_h_
#define __vpMbGenericTracker_h_

#include <visp3/mbt/vpMbDepthDenseTracker.h>
#include <visp3/mbt/vpMbDepthNormalTracker.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbKltTracker.h>

class VISP_EXPORT vpMbGenericTracker : public vpMbTracker
{
public:
  enum vpTrackerType {
    EDGE_TRACKER = 1 << 0, /*!< Model-based tracking using moving edges features. */
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
    KLT_TRACKER = 1 << 1, /*!< Model-based tracking using KLT features. */
#endif
    DEPTH_NORMAL_TRACKER = 1 << 2, /*!< Model-based tracking using depth normal features. */
    DEPTH_DENSE_TRACKER = 1 << 3   /*!< Model-based tracking using depth dense features. */
  };

  vpMbGenericTracker();
  vpMbGenericTracker(const unsigned int nbCameras, const int trackerType = EDGE_TRACKER);
  explicit vpMbGenericTracker(const std::vector<int> &trackerTypes);
  vpMbGenericTracker(const std::vector<std::string> &cameraNames, const std::vector<int> &trackerTypes);

  virtual ~vpMbGenericTracker();

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                       const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
                       const vpCameraParameters &cam2, const vpColor &color, const unsigned int thickness = 1,
                       const bool displayFullModel = false);
  virtual void display(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, const vpHomogeneousMatrix &c1Mo,
                       const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1, const vpCameraParameters &cam2,
                       const vpColor &color, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                       const std::map<std::string, vpCameraParameters> &mapOfCameraParameters, const vpColor &col,
                       const unsigned int thickness = 1, const bool displayFullModel = false);
  virtual void display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                       const std::map<std::string, vpCameraParameters> &mapOfCameraParameters, const vpColor &col,
                       const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual std::vector<std::string> getCameraNames() const;

  using vpMbTracker::getCameraParameters;
  virtual void getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const;
  virtual void getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const;

  virtual std::map<std::string, int> getCameraTrackerTypes() const;

  using vpMbTracker::getClipping;
  virtual void getClipping(unsigned int &clippingFlag1, unsigned int &clippingFlag2) const;
  virtual void getClipping(std::map<std::string, unsigned int> &mapOfClippingFlags) const;

  virtual inline vpColVector getError() const { return m_error; }

  virtual vpMbHiddenFaces<vpMbtPolygon> &getFaces();
  virtual vpMbHiddenFaces<vpMbtPolygon> &getFaces(const std::string &cameraName);

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  virtual std::list<vpMbtDistanceCircle *> &getFeaturesCircle();
  virtual std::list<vpMbtDistanceKltCylinder *> &getFeaturesKltCylinder();
  virtual std::list<vpMbtDistanceKltPoints *> &getFeaturesKlt();
#endif

  virtual double getGoodMovingEdgesRatioThreshold() const;

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  virtual std::vector<vpImagePoint> getKltImagePoints() const;
  virtual std::map<int, vpImagePoint> getKltImagePointsWithId() const;

  virtual unsigned int getKltMaskBorder() const;
  virtual int getKltNbPoints() const;

  virtual vpKltOpencv getKltOpencv() const;
  virtual void getKltOpencv(vpKltOpencv &klt1, vpKltOpencv &klt2) const;
  virtual void getKltOpencv(std::map<std::string, vpKltOpencv> &mapOfKlts) const;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  virtual std::vector<cv::Point2f> getKltPoints() const;
#endif

  virtual double getKltThresholdAcceptation() const;
#endif

  virtual void getLcircle(const std::string &cameraName, std::list<vpMbtDistanceCircle *> &circlesList,
                          const unsigned int level = 0) const;
  virtual void getLcylinder(const std::string &cameraName, std::list<vpMbtDistanceCylinder *> &cylindersList,
                            const unsigned int level = 0) const;
  virtual void getLline(const std::string &cameraName, std::list<vpMbtDistanceLine *> &linesList,
                        const unsigned int level = 0) const;

  virtual vpMe getMovingEdge() const;
  virtual void getMovingEdge(vpMe &me1, vpMe &me2) const;
  virtual void getMovingEdge(std::map<std::string, vpMe> &mapOfMovingEdges) const;

  virtual unsigned int getNbPoints(const unsigned int level = 0) const;
  virtual void getNbPoints(std::map<std::string, unsigned int> &mapOfNbPoints, const unsigned int level = 0) const;

  virtual inline unsigned int getNbPolygon() const;
  virtual void getNbPolygon(std::map<std::string, unsigned int> &mapOfNbPolygons) const;

  virtual vpMbtPolygon *getPolygon(const unsigned int index);
  virtual vpMbtPolygon *getPolygon(const std::string &cameraName, const unsigned int index);

  virtual std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > >
  getPolygonFaces(const bool orderPolygons = true, const bool useVisibility = true, const bool clipPolygon = false);
  virtual void getPolygonFaces(std::map<std::string, std::vector<vpPolygon> > &mapOfPolygons,
                               std::map<std::string, std::vector<std::vector<vpPoint> > > &mapOfPoints,
                               const bool orderPolygons = true, const bool useVisibility = true,
                               const bool clipPolygon = false);

  using vpMbTracker::getPose;
  virtual void getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const;
  virtual void getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const;

  virtual inline vpColVector getRobustWeights() const { return m_w; }

  virtual void init(const vpImage<unsigned char> &I);

#ifdef VISP_HAVE_MODULE_GUI
  using vpMbTracker::initClick;
  virtual void initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                         const std::string &initFile1, const std::string &initFile2, const bool displayHelp = false);
  virtual void initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                         const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp = false);
#endif

  using vpMbTracker::initFromPoints;
  virtual void initFromPoints(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                              const std::string &initFile1, const std::string &initFile2);
  virtual void initFromPoints(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                              const std::map<std::string, std::string> &mapOfInitPoints);

  using vpMbTracker::initFromPose;
  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            const std::string &initFile1, const std::string &initFile2);
  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                            const std::map<std::string, std::string> &mapOfInitPoses);

  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);
  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                            const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void loadConfigFile(const std::string &configFile);
  virtual void loadConfigFile(const std::string &configFile1, const std::string &configFile2);
  virtual void loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles);

  using vpMbTracker::loadModel;
  virtual void loadModel(const std::string &modelFile, const bool verbose = false);
  virtual void loadModel(const std::string &modelFile1, const std::string &modelFile2, const bool verbose = false);
  virtual void loadModel(const std::map<std::string, std::string> &mapOfModelFiles, const bool verbose = false);

  virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                           const vpHomogeneousMatrix &cMo_, const bool verbose = false);
  virtual void reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                           const std::string &cad_name1, const std::string &cad_name2, const vpHomogeneousMatrix &c1Mo,
                           const vpHomogeneousMatrix &c2Mo, const bool verbose = false);
  virtual void reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                           const std::map<std::string, std::string> &mapOfModelFiles,
                           const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                           const bool verbose = false);

  virtual void resetTracker();

  virtual void setAngleAppear(const double &a);
  virtual void setAngleAppear(const double &a1, const double &a2);
  virtual void setAngleAppear(const std::map<std::string, double> &mapOfAngles);

  virtual void setAngleDisappear(const double &a);
  virtual void setAngleDisappear(const double &a1, const double &a2);
  virtual void setAngleDisappear(const std::map<std::string, double> &mapOfAngles);

  virtual void setCameraParameters(const vpCameraParameters &camera);
  virtual void setCameraParameters(const vpCameraParameters &camera1, const vpCameraParameters &camera2);
  virtual void setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters);

  virtual void setCameraTransformationMatrix(const std::string &cameraName,
                                             const vpHomogeneousMatrix &cameraTransformationMatrix);
  virtual void
  setCameraTransformationMatrix(const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix);

  virtual void setClipping(const unsigned int &flags);
  virtual void setClipping(const unsigned int &flags1, const unsigned int &flags2);
  virtual void setClipping(const std::map<std::string, unsigned int> &mapOfClippingFlags);

  virtual void setDepthDenseFilteringMaxDistance(const double maxDistance);
  virtual void setDepthDenseFilteringMethod(const int method);
  virtual void setDepthDenseFilteringMinDistance(const double minDistance);
  virtual void setDepthDenseFilteringOccupancyRatio(const double occupancyRatio);
  virtual void setDepthDenseSamplingStep(const unsigned int stepX, const unsigned int stepY);

  virtual void setDepthNormalFaceCentroidMethod(const vpMbtFaceDepthNormal::vpFaceCentroidType &method);
  virtual void setDepthNormalFeatureEstimationMethod(const vpMbtFaceDepthNormal::vpFeatureEstimationType &method);
  virtual void setDepthNormalPclPlaneEstimationMethod(const int method);
  virtual void setDepthNormalPclPlaneEstimationRansacMaxIter(const int maxIter);
  virtual void setDepthNormalPclPlaneEstimationRansacThreshold(const double thresold);
  virtual void setDepthNormalSamplingStep(const unsigned int stepX, const unsigned int stepY);

  virtual void setDisplayFeatures(const bool displayF);

  virtual void setFarClippingDistance(const double &dist);
  virtual void setFarClippingDistance(const double &dist1, const double &dist2);
  virtual void setFarClippingDistance(const std::map<std::string, double> &mapOfClippingDists);

  virtual void setFeatureFactors(const std::map<vpTrackerType, double> &mapOfFeatureFactors);

  virtual void setGoodMovingEdgesRatioThreshold(const double threshold);

#ifdef VISP_HAVE_OGRE
  virtual void setGoodNbRayCastingAttemptsRatio(const double &ratio);
  virtual void setNbRayCastingAttemptsForVisibility(const unsigned int &attempts);
#endif

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  virtual void setKltMaskBorder(const unsigned int &e);
  virtual void setKltMaskBorder(const unsigned int &e1, const unsigned int &e2);
  virtual void setKltMaskBorder(const std::map<std::string, unsigned int> &mapOfErosions);

  virtual void setKltOpencv(const vpKltOpencv &t);
  virtual void setKltOpencv(const vpKltOpencv &t1, const vpKltOpencv &t2);
  virtual void setKltOpencv(const std::map<std::string, vpKltOpencv> &mapOfKlts);

  virtual void setKltThresholdAcceptation(const double th);

#endif

  virtual void setLod(const bool useLod, const std::string &name = "");

  virtual void setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name = "");
  virtual void setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name = "");

  virtual void setMovingEdge(const vpMe &me);
  virtual void setMovingEdge(const vpMe &me1, const vpMe &me2);
  virtual void setMovingEdge(const std::map<std::string, vpMe> &mapOfMe);

  virtual void setNearClippingDistance(const double &dist);
  virtual void setNearClippingDistance(const double &dist1, const double &dist2);
  virtual void setNearClippingDistance(const std::map<std::string, double> &mapOfDists);

  virtual void setOgreShowConfigDialog(const bool showConfigDialog);
  virtual void setOgreVisibilityTest(const bool &v);

  virtual void setOptimizationMethod(const vpMbtOptimizationMethod &opt);

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo);
  virtual void setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                       const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo);
  virtual void setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void setProjectionErrorComputation(const bool &flag);

  virtual void setReferenceCameraName(const std::string &referenceCameraName);

  virtual void setScanLineVisibilityTest(const bool &v);

  virtual void setTrackerType(const int type);
  virtual void setTrackerType(const std::map<std::string, int> &mapOfTrackerTypes);

  virtual void setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking);
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  virtual void setUseKltTracking(const std::string &name, const bool &useKltTracking);
#endif

  virtual void testTracking();

  virtual void track(const vpImage<unsigned char> &I);
  virtual void track(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2);
  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);
#ifdef VISP_HAVE_PCL
  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                     std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds);
#endif
  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                     std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
                     std::map<std::string, unsigned int> &mapOfPointCloudWidths,
                     std::map<std::string, unsigned int> &mapOfPointCloudHeights);

protected:
  virtual void computeProjectionError();

  virtual void computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);

  virtual void computeVVSInit();
  virtual void computeVVSInit(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);
  virtual void computeVVSInteractionMatrixAndResidu();
  virtual void computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                    std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist);
  using vpMbTracker::computeVVSWeights;
  virtual void computeVVSWeights();

  virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                          const int idFace = 0, const std::string &name = "");

  virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, const double radius, const int idFace = 0,
                            const std::string &name = "");

  virtual void initFaceFromCorners(vpMbtPolygon &polygon);

  virtual void initFaceFromLines(vpMbtPolygon &polygon);

#ifdef VISP_HAVE_PCL
  virtual void preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                           std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> &mapOfPointClouds);
#endif
  virtual void preTracking(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                           std::map<std::string, const std::vector<vpColVector> *> &mapOfPointClouds,
                           std::map<std::string, unsigned int> &mapOfPointCloudWidths,
                           std::map<std::string, unsigned int> &mapOfPointCloudHeights);

private:
  class TrackerWrapper : public vpMbEdgeTracker,
#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))
                         public vpMbKltTracker,
#endif
                         public vpMbDepthNormalTracker,
                         public vpMbDepthDenseTracker
  {
    friend class vpMbGenericTracker;

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
    explicit TrackerWrapper(const int trackerType);

    virtual ~TrackerWrapper();

    virtual inline vpColVector getError() const { return m_error; }

    virtual inline vpColVector getRobustWeights() const { return m_w; }

    virtual inline int getTrackerType() const { return m_trackerType; }

    virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                         const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);
    virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                         const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

    virtual void init(const vpImage<unsigned char> &I);

    virtual void loadConfigFile(const std::string &configFile);

    virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                             const vpHomogeneousMatrix &cMo_, const bool verbose = false);

    virtual void resetTracker();

    virtual void setCameraParameters(const vpCameraParameters &camera);

    virtual void setClipping(const unsigned int &flags);

    virtual void setFarClippingDistance(const double &dist);

    virtual void setNearClippingDistance(const double &dist);

    virtual void setOgreVisibilityTest(const bool &v);

    virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo);

    virtual void setProjectionErrorComputation(const bool &flag);

    virtual void setScanLineVisibilityTest(const bool &v);

    virtual void setTrackerType(const int type);

    virtual void testTracking();

    virtual void track(const vpImage<unsigned char> &I);
#ifdef VISP_HAVE_PCL
    using vpMbDepthNormalTracker::track;
    using vpMbDepthDenseTracker::track;
    virtual void track(const vpImage<unsigned char> *const ptr_I = NULL,
                       const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud = nullptr);
#endif

  protected:
    virtual void computeVVS(const vpImage<unsigned char> *const ptr_I);
    virtual void computeVVSInit();
    virtual void computeVVSInit(const vpImage<unsigned char> *const ptr_I);
    virtual void computeVVSInteractionMatrixAndResidu();
    using vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu;
    virtual void computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> *const ptr_I);
    using vpMbTracker::computeVVSWeights;
    virtual void computeVVSWeights();

    virtual void initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                            const int idFace = 0, const std::string &name = "");

    virtual void initCylinder(const vpPoint &p1, const vpPoint &p2, const double radius, const int idFace = 0,
                              const std::string &name = "");

    virtual void initFaceFromCorners(vpMbtPolygon &polygon);
    virtual void initFaceFromLines(vpMbtPolygon &polygon);

    virtual void initMbtTracking(const vpImage<unsigned char> *const ptr_I);

#ifdef VISP_HAVE_PCL
    virtual void postTracking(const vpImage<unsigned char> *const ptr_I = NULL,
                              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud = nullptr);
    virtual void preTracking(const vpImage<unsigned char> *const ptr_I = NULL,
                             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &point_cloud = nullptr);
#endif
    virtual void postTracking(const vpImage<unsigned char> *const ptr_I = NULL, const unsigned int pointcloud_width = 0,
                              const unsigned int pointcloud_height = 0);
    virtual void preTracking(const vpImage<unsigned char> *const ptr_I = NULL,
                             const std::vector<vpColVector> *const point_cloud = NULL,
                             const unsigned int pointcloud_width = 0, const unsigned int pointcloud_height = 0);
  };

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
};
#endif
