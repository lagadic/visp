/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2016 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr/download/ for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Model-based edge tracker with multiple cameras.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
 \file vpMbEdgeMultiTracker.h
 \brief Model-based edge tracker with multiple cameras.
*/

#ifndef __vpMbEdgeMultiTracker_h__
#define __vpMbEdgeMultiTracker_h__

#include <iostream>
#include <vector>

#include <visp3/mbt/vpMbEdgeTracker.h>

/*!
  \class vpMbEdgeMultiTracker
  \ingroup group_mbt_trackers
  \brief Make the complete stereo (or more) tracking of an object by using its
  CAD model.

  This class allows to track an object or a scene given its 3D model.
  The \ref tutorial-tracking-mb-stereo is also a good starting point to use
  this class.

  The tracker requires the knowledge of the 3D model that could be provided in
  a vrml or in a cao file. The cao format is described in loadCAOModel(). It
  may also use an xml file used to tune the behavior of the tracker and an
  init file used to compute the pose at the very first image.
*/
class VISP_EXPORT vpMbEdgeMultiTracker : public vpMbEdgeTracker
{
protected:
  //! Map of camera transformation matrix between the current camera frame to
  //! the reference camera frame (cCurrent_M_cRef)
  std::map<std::string, vpHomogeneousMatrix> m_mapOfCameraTransformationMatrix;

  //! Map of Model-based edge trackers
  std::map<std::string, vpMbEdgeTracker *> m_mapOfEdgeTrackers;

  //! Map of pyramidal images for each camera
  std::map<std::string, std::vector<const vpImage<unsigned char> *> > m_mapOfPyramidalImages;

  //! Name of the reference camera
  std::string m_referenceCameraName;
  //! Interaction matrix
  vpMatrix m_L_edgeMulti;
  //! (s - s*)
  vpColVector m_error_edgeMulti;
  //! Robust weights
  vpColVector m_w_edgeMulti;
  //! Weighted error
  vpColVector m_weightedError_edgeMulti;

public:
  // Default constructor <==> equivalent to vpMbEdgeTracker
  vpMbEdgeMultiTracker();
  // Constructor with a specified number of cameras, camera names are
  // generated
  explicit vpMbEdgeMultiTracker(const unsigned int nbCameras);
  // Constructor with a specified list of camera names
  explicit vpMbEdgeMultiTracker(const std::vector<std::string> &cameraNames);

  virtual ~vpMbEdgeMultiTracker();

  /** @name Inherited functionalities from vpMbEdgeMultiTracker */
  //@{
  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &cam_,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_, const vpCameraParameters &cam_,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                       const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1,
                       const vpCameraParameters &cam2, const vpColor &col, const unsigned int thickness = 1,
                       const bool displayFullModel = false);

  virtual void display(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, const vpHomogeneousMatrix &c1Mo,
                       const vpHomogeneousMatrix &c2Mo, const vpCameraParameters &cam1, const vpCameraParameters &cam2,
                       const vpColor &col, const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                       const std::map<std::string, vpCameraParameters> &mapOfCameraParameters, const vpColor &col,
                       const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual void display(const std::map<std::string, const vpImage<vpRGBa> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                       const std::map<std::string, vpCameraParameters> &mapOfCameraParameters, const vpColor &col,
                       const unsigned int thickness = 1, const bool displayFullModel = false);

  virtual std::vector<std::string> getCameraNames() const;

  virtual void getCameraParameters(vpCameraParameters &camera) const;
  virtual void getCameraParameters(vpCameraParameters &cam1, vpCameraParameters &cam2) const;
  virtual void getCameraParameters(const std::string &cameraName, vpCameraParameters &camera) const;
  virtual void getCameraParameters(std::map<std::string, vpCameraParameters> &mapOfCameraParameters) const;

  using vpMbTracker::getClipping;
  virtual unsigned int getClipping(const std::string &cameraName) const;

  virtual vpMbHiddenFaces<vpMbtPolygon> &getFaces();
  virtual vpMbHiddenFaces<vpMbtPolygon> &getFaces(const std::string &cameraName);
  virtual std::map<std::string, vpMbHiddenFaces<vpMbtPolygon> > getFaces() const;

  void getLcircle(std::list<vpMbtDistanceCircle *> &circlesList, const unsigned int level = 0) const;
  virtual void getLcircle(const std::string &cameraName, std::list<vpMbtDistanceCircle *> &circlesList,
                          const unsigned int level = 0) const;

  void getLcylinder(std::list<vpMbtDistanceCylinder *> &cylindersList, const unsigned int level = 0) const;
  virtual void getLcylinder(const std::string &cameraName, std::list<vpMbtDistanceCylinder *> &cylindersList,
                            const unsigned int level = 0) const;

  void getLline(std::list<vpMbtDistanceLine *> &linesList, const unsigned int level = 0) const;
  virtual void getLline(const std::string &cameraName, std::list<vpMbtDistanceLine *> &linesList,
                        const unsigned int level = 0) const;

  virtual void getMovingEdge(vpMe &p_me) const;
  virtual vpMe getMovingEdge() const;
  virtual void getMovingEdge(const std::string &cameraName, vpMe &p_me) const;
  virtual vpMe getMovingEdge(const std::string &cameraName) const;

  virtual unsigned int getNbPoints(const unsigned int level = 0) const;
  virtual unsigned int getNbPoints(const std::string &cameraName, const unsigned int level = 0) const;

  virtual unsigned int getNbPolygon() const;
  virtual unsigned int getNbPolygon(const std::string &cameraName) const;
  virtual std::map<std::string, unsigned int> getMultiNbPolygon() const;

  /*!
    Get the number of cameras.

    \return The number of cameras.
  */
  inline unsigned int getNumberOfCameras() const { return (unsigned int)m_mapOfEdgeTrackers.size(); }

  using vpMbTracker::getPose;
  virtual void getPose(vpHomogeneousMatrix &c1Mo, vpHomogeneousMatrix &c2Mo) const;
  virtual void getPose(const std::string &cameraName, vpHomogeneousMatrix &cMo_) const;
  virtual void getPose(std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses) const;

  virtual inline vpColVector getError() const { return m_error_edgeMulti; }

  virtual inline vpColVector getRobustWeights() const { return m_w_edgeMulti; }

  void init(const vpImage<unsigned char> &I);

#ifdef VISP_HAVE_MODULE_GUI
  virtual void initClick(const vpImage<unsigned char> &I, const std::vector<vpPoint> &points3D_list,
                         const std::string &displayFile = "");

  virtual void initClick(const vpImage<unsigned char> &I, const std::string &initFile, const bool displayHelp = false);

  virtual void initClick(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                         const std::string &initFile1, const std::string &initFile2, const bool displayHelp = false,
                         const bool firstCameraIsReference = true);

  virtual void initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                         const std::string &initFile, const bool displayHelp = false);

  virtual void initClick(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                         const std::map<std::string, std::string> &mapOfInitFiles, const bool displayHelp = false);
#endif

  virtual void initFromPose(const vpImage<unsigned char> &I, const std::string &initFile);
  virtual void initFromPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);
  virtual void initFromPose(const vpImage<unsigned char> &I, const vpPoseVector &cPo);

  virtual void initFromPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                            const bool firstCameraIsReference = true);
  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                            const vpHomogeneousMatrix &cMo_);
  virtual void initFromPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                            const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void loadConfigFile(const std::string &configFile);

  virtual void loadConfigFile(const std::string &configFile1, const std::string &configFile2,
                              const bool firstCameraIsReference = true);

  virtual void loadConfigFile(const std::map<std::string, std::string> &mapOfConfigFiles);

  virtual void loadModel(const std::string &modelFile, const bool verbose = false);

  virtual void reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                           const vpHomogeneousMatrix &cMo_, const bool verbose = false);
  virtual void reInitModel(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                           const std::string &cad_name, const vpHomogeneousMatrix &c1Mo,
                           const vpHomogeneousMatrix &c2Mo, const bool verbose = false,
                           const bool firstCameraIsReference = true);
  virtual void reInitModel(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                           const std::string &cad_name,
                           const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses,
                           const bool verbose = false);

  virtual void resetTracker();

  virtual void setAngleAppear(const double &a);
  virtual void setAngleDisappear(const double &a);

  virtual void setCameraParameters(const vpCameraParameters &camera);

  virtual void setCameraParameters(const vpCameraParameters &camera1, const vpCameraParameters &camera2,
                                   const bool firstCameraIsReference = true);

  virtual void setCameraParameters(const std::string &cameraName, const vpCameraParameters &camera);

  virtual void setCameraParameters(const std::map<std::string, vpCameraParameters> &mapOfCameraParameters);

  virtual void setCameraTransformationMatrix(const std::string &cameraName,
                                             const vpHomogeneousMatrix &cameraTransformationMatrix);

  virtual void
  setCameraTransformationMatrix(const std::map<std::string, vpHomogeneousMatrix> &mapOfTransformationMatrix);

  virtual void setClipping(const unsigned int &flags);
  virtual void setClipping(const std::string &cameraName, const unsigned int &flags);

  virtual void setCovarianceComputation(const bool &flag);

  virtual void setDisplayFeatures(const bool displayF);

  virtual void setFarClippingDistance(const double &dist);
  virtual void setFarClippingDistance(const std::string &cameraName, const double &dist);

  virtual void setGoodMovingEdgesRatioThreshold(const double threshold);

#ifdef VISP_HAVE_OGRE
  /*!
    Set the ratio of visibility attempts that has to be successful to consider
    a polygon as visible.

    \sa setNbRayCastingAttemptsForVisibility(const unsigned int &)

    \param ratio : Ratio of succesful attempts that has to be considered.
    Value has to be between 0.0 (0%) and 1.0 (100%).
  */
  void setGoodNbRayCastingAttemptsRatio(const double &ratio);

  /*!
    Set the number of rays that will be sent toward each polygon for
    visibility test. Each ray will go from the optic center of the camera to a
    random point inside the considered polygon.

    \sa setGoodNbRayCastingAttemptsRatio(const unsigned int &)

    \param attempts Number of rays to be sent.
  */
  void setNbRayCastingAttemptsForVisibility(const unsigned int &attempts);
#endif

  virtual void setLod(const bool useLod, const std::string &name = "");
  virtual void setLod(const bool useLod, const std::string &cameraName, const std::string &name);

  virtual void setMinLineLengthThresh(const double minLineLengthThresh, const std::string &name = "");
  virtual void setMinLineLengthThresh(const double minLineLengthThresh, const std::string &cameraName,
                                      const std::string &name);

  virtual void setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &name = "");
  virtual void setMinPolygonAreaThresh(const double minPolygonAreaThresh, const std::string &cameraName,
                                       const std::string &name);

  virtual void setMovingEdge(const vpMe &me);
  virtual void setMovingEdge(const std::string &cameraName, const vpMe &me);

  virtual void setNearClippingDistance(const double &dist);
  virtual void setNearClippingDistance(const std::string &cameraName, const double &dist);

  /*!
    Enable/Disable the appearance of Ogre config dialog on startup.

    \warning This method has only effect when Ogre is used and Ogre visibility
    test is enabled using setOgreVisibilityTest() with true parameter.

    \param showConfigDialog : if true, shows Ogre dialog window (used to set
    Ogre rendering options) when Ogre visibility is enabled. By default, this
    functionality is turned off.
  */
  virtual void setOgreShowConfigDialog(const bool showConfigDialog);

  virtual void setOgreVisibilityTest(const bool &v);

  virtual void setOptimizationMethod(const vpMbtOptimizationMethod &opt);

  virtual void setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo);

  virtual void setPose(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                       const vpHomogeneousMatrix &c1Mo, const vpHomogeneousMatrix &c2Mo,
                       const bool firstCameraIsReference = true);

  virtual void setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                       const vpHomogeneousMatrix &cMo_);

  virtual void setPose(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                       const std::map<std::string, vpHomogeneousMatrix> &mapOfCameraPoses);

  virtual void setProjectionErrorComputation(const bool &flag);

  virtual void setReferenceCameraName(const std::string &referenceCameraName);

  virtual void setScales(const std::vector<bool> &scales);

  virtual void setScanLineVisibilityTest(const bool &v);

  virtual void setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking);

  virtual void track(const vpImage<unsigned char> &I);
  virtual void track(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2);
  virtual void track(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages);
  //@}

protected:
  enum FeatureType { LINE, CYLINDER, CIRCLE };

  /** @name Protected Member Functions Inherited from vpMbEdgeMultiTracker */
  //@{
  virtual void cleanPyramid(std::map<std::string, std::vector<const vpImage<unsigned char> *> > &pyramid);

  virtual void computeProjectionError();

  virtual void computeVVS(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages, const unsigned int lvl);
  virtual void computeVVSFirstPhasePoseEstimation(const unsigned int iter, bool &isoJoIdentity_);
  virtual void computeVVSInit();
  virtual void computeVVSInteractionMatrixAndResidu();
  using vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu;
  virtual void computeVVSInteractionMatrixAndResidu(std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                                                    std::map<std::string, vpVelocityTwistMatrix> &mapOfVelocityTwist);
  virtual void computeVVSWeights();
  using vpMbTracker::computeVVSWeights;

  virtual void initPyramid(const std::map<std::string, const vpImage<unsigned char> *> &mapOfImages,
                           std::map<std::string, std::vector<const vpImage<unsigned char> *> > &pyramid);
  //@}
};

#endif
