#############################################################################
#
# $Id: CMakeSourceFileList.cmake,v 1.29 2008-12-17 14:45:01 fspindle Exp $
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP source file list.
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Set SRC_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update SRC_ALL variable if you add/remove a SRC_subdir 
# variable
#
# If you add/remove a directory, modify here
SET (SRC_CAMERA
  camera/vpCameraParameters.cpp 
  camera/vpMeterPixelConversion.cpp  
  camera/vpPixelMeterConversion.cpp  
  camera/calibration/vpCalibration.cpp
  camera/calibration/vpCalibrationTools.cpp
  )

IF(VISP_HAVE_XML2)
  LIST(APPEND SRC_CAMERA camera/vpXmlParserCamera.cpp)
ENDIF()

SET (SRC_COMPUTER_VISION
  computer-vision/homography-estimation/vpHomography.cpp
  computer-vision/homography-estimation/vpHomographyDLT.cpp
  computer-vision/homography-estimation/vpHomographyVVS.cpp
  computer-vision/homography-estimation/vpHomographyExtract.cpp
  computer-vision/homography-estimation/vpHomographyMalis.cpp
  computer-vision/homography-estimation/vpHomographyRansac.cpp
  computer-vision/pose-estimation/vpLevenbergMarquartd.cpp
  computer-vision/pose-estimation/vpPose.cpp
  computer-vision/pose-estimation/vpPoseFeatures.cpp
  computer-vision/pose-estimation/vpPoseDementhon.cpp
  computer-vision/pose-estimation/vpPoseLagrange.cpp
  computer-vision/pose-estimation/vpPoseLowe.cpp
  computer-vision/pose-estimation/vpPoseRansac.cpp
  computer-vision/pose-estimation/vpPoseVirtualVisualServoing.cpp
  )

SET (SRC_EXCEPTION
  exceptions/vpException.cpp
  )

SET (SRC_DEVICE_FRAMEGRABBER
  device/framegrabber/disk/vpDiskGrabber.cpp
  )

IF(VISP_HAVE_DC1394_2)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/1394/vp1394TwoGrabber.cpp)
ENDIF()
IF(VISP_HAVE_CMU1394)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/1394/vp1394CMUGrabber.cpp)
ENDIF()
IF(VISP_HAVE_V4L2)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/v4l2/vpV4l2Grabber.cpp)
ENDIF()
IF(VISP_HAVE_DIRECTSHOW)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/directshow/vpDirectShowGrabber.cpp)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/directshow/vpDirectShowGrabberImpl.cpp)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/directshow/vpDirectShowDevice.cpp)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/directshow/vpDirectShowSampleGrabberI.cpp)
ENDIF()
IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_DEVICE_FRAMEGRABBER device/framegrabber/OpenCV/vpOpenCVGrabber.cpp)
ENDIF()

SET (SRC_IMAGE
  image/vpColor.cpp
  image/vpImageConvert.cpp
  image/vpImageFilter.cpp
  image/vpImageIo.cpp
  image/vpImageTools.cpp
  image/vpRGBa.cpp
  image/vpImagePoint.cpp
  )

SET (SRC_KEY_POINT
  key-point/vpBasicKeyPoint.cpp
  )

IF(VISP_HAVE_OPENCV_NONFREE)
  LIST(APPEND SRC_KEY_POINT key-point/vpKeyPointSurf.cpp)
ENDIF()
IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_KEY_POINT key-point/vpPlanarObjectDetector.cpp)
  LIST(APPEND SRC_KEY_POINT key-point/vpFernClassifier.cpp)
ENDIF()

IF(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES)
  SET (SRC_DEVICE_KINECT
    device/kinect/vpKinect.cpp
  )
ENDIF()

SET (SRC_DEVICE_LASERSCANNER
  device/laserscanner/sick/vpSickLDMRS.cpp
  )

IF(VISP_HAVE_PARPORT)
  SET (SRC_DEVICE_LIGHT
    device/light/vpRingLight.cpp
    )
ENDIF()

SET (SRC_MATH
  math/kalman/vpKalmanFilter.cpp
  math/kalman/vpLinearKalmanFilterInstantiation.cpp
  math/matrix/vpColVector.cpp
  math/matrix/vpMatrix.cpp
  math/matrix/vpMatrix_lu.cpp
  math/matrix/vpMatrix_qr.cpp
  math/matrix/vpMatrix_svd.cpp
  math/matrix/vpMatrix_covariance.cpp
  math/matrix/vpRowVector.cpp
  math/matrix/vpSubMatrix.cpp
  math/matrix/vpSubColVector.cpp
  math/matrix/vpSubRowVector.cpp
  math/misc/vpMath.cpp 
  math/misc/vpNoise.cpp 
  math/misc/vpHinkley.cpp 
  math/robust/vpRobust.cpp
  math/robust/vpScale.cpp
  math/spline/vpBSpline.cpp
  math/spline/vpNurbs.cpp
  math/transformation/vpExponentialMap.cpp 
  math/transformation/vpForceTwistMatrix.cpp
  math/transformation/vpHomogeneousMatrix.cpp
  math/transformation/vpPoseVector.cpp
  math/transformation/vpQuaternionVector.cpp
  math/transformation/vpRotationMatrix.cpp
  math/transformation/vpRotationVector.cpp
  math/transformation/vpRxyzVector.cpp
  math/transformation/vpRzyxVector.cpp
  math/transformation/vpRzyzVector.cpp
  math/transformation/vpThetaUVector.cpp
  math/transformation/vpTranslationVector.cpp
  math/transformation/vpVelocityTwistMatrix.cpp
  )

IF(VISP_HAVE_LAPACK)
  LIST(APPEND SRC_MATH math/matrix/vpMatrix_cholesky.cpp)
ENDIF()

SET (SRC_ROBOT
  robot/robot/vpRobot.cpp
  robot/robot/vpRobotTemplate.cpp
  robot/real-robot/afma4/vpAfma4.cpp
  robot/real-robot/afma6/vpAfma6.cpp
  robot/real-robot/biclops/vpBiclops.cpp
  robot/real-robot/ptu46/vpPtu46.cpp
  robot/real-robot/viper/vpViper.cpp
  robot/real-robot/viper/vpViper650.cpp
  robot/real-robot/viper/vpViper850.cpp
  robot/simulator-robot/vpRobotCamera.cpp
  robot/simulator-robot/vpRobotSimulator.cpp
  robot/simulator-robot/vpSimulatorCamera.cpp
  robot/simulator-robot/vpSimulatorPioneer.cpp
  robot/simulator-robot/vpSimulatorPioneerPan.cpp
  )

IF(WIN32 OR VISP_HAVE_PTHREAD)
  list(APPEND SRC_ROBOT robot/simulator-robot/vpRobotWireFrameSimulator.cpp)
  list(APPEND SRC_ROBOT robot/simulator-robot/vpSimulatorAfma6.cpp)
  list(APPEND SRC_ROBOT robot/simulator-robot/vpSimulatorViper850.cpp)
ENDIF()

IF(VISP_HAVE_AFMA4)
  LIST(APPEND SRC_ROBOT robot/real-robot/afma4/vpRobotAfma4.cpp)
ENDIF()
IF(UNIX)
  LIST(APPEND SRC_ROBOT robot/real-robot/afma4/vpServolens.cpp)
ENDIF()
IF(VISP_HAVE_AFMA6)
  LIST(APPEND SRC_ROBOT robot/real-robot/afma6/vpRobotAfma6.cpp)
ENDIF()
IF(VISP_HAVE_BICLOPS)
  LIST(APPEND SRC_ROBOT robot/real-robot/biclops/vpRobotBiclopsController.cpp)
  LIST(APPEND SRC_ROBOT robot/real-robot/biclops/vpRobotBiclops.cpp)
ENDIF()
IF(VISP_HAVE_PTU46)
  LIST(APPEND SRC_ROBOT robot/real-robot/ptu46/vpRobotPtu46.cpp)
ENDIF()
IF(VISP_HAVE_PIONEER)
  LIST(APPEND SRC_ROBOT robot/real-robot/pioneer/vpRobotPioneer.cpp)
ENDIF()
IF(VISP_HAVE_VIPER650)
  LIST(APPEND SRC_ROBOT robot/real-robot/viper/vpRobotViper650.cpp)
ENDIF()
IF(VISP_HAVE_VIPER850)
  LIST(APPEND SRC_ROBOT robot/real-robot/viper/vpRobotViper850.cpp)
ENDIF()

SET (SRC_SERVO
  servo/vpAdaptiveGain.cpp
  servo/vpServo.cpp
  servo/vpServoData.cpp
  servo/vpServoDisplay.cpp
  )

SET (SRC_SIMULATOR
  simulator/image-simulator/vpImageSimulator.cpp
  simulator/wireframe-simulator/vpWireFrameSimulator.cpp
  simulator/wireframe-simulator/core/vpArit.c
  simulator/wireframe-simulator/core/vpAritio.c
  simulator/wireframe-simulator/core/vpBound.c
  simulator/wireframe-simulator/core/vpBoundio.c
  simulator/wireframe-simulator/core/vpClipping.c
  simulator/wireframe-simulator/core/vpDisplay.c
  simulator/wireframe-simulator/core/vpKeyword.c
  simulator/wireframe-simulator/core/vpLex.c
  simulator/wireframe-simulator/core/vpMyio.c
  simulator/wireframe-simulator/core/vpParser.c
  simulator/wireframe-simulator/core/vpProjection.c
  simulator/wireframe-simulator/core/vpRfstack.c
  simulator/wireframe-simulator/core/vpSkipio.c
  simulator/wireframe-simulator/core/vpTmstack.c
  simulator/wireframe-simulator/core/vpToken.c
  simulator/wireframe-simulator/core/vpViewio.c
  simulator/wireframe-simulator/core/vpVwstack.c
  )
IF(VISP_HAVE_X11 OR VISP_HAVE_GDI OR VISP_HAVE_OPENCV OR VISP_HAVE_D3D9 OR VISP_HAVE_GTK)
  list(APPEND SRC_SIMULATOR simulator/coin-simulator/vpProjectionDisplay.cpp)
ENDIF()
IF(VISP_HAVE_COIN_AND_GUI)
  LIST(APPEND SRC_SIMULATOR simulator/coin-simulator/vpAR.cpp)
  LIST(APPEND SRC_SIMULATOR simulator/coin-simulator/vpSimulator.cpp)
  LIST(APPEND SRC_SIMULATOR simulator/coin-simulator/vpViewer.cpp)
ENDIF()
IF(VISP_HAVE_OGRE)
  LIST(APPEND SRC_SIMULATOR simulator/ogre-simulator/vpAROgre.cpp)
ENDIF()

SET (SRC_TOOLS
  tools/geometry/vpPlane.cpp
  tools/geometry/vpRect.cpp
  tools/geometry/vpTriangle.cpp
  tools/geometry/vpPolygon.cpp
  tools/histogram/vpHistogram.cpp
  tools/histogram/vpHistogramPeak.cpp
  tools/histogram/vpHistogramValey.cpp
  tools/io/vpIoTools.cpp
  tools/io/vpKeyboard.cpp
  tools/io/vpParseArgv.cpp
  tools/time/vpTime.cpp
  )

IF(VISP_HAVE_X11 OR VISP_HAVE_GDI OR VISP_HAVE_OPENCV OR VISP_HAVE_D3D9 OR VISP_HAVE_GTK)
  list(APPEND SRC_TOOLS tools/plot/vpPlot.cpp)
  list(APPEND SRC_TOOLS tools/plot/vpPlotCurve.cpp)
  list(APPEND SRC_TOOLS tools/plot/vpPlotGraph.cpp)
ENDIF()
IF(VISP_HAVE_XML2)
  LIST(APPEND SRC_TOOLS tools/xml/vpXmlParser.cpp)
ENDIF()

IF(VISP_HAVE_PARPORT)
  LIST(APPEND SRC_TOOLS tools/io/vpParallelPort.cpp)
ENDIF()

SET (SRC_TRACKING
  tracking/dots/vpDot2.cpp
  tracking/dots/vpDot.cpp
  tracking/feature-builder/vpFeatureBuilderEllipse.cpp
  tracking/feature-builder/vpFeatureBuilderLine.cpp
  tracking/feature-builder/vpFeatureBuilderPoint3D.cpp
  tracking/feature-builder/vpFeatureBuilderPoint.cpp
  tracking/feature-builder/vpFeatureBuilderPointPolar.cpp
  tracking/feature-builder/vpFeatureBuilderSegment.cpp
  tracking/feature-builder/vpFeatureBuilderVanishingPoint.cpp
  tracking/forward-projection/vpCircle.cpp
  tracking/forward-projection/vpCylinder.cpp
  tracking/forward-projection/vpForwardProjection.cpp
  tracking/forward-projection/vpLine.cpp
  tracking/forward-projection/vpPoint.cpp
  tracking/forward-projection/vpSphere.cpp
  tracking/general-tracking-issues/vpTracker.cpp
  tracking/moving-edges/vpMe.cpp
  tracking/moving-edges/vpMeEllipse.cpp
  tracking/moving-edges/vpMeLine.cpp
  tracking/moving-edges/vpMeSite.cpp
  tracking/moving-edges/vpMeTracker.cpp
  tracking/moving-edges/vpMeNurbs.cpp

  tracking/mbt/vpMbTracker.cpp
  tracking/mbt/edge/vpMbEdgeTracker.cpp
  tracking/mbt/edge/vpMbtDistanceCylinder.cpp
  tracking/mbt/edge/vpMbtDistanceLine.cpp
  tracking/mbt/edge/vpMbtPolygon.cpp
  tracking/mbt/edge/vpMbtMeLine.cpp

  tracking/moments/vpMoment.cpp
  tracking/moments/vpMomentAlpha.cpp
  tracking/moments/vpMomentArea.cpp
  tracking/moments/vpMomentAreaNormalized.cpp
  tracking/moments/vpMomentBasic.cpp
  tracking/moments/vpMomentCentered.cpp
  tracking/moments/vpMomentCInvariant.cpp
  tracking/moments/vpMomentCommon.cpp
  tracking/moments/vpMomentDatabase.cpp
  tracking/moments/vpMomentGravityCenter.cpp
  tracking/moments/vpMomentGravityCenterNormalized.cpp
  tracking/moments/vpMomentObject.cpp

  tracking/template-tracker/vpTemplateTracker.cpp
  tracking/template-tracker/ssd/vpTemplateTrackerSSD.cpp
  tracking/template-tracker/ssd/vpTemplateTrackerSSDESM.cpp
  tracking/template-tracker/ssd/vpTemplateTrackerSSDForwardAdditional.cpp
  tracking/template-tracker/ssd/vpTemplateTrackerSSDForwardCompositional.cpp
  tracking/template-tracker/ssd/vpTemplateTrackerSSDInverseCompositional.cpp
  tracking/template-tracker/zncc/vpTemplateTrackerZNCC.cpp
  tracking/template-tracker/zncc/vpTemplateTrackerZNCCForwardAdditional.cpp
  tracking/template-tracker/zncc/vpTemplateTrackerZNCCInverseCompositional.cpp
  tracking/template-tracker/tools/vpTemplateTrackerBSpline.cpp
  tracking/template-tracker/tools/vpTemplateTrackerZone.cpp
  tracking/template-tracker/tools/vpTemplateTrackerTriangle.cpp
  tracking/template-tracker/warp/vpTemplateTrackerWarp.cpp
  tracking/template-tracker/warp/vpTemplateTrackerWarpAffine.cpp
  tracking/template-tracker/warp/vpTemplateTrackerWarpHomography.cpp
  tracking/template-tracker/warp/vpTemplateTrackerWarpHomographySL3.cpp
  tracking/template-tracker/warp/vpTemplateTrackerWarpSRT.cpp
  tracking/template-tracker/warp/vpTemplateTrackerWarpTranslation.cpp
  )

if(VISP_HAVE_XML2)
  list(APPEND SRC_TRACKING tracking/mbt/vpMbXmlParser.cpp)
  list(APPEND SRC_TRACKING tracking/mbt/edge/vpMbtXmlParser.cpp)
  list(APPEND SRC_TRACKING tracking/mbt/klt/vpMbtKltXmlParser.cpp)
  list(APPEND SRC_TRACKING tracking/mbt/hybrid/vpMbtEdgeKltXmlParser.cpp)
endif()

IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_TRACKING tracking/klt/vpKltOpencv.cpp)
  LIST(APPEND SRC_TRACKING tracking/mbt/hybrid/vpMbEdgeKltTracker.cpp)
  LIST(APPEND SRC_TRACKING tracking/mbt/klt/vpMbtKltPolygon.cpp)
  LIST(APPEND SRC_TRACKING tracking/mbt/klt/vpMbKltTracker.cpp)
ENDIF()

SET (SRC_VIDEO
  video/vpVideoReader.cpp
  video/vpVideoWriter.cpp
  )

IF(VISP_HAVE_FFMPEG)
  LIST(APPEND SRC_VIDEO video/vpFFMPEG.cpp)
ENDIF()

SET (SRC_DEVICE_DISPLAY
  device/display/vpDisplay.cpp
  )

IF(VISP_HAVE_GTK)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/vpDisplayGTK.cpp)
ENDIF()
IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/vpDisplayOpenCV.cpp)
ENDIF()
IF(VISP_HAVE_X11)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/vpDisplayX.cpp)
ENDIF()

IF(WIN32)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpDisplayWin32.cpp)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpWin32Window.cpp)
ENDIF()
IF(VISP_HAVE_GDI)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpGDIRenderer.cpp)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpWin32API.cpp)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpDisplayGDI.cpp)
ENDIF()
IF(VISP_HAVE_D3D9)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpD3DRenderer.cpp)
  LIST(APPEND SRC_DEVICE_DISPLAY device/display/windows/vpDisplayD3D.cpp)
ENDIF()

SET (SRC_VISUAL_FEATURE
  visual-feature/vpBasicFeature.cpp
  visual-feature/vpFeatureDepth.cpp
  visual-feature/vpFeatureDisplay.cpp
  visual-feature/vpFeatureEllipse.cpp
  visual-feature/vpFeatureLine.cpp
  visual-feature/vpFeatureLuminance.cpp
  visual-feature/vpFeatureMoment.cpp
  visual-feature/vpFeatureMomentAlpha.cpp
  visual-feature/vpFeatureMomentArea.cpp
  visual-feature/vpFeatureMomentAreaNormalized.cpp
  visual-feature/vpFeatureMomentBasic.cpp
  visual-feature/vpFeatureMomentCentered.cpp
  visual-feature/vpFeatureMomentCInvariant.cpp
  visual-feature/vpFeatureMomentCommon.cpp
  visual-feature/vpFeatureMomentDatabase.cpp
  visual-feature/vpFeatureMomentGravityCenter.cpp
  visual-feature/vpFeatureMomentGravityCenterNormalized.cpp
  visual-feature/vpFeaturePoint3D.cpp
  visual-feature/vpFeaturePoint.cpp
  visual-feature/vpFeaturePointPolar.cpp
  visual-feature/vpFeatureThetaU.cpp
  visual-feature/vpFeatureTranslation.cpp
  visual-feature/vpFeatureVanishingPoint.cpp
  visual-feature/vpFeatureSegment.cpp
  visual-feature/vpGenericFeature.cpp
  )

SET (SRC_NETWORK
  network/vpNetwork.cpp
  network/vpServer.cpp
  network/vpClient.cpp
  network/vpRequest.cpp
  )

SET (SRC_ALL
  ${SRC_CAMERA}
  ${SRC_COMPUTER_VISION}
  ${SRC_EXCEPTION}
  ${SRC_DEVICE_DISPLAY}
  ${SRC_DEVICE_FRAMEGRABBER}
  ${SRC_DEVICE_KINECT}
  ${SRC_DEVICE_LASERSCANNER}
  ${SRC_DEVICE_LIGHT}
  ${SRC_IMAGE}
  ${SRC_KEY_POINT}
  ${SRC_MATH}
  ${SRC_ROBOT}
  ${SRC_SERVO}
  ${SRC_SIMULATOR}
  ${SRC_TOOLS}
  ${SRC_TRACKING}
  ${SRC_VIDEO}
  ${SRC_VISUAL_FEATURE}
  ${SRC_NETWORK}
  )
 
