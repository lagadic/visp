#############################################################################
#
# $Id: CMakeSourceFileList.cmake,v 1.29 2008-12-17 14:45:01 fspindle Exp $
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
  camera/vpXmlParserCamera.cpp
  camera/calibration/vpCalibration.cpp
  camera/calibration/vpCalibrationTools.cpp
  )

SET (SRC_COMPUTER_VISION
  computer-vision/homography-estimation/vpHomography.cpp
  computer-vision/homography-estimation/vpHomographyDLT.cpp
  computer-vision/homography-estimation/vpHomographyVVS.cpp
  computer-vision/homography-estimation/vpHomographyExtract.cpp
  computer-vision/homography-estimation/vpHomographyMalis.cpp
  computer-vision/homography-estimation/vpHomographyRansac.cpp
  computer-vision/pose-estimation/vpLevenbergMarquartd.cpp
  computer-vision/pose-estimation/vpPose.cpp
  computer-vision/pose-estimation/vpPoseDementhon.cpp
  computer-vision/pose-estimation/vpPoseLagrange.cpp
  computer-vision/pose-estimation/vpPoseLowe.cpp
  computer-vision/pose-estimation/vpPoseRansac.cpp
  computer-vision/pose-estimation/vpPoseVirtualVisualServoing.cpp
  )

SET (SRC_EXCEPTION
  exceptions/vpException.cpp
  )

SET (SRC_FRAMEGRABBER_DEVICE
  framegrabber-device/disk/vpDiskGrabber.cpp
  )

IF(VISP_HAVE_DC1394_1)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/1394/vp1394Grabber.cpp)
ENDIF()
IF(VISP_HAVE_DC1394_2)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/1394/vp1394TwoGrabber.cpp)
ENDIF()
IF(VISP_HAVE_V4L2)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/v4l2/vpV4l2Grabber.cpp)
ENDIF()
IF(VISP_HAVE_DIRECTSHOW)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/directshow/vpDirectShowGrabber.cpp)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/directshow/vpDirectShowGrabberImpl.cpp)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/directshow/vpDirectShowDevice.cpp)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/directshow/vpDirectShowSampleGrabberI.cpp)
ENDIF()
IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_FRAMEGRABBER_DEVICE framegrabber-device/OpenCV/vpOpenCVGrabber.cpp)
ENDIF()

SET (SRC_IMAGE
  image/vpColor.cpp
  image/vpImageConvert.cpp
  image/vpImageFilter.cpp
  image/vpImageIo.cpp
  image/vpImageIoPnm.cpp
  image/vpImageTools.cpp
  image/vpRGBa.cpp
  image/vpImagePoint.cpp
  )

SET (SRC_KEY_POINT
  key-point/vpBasicKeyPoint.cpp
  )

IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_KEY_POINT key-point/vpKeyPointSurf.cpp)
  LIST(APPEND SRC_KEY_POINT key-point/vpPlanarObjectDetector.cpp)
  LIST(APPEND SRC_KEY_POINT key-point/vpFernClassifier.cpp)
ENDIF()

IF(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES)
  SET (SRC_KINECTDEVICE
    kinectdevice/vpKinect.cpp
  )
ENDIF()

SET (SRC_LASERSCANNER
  laserscanner/sick/vpSickLDMRS.cpp
  )

IF(VISP_HAVE_PARPORT)
  SET (SRC_LIGHT
    light/vpRingLight.cpp
    )
ENDIF()

SET (SRC_MATH
  math/kalman/vpKalmanFilter.cpp
  math/kalman/vpLinearKalmanFilterInstantiation.cpp
  math/matrix/vpColVector.cpp
  math/matrix/vpMatrix.cpp
  math/matrix/vpMatrix_lu.cpp
  math/matrix/vpMatrix_svd.cpp
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
  math/transformation/vpRotationMatrix.cpp
  math/transformation/vpRotationVector.cpp
  math/transformation/vpRxyzVector.cpp
  math/transformation/vpRzyxVector.cpp
  math/transformation/vpRzyzVector.cpp
  math/transformation/vpThetaUVector.cpp
  math/transformation/vpTranslationVector.cpp
  math/transformation/vpTwistMatrix.cpp
  math/transformation/vpVelocityTwistMatrix.cpp
  )

SET (SRC_ROBOT
  robot/robot/vpRobot.cpp
  robot/robot/vpRobotTemplate.cpp
  robot/real-robot/afma4/vpAfma4.cpp
  robot/real-robot/afma6/vpAfma6.cpp
  robot/real-robot/biclops/vpBiclops.cpp
  robot/real-robot/ptu46/vpPtu46.cpp
  robot/real-robot/viper/vpViper.cpp
  robot/real-robot/viper/vpViper850.cpp
  robot/simulator-robot/vpRobotCamera.cpp
  robot/simulator-robot/vpRobotSimulator.cpp
  robot/simulator-robot/vpSimulatorViper850.cpp
  robot/simulator-robot/vpSimulatorAfma6.cpp
  )

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
IF(VISP_HAVE_CYCAB)
  LIST(APPEND SRC_ROBOT robot/real-robot/cycab/vpRobotCycab.cpp)
ENDIF()
IF(VISP_HAVE_PTU46)
  LIST(APPEND SRC_ROBOT robot/real-robot/ptu46/vpRobotPtu46.cpp)
ENDIF()
IF(VISP_HAVE_VIPER850)
  LIST(APPEND SRC_ROBOT robot/real-robot/viper/vpRobotViper850.cpp)
ENDIF()

SET (SRC_SERVO
  servo/vpAdaptativeGain.cpp
  servo/vpServo.cpp
  servo/vpServoData.cpp
  servo/vpServoDisplay.cpp
  )

SET (SRC_SIMULATOR
  simulator/coin-simulator/vpProjectionDisplay.cpp
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
  tools/plot/vpPlot.cpp
  tools/plot/vpPlotCurve.cpp
  tools/plot/vpPlotGraph.cpp
  tools/time/vpTime.cpp
  )
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
  tracking/mbt/vpMbtDistanceLine.cpp
  tracking/mbt/vpMbtHiddenFace.cpp
  tracking/mbt/vpMbtMeLine.cpp
  tracking/mbt/vpMbEdgeTracker.cpp
  tracking/mbt/vpMbtXmlParser.cpp
  tracking/mbt/vpMbtDistanceCylinder.cpp
  tracking/moments/vpMomentObject.cpp
  tracking/moments/vpMomentAlpha.cpp
  tracking/moments/vpMomentBasic.cpp
  tracking/moments/vpMomentCentered.cpp
  tracking/moments/vpMomentCenteredNormalized.cpp
  tracking/moments/vpMomentCInvariant.cpp
  tracking/moments/vpMomentCommon.cpp
  tracking/moments/vpMoment.cpp
  tracking/moments/vpMomentDatabase.cpp
  tracking/moments/vpMomentGravityCenter.cpp
  tracking/moments/vpMomentGravityCenterNormalized.cpp
  tracking/moments/vpMomentObject.cpp
  tracking/moments/vpMomentAreaNormalized.cpp
  )

IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_TRACKING tracking/klt/vpKltOpencv.cpp)
ENDIF()

SET (SRC_VIDEO
  video/vpVideoReader.cpp
  video/vpVideoWriter.cpp
  )

IF(VISP_HAVE_FFMPEG)
  LIST(APPEND SRC_VIDEO video/vpFFMPEG.cpp)
ENDIF()

SET (SRC_VIDEO_DEVICE
  video-device/vpDisplay.cpp
  )

IF(VISP_HAVE_GTK)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/vpDisplayGTK.cpp)
ENDIF()
IF(VISP_HAVE_OPENCV)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/vpDisplayOpenCV.cpp)
ENDIF()
IF(VISP_HAVE_X11)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/vpDisplayX.cpp)
ENDIF()

IF(WIN32)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/windows/vpDisplayWin32.cpp)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/windows/vpWin32Window.cpp)
ENDIF()
IF(VISP_HAVE_GDI)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/windows/vpGDIRenderer.cpp)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/windows/vpDisplayGDI.cpp)
ENDIF()
IF(VISP_HAVE_D3D9)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/windows/vpD3DRenderer.cpp)
  LIST(APPEND SRC_VIDEO_DEVICE video-device/windows/vpDisplayD3D.cpp)
ENDIF()

SET (SRC_VISUAL_FEATURE
  visual-feature/vpBasicFeature.cpp
  visual-feature/vpFeatureDepth.cpp
  visual-feature/vpFeatureDisplay.cpp
  visual-feature/vpFeatureEllipse.cpp
  visual-feature/vpFeatureLine.cpp
  visual-feature/vpFeaturePoint3D.cpp
  visual-feature/vpFeaturePoint.cpp
  visual-feature/vpFeaturePointPolar.cpp
  visual-feature/vpFeatureThetaU.cpp
  visual-feature/vpFeatureTranslation.cpp
  visual-feature/vpFeatureVanishingPoint.cpp
  visual-feature/vpFeatureLuminance.cpp
  visual-feature/vpGenericFeature.cpp
  visual-feature/vpFeatureMoment.cpp
  visual-feature/vpFeatureMomentDatabase.cpp
  visual-feature/vpFeatureMomentCommon.cpp
  visual-feature/vpFeatureMomentAlpha.cpp
  visual-feature/vpFeatureMomentGravityCenter.cpp
  visual-feature/vpFeatureMomentBasic.cpp
  visual-feature/vpFeatureMomentGravityCenterNormalized.cpp
  visual-feature/vpFeatureMomentCentered.cpp
  visual-feature/vpFeatureMomentCInvariant.cpp
  visual-feature/vpFeatureMomentCommon.cpp
  visual-feature/vpFeatureMomentAreaNormalized.cpp
  )

SET (SRC_ALL
  ${SRC_CAMERA}
  ${SRC_COMPUTER_VISION}
  ${SRC_EXCEPTION}
  ${SRC_FRAMEGRABBER_DEVICE}
  ${SRC_IMAGE}
  ${SRC_KEY_POINT}
  ${SRC_KINECTDEVICE}
  ${SRC_LASERSCANNER}
  ${SRC_LIGHT}
  ${SRC_MATH}
  ${SRC_ROBOT}
  ${SRC_SERVO}
  ${SRC_SIMULATOR}
  ${SRC_TOOLS}
  ${SRC_TRACKING}
  ${SRC_VIDEO}
  ${SRC_VIDEO_DEVICE}
  ${SRC_VISUAL_FEATURE}
  )
 