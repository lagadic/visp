#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
# ViSP header file list.
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Set HEADER_subdir variable to all the files we want to parse during
# the build process. 
# Don't forget to update HEADER_ALL variable if you add/remove a 
# HEADER_subdir variable
#
# If you add/remove a directory, modify here

SET (HEADER_CAMERA
  camera/vpCameraParameters.h
  camera/vpMeterPixelConversion.h 
  camera/vpPixelMeterConversion.h
  camera/vpXmlParserCamera.h
  camera/calibration/vpCalibration.h
  camera/calibration/vpCalibrationException.h
  )

SET (HEADER_COMPUTER_VISION
  computer-vision/homography-estimation/vpHomography.h
  computer-vision/pose-estimation/vpLevenbergMarquartd.h
  computer-vision/pose-estimation/vpPoseException.h
  computer-vision/pose-estimation/vpPose.h
  computer-vision/pose-estimation/vpPoseFeatures.h
  )

SET (HEADER_DATA_STRUCTURE
  data-structure/vpList.h
  )

SET (HEADER_EXCEPTION
  exceptions/vpException.h
  )

SET (HEADER_DEVICE_FRAMEGRABBER
  device/framegrabber/1394/vp1394TwoGrabber.h
  device/framegrabber/1394/vp1394CMUGrabber.h
  device/framegrabber/disk/vpDiskGrabber.h
  device/framegrabber/generic-framegrabber/vpFrameGrabberException.h
  device/framegrabber/generic-framegrabber/vpFrameGrabber.h
  device/framegrabber/v4l2/vpV4l2Grabber.h
  device/framegrabber/directshow/vpDirectShowGrabber.h
  device/framegrabber/directshow/vpDirectShowGrabberImpl.h
  device/framegrabber/directshow/vpDirectShowDevice.h
  device/framegrabber/directshow/vpDirectShowSampleGrabberI.h
  device/framegrabber/OpenCV/vpOpenCVGrabber.h
  )

SET (HEADER_IMAGE
  image/vpColor.h
  image/vpImageConvert.h
  image/vpImageException.h
  image/vpImageFilter.h
  image/vpImage.h
  image/vpImageIo.h
  image/vpImageMorphology.h
  image/vpImageTools.h
  image/vpRGBa.h
  image/vpImagePoint.h
  )

SET (HEADER_KEY_POINT
  key-point/vpBasicKeyPoint.h
  key-point/vpKeyPointSurf.h
  key-point/vpPlanarObjectDetector.h
  key-point/vpFernClassifier.h
  )

SET (HEADER_DEVICE_KINECT
  device/kinect/vpKinect.h
  )

SET (HEADER_DEVICE_LASERSCANNER
  device/laserscanner/vpScanPoint.h
  device/laserscanner/vpLaserScan.h
  device/laserscanner/vpLaserScanner.h
  device/laserscanner/sick/vpSickLDMRS.h
  )

SET (HEADER_DEVICE_LIGHT
  device/light/vpRingLight.h
  )

SET (HEADER_MATH
  math/kalman/vpKalmanFilter.h
  math/kalman/vpLinearKalmanFilterInstantiation.h
  math/matrix/vpColVector.h
  math/matrix/vpMatrixException.h
  math/matrix/vpMatrix.h
  math/matrix/vpRowVector.h
  math/matrix/vpSubMatrix.h
  math/matrix/vpSubColVector.h
  math/matrix/vpSubRowVector.h
  math/matrix/vpGEMM.h
  math/misc/vpMath.h
  math/misc/vpHinkley.h
  math/misc/vpNoise.h
  math/robust/vpRansac.h
  math/robust/vpRobust.h
  math/robust/vpScale.h
  math/spline/vpBSpline.h
  math/spline/vpNurbs.h
  math/transformation/vpExponentialMap.h
  math/transformation/vpForceTwistMatrix.h
  math/transformation/vpHomogeneousMatrix.h
  math/transformation/vpPoseVector.h
  math/transformation/vpQuaternionVector.h
  math/transformation/vpRotationMatrix.h
  math/transformation/vpRotationVector.h
  math/transformation/vpRxyzVector.h
  math/transformation/vpRzyxVector.h
  math/transformation/vpRzyzVector.h
  math/transformation/vpThetaUVector.h
  math/transformation/vpTranslationVector.h
  math/transformation/vpVelocityTwistMatrix.h
  )

SET (HEADER_ROBOT
  robot/robot/vpRobotException.h
  robot/robot/vpRobot.h
  robot/robot/vpRobotTemplate.h
  robot/real-robot/afma4/vpAfma4.h
  robot/real-robot/afma4/vpRobotAfma4.h
  robot/real-robot/afma4/vpServolens.h
  robot/real-robot/afma6/vpAfma6.h
  robot/real-robot/afma6/vpRobotAfma6.h
  robot/real-robot/biclops/vpBiclops.h
  robot/real-robot/biclops/vpRobotBiclopsController.h
  robot/real-robot/biclops/vpRobotBiclops.h
  robot/real-robot/cycab/vpRobotCycab.h
  robot/real-robot/pioneer/vpUnicycle.h
  robot/real-robot/pioneer/vpPioneer.h
  robot/real-robot/pioneer/vpPioneerPan.h
  robot/real-robot/pioneer/vpRobotPioneer.h
  robot/real-robot/ptu46/vpPtu46.h
  robot/real-robot/ptu46/vpRobotPtu46.h
  robot/real-robot/viper/vpViper.h
  robot/real-robot/viper/vpViper650.h
  robot/real-robot/viper/vpViper850.h
  robot/real-robot/viper/vpRobotViper650.h
  robot/real-robot/viper/vpRobotViper850.h
  robot/simulator-robot/vpRobotCamera.h
  robot/simulator-robot/vpRobotSimulator.h
  robot/simulator-robot/vpRobotWireFrameSimulator.h
  robot/simulator-robot/vpSimulatorAfma6.h
  robot/simulator-robot/vpSimulatorCamera.h
  robot/simulator-robot/vpSimulatorPioneer.h
  robot/simulator-robot/vpSimulatorPioneerPan.h
  robot/simulator-robot/vpSimulatorViper850.h
  )

SET (HEADER_SERVO
  servo/vpAdaptiveGain.h
  servo/vpServoData.h
  servo/vpServoDisplay.h
  servo/vpServoException.h
  servo/vpServo.h
  )

SET (HEADER_SIMULATOR
  simulator/coin-simulator/vpAR.h
  simulator/coin-simulator/vpProjectionDisplay.h
  simulator/coin-simulator/vpSimulatorException.h
  simulator/coin-simulator/vpSimulator.h
  simulator/coin-simulator/vpViewer.h
  simulator/ogre-simulator/vpAROgre.h
  simulator/image-simulator/vpImageSimulator.h
  simulator/wireframe-simulator/vpWireFrameSimulator.h
  simulator/wireframe-simulator/core/vpArit.h
  simulator/wireframe-simulator/core/vpBound.h
  simulator/wireframe-simulator/core/vpCgiconstants.h
  simulator/wireframe-simulator/core/vpImstack.h
  simulator/wireframe-simulator/core/vpKeyword.h
  simulator/wireframe-simulator/core/vpLex.h
  simulator/wireframe-simulator/core/vpMy.h
  simulator/wireframe-simulator/core/vpRfstack.h
  simulator/wireframe-simulator/core/vpSkipio.h
  simulator/wireframe-simulator/core/vpTmstack.h
  simulator/wireframe-simulator/core/vpToken.h
  simulator/wireframe-simulator/core/vpView.h
  simulator/wireframe-simulator/core/vpVwstack.h
  )

SET (HEADER_TOOLS
  tools/geometry/vpPlane.h
  tools/geometry/vpRect.h
  tools/geometry/vpTriangle.h
  tools/geometry/vpPolygon.h
  tools/histogram/vpHistogram.h
  tools/histogram/vpHistogramPeak.h
  tools/histogram/vpHistogramValey.h
  tools/io/vpIoException.h
  tools/io/vpIoTools.h
  tools/io/vpKeyboard.h
  tools/io/vpParallelPort.h
  tools/io/vpParallelPortException.h
  tools/io/vpParseArgv.h
  tools/mutex/vpMutex.h
  tools/plot/vpPlot.h
  tools/plot/vpPlotCurve.h
  tools/plot/vpPlotGraph.h
  tools/time/vpTime.h
  tools/trace/vpDebug.h
  tools/xml/vpXmlParser.h
  )

SET (HEADER_TRACKING
  tracking/dots/vpDot2.h
  tracking/dots/vpDot.h
  tracking/feature-builder/vpFeatureBuilder.h
  tracking/forward-projection/vpCircle.h
  tracking/forward-projection/vpCylinder.h
  tracking/forward-projection/vpForwardProjection.h
  tracking/forward-projection/vpLine.h
  tracking/forward-projection/vpPoint.h
  tracking/forward-projection/vpSphere.h
  tracking/general-tracking-issues/vpTracker.h
  tracking/general-tracking-issues/vpTrackingException.h
  tracking/klt/vpKltOpencv.h
  tracking/moving-edges/vpMeEllipse.h
  tracking/moving-edges/vpMe.h
  tracking/moving-edges/vpMeLine.h
  tracking/moving-edges/vpMeSite.h
  tracking/moving-edges/vpMeTracker.h
  tracking/moving-edges/vpMeNurbs.h

  tracking/mbt/vpMbTracker.h
  tracking/mbt/vpMbHiddenFaces.h 
  tracking/mbt/edge/vpMbtDistanceLine.h
  tracking/mbt/edge/vpMbtPolygon.h
  tracking/mbt/edge/vpMbtMeLine.h
  tracking/mbt/edge/vpMbEdgeTracker.h
  tracking/mbt/edge/vpMbtXmlParser.h
  tracking/mbt/edge/vpMbtDistanceCylinder.h
  tracking/mbt/hybrid/vpMbEdgeKltTracker.h
  tracking/mbt/klt/vpMbtKltPolygon.h
  tracking/mbt/klt/vpMbKltTracker.h
  tracking/mbt/klt/vpMbtKltXmlParser.h

  tracking/moments/vpMomentObject.h
  tracking/moments/vpMomentAlpha.h
  tracking/moments/vpMomentBasic.h
  tracking/moments/vpMomentCentered.h
  tracking/moments/vpMomentCInvariant.h
  tracking/moments/vpMomentCommon.h
  tracking/moments/vpMoment.h
  tracking/moments/vpMomentDatabase.h
  tracking/moments/vpMomentGravityCenter.h
  tracking/moments/vpMomentGravityCenterNormalized.h
  tracking/moments/vpMomentObject.h
  tracking/moments/vpMomentAreaNormalized.h
  tracking/moments/vpMomentArea.h
  )

SET (HEADER_VIDEO
  video/vpVideoReader.h
  video/vpVideoWriter.h
  video/vpFFMPEG.h
  )

SET (HEADER_DEVICE_DISPLAY
  device/display/vpDisplayException.h
  device/display/vpDisplayGTK.h
  device/display/vpDisplayOpenCV.h
  device/display/vpDisplay.h
  device/display/vpDisplayX.h
  device/display/vpMouseButton.h
  device/display/windows/vpDisplayGDI.h
  device/display/windows/vpDisplayWin32.h
  device/display/windows/vpGDIRenderer.h
  device/display/windows/vpWin32Renderer.h
  device/display/windows/vpWin32Window.h
  device/display/windows/vpD3DRenderer.h
  device/display/windows/vpDisplayD3D.h
  device/display/windows/vpWin32API.h
  )

SET (HEADER_VISUAL_FEATURE
  visual-feature/vpBasicFeature.h
  visual-feature/vpFeatureDepth.h
  visual-feature/vpFeatureDisplay.h
  visual-feature/vpFeatureEllipse.h
  visual-feature/vpFeatureException.h
  visual-feature/vpFeatureLine.h
  visual-feature/vpFeaturePoint3D.h
  visual-feature/vpFeaturePoint.h
  visual-feature/vpFeaturePointPolar.h
  visual-feature/vpFeatureThetaU.h
  visual-feature/vpFeatureTranslation.h
  visual-feature/vpFeatureVanishingPoint.h
  visual-feature/vpFeatureMoment.h  
  visual-feature/vpFeatureMomentDatabase.h
  visual-feature/vpFeatureMomentCommon.h
  visual-feature/vpFeatureMomentAlpha.h
  visual-feature/vpFeatureMomentGravityCenter.h
  visual-feature/vpFeatureMomentBasic.h
  visual-feature/vpFeatureMomentGravityCenterNormalized.h
  visual-feature/vpFeatureMomentCentered.h
  visual-feature/vpFeatureMomentCInvariant.h
  visual-feature/vpFeatureMomentCommon.h
  visual-feature/vpFeatureMomentAreaNormalized.h
  visual-feature/vpFeatureMomentArea.h

  visual-feature/vpFeatureLuminance.h
  visual-feature/vpFeatureSegment.h
  visual-feature/vpGenericFeature.h
  )

SET (HEADER_NETWORK
  network/vpNetwork.h
  network/vpServer.h
  network/vpClient.h
  network/vpRequest.h
  )

SET (HEADER_ALL 
  ${HEADER_CAMERA}
  ${HEADER_COMPUTER_VISION}
  ${HEADER_DATA_STRUCTURE}
  ${HEADER_DEVICE_DISPLAY}
  ${HEADER_DEVICE_FRAMEGRABBER}
  ${HEADER_DEVICE_KINECT}
  ${HEADER_DEVICE_LASERSCANNER}
  ${HEADER_DEVICE_LIGHT}
  ${HEADER_EXCEPTION}
  ${HEADER_IMAGE}
  ${HEADER_KEY_POINT}
  ${HEADER_MATH}
  ${HEADER_ROBOT}
  ${HEADER_SIMULATOR}
  ${HEADER_SERVO}
  ${HEADER_TOOLS}
  ${HEADER_TRACKING}
  ${HEADER_VIDEO}
  ${HEADER_VISUAL_FEATURE}
  ${HEADER_NETWORK}
  )
