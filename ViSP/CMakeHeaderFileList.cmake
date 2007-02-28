#############################################################################
#
# $Id: CMakeHeaderFileList.cmake,v 1.10 2007-02-28 11:35:49 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
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
  )

SET (HEADER_COMPUTER_VISION
  computer-vision/homography-estimation/vpHomography.h
  computer-vision/pose-estimation/vpLevenbergMarquartd.h
  computer-vision/pose-estimation/vpPoseException.h
  computer-vision/pose-estimation/vpPose.h
  )

SET (HEADER_DATA_STRUCTURE
  data-structure/vpList.h
  data-structure/vpList.t.cpp
  )

SET (HEADER_EXCEPTION
  exceptions/vpException.h
  )

SET (HEADER_FRAMEGRABBER_DEVICE
  framegrabber-device/1394/vp1394Grabber.h
  framegrabber-device/1394/vp1394TwoGrabber.h
  framegrabber-device/disk/vpDiskGrabber.h
  framegrabber-device/generic-framegrabber/vpFrameGrabberException.h
  framegrabber-device/generic-framegrabber/vpFrameGrabber.h
  framegrabber-device/IC-Comp/vpIcCompGrabber.h
  framegrabber-device/itifg8/vpItifg8Grabber.h
  framegrabber-device/OSX-cfox/vpOSXcfoxGrabber.h
  framegrabber-device/v4l2/vpV4l2Grabber.h
  framegrabber-device/directshow/vpDirectShowGrabber.h
  framegrabber-device/directshow/vpDirectShowGrabberImpl.h
  framegrabber-device/directshow/vpDirectShowDevice.h
  framegrabber-device/directshow/vpDirectShowSampleGrabberI.h
  )

SET (HEADER_IMAGE
  image/vpColor.h
  image/vpImageConvert.h
  image/vpImageException.h
  image/vpImageFilter.h
  image/vpImage.h
  image/vpImageBase.t.cpp
  image/vpImageIo.h
  image/vpImageMorphology.h
  image/vpImageTools.h
  image/vpRGBa.h
  )

SET (HEADER_MATH
  math/matrix/vpColVector.h
  math/matrix/vpMatrixException.h
  math/matrix/vpMatrix.h
  math/matrix/vpRowVector.h
  math/misc/vpExponentialMap.h
  math/misc/vpMath.h
  math/misc/vpNoise.h
  math/robust/vpRansac.h
  math/robust/vpRansac.t.cpp
  math/robust/vpRobust.h
  math/robust/vpScale.h
  math/transformation/vpEulerVector.h
  math/transformation/vpHomogeneousMatrix.h
  math/transformation/vpPoseVector.h
  math/transformation/vpRotationMatrix.h
  math/transformation/vpRotationVector.h
  math/transformation/vpRxyzVector.h
  math/transformation/vpRzyxVector.h
  math/transformation/vpThetaUVector.h
  math/transformation/vpTranslationVector.h
  math/transformation/vpTwistMatrix.h
  )

SET (HEADER_ROBOT
  robot/robot/vpRobotException.h
  robot/robot/vpRobot.h
  robot/robot/vpRobotTemplate.h
  robot/simulation-robot/vpRobotCamera.h
  robot/real-robot/afma4/vpAfma4.h
  robot/real-robot/afma4/vpRobotAfma4Contrib.h
  robot/real-robot/afma4/vpRobotAfma4.h
  robot/real-robot/afma6/vpAfma6.h
  robot/real-robot/afma6/vpRobotAfma6Contrib.h
  robot/real-robot/afma6/vpRobotAfma6.h
  robot/real-robot/biclops/vpBiclops.h
  robot/real-robot/biclops/vpRobotBiclopsController.h
  robot/real-robot/biclops/vpRobotBiclops.h
  robot/real-robot/ptu46/vpPtu46.h
  robot/real-robot/ptu46/vpRobotPtu46.h
  )

SET (HEADER_SERVO
  servo/vpAdaptativeGain.h
  servo/vpServoData.h
  servo/vpServoDisplay.h
  servo/vpServoException.h
  servo/vpServo.h
  )

SET (HEADER_SIMULATOR
  simulator/vpProjectionDisplay.h
  simulator/vpSimulatorException.h
  simulator/vpSimulator.h
  simulator/vpViewer.h
  )

SET (HEADER_TOOLS
  tools/geometry/vpPlane.h
  tools/geometry/vpRect.h
  tools/io/vpIoException.h
  tools/io/vpIoTools.h
  tools/io/vpParseArgv.h
  tools/time/vpTime.h
  tools/trace/vpDebug.h
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
  tracking/moving-edges/vpMeEllipse.h
  tracking/moving-edges/vpMe.h
  tracking/moving-edges/vpMeLine.h
  tracking/moving-edges/vpMeSite.h
  tracking/moving-edges/vpMeTracker.h
  )

SET (HEADER_VIDEO_DEVICE
  video-device/vpDisplayException.h
  video-device/vpDisplayGTK.h
  video-device/vpDisplay.h
  video-device/vpDisplayX.h
  video-device/vpMouseButton.h
  video-device/windows/vpDisplayGDI.h
  video-device/windows/vpDisplayWin32.h
  video-device/windows/vpGDIRenderer.h
  video-device/windows/vpWin32Renderer.h
  video-device/windows/vpWin32Window.h
  video-device/windows/vpD3DRenderer.h
  video-device/windows/vpDisplayD3D.h
  )

SET (HEADER_VISUAL_FEATURE
  visual-feature/vpBasicFeature.h
  visual-feature/vpFeatureDisplay.h
  visual-feature/vpFeatureEllipse.h
  visual-feature/vpFeatureException.h
  visual-feature/vpFeatureLine.h
  visual-feature/vpFeaturePoint3D.h
  visual-feature/vpFeaturePoint.h
  visual-feature/vpFeatureThetaU.h
  visual-feature/vpFeatureTranslation.h
  visual-feature/vpFeatureVanishingPoint.h
  visual-feature/vpGenericFeature.h
  )

SET (HEADER_ALL 
  ${HEADER_CAMERA}
  ${HEADER_COMPUTER_VISION}
  ${HEADER_DATA_STRUCTURE}
  ${HEADER_EXCEPTION}
  ${HEADER_FRAMEGRABBER_DEVICE}
  ${HEADER_IMAGE}
  ${HEADER_MATH}
  ${HEADER_ROBOT}
  ${HEADER_SIMULATOR}
  ${HEADER_SERVO}
  ${HEADER_TOOLS}
  ${HEADER_TRACKING}
  ${HEADER_VIDEO_DEVICE}
  ${HEADER_VISUAL_FEATURE}
  )
