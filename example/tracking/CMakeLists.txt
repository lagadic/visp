#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP overall configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(example-tracking)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_io visp_gui)

set(example_cpp
  mbtEdgeKltTracking.cpp
  mbtEdgeKltMultiTracking.cpp
  mbtEdgeMultiTracking.cpp
  mbtEdgeTracking.cpp
  mbtGenericTracking.cpp
  mbtGenericTracking2.cpp
  mbtGenericTrackingDepth.cpp
  mbtGenericTrackingDepthOnly.cpp
  mbtKltTracking.cpp
  mbtKltMultiTracking.cpp
  templateTracker.cpp
  trackDot2WithAutoDetection.cpp
  trackMeCircle.cpp
  trackMeEllipse.cpp
  trackMeLine.cpp
#  trackMeNurbs.cpp
  trackDot.cpp
  trackDot2.cpp
  trackKltOpencv.cpp
)

if(BUILD_DEPRECATED_FUNCTIONS)
  if(MSVC)
    # Disable Visual C++ C4996 warning for MBT multi deprecation
    if(BUILD_SHARED_LIBS)
      vp_set_source_file_compile_flag(mbtEdgeMultiTracking.cpp /wd"4996")
      vp_set_source_file_compile_flag(mbtEdgeKltMultiTracking.cpp /wd"4996")
      vp_set_source_file_compile_flag(mbtKltMultiTracking.cpp /wd"4996")
    else()
      vp_warnings_disable(CMAKE_CXX_FLAGS /wd4996)
    endif()
  endif()

  if(VISP_HAVE_OGRE)
    visp_set_source_file_compile_flag(mbtEdgeKltTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtEdgeTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtKltTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTracking2.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTrackingDepth.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTrackingDepthOnly.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtEdgeMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-deprecated-declarations)
    visp_set_source_file_compile_flag(mbtEdgeKltMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-deprecated-declarations)
    visp_set_source_file_compile_flag(mbtKltMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-deprecated-declarations)
  else()
    visp_set_source_file_compile_flag(mbtEdgeMultiTracking.cpp -Wno-deprecated-declarations)
    visp_set_source_file_compile_flag(mbtEdgeKltMultiTracking.cpp -Wno-deprecated-declarations)
    visp_set_source_file_compile_flag(mbtKltMultiTracking.cpp -Wno-deprecated-declarations)
  endif()
else()
  if(VISP_HAVE_OGRE)
    visp_set_source_file_compile_flag(mbtEdgeKltTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtEdgeKltMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtEdgeMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtEdgeTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtKltTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtKltMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTracking2.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTrackingDepth.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(mbtGenericTrackingDepthOnly.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  endif()
endif()

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

# Add test
add_test(mbtEdgeTracking    mbtEdgeTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltTracking mbtEdgeKltTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtKltTracking     mbtKltTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtEdgeTracking-cao    mbtEdgeTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltTracking-cao mbtEdgeKltTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtKltTracking-cao     mbtKltTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtEdgeTracking-cao-nocyl    mbtEdgeTracking -c -f -C ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltTracking-cao-nocyl mbtEdgeKltTracking -c -f -C ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtEdgeTracking-cao-scanline    mbtEdgeTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)
add_test(mbtEdgeKltTracking-cao-scanline mbtEdgeKltTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)
add_test(mbtKltTracking-cao-scanline     mbtKltTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)

add_test(mbtEdgeMultiTracking-cao    mbtEdgeMultiTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltMultiTracking-cao mbtEdgeKltMultiTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtKltMultiTracking-cao     mbtKltMultiTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtEdgeMultiTracking-cao-scanline    mbtEdgeMultiTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)
add_test(mbtEdgeKltMultiTracking-cao-scanline mbtEdgeKltMultiTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)
add_test(mbtKltMultiTracking-cao-scanline     mbtKltMultiTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)

add_test(mbtGenericTracking               mbtGenericTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-cao           mbtGenericTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-cao-nocyl     mbtGenericTracking -c -f -C ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-cao-scanline  mbtGenericTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)

add_test(mbtGenericTracking-map               mbtGenericTracking2 -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-map-cao           mbtGenericTracking2 -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-map-cao-nocyl     mbtGenericTracking2 -c -f -C ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-map-cao-scanline  mbtGenericTracking2 -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 20)

add_test(mbtGenericTrackingDepth-edge-dense             mbtGenericTrackingDepth -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTrackingDepth-edge-dense-scanline    mbtGenericTrackingDepth -c -l ${OPTION_TO_DESACTIVE_DISPLAY} -e 14)
add_test(mbtGenericTrackingDepth-klt-dense              mbtGenericTrackingDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 2)
add_test(mbtGenericTrackingDepth-klt-dense-scanline     mbtGenericTrackingDepth -c -l ${OPTION_TO_DESACTIVE_DISPLAY} -t 2 -e 14)
add_test(mbtGenericTrackingDepth-hybrid-dense           mbtGenericTrackingDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -t 3)
add_test(mbtGenericTrackingDepth-hybrid-dense-scanline  mbtGenericTrackingDepth -c -l ${OPTION_TO_DESACTIVE_DISPLAY} -t 3 -e 14)

add_test(mbtGenericTrackingDepth-edge-normal            mbtGenericTrackingDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -T 4)
add_test(mbtGenericTrackingDepth-edge-normal-scanline   mbtGenericTrackingDepth -c -l ${OPTION_TO_DESACTIVE_DISPLAY} -T 4 -e 14)
add_test(mbtGenericTrackingDepth-klt-normal             mbtGenericTrackingDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -T 4 -t 2)
add_test(mbtGenericTrackingDepth-klt-normal-scanline    mbtGenericTrackingDepth -c -l ${OPTION_TO_DESACTIVE_DISPLAY} -T 4 -t 2 -e 14)
add_test(mbtGenericTrackingDepth-hybrid-normal          mbtGenericTrackingDepth -c ${OPTION_TO_DESACTIVE_DISPLAY} -T 4 -t 3)
add_test(mbtGenericTrackingDepth-hybrid-normal-scanline mbtGenericTrackingDepth -c -l ${OPTION_TO_DESACTIVE_DISPLAY} -T 4 -t 3 -e 14)

add_test(mbtGenericTrackingDepth-dense                          mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTrackingDepth-dense-scanline                 mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY} -l -e 14)
add_test(mbtGenericTrackingDepth-dense-disable                  mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY} -u 117)
add_test(mbtGenericTrackingDepth-normal                         mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY} -T 4)
add_test(mbtGenericTrackingDepth-normal-scanline                mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY} -l -T 4 -e 14)
add_test(mbtGenericTrackingDepth-normal-disable                 mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY} -T 4 -u 117)
add_test(mbtGenericTrackingDepth-dense-normal-scanline-disable  mbtGenericTrackingDepthOnly -c ${OPTION_TO_DESACTIVE_DISPLAY} -l -T 12 -e 14 -u 117)

#disable test with Ogre otherwise we get an Ogre exception with Ogre and remote access:
#terminate called after throwing an instance of 'Ogre::RenderingAPIException'
#what():  OGRE EXCEPTION(3:RenderingAPIException): Couldn`t open X display  in GLXGLSupport::getGLDisplay at /build/buildd/ogre-1.9-1.9.0+dfsg1/RenderSystems/GL/src/GLX/OgreGLXGLSupport.cpp (line 807)
#add_test(mbtEdgeTracking-cao-ogre-scanline    mbtEdgeTracking -c -f -o -l ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(mbtEdgeKltTracking-cao-ogre-scanline mbtEdgeKltTracking -c -f -o -l ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(mbtKltTracking-cao-ogre-scanline     mbtKltTracking -c -f -o -l ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(mbtEdgeMultiTracking-cao-ogre-scanline    mbtEdgeMultiTracking -c -f -o -l ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(mbtEdgeKltMultiTracking-cao-ogre-scanline mbtEdgeKltMultiTracking -c -f -o -l ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(mbtKltMultiTracking-cao-ogre-scanline     mbtKltMultiTracking -c -f -o -l ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(trackDot2WithAutoDetection trackDot2WithAutoDetection -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeCircle    trackMeCircle -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeEllipse   trackMeEllipse -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeLine      trackMeLine -c ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(trackMeNurbs     trackMeNurbs -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackDot         trackDot -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackDot2        trackDot2 -c ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(templateTracker-SSDESM-Affine        templateTracker -c -l 2 -t 0 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDESM-Homography    templateTracker -c -l 2 -t 0 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDESM-HomographySL3 templateTracker -c -l 2 -t 0 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDESM-SRT           templateTracker -c -l 2 -t 0 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDESM-Translation   templateTracker -c -l 2 -t 0 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(templateTrackerPyramidal-SSDESM-Affine        templateTracker -c -l 2 -t 0 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDESM-Homography    templateTracker -c -l 2 -t 0 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDESM-HomographySL3 templateTracker -c -l 2 -t 0 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDESM-SRT           templateTracker -c -l 2 -t 0 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDESM-Translation   templateTracker -c -l 2 -t 0 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDESM-RT            templateTracker -c -l 2 -t 0 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(templateTracker-SSDForwardAdditional-Affine        templateTracker -c -l 2 -t 1 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardAdditional-Homography    templateTracker -c -l 2 -t 1 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 1 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardAdditional-SRT           templateTracker -c -l 2 -t 1 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardAdditional-Translation   templateTracker -c -l 2 -t 1 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(templateTrackerPyramidal-SSDForwardAdditional-Affine        templateTracker -c -l 2 -t 1 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardAdditional-Homography    templateTracker -c -l 2 -t 1 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 1 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardAdditional-SRT           templateTracker -c -l 2 -t 1 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardAdditional-Translation   templateTracker -c -l 2 -t 1 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardAdditional-RT            templateTracker -c -l 2 -t 1 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(templateTracker-SSDForwardCompositional-Affine        templateTracker -c -l 2 -t 2 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardCompositional-Homography    templateTracker -c -l 2 -t 2 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardCompositional-HomographySL3 templateTracker -c -l 2 -t 2 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardCompositional-SRT           templateTracker -c -l 2 -t 2 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDForwardCompositional-Translation   templateTracker -c -l 2 -t 2 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(templateTrackerPyramidal-SSDForwardCompositional-Affine        templateTracker -c -l 2 -t 2 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardCompositional-Homography    templateTracker -c -l 2 -t 2 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardCompositional-HomographySL3 templateTracker -c -l 2 -t 2 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardCompositional-SRT           templateTracker -c -l 2 -t 2 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardCompositional-Translation   templateTracker -c -l 2 -t 2 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDForwardCompositional-RT            templateTracker -c -l 2 -t 2 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(templateTracker-SSDInverseCompositional-Affine        templateTracker -c -l 2 -t 3 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDInverseCompositional-Homography    templateTracker -c -l 2 -t 3 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 3 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDInverseCompositional-SRT           templateTracker -c -l 2 -t 3 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-SSDInverseCompositional-Translation   templateTracker -c -l 2 -t 3 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(templateTrackerPyramidal-SSDInverseCompositional-Affine        templateTracker -c -l 2 -t 3 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDInverseCompositional-Homography    templateTracker -c -l 2 -t 3 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 3 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDInverseCompositional-SRT           templateTracker -c -l 2 -t 3 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDInverseCompositional-Translation   templateTracker -c -l 2 -t 3 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-SSDInverseCompositional-RT            templateTracker -c -l 2 -t 3 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(templateTracker-ZNCCForwardAdditional-Affine        templateTracker -c -l 2 -t 4 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCForwardAdditional-Homography    templateTracker -c -l 2 -t 4 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 4 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCForwardAdditional-SRT           templateTracker -c -l 2 -t 4 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCForwardAdditional-Translation   templateTracker -c -l 2 -t 4 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(templateTrackerPyramidal-ZNCCForwardAdditional-Affine        templateTracker -c -l 2 -t 4 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCForwardAdditional-Homography    templateTracker -c -l 2 -t 4 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 4 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCForwardAdditional-SRT           templateTracker -c -l 2 -t 4 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCForwardAdditional-Translation   templateTracker -c -l 2 -t 4 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCForwardAdditional-RT            templateTracker -c -l 2 -t 4 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#add_test(templateTracker-ZNCCInverseCompositional-Affine        templateTracker -c -l 2 -t 5 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCInverseCompositional-Homography    templateTracker -c -l 2 -t 5 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 5 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCInverseCompositional-SRT           templateTracker -c -l 2 -t 5 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#add_test(templateTracker-ZNCCInverseCompositional-Translation   templateTracker -c -l 2 -t 5 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(templateTrackerPyramidal-ZNCCInverseCompositional-Affine        templateTracker -c -l 2 -t 5 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCInverseCompositional-Homography    templateTracker -c -l 2 -t 5 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 5 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCInverseCompositional-SRT           templateTracker -c -l 2 -t 5 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCInverseCompositional-Translation   templateTracker -c -l 2 -t 5 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(templateTrackerPyramidal-ZNCCInverseCompositional-RT            templateTracker -c -l 2 -t 5 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})

if(BUILD_MODULE_visp_tt_mi)
  add_test(templateTrackerPyramidal-MIInverseCompositional-Affine        templateTracker -c -l 2 -t 9 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
  add_test(templateTrackerPyramidal-MIInverseCompositional-Homography    templateTracker -c -l 2 -t 9 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
  add_test(templateTrackerPyramidal-MIInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 9 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
  add_test(templateTrackerPyramidal-MIInverseCompositional-SRT           templateTracker -c -l 2 -t 9 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
  add_test(templateTrackerPyramidal-MIInverseCompositional-Translation   templateTracker -c -l 2 -t 9 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})
  add_test(templateTrackerPyramidal-MIInverseCompositional-RT            templateTracker -c -l 2 -t 9 -w 5 -p ${OPTION_TO_DESACTIVE_DISPLAY})
endif()
