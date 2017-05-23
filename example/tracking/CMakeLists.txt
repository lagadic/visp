#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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

if(VISP_HAVE_OGRE)
  visp_set_source_file_compile_flag(mbtEdgeKltTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  visp_set_source_file_compile_flag(mbtEdgeKltMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  visp_set_source_file_compile_flag(mbtEdgeMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  visp_set_source_file_compile_flag(mbtEdgeTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  visp_set_source_file_compile_flag(mbtKltTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  visp_set_source_file_compile_flag(mbtKltMultiTracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
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

add_test(mbtEdgeTracking-cao-scanline    mbtEdgeTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltTracking-cao-scanline mbtEdgeKltTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtKltTracking-cao-scanline     mbtKltTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtEdgeMultiTracking-cao    mbtEdgeMultiTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltMultiTracking-cao mbtEdgeKltMultiTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtKltMultiTracking-cao     mbtKltMultiTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtEdgeMultiTracking-cao-scanline    mbtEdgeMultiTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtEdgeKltMultiTracking-cao-scanline mbtEdgeKltMultiTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtKltMultiTracking-cao-scanline     mbtKltMultiTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtGenericTracking               mbtGenericTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-cao           mbtGenericTracking -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-cao-nocyl     mbtGenericTracking -c -f -C ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-cao-scanline  mbtGenericTracking -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})

add_test(mbtGenericTracking-map               mbtGenericTracking2 -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-map-cao           mbtGenericTracking2 -c -f ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-map-cao-nocyl     mbtGenericTracking2 -c -f -C ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(mbtGenericTracking-map-cao-scanline  mbtGenericTracking2 -c -f -l ${OPTION_TO_DESACTIVE_DISPLAY})

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
