#############################################################################
#
# $Id$
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
# ViSP overall configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

set(EXAMPLES_REQUIRED_DEPS visp_core visp_mbt visp_tt visp_vision)

vp_check_dependencies(${EXAMPLES_REQUIRED_DEPS})

if(VP_DEPENDENCIES_FOUND)

# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
set(SOURCE
  mbtEdgeTracking.cpp
  mbtEdgeKltTracking.cpp
  mbtKltTracking.cpp
  templateTracker.cpp
  trackDot2WithAutoDetection.cpp
  trackMeCircle.cpp
  trackMeEllipse.cpp
  trackMeLine.cpp
  trackMeNurbs.cpp
  trackDot.cpp
  trackDot2.cpp
  trackKltOpencv.cpp
)

vp_include_modules_recurse(${EXAMPLES_REQUIRED_DEPS})

# rule for binary build
foreach(source ${SOURCE})
  # Compute the name of the binary to create
  get_filename_component(binary ${source} NAME_WE)

  # From source compile the binary and add link rules
  add_executable(${binary} ${source})
  target_link_libraries(${binary} ${EXAMPLES_REQUIRED_DEPS})

  add_dependencies(visp_examples ${binary})
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(${binary} PROPERTIES FOLDER "examples")
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

add_test(trackDot2WithAutoDetection trackDot2WithAutoDetection -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeCircle    trackMeCircle -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeEllipse   trackMeEllipse -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeLine      trackMeLine -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(trackMeNurbs     trackMeNurbs -c ${OPTION_TO_DESACTIVE_DISPLAY})
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

endif()
