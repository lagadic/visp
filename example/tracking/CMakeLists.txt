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

# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
SET (SOURCE
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

# rule for binary build
FOREACH(source ${SOURCE})
  # Compute the name of the binary to create
  GET_FILENAME_COMPONENT(binary ${source} NAME_WE)

  # From source compile the binary and add link rules
  ADD_EXECUTABLE(${binary} ${source})
  TARGET_LINK_LIBRARIES(${binary} ${VISP_INTERN_LIBRARY} ${VISP_EXTERN_LIBRARIES})


ENDFOREACH(source)

# Add test
ADD_TEST(mbtEdgeTracking    mbtEdgeTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(mbtEdgeKltTracking mbtEdgeKltTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(mbtKltTracking     mbtKltTracking -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackDot2WithAutoDetection trackDot2WithAutoDetection -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackMeCircle    trackMeCircle -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackMeEllipse   trackMeEllipse -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackMeLine      trackMeLine -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackMeNurbs     trackMeNurbs -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackDot         trackDot -c ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(trackDot2        trackDot2 -c ${OPTION_TO_DESACTIVE_DISPLAY})

#ADD_TEST(templateTracker-SSDESM-Affine        templateTracker -c -l 2 -t 0 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDESM-Homography    templateTracker -c -l 2 -t 0 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDESM-HomographySL3 templateTracker -c -l 2 -t 0 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDESM-SRT           templateTracker -c -l 2 -t 0 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDESM-Translation   templateTracker -c -l 2 -t 0 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

ADD_TEST(templateTrackerPyramidal-SSDESM-Affine        templateTracker -c -l 2 -t 0 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDESM-Homography    templateTracker -c -l 2 -t 0 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDESM-HomographySL3 templateTracker -c -l 2 -t 0 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDESM-SRT           templateTracker -c -l 2 -t 0 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDESM-Translation   templateTracker -c -l 2 -t 0 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#ADD_TEST(templateTracker-SSDForwardAdditional-Affine        templateTracker -c -l 2 -t 1 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardAdditional-Homography    templateTracker -c -l 2 -t 1 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 1 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardAdditional-SRT           templateTracker -c -l 2 -t 1 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardAdditional-Translation   templateTracker -c -l 2 -t 1 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

ADD_TEST(templateTrackerPyramidal-SSDForwardAdditional-Affine        templateTracker -c -l 2 -t 1 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardAdditional-Homography    templateTracker -c -l 2 -t 1 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 1 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardAdditional-SRT           templateTracker -c -l 2 -t 1 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardAdditional-Translation   templateTracker -c -l 2 -t 1 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#ADD_TEST(templateTracker-SSDForwardCompositional-Affine        templateTracker -c -l 2 -t 2 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardCompositional-Homography    templateTracker -c -l 2 -t 2 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardCompositional-HomographySL3 templateTracker -c -l 2 -t 2 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardCompositional-SRT           templateTracker -c -l 2 -t 2 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDForwardCompositional-Translation   templateTracker -c -l 2 -t 2 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

ADD_TEST(templateTrackerPyramidal-SSDForwardCompositional-Affine        templateTracker -c -l 2 -t 2 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardCompositional-Homography    templateTracker -c -l 2 -t 2 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardCompositional-HomographySL3 templateTracker -c -l 2 -t 2 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardCompositional-SRT           templateTracker -c -l 2 -t 2 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDForwardCompositional-Translation   templateTracker -c -l 2 -t 2 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#ADD_TEST(templateTracker-SSDInverseCompositional-Affine        templateTracker -c -l 2 -t 3 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDInverseCompositional-Homography    templateTracker -c -l 2 -t 3 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 3 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDInverseCompositional-SRT           templateTracker -c -l 2 -t 3 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-SSDInverseCompositional-Translation   templateTracker -c -l 2 -t 3 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

ADD_TEST(templateTrackerPyramidal-SSDInverseCompositional-Affine        templateTracker -c -l 2 -t 3 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDInverseCompositional-Homography    templateTracker -c -l 2 -t 3 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 3 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDInverseCompositional-SRT           templateTracker -c -l 2 -t 3 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-SSDInverseCompositional-Translation   templateTracker -c -l 2 -t 3 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#ADD_TEST(templateTracker-ZNCCForwardAdditional-Affine        templateTracker -c -l 2 -t 4 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCForwardAdditional-Homography    templateTracker -c -l 2 -t 4 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 4 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCForwardAdditional-SRT           templateTracker -c -l 2 -t 4 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCForwardAdditional-Translation   templateTracker -c -l 2 -t 4 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

ADD_TEST(templateTrackerPyramidal-ZNCCForwardAdditional-Affine        templateTracker -c -l 2 -t 4 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCForwardAdditional-Homography    templateTracker -c -l 2 -t 4 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCForwardAdditional-HomographySL3 templateTracker -c -l 2 -t 4 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCForwardAdditional-SRT           templateTracker -c -l 2 -t 4 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCForwardAdditional-Translation   templateTracker -c -l 2 -t 4 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})

#ADD_TEST(templateTracker-ZNCCInverseCompositional-Affine        templateTracker -c -l 2 -t 5 -w 0 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCInverseCompositional-Homography    templateTracker -c -l 2 -t 5 -w 1 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 5 -w 2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCInverseCompositional-SRT           templateTracker -c -l 2 -t 5 -w 3 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(templateTracker-ZNCCInverseCompositional-Translation   templateTracker -c -l 2 -t 5 -w 4 ${OPTION_TO_DESACTIVE_DISPLAY})

ADD_TEST(templateTrackerPyramidal-ZNCCInverseCompositional-Affine        templateTracker -c -l 2 -t 5 -w 0 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCInverseCompositional-Homography    templateTracker -c -l 2 -t 5 -w 1 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCInverseCompositional-HomographySL3 templateTracker -c -l 2 -t 5 -w 2 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCInverseCompositional-SRT           templateTracker -c -l 2 -t 5 -w 3 -p ${OPTION_TO_DESACTIVE_DISPLAY})
ADD_TEST(templateTrackerPyramidal-ZNCCInverseCompositional-Translation   templateTracker -c -l 2 -t 5 -w 4 -p ${OPTION_TO_DESACTIVE_DISPLAY})

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
