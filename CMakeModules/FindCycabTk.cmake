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
# Try to find CycabTk toolkit to control the Cycab car-like mobile robot.
# Once run this will define: 
#
# CycabTk_FOUND
# CycabTk_INCLUDE_DIR
# CycabTk_LIBRARIES
#
# Additional var set when the new version of cycabtk exists (to use)
# CycabTk_NEW_FOUND
# CycabTk_NEW_INCLUDE_DIR
# CycabTk_NEW_LIBRARIES
#
# Additional var set when the old version of cycabtk exists (obsolete)
# CycabTk_OLD_FOUND
# CycabTk_OLD_INCLUDE_DIR
# CycabTk_OLD_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

#
# check first if the old version of cycabtk exists
# This old version is to use with the Syndex base low level control of the Cycab
# running on cycab-hf1 computer
#
FIND_PATH(CycabTk_OLD_INCLUDE_DIR EtherCycab/EtherCycab.hpp
  $ENV{CycabTk_OLD_HOME}/include
  $ENV{HOME}/soft/Cycab/third-party/cycabtk-old/cycabtk-old/include
  /local/fspindle/soft/Cycab/third-party/cycabtk-old/cycabtk-old/include
)
#MESSAGE("DBG CycabTk_OLD_INCLUDE_DIR=${CycabTk_OLD_INCLUDE_DIR}")  

FIND_LIBRARY(CycabTk_OLD_LIBRARY
  NAMES cycabtk
  PATHS 
  $ENV{CycabTk_OLD_HOME}/lib
  $ENV{HOME}/soft/Cycab/third-party/cycabtk-old/cycabtk-old/lib
  /local/fspindle/soft/Cycab/third-party/cycabtk-old/cycabtk-old/lib
)
#MESSAGE("DBG CycabTk_OLD_LIBRARY=${CycabTk_OLD_LIBRARY}")


IF(CycabTk_OLD_INCLUDE_DIR AND CycabTk_OLD_LIBRARY)
  SET(CycabTk_OLD_FOUND TRUE)
  SET(CycabTk_OLD_INCLUDE_DIR ${CycabTk_OLD_INCLUDE_DIR})
  SET(CycabTk_OLD_LIBRARIES ${CycabTk_OLD_LIBRARY})
ELSE(CycabTk_OLD_INCLUDE_DIR AND CycabTk_OLD_LIBRARY)
  SET(CycabTk_OLD_FOUND FALSE)
ENDIF(CycabTk_OLD_INCLUDE_DIR AND CycabTk_OLD_LIBRARY)

#
# check secondly if the new version of cycabtk exists
# This new version is to use with the C base low level control of the Cycab
# throw the Peak CAN dongle
#
# --- Check library dependency
include(FindPkgConfig)
	
set( Boost_USE_MULTITHREADED ON )
find_package( Boost COMPONENTS serialization)

FIND_LIBRARY(HUGR_LIBRARY
  NAMES hugr
  PATHS 
  /usr/lib
  /usr/local/lib
)

IF(Boost_SERIALIZATION_FOUND AND HUGR_LIBRARY)
  SET(CycabTk_NEW_FOUND TRUE)
  SET(CycabTk_NEW_LIBRARIES ${HUGR_LIBRARY} ${Boost_SERIALIZATION_LIBRARIES})
ELSE(Boost_SERIALIZATION_FOUND AND HUGR_LIBRARY)
  SET(CycabTk_NEW_FOUND FALSE)
ENDIF(Boost_SERIALIZATION_FOUND AND HUGR_LIBRARY)

#MESSAGE("CycabTk new: ${CycabTk_NEW_FOUND} old: ${CycabTk_OLD_FOUND}")

IF(CycabTk_NEW_FOUND AND CycabTk_OLD_FOUND)
  OPTION(USE_CYCAB_WITH_OLD_CYCABTK "Use the old and obsolete CycabTk" FALSE)
ENDIF(CycabTk_NEW_FOUND AND CycabTk_OLD_FOUND)

IF(USE_CYCAB_WITH_OLD_CYCABTK)
  # either old or new cycabtk is detected
  SET(CycabTk_FOUND TRUE)
  SET(CycabTk_NEW_FOUND FALSE)
  SET(CycabTk_INCLUDE_DIR ${CycabTk_OLD_INCLUDE_DIR})
  SET(CycabTk_LIBRARIES ${CycabTk_OLD_LIBRARIES})
ELSE(USE_CYCAB_WITH_OLD_CYCABTK)
  IF(CycabTk_NEW_FOUND)
    SET(CycabTk_FOUND TRUE)
    SET(CycabTk_OLD_FOUND FALSE)
    SET(CycabTk_LIBRARIES ${CycabTk_NEW_LIBRARIES})
  ELSE(CycabTk_NEW_FOUND)
    IF(CycabTk_OLD_FOUND)
      SET(CycabTk_FOUND TRUE)
      SET(CycabTk_INCLUDE_DIR ${CycabTk_OLD_INCLUDE_DIR})
      SET(CycabTk_LIBRARIES ${CycabTk_OLD_LIBRARIES})
    ENDIF(CycabTk_OLD_FOUND)
  ENDIF(CycabTk_NEW_FOUND)
ENDIF(USE_CYCAB_WITH_OLD_CYCABTK)

MARK_AS_ADVANCED(
  CycabTk_INCLUDE_DIR
  CycabTk_LIBRARIES
  CycabTk_NEW_INCLUDE_DIR
  CycabTk_NEW_LIBRARIES
  CycabTk_OLD_INCLUDE_DIR
  CycabTk_OLD_LIBRARY
  CycabTk_OLD_LIBRARIES
  HUGR_LIBRARY
)