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
# Try to determine if calibration data (camera intrinsic/extrinsic, FT calib file)
# are available for Inria's Viper 850 robot. 
#
# VIPER850_DATA_FOUND
# VIPER850_DATA_PATH
#
# Authors:
# Fabien Spindler
#
#############################################################################

    
find_path(VIPER850_DATA_PATH
  NAMES include/const_camera_Viper850.xml
  PATHS
    /home/soft/Viper850/Viper850
    /home/soft/Viper850/current
    /udd/fspindle/robot/Viper850/current
    Z:/robot/Viper850/current
  )

#message("DBG VIPER850_DATA_PATH=${VIPER850_DATA_PATH}")
 
if(VIPER850_DATA_PATH)
  set(VIPER850_DATA_FOUND TRUE)
else()
  SET(VIPER850_FATA_FOUND FALSE)
endif()

mark_as_advanced(
  VIPER850_DATA_PATH
)

