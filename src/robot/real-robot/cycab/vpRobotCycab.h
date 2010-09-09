/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Interface for the car-like Cycab mobile robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpRobotCycab_h
#define vpRobotCycab_h

/*!

  \file vpRobotCycab.h

  Interface for the car-like Cycab mobile robot.

*/

/*!

  \class vpRobotCycab

  \ingroup Cycab RobotDriver

  \brief Interface for the car-like Cycab mobile robot.

*/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_CYCAB

#ifdef VISP_HAVE_CYCABTK_OLD
// Old low level controller based on Syndex (obsolete)
#  include <EtherCycab/EtherCycab.hpp>
#elif defined VISP_HAVE_CYCABTK
// New low level controller based on Syndex (to use)
#  include <hugr.hpp>
#  include <CycabStructs.hpp>

using namespace hugr;

#endif //VISP_HAVE_CYCABTK_OLD

class VISP_EXPORT vpRobotCycab
{
 public:
  vpRobotCycab();
  virtual ~vpRobotCycab();
  
  void setDualSteering(bool dual);
  void setCommand(double v, double phi);
  void getOdometry(double &vmean, double &phi);
  void getOdometry(double &vmean, double &phi, double &timestamp);
  void getOdometry(double &vfl, double &vfr, double &vrl, double &vrr,
		   double &phi);
  void getOdometry(double &vfl, double &vfr, double &vrl, double &vrr,
		   double &phi, double &timestamp);
  void getJoystickPosition(double &x, double &y);
  void getJoystickPosition(double &x, double &y, double &timestamp);

 private:
  bool dualSteering;
#ifdef VISP_HAVE_CYCABTK_OLD
  // Old low level controller based on Syndex (obsolete)
  bool end;
  LockedTimeReq LTR;
  char servername[1024];
  EtherCycab *cycab;
#elif defined VISP_HAVE_CYCABTK
  // New low level controller based on Syndex (to use)
  Store store;
  VariableId cycab_stateId, cycab_commandId;
#endif
};

#endif
#endif
