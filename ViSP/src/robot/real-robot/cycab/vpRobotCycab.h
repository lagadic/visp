/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
