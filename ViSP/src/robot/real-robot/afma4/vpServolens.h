/****************************************************************************
 *
 * $Id: vpServolens.h,v 1.1 2008-12-17 14:45:01 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Interface for the Servolens lens attached to the camera fixed on the 
 * Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpServolens_h
#define __vpServolens_h

/*!

  \file vpServolens.h

  Interface for the Servolens lens attached to the camera fixed on the 
  Afma4 robot.

*/

#include <visp/vpConfig.h>

/*!

  \class vpServolens

  \ingroup Afma4

  \brief Interface for the Servolens lens attached to the camera fixed on the 
  Afma4 robot.

*/

class VISP_EXPORT vpServolens
{
 public:
  typedef enum {
    ZOOM = 1,
    FOCUS= 2,
    IRIS = 3
  } vpServoType;
  typedef enum {
    ZOOM_MAX  = 10000,	// Valeur maxi zoom (mm/100)
    ZOOM_MIN  = 1000,	// Valeur mini zoom (mm/100)
    FOCUS_MAX = 1500,	// Valeur maxi focus (metres/100)
    FOCUS_MIN = 100,	// Valeur mini focus (metres/100)
    IRIS_MAX  = 1000,	// Valeur maxi diaph (ouverture/100)
    IRIS_MIN  = 160	// Valeur mini disph (ouverture/100)
  } vpLimitsType;
  typedef enum {
    AUTO       = 1,
    CONTROLLED = 2,
    RELEASED   = 3    
  } vpControllerType;

  vpServolens();
  vpServolens(const char *port);
  ~vpServolens();

  void open(const char *port="/dev/ttyS0");
  void close();
  void reset();

  void setController(vpControllerType controller);
  void setAutoIris(bool enable);
  void setPosition(vpServoType servo, unsigned position);
  bool getPosition(vpServoType servo, unsigned &position);

  void enablePrompt(bool active);

 private:

  void init();

  void enableCmdComplete(vpServoType servo, bool active);

  char wait();
  void wait(vpServoType servo);

  bool read(char *c, long timeout_s);
  void write(const char *s);

  bool clean(const char *in, char *out);

  int remfd; // file pointer of the host's tty
  bool isinit;
};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

