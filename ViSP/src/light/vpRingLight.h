/****************************************************************************
 *
 * $Id: vpRingLight.h,v 1.6 2008-09-26 15:20:54 fspindle Exp $
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
 * Ring light management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpRingLight_h
#define vpRingLight_h

/*!
  \file vpRingLight.h
  \brief Ring light management under unix.
*/

#include <iostream>

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_PARPORT

#  include <visp/vpRingLight.h>
#  include <visp/vpParallelPort.h>

/*!

  \class vpRingLight
  \ingroup Afma6
  \brief Ring light management under unix.

  \warning This class works only at Irisa with the Edixia's ring light system.

  Here is an example showing how to synchronise the framegrabbing with
  the lighting system:

  \code
  vpItifg8Grabber grabber;
  vpImage<unsigned char> I;

  vpRingLight light; // Open the device to access to the ring light.

  light.pulse(); // Send a pulse to the lighting system
  grabber.acquire(I); // Acquire an image
  \endcode

  Here is an example showing how to turn on the lighting during 10 seconds
  \code
  int nsec = 10; // Time to wait in seconds
  light.on(); // Turn the ring light on
  vpTime::wait(nsec * 1000); // Wait 10 s
  light.off(); // and then turn the ring light off
  \endcode



*/
class VISP_EXPORT vpRingLight
{

public:
  vpRingLight();
  ~vpRingLight();

  void pulse();
  void pulse(double time);
  void on();
  void off();

private:
  vpParallelPort parport;

} ;

#endif

#endif
