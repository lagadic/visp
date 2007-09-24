/****************************************************************************
 *
 * $Id: vpRingLight.h,v 1.2 2007-09-24 13:05:09 fspindle Exp $
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

#if ( (defined UNIX) && (! defined APPLE) ) // Only on Linux for the moment

#  include <visp/vpRingLight.h>
#  include <visp/vpParallelPort.h>

class VISP_EXPORT vpRingLight
{

public:
  vpRingLight();
  ~vpRingLight();

  void activate();

private:
  vpParallelPort parport;

} ;

#endif

#endif
