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
 * See the file LICENSE.txt at the root directory of this source
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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpServoDisplay_H
#define vpServoDisplay_H

/*!
  \file vpServoDisplay.h
  \brief interface with the image for feature display
*/

#include <visp/vpConfig.h>

// Servo
#include <visp/vpServo.h>

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
/*!
  \class vpServoDisplay
  \ingroup VsTask
  \brief Interface with the image for feature display.
*/
class VISP_EXPORT vpServoDisplay
{
public:
  static void display(vpServo &s,
		      const vpCameraParameters &cam,
		      vpImage<unsigned char> &I,
		      vpColor currentColor = vpColor::green,
		      vpColor desiredColor = vpColor::red,
		      unsigned int thickness=1) ;
  static void display(vpServo &s,
                      const vpCameraParameters &cam,
                      vpImage<vpRGBa> &I,
                      vpColor currentColor = vpColor::green,
                      vpColor desiredColor = vpColor::red,
		      unsigned int thickness=1) ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
