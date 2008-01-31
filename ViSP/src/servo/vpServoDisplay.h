/****************************************************************************
 *
 * $Id: vpServoDisplay.h,v 1.5 2008-01-31 14:51:43 asaunier Exp $
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

/*!
  \class vpServoDisplay
  \brief interface with the image for feature display
*/
class VISP_EXPORT vpServoDisplay
{

public:
  static void display(vpServo &s,
		      const vpCameraParameters &cam,
		      vpImage<unsigned char> &I,
		      vpColor::vpColorType currentColor = vpColor::green,
		      vpColor::vpColorType desiredColor = vpColor::red) ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
