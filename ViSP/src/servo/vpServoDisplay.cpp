/****************************************************************************
 *
 * $Id: vpServoDisplay.cpp,v 1.5 2007-03-14 08:58:11 fspindle Exp $
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

/*!
  \file vpServoDisplay.cpp
  \brief interface with the image for feature display
*/

// Servo
#include <visp/vpServo.h>

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>

#include <visp/vpDisplay.h>

#include <visp/vpServoDisplay.h>

#include <visp/vpBasicFeature.h>

/*!

  Display the current and the desired features in the image I.

  \warning To effectively display the dot graphics a call to
  vpDisplay::flush() is needed.

  \param s : Visual servoing control law.
  \param cam : Camera parameters.
  \param I : Image on which features have to be displayed.

  \param currentColor : Color for the current features. If vpColor::none,
  current features display is turned off.

  \param desiredColor : Color for the desired features. If vpColor::none,
  desired features display is turned off.
*/
void
vpServoDisplay::display(vpServo &s,
			const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			vpColor::vpColorType currentColor,
			vpColor::vpColorType desiredColor)
{



  for (s.featureList.front(),
	 s.desiredFeatureList.front() ;
       !s.featureList.outside() ;
       s.featureList.next(),
	 s.desiredFeatureList.next() )
  {
    vpBasicFeature *s_ptr = NULL;

    if (desiredColor != vpColor::none) {
      // desired list
      s_ptr=  s.desiredFeatureList.value() ;
      s_ptr->display(cam, I, desiredColor ) ;
    }
    if (currentColor != vpColor::none) {
      // current list
      s_ptr =  s.featureList.value() ;
      s_ptr->display(cam, I, currentColor ) ;
    }
  }


  //  vpDisplay::flush(I) ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
