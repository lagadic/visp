/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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

  \param thickness : Thickness of the feature representation.

*/
void
vpServoDisplay::display(vpServo &s,
                        const vpCameraParameters &cam,
                        vpImage<unsigned char> &I,
                        vpColor currentColor,
                        vpColor desiredColor,
			unsigned int thickness)
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
      s_ptr = s.desiredFeatureList.value() ;
      s_ptr->display(cam, I, desiredColor, thickness ) ;
    }
    if (currentColor != vpColor::none) {
      // current list
      s_ptr =  s.featureList.value() ;
      s_ptr->display(cam, I, currentColor, thickness ) ;
    }
  }
  //  vpDisplay::flush(I) ;
}

/*!

  Display the current and the desired features in the image I.

  \warning To effectively display the dot graphics a call to
  vpDisplay::flush() is needed.

  \param s : Visual servoing control law.
  \param cam : Camera parameters.
  \param I : Color image on which features have to be displayed.

  \param currentColor : Color for the current features. If vpColor::none,
  current features display is turned off.

  \param desiredColor : Color for the desired features. If vpColor::none,
  desired features display is turned off.

  \param thickness : Thickness of the feature representation.

 */
void
vpServoDisplay::display(vpServo &s,
                        const vpCameraParameters &cam,
                        vpImage<vpRGBa> &I,
                        vpColor currentColor,
                        vpColor desiredColor,
			unsigned int thickness)
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
      s_ptr = s.desiredFeatureList.value() ;
      s_ptr->display(cam, I, desiredColor, thickness ) ;
    }
    if (currentColor != vpColor::none) {
      // current list
      s_ptr =  s.featureList.value() ;
      s_ptr->display(cam, I, currentColor, thickness ) ;
    }
  }
  //  vpDisplay::flush(I) ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
