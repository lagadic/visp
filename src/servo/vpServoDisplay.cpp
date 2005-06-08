
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpServoDisplay.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpServoDisplay.cpp,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


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

void
vpServoDisplay::display(vpServo &s,
			const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			int currentColor,
			int desiredColor)
{



  for (s.featureList.front(),
	 s.desiredFeatureList.front() ;
       !s.featureList.outside() ;
       s.featureList.next(),
	 s.desiredFeatureList.next() )
  {
    vpBasicFeature *s_ptr = NULL;

    // desired list
    s_ptr=  s.desiredFeatureList.value() ;
    s_ptr->display(cam, I, desiredColor ) ;

    // current list
    s_ptr =  s.featureList.value() ;
    s_ptr->display(cam, I, currentColor ) ;
  }


  vpDisplay::flush(I) ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
