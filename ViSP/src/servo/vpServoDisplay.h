
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpServoDisplay.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpServoDisplay.h,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
 *
 * Description
 * ============
 *     interface with the image for feature display
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpServoDisplay_H
#define vpServoDisplay_H

/*!
  \file vpServoDisplay.h
  \brief interface with the image for feature display
*/

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
class vpServoDisplay
{

public:
  static void display(vpServo &s,
		      const vpCameraParameters &cam,
		      vpImage<unsigned char> &I,
		      int currentColor = vpColor::green,
		      int desiredColor = vpColor::red) ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
