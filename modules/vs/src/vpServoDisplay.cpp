/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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
#include <visp3/vs/vpServo.h>

// Meter/pixel conversion
#include <visp3/core/vpCameraParameters.h>

// Color / image / display
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

#include <visp3/core/vpDisplay.h>

#include <visp3/vs/vpServoDisplay.h>

#include <visp3/visual_features/vpBasicFeature.h>

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
void vpServoDisplay::display(const vpServo &s, const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                             vpColor currentColor, vpColor desiredColor, unsigned int thickness)
{
  std::list<vpBasicFeature *>::const_iterator it_s;
  std::list<vpBasicFeature *>::const_iterator it_s_star;

  for (it_s = s.featureList.begin(), it_s_star = s.desiredFeatureList.begin(); it_s != s.featureList.end();
       ++it_s, ++it_s_star) {
    if (desiredColor != vpColor::none) {
      // desired list
      (*it_s_star)->display(cam, I, desiredColor, thickness);
    }
    if (currentColor != vpColor::none) {
      // current list
      (*it_s)->display(cam, I, currentColor, thickness);
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
void vpServoDisplay::display(const vpServo &s, const vpCameraParameters &cam, const vpImage<vpRGBa> &I,
                             vpColor currentColor, vpColor desiredColor, unsigned int thickness)
{
  std::list<vpBasicFeature *>::const_iterator it_s;
  std::list<vpBasicFeature *>::const_iterator it_s_star;

  for (it_s = s.featureList.begin(), it_s_star = s.desiredFeatureList.begin(); it_s != s.featureList.end();
       ++it_s, ++it_s_star) {
    if (desiredColor != vpColor::none) {
      // desired list
      (*it_s_star)->display(cam, I, desiredColor, thickness);
    }
    if (currentColor != vpColor::none) {
      // current list
      (*it_s)->display(cam, I, currentColor, thickness);
    }
  }
  //  vpDisplay::flush(I) ;
}
