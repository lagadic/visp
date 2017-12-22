/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpFeatureDisplay_H
#define vpFeatureDisplay_H

/*!
  \file vpFeatureDisplay.h
  \brief interface with the image for feature display
*/

// Color / image / display
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

// Meter/pixel conversion
#include <visp3/core/vpCameraParameters.h>

/*!
  \class vpFeatureDisplay

  \ingroup group_core_gui
  \brief Interface with the image for feature display.
*/
class VISP_EXPORT vpFeatureDisplay
{

public:
  static void displayCylinder(double rho1, double theta1, double rho2, double theta2, const vpCameraParameters &cam,
                              const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
                              unsigned int thickness = 1);
  static void displayCylinder(double rho1, double theta1, double rho2, double theta2, const vpCameraParameters &cam,
                              const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
                              unsigned int thickness = 1);

  static void displayEllipse(double x, double y, double mu20, double mu11, double m02, const vpCameraParameters &cam,
                             const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
                             unsigned int thickness = 1);

  static void displayEllipse(double x, double y, double mu20, double mu11, double m02, const vpCameraParameters &cam,
                             const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
                             unsigned int thickness = 1);

  static void displayLine(double rho, double theta, const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                          const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void displayLine(double rho, double theta, const vpCameraParameters &cam, const vpImage<vpRGBa> &I,
                          const vpColor &color = vpColor::green, unsigned int thickness = 1);

  static void displayPoint(double x, double y, const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                           const vpColor &color = vpColor::green, unsigned int thickness = 1);
  static void displayPoint(double x, double y, const vpCameraParameters &cam, const vpImage<vpRGBa> &I,
                           const vpColor &color = vpColor::green, unsigned int thickness = 1);
};

#endif
