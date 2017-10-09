/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Image display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <limits>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpDisplayException.h>
#include <visp3/core/vpImageConvert.h>

#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpMath.h>

/*!
  \file vpDisplay.cpp
  \brief Generic class for image display.
*/

/*!
  Default constructor.
*/
vpDisplay::vpDisplay()
  : m_displayHasBeenInitialized(false), m_windowXPosition(0), m_windowYPosition(0),
    m_width(0), m_height(0), m_title(), m_scale(1), m_scaleType(SCALE_DEFAULT)
{
}

/*!
  Copy constructor.
*/
vpDisplay::vpDisplay(const vpDisplay &d)
  : m_displayHasBeenInitialized(d.m_displayHasBeenInitialized),
    m_windowXPosition(d.m_windowXPosition), m_windowYPosition(d.m_windowYPosition),
    m_width(d.m_width), m_height(d.m_height), m_title(d.m_title),
    m_scale(d.m_scale), m_scaleType(d.m_scaleType)
{
}

/*!
  Destructor that desallocates memory.
*/
vpDisplay::~vpDisplay()
{
  m_displayHasBeenInitialized = false;
}

/*!
  Get the window pixmap and put it in vpRGBa image.

  The code below shows how to use this method.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main()
{
  vpImage<unsigned char> I(240, 320); // Create a black grey level image
  vpImage<vpRGBa> Ioverlay;

  vpDisplay *d;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Updates the color image with the original loaded image and the overlay
  vpDisplay::getImage(I, Ioverlay);

  // Write the color image on the disk
  std::string ofilename("overlay.ppm");
  vpImageIo::write(Ioverlay, ofilename);

  // Wait for a click in the display window
  vpDisplay::getClick(I);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
  \endcode
*/
void
vpDisplay::getImage(const vpImage<unsigned  char> &Isrc, vpImage<vpRGBa> &Idest )
{
  if ( Isrc.display != NULL )
  {
    ( Isrc.display )->getImage ( Idest );
  }
  else
  {
    vpImageConvert::convert(Isrc, Idest);
  }
}

/*!
  Get the window pixmap and put it in vpRGBa image.

  The code below shows how to use this method.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black RGB color image
  vpImage<vpRGBa> Ioverlay;

  vpDisplay *d;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Updates the color image with the original loaded image and the overlay
  vpDisplay::getImage(I, Ioverlay);

  // Write the color image on the disk
  std::string ofilename("overlay.ppm");
  vpImageIo::write(Ioverlay, ofilename);

  // Wait for a click in the display window
  vpDisplay::getClick(I);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
  \endcode
*/
void
vpDisplay::getImage(const vpImage<vpRGBa> &Isrc, vpImage<vpRGBa> &Idest)
{
  if ( Isrc.display != NULL )
  {
    ( Isrc.display )->getImage ( Idest );
  }
  else {
    Idest = Isrc;
  }
}

/*!
  Set the down scale factor applied to the image in order to reduce the display size.
  \param scale : Scale factor applied to display a rescaled image.
 */
void vpDisplay::setDownScalingFactor(unsigned int scale)
{
  if (! m_displayHasBeenInitialized)
    m_scale = scale;
  else {
    std::cout << "Warning: Cannot apply the down scaling factor " << scale << " to the display window since the display is initialized yet..." << std::endl;
  }
}

/*!
 * Computes the down scaling factor that should be applied to the window size to display
 * the image given the resolution of the screen.
 * \param width, height : Image size.
 * \return
 */
unsigned int vpDisplay::computeAutoScale(unsigned int width, unsigned int height)
{
  unsigned int screen_width, screen_height;
  getScreenSize(screen_width, screen_height);
  double wscale = (std::max)(1., ceil(2.*(double)width / (double)screen_width));
  double hscale = (std::max)(1., ceil(2.*(double)height / (double)screen_height));
  unsigned int scale = (unsigned int)(std::max)(1u, (std::max)((unsigned int)wscale, (unsigned int)hscale));
  return scale;
}

/*!
 * Set the down scaling factor either in auto mode or set manually.
 */
void vpDisplay::setScale(vpScaleType scaleType, unsigned int width, unsigned int height)
{
  switch (scaleType) {
  case vpDisplay::SCALE_AUTO:
    setDownScalingFactor( computeAutoScale(width, height) );
    break;
  case vpDisplay::SCALE_DEFAULT:
  case vpDisplay::SCALE_1:
    break;
  case vpDisplay::SCALE_2:
    setDownScalingFactor(2);
    break;
  case vpDisplay::SCALE_3:
    setDownScalingFactor(3);
    break;
  case vpDisplay::SCALE_4:
    setDownScalingFactor(4);
    break;
  case vpDisplay::SCALE_5:
    setDownScalingFactor(5);
    break;
  case vpDisplay::SCALE_6:
    setDownScalingFactor(6);
    break;
  case vpDisplay::SCALE_7:
    setDownScalingFactor(7);
    break;
  case vpDisplay::SCALE_8:
    setDownScalingFactor(8);
    break;
  case vpDisplay::SCALE_9:
    setDownScalingFactor(9);
    break;
  case vpDisplay::SCALE_10:
    setDownScalingFactor(10);
    break;
  }
}

/*!
   Set the down scaling factor either in auto mode or set manually.

   This method has to be called before display initialization.

   \code
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

int main()
{
  vpImage<unsigned char> I(480, 640); // Black 640 by 480 image
#ifdef VISP_HAVE_X11
  vpDisplayX d;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI d;
#endif
  d.setDownScalingFactor(vpDisplay::SCALE_4); // Display in a 160 by 120 windows size
  d.init(I);
  vpDisplay::display(I);
  vpDisplay::flush(I);
  vpDisplay::getClick(I); // wait for a click to quit
}
   \endcode
 */
void vpDisplay::setDownScalingFactor(vpScaleType scaleType)
{
  if (! m_displayHasBeenInitialized)
    m_scaleType = scaleType;
}
