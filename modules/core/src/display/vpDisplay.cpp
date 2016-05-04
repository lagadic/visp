/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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
  : displayHasBeenInitialized(false), windowXPosition(0), windowYPosition(0), width(0), height(0), title_() {}

/*!
  Copy constructor.
*/
vpDisplay::vpDisplay(const vpDisplay &d)
  : displayHasBeenInitialized(false), windowXPosition(0), windowYPosition(0), width(0), height(0), title_()
{
  displayHasBeenInitialized = d.displayHasBeenInitialized;
  windowXPosition = d.windowXPosition;
  windowYPosition = d.windowYPosition;

  width  = d.width;
  height = d.height;
}

/*!
  Destructor that desallocates memory.
*/
vpDisplay::~vpDisplay()
{
  displayHasBeenInitialized = false ;
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
  vpDisplay::getImage(I, Ioverlay) ;

  // Write the color image on the disk
  std::string ofilename("overlay.ppm");
  vpImageIo::write(Ioverlay, ofilename) ;

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
  try
  {
    if ( Isrc.display != NULL )
    {
      ( Isrc.display )->getImage ( Idest ) ;
    }
    else
    {
      vpImageConvert::convert(Isrc, Idest);
//      vpERROR_TRACE ( "Display not initialized" ) ;
//      throw ( vpDisplayException ( vpDisplayException::notInitializedError,
//                                   "Display not initialized" ) ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
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
  vpDisplay::getImage(I, Ioverlay) ;

  // Write the color image on the disk
  std::string ofilename("overlay.ppm");
  vpImageIo::write(Ioverlay, ofilename) ;

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

  try
  {
    if ( Isrc.display != NULL )
    {
      ( Isrc.display )->getImage ( Idest ) ;
    }
    else {
      Idest = Isrc;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
