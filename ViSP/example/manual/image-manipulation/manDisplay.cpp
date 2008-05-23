/****************************************************************************
 *
 * $Id: manDisplay.cpp,v 1.3 2008-05-23 15:44:42 asaunier Exp $
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
 * Display example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manDisplay.cpp

  \brief Display example.

 */
/*!
  \example manDisplay.cpp

  \brief Display example.
 
 */
#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpColor.h>
#include <visp/vpDisplayGTK.h>

int main()
{
#ifdef VISP_HAVE_GTK

  // Create a grey level image
  vpImage<unsigned char> I ;

  // Load a grey image from the disk
  std::string filename = "/tmp/ViSP-images/Klimt/Klimt.pgm";
  vpImageIo::read(I, filename) ;

  // Create a display using GTK
  vpDisplayGTK display;

  // For this grey level image, open a GTK display at position 100,100
  // in the screen, and with title "GTK display"
  display.init(I, 100, 100, "GTK display") ;

  // Display the image
  vpDisplay::display(I) ;

  // Display in overlay a red cross at position 10,10 in the
  // image. The lines are 10 pixels long
  vpDisplay::displayCross(I, 100,10, 20, vpColor::red) ;

  // Display in overlay a horizontal red line
  vpDisplay::displayLine(I,10,0,10,I.getWidth(), vpColor::red) ;

  // Display in overlay a vertical green dot line
  vpDisplay::displayDotLine(I,0,20,I.getWidth(),20,vpColor::green) ;

  // Display in overlay a blue arrow
  vpDisplay::displayArrow(I,0,0,100,100,vpColor::blue) ;

  // Display in overlay some circles. The position of the center is 200, 200
  // the radius is increased by 20 pixels for each circle
  for (unsigned i=0 ; i < 5 ; i++)
    vpDisplay::displayCircle(I,200,200,20*i,vpColor::yellow) ;

  // Display in overlay a rectangle. 
  // The position of the top left corner is 300, 200.
  // The width is 200. The height is 100.
  vpDisplay::displayRectangle(I,300,200,200,100,vpColor::orange) ;

  // Display in overlay a yellow string
  vpDisplay::displayCharString(I, 85, 100,
			       "ViSP is a marvelous software",
			       vpColor::yellow) ;
  //Flush the display : without this line nothing will appear on the screen
  vpDisplay::flush(I);

  // Create a color image
  vpImage<vpRGBa> Ioverlay ;
  // Updates the color image with the original loaded image and the overlay
  vpDisplay::getImage(I, Ioverlay) ;

  // Write the color image on the disk
  filename = "/tmp/Klimt_grey.overlay.ppm";
  vpImageIo::write(Ioverlay, filename) ;

  // If click is allowed, wait for a mouse click to close the display
  std::cout << "\nA click to close the windows..." << std::endl;
  // Wait for a blocking mouse click
  vpDisplay::getClick(I) ;

  // Close the display
  vpDisplay::close(I);

#endif
} 
