/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
#include <visp/vpImagePoint.h>

int main()
{
  // Create a grey level image
  vpImage<unsigned char> I ;

  // Create image points for pixel coordinates
  vpImagePoint ip, ip1, ip2;

  // Load a grey image from the disk
  std::string filename = "/tmp/ViSP-images/Klimt/Klimt.pgm";
  vpImageIo::read(I, filename) ;

#ifdef VISP_HAVE_GTK
  // Create a display using GTK
  vpDisplayGTK display;

  // For this grey level image, open a GTK display at position 100,100
  // in the screen, and with title "GTK display"
  display.init(I, 100, 100, "GTK display") ;

  // Display the image
  vpDisplay::display(I) ;

  // Display in overlay a red cross at position 100,10 in the
  // image. The lines are 20 pixels long
  ip.set_i( 100 );
  ip.set_j( 10 );
  vpDisplay::displayCross(I, ip, 20, vpColor::red) ;

  // Display in overlay a horizontal red line
  ip1.set_i( 10 );
  ip1.set_j( 0 );
  ip2.set_i( 10 );
  ip2.set_j( I.getWidth() );
  vpDisplay::displayLine(I, ip1, ip2, vpColor::red) ;

  // Display in overlay a vertical green dot line
  ip1.set_i( 0 );
  ip1.set_j( 20 );
  ip2.set_i( I.getWidth() );
  ip2.set_j( 20 );
  vpDisplay::displayDotLine(I, ip1, ip2, vpColor::green) ;

  // Display in overlay a blue arrow
  ip1.set_i( 0 );
  ip1.set_j( 0 );
  ip2.set_i( 100 );
  ip2.set_j( 100 );
  vpDisplay::displayArrow(I, ip1, ip2, vpColor::blue) ;

  // Display in overlay some circles. The position of the center is 200, 200
  // the radius is increased by 20 pixels for each circle
  for (unsigned i=0 ; i < 5 ; i++) {
    ip.set_i( 200 );
    ip.set_j( 200 );
    vpDisplay::displayCircle(I, ip, 20*i, vpColor::yellow) ;
  }

  // Display in overlay a rectangle. 
  // The position of the top left corner is 300, 200.
  // The width is 200. The height is 100.
  ip.set_i( 300 );
  ip.set_j( 200 );  
  vpDisplay::displayRectangle(I, ip, 200, 100,vpColor::orange) ;

  // Display in overlay a yellow string
  ip.set_i( 85 );
  ip.set_j( 100 );  
  vpDisplay::displayCharString(I, ip,
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

  return 0;
} 
