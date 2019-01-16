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
 * Display example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example manDisplay.cpp

  \brief Display example.

 */

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/io/vpImageIo.h>

int main()
{
  try {
    // Create a grey level image
    vpImage<vpRGBa> I;

    // Create image points for pixel coordinates
    vpImagePoint ip, ip1, ip2;

    // Load a grey image from the disk. Klimt.ppm image is part of the ViSP
    // image data set available from
    // http://www.irisa.fr/lagadic/visp/download.html
    std::string filename = "./Klimt.ppm";
    vpImageIo::read(I, filename);

#ifdef VISP_HAVE_GTK
    // Create a display using GTK
    vpDisplayGTK display;

    // For this grey level image, open a GTK display at position 100,100
    // in the screen, and with title "GTK display"
    display.init(I, 100, 100, "GTK display");

    // Display the image
    vpDisplay::display(I);

    // Display in overlay a red cross at position 100,10 in the
    // image. The lines are 20 pixels long
    ip.set_i(200);
    ip.set_j(200);
    vpDisplay::displayCross(I, ip, 20, vpColor::red, 3);

    // Display in overlay a horizontal red line
    ip1.set_i(10);
    ip1.set_j(0);
    ip2.set_i(10);
    ip2.set_j(I.getWidth());
    vpDisplay::displayLine(I, ip1, ip2, vpColor::red, 3);

    // Display in overlay a vertical green dot line
    ip1.set_i(0);
    ip1.set_j(20);
    ip2.set_i(I.getWidth());
    ip2.set_j(20);
    vpDisplay::displayDotLine(I, ip1, ip2, vpColor::green, 3);

    // Display in overlay a blue arrow
    ip1.set_i(0);
    ip1.set_j(0);
    ip2.set_i(100);
    ip2.set_j(100);
    vpDisplay::displayArrow(I, ip1, ip2, vpColor::blue, 8, 4, 3);

    // Display in overlay some circles. The position of the center is 200, 200
    // the radius is increased by 20 pixels for each circle
    for (unsigned i = 0; i < 5; i++) {
      ip.set_i(200);
      ip.set_j(200);
      vpDisplay::displayCircle(I, ip, 20 * i, vpColor::white, false, 3);
    }

    // Display in overlay a rectangle.
    // The position of the top left corner is 300, 200.
    // The width is 200. The height is 100.
    ip.set_i(280);
    ip.set_j(150);
    vpDisplay::displayRectangle(I, ip, 270, 30, vpColor::purple, false, 3);

    // Display in overlay a yellow string
    ip.set_i(300);
    ip.set_j(160);
    vpDisplay::displayText(I, ip, "ViSP is a marvelous software", vpColor::black);
    // Flush the display : without this line nothing will appear on the screen
    vpDisplay::flush(I);

    // Create a color image
    vpImage<vpRGBa> Ioverlay;
    // Updates the color image with the original loaded image and the overlay
    vpDisplay::getImage(I, Ioverlay);

    // Write the color image on the disk
    filename = "./Klimt.overlay.ppm";
    vpImageIo::write(Ioverlay, filename);

    // If click is allowed, wait for a mouse click to close the display
    std::cout << "\nA click to close the windows..." << std::endl;
    // Wait for a blocking mouse click
    vpDisplay::getClick(I);

    // Close the display
    vpDisplay::close(I);
#endif

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
