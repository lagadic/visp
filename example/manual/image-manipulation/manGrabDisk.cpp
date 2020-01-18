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
 * Images grabbing example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGrabDisk.cpp

  \brief Images grabbing example with the vpDiskGrabber class.

 */
/*!
  \example manGrabDisk.cpp

  \brief Images grabbing example with the vpDiskGrabber class.

 */

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpImage.h>
#include <visp3/io/vpDiskGrabber.h>

int main()
{
  try {
    vpImage<unsigned char> I; // Grey level image

    // Declare a framegrabber able to read a sequence of successive
    // images from the disk
    vpDiskGrabber g;

    // Set the path to the directory containing the sequence
    g.setDirectory("/tmp");
    // Set the image base name. The directory and the base name constitute
    // the constant part of the full filename
    g.setBaseName("image");
    // Set the step between two images of the sequence
    g.setStep(3);
    // Set the number of digits to build the image number
    g.setNumberOfZero(4);
    // Set the first frame number of the sequence
    g.setImageNumber(1);
    // Set the file extension of the images of the sequence
    g.setExtension("pgm");

    // Open the framegrabber by loading the first image of the sequence
    g.open(I);

    // this is the loop over the image sequence
    for (int cpt = 0; cpt < 100; cpt++) {
      // read the image and then increment the image counter so that the next
      // call to acquire(I) will get the next image
      g.acquire(I);
    }

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
