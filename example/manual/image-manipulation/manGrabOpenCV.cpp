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
 * Images grabbing example.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGrabOpenCV.cpp

  \brief Images grabbing example with the vpOpenCVGrabber class.

 */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpOpenCVGrabber.h>

int main(){
  vpImage<unsigned char> I; // Grey level image
  
#ifdef VISP_HAVE_OPENCV
  vpOpenCVGrabber g; 	// Create the grabber
   
  g.open();             // Initialize the grabber
 
  g.setWidth(640);
  g.setHeight(480);     // If the camera supports 640x480 image size 
  g.setFramerate(30);   // If the camera supports 30fps framerate
  
  while(1)
    g.acquire(I);       // Acquire an image

#endif
}
