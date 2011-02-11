/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
  \file manGrabDirectShow.cpp

  \brief Images grabbing example with the vpDirectShowGrabber class.

 */
/*!
  \example manGrabDirectShow.cpp

  \brief Images grabbing example with the vpDirectShowGrabber class.
 
 */

#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpDirectShowGrabber.h>

int main()
{
  vpImage<unsigned char> I; // Grey level image
  
#ifdef VISP_HAVE_DIRECTSHOW
  vpDirectShowGrabber g; // Create the grabber
  if(g.getDeviceNumber() == 0) //test if a camera is connected
  {
    g.close();
    return -1;
  }
  
  g.open(); // Initialize the grabber
  
  g.setImageSize(640,480); // If the camera supports 640x480 image size
  g.setFramerate(30); // If the camera supports 30fps framerate
  
  while(1)
    g.acquire(I); // Acquire an image
#endif

  return 0;
}
