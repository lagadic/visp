/****************************************************************************
 *
 * $Id: manGrabV4l2.cpp,v 1.3 2008-05-23 15:44:42 asaunier Exp $
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
 * Images grabbing example.
 *
 * Authors:
 * Anthony Saunier
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file manGrabV4l2.cpp

  \brief Images grabbing example with the vpV4l2Grabber class.

 */
/*!
  \example manGrabV4l2.cpp

  \brief Images grabbing example with the vpV4l2Grabber class.
 
 */

#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpV4l2Grabber.h>
int main(){
#ifdef VISP_HAVE_V4L2

  vpImage<unsigned char> I; // Grey level image
  
  vpV4l2Grabber g;
  g.setInput(2);    // Input 2 on the board
  g.setWidth(768);  // Acquired images are 768 width
  g.setHeight(576); // Acquired images are 576 height
  g.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
  g.open(I);        // Open the grabber
  while(1)
    g.acquire(I);     // Acquire a 768x576 grey image

#endif
}
