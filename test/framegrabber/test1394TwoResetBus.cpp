/****************************************************************************
 *
 * $Id: test1394TwoResetBus.cpp,v 1.2 2008-06-06 11:26:41 asaunier Exp $
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
 * Firewire cameras video capture.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file test1394TwoResetBus.cpp

  \brief Resets the IEEE1394 bus using libdc1394-2.x library.

*/

#include <iostream>



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if defined(VISP_HAVE_DC1394_2)

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>


/*!
  \example test1394TwoResetBus.cpp

  Resets the IEEE1394 bus which first camera is attached to. Resetting
  the bus is "rude" to other devices because it causes them to
  re-enumerate on the bus and may cause a temporary disruption in
  their current activities.  Thus, use it sparingly.  Its primary use
  is if a program shuts down uncleanly and needs to free leftover ISO
  channels or bandwidth.  A bus reset will free those things as a side
  effect.

*/
int
main()
{
  try  {
    std::cout << "IEEE1394 bus reset in progress..." << std::endl;
    vp1394TwoGrabber g;
    g.resetBus(); // Reset the bus attached to the first camera found
    std::cout << "IEEE1394 bus was reset." << std::endl;
      
    vpImage<unsigned char> I;
    g.acquire(I);    
//     std::cout << "write /tmp/test.pgm" << std::endl;
//     vpImageIo::writePGM(I, "/tmp/test.pgm");
  }
  catch (...) {
    vpCERROR << "Failure: exit" << std::endl;
  }
}
#else
int
main()
{
  vpTRACE("Ieee 1394 grabber capabilities are not available...\n"
	  "You should install libdc1394-2 to use this binary.") ;
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
