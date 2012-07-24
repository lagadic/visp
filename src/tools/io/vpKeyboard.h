/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Keybord management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpKeyboard_h
#define vpKeyboard_h

/*!
  \file vpKeyboard.h
  \brief Keybord management under unix.
*/

#include <visp/vpConfig.h>

#if ( defined(UNIX) && !defined(WIN32) )

#  include <iostream>
#  include <termios.h>
#  include <unistd.h>
#  include <stdlib.h>


/*!

  \class vpKeyboard
  \ingroup Keyboard
  \brief Keybord management under unix.

  Gets a key from the keyboard without waiting for the enter key.

  \code
  vpKeyboard keyboard; // Turn on keyboard raw mode
  int c;
  while (...) {
    ...
    if (keyboard.kbhit()) { // Detect if a key was pressed
      c = keyboard.getchar (void); // Get the pressed key
      if (c == 'q' || c == 'Q') {
        break; // Quit the while()
      }
    ...
    }
  }

  // Keyboard raw mode is turned off by the vpKeyboard destructor
  \endcode

*/
class VISP_EXPORT vpKeyboard
{

public:
  vpKeyboard();
  ~vpKeyboard();
  int kbhit();
  int getchar();

private:
  void init();
  void end();

  void setRawMode(bool active);

  struct termios initial_settings, new_settings;
} ;

#endif // defined UNIX

#endif
