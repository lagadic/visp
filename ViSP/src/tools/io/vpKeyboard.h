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
 * This file is part of the ViSP toolkit.
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

#include <iostream>
#include <visp/vpConfig.h>




#if defined UNIX

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
  while (...) {
    ...
    if (keyboard.kbhit()) { // Detect if a key was pressed
      c = keyboard.getchar (); // Get the pressed key
      if (c == 'q' || c == 'Q')
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
