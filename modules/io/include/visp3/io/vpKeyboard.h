/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#include <visp3/core/vpConfig.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

/*!

  \class vpKeyboard
  \ingroup group_io_keyboard
  \brief Keybord management under unix (Linux or OSX). This class is not
  available under windows.

  Gets a key from the keyboard without waiting for the "Enter" key.
  \warning The key that was pressed is only detected if the terminal where the
  binary was launched is active.

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
};

#endif // defined UNIX

#endif
