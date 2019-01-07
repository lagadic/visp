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
 * Keybord management.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
#include <stdio.h>
#include <visp3/io/vpKeyboard.h>

/*!
  \file vpKeyboard.cpp
  \brief Keybord management under unix.
*/

/*!
  Activates the raw mode to read keys in an non blocking way.
*/
vpKeyboard::vpKeyboard() : initial_settings(), new_settings() { init(); }

/*!
  Stops the raw mode.
*/
vpKeyboard::~vpKeyboard() { end(); }

/*!

  Get the hit key. kbhit() indicates if a key was hitten.
*/
int vpKeyboard::getchar()
{
  int c;
  c = ::getchar();
  return c;
}

/*!


  \return 1 : if a key was hit.
*/
int vpKeyboard::kbhit()
{
  struct timeval tv = {0, 0};
  fd_set readfds;

  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) == 1;
}

/*!

  Activates the raw mode to read keys in an non blocking way.
*/
void vpKeyboard::init() { setRawMode(true); }

/*!

  Stops the raw mode.
*/
void vpKeyboard::end() { setRawMode(false); }

/*!
  Turns on/off the 'raw' mode.

  \param active : true to activate the raw mode, false to turn it off.

  If raw mode is active, there is no need to press return to get a key.
*/
void vpKeyboard::setRawMode(bool active)
{
  if (active) {

    tcgetattr(STDIN_FILENO, &initial_settings);

    // new_settings = initial_settings;
    //    cfmakeraw(&new_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= (unsigned int)~ICANON;
    new_settings.c_lflag &= (unsigned int)~ECHO;
    new_settings.c_lflag &= (unsigned int)~ISIG;
    // new_settings.c_oflag &= (unsigned int)~NL0;
    // new_settings.c_oflag &= (unsigned int)~CR0;
    new_settings.c_oflag &= (unsigned int)~TAB0;
    // new_settings.c_oflag &= (unsigned int)~BS0;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

  } else {
    tcsetattr(STDIN_FILENO, TCSANOW, &initial_settings);
  }
}

#endif // defined UNIX
