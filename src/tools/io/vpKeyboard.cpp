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



#if defined UNIX
#  include <stdio.h>
#  include <visp/vpKeyboard.h>

/*!
  \file vpKeyboard.cpp
  \brief Keybord management under unix.
*/


/*!
  Activates the raw mode to read keys in an non blocking way.
*/
vpKeyboard::vpKeyboard()
{
  init();
}

/*!
  Stops the raw mode.
*/
vpKeyboard::~vpKeyboard()
{
  end();
}

/*!

  Get the hit key. kbhit() indicates if a key was hitten.
*/
int
vpKeyboard::getchar()
{
  int c;
  c = ::getchar();
  return c;
}

/*!


  \return 1 : if a key was hit.
*/
int
vpKeyboard::kbhit()
{
  struct timeval tv = { 0, 0 };
  fd_set readfds;

  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) == 1;
}

/*!

  Activates the raw mode to read keys in an non blocking way.
*/
void
vpKeyboard::init()
{
  setRawMode(true);
}

/*!

  Stops the raw mode.
*/
void
vpKeyboard::end()
{
  setRawMode(false);
}

/*!
  Turns on/off the 'raw' mode.

  \param active : true to activate the raw mode, false to turn it off.

  If raw mode is active, there is no need to press return to get a key.
*/
void
vpKeyboard::setRawMode(bool active)
{
  if (active) {

    tcgetattr(STDIN_FILENO, &initial_settings);

    new_settings = initial_settings;
    //    cfmakeraw(&new_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_oflag &= ~NL0;
    new_settings.c_oflag &= ~CR0;
    new_settings.c_oflag &= ~TAB0;
    new_settings.c_oflag &=~BS0;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

  }
  else {
    tcsetattr(STDIN_FILENO, TCSANOW, &initial_settings);
  }
}

#endif // defined UNIX
