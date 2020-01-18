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
 * Example of keyboard management.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example keyboard.cpp

  Keyboard example.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
#include <iostream>
#include <signal.h>
#include <stdio.h>

#include <visp3/io/vpKeyboard.h>

int main()
{
  std::cout << "Push some characters on the keyboard..." << std::endl;
  printf("Hit 'q' or 'Q' to stop the loop ...\n");
  vpKeyboard keyboard;

  std::cout << "Start the keyboard scrutation..." << std::endl;
  for (;;) {

    if (keyboard.kbhit()) {
      int c = keyboard.getchar();
      printf("You hit key: %d '%c'\n", c, c);
      if (c == 'q' || c == 'Q') {
        printf("You hit key: %d %c we stop the loop\n", c, c);
        break;
      }
    }

    // My job is here
  }

  std::cout << "Enter an integer: ";
  int myvalue;
  std::cin >> myvalue;

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "vpKeyboard class works only on unix..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
