/****************************************************************************
 *
 * $Id: keyboard.cpp,v 1.6 2008-02-01 17:28:56 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if defined UNIX
#include <stdio.h>
#include <iostream>
#include <signal.h>

#include <visp/vpKeyboard.h>


int
main()
{
  int c;
  {
    std::cout << "Push some characters on the keyboard..." << std::endl;
    printf("Hit 'q' or 'Q' to stop the loop ...\n");
    vpKeyboard keyboard;

    std::cout << "Start the keyboard scrutation..." << std::endl;
    while (1) {

      if (keyboard.kbhit()) {
	c = keyboard.getchar () ;
	printf("You hit key: %d '%c'\n", c, c);
	if (c == 'q' || c == 'Q') {
	  printf("You hit key: %d %c we stop the loop\n", c, c);
	  break ;
	}
      }

      // My job is here

    }
  }

  std::cout << "Enter an integer: ";
  int myvalue;
  std::cin >> myvalue;

  return 0;
}
#else
int
main()
{
  vpTRACE("Sorry, for the moment, vpKeyboard class works only on unix...");
  return 0;
}
#endif
