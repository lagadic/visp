/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test for Virtuose SDK wrapper.
 *
 * Author:
 * Nicolò Pedemonte
 *
 *****************************************************************************/

/*!
  \example testVirtuose.cpp
    Test for reading the Virtuose's joint values inside the main function.
*/

#include <visp3/robot/vpVirtuose.h>

int main()
{
#if defined(VISP_HAVE_VIRTUOSE)
  try {
    vpVirtuose virtuose;
    virtuose.init();

    vpColVector q = virtuose.getArticularPosition();
    std::cout << "Joint position: " << q.t() << std::endl;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "You should install Virtuose SDK to use this binary..." << std::endl;
#endif
}

