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
 * Test for Virtuose SDK wrapper.
 *
 * Author:
 * Nicolò Pedemonte
 *
 *****************************************************************************/

/*!
  \example testVirtuose.cpp
    Test for reading the Virtuose's joint values inside the main function
  after checking the emergency button.
*/

#include <visp3/robot/vpVirtuose.h>

int main()
{
#if defined(VISP_HAVE_VIRTUOSE)
  try {
    vpVirtuose virtuose;
    virtuose.init();

    bool emergStop = virtuose.getEmergencyStop();
    if (emergStop) {
      std::cout << "The system is operational." << std::endl;
      vpColVector q = virtuose.getArticularPosition();
      std::cout << "The current joint values are : " << q.t() << std::endl;
    } else
      std::cout << "The system is not operational. \nPlease plug the "
                   "emergency stop to the system (or untrigger it)."
                << std::endl;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "You should install Virtuose SDK to use this binary..." << std::endl;
#endif
}
