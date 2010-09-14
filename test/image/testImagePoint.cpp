/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Test for vpImagePoint class.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example testImagePoint.cpp

  \brief Test vpImagePoint functionalities.

*/
#include <iostream>

#include <visp/vpImagePoint.h>

int main()
{
  vpImagePoint ip1;

  ip1.set_u(11.1);
  ip1.set_v(10);

  std::cout << "We define a first image point with coordinates: " 
	    << ip1 << std::endl;

  vpImagePoint ip2;

  ip2.set_j(11.1);
  ip2.set_i(10);
  
  std::cout << "We define a second image point with coordinates: " 
	    << ip2 << std::endl;

  if (ip1 != ip2) {
    std::cout << "Image point are different :-(!" << std::endl;
    return -1;
  }

  if (ip1 == ip2) {
    std::cout << "Image point are the same :-)" << std::endl;
  }

  return 0;
}
