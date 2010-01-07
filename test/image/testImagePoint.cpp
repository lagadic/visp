/****************************************************************************
 *
 * $Id$
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
