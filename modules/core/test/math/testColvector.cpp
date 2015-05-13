/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Test some vpColVector functionalities.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \example testColvector.cpp

  Test some vpColVector functionalities.
*/

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>

#include <stdlib.h>
#include <stdio.h>


int main()
{
  try {
    vpColVector V(4) ;
    V = 1.0;

    vpTRACE("------------------------");
    vpTRACE("call std::cout << V;");
    std::cout << V << std::endl;

    vpTRACE("------------------------");
    vpTRACE("call V.normalize();");
    V.normalize();

    vpTRACE("------------------------");
    vpTRACE("call std::cout << V;");
    std::cout << V << std::endl;
    return (0);
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return (1);
  }
}
