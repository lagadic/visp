/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Sick LD-MRS laser driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example SickLDMRS-Acq.cpp

  \brief Example that shows how to acquire Sick LD-MRS laser
  measurements.  

  \warning For the moment, this example is only working on UNIX
  platforms since the Sick LD-MRS driver was not ported to Windows.

  

*/
#include <visp/vpDebug.h>
#include <visp/vpSickLDMRS.h>
#include <visp/vpParseArgv.h>


#ifdef UNIX

int main()
{
  vpSickLDMRS laser;
  std::string ip = "131.254.12.119";
  
  laser.setIpAddress(ip);
  laser.setup();
  unsigned long int iter = 0;
 
  for ( ; ; ) {
    double t1 = vpTime::measureTimeMs();
    vpLaserScan laserscan[4];
    if (laser.measure(laserscan) == false)
      continue;
    
    iter ++;
    std::cout << "iter: " << iter << " time: " 
	      << vpTime::measureTimeMs() - t1 << " ms" << std::endl;
  }
  return 0;
}

#else // #ifdef UNIX

int main()
{ 
  std::cout << "This example is only working on UNIX platforms \n"
	    << "since the Sick LD-MRS driver was not ported to Windows." 
	    << std::endl;
 
  return 0;
}
#endif // #ifdef UNIX
