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
 * Sick LD-MRS laser driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example SickLDMRS-DisplayData.cpp

  \brief Example that shows how to acquire Sick LD-MRS laser
  measurements and how to process the data in order to display the
  laser scans.  

  \warning For the moment, this example is only working on UNIX
  platforms since the Sick LD-MRS driver was not ported to Windows.

  Layer 1 is displayed in red, layer 2 in green, layer 3 in blue and layer
  4 in yellow.

  Thanks to the -layer command line option, this example allows to
  select the layers that are displayed.

*/
#include <visp/vpDebug.h>
#include <visp/vpImagePoint.h>
#include <visp/vpSickLDMRS.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpParseArgv.h>

int main(int argc, const char ** argv)
{
#ifdef UNIX
  vpSickLDMRS laser;
  std::string ip = "131.254.12.119";
  
  int layerToDisplay = 0x1;

  // Parse the command line to set the variables
  vpParseArgv::vpArgvInfo argTable[] =
    {
      {"-layer", vpParseArgv::ARGV_INT, (char*) NULL, (char *) &layerToDisplay,
       "The layer to display:\n" 
       "\t\t. 0x1 for layer 1.\n"
       "\t\t. 0x2 for layer 2.\n"
       "\t\t. 0x4 for layer 3.\n"
       "\t\t. 0x8 for layer 4.\n"
       "\t\tTo display all the layers you should set 0xF value.\n"
      },
      {"-h", vpParseArgv::ARGV_HELP, (char*) NULL, (char *) NULL,
       "Display one or more measured layers form a Sick LD-MRS laser scanner."},
      {(char*) NULL, vpParseArgv::ARGV_END, (char*) NULL, (char*) NULL, (char*) NULL}
    } ;

  // Read the command line options
  if(vpParseArgv::parse(&argc, argv, argTable, 
            vpParseArgv::ARGV_NO_LEFTOVERS |
            vpParseArgv::ARGV_NO_ABBREV | 
            vpParseArgv::ARGV_NO_DEFAULTS)) {
    return (false);
  }


  laser.setIpAddress(ip);

  laser.setup();
 
  vpImage<unsigned char> map(700, 300);
  unsigned int width = map.getWidth();
  unsigned int height = map.getHeight();
  vpImagePoint O; // Beam origin
  O.set_i(height);
  O.set_j(width/2.);
  vpScanPoint p;
  vpColor color;

#if defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
#endif

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined (VISP_HAVE_GTK) )
  display.init(map);
#endif

  while(1) {
    double t1 = vpTime::measureTimeMs();
    vpLaserScan laserscan[4];
    laser.measure(laserscan);
    
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined (VISP_HAVE_GTK) )
    vpDisplay::display(map);
#endif
    
    // Parse the four layers
    for (int layer=0; layer<4; layer++) {
      if (! (int(pow(2, layer)) & layerToDisplay)) {
	std::cout << "Layer " << layer << " is not to display" << std::endl;
	continue;
      }
      
      switch (layer) {
      case 0: color = vpColor::red; break;
      case 1: color = vpColor::green; break;
      case 2: color = vpColor::blue; break;
      case 3: color = vpColor::yellow; break;
      }
      std::vector<vpScanPoint> pointsLayer = laserscan[layer].getScanPoints();      
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined (VISP_HAVE_GTK) )
     
      vpImagePoint E; // Beam echo
      double resolution = 100; // 100 pixels = 1 meter
      for (unsigned int i=0; i<pointsLayer.size(); i++) {
	p = pointsLayer[i];
	E.set_i(height   - resolution * p.getRadialDist() * cos(p.getHAngle()));
	E.set_j(width/2. - resolution * p.getRadialDist() * sin(p.getHAngle()));
	//std::cout << "E: " << E << std::endl;
	vpDisplay::displayLine(map, O, E, color);
      }
#endif
    }
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined (VISP_HAVE_GTK) )
    vpDisplay::flush(map);
#endif
    std::cout << "time: " << vpTime::measureTimeMs() - t1 << std::endl;

  }
#else
  std::cout << "This example is only working on UNIX platforms \n"
	    << "since the Sick LD-MRS driver was not ported to Windows." 
	    << std::endl;
#endif
 
  return 0;
}
