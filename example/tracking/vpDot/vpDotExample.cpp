/****************************************************************************
 *
 * $Id: vpDotExample.cpp,v 1.6 2006-05-30 08:42:21 fspindle Exp $
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
 * Dot tracking using vpDot class.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <stdio.h>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot.h>

/*!
  \example vpDotExample.cpp

  Example of dots tracking on an image sequence
*/

int
main()
{
  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " vpDotExample.cpp" <<endl << endl ;

  cout <<  " example of dots tracking on an image sequence" << endl ;
  cout <<  "-------------------------------------------------------" << endl ;
  cout << endl ;



  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I ;

  // Warning :
  // the image sequence is not provided with the ViSP package
  // therefore the program will return you an error :
  //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file /Images/cube-1/image.0001.pgm
  //  !!      vpDotExample.cpp: main(#95) :Error while reading the image
  //  terminate called after throwing an instance of 'vpImageException'
  //
  //  The sequence is available on the visp www site
  //  http://www.irisa.fr/lagadic/visp/visp.html
  //  in the download section. It is named "cube.tar.gz"

  // directory name
  char dir[FILENAME_MAX] ;
  sprintf(dir,"/Images/cube") ;

  // image base name.
  char s[FILENAME_MAX] ;
  int iter = 0 ;
  // set image name to [dir]/image.0000.pgm
  sprintf(s,"%s/image.%04d.pgm",dir,iter) ;


  // define the vpDot structure
  vpDot d ;

  // by using setGraphics, we request to see the all the pixel of the dot
  // in green on the screen.
  // It uses the overlay image plane.
  // The default of this setting is that it is time consumming
  d.setGraphics(true);
  // we also request to compute the dot moment m00, m10, m01, m11, m20, m02
  d.setComputeMoments(true);


  // Read the PGM image named "s" on the disk, and put the bitmap into the
  // image structure I.
  // I is initialized to the correct size
  //
  // exception readPGM may throw various exception if, for example,
  // the file does not exist, or if the memory cannot be allocated
  try{
    vpImageIo::readPGM(I,s) ;
  }
  catch(...)
  {
    // an exception is throwned if an exception from readPGM has been catched
    // here this will result in the end of the program
    // Note that another error message has been printed from readPGM
    // to give more information about the error
    ERROR_TRACE("Error while reading the image") ;
    throw ;
  }

  // We open a window using the X11 window manager
  // it will be located in 100,100 and titled "tracking using vpDot"
  // its size is automatically defined by the image (I) size
  vpDisplayX display(I,100,100,"tracking using vpDot") ;

  try{
    // display the image
    // The image class has a member that specify a pointer toward
    // the display that has been initialized in the display declaration
    // therefore is is no longuer necessary to make a reference to the
    // display variable.
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE("Error while displaying the image") ;
    throw ;
  }


  // tracking is initalized
  // if no other parameters are given to the iniTracking(..) method
  // a right mouse click on the dot is expected
  // dot location can also be specified explicitely in the initTracking
  // method  : d.initTracking(I,u,v)  where u is the column index and v is
  // the row index

  cout << "Click on a dot to initialize the tracking..." << endl;
  try{
    d.initTracking(I) ;
  }
  catch(...)
  {
    ERROR_TRACE("Error in tracking initialization ") ;
    throw ;
  }

  // dot coordinates (u,v) = (column,row)
  double u,v ;

  // track the dot and returns its coordinates in the image
  // results are given in float since many many are usually considered
  //
  // an expcetion is thrown by the track method if
  //  - dot is lost
  //  - the number of pixel is too small
  //  - too many pixels are detected (this is usual when a "big" specularity
  //    occurs. The threshold can be modified using the
  //    setNbMaxPoint(int) method
  try{
    d.track(I,u,v) ;
  }
  catch(...)
  {
    ERROR_TRACE("Error in tracking ") ;
    throw ;
  }

  // display a red cross (size 10) in the image at the dot center
  // of gravity location
  //
  // WARNING
  // in the vpDisplay class member's when pixel coordinates
  // are considered the first element is the row index and the second
  // is the column index:
  //   vpDisplay::displayCross(Image, row index, column index, size, color)
  //   therefore u and v are inverted wrt to the vpDot specification
  // Alternatively, to avoid this problem another set of member have
  // been defined in the vpDisplay class.
  // If the method name is postfixe with _uv the specification is :
  //   vpDisplay::displayCross_uv(Image, column index, row index, size, color)

  vpDisplay::displayCross(I,
			  (int)v, (int)u,
			  10,vpColor::red) ;
  // flush the X11 buffer
  vpDisplay::flush(I) ;

  // this is the loop over the image sequence
  while (iter < 400)
  {
    try {
      // set the new image name
      sprintf(s,"%s/image.%04d.pgm",dir,iter) ;
      cout << s <<endl ;
     // read the image
      vpImageIo::readPGM(I,s) ;
      vpDisplay::display(I) ;

      // track the dot
      d.track(I) ;
      cout << "Moments: \n" << endl;
      cout << "m00: " << d.m00 << endl;
      cout << "m11: " << d.m11 << endl;
      cout << "m02: " << d.m02 << endl;
      cout << "m20: " << d.m20 << endl;
      cout << "m10: " << d.m10 << endl;
      cout << "m01: " << d.m01 << endl;

      // just note that the coordinates can be recovered usint the
      // get_u() and get_v() method
      vpDisplay::displayCross(I,(int)d.get_v(), (int)d.get_u(),
			      10,vpColor::red) ;
      vpDisplay::flush(I) ;
    }
    catch(...)
      {
	ERROR_TRACE("Error in tracking loop") ;
	throw ;
      }
    iter +=5 ;
  }
}
#else
int
main()
{
  ERROR_TRACE("You do not have X11 functionalities to display images...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
