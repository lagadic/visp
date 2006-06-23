/****************************************************************************
 *
 * $Id: vpPoseExample.cpp,v 1.4 2006-06-23 14:45:05 brenier Exp $
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
 * Pose computation on an object made of dots.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoseExample.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoseExample.cpp,v 1.4 2006-06-23 14:45:05 brenier Exp $
 *
 * Description
 * ============
 *   reading of PGM image
 *   Display image using the X11 display
 *   track 4 dots (vpDots) in the image
 *   compute the pose
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpPose.h>
#include <visp/vpDot.h>
#include <visp/vpPixelMeterConversion.h>
/*!
  \example vpPoseExample.cpp

  \brief example of dots tracking in an image sequence and pose
  computation
*/

int
main()
{
  int i ;

  cout <<  "-------------------------------------------------------" << endl ;
  cout <<  " vpPoseExample.cpp" <<endl << endl ;

  cout <<  "  example of dots tracking in an image sequence and pose computation" << endl ;
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
  //  !!      vpPoseExample.cpp: main(#95) :Error while reading the image
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


  // define the vpDot structure, here 4 dots will tracked
  vpDot d[4] ;

  for (i=0 ; i < 4 ; i++)
    {
      // by using setGraphics, we request to see the all the pixel of the dot
      // in green on the screen.
      // It uses the overlay image plane.
      // The default of this setting is that it is time consumming
      d[i].setGraphics(true);
    }

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
    vpERROR_TRACE("Error while reading the image") ;
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
    vpERROR_TRACE("Error while displaying the image") ;
    throw ;
  }


  // dot coordinates (u,v) = (column,row)
  cout << "Click the four white  on the object corner clockwise" <<endl  ;
  double u[4],v[4] ;
  try{
    for (i=0 ; i < 4 ; i++)
      {
	// tracking is initalized
	// if no other parameters are given to the iniTracking(..) method
	// a right mouse click on the dot is expected
	// dot location can also be specified explicitely in the initTracking
	// method  : d.initTracking(I,u,v)  where u is the column index and v is
	// the row index

	d[i].initTracking(I) ;
	// track the dot and returns its coordinates in the image
	// results are given in float since many many are usually considered
	//
	// an expcetion is thrown by the track method if
	//  - dot is lost
	//  - the number of pixel is too small
	//  - too many pixels are detected (this is usual when a "big" specularity
	//    occurs. The threshold can be modified using the
	//    setNbMaxPoint(int) method
	d[i].track(I,u[i],v[i]) ;
     }

  }
  catch(...)
  {
    vpERROR_TRACE("Error in tracking initialization ") ;
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

  for (i=0 ; i < 4 ; i++)
    vpDisplay::displayCross(I,
			    (int)v, (int)u,
			    10,
			    vpColor::red) ;

  // flush the X11 buffer
  vpDisplay::flush(I) ;


  // --------------------------------------------------------
  // Now wil compute the pose
  //

  // The pose will be contained in an homogeneous matrix cMo
  vpHomogeneousMatrix cMo ;


  // We need a structure that content both the 3D coordinates of the point
  // in the object frame and the 2D coordinates of the point expressed in meter
  // the vpPoint class is ok for that
  vpPoint P[4]  ;

  // The vpPose class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
  vpPose pose ;
  //  the list of point is cleared (if that's not done before)
  pose.clearPoint() ;

  // we set the 3D points coordinates (in meter !) in the object/world frame
  double L=0.04 ;
  P[0].setWorldCoordinates(-L,-L, 0 ) ; // (X,Y,Z)
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;


  // set the camera intrinsic parameters
  // see more details about the model in vpCameraParameters
  double px = 600 ;
  double py = 600 ;
  double u0 = 192 ;
  double v0 = 144 ;
  vpCameraParameters cam(px,py,u0,v0) ;

  // pixel-> meter conversion
  for (i=0 ; i < 4 ; i++)
    {
      // u[i]. v[i] are expressed in pixel
      // conversion in meter is achieved using
      // x = (u-u0)/px
      // y = (v-v0)/py
      // where px, py, u0, v0 are the intrinsic camera parameters
      double x,y ;
      vpPixelMeterConversion::convertPoint(cam,
					   u[i], v[i],
					   x,y)  ;
      P[i].set_x(x) ;
      P[i].set_y(y) ;
    }


  // The pose structure is build, we put in the point list the set of point
  // here both 2D and 3D world coordinates are known
  for (i=0 ; i < 4 ; i++)
  {
    pose.addPoint(P[i]) ; // and added to the pose computation point list
  }

  // compute the initial pose using Dementhon method followed by a non linear
  // minimisation method

  // Pose by Lagrange it provides an initialization of the pose
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  // the pose is now refined using the virtual visual servoing approach
  // Warning: cMo needs to be initialized otherwise it may  diverge
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;

  // display the compute pose
  pose.display(I,cMo,cam, 0.05, vpColor::red) ;

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

      // kill the point list
      pose.clearPoint() ;

      // track the dot
      for (i=0 ; i < 4 ; i++)
	{
	  // track the point
	  d[i].track(I,u[i],v[i]) ;

	 // display point location
	  vpDisplay::displayCross(I,(int)v[i], (int)u[i],
				  10,vpColor::red) ;

	  // pixel->meter conversion
	  {
	    double x,y ;
	    vpPixelMeterConversion::convertPoint(cam,
						 u[i], v[i],
						 x,y)  ;
	    P[i].set_x(x) ;
	    P[i].set_y(y) ;
	  }

	  // and added to the pose computation point list
	  pose.addPoint(P[i]) ;
       }
      // the pose structure has been updated


      // the pose is now updated using the virtual visual servoing approach
      // Dementhon or lagrange is no longuer necessary, pose at the
      // previous iteration is sufficient
      pose.computePose(vpPose::VIRTUAL_VS, cMo) ;

      // display the compute pose
      pose.display(I,cMo,cam, 0.05, vpColor::red) ;

      vpDisplay::flush(I) ;
    }
    catch(...)
      {
	vpERROR_TRACE("Error in tracking loop") ;
	throw ;
      }
    iter +=5 ;
  }
}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11 functionalities to display images...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
