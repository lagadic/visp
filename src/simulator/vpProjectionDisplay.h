
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpProjectionDisplay.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpProjectionDisplay.h,v 1.2 2005-09-02 14:35:18 fspindle Exp $
 *
 * Description
 * ============
 *     interface with the image for feature display
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpProjectionDisplay_H
#define vpProjectionDisplay_H

/*!
  \file vpProjectionDisplay.h
  \brief interface with the image for feature display
*/

#include <visp/vpConfig.h>
#ifdef HAVE_LIBX11

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpForwardProjection.h>
#include <visp/vpList.h>

/*!
  \class vpProjectionDisplay
  \brief interface with the image for feature display
*/
class vpProjectionDisplay
{
private:
  vpImage<unsigned char> Icam ;
  vpImage<unsigned char> Iext ;

  vpDisplayX dIcam ;
  vpDisplayX dIext ;
public:
  void init() ;
  void init(int select) ;
  void close() ;
  static int internalView() { return 0x01 ; }
  static int externalView() { return 0x02 ; }

  vpProjectionDisplay() { init() ;}
  vpProjectionDisplay(int select) { init(select) ;}
private:
  vpList<vpForwardProjection *> listFp ;
public:
  void insert( vpForwardProjection &fp) ;

public:

  void display(const vpHomogeneousMatrix &cextMo,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const int color,
	       const int select) ;

  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cextMo,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const int color ) ;


private:
  vpPoint o ;
  vpPoint x ;
  vpPoint y ;
  vpPoint z ;

public:
  void displayCamera(vpImage<unsigned char> &I,
		     const vpHomogeneousMatrix &cextMo,
		     const vpHomogeneousMatrix &cMo,
		     const vpCameraParameters &cam) ;
} ;



#endif // HAVE_LIBX11
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
