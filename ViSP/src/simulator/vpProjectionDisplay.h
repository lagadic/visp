/****************************************************************************
 *
 * $Id: vpProjectionDisplay.h,v 1.9 2007-05-11 16:53:35 fspindle Exp $
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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpProjectionDisplay_H
#define vpProjectionDisplay_H

/*!
  \file vpProjectionDisplay.h
  \brief interface with the image for feature display
*/

#include <visp/vpConfig.h>
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))

#include <visp/vpConfig.h>
// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpForwardProjection.h>
#include <visp/vpList.h>

/*!
  \class vpProjectionDisplay
  \brief interface with the image for feature display
*/
class VISP_EXPORT vpProjectionDisplay
{
private:
  vpImage<unsigned char> Icam ;
  vpImage<unsigned char> Iext ;

#if defined VISP_HAVE_X11
  vpDisplayX dIcam ;
  vpDisplayX dIext ;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK dIcam ;
  vpDisplayGTK dIext ;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI dIcam ;
  vpDisplayGDI dIext ;
#endif
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
	       const vpColor::vpColorType color,
	       const int select) ;

  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cextMo,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const vpColor::vpColorType color ) ;


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



#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
