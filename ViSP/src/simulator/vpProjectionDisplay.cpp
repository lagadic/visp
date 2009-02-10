/****************************************************************************
 *
 * $Id: vpProjectionDisplay.cpp,v 1.12 2008-02-01 15:11:39 fspindle Exp $
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


/*!
  \file vpProjectionDisplay.cpp
  \brief interface with the image for feature display
*/

#include <visp/vpConfig.h>
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPoint.h>
#include <visp/vpMath.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>

#include <visp/vpDisplay.h>

#include <visp/vpProjectionDisplay.h>

#include <visp/vpBasicFeature.h>


void
vpProjectionDisplay::insert( vpForwardProjection &fp)
{
  // vpForwardProjection *f ;
  //  f = fp.duplicate() ;
  //  f->setDeallocate(vpForwardProjection::vpDisplayForwardProjection) ;

  listFp += &fp ;
}

void
vpProjectionDisplay::init()
{
  o.setWorldCoordinates(0,0,0) ;
  x.setWorldCoordinates(0.1,0,0) ;
  y.setWorldCoordinates(0,0.1,0) ;
  z.setWorldCoordinates(0,0,0.1) ;
}
void
vpProjectionDisplay::init(const int select)
{
  if (select & vpProjectionDisplay::internalView())
  {
    Icam.resize(256,256) ;
    dIcam.init(Icam,100,100) ;
  }
  if (select & vpProjectionDisplay::externalView())
  {
    Iext.resize(256,256) ;
    dIext.init(Iext,400,100) ;
  }

  init() ;
}


void
vpProjectionDisplay::close()
{

}
/*

void
vpProjectionDisplay::display(const vpHomogeneousMatrix &cextMo,
			     const vpHomogeneousMatrix &cMo,
			     const vpCameraParameters &cam,
			     const vpColor::vpColorType color,
			     const int select)
{
  if (select & vpProjectionDisplay::internalView())
    for (listFp.front() ; !listFp.outside() ; listFp.next() )
    {
      vpForwardProjection *fp = listFp.value() ;
      fp->display(Icam,cMo,cam,color) ;
    }

  if (select & vpProjectionDisplay::externalView())
    for (listFp.front() ; !listFp.outside() ; listFp.next() )
    {
      vpForwardProjection *fp = listFp.value() ;
      fp->display(Iext,cextMo,cam,color) ;
    }
}*/

void
vpProjectionDisplay::display(vpImage<unsigned char> &I,
			     const vpHomogeneousMatrix &cextMo,
			     const vpHomogeneousMatrix &cMo,
			     const vpCameraParameters &cam,
			     const vpColor::vpColorType color)
{

    for (listFp.front() ; !listFp.outside() ; listFp.next() )
    {
      vpForwardProjection *fp = listFp.value() ;
      fp->display(I,cextMo,cam, color) ;
    }

    displayCamera(I,cextMo,cMo, cam) ;
}


void
vpProjectionDisplay::displayCamera(vpImage<unsigned char> &I,
				   const vpHomogeneousMatrix &cextMo,
				   const vpHomogeneousMatrix &cMo,
				   const vpCameraParameters &cam)
{
  vpHomogeneousMatrix c1Mc ;
  c1Mc = cextMo*cMo.inverse() ;

  o.track(c1Mc) ;
  x.track(c1Mc) ;
  y.track(c1Mc) ;
  z.track(c1Mc) ;

  double ox=0, oy=0, x1=0, y1=0;

  vpMeterPixelConversion::convertPoint(cam,o.p[0],o.p[1],ox,oy) ;
  //o.print() ;
  //  vpTRACE("%f %f",ox,oy) ;

  vpMeterPixelConversion::convertPoint(cam,x.p[0],x.p[1],x1,y1) ;
  vpDisplay::displayArrow(I,
			  vpMath::round(oy), vpMath::round(ox),
			  vpMath::round(y1), vpMath::round(x1),
			  vpColor::green) ;

  vpMeterPixelConversion::convertPoint(cam,y.p[0],y.p[1],x1,y1) ;
  vpDisplay::displayArrow(I,
			  vpMath::round(oy), vpMath::round(ox),
			  vpMath::round(y1), vpMath::round(x1),
			  vpColor::blue) ;

  vpMeterPixelConversion::convertPoint(cam,z.p[0],z.p[1],x1,y1) ;
  vpDisplay::displayArrow(I,
			  vpMath::round(oy), vpMath::round(ox),
			  vpMath::round(y1), vpMath::round(x1),
			  vpColor::red) ;


}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
