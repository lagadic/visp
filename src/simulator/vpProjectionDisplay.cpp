
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpProjectionDisplay.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpProjectionDisplay.cpp,v 1.1.1.1 2005-06-08 07:08:13 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpProjectionDisplay.cpp
  \brief interface with the image for feature display
*/


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
			     const int color,
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
			     const int color )
{

    for (listFp.front() ; !listFp.outside() ; listFp.next() )
    {
      vpForwardProjection *fp = listFp.value() ;
      fp->display(I,cextMo,cam,color) ;
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

  double ox,oy, x1,y1 ;

  vpMeterPixelConversion::convertPoint(cam,o.p[0],o.p[1],ox,oy) ;
  o.print() ;
  TRACE("%f %f",ox,oy) ;

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

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
