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

  listFp.push_back(&fp);
}

void
vpProjectionDisplay::init()
{
  o.setWorldCoordinates(0,0,0) ;
  x.setWorldCoordinates(0.1,0,0) ;
  y.setWorldCoordinates(0,0.1,0) ;
  z.setWorldCoordinates(0,0,0.1) ;
  traj.resize(0,2);
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

void
vpProjectionDisplay::display(vpImage<unsigned char> &I,
			     const vpHomogeneousMatrix &cextMo,
			     const vpHomogeneousMatrix &cMo,
			     const vpCameraParameters &cam,
			     const vpColor color,
			     const bool &displayTraj)
{

  for (std::list<vpForwardProjection *>::const_iterator it = listFp.begin() ; it != listFp.end(); ++it )
    {
      vpForwardProjection *fp = *it ;
      fp->display(I,cextMo,cam, color) ;
    }

    if(displayTraj)	// display past camera positions
    	for(unsigned int i=0;i<traj.getRows();++i)
    		vpDisplay::displayCircle(I,(int)traj[i][0],(int)traj[i][1],2,vpColor::green,true);

    displayCamera(I,cextMo,cMo, cam);

    if(displayTraj)	// store current camera position
    {
      const unsigned int n = traj.getRows();
    	traj.resize(n+1,2,false);
    	vpMeterPixelConversion::convertPoint(cam,o.p[0],o.p[1],traj[n][1],traj[n][0]);
    }
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

  if(o.get_Z() < 0)	// do not print camera if behind the external camera
	  return;

  x.track(c1Mc) ;
  y.track(c1Mc) ;
  z.track(c1Mc) ;

  vpImagePoint ipo;
  vpImagePoint ipx;

  vpMeterPixelConversion::convertPoint(cam, o.p[0], o.p[1], ipo) ;

  vpMeterPixelConversion::convertPoint(cam, x.p[0], x.p[1], ipx) ;
  vpDisplay::displayArrow(I, ipo, ipx, vpColor::green) ;

  vpMeterPixelConversion::convertPoint(cam, y.p[0], y.p[1], ipx) ;
  vpDisplay::displayArrow(I, ipo, ipx, vpColor::blue) ;

  vpMeterPixelConversion::convertPoint(cam, z.p[0], z.p[1], ipx) ;
  vpDisplay::displayArrow(I, ipo, ipx, vpColor::red) ;
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
