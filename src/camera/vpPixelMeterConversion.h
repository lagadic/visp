/****************************************************************************
 *
 * $Id: vpPixelMeterConversion.h,v 1.5 2008-01-31 14:43:50 asaunier Exp $
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
 * Pixel to meter conversion.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/

#ifndef vpPixelMeterConversion_H
#define vpPixelMeterConversion_H


/*!
  \file vpPixelMeterConversion.h
  \brief pixel to meter conversion

*/
#include <visp/vpConfig.h>
#include <visp/vpCameraParameters.h>
#include<visp/vpException.h>
#include<visp/vpMath.h>
#include<visp/vpDebug.h>

/*!
  \class vpPixelMeterConversion
  \brief pixel to meter conversion

*/
class VISP_EXPORT vpPixelMeterConversion
{
public:

/*!
  \brief point coordinates conversion (u,v)->(x,y)
  The used formula depends on the projection model of the camera.

  \param cam : camera parameters.
  \param u,v : input coordinates in pixels.
  \param x,y : output coordinates in meter.

  \f$ x = (u-u_0)/p_x \f$ and \f$ y = (v-v_0)/p_y  \f$ in the case of
  perspective projection without distortion.

  \f$ x = (u-u_0)*(1+k_{du}*r^2)/p_x \f$ and
  \f$ y = (v-v_0)*(1+k_{du}*r^2)/p_y  \f$
  with \f$ r^2=((u - u_0)/p_x)^2+((v-v_0)/p_y)^2 \f$ in the case of perspective
  projection with distortion.
*/
inline static void
convertPoint(const vpCameraParameters &cam,
  const double &u, const double &v,
  double &x, double &y)
{
  switch(cam.projModel){
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      convertPointWithoutDistortion(cam,u,v,x,y);
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
      convertPointWithDistortion(cam,u,v,x,y);
      break;
  }       
}

/*!
  \brief point coordinates conversion without distortion (u,v)->(x,y)
  \param cam : camera parameters.
  \param u,v : input coordinates in pixels.
  \param x,y : output coordinates in meter.

  \f$ x = (u-u_0)/p_x \f$ and  \f$ y = (v-v_0)/p_y  \f$
*/
inline static void
convertPointWithoutDistortion(
  const vpCameraParameters &cam,
  const double &u, const double &v,
  double &x, double &y)
{
    x = (u - cam.u0)*cam.inv_px ;
    y = (v - cam.v0)*cam.inv_py ;
}

/*!
  \brief point coordinates conversion with distortion (u,v)->(x,y)
  \param cam : camera parameters.
  \param u,v : input coordinates in pixels.
  \param x,y : output coordinates in meter.

  \f$ x = (u-u_0)*(1+k_{du}*r^2)/p_x \f$ and
  \f$ y = (v-v_0)*(1+k_{du}*r^2)/p_y \f$
  with \f$ r^2=((u - u_0)/p_x)^2 + ((v-v_0)/p_y)^2 \f$
*/
inline static void
convertPointWithDistortion(
  const vpCameraParameters &cam,
  const double &u, const double &v,
  double &x, double &y)
{
  double r2 = 1.+cam.kdu*(vpMath::sqr((u - cam.u0)*cam.inv_px) +
              vpMath::sqr((v-cam.v0)*cam.inv_py));
  x = (u - cam.u0)*r2*cam.inv_px ;
  y = (v - cam.v0)*r2*cam.inv_py ;
}
  //! line coordinates conversion (rho,theta)
  static void convertLine(const vpCameraParameters &cam,
		      const double &rho_p, const double &theta_p,
		      double &rho_m, double &theta_m) ;


  static void convertMoment(const vpCameraParameters &cam,
			    int order,
			    const vpMatrix &moment_pixel,
			    vpMatrix &moment_meter) ;
} ;

#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

