/****************************************************************************
 *
 * $Id: vpPixelMeterConversion.h,v 1.4 2007-11-19 15:40:58 asaunier Exp $
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
  \param cam : camera parameters.
  \param u,v : input coordinates in pixels.
  \param x,y : output coordinates in meter.
  \param usedistortion : set as true if it is needed to use camera parameters
    with distortion.

  \f$ x = (u-u_0)*(1+k_d*r^2)/p_x \f$ and  \f$ y = (v-v_0)*(1+k_d*r^2)/p_y  \f$
      with \f$ r^2=((u - u_0)/p_x)^2+((v-v_0)/p_y)^2 \f$
*/
inline static void
convertPoint(const vpCameraParameters &cam,
  const double &u, const double &v,
  double &x, double &y, bool usedistortion = false)
{
  if(usedistortion == false)
    convertPointWithoutDistortion(cam,u,v,x,y);
  else
    convertPointWithDistortion(cam,u,v,x,y);
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
  if (fabs(cam.get_px())<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  if (fabs(cam.get_py())<1e-6)
  {
    vpERROR_TRACE("Camera parameter py = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double px = cam.get_px();
  double py = cam.get_py();
    x = (u - u0)/px ;
    y = (v - v0)/py ;
}

/*!
  \brief point coordinates conversion with distortion (u,v)->(x,y)
  \param cam : camera parameters.
  \param u,v : input coordinates in pixels.
  \param x,y : output coordinates in meter.

  \f$ x = (u-u_0)*(1+k_d*r^2)/p_x \f$ and  \f$ y = (v-v_0)*(1+k_d*r^2)/p_y  \f$
      with \f$ r^2=((u - u_0)/p_x)^2+((v-v_0)/p_y)^2 \f$
*/
inline static void
convertPointWithDistortion(
  const vpCameraParameters &cam,
  const double &u, const double &v,
  double &x, double &y)
{

  if (fabs(cam.get_px_pm())<1e-6)
  {
    vpERROR_TRACE("Camera parameter px = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  if (fabs(cam.get_py_pm())<1e-6)
  {
    vpERROR_TRACE("Camera parameter py = 0") ;
    throw(vpException(vpException::divideByZeroError,
          "Camera parameter px = 0")) ;
  }
  double u0 = cam.get_u0_pm();
  double v0 = cam.get_v0_pm();
  double px = cam.get_px_pm();
  double py = cam.get_py_pm();
  double kd = cam.get_kd_pm();
  double r2 = vpMath::sqr((u - u0)/px) + vpMath::sqr((v-v0)/py);
  x = (u - u0)*(1+kd*r2)/px ;
  y = (v - v0)*(1+kd*r2)/py ;
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

