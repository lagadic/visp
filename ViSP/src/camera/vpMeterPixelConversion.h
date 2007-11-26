/****************************************************************************
 *
 * $Id: vpMeterPixelConversion.h,v 1.5 2007-11-26 10:34:39 fspindle Exp $
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
 * Meter to pixel conversion.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/


#ifndef vpMeterPixelConversion_H
#define vpMeterPixelConversion_H


/*!
  \file vpMeterPixelConversion.h
  \brief meter to pixel  conversion

*/

#include <visp/vpConfig.h>
#include <visp/vpCameraParameters.h>
#include<visp/vpException.h>
#include<visp/vpMath.h>

/*!
  \class vpMeterPixelConversion
  \brief meter to pixel  conversion

*/
class VISP_EXPORT vpMeterPixelConversion
{
public:

/*!
  \brief point coordinates conversion (x,y)->(u,v)
  \param cam : camera parameters.
  \param x,y : input coordinates in meter.
  \param u,v : output coordinates in pixels.
  \param usedistortion : set as true if it is needed to use camera parameters
  with distortion.

  \f$ u = x*p_x*(1+k_d*r^2)+u_0 \f$ and  \f$ v = y*p_y*(1+k_d*r^2)+v_0  \f$
      with \f$ r^2 = x^2+y^2 \f$
*/

  inline static void
  convertPoint(const vpCameraParameters &cam,
              const double &x, const double &y,
              double &u, double &v, bool usedistortion = false)

  {
    if(usedistortion == false)
      convertPointWithoutDistortion(cam,x,y,u,v);
    else
      convertPointWithDistortion(cam,x,y,u,v);
  }

  /*!
    \brief point coordinates conversion without distortion (x,y)->(u,v)

    \f$ u = x*p_x+u_0 \f$ and  \f$ v = y*p_y+v_0  \f$
  */

  inline static void
  convertPointWithoutDistortion(const vpCameraParameters &cam,
              const double &x, const double &y,
              double &u, double &v)

  {
      u = x * cam.get_px() + cam.get_u0() ;
      v = y * cam.get_py() + cam.get_v0() ;
  }
  /*!
    \brief point coordinates conversion with distortion (x,y)->(u,v)
    \param cam : camera parameters.
    \param x,y : input coordinates in meter.
    \param u,v : output coordinates in pixels.

    \f$ u = x*p_x*(1+k_d*r^2)+u_0 \f$ and  \f$ v = y*p_y*(1+k_d*r^2)+v_0  \f$
        with \f$ r^2 = x^2+y^2 \f$
  */
  inline static void
  convertPointWithDistortion(const vpCameraParameters &cam,
              const double &x, const double &y,
              double &u, double &v)

  {
      u = x * cam.get_px_mp()*(1+cam.get_kd_mp()*(vpMath::sqr(x)+vpMath::sqr(y)))
          + cam.get_u0_mp() ;
      v = y * cam.get_py_mp()*(1+cam.get_kd_mp()*(vpMath::sqr(x)+vpMath::sqr(y)))
          + cam.get_v0_mp() ;
  }
  //! line coordinates conversion (rho,theta)
  static void convertLine(const vpCameraParameters &cam,
			  const double &rho_m, const double &theta_m,
			  double &rho_p, double &theta_p) ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

