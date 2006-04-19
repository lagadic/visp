
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPixelMeterConversion.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpPixelMeterConversion.h,v 1.2 2006-04-19 09:01:20 fspindle Exp $
 *
 * Description
 * ============
 * pixel to meter conversion
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef vpPixelMeterConversion_H
#define vpPixelMeterConversion_H


/*!
  \file vpPixelMeterConversion.h
  \brief pixel to meter conversion

*/
#include <visp/vpCameraParameters.h>


/*!
  \class vpPixelMeterConversion
  \brief pixel to meter conversion

*/
class vpPixelMeterConversion
{
public:
  //! point coordinates conversion (u,v)->(x,y)
  static void convertPoint(const vpCameraParameters &cam,
		      const double u, const double v,
		      double &x, double &y) ;
  //! line coordinates conversion (rho,theta)
  static void convertLine(const vpCameraParameters &cam,
		      const double rho_p, const double theta_p,
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

