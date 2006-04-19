
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMeterPixelConversion.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpMeterPixelConversion.h,v 1.2 2006-04-19 09:01:20 fspindle Exp $
 *
 * Description
 * ============
 *  meter to pixel  conversion
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef vpMeterPixelConversion_H
#define vpMeterPixelConversion_H


/*!
  \file vpMeterPixelConversion.h
  \brief meter to pixel  conversion

*/
#include <visp/vpCameraParameters.h>


/*!
  \class vpMeterPixelConversion
  \brief meter to pixel  conversion

*/
class vpMeterPixelConversion
{
public:

  //! point coordinates conversion (u,v)->(x,y)
  static void convertPoint(const vpCameraParameters &cam,
			   const double x, const double y,
			   double &u, double &v) ;
  //! line coordinates conversion (rho,theta)
  static void convertLine(const vpCameraParameters &cam,
			  const double rho_m, const double theta_m,
			  double &rho_p, double &theta_p) ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

