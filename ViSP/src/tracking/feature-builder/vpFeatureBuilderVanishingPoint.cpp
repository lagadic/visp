/****************************************************************************
 *
 * $Id: vpFeatureBuilderVanishingPoint.cpp,v 1.6 2007-04-20 14:22:22 asaunier Exp $
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
 * Conversion between tracker and visual feature vanishing point.
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/


/*!
  \file vpFeatureBuilderVanishingPoint.cpp
  \brief  conversion between vpPoint
  and visual feature vanishing point.
*/
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureException.h>
#include <visp/vpException.h>


/*!
  Initialize a vpFeatureVanishingPoint thanks to a vpPoint.
  The vpFeatureVanishingPoint is initialized thanks to the parameters of the point in the image plan.
  All the parameters are given in meter.

  \param s : Visual feature to initialize.

  \param t : The vpPoint used to create the vpFeatureVanishingPoint.
*/
void
vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpPoint &t)
{
  try
  {
    s.set_x( t.get_x()) ;
    s.set_y( t.get_y()) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Cannot create vanishing point feature") ;
    throw ;
  }
}


/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpFeatureLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the intersection point in the image plan.
  All the parameters are given in meter.

  \warning An exception is thrown if the two lines are parallels 

  \param s : Visual feature to initialize.

  \param L1 : The first vpFeatureLine.

  \param L2 : The second vpFeatureLine.
*/
void
vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpFeatureLine &L1, const vpFeatureLine &L2 )
{
  double rho_l;
  double rho_r;
  double theta_l;
  double theta_r;
  double c_l;
  double s_l;
  double c_r;
  double s_r;

  rho_l   = L1.getRho();
  rho_r   = L2.getRho();
  theta_l = L1.getTheta();
  theta_r = L2.getTheta();
  c_l = cos(theta_l);
  c_r = cos(theta_r);
  s_l = sin(theta_l);
  s_r = sin(theta_r);


  double x,y;

  double min = 0.0001;
  if(fabs(theta_r-theta_l)<min || fabs(fabs(theta_r-theta_l)-M_PI)<min \
     || fabs(fabs(theta_r-theta_l)-2*M_PI)<min)
  {
    vpCERROR<<"there is no vanishing point : the lines are parallel in the image plane"<<std::endl;
    throw(" ");
  }

  y = (rho_r *c_l - rho_l * c_r) / (-s_l * c_r + s_r * c_l );
  x = (rho_r *s_l - rho_l * s_r) / (-c_l * s_r + c_r * s_l );

  s.set_x ( x );
  s.set_y ( y );
}



/*!
  Initialize a vpFeatureVanishingPoint thanks to two vpLine.
  The vpFeatureVanishingPoint is initialized thanks to the coordinate of the intersection point in the image plan.
  All the parameters are given in meter.

  \warning An exception is thrown if the two lines are parallels 

  \param s : Visual feature to initialize.

  \param L1 : The first vpLine.

  \param L2 : The second vpLine.
*/
void
vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpLine &L1, const vpLine &L2 )
{
  vpFeatureLine l1,l2 ;
  vpFeatureBuilder::create (l1,L1) ;
  vpFeatureBuilder::create (l2,L2) ;

  vpFeatureBuilder::create (s   , l1  ,l2)  ;

}
