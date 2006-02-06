/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureBuilderVanishingPoint.cpp
 * Project:   ViSP 2.0
 * Author:   Odile Bourquardez
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureBuilderVanishingPoint.cpp,v 1.1 2006-02-06 14:24:25 obourqua Exp $
 *
 * Description
 * ============
 *
 *
 *
 * ++++++++++++
 */
/*!
  \file vpFeatureBuilderPoint.cpp
  \brief  conversion between vpPoint
  and visual feature vanishing Point
*/
#include<visp/vpFeatureBuilder.h>
#include<visp/vpFeatureException.h>
#include<visp/vpException.h>


/*!
  create vpFeatureVanishingPoint feature from the 2D coordinates of a point
in the image plane
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
    ERROR_TRACE("Cannot create vanishing point feature") ;
    throw ;
  }
}


/*!
  create vpFeatureVanishingPoint feature from 2 FeatureLine, ie lines in the
  image plane (error if the 2 lines are parallel)
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
    CERROR<<"there is no vanishing point : the lines are parallel in the image plane"<<endl;
    throw(" ");
  }

  y = (rho_r *c_l - rho_l * c_r) / (-s_l * c_r + s_r * c_l );
  x = (rho_r *s_l - rho_l * s_r) / (-c_l * s_r + c_r * s_l );

  s.set_x ( x );
  s.set_y ( y );
}

/*!
  create vpFeatureVanishingPoint feature from 2 Lines, (error if the 2 lines
  are parallel in the image plane)
*/
void
vpFeatureBuilder::create(vpFeatureVanishingPoint &s, const vpLine &L1, const vpLine &L2 )
{
  vpFeatureLine l1,l2 ;
  vpFeatureBuilder::create (l1,L1) ;
  vpFeatureBuilder::create (l2,L2) ;

  vpFeatureBuilder::create (s   , l1  ,l2)  ;

}
