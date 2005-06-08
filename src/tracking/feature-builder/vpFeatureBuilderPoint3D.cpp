

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureBuilderPoint3D.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureBuilderPoint3D.cpp,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     conversion between tracker and visual feature
 *     and visual feature 3D Point
 *
 * ++++++++++++
 */

/*!
  \file vpFeatureBuilderPoint3D.cpp
  \brief  conversion between tracker
  and visual feature 3D Point
*/
#include<visp/vpFeatureBuilder.h>
#include<visp/vpFeatureException.h>
#include<visp/vpException.h>


void
vpFeatureBuilder::create(vpFeaturePoint3D &s, const vpPoint &t )
{
  try
  {



    s.set_X( t.cP[0]/t.cP[3]) ;
    s.set_Y( t.cP[1]/t.cP[3])  ;
    s.set_Z( t.cP[2]/t.cP[3])  ;


  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
