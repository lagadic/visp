/****************************************************************************
 *
 * $Id: vpFeatureBuilderPoint3D.cpp,v 1.5 2007-04-18 16:14:29 asaunier Exp $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Conversion between tracker and visual feature 3D Point.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpFeatureBuilderPoint3D.cpp
  \brief  conversion between tracker
  and visual feature 3D Point
*/
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureException.h>
#include <visp/vpException.h>


/*!

  Initialize a 3D point feature using the coordinates of the point
  \f$(X,Y,Z)\f$ in the camera frame. The values of \f$(X,Y,Z)\f$ are
  expressed in meters.

  \warning To be sure that the vpFeaturePoint is well initialized, you have to be sure that at least the point
  coordinates in the camera frame are computed and stored in the vpPoint.

  \param s : Visual feature to initialize.

  \param t : The vpPoint used to create the vpFeaturePoint.
*/
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
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
