/****************************************************************************
 *
 * $Id$
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
 * Defines a 2D triangle.
 *
 * Author:
 * Amaury Dame
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp/vpTriangle.h>
#include <visp/vpDebug.h>

/*!
  Basic constructor.
  
  By default, the three 2D points coordinates which define the triangle are \f$ (0,0) \f$, \f$ (1,0) \f$ and \f$ (0,1) \f$.
*/
vpTriangle::vpTriangle()
{
  goodTriange = true;
  init (vpImagePoint(0,0),vpImagePoint(1,0),vpImagePoint(0,1));
}

/*!
  Constructor which initialise the triangle thanks to the three 2D points \f$ iP1 \f$, \f$ iP2 \f$ and \f$ iP3 \f$
  
  \param iP1 : The first apex of the triangle.
  \param iP2 : The first apex of the triangle.
  \param iP3 : The first apex of the triangle.
*/
vpTriangle::vpTriangle(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3)
{
  goodTriange = true;
  
  init(iP1,iP2,iP3);
}

/*!
  Copy constructor
  
  \param tri : The triangle used for the initialisation.
*/
vpTriangle::vpTriangle(const vpTriangle &tri)
{
  goodTriange = tri.goodTriange;
  S1 = tri.S1;
  uvinv00 = tri.uvinv00;
  uvinv01 = tri.uvinv01;
  uvinv10 = tri.uvinv10;
  uvinv11 = tri.uvinv11;
  ptempo0 = tri.ptempo0;
  ptempo1 = tri.ptempo1;
}

/*!
  Basic destructor
*/
vpTriangle::~vpTriangle()
{
}

/*!
  Initialise the triangle thanks to the three 2D points \f$ iP1 \f$, \f$ iP2 \f$ and \f$ iP3 \f$
  
  \param iP1 : The first apex of the triangle.
  \param iP2 : The first apex of the triangle.
  \param iP3 : The first apex of the triangle.
*/
void
vpTriangle::buildFrom(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3)
{
  init(iP1,iP2,iP3);
}


void
vpTriangle::init(const vpImagePoint &iP1, const vpImagePoint &iP2, const vpImagePoint &iP3)
{
  apex1 = iP1;
  apex2 = iP2;
  apex3 = iP3;
  
  vpMatrix uv(2,2);
  vpMatrix uvinv(2,2);

  uv[0][0] = iP2.get_i() - iP1.get_i();
  uv[1][0] = iP3.get_i() - iP1.get_i();
  uv[0][1] = iP2.get_j() - iP1.get_j();
  uv[1][1] = iP3.get_j() - iP1.get_j();
  try
  {
    uvinv=uv.inverseByLU();
    goodTriange = true;
  }
  catch(...)
  {
    goodTriange = false;
    std::cout<<"Empty triangle"<<std::endl;
  }
  
  uvinv00=uvinv[0][0];
  uvinv01=uvinv[0][1];
  uvinv10=uvinv[1][0];
  uvinv11=uvinv[1][1];
  S1 = iP1;
}


/*!
  Check if the 2D point \f$ iP \f$ is inside the triangle.
  
  \param iP : The point which coulb be inside the triangle.
  \param threshold : A threshold used to define the accuracy of the computation when the point is very near from the edges of the triangle. 0 is the smallest value.
  
  \return Returns true if the point is inside the triangle. Returns false otherwise.
*/
bool
vpTriangle::inTriangle(const vpImagePoint &iP, double threshold)
{
  if(!goodTriange)
    return false;
  
  if(threshold < 0)
    threshold = 0;
  
  ptempo0 = iP.get_i() - S1.get_i();
  ptempo1 = iP.get_j() - S1.get_j();
  
  double p_ds_uv0=ptempo0*uvinv00+ptempo1*uvinv10;
  double p_ds_uv1=ptempo0*uvinv01+ptempo1*uvinv11;
  
  return (p_ds_uv0+p_ds_uv1<1.+threshold && p_ds_uv0>-threshold && p_ds_uv1>-threshold);
}

