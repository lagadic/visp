/****************************************************************************
 *
 * $Id: vpEulerVector.cpp,v 1.4 2007-04-27 16:40:15 fspindle Exp $
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
 * Euler angles parameterization for the rotation.
 * Euler(phi,theta,psi)= Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <math.h>
#include <visp/vpEulerVector.h>


/*!
  \file vpEulerVector.cpp

  \brief class that consider the case of the Euler angles  parameterization
  for the  rotation

  Euler(phi,theta,psi) = Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/

/*!
  \brief  affectation of two Euler vector matrix
*/
vpEulerVector &
vpEulerVector::operator=(const vpEulerVector &m)
{

  for (int i=0; i<3; i++)
  {
      r[i] = m.r[i] ;
  }
  return *this;
}


//! copy constructor
vpEulerVector::vpEulerVector(const vpEulerVector &m) : vpRotationVector()
{
  *this = m ;
}

//! initialize a Euler vector from a rotation matrix
vpEulerVector::vpEulerVector(const vpRotationMatrix& R)
{
    buildFrom(R) ;
}


//! initialize a Euler vector from a Theta U vector
vpEulerVector::vpEulerVector(const vpThetaUVector& tu)
{
    buildFrom(tu) ;
}

//! convert a rotation matrix into Euler vector
vpEulerVector
vpEulerVector::buildFrom(const vpRotationMatrix& R)
{
    double phi ;
    if ((fabs(R[1][2]) < 1e-6) &&(fabs(R[0][2]) < 1e-6))
	phi = 0 ;
    else
	phi = atan2(R[1][2],R[0][2]) ;
    double cphi = cos(phi) ;
    double sphi = sin(phi) ;

    double theta = atan2(cphi*R[0][2]+sphi*R[1][2],R[2][2]);

    double psi = atan2(-sphi*R[0][0]+cphi*R[1][0],-sphi*R[0][1]+cphi*R[1][1]) ;

    r[0] = phi ;
    r[1] = theta ;
    r[2] = psi ;

    return *this ;
}


//! convert a rotation matrix into Theta U vector
vpEulerVector
vpEulerVector::buildFrom(const vpThetaUVector& tu)
{

    vpRotationMatrix R ;
    R.buildFrom(tu) ;
    buildFrom(R) ;

    return *this ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

