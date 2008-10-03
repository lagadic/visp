/****************************************************************************
 *
 * $Id: vpThetaUVector.cpp,v 1.11 2008-10-03 15:50:16 fspindle Exp $
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
 * Theta U parameterization for the rotation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpThetaUVector.cpp
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

#include <visp/vpThetaUVector.h>

#define vpDEBUG_LEVEL1 0

const double vpThetaUVector::minimum = 0.0001;

/*!
  \brief  affectation of two Theta U vector
*/
vpThetaUVector &
vpThetaUVector::operator=(const vpThetaUVector &m)
{

  for (int i=0; i<3; i++)
  {
      r[i] = m.r[i] ;
  }
  return *this;
}


//! copy constructor
vpThetaUVector::vpThetaUVector(const vpThetaUVector &m) : vpRotationVector()
{
  *this = m ;
}

//! initialize a Theta U vector from a rotation matrix
vpThetaUVector::vpThetaUVector(const vpRotationMatrix& R)
{

    buildFrom(R) ;
}

//! initialize a Theta U vector from a RzyxVector
vpThetaUVector::vpThetaUVector(const vpRzyxVector& rzyx)
{
  buildFrom(rzyx) ;
} 
//! initialize a Theta U vector from a RzyzVector
vpThetaUVector::vpThetaUVector(const vpRzyzVector& rzyz)
{
  buildFrom(rzyz) ;
}
//! initialize a Theta U vector from a RxyzVector
vpThetaUVector::vpThetaUVector(const vpRxyzVector& rxyz)
{
  buildFrom(rxyz) ;
}

//! convert a rotation matrix into Theta U vector
vpThetaUVector
vpThetaUVector::buildFrom(const vpRotationMatrix& R)
{
    double s,c,theta,sinc;

    s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
	+ (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
	+ (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
    s = sqrt(s)/2.0;
    c = (R[0][0]+R[1][1]+R[2][2]-1.0)/2.0;
    theta=atan2(s,c);  /* theta in [0, PI] since s > 0 */

    if ((s > minimum) || (c > 0.0)) /* general case */
    {
      sinc = vpMath::sinc(s,theta);

      r[0] = (R[2][1]-R[1][2])/(2*sinc);
      r[1] = (R[0][2]-R[2][0])/(2*sinc);
      r[2] = (R[1][0]-R[0][1])/(2*sinc);
    }
    else /* theta near PI */
    {
       r[0] = theta*(sqrt((R[0][0]-c)/(1-c)));
       if ((R[2][1]-R[1][2]) < 0) r[0] = -r[0];
       r[1] = theta*(sqrt((R[1][1]-c)/(1-c)));
       if ((R[0][2]-R[2][0]) < 0) r[1] = -r[1];
       r[2] = theta*(sqrt((R[2][2]-c)/(1-c)));
       if ((R[1][0]-R[0][1]) < 0) r[2] = -r[2];
    }


    if (vpDEBUG_LEVEL1)  // test new version wrt old version
    {
      // old version
      int i;
      //    double s,c;
      double ang;
      double r2[3]; // has to be replaced by r below if good version

      s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
	+ (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
	+ (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
      s = sqrt(s)/2.0;
      c = (R[0][0]+R[1][1]+R[2][2]-1)/2.0;
      ang=atan2(s,c);
      if (ang > minimum)
      {
        if (s > minimum)
	      {
	          r2[0] = (R[2][1]-R[1][2])/(2*s);
	          r2[1] = (R[0][2]-R[2][0])/(2*s);
	          r2[2] = (R[1][0]-R[0][1])/(2*s);
	      }
	      else
	      {
	          r2[0] = (sqrt((R[0][0]-c)/(1-c)));
	          if ((R[2][1]-R[1][2]) < 0) r2[0] = -r2[0];
	          r2[1] = (sqrt((R[1][1]-c)/(1-c)));
	          if ((R[0][2]-R[2][0]) < 0) r2[1] = -r2[1];
	          r2[2] = (sqrt((R[2][2]-c)/(1-c)));
	          if ((R[1][0]-R[0][1]) < 0) r2[2] = -r2[2];
	      }
	      for (i=0;i<3;i++) r2[i] = r2[i]*ang;
      }
      else
      {
        r2[0] =   r2[1] =   r2[2] = 0.0;
      }
      // end old version
      // verification of the new version
      int pb = 0;

      for (i=0;i<3;i++)
      {
        if (fabs(r[i] - r2[i]) > 1e-5) pb = 1;
      }
      if (pb == 1)
      {
	      printf("vpThetaUVector::buildFrom(const vpRotationMatrix& R)\n");
	      printf(" r      : %lf %lf %lf\n",r[0],r[1],r[2]);
        printf(" r2     : %lf %lf %lf\n",r2[0],r2[1],r2[2]);
        printf(" r - r2 : %lf %lf %lf\n",r[0]-r2[0],r[1]-r2[1],r[2]-r2[2]);
      }
    // end of the verification
    }

    return *this ;
}

vpThetaUVector
vpThetaUVector::buildFrom(const vpRzyxVector& e)
{
    vpRotationMatrix R(e) ;

    buildFrom(R) ;
    return *this ;
}

vpThetaUVector
vpThetaUVector::buildFrom(const vpEulerVector& e)
{
    vpRotationMatrix R(e) ;

    buildFrom(R) ;
    return *this ;
}


vpThetaUVector
vpThetaUVector::buildFrom(const vpRzyzVector& e)
{
    vpRotationMatrix R(e) ;

    buildFrom(R) ;
    return *this ;
}

vpThetaUVector
vpThetaUVector::buildFrom(const vpRxyzVector& e)
{
    vpRotationMatrix R(e) ;

    buildFrom(R) ;
    return *this ;
}


/*!

  Extract the rotation angle \f$ \theta \f$ and the unit vector
  \f$ u \f$ from the \f$ \theta u \f$ representation.

  \param theta : Rotation angle \f$ \theta \f$.

  \param u : Unit vector \f$ u = (u_{x},u_{y},u_{z})^{\top} \f$
  representing the rotation axis.

*/
void 
vpThetaUVector::extract(double &theta, vpColVector &u) const
{
  u.resize(3);

  theta = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);

  for (int i=0 ; i < 3 ; i++) 
    u[i] = r[i] / theta ;
}


#undef vpDEBUG_LEVEL1
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
