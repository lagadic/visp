
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpThetaUVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpThetaUVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpThetaUVector.cpp,v 1.2 2005-07-19 13:03:39 obourqua Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Theta U parameterization for the
 *   rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpThetaUVector.cpp
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

#include <vpThetaUVector.h>

/*!
  \class vpThetaUVector
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

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
vpThetaUVector::vpThetaUVector(const vpThetaUVector &m)
{

    *this = m ;
}

//! initialize a Theta U vector from a rotation matrix
vpThetaUVector::vpThetaUVector(const vpRotationMatrix& R)
{

    buildFrom(R) ;
}

#define MINIMUM 0.0001
//! convert a rotation matrix into Theta U vector
vpThetaUVector
vpThetaUVector::buildFrom(const vpRotationMatrix& R)
{
    short i;
    double s,c,ang;

    s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1])
	+ (R[2][0]-R[0][2])*(R[2][0]-R[0][2])
	+ (R[2][1]-R[1][2])*(R[2][1]-R[1][2]);
    s = sqrt(s)/2.0;
    c = (R[0][0]+R[1][1]+R[2][2]-1)/2.0;
    ang=atan2(s,c);
    if (ang > MINIMUM)
    {
	if (s > MINIMUM)
	{
	    r[0] = (R[2][1]-R[1][2])/(2*s);
	    r[1] = (R[0][2]-R[2][0])/(2*s);
	    r[2] = (R[1][0]-R[0][1])/(2*s);
	}
	else
	{
	    r[0] = (sqrt((R[0][0]-c)/(1-c)));
	    if ((R[2][1]-R[1][2]) < 0) r[0] = -r[0];
	    r[1] = (sqrt((R[1][1]-c)/(1-c)));
	    if ((R[0][2]-R[2][0]) < 0) r[1] = -r[1];
	    r[2] = (sqrt((R[2][2]-c)/(1-c)));
	    if ((R[1][0]-R[0][1]) < 0) r[2] = -r[2];
	}
	for (i=0;i<3;i++) r[i] = r[i]*ang;
    }
    else
    {
	r[0] =   r[1] =   r[2] = 0.0;
    }

    return *this ;
}
#undef MINIMUM

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

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
