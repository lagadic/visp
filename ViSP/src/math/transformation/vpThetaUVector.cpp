
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
 *  $Id: vpThetaUVector.cpp,v 1.4 2005-09-02 14:35:17 fspindle Exp $
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

#include <visp/vpThetaUVector.h>

#define DEBUG_LEVEL1 0

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

#ifdef MINIMUM
#undef MINIMUM
#endif
#define MINIMUM 0.0001


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

    if ((s > MINIMUM) || (c > 0.0)) /* general case */
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


    if (DEBUG_LEVEL1)  // test new version wrt old version
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
      if (ang > MINIMUM)
      {
  	if (s > MINIMUM)
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
#undef MINIMUM
#undef ANG_MIN_SINC
#undef SINC

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
#undef DEBUG_LEVEL1
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
