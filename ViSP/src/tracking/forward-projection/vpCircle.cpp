

/*
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#    Contact:
#       Eric Marchand
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: marchand@irisa.fr
#    www  : http://www.irisa.fr/vista
#
#----------------------------------------------------------------------------
*/


#include <visp/vpCircle.h>

#include <visp/vpFeatureDisplay.h>

void
vpCircle::init()
{

  oP.resize(7) ;
  cP.resize(7) ;

  p.resize(5) ;
}

void
vpCircle::setWorldCoordinates(const vpColVector& _oP)
{
  oP = _oP ;
}

void
vpCircle::setWorldCoordinates(const double _A, const double _B,
			       const double _C,
			       const double _X0, const double _Y0,
			       const double _Z0,
			       const double _R)
{
  oP[0] = _A ;
  oP[1] = _B ;
  oP[2] = _C ;
  oP[3] = _X0 ;
  oP[4] = _Y0 ;
  oP[5] = _Z0 ;
  oP[6] = _R ;
}



vpCircle::vpCircle()
{
  init() ;
}


vpCircle::vpCircle(const vpColVector& _oP)
{
  init() ;
  setWorldCoordinates(_oP) ;
}

vpCircle::vpCircle(const double _A, const double _B,
		   const double _C,
		   const double _X0, const double _Y0,
		   const double _Z0,
		   const double _R)
{
  init() ;
  setWorldCoordinates(_A,  _B,   _C,
		      _X0, _Y0,  _Z0,
		      _R) ;
}

vpCircle::~vpCircle()
{
}



//! perspective projection of the circle
void
vpCircle::projection(const vpColVector &_cP, vpColVector &_p)
{

  vpColVector K(6) ;

  {
    double A = _cP[0] ;
    double B = _cP[1] ;
    double C = _cP[2] ;

    double X0 = _cP[3] ;
    double Y0 = _cP[4] ;
    double Z0 = _cP[5] ;

    double r =  _cP[6];

    // projection
    double s = X0*X0 + Y0*Y0 + Z0*Z0 - r*r ;
    double det = A*X0+B*Y0+C*Z0;
    A = A/det ;
    B = B/det ;
    C = C/det ;

    K[0] = 1 - 2*A*X0 + A*A*s;
    K[1] = 1 - 2*B*Y0 + B*B*s;
    K[2] = -A*Y0 - B*X0 + A*B*s;
    K[3] = -C*X0 - A*Z0 + A*C*s;
    K[4] = -C*Y0 - B*Z0 + B*C*s;
    K[5] = 1 - 2*C*Z0 + C*C*s;

  }
  double det  = K[2]*K[2] -K[0]*K[1];
  if (fabs(det) < 1e-8)
  {
    ERROR_TRACE("division par 0") ;
    throw(vpException(vpException::divideByZeroERR,
		      "division par 0")) ;

  }


  double xc = (K[1]*K[3]-K[2]*K[4])/det;
  double yc = (K[0]*K[4]-K[2]*K[3])/det;

  double c = sqrt( (K[0]-K[1])*(K[0]-K[1]) + 4*K[2]*K[2] );
  double s = 2*(K[0]*xc*xc + 2*K[2]*xc*yc + K[1]*yc*yc - K[5]);

  double A,B,E ;

  if (fabs(K[2])<1e-6)
  {
    E = 0.0;
    if (K[0] > K[1])
    {
      A = sqrt(s/(K[0] + K[1] + c));
      B = sqrt(s/(K[0] + K[1] - c));
    }
    else
    {
      A = sqrt(s/(K[0] + K[1] - c));
      B = sqrt(s/(K[0] + K[1] + c));
    }
  }
  else
  {
    E = (K[1] - K[0] + c)/(2*K[2]);
    if ( fabs(E) > 1.0)
    {
      A = sqrt(s/(K[0] + K[1] + c));
      B = sqrt(s/(K[0] + K[1] - c));
    }
    else
    {
      A = sqrt(s/(K[0] + K[1] - c));
      B = sqrt(s/(K[0] + K[1] + c));
      E = -1.0/E;
    }
  }

  det =  (1.0 + vpMath::sqr(E));
  double m20 = (vpMath::sqr(A) +  vpMath::sqr(B*E))  /det ;
  double m11 = (vpMath::sqr(A)  - vpMath::sqr(B)) *E / det ;
  double m02 = (vpMath::sqr(B) + vpMath::sqr(A*E))   / det ;

  _p[0] = xc ;
  _p[1] = yc ;
  _p[2] = m20 ;
  _p[3] = m11 ;
  _p[4] = m02 ;
}

//! perspective projection of the circle
void
vpCircle::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)
{

  double A,B,C ;
  A = cMo[0][0]*oP[0] + cMo[0][1]*oP[1]  + cMo[0][2]*oP[2];
  B = cMo[1][0]*oP[0] + cMo[1][1]*oP[1]  + cMo[1][2]*oP[2];
  C = cMo[2][0]*oP[0] + cMo[2][1]*oP[1]  + cMo[2][2]*oP[2];

  double X0,Y0,Z0 ;
  X0 = cMo[0][3] + cMo[0][0]*oP[3] + cMo[0][1]*oP[4] + cMo[0][2]*oP[5];
  Y0 = cMo[1][3] + cMo[1][0]*oP[3] + cMo[1][1]*oP[4] + cMo[1][2]*oP[5];
  Z0 = cMo[2][3] + cMo[2][0]*oP[3] + cMo[2][1]*oP[4] + cMo[2][2]*oP[5];
  double R = oP[6] ;

  _cP[0] = A ;
  _cP[1] = B ;
  _cP[2] = C ;

  _cP[3] = X0 ;
  _cP[4] = Y0 ;
  _cP[5] = Z0 ;

  _cP[6] = R ;

  // TRACE("_cP :") ; cout << _cP.t() ;

}

void vpCircle::display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const int color)
{
  vpFeatureDisplay::displayEllipse(p[0],p[1],p[2],p[3], p[4],
				   cam, I,color) ;
}

// non destructive wrt. cP and p
void vpCircle::display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const int color)
{
  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;
  projection(_cP,_p) ;
  vpFeatureDisplay::displayEllipse(_p[0],_p[1],_p[2],_p[3], _p[4],
				   cam, I, color) ;

}
//! for memory issue (used by the vpServo class only)
vpCircle *vpCircle::duplicate() const
{
  vpCircle *feature = new vpCircle(*this) ;
  return feature ;
}
