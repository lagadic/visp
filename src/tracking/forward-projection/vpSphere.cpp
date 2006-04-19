

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


#include <visp/vpSphere.h>
#include <visp/vpFeatureDisplay.h>


void
vpSphere::init()
{

  oP.resize(4) ;
  cP.resize(4) ;

  p.resize(5) ;
}

void
vpSphere::setWorldCoordinates(const vpColVector& oP)
{
  this->oP = oP ;
}

void
vpSphere::setWorldCoordinates(const double X0, const double Y0,
			      const double Z0, const double R)
{
  oP[0] = X0 ;
  oP[1] = Y0 ;
  oP[2] = Z0 ;
  oP[3] = R ;
}



vpSphere::vpSphere()
{
  init() ;
}


vpSphere::vpSphere(const vpColVector& oP)
{
  init() ;
  setWorldCoordinates(oP) ;
}

vpSphere::vpSphere(const double X0, const double Y0,
		   const double Z0, const double R)
{
  init() ;
  setWorldCoordinates(X0, Y0,  Z0,  R) ;
}

vpSphere::~vpSphere()
{
}



//! perspective projection of the circle
void
vpSphere::projection(const vpColVector &cP, vpColVector &p)
{
  double x0, y0, z0;  //variables intermediaires
  double k0, k1, k2, k3, k4;  //variables intermediaires
  double E, A, B; //variables intermediaires

  //calcul des parametres M20, M11, M02 de l'ellipse
  double s, a, b, r, e;  //variables intermediaires
  r =  cP[3];

  x0 = cP[0] ;
  y0 = cP[1] ;
  z0 = cP[2] ;


  s = r*r - y0*y0 -z0*z0;

  k0 = (r*r - x0*x0 -z0*z0)/s;
  k1 = (x0*y0)/s;
  k2 = (x0*z0)/s;
  k3 = (y0*z0)/s;
  k4 = (r*r - x0*x0 -y0*y0)/s;

  if ((s = z0*z0 - r*r) < 0.0)
  {
    ERROR_TRACE("sphere derriere le plan image\n");
  }

  p[0] =  x0*z0/s ;  //x
  p[1] =  y0*z0/s ;  //y

  if (fabs(x0)  > 1e-6)
  {
    //   ERROR_TRACE(" %f",r) ;
    e = y0/x0;
    b = r/sqrt(s);
    if ((a = x0*x0 + y0*y0 + z0*z0 - r*r) < 0.0)
    {
      ERROR_TRACE("sphere derriere le plan image\n");
    }
    a = r*sqrt(a)/s;
    if (fabs(e) <= 1.0)
    {
      E =  e;
      A = a;
      B = b;
    }
    else
    {
      E = -1.0/e;
      A = b;
      B = a;
    }
  }
  else
  {
    //   ERROR_TRACE(" %f",r) ;
    E = 0.0;
    A = r/sqrt(s);
    B = r*sqrt(y0*y0+z0*z0-r*r)/s;
  }

  p[2] = ( A*A + B*B * E*E) / (1.0 + E*E);  // mu20
  p[3] = ( A*A - B*B) * E / (1.0 + E*E);    // mu11
  p[4] = ( B*B + A*A * E*E) / (1.0 + E*E);  // mu02

  // ERROR_TRACE(" %f",r) ;

  //  cout << p.t() ;

}

//! perspective projection of the circle
void
vpSphere::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP)
{

  double x0, y0, z0;  //variables intermediaires


  x0 = cMo[0][0]*oP[0] + cMo[0][1]*oP[1]  + cMo[0][2]*oP[2] + cMo[0][3];
  y0 = cMo[1][0]*oP[0] + cMo[1][1]*oP[1]  + cMo[1][2]*oP[2] + cMo[1][3];
  z0 = cMo[2][0]*oP[0] + cMo[2][1]*oP[1]  + cMo[2][2]*oP[2] + cMo[2][3];

  cP[3] = oP[3];

  cP[0] = x0 ;
  cP[1] = y0 ;
  cP[2] = z0 ;


}

//! for memory issue (used by the vpServo class only)
vpSphere *vpSphere::duplicate() const
{
  vpSphere *feature = new vpSphere(*this) ;
  return feature ;
}



// non destructive wrt. cP and p
void vpSphere::display(vpImage<unsigned char> &I,
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



void vpSphere::display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const int color)
{
  vpFeatureDisplay::displayEllipse(p[0],p[1],p[2],p[3], p[4],
				   cam, I,color) ;
}
