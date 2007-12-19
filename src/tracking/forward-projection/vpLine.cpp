/****************************************************************************
 *
 * $Id: vpLine.cpp,v 1.7 2007-12-19 17:36:29 fspindle Exp $
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
 * Line feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpLine.h>

#include <visp/vpDebug.h>
#include <visp/vpMath.h>

#include <visp/vpFeatureDisplay.h>

/*!
  \file vpLine.cpp
  \brief   class that defines what is a line
*/



void
vpLine::init()
{

  oP.resize(8) ;
  cP.resize(8) ;
  p.resize(2) ;


}

vpLine::vpLine()
{
  init() ;
}



//! set the line world coordinates
void
vpLine::setWorldCoordinates(const double &A1, const double &B1,
			    const double &C1, const double &D1,
			    const double &A2, const double &B2,
			    const double &C2, const double &D2)
{
  oP[0] = A1 ;
  oP[1] = B1 ;
  oP[2] = C1 ;
  oP[3] = D1 ;

  oP[4] = A2 ;
  oP[5] = B2 ;
  oP[6] = C2 ;
  oP[7] = D2 ;
}


//! set the line world coordinates
void
vpLine::setWorldCoordinates(const vpColVector &_oP)
{
  oP = _oP ;
}


//! set the line world coordinates from two planes
void
vpLine::setWorldCoordinates(const vpColVector &_oP1,
			    const vpColVector &_oP2)
{
  for (int i=0 ; i < 4 ; i++)
  {
    oP[i]   = _oP1[i] ;
    oP[i+4] = _oP2[i] ;
  }

}



//! perspective projection of the line
void
vpLine::projection(const vpColVector &_cP, vpColVector &_p)
{
 //projection

  double A1, A2, B1, B2, C1, C2, D1, D2;

  A1=_cP[0] ;
  B1=_cP[1] ;
  C1=_cP[2] ;
  D1=_cP[3] ;

  A2=_cP[4] ;
  B2=_cP[5] ;
  C2=_cP[6] ;
  D2=_cP[7] ;

  double a, b, c, s;
  a = A1*D2 - A2*D1;
  b = B1*D2 - B2*D1;
  c = C1*D2 - C2*D1;
  s = sqrt( a*a+b*b );

  double rho = -c/s ;
  double theta = atan2( b, a);

  while (theta > M_PI/2) { theta -= M_PI ; rho *= -1 ; }
  while (theta < -M_PI/2) { theta += M_PI ; rho *= -1 ; }

  _p[0] = rho ;
  _p[1] = theta ;
}

void
vpLine::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)
{

  double a1, a2, b1, b2, c1, c2, d1, d2;
  double A1, A2, B1, B2, C1, C2, D1, D2;

  a1=oP[0] ;
  b1=oP[1] ;
  c1=oP[2] ;
  d1=oP[3] ;

  a2=oP[4] ;
  b2=oP[5] ;
  c2=oP[6] ;
  d2=oP[7] ;

  A1 = cMo[0][0]*a1 + cMo[0][1]*b1  + cMo[0][2]*c1;
  B1 = cMo[1][0]*a1 + cMo[1][1]*b1  + cMo[1][2]*c1;
  C1 = cMo[2][0]*a1 + cMo[2][1]*b1  + cMo[2][2]*c1;
  D1 = d1 - (cMo[0][3]*A1 + cMo[1][3]*B1  + cMo[2][3]*C1);

  A2 = cMo[0][0]*a2 + cMo[0][1]*b2  + cMo[0][2]*c2;
  B2 = cMo[1][0]*a2 + cMo[1][1]*b2  + cMo[1][2]*c2;
  C2 = cMo[2][0]*a2 + cMo[2][1]*b2  + cMo[2][2]*c2;
  D2 = d2 - (cMo[0][3]*A2 + cMo[1][3]*B2  + cMo[2][3]*C2);


  if (fabs(D2) < 1e-8)
  {
    //swap the two plane
    vpMath::swap(A1,A2) ;
    vpMath::swap(B1,B2) ;
    vpMath::swap(C1,C2) ;
    vpMath::swap(D1,D2) ;
  }

  //  vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
  //  vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;


  if ((fabs(D1) > 1e-8) || (fabs(D2) > 1e-8))
  {
    // Rajout des quatre contraintes sur la droite

    // Calcul du plan P1 passant par l'origine avec les contraintes
    // Contrainte d1 = 0
    double alpha1 ;
    double beta1  ;
    double gamma1 ;

    {
      alpha1 = D2*A1 - D1*A2 ;
      beta1  = D2*B1 - D1*B2 ;
      gamma1 = D2*C1 - D1*C2 ;
    }

    // Contrainte a1^2 + b1^2 + c1^2 = 1
    double s1 = sqrt (alpha1*alpha1 + beta1*beta1 + gamma1*gamma1);
    A1 =  alpha1/s1 ;
    B1 =  beta1/s1 ;
    C1 =  gamma1/s1 ;
    D1 = 0 ;

    //   vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;

    //std::cout <<"--> "<< A1 << "  " << B1 << "  " << C1<< "  " << D1 <<std::endl ;

    // ajout de la contrainte a1 a2 + b1 b2 + c1 c2 = 0
    double x1,y1 ;
    if (fabs(A1) > 0.01)
    {

      //    vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
      //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;
      x1 = A1*B2 - B1*A2;
      y1 = A1*C2 - C1*A2;
      A2 = -(B1*x1+C1*y1);
      B2= ((A1*A1+C1*C1)*x1-B1*C1*y1)/A1;
      C2 = (-B1*C1*x1+ (A1*A1+B1*B1)*y1)/A1;
    }
    else if (fabs(B1) > 0.01){

      //    vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
      //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

      x1 = A1*B2 - B1*A2;
      y1 = C1*B2 - B1*C2;
      A2 = -((B1*B1+C1*C1)*x1-A1*C1*y1)/B1;
      B2= A1*x1+C1*y1;
      C2 = -(-A1*C1*x1+(A1*A1+B1*B1)*y1)/B1;
    }
    else {

      //    vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
      //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

      x1 = A1*C2 - C1*A2;
      y1 = B1*C2 - C1*B2;
      A2= (-(B1*B1+C1*C1)*x1+A1*B1*y1)/C1;
      B2 = (A1*B1*x1-(A1*A1+C1*C1)*y1)/C1;
      C2 = A1*x1+B1*y1;
    }
    // Contrainte de normalisation
    //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

    double s2 = sqrt (A2*A2 +  B2*B2 + C2*C2);
    A2 = A2/s2;
    B2 = B2/s2;
    C2 = C2/s2;
    D2 = D2/s2;
  }
  else
  {
    // Cas degenere D1 = D2 = 0
  }
  _cP[0] =  A1;
  _cP[1] =  B1;
  _cP[2] =  C1;
  _cP[3] =  D1;


  _cP[4] =  A2;
  _cP[5] =  B2;
  _cP[6] =  C2;
  _cP[7] =  D2;

  if (D2 < 0)
  {
    _cP[4] *= -1 ;
    _cP[5] *= -1 ;
    _cP[6] *= -1 ;
    _cP[7] *= -1 ;
  }

}


void vpLine::display(vpImage<unsigned char> &I,
		     const vpCameraParameters &cam,
		     const bool usedistortion,
		     const vpColor::vpColorType color)
{
  vpFeatureDisplay::displayLine(p[0],p[1],
				cam, I, usedistortion, color) ;
}

// non destructive wrt. cP and p
void vpLine::display(vpImage<unsigned char> &I,
		     const vpHomogeneousMatrix &cMo,
		     const vpCameraParameters &cam,
		     const bool usedistortion,
		     const vpColor::vpColorType color)
{
  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;
  projection(_cP,_p) ;
  vpFeatureDisplay::displayLine(_p[0],_p[1],
				cam, I, usedistortion, color) ;

}
//! for memory issue (used by the vpServo class only)
vpLine *vpLine::duplicate() const
{
  vpLine *feature = new vpLine(*this) ;
  return feature ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
