
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeatureLine.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureLine.cpp,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines  2D line visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpFeatureLine.cpp
  \brief Class that defines 2D line visual feature
*/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeatureLine.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>

// simple math function (round)
#include <visp/vpMath.h>

// Display Issue

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>



#include <visp/vpFeatureDisplay.h>


/*



attributes and members directly related to the vpBasicFeature needs
other functionalities ar usefull but not mandatory





*/

/*!
  \brief initialization
  x cos(theta) + y sin(theta) - rho = 0
  s[0] = rho
  s[1] = theta
*/
void
vpFeatureLine::init()
{
    //feature dimension
    dim_s = 2 ;

    // memory allocation
    //  x cos(theta) + y sin(theta) - rho = 0
    // s[0] = rho
    // s[1] = theta
    s.resize(dim_s) ;

    A = B = C = D = 0.0 ;
}

vpFeatureLine::vpFeatureLine() : vpBasicFeature()
{
    init() ;
}



//! set the line xy and Z-coordinates
void
vpFeatureLine::setRhoTheta(const double _rho, const double _theta)
{
  s[0] = _rho ;
  s[1] = _theta ;
}

//! set the line xy and Z-coordinates
void
vpFeatureLine::setABCD(const double _A, const double _B,
		       const double _C, const double _D)
{
  A = _A ;
  B = _B ;
  C = _C ;
  D = _D ;
}

//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpFeatureLine::interaction(const int select) const
{
  vpMatrix L ;

  L.resize(0,6) ;

  double rho = s[0] ;
  double theta = s[1] ;


  double co = cos(theta);
  double si = sin(theta);

  if (fabs(D) < 1e-6)
  {
    ERROR_TRACE("Incorrect plane  coordinates D is null, D = %f",D) ;

    throw(vpFeatureException(vpFeatureException::badInitializationERR,
			     "Incorrect plane  coordinates D")) ;
  }

  double lambda_theta =( A*si - B*co) /D;
  double lambda_rho =  (C + rho*A*co + rho*B*si)/D;



  if (vpFeatureLine::selectRho() & select )
  {
    vpMatrix Lrho(1,6) ;


    Lrho[0][0]= co*lambda_rho;
    Lrho[0][1]= si*lambda_rho;
    Lrho[0][2]= -rho*lambda_rho;
    Lrho[0][3]= si*(1.0 + rho*rho);
    Lrho[0][4]= -co*(1.0 + rho*rho);
    Lrho[0][5]= 0.0;

    L = vpMatrix::stackMatrices(L,Lrho) ;
  }

  if (vpFeatureLine::selectTheta() & select )
  {
    vpMatrix Ltheta(1,6) ;

    Ltheta[0][0] = co*lambda_theta;
    Ltheta[0][1] = si*lambda_theta;
    Ltheta[0][2] = -rho*lambda_theta;
    Ltheta[0][3] = -rho*co;
    Ltheta[0][4] = -rho*si;
    Ltheta[0][5] = -1.0;

    L = vpMatrix::stackMatrices(L,Ltheta) ;
  }
  return L ;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector
vpFeatureLine::error(const vpBasicFeature &s_star,
		      const int select)
{
  vpColVector e(0) ;

  try{
    if (vpFeatureLine::selectRho() & select )
    {
      vpColVector erho(1) ;
      erho[0] = s[0] - s_star[0] ;



      e = vpMatrix::stackMatrices(e,erho) ;
    }

    if (vpFeatureLine::selectTheta() & select )
    {

      double err = s[1] - s_star[1] ;
      while (err < -M_PI) err += 2*M_PI ;
      while (err > M_PI) err -= 2*M_PI ;

      vpColVector etheta(1) ;
      etheta[0] = err ;
      e =  vpMatrix::stackMatrices(e,etheta) ;
    }
  }
  catch(vpMatrixException me)
  {
    ERROR_TRACE("caught a Matric related error") ;
    cout <<endl << me << endl ;
    throw(me) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("caught another error") ;
    cout <<endl << me << endl ;
    throw(me) ;
  }


  return e ;

}


void
vpFeatureLine::print(const int select ) const
{

  cout <<"Line:\t  " << A <<"X+" << B <<"Y+" << C <<"Z +" << D <<"=0" <<endl ;;
  if (vpFeatureLine::selectRho() & select )
    cout << "     \trho=" << s[0] ;
  if (vpFeatureLine::selectTheta() & select )
    cout << "     \ttheta=" << s[1] ;
  cout <<endl ;
}

void
vpFeatureLine::buildFrom(const double _rho, const double _theta)
{
  s[0] = _rho ;
  s[1] = _theta ;
}

void vpFeatureLine::buildFrom(const double _rho, const double _theta,
			      const double _A, const double _B,
			      const double _C, const double _D)
{
  s[0] = _rho ;
  s[1] = _theta ;
  A = _A ;
  B = _B ;
  C = _C ;
  D = _D ;
}

vpFeatureLine *vpFeatureLine::duplicate() const
{
  vpFeatureLine *feature  =  new vpFeatureLine ;
  return feature ;
}



//! display point feature
void
vpFeatureLine::display( const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			int color) const
{
  try{
    double rho,theta ;
    rho = getRho() ;
    theta = getTheta() ;

    vpFeatureDisplay::displayLine(rho,theta,cam,I,color) ;

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
