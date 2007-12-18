/****************************************************************************
 *
 * $Id: vpFeatureLine.cpp,v 1.11 2007-12-18 15:17:52 fspindle Exp $
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
 * 2D line visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


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
vpFeatureLine::setRhoTheta(const double rho, const double theta)
{
  s[0] = rho ;
  s[1] = theta ;
}

//! set the line xy and Z-coordinates
void
vpFeatureLine::setABCD(const double A, const double B,
		       const double C, const double D)
{
  this->A = A ;
  this->B = B ;
  this->C = C ;
  this->D = D ;
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
    vpERROR_TRACE("Incorrect plane  coordinates D is null, D = %f",D) ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
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
    vpERROR_TRACE("caught a Matric related error") ;
    std::cout <<std::endl << me << std::endl ;
    throw(me) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("caught another error") ;
    std::cout <<std::endl << me << std::endl ;
    throw(me) ;
  }


  return e ;

}


void
vpFeatureLine::print(const int select ) const
{

  std::cout <<"Line:\t  " << A <<"X+" << B <<"Y+" << C <<"Z +" << D <<"=0" <<std::endl ;;
  if (vpFeatureLine::selectRho() & select )
    std::cout << "     \trho=" << s[0] ;
  if (vpFeatureLine::selectTheta() & select )
    std::cout << "     \ttheta=" << s[1] ;
  std::cout <<std::endl ;
}

void
vpFeatureLine::buildFrom(const double rho, const double theta)
{
  s[0] = rho ;
  s[1] = theta ;
}

void vpFeatureLine::buildFrom(const double rho, const double theta,
			      const double A, const double B,
			      const double C, const double D)
{
  s[0] = rho ;
  s[1] = theta ;
  this->A = A ;
  this->B = B ;
  this->C = C ;
  this->D = D ;
}

vpFeatureLine *vpFeatureLine::duplicate() const
{
  vpFeatureLine *feature  =  new vpFeatureLine ;
  return feature ;
}



/*!

  Display line feature.

  \param cam : Camera parameters.
  \param I : Image on which features have to be displayed.
  \param useDistortion : Not used.
  \param color : Color used to display the feature.

*/
void
vpFeatureLine::display( const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			bool /* useDistortion */,
			vpColor::vpColorType color) const
{
  try{
    double rho,theta ;
    rho = getRho() ;
    theta = getTheta() ;

    vpFeatureDisplay::displayLine(rho,theta,cam,I,color) ;

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
