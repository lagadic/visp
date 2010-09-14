/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * 2D point visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpFeaturePoint.cpp
  \brief Class that defines 2D point visual feature
*/


#include <visp/vpBasicFeature.h>
#include <visp/vpFeaturePoint.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>

// math
#include <visp/vpMath.h>

#include <visp/vpFeatureDisplay.h>



/*

attributes and members directly related to the vpBasicFeature needs
other functionalities ar usefull but not mandatory

*/

/*!
  Initialize the memory space requested for 2D point visual feature.
*/
void
vpFeaturePoint::init()
{
    //feature dimension
    dim_s = 2 ;
    nbParameters = 3;

    // memory allocation
    s.resize(dim_s) ;
    if (flags == NULL)
      flags = new bool[nbParameters];
    for (int i = 0; i < nbParameters; i++) flags[i] = false;

    //default value Z (1 meters)
    Z = 1;

}

/*! 
  Default constructor that build a visual feature.
*/
vpFeaturePoint::vpFeaturePoint() : vpBasicFeature()
{
    init() ;
}


/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/
void
vpFeaturePoint::set_Z(const double Z)
{
    this->Z = Z ;
    flags[2] = true;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
vpFeaturePoint::get_Z() const
{
    return Z ;
}


/*!  

  Set the value of \f$ x \f$ which represents the x coordinate of the point
  in image plan. It is one parameter of the visual feature \f$ s \f$.

  \param x : \f$ x \f$ value to set.
*/
void
vpFeaturePoint::set_x(const double x)
{
    s[0] = x ;
    flags[0] = true;
}


/*!
  Get the value of \f$ x \f$ which represents the x coordinate of the point in the image plan. It is one parameter of the visual feature \f$ s \f$.

  \return The value of \f$ x \f$.
*/
double
vpFeaturePoint::get_x() const
{
    return s[0] ;
}


/*!
  Set the value of \f$ y \f$ which represents the x coordinate of the point in the image plan. It is one parameter of the visual feature \f$ s \f$.

  \param y : \f$ y \f$ value to set.
*/
void
vpFeaturePoint::set_y(const double y)
{
    s[1] = y ;
    flags[1] = true;
}


/*!
  Get the value of \f$ y \f$ which represents the x coordinate of the point in the image plan. It is one parameter of the visual feature \f$ s \f$.

  \return The value of \f$ y \f$.
*/
double
vpFeaturePoint::get_y() const
{
    return s[1] ;
}


/*!

  Set the value of \f$ x \f$, \f$ y \f$ and \f$ Z \f$. \f$ x \f$ and \f$ y \f$
  represent the coordinates of the point in the image plan and are the
  parameters of the visual feature \f$ s \f$. \f$ Z \f$ is the 3D coordinate in the camera frame
  representing the depth.

  \param x : \f$ x \f$ value to set.
  \param y : \f$ y \f$ value to set.
  \param Z : \f$ Z \f$ value to set.
*/
void
vpFeaturePoint::set_xyZ(const double x,
			const double y,
			const double Z)
{
  set_x(x) ;
  set_y(y) ;
  set_Z(Z) ;
  for( int i = 0; i < nbParameters; i++) flags[i] = true;
}


/*!

  Compute and return the interaction matrix \f$ L \f$. The computation is made
  thanks to the values of the point features \f$ x \f$ and \f$ y \f$ and the
  depth \f$ Z \f$.

  \f[ L = \left[\begin{array}{c}L_{x} \\ L_{y}\end{array}\right] =  
  \left[\begin{array}{cccccc}
  -1/Z & 0 & x/Z & xy & -(1+x^2) & y \\
  0 & -1/Z & y/Z & 1+y^2 & -xy & -x
  \end{array}\right]\f]

  \param select : Selection of a subset of the possible point features. 
  - To compute the interaction matrix for all the two point features use
    vpBasicFeature::FEATURE_ALL. In that case the dimension of the interaction
    matrix is \f$ [2 \times 6] \f$
  - To compute the interaction matrix for only one of the point component
    feature (\f$ x, y \f$) use one of the corresponding function selectX() or
    selectY(). In that case the returned interaction matrix is \f$ [1 \times 6]
    \f$ dimension.

  \return The interaction matrix computed from the point features.

  The code below shows how to compute the interaction matrix associated to the
  visual feature \f$ s = x \f$.
  \code
  // Creation of the current feature s
  vpFeaturePoint s;
  s.buildFrom(0, 0, 1);

  vpMatrix L_x = s.interaction( vpFeaturePoint::selectX() );
  \endcode

  The code below shows how to compute the interaction matrix associated to the
  visual feature \f$ s = (x, y) \f$.
  \code
  // Creation of the current feature s
  vpFeaturePoint s;
  s.buildFrom(0, 0, 1);

  vpMatrix L_x = s.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode
*/
vpMatrix
vpFeaturePoint::interaction(const int select)
{
  vpMatrix L ;

  L.resize(0,6) ;

  if (deallocate == vpBasicFeature::user)
  {
    for (int i = 0; i < nbParameters; i++)
    {
      if (flags[i] == false)
      {
        switch(i){
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but x was not set yet");
        break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but y was not set yet");
        break;
        case 2:
          vpTRACE("Warning !!!  The interaction matrix is computed but Z was not set yet");
        break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  double x = get_x() ;
  double y = get_y() ;
  double Z = get_Z() ;

  if (Z < 0)
  {
    vpERROR_TRACE("Point is behind the camera ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point is behind the camera ")) ;
  }

  if (fabs(Z) < 1e-6)
  {
    vpERROR_TRACE("Point Z coordinates is null ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point Z coordinates is null")) ;
  }

  if (vpFeaturePoint::selectX() & select )
  {
    vpMatrix Lx(1,6) ; Lx = 0;

    Lx[0][0] = -1/Z  ;
    Lx[0][1] = 0 ;
    Lx[0][2] = x/Z ;
    Lx[0][3] = x*y ;
    Lx[0][4] = -(1+x*x) ;
    Lx[0][5] = y ;

    L = vpMatrix::stackMatrices(L,Lx) ;
  }

  if (vpFeaturePoint::selectY() & select )
  {
    vpMatrix Ly(1,6) ; Ly = 0;

    Ly[0][0] = 0 ;
    Ly[0][1]  = -1/Z ;
    Ly[0][2] = y/Z ;
    Ly[0][3] = 1+y*y ;
    Ly[0][4] = -x*y ;
    Ly[0][5] = -x ;

    L = vpMatrix::stackMatrices(L,Ly) ;
  }
  return L ;
}


/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  \param s_star : Desired visual feature.

  \param select : The error can be computed for a selection of a
  subset of the possible point features.
  - To compute the error for all the two point features use vpBasicFeature::FEATURE_ALL. In that case the error vector is a 2 dimension column vector.
  - To compute the error for only one of the point component feature (\f$ x, y \f$) use one of the corresponding function selectX() or selectY(). In that case the error vector is a 1 dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired visual feature.

  The code below shows how to use this method to manipulate the \f$ x \f$ subset:
  \code
  // Creation of the current feature s
  vpFeaturePoint s;
  s.buildFrom(0, 0, 1);

  // Creation of the desired feature s*
  vpFeaturePoint s_star;
  s_star.buildFrom(1, 1, 1);

  // Compute the interaction matrix for the x feature
  vpMatrix L_x = s.interaction( vpFeaturePoint::selectX() );

  // Compute the error vector (s-s*) for the x feature
  s.error(s_star, vpFeaturePoint::selectX());
  \endcode
*/
vpColVector
vpFeaturePoint::error(const vpBasicFeature &s_star,
		      const int select)
{
  vpColVector e(0) ;

  try{
    if (vpFeaturePoint::selectX() & select )
    {
      vpColVector ex(1) ;
      ex[0] = s[0] - s_star[0] ;

      e = vpMatrix::stackMatrices(e,ex) ;
    }

    if (vpFeaturePoint::selectY() & select )
    {
      vpColVector ey(1) ;
      ey[0] = s[1] - s_star[1] ;
      e =  vpMatrix::stackMatrices(e,ey) ;
    }
  }
  catch(vpMatrixException me)
  {
    vpERROR_TRACE("caught a Matrix related error") ;
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


/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible point features.
  - To print all the two point features use vpBasicFeature::FEATURE_ALL.
  - To print only one of the point component feature (\f$ x, y \f$) use one of the corresponding function selectX() or selectY().

  \code
  vpFeaturePoint s; // Current visual feature s

  // Creation of the current feature s
  s.buildFrom(0, 0, 1);

  s.print(); // print all the 2 components of the feature
  s.print(vpBasicFeature::FEATURE_ALL);  // same behavior then previous line
  s.print(vpFeaturePoint::selectX()); // print only the x component
  \endcode
*/
void
vpFeaturePoint::print(const int select ) const
{

  std::cout <<"Point:  Z=" << get_Z() ;
  if (vpFeaturePoint::selectX() & select )
    std::cout << " x=" << get_x() ;
  if (vpFeaturePoint::selectY() & select )
    std::cout << " y=" << get_y() ;
  std::cout <<std::endl ;
}


/*!
  Build a 2D point visual feature from the point coordinates in the image plan \f$ x \f$ and \f$ y \f$. The parameter Z which describes the depth, is set in the same time.

  See the vpFeaturePoint class description for more details about \f$ x \f$ and \f$ y \f$.

  \param x : The \f$ x \f$ parameter.
  \param y : The \f$ y \f$ parameter.
  \param Z : The \f$ Z \f$ parameter.
*/
void
vpFeaturePoint::buildFrom(const double x, const double y, const double Z)
{

  s[0] = x ;
  s[1] = y ;

  this->Z = Z  ;

  if (Z < 0)
  {
    vpERROR_TRACE("Point is behind the camera ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point is behind the camera ")) ;
  }

  if (fabs(Z) < 1e-6)
  {
    vpERROR_TRACE("Point Z coordinates is null ") ;
    std::cout <<"Z = " << Z << std::endl ;

    throw(vpFeatureException(vpFeatureException::badInitializationError,
			     "Point Z coordinates is null")) ;
  }

  for( int i = 0; i < nbParameters; i++) flags[i] = true;

}


/*!

  Display point feature.

  \param cam : Camera parameters.
  \param I : Image.
  \param color : Color to use for the display.
  \param thickness : Thickness of the feature representation.

*/
void
vpFeaturePoint::display(const vpCameraParameters &cam,
			vpImage<unsigned char> &I,
			vpColor color, 
			unsigned int thickness) const
{
  try{
    double x,y ;
    x = get_x() ;
    y = get_y() ;

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness) ;

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}

/*!

  Display point feature.

  \param cam : Camera parameters.
  \param I : color Image.
  \param color : Color to use for the display.
  \param thickness : Thickness of the feature representation.

*/
void
vpFeaturePoint::display(const vpCameraParameters &cam,
                        vpImage<vpRGBa> &I,
                        vpColor color, 
			unsigned int thickness) const
{
  try{
    double x,y ;
    x = get_x() ;
    y = get_y() ;

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness) ;

  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
}


/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeaturePoint s;
  s_star = s.duplicate(); // s_star is now a vpFeaturePoint
  \endcode

*/
vpFeaturePoint *vpFeaturePoint::duplicate() const
{
  vpFeaturePoint *feature = new vpFeaturePoint ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
