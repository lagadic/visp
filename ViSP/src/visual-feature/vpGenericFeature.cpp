/****************************************************************************
 *
 * $Id: vpGenericFeature.cpp,v 1.16 2008-02-26 10:32:11 asaunier Exp $
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
 * Generic feature (used to create new feature not implemented in ViSP).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpGenericFeature.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpFeatureException.h>

// Debug trace
#include <visp/vpDebug.h>



/*!
  \file vpGenericFeature.cpp
  \brief class that defines what is a generic feature (used to create new
  feature not implemented in ViSP2
*/

vpGenericFeature::~vpGenericFeature()
{

}

void vpGenericFeature::init()
{
  s = 0 ;
}


vpGenericFeature::vpGenericFeature()
{
  vpERROR_TRACE("You are not allow to use this constructor ") ;
  vpERROR_TRACE("Please, use  vpGenericFeature::vpGenericFeature(int _dim) "
	      "constructor") ;
  vpERROR_TRACE("And provide the dimension of the visual feature ") ;

  throw(vpException(vpException::cannotUseConstructorError,
			     "You are not allow to use this constructor ")) ;
}

vpGenericFeature::vpGenericFeature(int dim_s)
{
  this->dim_s = dim_s ;
  s.resize(dim_s) ;
  errorStatus = errorNotInitalized ;
}

void
vpGenericFeature::setError(vpColVector &error)
{
  if (error.getRows() != dim_s)
  {
    vpERROR_TRACE("size mismatch between error dimension"
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between error dimension"
			     "and feature dimension"));

  }
  errorStatus = errorInitialized ;
  err = error ;
}


/*!
  \brief compute the error between two visual features from a subset
  a the possible features

  \exception if errorHasBeenInitialized is true (that is if
  vpGenericFeature::setError have been used) then s_star is useless.  In that
  since the error HAS TO BE recomputed at each iteration
  errorHasBeenInitialized is set to errHasToBeUpdated if
  vpGenericFeature::serError is not used in the loop then an exception is
  thrown

  obviously if vpGenericFeature::setError is not used then s_star is considered
  and this warning is meaningless
*/
vpColVector
vpGenericFeature::error(const vpBasicFeature &s_star,
			const int select)
{
  if (s_star.get_s().getRows() != dim_s)
  {
    vpERROR_TRACE("size mismatch between s* dimension "
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between s* dimension "
			     "and feature dimension"));

  }

  vpColVector e(0) ;

  try
  {
    if (errorStatus == errorHasToBeUpdated)
    {
      vpERROR_TRACE("Error has no been updated since last iteration"
		  "you should have used vpGenericFeature::setError"
		  "in you visual servoing loop") ;
      throw(vpFeatureException(vpFeatureException::badErrorVectorError,
			       "Error has no been updated since last iteration"));
    }
    else
      if (errorStatus == errorInitialized)
      {
	vpDEBUG_TRACE(25,"Error init: e=e.");
	errorStatus = errorHasToBeUpdated ;
	for (int i=0 ; i < dim_s ; i++)
	  if (FEATURE_LINE[i] & select )
	  {
	    vpColVector ex(1) ;
	    ex[i] = err[i] ;

	    e = vpMatrix::stackMatrices(e,ex) ;
	  }
      }
      else
      {
	vpDEBUG_TRACE(25,"Error not init: e=s-s*.");

	for (int i=0 ; i < dim_s ; i++)
	  if (FEATURE_LINE[i] & select )
	  {
	    vpColVector ex(1) ;
	    ex[0] = s[i] - s_star[i] ;

	    e = vpMatrix::stackMatrices(e,ex) ;
	  }

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

//! compute the error between a visual features and zero
vpColVector
vpGenericFeature::error( const int select)
{


  vpColVector e(0) ;

  try
  {
    if (errorStatus == errorHasToBeUpdated)
    {
      vpERROR_TRACE("Error has no been updated since last iteration"
		  "you should have used vpGenericFeature::setError"
		  "in you visual servoing loop") ;
      throw(vpFeatureException(vpFeatureException::badErrorVectorError,
			       "Error has no been updated since last iteration"));
    }
    else
      if (errorStatus == errorInitialized)
      {
	errorStatus = errorHasToBeUpdated ;
	for (int i=0 ; i < dim_s ; i++)
	  if (FEATURE_LINE[i] & select )
	  {
	    vpColVector ex(1) ;
	    ex[i] = err[i] ;

	    e = vpMatrix::stackMatrices(e,ex) ;
	  }
      }
      else
      {

	for (int i=0 ; i < dim_s ; i++)
	  if (FEATURE_LINE[i] & select )
	  {
	    vpColVector ex(1) ;
	    ex[i] = s[i]  ;

	    e = vpMatrix::stackMatrices(e,ex) ;
	  }

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


//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpGenericFeature::interaction(const int select) const
{
  if (L.getRows() == 0)
  {
    std::cout << "interaction matrix " << L << std::endl ;
    vpERROR_TRACE("Interaction has not been initialized");
    std::cout << "A possible reason (may be) is that you have set" << std::endl ;
    std::cout << "the interaction matrix for s and compute a control " << std::endl ;
    std::cout << "with Ls=s* (default) or vice versa" << std::endl ;

    throw(vpFeatureException(vpFeatureException::notInitializedError,
			     "size mismatch between s* dimension "
			     "and feature dimension"));

  }

  vpMatrix Ls ;

  Ls.resize(0,6) ;

  for (int i=0 ; i < dim_s ; i++)
    if (FEATURE_LINE[i] & select )
    {
      vpMatrix Lx(1,6) ; Lx = 0;

      for (int j=0 ; j < 6 ; j++)
	Lx[0][j] = L[i][j] ;

      Ls = vpMatrix::stackMatrices(Ls,Lx) ;
    }

  return Ls ;
}

/*!
  \brief set the value of the interaction
  \exception an exception is thrown if the number of row of the interaction
  matrix is different from the dimension of the visual feature as specified
  in the constructor
*/
void
vpGenericFeature::setInteractionMatrix(const vpMatrix &L)
{

  if (L.getRows() != dim_s)
  {
    std::cout << L.getRows() <<"  " << dim_s << std::endl ;;
    vpERROR_TRACE("size mismatch between interaction matrix size "
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between interaction matrix size "
			     "and feature dimension"));

  }

  this->L = L ;

}

/*!
  \brief set the value of the interaction
  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void
vpGenericFeature::set_s(const vpColVector &s)
{

  if (s.getRows() != dim_s)
  {
    vpERROR_TRACE("size mismatch between s dimension"
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between s dimension"
			     "and feature dimension"));

  }
  this->s = s ;
}

/*!
  \brief set the value of the interaction
  \exception an exception is thrown if the number of parameters 3
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void
vpGenericFeature::set_s(const double s0, const double s1, const double s2)
{

  if (3 != dim_s)
  {
    vpERROR_TRACE("size mismatch between number of parameters"
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between  number of parameters"
			     "and feature dimension"));

  }
  s[0] = s0 ; s[1] = s1 ; s[2] = s2 ;
}

/*!
  \brief set the value of the interaction
  \exception an exception is thrown if the number of parameters 2
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void
vpGenericFeature::set_s(const double s0, const double s1)
{

  if (2 != dim_s)
  {
    vpERROR_TRACE("size mismatch between number of parameters"
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between  number of parameters"
			     "and feature dimension"));

  }
  s[0] = s0 ; s[1] = s1 ;
}

/*!
  \brief set the value of the interaction
  \exception an exception is thrown if the number of parameters 1
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void
vpGenericFeature::set_s(const double s0)
{

  if (1 != dim_s)
  {
    vpERROR_TRACE("size mismatch between number of parameters"
		"and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError,
			     "size mismatch between  number of parameters"
			     "and feature dimension"));

  }
  s[0] = s0 ;
}

//! print the name of the feature
void
vpGenericFeature::print(const int select) const
{

  std::cout <<"Generic Feature: "  ;
  for (int i=0 ; i < dim_s ; i++)
    if (FEATURE_LINE[i] & select )
    {
      std::cout << " s["<<i << "]=" << s[i] ;
    }

  std::cout <<std::endl ;
}

vpGenericFeature *vpGenericFeature::duplicate() const
{
  vpGenericFeature *feature= new vpGenericFeature(dim_s) ;

  vpTRACE("dims = %d",dim_s) ;
  return feature ;
}

/*!
  Not implemented.
*/
void
vpGenericFeature::display(const vpCameraParameters &/* cam */,
			  vpImage<unsigned char> &/* I */,
			  vpColor::vpColorType /* color */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}
/*!
  Not implemented.
 */
void
vpGenericFeature::display(const vpCameraParameters &/* cam */,
                          vpImage<vpRGBa> &/* I */,
                          vpColor::vpColorType /* color */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
