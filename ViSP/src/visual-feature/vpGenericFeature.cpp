
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpGenericFeature.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpGenericFeature.cpp,v 1.8 2006-04-19 09:01:23 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a generic feature (used to create new
 *     feature not implemented in ViSP2
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
  ERROR_TRACE("You are not allow to use this constructor ") ;
  ERROR_TRACE("Please, use  vpGenericFeature::vpGenericFeature(int _dim) "
	      "constructor") ;
  ERROR_TRACE("And provide the dimension of the visual feature ") ;

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
    ERROR_TRACE("size mismatch between error dimension"
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
    ERROR_TRACE("size mismatch between s* dimension "
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
      ERROR_TRACE("Error has no been updated since last iteration"
		  "you should have used vpGenericFeature::setError"
		  "in you visual servoing loop") ;
      throw(vpFeatureException(vpFeatureException::badErrorVectorError,
			       "Error has no been updated since last iteration"));
    }
    else
      if (errorStatus == errorInitialized)
      {
	DEBUG_TRACE(25,"Error init: e=e.");
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
	DEBUG_TRACE(25,"Error not init: e=s-s*.");

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

//! compute the error between a visual features and zero
vpColVector
vpGenericFeature::error( const int select)
{


  vpColVector e(0) ;

  try
  {
    if (errorStatus == errorHasToBeUpdated)
    {
      ERROR_TRACE("Error has no been updated since last iteration"
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


//! compute the interaction matrix from a subset a the possible features
vpMatrix
vpGenericFeature::interaction(const int select) const
{
  if (L.getRows() == 0)
  {
    cout << "interaction matrix " << L << endl ;
    ERROR_TRACE("Interaction has not been initialized");
    cout << "A possible reason (may be) is that you have set" << endl ;
    cout << "the interaction matrix for s and compute a control " << endl ;
    cout << "with Ls=s* (default) or vice versa" << endl ;

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
    cout << L.getRows() <<"  " << dim_s << endl ;;
    ERROR_TRACE("size mismatch between interaction matrix size "
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
    ERROR_TRACE("size mismatch between s dimension"
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
    ERROR_TRACE("size mismatch between number of parameters"
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
    ERROR_TRACE("size mismatch between number of parameters"
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
    ERROR_TRACE("size mismatch between number of parameters"
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

  cout <<"Generic Feature: "  ;
  for (int i=0 ; i < dim_s ; i++)
    if (FEATURE_LINE[i] & select )
    {
      cout << " s["<<i << "]=" << s[i] ;
    }

  cout <<endl ;
}

vpGenericFeature *vpGenericFeature::duplicate() const
{
  vpGenericFeature *feature= new vpGenericFeature(dim_s) ;

  TRACE("dims = %d",dim_s) ;
  return feature ;
}

void
vpGenericFeature::display(const vpCameraParameters &cam,
			      vpImage<unsigned char> &I,
			      int color) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    ERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
