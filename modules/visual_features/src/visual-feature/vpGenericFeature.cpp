/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Generic feature (used to create new feature not implemented in ViSP).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/visual_features/vpGenericFeature.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

/*!
  \file vpGenericFeature.cpp
  Class that defines what is a generic feature. This class could be used to
  create new features not implemented in ViSP.
*/

vpGenericFeature::~vpGenericFeature() {}

void vpGenericFeature::init() { s = 0; }

/*!

  Default constructor. You are not allowed to use this
  constructor. Please use the vpGenericFeature::vpGenericFeature(int
  _dim) constructor.

  \exception vpException::cannotUseConstructorError : If the use call
  this constructor.

*/
vpGenericFeature::vpGenericFeature() : L(), err(), errorStatus(errorNotInitalized)
{
  /*
  vpERROR_TRACE("You are not allow to use this constructor ") ;
  vpERROR_TRACE("Please, use  vpGenericFeature::vpGenericFeature(int _dim) "
              "constructor") ;
  vpERROR_TRACE("And provide the dimension of the visual feature ") ;
  throw(vpException(vpException::cannotUseConstructorError,
                             "You are not allow to use this constructor ")) ;
  */
}

/*!
  Constructor of the class you have to use. The feature table is initilialized
  with the good dimension.

  \param dimension_gen_s : Dimension of the generic feature. It corresponds to
  the number of features you want to create.
*/
vpGenericFeature::vpGenericFeature(unsigned int dimension_gen_s) : L(), err(), errorStatus(errorNotInitalized)
{
  this->dim_s = dimension_gen_s;
  s.resize(dimension_gen_s);
}

/*!

  Set the error vector \f$(s-s*)\f$.

  \param error_vector : Error vector \f$(s-s*)\f$.

  \exception vpFeatureException::sizeMismatchError : If the size of
  the error vector is bad.
*/
void vpGenericFeature::setError(const vpColVector &error_vector)
{
  if (error_vector.getRows() != dim_s) {
    vpERROR_TRACE("size mismatch between error dimension"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between error dimension"
                                                                    "and feature dimension"));
  }
  errorStatus = errorInitialized;
  err = error_vector;
}

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  \exception if errorHasBeenInitialized is true (that is if
  vpGenericFeature::setError have been used) then s_star is useless.  In that
  since the error HAS TO BE recomputed at each iteration
  errorHasBeenInitialized is set to errHasToBeUpdated if
  vpGenericFeature::serError is not used in the loop then an exception is
  thrown

  obviously if vpGenericFeature::setError is not used then s_star is
  considered and this warning is meaningless.

  \param s_star : Desired visual feature.

  \param select : The error can be computed for a selection of a
  subset of the possible features.
  - To compute the error for all the features use
    vpBasicFeature::FEATURE_ALL. In that case the error vector column
    vector whose dimension is equal to the number of features.
  - To compute the error for only one of the component feature you
    have to say which one you want to take into account. If it is the
    first one set select to vpBasicFeature::FEATURE_LINE[0], if it is
    the second one set select to vpBasicFeature::FEATURE_LINE[1], and
    so on. In that case the error vector is a 1 dimension column
    vector.
  - To compute the error for only two of the component feature you
    have to say which ones you want to take into account. If it is the
    first one and the second one set select to
    vpBasicFeature::FEATURE_LINE[0] | vpBasicFeature::FEATURE_LINE[1]. In
    that case the error vector is a 2 dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  The code below shows how to use this method to manipulate the two
  visual features over three:

  \code
  // Creation of the current feature s
  vpGenericFeature s(3);
  s.set_s(0, 0, 0);

  // Creation of the desired feature s*
  vpGenericFeature s_star(3);
  s_star.set_s(1, 1, 1);

  // Here you have to compute the interaction matrix L
  s.setInteractionMatrix(L);

  // Compute the error vector (s-s*) for the two first features
  s.error(s_star, vpBasicFeature::FEATURE_LINE[0] | vpBasicFeature::FEATURE_LINE[1]);
  \endcode
*/
vpColVector vpGenericFeature::error(const vpBasicFeature &s_star, const unsigned int select)
{
  if (s_star.get_s().getRows() != dim_s) {
    vpERROR_TRACE("size mismatch between s* dimension "
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between s* dimension "
                                                                    "and feature dimension"));
  }

  vpColVector e(0);

  try {
    if (errorStatus == errorHasToBeUpdated) {
      vpERROR_TRACE("Error has no been updated since last iteration"
                    "you should have used vpGenericFeature::setError"
                    "in you visual servoing loop");
      throw(vpFeatureException(vpFeatureException::badErrorVectorError,
                               "Error has no been updated since last iteration"));
    } else if (errorStatus == errorInitialized) {
      vpDEBUG_TRACE(25, "Error init: e=e.");
      errorStatus = errorHasToBeUpdated;
      for (unsigned int i = 0; i < dim_s; i++)
        if (FEATURE_LINE[i] & select) {
          vpColVector ex(1);
          ex[i] = err[i];

          e = vpColVector::stack(e, ex);
        }
    } else {
      vpDEBUG_TRACE(25, "Error not init: e=s-s*.");

      for (unsigned int i = 0; i < dim_s; i++)
        if (FEATURE_LINE[i] & select) {
          vpColVector ex(1);
          ex[0] = s[i] - s_star[i];

          e = vpColVector::stack(e, ex);
        }
    }
  } catch (...) {
    throw;
  }
  return e;
}

/*!

  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features. But in this
  case the desired feature is considered as set to 0.

  \param select : The error can be computed for a selection of a
  subset of the possible features.
  - To compute the error for all the features use
    vpBasicFeature::FEATURE_ALL. In that case the error vector column
    vector whose dimension is equal to the number of features.
  - To compute the error for only one of the component feature you
    have to say which one you want to take into account. If it is the
    first one set select to vpBasicFeature::FEATURE_LINE[0], if it is
    the second one set select to vpBasicFeature::FEATURE_LINE[1], and
    so on. In that case the error vector is a 1 dimension column
    vector.
  - To compute the error for only two of the component feature you
    have to say which ones you want to take into account. If it is the
    first one and the second one set select to
    vpBasicFeature::FEATURE_LINE[0] | vpBasicFeature::FEATURE_LINE[1]. In
    that case the error vector is a 2 dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature which is automatically set to zero.

  The code below shows how to use this method to manipulate the two
  visual features over three:
  \code
  // Creation of the current feature s
  vpGenericFeature s(3);
  s.set_s(0, 0, 0);

  // Here you have to compute the interaction matrix L
  s.setInteractionMatrix(L);

  // Compute the error vector (s-s*) for the two first features
  s.error(vpBasicFeature::FEATURE_LINE[0] | vpBasicFeature::FEATURE_LINE[1]);
  \endcode
*/
vpColVector vpGenericFeature::error(const unsigned int select)
{
  vpColVector e(0);

  try {
    if (errorStatus == errorHasToBeUpdated) {
      vpERROR_TRACE("Error has no been updated since last iteration"
                    "you should have used vpGenericFeature::setError"
                    "in you visual servoing loop");
      throw(vpFeatureException(vpFeatureException::badErrorVectorError,
                               "Error has no been updated since last iteration"));
    } else if (errorStatus == errorInitialized) {
      errorStatus = errorHasToBeUpdated;
      for (unsigned int i = 0; i < dim_s; i++)
        if (FEATURE_LINE[i] & select) {
          vpColVector ex(1);
          ex[i] = err[i];

          e = vpColVector::stack(e, ex);
        }
    } else {

      for (unsigned int i = 0; i < dim_s; i++)
        if (FEATURE_LINE[i] & select) {
          vpColVector ex(1);
          ex[i] = s[i];

          e = vpColVector::stack(e, ex);
        }
    }
  } catch (...) {
    throw;
  }

  return e;
}

/*!

  Compute and return the interaction matrix \f$ L \f$ for the whole
  features or a part of them.

  \param select : Selection of a subset of the possible features.
  - To compute the interaction matrix for all the features use
    vpBasicFeature::FEATURE_ALL. In that case the dimension of the interaction
    matrix is \f$ [number of features \times 6] \f$
  - To compute the interaction matrix for only one of the component
    feature you have to say which one you want to take into
    account. If it is the first one set select to
    vpBasicFeature::FEATURE_LINE[0], if it is the second one set
    select to vpBasicFeature::FEATURE_LINE[1], and so on. In that case
    the returned interaction matrix is \f$ [1 \times 6] \f$ dimension.
  - To compute the interaction matrix for only two of the component
    features you have to say which ones you want to take into
    account. If it is the first one and the second one set select to
    vpBasicFeature::FEATURE_LINE[0] | vpBasicFeature::FEATURE_LINE[1]. In
    that case the returned interaction matrix is \f$ [2 \times 6] \f$
    dimension.

  \return The interaction matrix computed from the features.

  The code below shows how to compute the interaction matrix associated to the
  first visual feature.
  \code
  // Creation of the current feature s
  vpGenericFeature s(3);
  s.set_s(0, 0, 0);

  // Here you have to compute the interaction matrix L for all the three
  features s.setInteractionMatrix(L);

  vpMatrix L_x = s.interaction( vpBasicFeature::FEATURE_LINE[0] );
  \endcode

  The code below shows how to compute the interaction matrix associated to two
  visual features over three.
  \code
  // Creation of the current feature s
  vpGenericFeature s(3);
  s.set_s(0, 0, 0);

  // Here you have to compute the interaction matrix L
  s.setInteractionMatrix(L);

  vpMatrix L_x = s.interaction( vpBasicFeature::FEATURE_LINE[0]|vpBasicFeature::FEATURE_LINE[1] );
  \endcode
*/
vpMatrix vpGenericFeature::interaction(const unsigned int select)
{
  if (L.getRows() == 0) {
    std::cout << "interaction matrix " << L << std::endl;
    vpERROR_TRACE("Interaction has not been initialized");
    std::cout << "A possible reason (may be) is that you have set" << std::endl;
    std::cout << "the interaction matrix for s and compute a control " << std::endl;
    std::cout << "with Ls=s* (default) or vice versa" << std::endl;

    throw(vpFeatureException(vpFeatureException::notInitializedError, "size mismatch between s* dimension "
                                                                      "and feature dimension"));
  }

  vpMatrix Ls;

  Ls.resize(0, 6);

  for (unsigned int i = 0; i < dim_s; i++)
    if (FEATURE_LINE[i] & select) {
      vpMatrix Lx(1, 6);
      Lx = 0;

      for (int j = 0; j < 6; j++)
        Lx[0][j] = L[i][j];

      Ls = vpMatrix::stack(Ls, Lx);
    }

  return Ls;
}

/*!
  \brief set the value of the interaction matrix.

  \param L_ : The matrix corresponding to the interaction matrix you computed.

  \exception an exception is thrown if the number of row of the interaction
  matrix is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::setInteractionMatrix(const vpMatrix &L_)
{
  if (L_.getRows() != dim_s) {
    std::cout << L_.getRows() << "  " << dim_s << std::endl;
    ;
    vpERROR_TRACE("size mismatch between interaction matrix size "
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between interaction matrix size "
                                                                    "and feature dimension"));
  }

  this->L = L_;
}

/*!
  \brief set the value of all the features.

  \param s_vector : It is a vector containing the value of the visual
  features.

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::set_s(const vpColVector &s_vector)
{

  if (s_vector.getRows() != dim_s) {
    vpERROR_TRACE("size mismatch between s dimension"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between s dimension"
                                                                    "and feature dimension"));
  }
  this->s = s_vector;
}

/*!
  \brief get the value of all the features.

  \param s_vector : It is a vector which will contain the value of the visual
  features.

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::get_s(vpColVector &s_vector) const
{
  if (s_vector.getRows() != dim_s) {
    vpERROR_TRACE("size mismatch between s dimension"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between s dimension"
                                                                    "and feature dimension"));
  }
  s_vector = this->s;
}

/*!
  \brief set the value of three features if the number of feature is equal
  to 3.

  \param s0 : value of the first visual feature

  \param s1 : value of the second visual feature

  \param s2 : value of the third visual feature

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::set_s(const double s0, const double s1, const double s2)
{

  if (3 != dim_s) {
    vpERROR_TRACE("size mismatch between number of parameters"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between  number of parameters"
                                                                    "and feature dimension"));
  }
  s[0] = s0;
  s[1] = s1;
  s[2] = s2;
}

/*!
  \brief get the value of three features if the number of feature is equal
  to 3.

  \param s0 : value of the first visual feature

  \param s1 : value of the second visual feature

  \param s2 : value of the third visual feature

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::get_s(double &s0, double &s1, double &s2) const
{

  if (3 != dim_s) {
    vpERROR_TRACE("size mismatch between number of parameters"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between  number of parameters"
                                                                    "and feature dimension"));
  }
  s0 = s[0];
  s1 = s[1];
  s2 = s[2];
}

/*!
  \brief set the value of two features if the number of feature is equal to 2.

  \param s0 : value of the first visual feature

  \param s1 : value of the second visual feature

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::set_s(const double s0, const double s1)
{

  if (2 != dim_s) {
    vpERROR_TRACE("size mismatch between number of parameters"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between  number of parameters"
                                                                    "and feature dimension"));
  }
  s[0] = s0;
  s[1] = s1;
}

/*!
  \brief get the value of two features if the number of feature is equal to 2.

  \param s0 : value of the first visual feature

  \param s1 : value of the second visual feature

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::get_s(double &s0, double &s1) const
{

  if (2 != dim_s) {
    vpERROR_TRACE("size mismatch between number of parameters"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between  number of parameters"
                                                                    "and feature dimension"));
  }
  s0 = s[0];
  s1 = s[1];
}

/*!
  \brief set the value of one feature if the number of feature is equal to 1.

  \param s0 : value of the visual feature

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::set_s(const double s0)
{

  if (1 != dim_s) {
    vpERROR_TRACE("size mismatch between number of parameters"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between  number of parameters"
                                                                    "and feature dimension"));
  }
  s[0] = s0;
}

/*!
  \brief get the value of one feature if the number of feature is equal to 1.

  \param s0 : value of the visual feature

  \exception an exception is thrown if the number of row of the vector s
  is different from the dimension of the visual feature as specified
  in the constructor
*/
void vpGenericFeature::get_s(double &s0) const
{

  if (1 != dim_s) {
    vpERROR_TRACE("size mismatch between number of parameters"
                  "and feature dimension");
    throw(vpFeatureException(vpFeatureException::sizeMismatchError, "size mismatch between  number of parameters"
                                                                    "and feature dimension"));
  }
  s0 = s[0];
}

/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible features.
  - To print all the features use vpBasicFeature::FEATURE_ALL.
  - To print only one of the component features you have to say which
    one you want to take into account. If it is the first one set
    select to vpBasicFeature::FEATURE_LINE[0], if it is the second one
    set select to vpBasicFeature::FEATURE_LINE[1], and so on.

  \code
  vpGenericFeature s; // Current visual feature s

  // Creation of the current feature s
  s.set_s(0, 0, 0);

  s.print(); // print all components of the feature
  s.print(vpBasicFeature::FEATURE_ALL);  // same behavior then previous line
  s.print(vpBasicFeature::FEATURE_LINE[0]); // print only the first component
  \endcode
*/
void vpGenericFeature::print(const unsigned int select) const
{

  std::cout << "Generic Feature: ";
  for (unsigned int i = 0; i < dim_s; i++)
    if (FEATURE_LINE[i] & select) {
      std::cout << " s[" << i << "]=" << s[i];
    }

  std::cout << std::endl;
}

vpGenericFeature *vpGenericFeature::duplicate() const
{
  vpGenericFeature *feature = new vpGenericFeature(dim_s);

  vpTRACE("dims = %d", dim_s);
  return feature;
}

/*!
  Not implemented.
*/
void vpGenericFeature::display(const vpCameraParameters & /* cam */, const vpImage<unsigned char> & /* I */,
                               const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}
/*!
  Not implemented.
 */
void vpGenericFeature::display(const vpCameraParameters & /* cam */, const vpImage<vpRGBa> & /* I */,
                               const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
