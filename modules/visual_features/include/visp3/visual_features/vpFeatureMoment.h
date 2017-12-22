/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Base for all moment features
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
\file vpFeatureMoment.h
\brief Base class for moment features.

Handles common system operations like selection, duplication. Functionality is
computed in derived classes.
*/
#ifndef __FEATUREMOMENT_H__
#define __FEATUREMOMENT_H__

#include <vector>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpBasicFeature.h>

class vpMomentObject;
class vpMomentDatabase;
class vpFeatureMomentDatabase;
class vpMoment;

/*!
\class vpFeatureMoment

\ingroup group_visual_features

\brief This class defines shared system methods/attributes for 2D moment
features but no functional code. It is used to compute interaction matrices
for moment features.

While vpMoment-type classes do only compute moment values and can by used for
almost anything, vpFeatureMoment-type classes are specifically designed for
visual servoing. More importantly, a vpFeatureMoment is used to compute the
interaction matrix associated to it's moment primitive.

This class is virtual and cannot be used directly. It defines the following
characteristics common to all moment features:
- Plane orientation parameters (A,B,C):
Each camera frame corresponds to a physical planar object contained in a
plane. This plane's equation has the following form: \f$ A \times x+B \times y
+ C = \frac{1}{Z} \f$. These parameters can be updated anytime.
- Get corresponding moment primitive: for example a vpFeatureMomentCInvariant
will provide access to a vpMomentCInvariant instance.
- Provide access to a feature database (vpFeatureMomentDatabase).
- All interaction matrices (different from vpBasicFeature::interaction which
selects the required interaction matrix).

Like vpMoment, vpFeatureMoment provides a vpFeatureMoment::update() method.
But unlike vpMoment::update() which only acknowledges the new object, the
vpFeatureMoment::update() acknowledges the new plane parameters AND computes
the interaction matrices associated with the feature.

A vpFeatureMoment will be often part of a vpFeatureMomentDatabase in the same
way a vpMoment is part of a vpMomentDatabase. This database is specified
inside the vpFeatureMoment::vpFeatureMoment() constructor. As a result, a
vpFeatureMoment will be able to access other vpFeatureMoments through this
database.

A vpBasicFeature can be duplicated into a vpMomentGenericFeature. In that
case, all data in the vpBasicFeature is copied but the feature's name is lost.
For example if a vpFeatureMomentCInvariant is duplicated, the duplicata will
be operational but could not be used in a vpFeatureMomentDatabase.

Note that you can use vpFeatureMoment to do visual servoing but it is not it's
only purpose. You may compute your interaction matrices with
vpFeatureMoment::update() and use them for any purpose.

\attention - A vpFeatureMoment is not responsible for updating the moment
primitives it depends on. Make sure your vpMoments are all up to date before
computing an interaction matrix using vpFeatureMoment.

\attention - Be careful with orders. Often, computing a feature of order n
requires vpMoment primitives of order n+1. Make sure to check the
documentation of the specialised vpFeatureMoment classes when deciding to
which order you want to initialize the object. An object of order 6 should be
sufficient for all classic implementations of vpFeatureMoment.

Here is an example of how to use a vpFeatureMoment (in this case
vpFeatureMomentBasic).
\code
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpFeatureMoment.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  // Init object of order 3 because we need vpFeatureMomentBasic of order 2 which
  // implies third-order moment primitives
  vpMomentObject obj(3);
  obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
  obj.fromVector(vec_p);

  vpMomentDatabase mdb; //database for moment primitives. This will
  //only contain the basic moment.
  vpMomentBasic bm; //basic moment (this particular moment is nothing
  //more than a shortcut to the vpMomentObject)
  bm.linkTo(mdb); //add basic moment to moment database

  vpFeatureMomentBasic fmb(mdb,0,0,1,NULL);

  //update and compute the vpMoment BEFORE doing any operations with vpFeatureMoment
  bm.update(obj);
  bm.compute();

  fmb.update(0,0,1); //update the vpFeatureMoment with a plane
  //configuration
  std::cout << fmb.interaction(1,1) << std::endl;
}
\endcode
*/
class VISP_EXPORT vpFeatureMoment : public vpBasicFeature
{
protected:
  const vpMoment *moment;
  const vpMoment &getMoment() const { return *moment; }
  vpMomentDatabase &moments;
  vpFeatureMomentDatabase *featureMomentsDataBase;
  std::vector<vpMatrix> interaction_matrices;

  double A;
  double B;
  double C;
  char _name[255];

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpFeatureMoment(const vpFeatureMoment &fm)
  //    : vpBasicFeature(), moment(NULL), moments(fm.moments),
  //    featureMomentsDataBase(NULL),
  //      interaction_matrices(), A(0), B(0), C(0)
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //  vpFeatureMoment &operator=(const vpFeatureMoment &){
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane, feature database and matrix size. \param
  data_base : Moment database. The database of moment primitives (first
  parameter) is mandatory. It is used to access different moment values later
  used to compute the final matrix. \param A_ : Plane coefficient in a \f$ A
  \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param B_ : Plane
  coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
  \param C_ : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param featureMoments : Feature database \param
  nbmatrices : If you want to create a new vpFeatureMoment implementation,
  your feature will often have a matrix size of n lines. You can specify the
  number of lines by this parameter.
  */
  vpFeatureMoment(vpMomentDatabase &data_base, double A_ = 0.0, double B_ = 0.0, double C_ = 0.0,
                  vpFeatureMomentDatabase *featureMoments = NULL, unsigned int nbmatrices = 1)
    : vpBasicFeature(), moment(NULL), moments(data_base), featureMomentsDataBase(featureMoments),
      interaction_matrices(nbmatrices), A(A_), B(B_), C(C_), _name()
  {
  }

  virtual ~vpFeatureMoment();

  /** @name Inherited functionalities from vpFeatureMoment */
  //@{
  virtual void compute_interaction(void);
  vpBasicFeature *duplicate() const;
  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  int getDimension(unsigned int select = FEATURE_ALL) const;
  void init(void);
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);
  void linkTo(vpFeatureMomentDatabase &featureMoments);

  /*!
      Name of the moment corresponding to the feature. This allows to locate
     the moment associated with the feature in the provided database.
      */
  virtual const char *momentName() const = 0;
  /*!
      Name of the feature used to locate it in the database of features.
      */
  virtual const char *name() const = 0;
  void print(const unsigned int select = FEATURE_ALL) const;
  virtual void printDependencies(std::ostream &os) const;

  void update(double A, double B, double C);

  //@}
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpFeatureMoment &featM);
};

/*!
\class vpMomentGenericFeature

\ingroup group_visual_features

\brief This class defines a generic feature used for moment feature
duplication.

A vpBasicFeature can be duplicated into a vpMomentGenericFeature. In that
case, all data in the vpBasicFeature is copied but the feature's name is lost.
For example if a vpFeatureMomentCInvariant is duplicated, the duplicata will
be operational but could not be used in a vpFeatureMomentDatabase. The reason
for this is that a vpMomentGenericFeature can refer to anything therefore it
has no specific name.

Duplication is mostly used internally in ViSP.

*/
class VISP_EXPORT vpMomentGenericFeature : public vpFeatureMoment
{
public:
  vpMomentGenericFeature(vpMomentDatabase &data_base, double A_, double B_, double C_,
                         vpFeatureMomentDatabase *featureMoments, const vpMoment *p_moment)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments)
  {
    this->moment = p_moment;
  }
  /*!
  No specific moment name.
  */
  const char *momentName() const { return NULL; }
  /*!
  No specific feature name.
  */
  virtual const char *name() const { return NULL; }
};

#endif
