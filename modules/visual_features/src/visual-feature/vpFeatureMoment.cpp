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
 * Base for all moment features
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp3/core/vpMath.h>
#include <visp3/core/vpMoment.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/visual_features/vpFeatureMoment.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>

#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

#include <vector>
#include <visp3/core/vpDebug.h>

class vpBasicFeature;

/*!
  Initialize common parameters for moment features.
*/
void vpFeatureMoment::init()
{
  // feature dimension
  /*
   * The dimension of the visual feature is set according to the size of the
   * vpMoment associated to it. This partly explains why vpFeatureMomentBasic
   * cannot be used directly as a visual feature.
   */
  if (this->moment != NULL)
    dim_s = (unsigned int)this->moment->get().size();
  else
    dim_s = 0;

  nbParameters = 1;

  // memory allocation
  s.resize(dim_s);
  for (unsigned int i = 0; i < dim_s; i++)
    s[i] = 0;

  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;
}

/*!
  Feature's dimension according to selection.
*/
int vpFeatureMoment::getDimension(unsigned int select) const
{
  int dim = 0;

  for (unsigned int i = 0; i < dim_s; ++i)
    if (vpBasicFeature::FEATURE_LINE[i] & select)
      dim++;

  return dim;
}

/*!
  Outputs the content of the feature: it's corresponding selected moments.
*/
void vpFeatureMoment::print(unsigned int select) const
{
  for (unsigned int i = 0; i < dim_s; ++i) {
    if (vpBasicFeature::FEATURE_LINE[i] & select) {
      std::cout << s[i] << ",";
    }
  }

  std::cout << std::endl;
}

/*!
  Not implemented since visual representation of a moment doesn't often make
  sense.
*/
void vpFeatureMoment::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color,
                              unsigned int thickness) const
{
  // visual representation of a moment doesn't often make sense
  (void)cam;
  (void)I;
  (void)color;
  (void)thickness;
}

/*!
  Not implemented since visual representation of a moment doesn't often make
  sense.
*/
void vpFeatureMoment::display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                              unsigned int thickness) const
{
  (void)cam;
  (void)I;
  (void)color;
  (void)thickness;
}

/*!
  Updates the interaction matrices with the image plane the camera is facing.
  The plane must be in the format: \f$ \frac{1}{Z}=Ax+By+C \f$ . The moment
  primitives MUST be updated before calling this function.

  This method also computes the interaction matrix. Therefore, you must call
  vpFeatureMoment::update before calling vpFeatureMoment::interaction.

  \attention The behaviour of this method is not the same as vpMoment::update
  which only acknowledges the new object. This method also computes the
  interaction matrices.

  \param A_ : A coefficient of the plane.
  \param B_ : B coefficient of the plane.
  \param C_ : C coefficient of the plane.
*/
void vpFeatureMoment::update(double A_, double B_, double C_)
{
  this->A = A_;
  this->B = B_;
  this->C = C_;

  if (moment == NULL) {
    bool found;
    this->moment = &(moments.get(momentName(), found));
    if (!found)
      throw vpException(vpException::notInitialized, "Moment not found for feature");
  }
  nbParameters = 1;
  if (this->moment != NULL) {
    dim_s = (unsigned int)this->moment->get().size();

    s.resize(dim_s);

    for (unsigned int i = 0; i < dim_s; i++)
      s[i] = this->moment->get()[i];

    if (flags == NULL)
      flags = new bool[nbParameters];
    for (unsigned int i = 0; i < nbParameters; i++)
      flags[i] = false;
  } else
    dim_s = 0;

  compute_interaction();
}

/*!
  Retrieves the interaction matrix. No computation is done.

  \param select : Feature selector.

  \return The corresponding interaction matrix.

  There is no rule about the format of the feature selector. It may be
  different for different features.  For example, for
  vpFeatureMomentBasic or vpFeatureMomentCentered features, select may
  refer to the \f$ (i,j) \f$ couple in the \f$ j \times order + i \f$
  format, but for vpFeatureMomentCInvariant the selector allows to
  select couples \f$ (i,j,k,l...) \f$ in the following format: 1 << i
  + 1 << j + 1 << k + 1 << l.
*/
vpMatrix vpFeatureMoment::interaction(unsigned int select)
{
  vpMatrix L(0, 0);

  for (unsigned int i = 0; i < dim_s; ++i) {
    if (vpBasicFeature::FEATURE_LINE[i] & select) {
      L.stack(interaction_matrices[i]);
    }
  }

  return L;
}

/*!  Duplicates the feature into a vpGenericFeature harbouring the
  same properties.  The resulting feature is of vpMomentGenericFeature
  type. While it still can compute interaction matrices and has acces
  to it's moment primitive, it has lost all precise information about
  its precise type and therefore cannot be used in a feature database.

  \return The corresponding feature.
*/
vpBasicFeature *vpFeatureMoment::duplicate() const
{
  vpFeatureMoment *feat = new vpMomentGenericFeature(moments, A, B, C, featureMomentsDataBase, moment);
  feat->interaction_matrices = interaction_matrices;
  feat->dim_s = dim_s;
  feat->nbParameters = nbParameters;
  // memory allocation
  feat->s.resize(dim_s);
  for (unsigned int i = 0; i < dim_s; i++)
    feat->s[i] = this->s[i];

  feat->flags = new bool[(unsigned int)nbParameters];
  for (unsigned int i = 0; i < (unsigned int)nbParameters; i++)
    feat->flags[i] = flags[i];

  return feat;
}

/*!
  Links the feature to the feature's database. NB: The feature's database is
  different from the moment's database. \param featureMoments : database in
  which the moment features are stored.

*/
void vpFeatureMoment::linkTo(vpFeatureMomentDatabase &featureMoments)
{
  if (strlen(name()) >= 255) {
    throw(vpException(vpException::memoryAllocationError, "Not enough memory to intialize the moment name"));
  }

  std::strcpy(_name, name());
  this->featureMomentsDataBase = &featureMoments;

  featureMoments.add(*this, _name);
}

void vpFeatureMoment::compute_interaction() {}

vpFeatureMoment::~vpFeatureMoment() {}

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpFeatureMoment &featM)
{
  /*
  A const_cast is forced here since interaction() defined in vpBasicFeature()
  is not const But introducing const in vpBasicFeature() can break a lot of
  client code
  */
  vpMatrix Lcomplete((unsigned int)featM.getDimension(),
                     6); // 6 corresponds to 6velocities in standard interaction matrix
  Lcomplete = const_cast<vpFeatureMoment &>(featM).interaction(vpBasicFeature::FEATURE_ALL);
  Lcomplete.matlabPrint(os);
  return os;
}

/*!
Interface function to display the moments and other interaction matrices
on which a particular vpFeatureMoment is dependent upon
Not made pure to maintain compatibility
Recommended : Types inheriting from vpFeatureMoment should implement this
function
*/
void vpFeatureMoment::printDependencies(std::ostream &os) const
{
  os << " WARNING : Falling back to base class version of "
        "printDependencies() in vpFeatureMoment. To prevent that, this has "
        "to be implemented in the derived classes!"
     << std::endl;
}
