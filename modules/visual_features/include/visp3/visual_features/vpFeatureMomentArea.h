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
 * Definition of vpFeatureMomentArea associated to vpMomentArea
 *
 * Authors:
 * Manikandan Bakthavatchalam
 *
 *****************************************************************************/
/*!
  \file vpFeatureMomentArea.h
  \brief Implementation of the interaction matrix computation for
  vpMomentArea.
*/
#ifndef __FEATUREMOMENTAREA_H__
#define __FEATUREMOMENTAREA_H__
#include <visp3/visual_features/vpFeatureMoment.h>

class vpMomentDatabase;

/*!
  \class vpFeatureMomentArea

  \ingroup group_visual_features

  \brief Surface moment feature. Computes the interaction matrix associated
  with vpMomentArea.

*/

class VISP_EXPORT vpFeatureMomentArea : public vpFeatureMoment
{
public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param data_base : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A_ : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B_ : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C_ : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.

  */
  vpFeatureMomentArea(vpMomentDatabase &data_base, double A_, double B_, double C_,
                      vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 1)
  {
  }

  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentArea"; }
  /*!
    feature name
    */
  const char *name() const { return "vpFeatureMomentArea"; }
};
#endif
