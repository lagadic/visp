/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpFeatureMomentArea.h
 * \brief Implementation of the interaction matrix computation for
 * vpMomentArea.
 */
#ifndef _vpFeatureMomentArea_h_
#define _vpFeatureMomentArea_h_

#include <visp3/core/vpConfig.h>
#include <visp3/visual_features/vpFeatureMoment.h>

BEGIN_VISP_NAMESPACE
class vpMomentDatabase;

/*!
 * \class vpFeatureMomentArea
 *
 * \ingroup group_visual_features
 *
 * \brief Surface moment feature. Computes the interaction matrix associated
 * with vpMomentArea.
*/
class VISP_EXPORT vpFeatureMomentArea : public vpFeatureMoment
{
public:
  /*!
   * Initializes the feature with information about the database of moment
   * primitives, the object plane and feature database.
   * \param data_base : Moment database. The database of moment primitives (first parameter) is mandatory.
   * It is used to access different moment values later used to compute the final matrix.
   * \param A_ : Plane coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
   * \param B_ : Plane coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
   * \param C_ : Plane coefficient in a \f$ A \times x+B \times y + C = \frac{1}{Z} \f$ plane.
   * \param featureMoments : Feature database.
   */
  vpFeatureMomentArea(vpMomentDatabase &data_base, double A_, double B_, double C_,
                      vpFeatureMomentDatabase *featureMoments = nullptr)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 1)
  { }

  void compute_interaction() VP_OVERRIDE;

  /*!
   * Associated moment name.
   */
  const std::string momentName() const VP_OVERRIDE
  {
    return "vpMomentArea";
  }

  /*!
   * Feature name.
   */
  const std::string name() const VP_OVERRIDE
  {
    return "vpFeatureMomentArea";
  }
};
END_VISP_NAMESPACE
#endif
