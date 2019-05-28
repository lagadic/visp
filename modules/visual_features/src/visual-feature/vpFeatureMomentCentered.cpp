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
 * Implementation for all supported moment features.
 *
 * Authors:
 * Filip Novotny
 * Manikandan Bakthavatchalam
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#include <limits>
#include <vector>

#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenter.h>

/*!
  Default constructor
  \param moments_ : Database of moment primitives.
  \param A_ : First plane coefficient for a plane equation of the following
  type Ax+By+C=1/Z. \param B_ : Second plane coefficient for a plane equation
  of the following type Ax+By+C=1/Z. \param C_ : Third plane coefficient for a
  plane equation of the following type Ax+By+C=1/Z. \param featureMoments :
  Database of features.
*/
vpFeatureMomentCentered::vpFeatureMomentCentered(vpMomentDatabase &moments_, double A_, double B_, double C_,
                                                 vpFeatureMomentDatabase *featureMoments)
  : vpFeatureMoment(moments_, A_, B_, C_, featureMoments), order(0)
{
}

/*!
Interaction matrix corresponding to \f$ \mu_{ij} \f$ moment
\param select_one : first index (i)
\param select_two : second index (j)
\return Interaction matrix corresponding to the moment
*/
vpMatrix vpFeatureMomentCentered::interaction(unsigned int select_one, unsigned int select_two) const
{
  if (select_one + select_two > moment->getObject().getOrder())
    throw vpException(vpException::badValue, "The requested value has not "
                                             "been computed, you should "
                                             "specify a higher order.");
  return interaction_matrices[select_two * order + select_one];
}

/*!
 *   Core function for the interaction matrix computation for moment m_pq
 *   Given its dependent moment and interaction matrices, computes the
 * interaction matrix of centred moments
 */
vpMatrix vpFeatureMomentCentered::compute_Lmu_pq(const unsigned int &p, const unsigned int &q, const double &xg,
                                                 const double &yg, const vpMatrix &L_xg, const vpMatrix &L_yg,
                                                 const vpMomentBasic &m,
                                                 const vpFeatureMomentBasic &feature_moment_m) const
{
  // term1, term2 and Lterm3 (matrix) will be repeatedly computed inside the
  // innermost loop
  double term1 = 0.0;
  double term2 = 0.0;
  vpMatrix Lterm3(1, 6);

  double qcombl = 0.0;
  double pcombkqcombl = 0.0;

  double mkl = 0.0;
  vpMatrix L_mkl;

  int qml = 0;           // q-l
  double minus1pow = 0.; // (-1)^(p+q-k-l)
  double pintom = 0.;

  for (unsigned int k = 0; k <= p; ++k) {
    int pmk = (int)p - (int)k;
    double pcombk = static_cast<double>(vpMath::comb(p, k));
    for (unsigned int l = 0; l <= q; ++l) {
      qml = (int)q - (int)l;
      qcombl = static_cast<double>(vpMath::comb(q, l));
      minus1pow = pow((double)-1, (double)(pmk + qml));
      pcombkqcombl = pcombk * qcombl;
      mkl = m.get(k, l);
      pintom = pcombkqcombl * mkl;
      L_mkl = feature_moment_m.interaction(k, l);
      if (pmk > 0)
        term1 += pintom * pmk * pow(xg, pmk - 1) * pow(yg, qml) * minus1pow;
      if (qml > 0)
        term2 += pintom * qml * pow(xg, pmk) * pow(yg, qml - 1) * minus1pow;
      Lterm3 += pcombkqcombl * pow(xg, pmk) * pow(yg, qml) * L_mkl * minus1pow;
    }
  }

  // L_xg and L_yg stay constant with respect to the above loops. Lterm3 is
  // summed over
  vpMatrix L_mupq = L_xg * term1 + L_yg * term2 + Lterm3;
  return L_mupq;
}

/*!
  Interface to the interaction matrix computation for centered moments. Called
internally. Calls compute_Lmu_pq() for main computation moments (upto order-1)
Dependencies:
  Moment classes
  - vpMomentBasic
  Interaction matrix classes
  - vpMomentGravityCenter
  - vpFeatureMomentBasic
  - vpFeatureMomentGravityCenter
*/
void vpFeatureMomentCentered::compute_interaction()
{
#ifdef VISP_MOMENTS_COMBINE_MATRICES
  const vpMomentObject &momentObject = moment->getObject();
  order = momentObject.getOrder() + 1;
  interaction_matrices.resize(order * order);
  for (std::vector<vpMatrix>::iterator i = interaction_matrices.begin(); i != interaction_matrices.end(); ++i)
    i->resize(1, 6);

  bool found_moment_gravity;
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(moments.get("vpMomentGravityCenter", found_moment_gravity));
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");
  double xg = momentGravity.get()[0];
  double yg = momentGravity.get()[1];

  bool found_feature_gravity_center;
  vpFeatureMomentGravityCenter &featureMomentGravityCenter = (static_cast<vpFeatureMomentGravityCenter &>(
      featureMomentsDataBase->get("vpFeatureMomentGravityCenter", found_feature_gravity_center)));
  if (!found_feature_gravity_center)
    throw vpException(vpException::notInitialized, "vpFeatureMomentGravityCenter not found");
  vpMatrix Lxg = featureMomentGravityCenter.interaction(1 << 0);
  vpMatrix Lyg = featureMomentGravityCenter.interaction(1 << 1);

  bool found_moment_basic;
  const vpMomentBasic &momentbasic =
      static_cast<const vpMomentBasic &>(moments.get("vpMomentBasic", found_moment_basic));
  if (!found_moment_basic)
    throw vpException(vpException::notInitialized, "vpMomentBasic not found");

  bool found_featuremoment_basic;
  vpFeatureMomentBasic &featureMomentBasic = (static_cast<vpFeatureMomentBasic &>(
      featureMomentsDataBase->get("vpFeatureMomentBasic", found_featuremoment_basic)));
  if (!found_featuremoment_basic)
    throw vpException(vpException::notInitialized, "vpFeatureMomentBasic not found");

  // Calls the main compute_Lmu_pq function for moments upto order-1
  for (int i = 0; i < (int)order - 1; i++) {
    for (int j = 0; j < (int)order - 1 - i; j++) {
      interaction_matrices[(unsigned int)j * order + (unsigned int)i] =
          compute_Lmu_pq(i, j, xg, yg, Lxg, Lyg, momentbasic, featureMomentBasic);
    }
  }
#else  // #ifdef VISP_MOMENTS_COMBINE_MATRICES
  bool found_moment_centered;
  bool found_moment_gravity;

  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered)));
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(moments.get("vpMomentGravityCenter", found_moment_gravity));

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");

  int delta;
  int epsilon;
  const vpMomentObject &momentObject = moment->getObject();
  order = momentObject.getOrder() + 1;
  interaction_matrices.resize(order * order);
  for (std::vector<vpMatrix>::iterator i = interaction_matrices.begin(); i != interaction_matrices.end(); ++i)
    i->resize(1, 6);
  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    delta = 0;
    epsilon = 1;
  } else {
    delta = 1;
    epsilon = 4;
  }
  double n11 = momentCentered.get(1, 1) / momentObject.get(0, 0);
  double n20 = momentCentered.get(2, 0) / momentObject.get(0, 0);
  double n02 = momentCentered.get(0, 2) / momentObject.get(0, 0);
  double Xg = momentGravity.getXg();
  double Yg = momentGravity.getYg();
  double mu00 = momentCentered.get(0, 0);

  unsigned int VX = 0;
  unsigned int VY = 1;
  unsigned int VZ = 2;
  unsigned int WX = 3;
  unsigned int WY = 4;
  unsigned int WZ = 5;

  interaction_matrices[0][0][VX] = -(delta)*A * mu00;
  interaction_matrices[0][0][VY] = -(delta)*B * mu00;

  // Since mu10=0 and mu01=0
  // interaction_matrices[0][0][WX] =  (3*delta)*MU(0,1)+(3*delta)*Yg*mu00;
  // interaction_matrices[0][0][WY] = -(3*delta)*MU(1,0)-(3*delta)*Xg*mu00;
  // we get the simplification:
  interaction_matrices[0][0][WX] = (3 * delta) * Yg * mu00;
  interaction_matrices[0][0][WY] = -(3 * delta) * Xg * mu00;
  interaction_matrices[0][0][VZ] =
      -A * interaction_matrices[0][0][WY] + B * interaction_matrices[0][0][WX] + (2 * delta) * C * mu00;
  interaction_matrices[0][0][WZ] = 0.;

  for (int i = 1; i < (int)order - 1; i++) {
    unsigned int i_ = (unsigned int)i;
    unsigned int im1_ = i_ - 1;
    unsigned int ip1_ = i_ + 1;

    double mu_im10 = momentCentered.get(im1_, 0);
    double mu_ip10 = momentCentered.get(ip1_, 0);
    double mu_im11 = momentCentered.get(im1_, 1);
    double mu_i0 = momentCentered.get(i_, 0);
    double mu_i1 = momentCentered.get(i_, 1);

    interaction_matrices[i_][0][VX] = -(i + delta) * A * mu_i0 - (i * B * mu_im11);
    interaction_matrices[i_][0][VY] = -(delta)*B * mu_i0;

    interaction_matrices[i_][0][WX] =
        (i + 3 * delta) * mu_i1 + (i + 3 * delta) * Yg * mu_i0 + i * Xg * mu_im11 - i * epsilon * n11 * mu_im10;
    interaction_matrices[i_][0][WY] =
        -(i + 3 * delta) * mu_ip10 - (2 * i + 3 * delta) * Xg * mu_i0 + i * epsilon * n20 * mu_im10;
    interaction_matrices[i_][0][VZ] =
        -A * interaction_matrices[i_][0][WY] + B * interaction_matrices[i_][0][WX] + (i + 2 * delta) * C * mu_i0;
    interaction_matrices[i_][0][WZ] = i * mu_im11;
  }

  for (int j = 1; j < (int)order - 1; j++) {
    unsigned int j_ = (unsigned int)j;
    unsigned int jm1_ = j_ - 1;
    unsigned int jp1_ = j_ + 1;

    double mu_0jm1 = momentCentered.get(0, jm1_);
    double mu_0jp1 = momentCentered.get(0, jp1_);
    double mu_1jm1 = momentCentered.get(1, jm1_);
    double mu_0j = momentCentered.get(0, j_);
    double mu_1j = momentCentered.get(1, j_);

    interaction_matrices[j_ * order][0][VX] = -(delta)*A * mu_0j;
    interaction_matrices[j_ * order][0][VY] = -j * A * mu_1jm1 - (j + delta) * B * mu_0j;

    interaction_matrices[j_ * order][0][WX] =
        (j + 3 * delta) * mu_0jp1 + (2 * j + 3 * delta) * Yg * mu_0j - j * epsilon * n02 * mu_0jm1;
    interaction_matrices[j_ * order][0][WY] =
        -(j + 3 * delta) * mu_1j - (j + 3 * delta) * Xg * mu_0j - j * Yg * mu_1jm1 + j * epsilon * n11 * mu_0jm1;
    interaction_matrices[j_ * order][0][VZ] = -A * interaction_matrices[j_ * order][0][WY] +
                                              B * interaction_matrices[j_ * order][0][WX] + (j + 2 * delta) * C * mu_0j;
    interaction_matrices[j_ * order][0][WZ] = -j * mu_1jm1;
  }

  for (int j = 1; j < (int)order - 1; j++) {
    unsigned int j_ = (unsigned int)j;
    unsigned int jm1_ = j_ - 1;
    unsigned int jp1_ = j_ + 1;
    for (int i = 1; i < (int)order - j - 1; i++) {
      unsigned int i_ = (unsigned int)i;
      unsigned int im1_ = i_ - 1;
      unsigned int ip1_ = i_ + 1;

      double mu_ijm1 = momentCentered.get(i_, jm1_);
      double mu_ij = momentCentered.get(i_, j_);
      double mu_ijp1 = momentCentered.get(i_, jp1_);
      double mu_im1j = momentCentered.get(im1_, j_);
      double mu_im1jp1 = momentCentered.get(im1_, jp1_);
      double mu_ip1jm1 = momentCentered.get(ip1_, jm1_);
      double mu_ip1j = momentCentered.get(ip1_, j_);

      interaction_matrices[j_ * order + i_][0][VX] = -(i + delta) * A * mu_ij - i * B * mu_im1jp1;
      interaction_matrices[j_ * order + i_][0][VY] = -j * A * mu_ip1jm1 - (j + delta) * B * mu_ij;

      interaction_matrices[j_ * order + i_][0][WX] = (i + j + 3 * delta) * mu_ijp1 +
                                                     (i + 2 * j + 3 * delta) * Yg * mu_ij + i * Xg * mu_im1jp1 -
                                                     i * epsilon * n11 * mu_im1j - j * epsilon * n02 * mu_ijm1;
      interaction_matrices[j_ * order + i_][0][WY] = -(i + j + 3 * delta) * mu_ip1j -
                                                     (2 * i + j + 3 * delta) * Xg * mu_ij - j * Yg * mu_ip1jm1 +
                                                     i * epsilon * n20 * mu_im1j + j * epsilon * n11 * mu_ijm1;
      interaction_matrices[j_ * order + i_][0][VZ] = -A * interaction_matrices[j_ * order + i_][0][WY] +
                                                     B * interaction_matrices[j_ * order + i_][0][WX] +
                                                     (i + j + 2 * delta) * C * mu_ij;
      interaction_matrices[j_ * order + i_][0][WZ] = i * mu_im1jp1 - j * mu_ip1jm1;
    }
  }
#endif // #ifdef VISP_MOMENTS_COMBINE_MATRICES
}

/*!
  \relates vpFeatureMomentCentered
  Print all the interaction matrices of visual features
 */
std::ostream &operator<<(std::ostream &os, const vpFeatureMomentCentered &mu)
{
  vpTRACE(" << Ls - CENTRED MOMENTS >>");
  unsigned int order_m_1 = (unsigned int)(mu.order - 1);
  for (unsigned int i = 0; i < order_m_1; i++) {
    for (unsigned int j = 0; j < order_m_1 - i; j++) {
      os << "L_mu[" << i << "," << j << "] = ";
      mu.interaction(i, j).matlabPrint(os);
    }
  }
  return os;
}
