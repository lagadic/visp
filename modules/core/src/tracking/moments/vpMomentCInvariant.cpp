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
 * Descriptor for various invariants used to drive space roations around X and
 *Y axis.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentObject.h>

/*!
  Default constructor.
  (option to use a different calculation mode for sx and sy)
*/
vpMomentCInvariant::vpMomentCInvariant(bool flg_sxsynormalization)
  : I(16), II(4), c(4), s(4), K(0.0), cn(4), sn(4), In1(0.0), flg_sxsynormalization_(flg_sxsynormalization)
{
  values.resize(14);
}

/*!
  Computes some temporary invariants which are used internally to compute the
  final invariants. \param momentCentered : centered moments \param I :
  invariant output values
*/
void vpMomentCInvariant::computeI(const vpMomentCentered &momentCentered, std::vector<double> &I_val)
{

  double mu30 = momentCentered.get(3, 0);
  double mu30_2 = mu30 * mu30;
  double mu30_3 = mu30_2 * mu30;

  double mu03 = momentCentered.get(0, 3);
  double mu03_2 = mu03 * mu03;
  double mu03_3 = mu03 * mu03_2;

  double mu20 = momentCentered.get(2, 0);
  double mu02 = momentCentered.get(0, 2);
  double mu50 = momentCentered.get(5, 0);
  double mu32 = momentCentered.get(3, 2);
  double mu14 = momentCentered.get(1, 4);
  double mu05 = momentCentered.get(0, 5);
  double mu23 = momentCentered.get(2, 3);
  double mu41 = momentCentered.get(4, 1);
  double mu40 = momentCentered.get(4, 0);
  double mu04 = momentCentered.get(0, 4);
  double mu31 = momentCentered.get(3, 1);
  double mu13 = momentCentered.get(1, 3);
  double mu22 = momentCentered.get(2, 2);
  double mu21 = momentCentered.get(2, 1);
  double mu12 = momentCentered.get(1, 2);
  double mu11 = momentCentered.get(1, 1);

  double mu11_2 = mu11 * mu11;
  double mu12_2 = mu12 * mu12;
  double mu21_2 = mu21 * mu21;
  double mu22_2 = mu22 * mu22;
  double mu13_2 = mu13 * mu13;
  double mu31_2 = mu31 * mu31;
  double mu04_2 = mu04 * mu04;
  double mu40_2 = mu40 * mu40;
  double mu21_3 = mu21 * mu21_2;
  double mu12_3 = mu12_2 * mu12;
  double mu12_4 = mu12_3 * mu12;
  double mu21_4 = mu21_2 * mu21_2;

  // double kappa = mu30_2+mu03_2-3*mu21_2+6*mu21*mu03;        //Used in I8
  // calculation but simplified with MAPLE and found it to be wrong
  double zeta = mu20 - mu02;
  double zeta_2 = zeta * zeta;
  double omicron = (mu03_2 + 3 * mu03 * mu21 + mu30 * (mu30 + 3 * mu12));
  double omega = mu50 + 2 * mu32 + mu14;
  double nu = mu05 + 2 * mu23 + mu41;
  double ro = mu50 - 2 * mu32 - 3 * mu14;
  double gamma = mu05 - 2 * mu23 - 3 * mu41;

  double delta = mu50 - 10 * mu32 + 5 * mu14;
  double phi = mu05 - 10 * mu23 + 5 * mu41;
  double omega_2 = omega * omega;
  double nu_2 = nu * nu;
  double ro_2 = ro * ro;
  double gamma_2 = gamma * gamma;
  double delta_2 = delta * delta;
  double phi_2 = phi * phi;

  I_val[1] = -mu20 * mu02 + mu11_2;
  I_val[2] = zeta_2 + 4 * mu11_2;
  I_val[3] = (mu30 - 3 * mu12) * (mu30 - 3 * mu12) + (mu03 - 3 * mu21) * (mu03 - 3 * mu21);
  I_val[4] = (mu30 + mu12) * (mu30 + mu12) + (mu21 + mu03) * (mu21 + mu03);
  I_val[5] = -mu30_2 * mu03_2 + (-4 * mu12_3 + 6 * mu21 * mu12 * mu03) * mu30 - 4 * mu21_3 * mu03 + 3 * mu21_2 * mu12_2;
  I_val[6] = 3 * mu12_4 + 2 * mu30 * mu12_3 + (3 * mu30_2 - 6 * mu03 * mu21) * mu12_2 -
             6 * mu30 * mu21 * (mu21 + mu03) * mu12 + 2 * mu30_2 * mu03_2 + 2 * mu21_3 * mu03 + 3 * mu21_2 * mu03_2 +
             3 * mu21_4;
  I_val[7] = (3 * mu21 + 2 * mu03) * mu12_3 + 3 * mu30 * (mu03 + 2 * mu21) * mu12_2 -
             3 * mu21 * (mu30 + mu03 + mu21) * (-mu30 + mu03 + mu21) * mu12 +
             mu30 * (-mu30_2 * mu03 - 2 * mu21_3 - 3 * mu03 * mu21_2 + mu03_3);
  // I_val[8]=3*mu21_4-3*mu21_3*mu03+(3*mu03_2+kappa-6*mu12_2)*mu21_2-mu03*(-15*mu12_2+kappa)*mu21-(-3*mu12_2*mu30+(2*kappa-3*mu03_2)*mu12+kappa*mu30)*mu12;
  I_val[8] = 3 * mu03 * mu21_3 - 2 * mu03_2 * mu21_2 + mu21_2 * mu30_2 + 3 * mu12_2 * mu03 * mu21 -
             mu03 * mu21 * mu30_2 - mu03_3 * mu21 + 3 * mu12_3 * mu30 - 2 * mu12_2 * mu30_2 + mu12_2 * mu03_2 -
             mu12 * mu30_3 - mu12 * mu30 * mu03_2 + 3 * mu12 * mu30 * mu21_2 - 6 * mu12 * mu30 * mu03 * mu21;
  I_val[9] = omicron * omicron;

  I_val[10] = mu40 * mu04 - 4 * mu31 * mu13 + 3 * mu22_2;
  I_val[11] = 3 * mu13_2 + 2 * mu31 * mu13 + (-3 * mu40 - 3 * mu04) * mu22 - 2 * mu40 * mu04 + 3 * mu31_2;
  I_val[12] = 3 * mu04_2 + (2 * mu40 + 12 * mu22) * mu04 + 3 * mu40_2 + 12 * mu40 * mu22 + 16 * mu31 * mu13;
  I_val[13] = omega_2 + nu_2;
  I_val[14] = ro_2 + gamma_2;
  I_val[15] = delta_2 + phi_2;

  double a;
  if (getObject().getType() == vpMomentObject::DISCRETE)
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
  else
    a = getObject().get(0, 0);

  c[1] = momentCentered.get(2, 0) - momentCentered.get(0, 2);
  s[1] = 2 * momentCentered.get(1, 1);
  c[2] = momentCentered.get(0, 3) - 3 * momentCentered.get(2, 1);
  s[2] = momentCentered.get(3, 0) - 3 * momentCentered.get(1, 2);
  c[3] = c[1] * c[1] - s[1] * s[1];
  s[3] = 2 * s[1] * c[1];

  II[1] = c[1] * c[1] + s[1] * s[1];
  II[2] = c[2] * c[2] + s[2] * s[2];
  II[3] = momentCentered.get(2, 0) + momentCentered.get(0, 2);

  K = (II[1] * (II[3] * sqrt(std::fabs(II[3])))) / sqrt(std::fabs(a));

  /*
   * Intermediate quantities required for calculation of normalized version of
   * Sx and Sy The pij doubles below are the respective centered moment values
   * mu_ij scaled by mu20 + mu02
   */
  double p20 = momentCentered.get(2, 0) / II[3]; // II[3] is the normalization factor for the 2nd order moments
  double p11 = momentCentered.get(1, 1) / II[3];
  double p02 = momentCentered.get(0, 2) / II[3];

  double d =
      sqrt(std::fabs(a)) / (II[3] * sqrt(std::fabs(II[3]))); // d is the normalization factor for 3rd order moments
  double p30 = momentCentered.get(3, 0) * d;
  double p21 = momentCentered.get(2, 1) * d;
  double p12 = momentCentered.get(1, 2) * d;
  double p03 = momentCentered.get(0, 3) * d;

  cn[1] = p20 - p02;
  sn[1] = 2.0 * p11;
  sn[2] = p30 - 3.0 * p12;
  cn[2] = p03 - 3.0 * p21;

  cn[3] = cn[1] * cn[1] - sn[1] * sn[1];
  sn[3] = 2.0 * sn[1] * cn[1];

  In1 = cn[1] * cn[1] + sn[1] * sn[1];
}

/*!
  Computes translation-plane-rotation-scale invariants.
  Depends on vpMomentCentered.
  All possible invariants are computed here. The selection of the invariant is
  done afterwards.
*/
void vpMomentCInvariant::compute()
{
  if (getObject().getOrder() < 5)
    throw vpException(vpException::notInitialized, "Order is not high enough for vpMomentCInvariant. "
                                                   "Specify at least order 5.");
  bool found_moment_centered;
  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(getMoments().get("vpMomentCentered", found_moment_centered)));

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");

  computeI(momentCentered, I);
  double II3_2 = II[3] * II[3];
  double II3_3 = II3_2 * II[3];

  double a;
  if (getObject().getType() == vpMomentObject::DISCRETE)
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
  else
    a = getObject().get(0, 0);

  values[0] = I[1] / I[2];
  values[1] = I[3] / I[4];

  values[2] = I[5] / I[6];

  values[3] = I[7] / I[6];

  values[4] = I[8] / I[6];

  values[5] = I[9] / I[6];

  values[6] = I[11] / I[10];

  values[7] = I[12] / I[10];

  values[8] = I[13] / I[15];

  values[9] = I[14] / I[15];

  if (flg_sxsynormalization_)
    calcSxSyNormalized(values[10], values[11]);
  else
    calcSxSy(values[10], values[11]);

  values[12] = II[1] / (II3_2);     // Px
  values[13] = a * II[2] / (II3_3); // Py
}

/*!
   Sx and Sy as it was inside compute()
 */
void vpMomentCInvariant::calcSxSy(double &sx, double &sy) const
{
  sx = (c[2] * c[3] + s[2] * s[3]) / K;
  sy = (s[2] * c[3] - c[2] * s[3]) / K;
}

/*!
 * Sx and Sy from normalized 2nd and 3rd order moments
 * Numerically better (than in the usual Sx,Sy when K appears in the
 * denominator)
 */
void vpMomentCInvariant::calcSxSyNormalized(double &sx, double &sy) const
{
  sx = (cn[2] * cn[3] + sn[2] * sn[3]) / In1;
  sy = (sn[2] * cn[3] - cn[2] * sn[3]) / In1;
}

/*!
  Prints the temporary invariants.  Used for debug purposes only
  \param index : index of the temporary invariant
*/
void vpMomentCInvariant::printI(unsigned int index) { std::cout << "I(" << index << ")=" << I[index] << std::endl; }

/*!
  Print out all invariants that were computed
  There are 15 of them, as in [Point-based and region based.ITRO05]
  \cite Tahri05z
 */
void vpMomentCInvariant::printInvariants(std::ostream &os) const
{
  for (unsigned int i = 1; i < I.size(); ++i) { // i = 1 since vector I has been indexed from 1 in
                                                // vpMomentCinvariant
    os << "I[" << i << "]=" << I[i] << std::endl;
  }
  os << std::endl;
}

/*!
  Outputs the moment's values to a stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentCInvariant &c)
{
  for (unsigned int i = 0; i < c.values.size(); i++) {
    os << c.values[i] << "," << std::endl;
  }
  return os;
}
