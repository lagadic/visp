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
 * Alpha moment descriptor for in-plane orientation.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <cmath>
#include <visp3/core/vpMomentAlpha.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentGravityCenter.h>

/*!
  Empty constructor. Initializes alpha moment as a reference alpha with a
  value in \f$[-\pi/2 ; \pi/2]\f$. A default-constructed alpha moment may be
  used as a reference information for other alphas. A reference alpha is a
  class harbouring an alpha value computed for a \f$[-\pi/2 ; \pi/2]\f$ portion
  of the circle.
 */
vpMomentAlpha::vpMomentAlpha() : m_isRef(true), m_symmetric(false), m_mu3Ref(), m_alphaRef(0.), m_symmetricThreshold(1e-6) { values.resize(1); }

/*!
  Common constructor. Initializes alpha moment as a non-reference alpha with a
  value computed in \f$[-\pi ; \pi]\f$ when the object is non symmetric.
  \param mu3_ref : Vector of 3rd order centered moments corresponding to the reference alpha in the following
  order: \f$\mu_{30},\mu_{21},\mu_{12},\mu_{03}\f$.
  \param alpha_ref : Value of the reference alpha that has \e mu3_ref 3rd order moments.
  \param threshold : Threshold used to determine object symmetry along its 2 axis. The object is declared symmetric
  if all the four 3rd order centered moments \e mu3_ref have values lower than this threshold. If the object is symmetric,
  the alpha angle is commuted in [\f$[-\pi/2 ; \pi/2]\f$]. If the object is non symmetric, the alpha angle is
  commuted in [\f$[-\pi ; \pi]\f$]
*/
vpMomentAlpha::vpMomentAlpha(const std::vector<double> &mu3_ref, double alpha_ref, double threshold)
  : vpMoment(), m_isRef(false), m_symmetric(true), m_mu3Ref(mu3_ref), m_alphaRef(alpha_ref), m_symmetricThreshold(threshold)
{
  for (std::vector<double>::const_iterator it = mu3_ref.begin(); it != mu3_ref.end(); ++it) {
    if (std::fabs(*it) > m_symmetricThreshold) {
      m_symmetric = false;
      break;
    }
  }

  values.resize(1);
}

/*!
  Compute the value of the alpha-moment.
  Depends on vpMomentCentered.
 */
void vpMomentAlpha::compute()
{
  // symmetric = symmetric | this->getObject().isSymmetric();
  bool found_moment_centered;

  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(getMoments().get("vpMomentCentered", found_moment_centered)));

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");

  double alpha = 0.5 * atan2(2.0 * momentCentered.get(1, 1),  (momentCentered.get(2, 0) - momentCentered.get(0, 2)));

  std::vector<double> rotMu(4);

  if (m_isRef) {
    m_alphaRef = alpha;
  } else {
    if (! m_symmetric) {
      double r11 = cos(alpha - m_alphaRef);
      double r12 = sin(alpha - m_alphaRef);
      double r21 = -r12;
      double r22 = r11;
      unsigned int idx = 0;
      unsigned int order = 4;
      for (unsigned int c = 0; c < (order) * (order); c++) {
        unsigned int i = c % order;
        unsigned int j = c / order;

        if (i + j == 3) {
          double r11_k = 1.;
          for (unsigned int k = 0; k <= i; k++) {
            double r12_i_k = pow(r12, (int)(i - k));
            double comb_i_k = static_cast<double>(vpMath::comb(i, k));
            for (unsigned int l = 0; l <= j; l++) {
              rotMu[idx] += static_cast<double>(comb_i_k * vpMath::comb(j, l) * r11_k * pow(r21, (int)l) * r12_i_k *
                                                pow(r22, (int)(j - l)) *
                                                momentCentered.get(k + l, (unsigned int)(int)(i + j - k - l)));
            }
            r11_k *= r11;
          }
          idx++;
        }
      }

      double sum = 0.;
      bool signChange = false;
      for (unsigned int i = 0; i < 4; i++) {
        if (std::fabs(rotMu[i]) > m_symmetricThreshold &&
            std::fabs(m_mu3Ref[i]) > m_symmetricThreshold && rotMu[i] * m_mu3Ref[i] < 0) {
          signChange = true;
        }
        sum += std::fabs(rotMu[i] * m_mu3Ref[i]);
      }

      if (sum < std::numeric_limits<double>::epsilon()) { // FS: Is this test useful ?
        signChange = false;
      }

      if (signChange) {
         if (alpha < 0) {
          alpha += M_PI;
         }
        else {
          alpha -= M_PI;
         }
      }
    }
  }
  values[0] = alpha;
}

/*!
  Prints the value of the major-axis orientation in degrees and rad
 */
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentAlpha &c)
{
  os << (__FILE__) << std::endl;
  os << "Alpha = " << c.values[0] << "rad = " << vpMath::deg(c.values[0]) << "deg " << std::endl;
  return os;
}

/*!
Prints the dependencies of alpha, namely centered moments mu11, mu20 ad mu02
*/
void vpMomentAlpha::printDependencies(std::ostream &os) const
{
  os << (__FILE__) << std::endl;
  bool found_moment_centered;
  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(getMoments().get("vpMomentCentered", found_moment_centered)));
  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");

  os << "mu11 = " << momentCentered.get(1, 1) << "\t";
  os << "mu20 = " << momentCentered.get(2, 0) << "\t";
  os << "mu02 = " << momentCentered.get(0, 2) << std::endl;
}
