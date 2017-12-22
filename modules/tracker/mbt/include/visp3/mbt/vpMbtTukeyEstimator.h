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
 * Tukey M-estimator.
 *
 *****************************************************************************/

#ifndef __vpMbtTukeyEstimator_h_
#define __vpMbtTukeyEstimator_h_

#include <vector>
#include <visp3/core/vpColVector.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template <typename T> class VISP_EXPORT vpMbtTukeyEstimator
{
public:
  void MEstimator(const std::vector<T> &residues, std::vector<T> &weights, const T NoiseThreshold);
  void MEstimator(const vpColVector &residues, vpColVector &weights, const double NoiseThreshold);

private:
  T getMedian(std::vector<T> &vec);
  void MEstimator_impl(const std::vector<T> &residues, std::vector<T> &weights, const T NoiseThreshold);
  void MEstimator_impl_ssse3(const std::vector<T> &residues, std::vector<T> &weights, const T NoiseThreshold);
  void psiTukey(const T sig, std::vector<T> &x, std::vector<T> &weights);
  void psiTukey(const T sig, std::vector<T> &x, vpColVector &weights);

  std::vector<T> m_normres;
  std::vector<T> m_residues;
};
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS
#endif
