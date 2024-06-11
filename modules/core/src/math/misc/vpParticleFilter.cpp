/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Hinkley's cumulative sum test implementation.
 *
*****************************************************************************/

/*!

  \file vpParticleFilter.cpp

  \brief Definition of the vpParticleFilter class corresponding to the implementation
  of a particle filter.

*/

#include <visp3/core/vpParticleFilter.h>

BEGIN_VISP_NAMESPACE

vpColVector vpParticleFilter::simpleMean(const std::vector<vpColVector> &particles, const std::vector<double> &weights)
{
  size_t nbParticles = particles.size();
  if (nbParticles == 0) {
    throw(vpException(vpException::dimensionError, "No particles to add when computing the mean"));
  }
  vpColVector res = particles[0] * weights[0];
  for (size_t i = 1; i < nbParticles; ++i) {
    res += particles[i] * weights[i];
  }
  return res;
}

bool vpParticleFilter::simpleResamplingCheck(const unsigned int &N, const std::vector<double> &weights)
{
  double sumWeights = 0.;
  double sumSquare = 0.;
  for (unsigned int i = 0; i < N; ++i) {
    sumWeights += weights[i];
    sumSquare += weights[i] * weights[i];
  }
  double temp = (2 * sumWeights * sumWeights) / sumSquare;
  double N_eff = 1.0 / temp;
  return N_eff < N / 2.0;
}

std::pair<std::vector<vpColVector>, std::vector<double>> vpParticleFilter::simpleImportanceResampling(const std::vector<vpColVector> &particles, const std::vector<double> &weights)
{
  static vpUniRand sampler(vpTime::measureTimeMicros());
  unsigned int nbParticles = particles.size();
  double x = 0.;
  double sumWeights = 0.;
  std::vector<int> idx(nbParticles);

  // Draw indices of the randomly chosen particles from the vector of particles
  for (unsigned int i = 0; i < nbParticles; ++i) {
    x = sampler();
    sumWeights = 0.0;
    for (unsigned int j = 0; j < nbParticles; ++j) {
      if (x < sumWeights + weights[j]) {
        idx[i] = j;
        break;
      }
      sumWeights += weights[j];
    }
  }

  // Draw the randomly chosen particles corresponding to the indices
  std::pair<std::vector<vpColVector>, std::vector<double>> newParticlesWeights;
  newParticlesWeights.first.resize(nbParticles);
  for (unsigned int i = 0; i < nbParticles; ++i) {
    newParticlesWeights.first[i] = particles[idx[i]];
  }

  // Reinitialize the weights
  newParticlesWeights.second.resize(nbParticles, 1.0/ static_cast<double>(nbParticles));
  return newParticlesWeights;
}

void vpParticleFilter::predictMonothread(const double &dt, const vpColVector &u)
{
  for (unsigned int i = 0; i < m_N; ++i) {
    if (m_useCommandStateFunction) {
      m_particles[i] = m_bx(u, m_particles[i], dt);
    }
    else if (m_useProcessFunction) {
      m_particles[i] = m_f(m_particles[i], dt);
    }
    else {
      throw(vpException(vpException::notInitialized, "vpParticleFilter has not been initialized before calling predict"));
    }
  }
}

void vpParticleFilter::updateMonothread(const vpColVector &z)
{
  for (unsigned int i = 0; i < m_N; ++i) {
    m_w[i] = m_likelihood(m_particles[i], z);
  }
}
END_VISP_NAMESPACE
