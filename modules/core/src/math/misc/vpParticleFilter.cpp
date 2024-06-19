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

vpUniRand vpParticleFilter::sampler;
vpUniRand vpParticleFilter::samplerRandomIdx;

vpParticleFilter::vpParticleFilter(const unsigned int &N, const std::vector<double> &stdev, const long &seed, const int &nbThreads)
  : m_N(N)
  , m_particles(N)
  , m_w(N, 1./static_cast<double>(N))
  , m_useProcessFunction(false)
  , m_useCommandStateFunction(false)
{
#ifndef VISP_HAVE_OPENMP
  m_nbMaxThreads = 1;
  if (nbThreads > 1) {
    std::cout << "[vpParticleFilter::vpParticleFilter] WARNING: OpenMP is not available, maximum number of threads to use clamped to 1" << std::endl;
  }
#else
  int maxThreads = omp_get_max_threads();
  if (nbThreads <= 0) {
    m_nbMaxThreads = maxThreads;
  }
  else if (nbThreads > maxThreads) {
    m_nbMaxThreads = maxThreads;
    std::cout << "[vpParticleFilter::vpParticleFilter] WARNING: maximum number of threads to use clamped to "
      << maxThreads << " instead of " << nbThreads << " due to OpenMP restrictions." << std::endl;
    std::cout << "[vpParticleFilter::vpParticleFilter] If you want more, consider to use omp_set_num_threads before." << std::endl;
  }
  else {
    m_nbMaxThreads = nbThreads;
  }
#endif
  // Generating the random generators
  unsigned int sizeState = stdev.size();
  m_noiseGenerators.resize(m_nbMaxThreads);
  unsigned long long seedForGenerator;
  if (seed > 0) {
    seedForGenerator = seed;
  }
  else {
    seedForGenerator = vpTime::measureTimeMicros();
  }

  // Sampler for the simpleImportanceResampling method
  sampler.setSeed(seed, 0x123465789ULL);
  samplerRandomIdx.setSeed(seed + 4224, 0x123465789ULL);

  vpUniRand seedGenerator(seedForGenerator);
  for (unsigned int threadId = 0; threadId < m_nbMaxThreads; ++threadId) {
    for (unsigned int stateId = 0; stateId < sizeState; ++stateId) {
      m_noiseGenerators[threadId].push_back(vpGaussRand(stdev[stateId], 0., seedGenerator.uniform(0., 1e9)));
    }
  }
}

void vpParticleFilter::init(const vpColVector &x0, const vpProcessFunction &f,
            const vpLikelihoodFunction &l,
            const vpResamplingConditionFunction &checkResamplingFunc, const vpResamplingFunction &resamplingFunc,
            const vpFilterFunction &filterFunc, const vpStateAddFunction &addFunc)
{
  m_f = f;
  m_stateFilterFunc = filterFunc;
  m_likelihood = l;
  m_checkIfResample = checkResamplingFunc;
  m_resampling = resamplingFunc;
  m_stateAdd = addFunc;
  m_useProcessFunction = true;
  m_useCommandStateFunction = false;

  // Initialize the different particles
  initParticles(x0);
}

void vpParticleFilter::init(const vpColVector &x0, const vpCommandStateFunction &bx,
            const vpLikelihoodFunction &l,
            const vpResamplingConditionFunction &checkResamplingFunc, const vpResamplingFunction &resamplingFunc,
            const vpFilterFunction &filterFunc, const vpStateAddFunction &addFunc)
{
  m_bx = bx;
  m_stateFilterFunc = filterFunc;
  m_likelihood = l;
  m_checkIfResample = checkResamplingFunc;
  m_resampling = resamplingFunc;
  m_stateAdd = addFunc;
  m_useProcessFunction = false;
  m_useCommandStateFunction = true;

  // Initialize the different particles
  initParticles(x0);
}

void vpParticleFilter::filter(const vpColVector &z, const double &dt, const vpColVector &u)
{
  predict(dt, u);
  update(z);
}

void vpParticleFilter::predict(const double &dt, const vpColVector &u)
{
  if (m_nbMaxThreads == 1) {
    predictMonothread(dt, u);
  }
#ifdef VISP_HAVE_OPENMP
  else {
    predictMultithread(dt, u);
  }
#endif
}

void vpParticleFilter::update(const vpColVector &z)
{
  if (m_nbMaxThreads == 1) {
    updateMonothread(z);
  }
#ifdef VISP_HAVE_OPENMP
  else {
    updateMultithread(z);
  }
#endif
  bool shouldResample = m_checkIfResample(m_N, m_w);
  if (shouldResample) {
    vpParticlesWithWeights particles_weights = m_resampling(m_particles, m_w);
    m_particles = std::move(particles_weights.m_particles);
    m_w = std::move(particles_weights.m_weights);
  }
}

vpColVector vpParticleFilter::computeFilteredState()
{
  return m_stateFilterFunc(m_particles, m_w, m_stateAdd);
}

vpColVector vpParticleFilter::weightedMean(const std::vector<vpColVector> &particles, const std::vector<double> &weights, const vpStateAddFunction &addFunc)
{
  size_t nbParticles = particles.size();
  if (nbParticles == 0) {
    throw(vpException(vpException::dimensionError, "No particles to add when computing the mean"));
  }
  vpColVector res = particles[0] * weights[0];
  for (size_t i = 1; i < nbParticles; ++i) {
    res = addFunc(res, particles[i] * weights[i]);
  }
  return res;
}

bool vpParticleFilter::simpleResamplingCheck(const unsigned int &N, const std::vector<double> &weights)
{
  double sumSquare = 0.;
  for (unsigned int i = 0; i < N; ++i) {
    sumSquare += weights[i] * weights[i];
  }
  if (sumSquare < std::numeric_limits<double>::epsilon()) {
    // All the particles diverged
    return true;
  }
  double N_eff = 1.0 / sumSquare;
  return (N_eff < (N / 2.0));
}

vpParticleFilter::vpParticlesWithWeights vpParticleFilter::simpleImportanceResampling(const std::vector<vpColVector> &particles, const std::vector<double> &weights)
{
  unsigned int nbParticles = particles.size();
  double x = 0.;
  double sumWeights = 0.;
  std::vector<int> idx(nbParticles);

  // Draw indices of the randomly chosen particles from the vector of particles
  for (unsigned int i = 0; i < nbParticles; ++i) {
    x = sampler();
    sumWeights = 0.0;
    int index = samplerRandomIdx.uniform(0, nbParticles); // In case all the weights are null
    for (unsigned int j = 0; j < nbParticles; ++j) {
      if (x < sumWeights + weights[j]) {
        index = j;
        break;
      }
      sumWeights += weights[j];
    }
    idx[i] = index;
  }

  // Draw the randomly chosen particles corresponding to the indices
  vpParticlesWithWeights newParticlesWeights;
  newParticlesWeights.m_particles.resize(nbParticles);
  for (unsigned int i = 0; i < nbParticles; ++i) {
    newParticlesWeights.m_particles[i] = particles[idx[i]];
  }

  // Reinitialize the weights
  newParticlesWeights.m_weights.resize(nbParticles, 1.0/ static_cast<double>(nbParticles));
  return newParticlesWeights;
}

void vpParticleFilter::initParticles(const vpColVector &x0)
{
  unsigned int sizeState = x0.size();
  unsigned int chunkSize = m_N / m_nbMaxThreads;
  double uniformWeight = 1. / static_cast<double>(m_N);
  for (unsigned int i = 0; i < m_nbMaxThreads; ++i) {
    unsigned int idStart = chunkSize * i;
    unsigned int idStop = chunkSize * (i + 1);
    // Last chunk must go until the end
    if (i == m_nbMaxThreads - 1) {
      idStop = m_N;
    }
    for (unsigned int id = idStart; id < idStop; ++id) {
      // Generating noise
      vpColVector noise(sizeState);
      for (unsigned int idState = 0; idState < sizeState; ++idState) {
        noise[idState] = m_noiseGenerators[i][idState]();
      }
      // Adding noise to the initial state
      m_particles[id] = m_stateAdd(x0, noise);

      // (Re)initializing its weight
      m_w[id] = uniformWeight;
    }
  }
}

#ifdef VISP_HAVE_OPENMP
void vpParticleFilter::predictMultithread(const double &dt, const vpColVector &u)
{
  int iam, nt, ipoints, istart, npoints(m_N);
  unsigned int sizeState = m_particles[0].size();
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    ipoints = npoints / nt;
    // size of partition
    istart = iam * ipoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      ipoints = npoints - istart;
    }

    for (int i = istart; i< istart + ipoints; ++i) {
      // Updating the particles following the process (or command) function
      if (m_useCommandStateFunction) {
        m_particles[i] = m_bx(u, m_particles[i], dt);
      }
      else if (m_useProcessFunction) {
        m_particles[i] = m_f(m_particles[i], dt);
      }

      // Generating noise to add to the particle
      vpColVector noise(sizeState);
      for (unsigned int j = 0; j < sizeState; ++j) {
        noise[j] = m_noiseGenerators[iam][j]();
      }

      // Adding the noise to the particle
      m_particles[i] = m_stateAdd(m_particles[i], noise);
    }
  }
}

double threadLikelihood(const vpParticleFilter::vpLikelihoodFunction &likelihood, const std::vector<vpColVector> &v_particles,
                        const vpColVector &z, std::vector<double> &w, const int &istart, const int &ipoints)
{
  double sum(0.0);
  for (int i = istart; i< istart + ipoints; ++i) {
    w[i] = w[i] * likelihood(v_particles[i], z);
    sum += w[i];
  }
  return sum;
}

void vpParticleFilter::updateMultithread(const vpColVector &z)
{
  double sumWeights = 0.0;
  int iam, nt, ipoints, istart, npoints(m_N);
  vpColVector tempSums(m_nbMaxThreads, 0.0);
  // Compute the weights depending on the likelihood of a particle with regard to the measurements
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart)
  {
    iam = omp_get_thread_num();
    nt = omp_get_num_threads();
    ipoints = npoints / nt;
    // size of partition
    istart = iam * ipoints; // starting array index
    if (iam == nt-1) {
      // last thread may do more
      ipoints = npoints - istart;
    }
    tempSums[iam] = threadLikelihood(m_likelihood, m_particles, z, m_w, istart, ipoints);
  }
  sumWeights = tempSums.sum();

  if (sumWeights > std::numeric_limits<double>::epsilon()) {
#pragma omp parallel default(shared) private(iam, nt, ipoints, istart)
    {
      iam = omp_get_thread_num();
      nt = omp_get_num_threads();
      ipoints = npoints / nt;
      // size of partition
      istart = iam * ipoints; // starting array index
      if (iam == nt-1) {
        // last thread may do more
        ipoints = npoints - istart;
      }

      // Normalize the weights
      for (int i = istart; i < istart + ipoints; ++i) {
        m_w[i] = m_w[i] / sumWeights;
      }
    }
  }
}
#endif

void vpParticleFilter::predictMonothread(const double &dt, const vpColVector &u)
{
  unsigned int sizeState = m_particles[0].size();
  for (unsigned int i = 0; i < m_N; ++i) {
    // Updating the particle following the process (or command) function
    if (m_useCommandStateFunction) {
      m_particles[i] = m_bx(u, m_particles[i], dt);
    }
    else if (m_useProcessFunction) {
      m_particles[i] = m_f(m_particles[i], dt);
    }
    else {
      throw(vpException(vpException::notInitialized, "vpParticleFilter has not been initialized before calling predict"));
    }

    // Generating noise to add to the particle
    vpColVector noise(sizeState);
    for (unsigned int j = 0; j < sizeState; ++j) {
      noise[j] = m_noiseGenerators[0][j]();
    }

    // Adding the noise to the particle
    m_particles[i] = m_stateAdd(m_particles[i], noise);
  }
}

void vpParticleFilter::updateMonothread(const vpColVector &z)
{
  double sumWeights = 0.;
  // Compute the weights depending on the likelihood of a particle with regard to the measurements
  for (unsigned int i = 0; i < m_N; ++i) {
    m_w[i] = m_w[i] * m_likelihood(m_particles[i], z);
    sumWeights += m_w[i];
  }

  // Normalize the weights
  if (sumWeights > std::numeric_limits<double>::epsilon()) {
    for (unsigned int i = 0; i < m_N; ++i) {
      m_w[i] = m_w[i] / sumWeights;
    }
  }
}
END_VISP_NAMESPACE
