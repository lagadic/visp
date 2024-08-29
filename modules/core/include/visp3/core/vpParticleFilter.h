/*
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
 * Display a point cloud using PCL library.
 */

#ifndef VP_PARTICLE_FILTER_H
#define VP_PARTICLE_FILTER_H

#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpMatrix.h>

#include <functional> // std::function

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpParticleFilter
  \tparam MeasurementsType The class that corresponds to the measurements used to compute
  the weights of the Particle Filter
  \brief The class permits to use a Particle Filter.
*/
template <typename MeasurementsType>
class vpParticleFilter
{
public:
  /**
   * \brief Structure of vectors for which the i^th element of the weights vector is
   * associated to the i^th element of the particles vectors.
   */
  typedef struct vpParticlesWithWeights
  {
    std::vector<vpColVector> m_particles; /*!< Particles vector*/
    std::vector<double> m_weights; /*!< Weights vector.*/
  } vpParticlesWithWeights;

  /**
   * \brief Function that computes either the equivalent of an addition in the state space.
   * The first argument is the vector to which we must add something
   * and the second argument is the thing to be added. The return is the
   * result of this operation.
   */
  typedef std::function<vpColVector(const vpColVector &, const vpColVector &)> vpStateAddFunction;

  /**
   * \brief Process model function, which projects a particle forward in time.
   * The first argument is a particle, the second is the period and the return is the
   * particle projected in the future.
   */
  typedef std::function<vpColVector(const vpColVector &, const double &)> vpProcessFunction;

  /**
   * \brief Command model function, which projects a particle forward in time according to
   * the command and its previous state.
   * The first argument is the command(s), the second is the particle and the third is the period.
   * The return is the updated particle after period seconds.
   */
  typedef std::function<vpColVector(const vpColVector &, const vpColVector &, const double &)> vpCommandStateFunction;

  /**
   * \brief Likelihood function, which evaluates the likelihood of a particle with regard to the measurements.
   * The first argument is the particle that is evaluated.
   * The second argument is the measurements vector.
   * The return is the likelihood of the particle, which equals to 0 when the particle does not match
   * at all the measurements and to 1 when it matches completely.
   */
  typedef std::function<double(const vpColVector &, const MeasurementsType &)> vpLikelihoodFunction;

  /**
   * \brief Filter function, which computes the filtered state of the particle filter.
   * The first argument is the vector containing the particles,
   * the second argument is the associated vector of weights and the third argument is a function
   * to perform an addition in the state space.
   * The return is the corresponding filtered state.
   */
  typedef std::function<vpColVector(const std::vector<vpColVector> &, const std::vector<double> &, const vpStateAddFunction &)> vpFilterFunction;

  /**
   * \brief Function that takes as argument the number of particles and the vector of weights
   * associated to each particle and returns true if the resampling must be performed and false
   * otherwise.
   */
  typedef std::function<bool(const unsigned int &, const std::vector<double> &)> vpResamplingConditionFunction;

  /**
   * \brief Function that takes as argument the vector of particles and the vector of
   * associated weights. It returns a pair < new_vector_particles, new_weights >.
   */
  typedef std::function<vpParticlesWithWeights(const std::vector<vpColVector> &, const std::vector<double> &)> vpResamplingFunction;

  /**
   * \brief Construct a new vpParticleFilter object.
   *
   * \param[in] N The number of particles.
   * \param[in] stdev The standard deviations of the noise, each item correspond to one component of the state.
   * \param[in] seed The seed to use to create the noise generators. A negative value makes the seed to
   * be based on the current time.
   * \param[in] nbThreads The number of threads the user would like to use. Negative value to set the maximum number
   * of threads available when using OpenMP.
   */
  VP_EXPLICIT vpParticleFilter(const unsigned int &N, const std::vector<double> &stdev, const long &seed = -1, const int &nbThreads = -1);

  inline virtual ~vpParticleFilter() { }

  /**
   * \brief Set the guess of the initial state.
   *
   * \param[in] x0 Guess of the initial state.
   * \param[in] f Process model function, which projects the particles forward in time.
   * The first argument is a particle, the second is the period and the return is the
   * particle projected in the future.
   * \param[in] l Likelihood function, that evaluates how much a particle matches the measurements.
   * 0 means that the particle does not match the measurement at all and 1 that it maches completely.
   * \param[in] checkResamplingFunc The function that returns true when the filter starts to degenerate
   * and false otherwise.
   * \param[in] resamplingFunc The resampling function that generate new particles and associated weights
   * when the filter starts to degenerate.
   * \param[in] filterFunc The function to compute the filtered state from the particles and their weights.
   * \param[in] addFunc The function that permits to perform an addition in the state space.
   */
  void init(const vpColVector &x0, const vpProcessFunction &f,
            const vpLikelihoodFunction &l,
            const vpResamplingConditionFunction &checkResamplingFunc, const vpResamplingFunction &resamplingFunc,
            const vpFilterFunction &filterFunc = weightedMean,
            const vpStateAddFunction &addFunc = simpleAdd);

  /**
   * \brief Set the guess of the initial state.
   *
   * \param[in] x0 Guess of the initial state.
   * \param[in] bx Process model function, which projects the particles forward in time based on the previous
   * state and on the input commands. The first argument is the command vector, the second is a particle and
   * the last is the period. The return is the particle projected in the future.
   * \param[in] l Likelihood function, that evaluates how much a particle matches the measurements.
   * 0 means that the particle does not match the measurement at all and 1 that it maches completely.
   * \param[in] checkResamplingFunc The function that returns true when the filter starts to degenerate
   * and false otherwise.
   * \param[in] resamplingFunc The resampling function that generate new particles and associated weights
   * when the filter starts to degenerate.
   * \param[in] filterFunc The function to compute the filtered state from the particles and their weights.
   * \param[in] addFunc The function that permits to perform an addition in the state space.
   */
  void init(const vpColVector &x0, const vpCommandStateFunction &bx,
            const vpLikelihoodFunction &l,
            const vpResamplingConditionFunction &checkResamplingFunc, const vpResamplingFunction &resamplingFunc,
            const vpFilterFunction &filterFunc = weightedMean,
            const vpStateAddFunction &addFunc = simpleAdd);

  /**
   * \brief Perform first the prediction step and then the update step.
   * If needed, resampling will also be performed.
   *
   * \param[in] z The new measurement.
   * \param[in] dt The time in the future we must predict.
   * \param[in] u The command(s) given to the system, if the impact of the system is known.
   *
   * \warning To use the commands, use the dedicated constructor or call
   * vpUnscentedKalman::setCommandStateFunction beforehand. In the second case, the process
   * function will be ignored.
   */
  void filter(const MeasurementsType &z, const double &dt, const vpColVector &u = vpColVector());

  /**
   * \brief Predict the new state based on the last state and how far in time we want to predict.
   *
   * \param[in] dt The time in the future we must predict.
   * \param[in] u The command(s) given to the system, if the impact of the system is known.
   *
   * \warning To use the commands, use the dedicated constructor or call
   * vpUnscentedKalman::setCommandStateFunction beforehand. In the second case, the process
   * function will be ignored.
   */
  void predict(const double &dt, const vpColVector &u = vpColVector());

  /**
   * \brief Update the weights of the particles based on a new measurement.
   * The weights will be normalized (i.e. each weight will be divided by the sum of the weights).
   *
   * \param[in] z The measurements at the current timestep.
   */
  void update(const MeasurementsType &z);

  /**
   * \brief Compute the filtered state from the particles and their associated weights.
   *
   * \return vpColVector The filtered state.
   */
  vpColVector computeFilteredState();

  /**
   * \brief Set the process function to use when projecting the particles in the future.
   *
   * \param f The process function to use.
   *
   * \warning It will deactive the command function.
   */
  inline void setProcessFunction(const vpProcessFunction &f)
  {
    m_f = f;
    m_useCommandStateFunction = false;
    m_useProcessFunction = true;
  }

  /**
   * \brief Set the command function to use when projecting the particles in the future.
   *
   * \param bx The command function to use.
   *
   * \warning It will deactivate the process function.
   */
  inline void setCommandStateFunction(const vpCommandStateFunction &bx)
  {
    m_bx = bx;
    m_useCommandStateFunction = true;
    m_useProcessFunction = false;
  }

  /**
   * \brief Set the likelihood function that updates the weights of the particles
   * based on the new measurements.
   *
   * \param likelihood The likelihood function.
   */
  inline void setLikelihoodFunction(const vpLikelihoodFunction &likelihood)
  {
    m_likelihood = likelihood;
  }

  /**
   * \brief Set the filter function that compute the filtered state from the particles
   * and their associated weights.
   *
   * \param filterFunc The filtering function to use.
   */
  inline void setFilterFunction(const vpFilterFunction &filterFunc)
  {
    m_stateFilterFunc = filterFunc;
  }

  /**
   * \brief Set the function that returns true when the filter starts to degenerate
   * and false otherwise.
   *
   * \param resamplingCondFunc The evaluation function to use.
   */
  inline void setCheckResamplingFunction(const vpResamplingConditionFunction &resamplingCondFunc)
  {
    m_checkIfResample = resamplingCondFunc;
  }

  /**
   * \brief Set the resampling function that generate new particles and associated weights
   * when the filter starts to degenerate.
   *
   * \param resamplingFunc The resampling function to use.
   */
  inline void setResamplingFunction(const vpResamplingFunction &resamplingFunc)
  {
    m_resampling = resamplingFunc;
  }

  /**
   * \brief Simple function to compute an addition, which just does \f$ \textbf{res} = \textbf{a} + \textbf{toAdd} \f$
   *
   * \param[in] a Vector to which we must add something.
   * \param[in] toAdd The something we must add to \b a .
   * \return vpColVector \f$ \textbf{res} = \textbf{a} + \textbf{toAdd} \f$
   */
  inline static vpColVector simpleAdd(const vpColVector &a, const vpColVector &toAdd)
  {
    vpColVector res = a + toAdd;
    return res;
  }

  /**
   * \brief Simple function to compute a weighted mean, which just does
   * \f$ \textbf{res} = \sum^{N-1}_{i=0} weights[i] \textbf{particles}[i] \f$
   *
   * \param[in] particles Vector that contains all the particles.
   * \param[in] weights Vector that contains the weights associated to the particles.
   * \param[in] addFunc How to perform the addition.
   * \return vpColVector \f$ \textbf{res} = \sum^{N-1}_{i=0} weights[i] \textbf{particles}[i] \f$
   */
  static vpColVector weightedMean(const std::vector<vpColVector> &particles, const std::vector<double> &weights, const vpStateAddFunction &addFunc);

  /**
   * \brief Returns true if the following condition is fulfilled, or if all the particles diverged:
   * \f$ \frac{2}{\sum_i (\frac{w_i}{\sum_j w_j})^2} < N \f$
   *
   * \param[in] N The number of particles.
   * \param[in] weights The weights associated to each particle.
   * \return true Resampling must be performed.
   * \return false Resampling is not needed.
   */
  static bool simpleResamplingCheck(const unsigned int &N, const std::vector<double> &weights);

  /**
   * \brief Function implementing the resampling of a Simple Importance Resampling Particle Filter.
   *
   * \param[in] particles Vector containing the particles.
   * \param[in] weights Vector containing the associated weights.
   * \return vpParticlesWithWeights A pair of vector of particles and
   * vector of associated weights.
   */
  static vpParticlesWithWeights simpleImportanceResampling(const std::vector<vpColVector> &particles, const std::vector<double> &weights);

private:
  void initParticles(const vpColVector &x0);
#ifdef VISP_HAVE_OPENMP
  void predictMultithread(const double &dt, const vpColVector &u);
  void updateMultithread(const MeasurementsType &z);
#endif

  void predictMonothread(const double &dt, const vpColVector &u);
  void updateMonothread(const MeasurementsType &z);

  static vpUniRand sampler;
  static vpUniRand samplerRandomIdx;

  unsigned int m_N; /*!< Number of particles.*/
  unsigned int m_nbMaxThreads; /*!< Maximum number of threads to use.*/
  std::vector<std::vector<vpGaussRand>> m_noiseGenerators; /*!< The noise generator adding noise to the particles at each time step.*/
  std::vector<vpColVector> m_particles; /*!< The particles.*/
  std::vector<double> m_w; /*!< The weights associated to each particles.*/

  vpColVector m_Xest; /*!< The estimated (i.e. filtered) state variables.*/

  vpProcessFunction m_f; /*!< Process model function, which projects the sigma points forward in time.*/
  vpLikelihoodFunction m_likelihood; /*!< Likelihood function, which evaluates how much a particle matches the measurements.*/
  vpCommandStateFunction m_bx; /*!< Function that permits to compute the effect of the commands on the prior, with knowledge of the state.*/
  vpFilterFunction m_stateFilterFunc; /*!< Function to compute a weighted mean in the state space.*/
  vpResamplingConditionFunction m_checkIfResample; /*!< Return true if resampling must be performed, false otherwise.*/
  vpResamplingFunction m_resampling; /*!< Performs resampling, i.e. samples particles and weights when the particle filter degenerates.*/
  vpStateAddFunction m_stateAdd; /*!< Function to performs an addition in the state space.*/

  bool m_useProcessFunction; /*!< Set to true when the Particle filter should use the process function.*/
  bool m_useCommandStateFunction; /*!< Set to true when the Particle filter should use the command function.*/
};

template <typename MeasurementsType>
vpUniRand vpParticleFilter<MeasurementsType>::sampler;

template <typename MeasurementsType>
vpUniRand vpParticleFilter<MeasurementsType>::samplerRandomIdx;

template <typename MeasurementsType>
vpParticleFilter<MeasurementsType>::vpParticleFilter(const unsigned int &N, const std::vector<double> &stdev, const long &seed, const int &nbThreads)
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
  omp_set_num_threads(m_nbMaxThreads);
#endif
  // Generating the random generators
  unsigned int sizeState = static_cast<unsigned int>(stdev.size());
  m_noiseGenerators.resize(m_nbMaxThreads);
  unsigned long long seedForGenerator;
  if (seed > 0) {
    seedForGenerator = seed;
  }
  else {
    seedForGenerator = static_cast<unsigned long long>(vpTime::measureTimeMicros());
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::init(const vpColVector &x0, const vpProcessFunction &f,
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::init(const vpColVector &x0, const vpCommandStateFunction &bx,
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::filter(const MeasurementsType &z, const double &dt, const vpColVector &u)
{
  predict(dt, u);
  update(z);
}

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::predict(const double &dt, const vpColVector &u)
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::update(const MeasurementsType &z)
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

template <typename MeasurementsType>
vpColVector vpParticleFilter<MeasurementsType>::computeFilteredState()
{
  return m_stateFilterFunc(m_particles, m_w, m_stateAdd);
}

template <typename MeasurementsType>
vpColVector vpParticleFilter<MeasurementsType>::weightedMean(const std::vector<vpColVector> &particles, const std::vector<double> &weights, const vpStateAddFunction &addFunc)
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

template <typename MeasurementsType>
bool vpParticleFilter<MeasurementsType>::simpleResamplingCheck(const unsigned int &N, const std::vector<double> &weights)
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

template <typename MeasurementsType>
typename vpParticleFilter<MeasurementsType>::vpParticlesWithWeights vpParticleFilter<MeasurementsType>::simpleImportanceResampling(const std::vector<vpColVector> &particles, const std::vector<double> &weights)
{
  unsigned int nbParticles = static_cast<unsigned int>(particles.size());
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::initParticles(const vpColVector &x0)
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
template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::predictMultithread(const double &dt, const vpColVector &u)
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

template <typename MeasurementsType>
double threadLikelihood(const typename vpParticleFilter<MeasurementsType>::vpLikelihoodFunction &likelihood, const std::vector<vpColVector> &v_particles,
                        const MeasurementsType &z, std::vector<double> &w, const int &istart, const int &ipoints)
{
  double sum(0.0);
  for (int i = istart; i< istart + ipoints; ++i) {
    w[i] = w[i] * likelihood(v_particles[i], z);
    sum += w[i];
  }
  return sum;
}

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::updateMultithread(const MeasurementsType &z)
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
    tempSums[iam] = threadLikelihood<MeasurementsType>(m_likelihood, m_particles, z, m_w, istart, ipoints);
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::predictMonothread(const double &dt, const vpColVector &u)
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

template <typename MeasurementsType>
void vpParticleFilter<MeasurementsType>::updateMonothread(const MeasurementsType &z)
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
#endif
#endif
