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
 * Test Particle Filter functionalities.
 */

/*!
  \example catchParticleFilter1.cpp

  Test some vpParticleFilter functionalities.
  The aim is to fit a polynomial to input data.
*/
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <limits>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpParticleFilter.h>
#include <visp3/core/vpUniRand.h>

#ifdef VISP_HAVE_DISPLAY
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#endif

#include <catch_amalgamated.hpp>

#include "catchParticleFilter.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
bool opt_display = false; //!< If true, activate debug display
}

TEST_CASE("2nd-degree", "[vpParticleFilter][Polynomial interpolation]")
{
  /// ----- Simulation parameters -----
  const unsigned int width = 600; //!< The width of the simulated image
  const unsigned int height = 400; //!< The height of the simulated image
  const unsigned int degree = 2; //!< The degree of the polynomial in the simulated image
  const unsigned int nbInitPoints = 10; //!< Number of points to compute the initial guess of the PF state
  const uint64_t seedCurve = 4224; //!< The seed to generate the curve
  const uint64_t seedInitPoints = 2112; //!< The seed to choose the init points
  const unsigned int nbTestRepet = 10/2; //!< The number of times the test is repeated
  const unsigned int nbWarmUpIter = 10/2; //!< Number of iterations for the warmup loop
  const unsigned int nbEvalIter = 20; //!< Number of iterations for the evaluation loop
  const double dt = 0.040; //!< Simulated period of acquisition
  const int32_t seedShuffle = 4221; //!< The seed to shuffle the curve points

  /// ----- PF parameters -----
  // The maximum amplitude for the likelihood compute.
  // A particle whose "distance" with the measurements is greater than this value has a likelihood of 0
  const double ampliMaxLikelihood = 16.;
  const double sigmaLikelihood = ampliMaxLikelihood / 3.; //:< The corresponding standard deviation
  const unsigned int nbParticles = 300; //!< Number of particles used by the particle filter
  const double ratioAmpliMax(0.25); //!< Ratio of the initial guess values to use to add noise to the PF state
  const long seedPF = 4221; //!< Seed of the particle filter
  const int nbThreads = 1; //<! Number of threads to use for the PF
  vpUniRand rngCurvePoints(seedCurve);
  vpUniRand rngInitPoints(seedInitPoints);

  SECTION("Noise-free", "The init points are directly extracted from the curve points, without any additional noise")
  {
    const double maxToleratedError = 5.;
    double x0 = rngCurvePoints.uniform(0., width);
    double x1 = rngCurvePoints.uniform(0., width);
    double y0 = rngCurvePoints.uniform(0., height);
    double y1 = rngCurvePoints.uniform(0., height);
    vpColVector coeffs = computeABC(x0, y0, x1, y1);
    std::vector<vpImagePoint> curvePoints = generateSimulatedImage(0, width, 1., coeffs);

#ifdef VISP_HAVE_DISPLAY
    vpImage<vpRGBa> I(height, width);
    std::shared_ptr<vpDisplay> pDisplay;
    if (opt_display) {
      pDisplay = vpDisplayFactory::createDisplay(I);
    }
#endif

    for (unsigned int iter = 0; iter < nbTestRepet; ++iter) {
      // Randomly select the initialization points
      std::vector<vpImagePoint> suffledVector = vpUniRand::shuffleVector(curvePoints, seedShuffle);
      std::vector<vpImagePoint> initPoints;
      for (unsigned int j = 0; j < nbInitPoints; ++j) {
        initPoints.push_back(suffledVector[j]);
      }

      // Compute the initial model
      vpParabolaModel modelInitial = computeInitialGuess(initPoints, degree, height, width);
      vpColVector X0 = modelInitial.toVpColVector();

      // Initialize the Particle Filter
      std::vector<double> stdevsPF;
      for (unsigned int i = 0; i < degree + 1; ++i) {
        stdevsPF.push_back(ratioAmpliMax * X0[0] / 3.);
      }
      vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = fx;
      vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood, height, width);
      using std::placeholders::_1;
      using std::placeholders::_2;
      vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
      vpAverageFunctor averageCpter(degree, height, width);
      using std::placeholders::_3;
      vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&vpAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
      vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThreads);
      filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);

      for (unsigned int i = 0; i < nbWarmUpIter; ++i) {
        filter.filter(curvePoints, dt);
      }

      double meanError = 0.;
      for (unsigned int i = 0; i < nbEvalIter; ++i) {
        filter.filter(curvePoints, dt);
        vpColVector Xest = filter.computeFilteredState();
        vpParabolaModel model(Xest, height, width);
        double rmse = evaluate(curvePoints, model);
        meanError += rmse;

#ifdef VISP_HAVE_DISPLAY
        if (opt_display) {
          vpDisplay::display(I);
          displayGeneratedImage(I, curvePoints, vpColor::red, "GT", 20, 20);
          model.display(I, vpColor::blue, "Model", 40, 20);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
#endif
      }
      meanError /= static_cast<double>(nbEvalIter);
      std::cout << "Mean(rmse) = " << meanError << std::endl;
      CHECK(meanError <= maxToleratedError);
    }
  }

  SECTION("Noisy", "Noise is added to the init points")
  {
    const double maxToleratedError = 12.;
    double x0 = rngCurvePoints.uniform(0., width);
    double x1 = rngCurvePoints.uniform(0., width);
    double y0 = rngCurvePoints.uniform(0., height);
    double y1 = rngCurvePoints.uniform(0., height);
    vpColVector coeffs = computeABC(x0, y0, x1, y1);
    std::vector<vpImagePoint> curvePoints = generateSimulatedImage(0, width, 1., coeffs);

#ifdef VISP_HAVE_DISPLAY
    vpImage<vpRGBa> I(height, width);
    std::shared_ptr<vpDisplay> pDisplay;
    if (opt_display) {
      pDisplay = vpDisplayFactory::createDisplay(I);
    }
#endif

    const double ampliMaxInitNoise = 24.;
    const double stdevInitNoise = ampliMaxInitNoise / 3.;
    vpGaussRand rngInitNoise(stdevInitNoise, 0., seedInitPoints);

    for (unsigned int iter = 0; iter < nbTestRepet; ++iter) {
      // Randomly select the initialization points
      std::vector<vpImagePoint> suffledVector = vpUniRand::shuffleVector(curvePoints, seedShuffle);
      std::vector<vpImagePoint> initPoints;
      for (unsigned int j = 0; j < nbInitPoints; ++j) {
        vpImagePoint noisyPt(suffledVector[j].get_i() + rngInitNoise(), suffledVector[j].get_j() + rngInitNoise());
        initPoints.push_back(noisyPt);
      }

      // Compute the initial model
      vpParabolaModel modelInitial = computeInitialGuess(initPoints, degree, height, width);
      vpColVector X0 = modelInitial.toVpColVector();

      // Initialize the Particle Filter
      std::vector<double> stdevsPF;
      for (unsigned int i = 0; i < degree + 1; ++i) {
        stdevsPF.push_back(ratioAmpliMax * X0[0] / 3.);
      }
      vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = fx;
      vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood, height, width);
      using std::placeholders::_1;
      using std::placeholders::_2;
      vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
      vpAverageFunctor averageCpter(degree, height, width);
      using std::placeholders::_3;
      vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&vpAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
      vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThreads);
      filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);

      for (unsigned int i = 0; i < nbWarmUpIter; ++i) {
        filter.filter(curvePoints, dt);
      }

      double meanError = 0.;
      for (unsigned int i = 0; i < nbEvalIter; ++i) {
        filter.filter(curvePoints, dt);
        vpColVector Xest = filter.computeFilteredState();
        vpParabolaModel model(Xest, height, width);
        double rmse = evaluate(curvePoints, model);
        meanError += rmse;

#ifdef VISP_HAVE_DISPLAY
        if (opt_display) {
          vpDisplay::display(I);
          displayGeneratedImage(I, curvePoints, vpColor::red, "GT", 20, 20);
          model.display(I, vpColor::blue, "Model", 40, 20);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
#endif
      }
      meanError /= static_cast<double>(nbEvalIter);
      std::cout << "Mean(rmse) = " << meanError << std::endl;
      CHECK(meanError <= maxToleratedError);
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  auto cli = session.cli()
    | Catch::Clara::Opt(opt_display)["--display"]("Activate debug display");

  session.cli(cli);
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
