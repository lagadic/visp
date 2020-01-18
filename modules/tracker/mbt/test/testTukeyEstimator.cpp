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
 * Test Tukey M-Estimator.
 *
 *****************************************************************************/

/*!
  \example testTukeyEstimator.cpp

  \brief Test Tukey M-Estimator.
*/

#include <cstdlib>
#include <iostream>
#include <time.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpRobust.h>
#include <visp3/mbt/vpMbtTukeyEstimator.h>

int main(int /*argc*/, const char ** /*argv*/)
{
  size_t nb_elements = 1000;
  int nb_iterations = 100;
  double stdev = 0.5, mean = 0.0, noise_threshold = 1e-3;

  vpGaussRand noise(stdev, mean);
  noise.seed((unsigned int)time(NULL));

  vpColVector residues_col((unsigned int)nb_elements);
  vpColVector weights_col((unsigned int)nb_elements, 1.0), weights_col_save;
  for (size_t i = 0; i < nb_elements; i++) {
    residues_col[(unsigned int)i] = noise();
  }

  vpRobust robust((unsigned int)nb_elements);
  robust.setThreshold(noise_threshold);
  double t_robust = vpTime::measureTimeMs();
  for (int i = 0; i < nb_iterations; i++) {
    robust.MEstimator(vpRobust::TUKEY, residues_col, weights_col);
  }
  t_robust = vpTime::measureTimeMs() - t_robust;

  {

    vpMbtTukeyEstimator<double> tukey_estimator;
    std::vector<double> residues(nb_elements);
    for (size_t i = 0; i < residues.size(); i++) {
      residues[i] = residues_col[(unsigned int)i];
    }

    std::vector<double> weights(nb_elements, 1);
    double t = vpTime::measureTimeMs();
    for (int i = 0; i < nb_iterations; i++) {
      tukey_estimator.MEstimator(residues, weights, noise_threshold);
    }
    t = vpTime::measureTimeMs() - t;

    std::cout << "t_robust=" << t_robust << " ms ; t (double)=" << t << " ; ratio=" << (t_robust / t) << std::endl;

    for (size_t i = 0; i < weights.size(); i++) {
      if (!vpMath::equal(weights[i], weights_col[(unsigned int)i], noise_threshold)) {
        std::cerr << "Difference between vpRobust::TUKEY and "
                     "vpMbtTukeyEstimator (double)!"
                  << std::endl;
        std::cerr << "weights_col[" << i << "]=" << weights_col[(unsigned int)i] << std::endl;
        std::cerr << "weights[" << i << "]=" << weights[i] << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  // Generate again for weights != 1
  for (size_t i = 0; i < nb_elements; i++) {
    residues_col[(unsigned int)i] = noise();
  }
  weights_col_save = weights_col;
  t_robust = vpTime::measureTimeMs();
  for (int i = 0; i < nb_iterations; i++) {
    robust.MEstimator(vpRobust::TUKEY, residues_col, weights_col);
  }
  t_robust = vpTime::measureTimeMs() - t_robust;

  {
    vpMbtTukeyEstimator<float> tukey_estimator;
    std::vector<float> residues(nb_elements);
    std::vector<float> weights(nb_elements);
    for (size_t i = 0; i < residues.size(); i++) {
      residues[i] = (float)residues_col[(unsigned int)i];
      weights[i] = (float)weights_col_save[(unsigned int)i];
    }

    double t = vpTime::measureTimeMs();
    for (int i = 0; i < nb_iterations; i++) {
      tukey_estimator.MEstimator(residues, weights, (float)noise_threshold);
    }
    t = vpTime::measureTimeMs() - t;

    std::cout << "t_robust=" << t_robust << " ms ; t (float)=" << t << " ; ratio=" << (t_robust / t) << std::endl;

    for (size_t i = 0; i < weights.size(); i++) {
      if (!vpMath::equal(weights[i], weights_col[(unsigned int)i], noise_threshold)) {
        std::cerr << "Difference between vpRobust::TUKEY and "
                     "vpMbtTukeyEstimator (float)!"
                  << std::endl;
        std::cerr << "weights_col[" << i << "]=" << weights_col[(unsigned int)i] << std::endl;
        std::cerr << "weights[" << i << "]=" << weights[i] << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  // Generate again for weights != 1 and vpColVector type
  for (size_t i = 0; i < nb_elements; i++) {
    residues_col[(unsigned int)i] = noise();
  }
  weights_col_save = weights_col;
  t_robust = vpTime::measureTimeMs();
  for (int i = 0; i < nb_iterations; i++) {
    robust.MEstimator(vpRobust::TUKEY, residues_col, weights_col);
  }
  t_robust = vpTime::measureTimeMs() - t_robust;

  {
    vpMbtTukeyEstimator<double> tukey_estimator;
    vpColVector residues = residues_col;
    vpColVector weights = weights_col_save;

    double t = vpTime::measureTimeMs();
    for (int i = 0; i < nb_iterations; i++) {
      tukey_estimator.MEstimator(residues, weights, noise_threshold);
    }
    t = vpTime::measureTimeMs() - t;

    std::cout << "t_robust=" << t_robust << " ms ; t (vpColVector)=" << t << " ; ratio=" << (t_robust / t) << std::endl;

    for (size_t i = 0; i < weights.size(); i++) {
      if (!vpMath::equal(weights[(unsigned int)i], weights_col[(unsigned int)i], noise_threshold)) {
        std::cerr << "Difference between vpRobust::TUKEY and "
                     "vpMbtTukeyEstimator (float)!"
                  << std::endl;
        std::cerr << "weights_col[" << i << "]=" << weights_col[(unsigned int)i] << std::endl;
        std::cerr << "weights[" << i << "]=" << weights[(unsigned int)i] << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  std::cout << "vpMbtTukeyEstimator returns the same values than vpRobust::TUKEY." << std::endl;
  return EXIT_SUCCESS;
}
