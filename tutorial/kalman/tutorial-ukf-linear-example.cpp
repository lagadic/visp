/****************************************************************************
 *
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
*****************************************************************************/

//! \example tutorial-ukf-linear-example.cpp

// UKF includes
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>

// ViSP includes
#include <visp3/core/vpUniRand.h>
#include <visp3/gui/vpPlot.h>

/**
 * \brief The process function, that updates the prior.
 *
 * \param[in] chi The sigma points.
 * \param[in] dt The period.
 * \return std::vector<vpColVector> The sigma points projected in the future.
 */
std::vector<vpColVector> fx(const std::vector<vpColVector> &chi, const double &dt)
{
  unsigned int nbPoints = chi.size();
  std::vector<vpColVector> points;
  for (unsigned int i = 0; i < nbPoints; ++i) {
    vpColVector point(2);
    point[0] = chi[i][1] * dt + chi[i][0];
    point[1] = chi[i][1];
    points.push_back(point);
  }
  return points;
}

/**
 * \brief The measurement function, that project the prior in the measurement space.
 *
 * \param[in] chi The prior.
 * \return std::vector<vpColVector> The prior projected in the measurement space.
 */
std::vector<vpColVector> hx(const std::vector<vpColVector> &chi)
{
  unsigned int nbPoints = chi.size();
  std::vector<vpColVector> points;
  for (unsigned int i = 0; i < nbPoints; ++i) {
    vpColVector point(1);
    point[0] = chi[i][0];
    points.push_back(point);
  }
  return points;
}

int main(/*const int argc, const char *argv[]*/)
{
  const double dt = 1.; // Period of 1s
  const double gt_dx = 1.; // Ground truth displacement between two measurements
  const double gt_v = gt_dx / dt; // Ground truth velocity
  const double processVariance = 0.03;

  // Initialize the attributes of the UKF
  vpUKSigmaDrawerMerwe drawer(2, 0.3, 2., 0.1);
  vpMatrix P0(2, 2); //  The initial guess of the process covariance
  P0.eye(2, 2);
  P0 = P0 * 10.;
  vpMatrix R(2, 2); // The covariance of the noise introduced by the measurement
  R.eye(1, 1);
  R = R * 0.5;
  vpMatrix Q(2, 2); // The covariance of the process
  Q[0][0] = std::pow(dt, 3) / 3.;
  Q[0][1] = std::pow(dt, 2)/2.;
  Q[1][0] = std::pow(dt, 2)/2.;
  Q[1][1] = dt;
  Q = Q * processVariance;
  vpUnscentedKalman::process_function f = fx;
  vpUnscentedKalman::measurement_function h = hx;

  // Initialize the UKF
  vpUnscentedKalman ukf(Q, R, &drawer, f, h);
  ukf.init(vpColVector(2, 0.), P0);

  // Initialize the plot
  vpPlot plot(2);
  plot.initGraph(0, 3);
  plot.setTitle(0, "Position");
  plot.setUnitX(0, "Time (s)");
  plot.setUnitY(0, "Position (m)");
  plot.setLegend(0, 0, "GT");
  plot.setLegend(0, 1, "Measure");
  plot.setLegend(0, 2, "Filtered");

  plot.initGraph(1, 3);
  plot.setTitle(1, "Velocity");
  plot.setUnitX(1, "Time (s)");
  plot.setUnitY(1, "Velocity (m/s)");
  plot.setLegend(1, 0, "GT");
  plot.setLegend(1, 1, "Measure");
  plot.setLegend(1, 2, "Filtered");

  // Initialize measurement noise
  vpUniRand rng(vpTime::measureTimeMicros());

  // Main loop
  double gt_x = 0.;
  double z_prec = 0.;
  for (int i = 0; i < 50; ++i) {
    // Perform the measurement
    double z_meas = gt_x + rng.uniform(-0.5, 0.5);
    vpColVector z(1);
    z[0] = z_meas;

    // Use the UKF to filter the measurement
    ukf.filter(z, dt);
    vpColVector Xest = ukf.getXest();

    // Plot the ground truth, measurement and filtered state
    plot.plot(0, 0, i, gt_x);
    plot.plot(0, 1, i, z_meas);
    plot.plot(0, 2, i, Xest[0]);

    double v_meas = (z_meas - z_prec) / dt;
    plot.plot(1, 0, i, gt_v);
    plot.plot(1, 1, i, v_meas);
    plot.plot(1, 2, i, Xest[1]);

    // Update
    gt_x += gt_dx;
    z_prec = z_meas;
  }
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();
  return 0;
}
