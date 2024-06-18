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
 */

/** \example ukf-linear-example.cpp
 * Example of a simple linear use-case of the Unscented Kalman Filter (UKF). Using a UKF
 * in this case is not necessary, it is done for learning purpose only.
 *
 * The system we are interested in is a system moving on a 2D-plane.
 *
 * The state vector of the UKF is:
 * \f[
 * \begin{array}{lcl}
 *   \textbf{x}[0] &=& x \\
 *   \textbf{x}[1] &=& \dot{x} \\
 *   \textbf{x}[1] &=& y \\
 *   \textbf{x}[2] &=& \dot{y}
 * \end{array}
 * \f]
 *
 * The measurement \f$ \textbf{z} \f$ corresponds to the position along the x-axis
 * and y-axis. The measurement vector can be written as:
 * \f[
 * \begin{array}{lcl}
 *   \textbf{z}[0] &=& x \\
 *   \textbf{z}[1] &=& y
 * \end{array}
 * \f]
 * Some noise is added to the measurement vector to simulate a sensor which is
 * not perfect.
*/

// UKF includes
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpGaussRand.h>
#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/**
 * \brief The process function, that updates the prior.
 *
 * \param[in] chi A sigma point.
 * \param[in] dt The period.
 * \return vpColVector The sigma point projected in the future.
 */
vpColVector fx(const vpColVector &chi, const double &dt)
{
  vpColVector point(4);
  point[0] = chi[1] * dt + chi[0];
  point[1] = chi[1];
  point[2] = chi[3] * dt + chi[2];
  point[3] = chi[3];
  return point;
}

/**
 * \brief The measurement function, that project the prior in the measurement space.
 *
 * \param[in] chi The prior.
 * \return vpColVector The prior projected in the measurement space.
 */
vpColVector hx(const vpColVector &chi)
{
  vpColVector point(2);
  point[0] = chi[0];
  point[1] = chi[2];
  return point;
}

int main(const int argc, const char *argv[])
{
  bool opt_useDisplay = true;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "-d") {
      opt_useDisplay = false;
    }
    else if ((arg == "-h") || (arg == "--help")) {
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "  " << argv[0] << " [-d][-h]" << std::endl;
      std::cout << std::endl << std::endl;
      std::cout << "DETAILS" << std::endl;
      std::cout << "  -d" << std::endl;
      std::cout << "    Deactivate display." << std::endl;
      std::cout << std::endl;
      std::cout << "  -h, --help" << std::endl;
      return 0;
    }
  }

  const double dt = 0.01; // Period of 1s
  const double gt_dx = 0.01; // Ground truth displacement along x axis between two measurements
  const double gt_dy = 0.005; // Ground truth displacement along x axis between two measurements
  vpColVector gt_dX(2); // Ground truth displacement between two measurements
  gt_dX[0] = gt_dx;
  gt_dX[1] = gt_dy;
  const double gt_vx = gt_dx / dt; // Ground truth velocity along x axis
  const double gt_vy = gt_dy / dt; // Ground truth velocity along y axis
  const double processVariance = 0.000004;
  const double sigmaXmeas = 0.05; // Standard deviation of the measure along the x-axis
  const double sigmaYmeas = 0.05; // Standard deviation of the measure along the y-axis

  // Initialize the attributes of the UKF
  std::shared_ptr<vpUKSigmaDrawerAbstract> drawer = std::make_shared<vpUKSigmaDrawerMerwe>(4, 0.3, 2., -1.);
  vpMatrix P0(4, 4); //  The initial guess of the process covariance
  P0.eye(4, 4);
  P0 = P0 * 1.;
  vpMatrix R(2, 2); // The covariance of the noise introduced by the measurement
  R.eye(2, 2);
  R = R * 0.01;
  vpMatrix Q(4, 4, 0.); // The covariance of the process
  vpMatrix Q1d(2, 2); // The covariance of a process whose states are {x, dx/dt} and for which the variance is 1
  Q1d[0][0] = std::pow(dt, 3) / 3.;
  Q1d[0][1] = std::pow(dt, 2)/2.;
  Q1d[1][0] = std::pow(dt, 2)/2.;
  Q1d[1][1] = dt;
  Q.insert(Q1d, 0, 0);
  Q.insert(Q1d, 2, 2);
  Q = Q * processVariance;
  vpUnscentedKalman::vpProcessFunction f = fx;
  vpUnscentedKalman::vpMeasurementFunction h = hx;

  // Initialize the UKF
  vpUnscentedKalman ukf(Q, R, drawer, f, h);
  ukf.init(vpColVector({ 0., 0.75 * gt_vx, 0., 0.75 * gt_vy }), P0);

#ifdef VISP_HAVE_DISPLAY
  vpPlot *plot = nullptr;
  // Initialize the plot
  if (opt_useDisplay) {
    plot = new vpPlot(4);
    plot->initGraph(0, 3);
    plot->setTitle(0, "Position along X-axis");
    plot->setUnitX(0, "Time (s)");
    plot->setUnitY(0, "Position (m)");
    plot->setLegend(0, 0, "GT");
    plot->setLegend(0, 1, "Measure");
    plot->setLegend(0, 2, "Filtered");

    plot->initGraph(1, 3);
    plot->setTitle(1, "Velocity along X-axis");
    plot->setUnitX(1, "Time (s)");
    plot->setUnitY(1, "Velocity (m/s)");
    plot->setLegend(1, 0, "GT");
    plot->setLegend(1, 1, "Measure");
    plot->setLegend(1, 2, "Filtered");

    plot->initGraph(2, 3);
    plot->setTitle(2, "Position along Y-axis");
    plot->setUnitX(2, "Time (s)");
    plot->setUnitY(2, "Position (m)");
    plot->setLegend(2, 0, "GT");
    plot->setLegend(2, 1, "Measure");
    plot->setLegend(2, 2, "Filtered");

    plot->initGraph(3, 3);
    plot->setTitle(3, "Velocity along Y-axis");
    plot->setUnitX(3, "Time (s)");
    plot->setUnitY(3, "Velocity (m/s)");
    plot->setLegend(3, 0, "GT");
    plot->setLegend(3, 1, "Measure");
    plot->setLegend(3, 2, "Filtered");
  }
#endif

  // Initialize measurement noise
  vpGaussRand rngX(sigmaXmeas, 0., 4224);
  vpGaussRand rngY(sigmaYmeas, 0., 2112);

  // Main loop
  vpColVector gt_X(2, 0.);
  vpColVector z_prec(2, 0.);
  for (int i = 0; i < 100; ++i) {
    // Perform the measurement
    double x_meas = gt_X[0] + rngX();
    double y_meas = gt_X[1] + rngY();
    vpColVector z(2);
    z[0] = x_meas;
    z[1] = y_meas;

    // Use the UKF to filter the measurement
    ukf.filter(z, dt);
    vpColVector Xest = ukf.getXest();

#ifdef VISP_HAVE_DISPLAY
    if (opt_useDisplay) {
    // Plot the ground truth, measurement and filtered state
      plot->plot(0, 0, i, gt_X[0]);
      plot->plot(0, 1, i, x_meas);
      plot->plot(0, 2, i, Xest[0]);

      double vX_meas = (x_meas - z_prec[0]) / dt;
      plot->plot(1, 0, i, gt_vx);
      plot->plot(1, 1, i, vX_meas);
      plot->plot(1, 2, i, Xest[1]);

      plot->plot(2, 0, i, gt_X[1]);
      plot->plot(2, 1, i, y_meas);
      plot->plot(2, 2, i, Xest[2]);

      double vY_meas = (y_meas - z_prec[1]) / dt;
      plot->plot(3, 0, i, gt_vy);
      plot->plot(3, 1, i, vY_meas);
      plot->plot(3, 2, i, Xest[3]);
    }
#endif

    // Update
    gt_X += gt_dX;
    z_prec = z;
  }

  if (opt_useDisplay) {
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();
  }

#ifdef VISP_HAVE_DISPLAY
  if (opt_useDisplay) {
    delete plot;
  }
#endif

  vpColVector X_GT({ gt_X[0], gt_vx, gt_X[1], gt_vy });
  vpColVector finalError = ukf.getXest() - X_GT;
  const double maxError = 0.12;
  if (finalError.frobeniusNorm() > maxError) {
    std::cerr << "Error: max tolerated error = " << maxError << ", final error = " << finalError.frobeniusNorm() << std::endl;
    return -1;
  }
  return 0;
}
#else
int main()
{
  std::cout << "vpUnscentedKalman is only available if you compile ViSP in C++11 standard or higher." << std::endl;
  return 0;
}
#endif
