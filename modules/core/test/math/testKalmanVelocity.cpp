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
 * Tests some vpLinearKalmanFilterInstantiation functionalities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testKalmanVelocity.cpp

  \brief Test some vpLinearKalmanFilterInstantiation functionalities
  with constant velocity state model.
*/

#include <fstream>
#include <iostream>
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

typedef enum {
  Position, // Considered measures are the succesive positions of the target
  Velocity  // Considered measures are the succesive velocities of the target
} vpMeasureType;

int main()
{
  try {
    unsigned int nsignal = 2; // Number of signal to filter
    unsigned int niter = 200;
    unsigned int size_state_vector = 2 * nsignal;
    unsigned int size_measure_vector = 1 * nsignal;
    // vpMeasureType measure_t = Velocity;
    vpMeasureType measure_t = Position;

    std::string filename = "/tmp/log.dat";
    std::ofstream flog(filename.c_str());

    vpLinearKalmanFilterInstantiation kalman;

    vpColVector sigma_measure(size_measure_vector);
    for (unsigned int signal = 0; signal < nsignal; signal++)
      sigma_measure = 0.000001;
    vpColVector sigma_state(size_state_vector);

    switch (measure_t) {
    case Velocity:
      for (unsigned int signal = 0; signal < nsignal; signal++) {
        sigma_state[2 * signal] = 0.; // not used
        sigma_state[2 * signal + 1] = 0.000001;
      }
      break;
    case Position:
      for (unsigned int signal = 0; signal < nsignal; signal++) {
        sigma_state[2 * signal] = 0.000001;
        sigma_state[2 * signal + 1] = 0; // not used
      }
      break;
    }

    vpColVector measure(size_measure_vector);

    for (unsigned int signal = 0; signal < nsignal; signal++) {
      measure[signal] = 3 + 2 * signal;
    }

    kalman.verbose(true);

    vpLinearKalmanFilterInstantiation::vpStateModel model;
    double dt = 0.04; // Sampling period
    double rho = 0.5;
    double dummy = 0; // non used parameter
    switch (measure_t) {
    case Velocity:
      model = vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel;
      kalman.setStateModel(model);
      kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dummy);
      break;
    case Position:
      model = vpLinearKalmanFilterInstantiation::stateConstVel_MeasurePos;
      kalman.setStateModel(model);
      kalman.initFilter(nsignal, sigma_state, sigma_measure, dummy, dt);
      break;
    }

    for (unsigned int iter = 0; iter <= niter; iter++) {
      std::cout << "-------- iter " << iter << " ------------" << std::endl;
      for (unsigned int signal = 0; signal < nsignal; signal++) {
        measure[signal] = 3 + 2 * signal + 0.3 * sin(vpMath::rad(360. / niter * iter));
      }
      std::cout << "measure : " << measure.t() << std::endl;

      flog << measure.t();

      //    kalman.prediction();
      kalman.filter(measure);
      flog << kalman.Xest.t() << std::endl;

      std::cout << "Xest: " << kalman.Xest.t() << std::endl;
    }

    flog.close();
    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
