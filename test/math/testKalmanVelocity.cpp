/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
#include <iostream>
#include <fstream>

#include <visp/vpLinearKalmanFilterInstantiation.h>

typedef enum {
  Position, // Considered measures are the succesive positions of the target
  Velocity  // Considered measures are the succesive velocities of the target
} vpMeasureType;

int
main()
{
  int nsignal = 2; // Number of signal to filter
  int niter = 200;
  int size_state_vector = 2*nsignal;
  int size_measure_vector = 1*nsignal;
  //vpMeasureType measure_t = Velocity;
    vpMeasureType measure_t = Position;

  std::string filename = "/tmp/log.dat";
  std::ofstream flog(filename.c_str());

  vpLinearKalmanFilterInstantiation kalman;

  vpColVector sigma_measure(size_measure_vector);
  for (int signal=0; signal < nsignal; signal ++) 
    sigma_measure = 0.000001;
  vpColVector sigma_state(size_state_vector);

  switch (measure_t) {
  case Velocity:
    for (int signal=0; signal < nsignal; signal ++) {
      sigma_state[2*signal] = 0.; // not used
      sigma_state[2*signal+1] = 0.000001;
    }
    break;
  case Position:
    for (int signal=0; signal < nsignal; signal ++) {
      sigma_state[2*signal] = 0.000001; 
      sigma_state[2*signal+1] = 0; // not used
    }
    break;
  }
  
  vpColVector measure(size_measure_vector);

  for (int signal=0; signal < nsignal; signal ++) {
    measure[signal] = 3+2*signal;
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

  for (int iter=0; iter <= niter; iter++) {
    std::cout << "-------- iter " << iter << " ------------" << std::endl;
    for (int signal=0; signal < nsignal; signal ++) {
      measure[signal] = 3+2*signal + 0.3*sin(vpMath::rad(360./niter*iter));
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
}
