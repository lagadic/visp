/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
  \example testKalmanAcceleration.cpp

  \brief Test some vpLinearKalmanFilterInstantiation functionalities
  with constant acceleration state model.
*/
#include <iostream>
#include <fstream>

#include <visp/vpLinearKalmanFilterInstantiation.h>

int
main()
{
  int nsignal = 1; // Number of signal to filter
  int niter = 100;

  std::string filename = "/tmp/log.dat";
  std::ofstream flog(filename.c_str());

  vpLinearKalmanFilterInstantiation kalman;
 
  vpLinearKalmanFilterInstantiation::vpStateModel model;
  model = vpLinearKalmanFilterInstantiation::stateConstAccWithColoredNoise_MeasureVel;
  kalman.setStateModel(model);

  int size_state_vector = kalman.getStateSize()*nsignal;
  int size_measure_vector = kalman.getMeasureSize()*nsignal;

  vpColVector sigma_measure(size_measure_vector);
  for (int signal=0; signal < nsignal; signal ++) 
    sigma_measure = 0.0001;
  vpColVector sigma_state(size_state_vector);
  for (int signal=0; signal < nsignal; signal ++) { 
    sigma_state[3*signal] = 0.; // not used
    sigma_state[3*signal+1] = 0.000001;
    sigma_state[3*signal+2] = 0.000001;
  }
  
  vpColVector velocity_measure(size_measure_vector);

  double rho = 0.9; // correlation
  double dt = 0.2; // sampling period

  for (int signal=0; signal < nsignal; signal ++) 
    velocity_measure[signal] = 3+2*signal;

  kalman.verbose(false);
  kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dt);


  for (int iter=0; iter <= niter; iter++) {
    std::cout << "-------- iter " << iter << " ------------" << std::endl;
    for (int signal=0; signal < nsignal; signal ++) {
      velocity_measure[signal] = 3+2*signal 
	+ 0.3*sin(vpMath::rad(360./niter*iter));
    }
    std::cout << "measure : " << velocity_measure.t() << std::endl;

    flog << velocity_measure.t();

    //    kalman.prediction();
    kalman.filter(velocity_measure);
    flog << kalman.Xest.t();
    flog << kalman.Xpre.t();

    std::cout << "Xest: " << kalman.Xest.t() << std::endl;
    std::cout << "Xpre: " << kalman.Xpre.t() << std::endl;

    flog <<  std::endl;
  }

  flog.close();
  return 0;
}
