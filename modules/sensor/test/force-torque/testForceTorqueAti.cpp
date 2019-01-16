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
 * Test force/torque ATI sensor.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testForceTorqueAti.cpp
  This example shows how to retrieve data from an ATI F/T sensor connected to
  a DAQ board. He we have 2 threads:
  - the main thread running at 1KHz that get the measures from an ATI F/T
  sensor and records data in recorded-ft-sync.txt file
  - the scope thread that plots the F/T values in real-time.
*/

#include <iostream>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/sensor/vpForceTorqueAtiSensor.h>

#if defined(VISP_HAVE_PTHREAD)

typedef enum { BIAS_DONE, UNBIAS_DONE, TO_BIAS, TO_UNBIAS } BiasState;

vpMutex s_mutex_data;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
typedef struct {
  vpColVector ft;
  double timestamp;
  BiasState bias_state;
} t_shared_data;
#endif

t_shared_data s_shared_data;

vpMutex s_mutex_state;
bool s_state_stop = false;

vpThread::Return scopeFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args

#ifdef VISP_HAVE_DISPLAY
  vpPlot scope(2, 700, 700, 100, 200, "ATI F/T sensor data");
  scope.initGraph(0, 3);
  scope.initGraph(1, 3);
  scope.setTitle(0, "Forces (N)");
  scope.setTitle(1, "Torques (Nm)");
  scope.setLegend(0, 0, "x");
  scope.setLegend(0, 1, "y");
  scope.setLegend(0, 2, "z");
  scope.setLegend(1, 0, "x");
  scope.setLegend(1, 1, "y");
  scope.setLegend(1, 2, "z");
#endif

  t_shared_data shared_data;
#ifdef VISP_HAVE_DISPLAY
  bool click = false;
  vpMouseButton::vpMouseButtonType button;
#else
  double start_time = vpTime::measureTimeMs();
#endif
  do {
#ifdef VISP_HAVE_DISPLAY
    { // Get new measures to plot
      vpMutex::vpScopedLock lock(s_mutex_data);
      shared_data.ft = s_shared_data.ft;
      shared_data.timestamp = s_shared_data.timestamp;
      shared_data.bias_state = s_shared_data.bias_state;
    }

    vpColVector force = shared_data.ft.extract(0, 3);
    vpColVector torque = shared_data.ft.extract(3, 3);
    scope.plot(0, shared_data.timestamp, force);
    scope.plot(1, shared_data.timestamp, torque);

    vpDisplay::displayText(scope.I, 15, 500, "Left  click to quit", vpColor::red);
    vpDisplay::displayText(scope.I, 30, 500, "Right click to bias/unbias", vpColor::red);
    if (shared_data.bias_state == BIAS_DONE)
      vpDisplay::displayText(scope.I, 45, 500, "Sensor is biased...", vpColor::blue);
    else if (shared_data.bias_state == UNBIAS_DONE)
      vpDisplay::displayText(scope.I, 45, 500, "Sensor is unbiased...", vpColor::blue);
    else if (shared_data.bias_state == TO_BIAS)
      vpDisplay::displayText(scope.I, 45, 500, "Sensor bias in progress...", vpColor::blue);
    else if (shared_data.bias_state == TO_UNBIAS)
      vpDisplay::displayText(scope.I, 45, 500, "Sensor unbias in progress...", vpColor::blue);
    vpDisplay::flush(scope.I);
    click = vpDisplay::getClick(scope.I, button, false);
    if (click && button == vpMouseButton::button3) {
      if (shared_data.bias_state == BIAS_DONE)
        shared_data.bias_state = TO_UNBIAS;
      else if (shared_data.bias_state == UNBIAS_DONE)
        shared_data.bias_state = TO_BIAS;
      { // Set new bias state
        vpMutex::vpScopedLock lock(s_mutex_data);
        s_shared_data.bias_state = shared_data.bias_state;
      }
    }

#endif
  }
#ifdef VISP_HAVE_DISPLAY
  while (!(click && button == vpMouseButton::button1)); // Stop recording by a user left click
#else
  while (vpTime::measureTimeMs() - start_time < 20000); // Stop recording after 20 seconds
#endif

  { // Update state to stop
    vpMutex::vpScopedLock lock(s_mutex_state);
    s_state_stop = true;
  }

  std::cout << "End of scope thread" << std::endl;
  return 0;
}

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_ATIDAQ) && defined(VISP_HAVE_COMEDI)

#ifdef VISP_HAVE_VIPER850_DATA
  (void)argc;
  (void)argv;
  std::string calibfile = std::string(VISP_VIPER850_DATA_PATH) + std::string("/ati/FT17824.cal");
  if (!vpIoTools::checkFilename(calibfile)) {
    std::cout << "ATI F/T calib file \"" << calibfile << "\" doesn't exist";
    return 0;
  }
#else
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <ATI calibration file FT*.cal]>" << std::endl;
    return -1;
  }
  std::string calibfile(argv[1]);
#endif

  vpForceTorqueAtiSensor ati;
  ati.setCalibrationFile(calibfile);
  ati.open();
  std::cout << "ATI F/T sensor characteristics: \n" << ati << std::endl;

  ati.bias();
  std::cout << "Data recording in progress..." << std::endl;

  // Start scope thread
  vpThread thread_scope(scopeFunction);

  std::string file("recorded-ft-sync.txt");
  std::ofstream f(file.c_str());
  bool state_stop;
  t_shared_data shared_data;

  double start_time = vpTime::measureTimeMs();

  do {
    double loop_time = vpTime::measureTimeMs();
    vpColVector ft = ati.getForceTorque();
    double timestamp = loop_time - start_time;

    { // Update shared F/T measure used by the scope to plot curves
      vpMutex::vpScopedLock lock(s_mutex_data);
      shared_data.bias_state = s_shared_data.bias_state;
    }
    if (shared_data.bias_state == TO_BIAS) {
      std::cout << "Bias sensor" << std::endl;
      ati.bias();
      std::cout << "Unbias sensor" << std::endl;
      shared_data.bias_state = BIAS_DONE;
    } else if (shared_data.bias_state == TO_UNBIAS) {
      ati.unbias();
      shared_data.bias_state = UNBIAS_DONE;
    }

    { // Update shared F/T measure used by the scope to plot curves
      vpMutex::vpScopedLock lock(s_mutex_data);
      s_shared_data.ft = ft;
      s_shared_data.timestamp = timestamp;
      s_shared_data.bias_state = shared_data.bias_state;
    }
    { // Get state to stop
      vpMutex::vpScopedLock lock(s_mutex_state);
      state_stop = s_state_stop;
    }

    f << timestamp << " " << ft.t() << std::endl;
    vpTime::wait(loop_time, 1); // Get a new data each 1 millisecond
  } while (!state_stop);

  // Wait until thread ends up
  thread_scope.join();

  ati.close();
  f.close();
  std::cout << "Data recorded in " << file << std::endl;
#else
  (void)argc;
  (void)argv;
  std::cout << "You should install comedi and build atidaq to enable this test..." << std::endl;
#endif
}

#else
int main() { std::cout << "You should build this test with threading capabilities..." << std::endl; }
#endif
