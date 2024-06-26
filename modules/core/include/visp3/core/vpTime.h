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
 * Time management and measurement.
 */

/*!
  \file vpTime.h
  \brief Time management and measurement
 */

#ifndef VP_TIME_H
#define VP_TIME_H

#include <iostream>
#include <string>
#include <visp3/core/vpConfig.h>
#if VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11
#include <chrono>
#endif

#include <visp3/core/vpException.h>

BEGIN_VISP_NAMESPACE
/*!
 * \ingroup group_core_time
 * \brief Time management and measurement.
 *
 * The example below shows how to synchronize a loop to a given framerate.
 *
 * \code
 * #include <visp3/core/vpTime.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   double t;
 *   for ( ; ; ) {
 *     t = vpTime::measureTimeMs();
 *     ...
 *     vpTime::wait(t, 40); // Loop time is set to 40 ms, ie 25 Hz
 *   }
 * }
 * \endcode
*/

namespace vpTime
{
VISP_EXPORT std::string getDateTime(const std::string &format = "%Y/%m/%d %H:%M:%S");
VISP_EXPORT double getMinTimeForUsleepCall();
VISP_EXPORT double measureTimeSecond();
VISP_EXPORT double measureTimeMs();
VISP_EXPORT double measureTimeMicros();
VISP_EXPORT void sleepMs(double t);
VISP_EXPORT int wait(double t0, double t);
VISP_EXPORT void wait(double t);
} // namespace vpTime

class VISP_EXPORT vpChrono
{
public:
  vpChrono();

  double getDurationMicros();
  double getDurationMs();
  double getDurationSeconds();
  void start(bool reset = true);
  void stop();

private:
  double m_durationMs;
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98) && (defined(_MSC_VER) && _MSC_VER >= 1900 /* VS2015 */ || !defined(_MSC_VER))
  std::chrono::steady_clock::time_point m_lastTimePoint;
#else
  double m_lastTimePoint;
#endif
};
END_VISP_NAMESPACE
#endif
