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
 * Test various Statistical Process Control methods.
 */

/*!
  \example testSPC.cpp
  \brief Test various Statistical Process Control methods.
*/

#include <iostream>
#include <string>

#include <visp3/core/vpStatisticalTestEWMA.h>
#include <visp3/core/vpStatisticalTestHinkley.h>
#include <visp3/core/vpStatisticalTestMeanAdjustedCUSUM.h>
#include <visp3/core/vpStatisticalTestShewhart.h>
#include <visp3/core/vpStatisticalTestSigma.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool initializeShewhartTest(const float &mean, const float &stdev, const bool &verbose, const std::string &testName, vpStatisticalTestShewhart &tester)
{
  const bool activateWECOrules = true;
  tester.init(activateWECOrules, vpStatisticalTestShewhart::CONST_ALL_WECO_ACTIVATED, mean, stdev);
  bool isInitOK = true;
  const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT_INIT = vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  vpStatisticalTestAbstract::vpMeanDriftType drift;
  unsigned int i = 0;
  while ((i < vpStatisticalTestShewhart::NB_DATA_SIGNAL - 1) && isInitOK) {
    drift = tester.testDownUpwardMeanDrift(mean);
    isInitOK = (drift == EXPECTED_DRIFT_INIT);
    if (isInitOK) {
      ++i;
    }
  }
  if (!isInitOK) {
    if (verbose) {
      std::cerr << "\t" << testName << " test initialization failed: " << std::endl;
      std::cerr << "\t\ts(t) = " << tester.getSignal() << std::endl;
      float limitDown = 0.f, limitUp = 0.f;
      tester.getLimits(limitDown, limitUp);
      std::cerr << "\t\tlim_- = " << limitDown << std::endl;
      std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
      std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
      std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT_INIT) << std::endl;
    }
  }
  return isInitOK;
}

void usage(const char *name)
{
  std::cout << "SYNOPSIS " << std::endl;
  std::cout << name << "[--mean <value>] [--stdev <value>] [-v, --verbose] [-h, --help]" << std::endl;
  std::cout << "\nOPTIONS" << std::endl;
  std::cout << "  --mean" << std::endl;
  std::cout << "    Permits to set the mean of the input signal." << std::endl;
  std::cout << std::endl;
  std::cout << "  --stdev" << std::endl;
  std::cout << "    Permits to set the standard deviation of the input signal." << std::endl;
  std::cout << std::endl;
  std::cout << "  -v, --verbose" << std::endl;
  std::cout << "    Activate verbose mode." << std::endl;
  std::cout << std::endl;
  std::cout << "  -h, --help" << std::endl;
  std::cout << "    Display the help." << std::endl;
  std::cout << std::endl;
}

bool getOptions(int argc, const char **argv, float &opt_mean, float &opt_stdev, bool &opt_verbose)
{
  int i = 1;
  while (i < argc) {
    std::string argname(argv[i]);
    if ((argname == "--mean") && ((i + 1) < argc)) {
      opt_mean = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((argname == "--stdev") && ((i + 1) < argc)) {
      opt_stdev = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((argname == "-v") || (argname == "--verbose")) {
      opt_verbose = true;
    }
    else if ((argname == "-h") || (argname == "--help")) {
      usage(argv[0]);
      return false;
    }
    else if ((argname == "-c") || (argname == "-d")) {
      // Arguments given by CTest by default, do nothing
    }
    else {
      usage(argv[0]);
      std::cerr << "Error: unrecognised argument \"" << argv[i] << "\"" << std::endl;
      return false;
    }
    ++i;
  }
  return true;
}

int main(int argc, const char **argv)
{
  float opt_mean = 0.f;
  float opt_stdev = 1.f;
  bool opt_verbose = false;

  bool isParsingOk = getOptions(argc, argv, opt_mean, opt_stdev, opt_verbose);
  if (!isParsingOk) {
    return EXIT_FAILURE;
  }

  bool success = true;

  // vpStatisticalTestEWMA tests
  {
    if (opt_verbose) {
      std::cout << "------ vpStatisticalTestEWMA tests ------" << std::endl;
    }
    const float alpha = 0.1f;
    vpStatisticalTestEWMA tester(alpha);

    // ---- Upward drift test ----
    {
      tester.init(alpha, opt_mean, opt_stdev);

      //     w(t = 1) >= mu + 3 sigma sqrt(alpha / (2 - alpha))
      // <=> alpha s(t=1) + (1 - alpha) mu >= mu + 3 sigma sqrt(alpha / (2 - alpha))
      // <=> s(t=1) >= (1 / alpha) (alpha mu + 3 sigma sqrt(alpha / (2 - alpha))
      float signal = (1.f / alpha) * (alpha * opt_mean + 3.f * opt_stdev * std::sqrt(alpha / (2.f - alpha)));
      signal += 0.5f; // To be sure we are greater than the threshold
      vpStatisticalTestAbstract::vpMeanDriftType drift = tester.testDownUpwardMeanDrift(signal);
      if (drift != vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Upward drift test failed: " << std::endl;
          std::cerr << "\tw(t) = " << tester.getWt() << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "\tlimit_up = " << limitUp << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Upward drift test succeeded." << std::endl;
      }
    }

    // ---- Downward drift test ----
    {
      tester.init(alpha, opt_mean, opt_stdev);
      //     w(t = 1) <= mu - 3 sigma sqrt(alpha / (2 - alpha))
      // <=> alpha s(t=1) + (1 - alpha) mu <= mu - 3 sigma sqrt(alpha / (2 - alpha))
      // <=> s(t=1) <= (1 / alpha) (alpha mu - 3 sigma sqrt(alpha / (2 - alpha))
      float signal = (1.f / alpha) * (alpha * opt_mean - 3.f * opt_stdev * std::sqrt(alpha / (2.f - alpha)));
      signal -= 0.5f; // To be sure we are greater than the threshold
      vpStatisticalTestAbstract::vpMeanDriftType drift = tester.testDownUpwardMeanDrift(signal);
      if (drift != vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Downward drift test failed: " << std::endl;
          std::cerr << "\tw(t) = " << tester.getWt() << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "\tlimit_down = " << limitDown << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Downward drift test succeeded." << std::endl;
      }
    }
  }

  // vpStatisticalTestHinkley tests
  {
    if (opt_verbose) {
      std::cout << "------ vpStatisticalTestHinkley tests ------" << std::endl;
    }

    const float h = 4.76f, k = 1.f;
    vpStatisticalTestHinkley tester(h, k, opt_mean, opt_stdev);
    const unsigned int HINKLEY_NB_DATA = 4;
    const float HINKLEY_SAMPLE = 2.f * opt_stdev;

    // Hinkley's test upward drift
    {
      const float HINKLEY_DATA[HINKLEY_NB_DATA] = { HINKLEY_SAMPLE, HINKLEY_SAMPLE, HINKLEY_SAMPLE, HINKLEY_SAMPLE };
      const vpStatisticalTestAbstract::vpMeanDriftType HINKLEY_EXPECTED_RES[HINKLEY_NB_DATA] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD };
      bool isTestOk = true;
      unsigned int id = 0;
      vpStatisticalTestAbstract::vpMeanDriftType drift;
      while ((id < HINKLEY_NB_DATA) && isTestOk) {
        drift = tester.testDownUpwardMeanDrift(HINKLEY_DATA[id]);
        isTestOk = (drift == HINKLEY_EXPECTED_RES[id]);
        if (isTestOk) {
          ++id;
        }
      }
      if (!isTestOk) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Upward drift test failed: " << std::endl;
          float Tk = tester.getTk(), Nk = tester.getNk();
          std::cerr << "T(k) = " << Tk << " | N(k) = " << Nk << " => S+(k) = " << Tk - Nk << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "lim_+ = " << limitUp << std::endl;
          std::cerr << "drift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
          std::cerr << "expected drift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(HINKLEY_EXPECTED_RES[id]) << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Upward drift test succeeded." << std::endl;
      }
    }

    // Hinkley's test downward drift
    {
      const float HINKLEY_DATA[HINKLEY_NB_DATA] = { -1.f * HINKLEY_SAMPLE, -1.f * HINKLEY_SAMPLE, -1.f * HINKLEY_SAMPLE, -1.f * HINKLEY_SAMPLE };
      const vpStatisticalTestAbstract::vpMeanDriftType HINKLEY_EXPECTED_RES[HINKLEY_NB_DATA] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD };
      bool isTestOk = true;
      unsigned int id = 0;
      vpStatisticalTestAbstract::vpMeanDriftType drift;
      while ((id < HINKLEY_NB_DATA) && isTestOk) {
        drift = tester.testDownUpwardMeanDrift(HINKLEY_DATA[id]);
        isTestOk = (drift == HINKLEY_EXPECTED_RES[id]);
        if (isTestOk) {
          ++id;
        }
      }
      if (!isTestOk) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Downward drift test failed: " << std::endl;
          float Sk = tester.getSk(), Mk = tester.getMk();
          std::cerr << "S(k) = " << Sk << " | M(k) = " << Mk << " => S+(k) = " << Mk - Sk << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "lim_- = " << limitDown << std::endl;
          std::cerr << "drift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
          std::cerr << "expected drift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(HINKLEY_EXPECTED_RES[id]) << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Downward drift test succeeded." << std::endl;
      }
    }
  }

  // vpStatisticalTestMeanAdjustedCUSUM tests
  {
    if (opt_verbose) {
      std::cout << "------ vpStatisticalTestMeanAdjustedCUSUM tests ------" << std::endl;
    }

    const float h = 4.76f, k = 1.f;
    vpStatisticalTestMeanAdjustedCUSUM tester(h, k);
    tester.init(h, k, opt_mean, opt_stdev);
    const unsigned int CUSUM_NB_DATA = 4;
    const float CUSUM_SAMPLE = 2.f * opt_stdev;

    // Mean adjusted CUSUM test upward drift
    {
      const float CUSUM_DATA[CUSUM_NB_DATA] = { CUSUM_SAMPLE, CUSUM_SAMPLE, CUSUM_SAMPLE, CUSUM_SAMPLE };
      const vpStatisticalTestAbstract::vpMeanDriftType CUSUM_EXPECTED_RES[CUSUM_NB_DATA] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD };
      bool isTestOk = true;
      unsigned int id = 0;
      vpStatisticalTestAbstract::vpMeanDriftType drift;
      while ((id < CUSUM_NB_DATA) && isTestOk) {
        drift = tester.testDownUpwardMeanDrift(CUSUM_DATA[id]);
        isTestOk = (drift == CUSUM_EXPECTED_RES[id]);
        if (isTestOk) {
          ++id;
        }
      }
      if (!isTestOk) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Upward drift test failed: " << std::endl;
          std::cerr << "\tS+(k) = " << tester.getTestSignalPlus() << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "\tlim_+ = " << limitUp << std::endl;
          std::cerr << "\tdrift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
          std::cerr << "\texpected drift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(CUSUM_EXPECTED_RES[id]) << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Upward drift test succeeded." << std::endl;
      }
    }

    // Mean adjusted CUSUM test upward drift
    {
      const float CUSUM_DATA[CUSUM_NB_DATA] = { -1.f * CUSUM_SAMPLE, -1.f * CUSUM_SAMPLE, -1.f * CUSUM_SAMPLE, -1.f * CUSUM_SAMPLE };
      const vpStatisticalTestAbstract::vpMeanDriftType CUSUM_EXPECTED_RES[CUSUM_NB_DATA] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD };
      bool isTestOk = true;
      unsigned int id = 0;
      vpStatisticalTestAbstract::vpMeanDriftType drift;
      while ((id < CUSUM_NB_DATA) && isTestOk) {
        drift = tester.testDownUpwardMeanDrift(CUSUM_DATA[id]);
        isTestOk = (drift == CUSUM_EXPECTED_RES[id]);
        if (isTestOk) {
          ++id;
        }
      }
      if (!isTestOk) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Downward drift test failed: " << std::endl;
          std::cerr << "\tS-(k) = " << tester.getTestSignalMinus() << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "\tlim_- = " << limitDown << std::endl;
          std::cerr << "\tdrift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
          std::cerr << "\texpected drift type : " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(CUSUM_EXPECTED_RES[id]) << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Downward drift test succeeded." << std::endl;
      }
    }
  }

  // vpStatisticalTestShewhart tests
  {
    if (opt_verbose) {
      std::cout << "------ vpStatisticalTestShewhart tests ------" << std::endl;
    }
    const bool activateWECOrules = true;
    vpStatisticalTestShewhart tester(activateWECOrules, vpStatisticalTestShewhart::CONST_ALL_WECO_ACTIVATED, opt_mean, opt_stdev);

    // Upward drift test
    {
      if (opt_verbose) {
        std::cout << "Upward drift tests" << std::endl;
      }

      // 3-sigma test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "3-sigma", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const float signal = 3.5f * opt_stdev;
          vpStatisticalTestAbstract::vpMeanDriftType drift = tester.testDownUpwardMeanDrift(signal);
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT = vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD;
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM = vpStatisticalTestShewhart::THREE_SIGMA_WECO;
          if ((drift != EXPECTED_DRIFT) || (alarm != EXPECTED_ALARM)) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\t3-sigma test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\t3-sigma test succeeded " << std::endl;
          }
        }
      }

      // 2-sigma test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "2-sigma", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const unsigned int NB_SAMPLES = 3;
          const float DATA[NB_SAMPLES] = { 2.75f * opt_stdev, 1.5f * opt_stdev, 2.5f * opt_stdev };
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT[NB_SAMPLES] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD };
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM[NB_SAMPLES] = { vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::TWO_SIGMA_WECO };
          vpStatisticalTestAbstract::vpMeanDriftType drift;
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          unsigned int i = 0;
          bool isTestOk = true;
          while ((i < NB_SAMPLES) && isTestOk) {
            drift = tester.testDownUpwardMeanDrift(DATA[i]);
            alarm = tester.getAlarm();
            isTestOk = ((drift == EXPECTED_DRIFT[i]) && (alarm == EXPECTED_ALARM[i]));
            if (isTestOk) {
              ++i;
            }
          }

          if (!isTestOk) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\t2-sigma test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT[i]) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM[i]) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\t2-sigma test succeeded " << std::endl;
          }
        }
      }

      // 1-sigma test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "1-sigma", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const unsigned int NB_SAMPLES = 5;
          const float DATA[NB_SAMPLES] = { 2.75f * opt_stdev, 1.5f * opt_stdev, 0.5f * opt_stdev, 1.5f * opt_stdev, 2.5f * opt_stdev };
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT[NB_SAMPLES] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD };
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM[NB_SAMPLES] = { vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::ONE_SIGMA_WECO };
          vpStatisticalTestAbstract::vpMeanDriftType drift;
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          unsigned int i = 0;
          bool isTestOk = true;
          while ((i < NB_SAMPLES) && isTestOk) {
            drift = tester.testDownUpwardMeanDrift(DATA[i]);
            alarm = tester.getAlarm();
            isTestOk = ((drift == EXPECTED_DRIFT[i]) && (alarm == EXPECTED_ALARM[i]));
            if (isTestOk) {
              ++i;
            }
          }

          if (!isTestOk) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\t1-sigma test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT[i]) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM[i]) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\t1-sigma test succeeded " << std::endl;
          }
        }
      }

      // Same-side test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "Same-side", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const unsigned int NB_SAMPLES = 8;
          const float DATA[NB_SAMPLES] = { 2.75f * opt_stdev, 0.5f * opt_stdev, 1.5f * opt_stdev, 0.5f * opt_stdev, 2.75f * opt_stdev, 0.5f * opt_stdev, 1.5f * opt_stdev, 0.5f * opt_stdev };
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT[NB_SAMPLES] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD };
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM[NB_SAMPLES] = { vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::SAME_SIDE_WECO };
          vpStatisticalTestAbstract::vpMeanDriftType drift;
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          unsigned int i = 0;
          bool isTestOk = true;
          while ((i < NB_SAMPLES) && isTestOk) {
            drift = tester.testDownUpwardMeanDrift(DATA[i]);
            alarm = tester.getAlarm();
            isTestOk = ((drift == EXPECTED_DRIFT[i]) && (alarm == EXPECTED_ALARM[i]));
            if (isTestOk) {
              ++i;
            }
          }

          if (!isTestOk) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\tSame-side test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT[i]) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM[i]) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\tSame-side test succeeded " << std::endl;
          }
        }
      }
    }

    // Downward drift test
    {
      if (opt_verbose) {
        std::cout << "Downward drift tests" << std::endl;
      }

      // 3-sigma test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "3-sigma", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const float signal = -3.5f * opt_stdev;
          vpStatisticalTestAbstract::vpMeanDriftType drift = tester.testDownUpwardMeanDrift(signal);
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT = vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD;
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM = vpStatisticalTestShewhart::THREE_SIGMA_WECO;
          if ((drift != EXPECTED_DRIFT) || (alarm != EXPECTED_ALARM)) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\t3-sigma test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_- = " << limitDown << std::endl;
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\t3-sigma test succeeded " << std::endl;
          }
        }
      }

      // 2-sigma test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "2-sigma", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const unsigned int NB_SAMPLES = 3;
          const float DATA[NB_SAMPLES] = { -2.75f * opt_stdev, -1.5f * opt_stdev, -2.5f * opt_stdev };
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT[NB_SAMPLES] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD };
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM[NB_SAMPLES] = { vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::TWO_SIGMA_WECO };
          vpStatisticalTestAbstract::vpMeanDriftType drift;
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          unsigned int i = 0;
          bool isTestOk = true;
          while ((i < NB_SAMPLES) && isTestOk) {
            drift = tester.testDownUpwardMeanDrift(DATA[i]);
            alarm = tester.getAlarm();
            isTestOk = ((drift == EXPECTED_DRIFT[i]) && (alarm == EXPECTED_ALARM[i]));
            if (isTestOk) {
              ++i;
            }
          }

          if (!isTestOk) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\t2-sigma test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_- = " << limitDown << std::endl;
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT[i]) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM[i]) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\t2-sigma test succeeded " << std::endl;
          }
        }
      }

      // 1-sigma test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "1-sigma", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const unsigned int NB_SAMPLES = 5;
          const float DATA[NB_SAMPLES] = { -2.75f * opt_stdev, -1.5f * opt_stdev, 1.5f * opt_stdev, -1.5f * opt_stdev, -2.5f * opt_stdev };
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT[NB_SAMPLES] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD };
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM[NB_SAMPLES] = { vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::ONE_SIGMA_WECO };
          vpStatisticalTestAbstract::vpMeanDriftType drift;
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          unsigned int i = 0;
          bool isTestOk = true;
          while ((i < NB_SAMPLES) && isTestOk) {
            drift = tester.testDownUpwardMeanDrift(DATA[i]);
            alarm = tester.getAlarm();
            isTestOk = ((drift == EXPECTED_DRIFT[i]) && (alarm == EXPECTED_ALARM[i]));
            if (isTestOk) {
              ++i;
            }
          }

          if (!isTestOk) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\t1-sigma test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_- = " << limitDown << std::endl;
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT[i]) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM[i]) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\t1-sigma test succeeded " << std::endl;
          }
        }
      }

      // Same-side test
      {
        bool isInitOK = initializeShewhartTest(opt_mean, opt_stdev, opt_verbose, "Same-side", tester);
        if (!isInitOK) {
          success = false;
        }
        else {
          const unsigned int NB_SAMPLES = 8;
          const float DATA[NB_SAMPLES] = { -2.75f * opt_stdev, -0.5f * opt_stdev, -1.5f * opt_stdev, -0.5f * opt_stdev, -2.75f * opt_stdev, -0.5f * opt_stdev, -1.5f * opt_stdev, -0.5f * opt_stdev };
          const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT[NB_SAMPLES] = { vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_NONE, vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD };
          const vpStatisticalTestShewhart::vpWecoRulesAlarm EXPECTED_ALARM[NB_SAMPLES] = { vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::NONE_WECO, vpStatisticalTestShewhart::SAME_SIDE_WECO };
          vpStatisticalTestAbstract::vpMeanDriftType drift;
          vpStatisticalTestShewhart::vpWecoRulesAlarm alarm = tester.getAlarm();
          unsigned int i = 0;
          bool isTestOk = true;
          while ((i < NB_SAMPLES) && isTestOk) {
            drift = tester.testDownUpwardMeanDrift(DATA[i]);
            alarm = tester.getAlarm();
            isTestOk = ((drift == EXPECTED_DRIFT[i]) && (alarm == EXPECTED_ALARM[i]));
            if (isTestOk) {
              ++i;
            }
          }

          if (!isTestOk) {
            success = false;
            if (opt_verbose) {
              std::cerr << "\tSame-side test failed: " << std::endl;
              std::vector<float> s = tester.getSignals();
              std::cerr << "\t\ts(t) = [ ";
              for (size_t i = 0; i < s.size(); ++i) {
                std::cerr << s[i] << " ";
              }
              std::cerr << "]" << std::endl;
              float limitDown = 0.f, limitUp = 0.f;
              tester.getLimits(limitDown, limitUp);
              std::cerr << "\t\tlim_+ = " << limitUp << std::endl;
              std::cerr << "\t\tlim_- = " << limitDown << std::endl;
              std::cerr << "\t\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
              std::cerr << "\t\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT[i]) << std::endl;
              std::cerr << "\t\tdetected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(alarm) << std::endl;
              std::cerr << "\t\texpected alarm = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(EXPECTED_ALARM[i]) << std::endl;
            }
          }
          else if (opt_verbose) {
            std::cout << "\tSame-side test succeeded " << std::endl;
          }
        }
      }
    }
  }

  // vpStatisticalTestSigma tests
  {
    if (opt_verbose) {
      std::cout << "------ vpStatisticalTestSigma tests ------" << std::endl;
    }
    const float h = 3.f;
    vpStatisticalTestSigma tester(h, opt_mean, opt_stdev);

    // Upward drift test
    {
      const float signal = 3.5f * opt_stdev;
      vpStatisticalTestAbstract::vpMeanDriftType drift = tester.testDownUpwardMeanDrift(signal);
      const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT = vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD;
      if (drift != EXPECTED_DRIFT) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Upward drift test failed: " << std::endl;
          std::cerr << "\ts(t) = " << tester.getSignal() << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "\tlim_+ = " << limitUp << std::endl;
          std::cerr << "\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
          std::cerr << "\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT) << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Upward drift test succeeded " << std::endl;
      }
    }

    // Downward drift test
    {
      const float signal = -3.5f * opt_stdev;
      vpStatisticalTestAbstract::vpMeanDriftType drift = tester.testDownUpwardMeanDrift(signal);
      const vpStatisticalTestAbstract::vpMeanDriftType EXPECTED_DRIFT = vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD;
      if (drift != EXPECTED_DRIFT) {
        success = false;
        if (opt_verbose) {
          std::cerr << "Downward drift test failed: " << std::endl;
          std::cerr << "\ts(t) = " << tester.getSignal() << std::endl;
          float limitDown = 0.f, limitUp = 0.f;
          tester.getLimits(limitDown, limitUp);
          std::cerr << "\tlim_- = " << limitDown << std::endl;
          std::cerr << "\tdetected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift) << std::endl;
          std::cerr << "\texpected drift = " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(EXPECTED_DRIFT) << std::endl;
        }
      }
      else if (opt_verbose) {
        std::cout << "Downward drift test succeeded " << std::endl;
      }
    }
  }

  if (success) {
    std::cout << "Test succeed" << std::endl;
  }
  else {
    std::cout << "Test failed" << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
