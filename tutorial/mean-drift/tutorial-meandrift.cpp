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

//! \example tutorial-meandrift.cpp

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpStatisticalTestEWMA.h>
#include <visp3/core/vpStatisticalTestHinkley.h>
#include <visp3/core/vpStatisticalTestMeanAdjustedCUSUM.h>
#include <visp3/core/vpStatisticalTestSigma.h>
#include <visp3/gui/vpPlot.h>

namespace TutorialMeanDrift
{
  /**
   * \brief Structure that permits to choose which process test to use.
   *
   */
typedef enum TypeTest
{
  HINLKEY_TYPE_TEST = 0, /*!< Use Hinkley to perform the tests.*/
  EWMA_TYPE_TEST = 1, /*!< Use Exponentially Weighted Moving Average to perform the tests.*/
  MEAN_ADJUSTED_CUSUM_TYPE_TEST = 2, /*!< Use mean adjusted Cumulative Sum to perform the tests.*/
  SIGMA_TYPE_TEST = 3, /*!< Simple test based on the comparisong with the standard deviation.*/
  COUNT_TYPE_TEST = 4, /*!< Number of different aavailable methods.*/
  UNKOWN_TYPE_TEST = COUNT_TYPE_TEST /*!< Unknown method.*/
}TypeTest;

/**
 * \brief Permit to cast a \b TypeTest object into a string, for display purpose.
 *
 * \param[in] choice The \b TypeTest object we want to know the name.
 * \return std::string The corresponding name.
 */
std::string typeTestToString(const TypeTest &type)
{
  std::string result;
  switch (type) {
  case HINLKEY_TYPE_TEST:
    result = "hinkley";
    break;
  case EWMA_TYPE_TEST:
    result = "ewma";
    break;
  case MEAN_ADJUSTED_CUSUM_TYPE_TEST:
    result = "cusum";
    break;
  case SIGMA_TYPE_TEST:
    result = "sigma";
    break;
  case UNKOWN_TYPE_TEST:
  default:
    result = "unknown-type-test";
    break;
  }
  return result;
}

/**
 * \brief Permit to cast a string into a \b TypeTest, to
 * cast a command line argument.
 *
 * \param[in] name The name of the process test the user wants to use.
 * \return TypeTest The corresponding \b TypeTest object.
 */
TypeTest typeTestFromString(const std::string &name)
{
  TypeTest result = UNKOWN_TYPE_TEST;
  unsigned int count = static_cast<unsigned int>(COUNT_TYPE_TEST);
  unsigned int id = 0;
  bool hasNotFound = true;
  while ((id < count) && hasNotFound) {
    TypeTest temp = static_cast<TypeTest>(id);
    if (typeTestToString(temp) == name) {
      result = temp;
      hasNotFound = false;
    }
    ++id;
  }
  return result;
}

/**
 * \brief Get the list of available \b TypeTest objects that are handled.
 *
 * \param[in] prefix The prefix that should be placed before the list.
 * \param[in] sep The separator between each element of the list.
 * \param[in] suffix The suffix that should terminate the list.
 * \return std::string The list of handled type of process tests, presented as a string.
 */
std::string getAvailableTypeTest(const std::string &prefix = "<", const std::string &sep = " , ",
                                 const std::string &suffix = ">")
{
  std::string msg(prefix);
  unsigned int count = static_cast<unsigned int>(COUNT_TYPE_TEST);
  unsigned int lastId = count - 1;
  for (unsigned int i = 0; i < lastId; i++) {
    msg += typeTestToString(static_cast<TypeTest>(i)) + sep;
  }
  msg += typeTestToString(static_cast<TypeTest>(lastId)) + suffix;
  return msg;
}

/**
 * \brief Structure that contains the parameters of the different algorithms.
 */
typedef struct ParametersForAlgo
{
  unsigned int m_global_nbsamples; /*!< Number of samples to compute the mean and stdev, common to all the algorithms.*/
  float m_cusum_h; /*!< Alarm factor for the mean adjusted CUSUM test.*/
  float m_cusum_k; /*!< Detection factor for the mean adjusted CUSUM test.*/
  float m_ewma_alpha; /*!< Forgetting factor for the EWMA test.*/
  float m_hinkley_alpha; /*!< Alarm threshold for the Hinkley's test. */
  float m_hinkley_delta; /*!< Detection threshold for the Hinkley's test. */
  float m_sigma_h; /*!< Alarm factor for the sigma test.*/

  ParametersForAlgo()
    : m_global_nbsamples(30)
    , m_cusum_h(4.76f)
    , m_cusum_k(1.f)
    , m_ewma_alpha(0.1f)
    , m_hinkley_alpha(4.76f)
    , m_hinkley_delta(1.f)
    , m_sigma_h(3.f)
  { }
}ParametersForAlgo;
}

int testOnSynthetic(const TutorialMeanDrift::TypeTest &type, const TutorialMeanDrift::ParametersForAlgo parameters,
                    const float &mean, const float &mean_drift, const float &stdev)
{
  const float dt = 10.f; // Emulate a 10ms period

  vpPlot plotter(1);
  plotter.initGraph(0, 1);
  plotter.setTitle(0, "Evolution of the signal");
  plotter.setUnitX(0, "Frame ID");
  plotter.setUnitY(0, "No units");

  unsigned int idFrame = 0;
  vpStatisticalTestAbstract *p_test = nullptr;
  switch (type) {
  case TutorialMeanDrift::EWMA_TYPE_TEST:
    p_test = new vpStatisticalTestEWMA(parameters.m_ewma_alpha);
    break;
  case TutorialMeanDrift::HINLKEY_TYPE_TEST:
    p_test = new vpStatisticalTestHinkley(parameters.m_hinkley_alpha, parameters.m_hinkley_delta, parameters.m_global_nbsamples);
    break;
  case TutorialMeanDrift::MEAN_ADJUSTED_CUSUM_TYPE_TEST:
    p_test = new vpStatisticalTestMeanAdjustedCUSUM(parameters.m_cusum_h, parameters.m_cusum_k, parameters.m_global_nbsamples);
    break;
  case TutorialMeanDrift::SIGMA_TYPE_TEST:
    p_test = new vpStatisticalTestSigma(parameters.m_sigma_h, parameters.m_global_nbsamples);
    break;
  default:
    throw(vpException(vpException::badValue, TutorialMeanDrift::typeTestToString(type) + " is not handled."));
    break;
  }

  float signal;
  std::cout << "Actual mean of the input signal: " << mean << std::endl;
  std::cout << "Actual stdev of the input signal: " << stdev << std::endl;
  std::cout << "Mean drift of the input signal: " << mean_drift << std::endl;

  // Initial computation of the mean and stdev of the input signal
  for (unsigned int i = 0; i < parameters.m_global_nbsamples; ++i) {
    vpGaussRand rndGen(stdev, mean, static_cast<float>(idFrame) * dt);
    signal = rndGen();
    p_test->testDownUpwardMeanShift(signal);
    ++idFrame;
  }
  std::cout << "Estimated mean of the input signal: " << p_test->getMean() << std::endl;
  std::cout << "Estimated stdev of the input signal: " << p_test->getStdev() << std::endl;

  float mean_eff = mean;
  bool hasToRun = true;
  vpStatisticalTestAbstract::vpMeanDriftType drift_type = vpStatisticalTestAbstract::NO_MEAN_DRIFT;
  while (hasToRun) {
    vpGaussRand rndGen(stdev, mean_eff, static_cast<float>(idFrame) * dt);
    signal = rndGen();
    plotter.plot(0, 0, idFrame - parameters.m_global_nbsamples, signal);
    drift_type = p_test->testDownUpwardMeanShift(signal);
    if (drift_type != vpStatisticalTestAbstract::NO_MEAN_DRIFT) {
      hasToRun = false;
    }
    else {
      mean_eff += mean_drift;
      ++idFrame;
    }
  }
  std::cout << "Test failed at frame: " << idFrame - parameters.m_global_nbsamples << std::endl;
  std::cout << "Type of mean drift: " << vpStatisticalTestAbstract::vpMeanDriftTypeToString(drift_type) << std::endl;
  std::cout << "Last signal value: " << signal << std::endl;
  if (type == TutorialMeanDrift::EWMA_TYPE_TEST) {
    vpStatisticalTestEWMA *p_testEwma = dynamic_cast<vpStatisticalTestEWMA *>(p_test);
    std::cout << "\tw(t) = " << p_testEwma->getWt() << std::endl;
  }
  else if (type == TutorialMeanDrift::MEAN_ADJUSTED_CUSUM_TYPE_TEST) {
    vpStatisticalTestMeanAdjustedCUSUM *p_testCusum = dynamic_cast<vpStatisticalTestMeanAdjustedCUSUM *>(p_test);
    std::cout << "\tLower cusum = " << p_testCusum->getTestSignalMinus() << std::endl;
    std::cout << "\tUpper cusum = " << p_testCusum->getTestSignalPlus() << std::endl;
  }
  float limitDown = 0.f, limitUp = 0.f;
  p_test->getLimits(limitDown, limitUp);
  std::cout << "\tLimit down = " << limitDown << std::endl;
  std::cout << "\tLimit up = " << limitUp << std::endl;
  std::cout << "End of test on synthetic data. Press enter to leave." << std::endl;
  std::cin.get();
  return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
  TutorialMeanDrift::TypeTest opt_typeTest = TutorialMeanDrift::EWMA_TYPE_TEST;
  TutorialMeanDrift::ParametersForAlgo parameters;
  float opt_mean = 6.f;
  float opt_meandrift = 0.1f;
  float opt_stdev = 1.f;

  int i = 1;
  while (i < argc) {
    if ((std::string(argv[i]) == "--test") && ((i + 1) < argc)) {
      opt_typeTest = TutorialMeanDrift::typeTestFromString(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--nb-samples") && ((i + 1) < argc)) {
      parameters.m_global_nbsamples = std::atoi(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--mean") && ((i + 1) < argc)) {
      opt_mean = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--mean-drift") && ((i + 1) < argc)) {
      opt_meandrift = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--stdev") && ((i + 1) < argc)) {
      opt_stdev = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--cusum-h") && ((i + 1) < argc)) {
      parameters.m_cusum_h = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--cusum-k") && ((i + 1) < argc)) {
      parameters.m_cusum_k = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--ewma-alpha") && ((i + 1) < argc)) {
      parameters.m_ewma_alpha = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--hinkley-alpha") && ((i + 1) < argc)) {
      parameters.m_hinkley_alpha = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--hinkley-delta") && ((i + 1) < argc)) {
      parameters.m_hinkley_delta = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--sigma-h") && ((i + 1) < argc)) {
      parameters.m_sigma_h = std::atof(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--test <type>]"
        << " [--nb-samples <value>]"
        << " [--mean <value>]"
        << " [--mean-drift <value>]"
        << " [--stdev <value>]"
        << " [--cusum-h <value>]"
        << " [--cusum-k <value>]"
        << " [--ewma-alpha <value ]0; 1[>]"
        << " [--hinkley-alpha <]0; inf[>]"
        << " [--hinkley-delta <]0; inf[>]"
        << " [--sigma-h <value>]"
        << " [--help,-h]" << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --test <type-name>" << std::endl
        << "      Type of test to perform on the data." << std::endl
        << "      Available values: " << TutorialMeanDrift::getAvailableTypeTest() << std::endl
        << std::endl
        << "  --nb-samples <value>" << std::endl
        << "      Number of samples to compute the mean and standard deviation of the monitored signal." << std::endl
        << "      Default: " << parameters.m_global_nbsamples << std::endl
        << std::endl
        << "  --mean <value>" << std::endl
        << "      Mean of the signal." << std::endl
        << "      Default: " << opt_mean<< std::endl
        << std::endl
        << "  --mean-drift <value>" << std::endl
        << "      Mean drift for the synthetic data." << std::endl
        << "      Default: " << opt_meandrift << std::endl
        << std::endl
        << "  --stdev <value>" << std::endl
        << "      Standard deviation of the signal." << std::endl
        << "      Default: " << opt_stdev << std::endl
        << std::endl
        << "  --cusum-h <value>" << std::endl
        << "      The alarm factor that permits to the CUSUM test to determine when the process is out of control" << std::endl
        << "      from the standard deviation of the signal." << std::endl
        << "      Default: " << parameters.m_cusum_h << std::endl
        << std::endl
        << "  --cusum-k <value>" << std::endl
        << "      The factor that permits to determine the slack of the CUSUM test, " << std::endl
        << "      i.e. the minimum value of the jumps we want to detect, from the standard deviation of the signal." << std::endl
        << "      Default: " << parameters.m_cusum_k << std::endl
        << std::endl
        << "  --ewma-alpha <value ]0; 1[>" << std::endl
        << "      Forgetting factor for the Exponential Weighted Moving Average (EWMA)." << std::endl
        << "      Default: " << parameters.m_ewma_alpha << std::endl
        << std::endl
        << "  --hinkley-alpha <value ]0; inf[>" << std::endl
        << "      The alarm threshold indicating that a mean drift occurs for the Hinkley's test." << std::endl
        << "      Default: " << parameters.m_hinkley_alpha << std::endl
        << std::endl
        << "  --hinkley-delta <value>" << std::endl
        << "      Detection threshold indicating minimal magnitude we want to detect for the Hinkley's test." << std::endl
        << "      Default: " << parameters.m_hinkley_delta << std::endl
        << std::endl
        << "  --sigma-h <value>" << std::endl
        << "      The alarm factor of the sigma test." << std::endl
        << "      Default: " << parameters.m_sigma_h << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "      Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
    else {
      std::cout << "\nERROR " << std::endl << "  Unknown option " << argv[i] << std::endl;
      return EXIT_FAILURE;
    }
    ++i;
  }

  return testOnSynthetic(opt_typeTest, parameters, opt_mean, opt_meandrift, opt_stdev);
}
