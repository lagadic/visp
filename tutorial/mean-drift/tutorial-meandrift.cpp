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

#include <cstring> //std::memcpy

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpStatisticalTestEWMA.h>
#include <visp3/core/vpStatisticalTestHinkley.h>
#include <visp3/core/vpStatisticalTestMeanAdjustedCUSUM.h>
#include <visp3/core/vpStatisticalTestShewhart.h>
#include <visp3/core/vpStatisticalTestSigma.h>
#include <visp3/gui/vpPlot.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if defined(VISP_HAVE_DISPLAY)
namespace TutorialMeanDrift
{
  //! [Enum_For_Test_Choice]
  /**
   * \brief Enumeration that permits to choose which process test to use.
   *
   */
typedef enum TypeTest
{
  HINLKEY_TYPE_TEST = 0, /*!< Use Hinkley's test.*/
  EWMA_TYPE_TEST = 1, /*!< Use Exponentially Weighted Moving Average to perform the tests.*/
  MEAN_ADJUSTED_CUSUM_TYPE_TEST = 2, /*!< Use mean adjusted Cumulative Sum to perform the tests.*/
  SHEWHART_TYPE_TEST = 3, /*!< Shewhart's test.*/
  SIGMA_TYPE_TEST = 4, /*!< Simple test based on the comparisong with the standard deviation.*/
  COUNT_TYPE_TEST = 5, /*!< Number of different aavailable methods.*/
  UNKOWN_TYPE_TEST = COUNT_TYPE_TEST /*!< Unknown method.*/
}TypeTest;
//! [Enum_For_Test_Choice]

/**
 * \brief Permit to cast a \b TypeTest object into a string, for display purpose.
 *
 * \param[in] type The \b TypeTest object we want to know the name.
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
  case SHEWHART_TYPE_TEST:
    result = "shewhart";
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
 * \brief Cast a number type into a string.
 *
 * \tparam T Type of number.
 * \param[in] number The number to cast.
 * \return std::string The corresponding string.
 */
template <typename T>
std::string numberToString(const T &number)
{
  std::stringstream ss;
  ss << number;
  return ss.str();
}

/**
 * \brief Cast a boolean into a string.
 *
 * \param[in] boolean The boolean to cast.
 * \return std::string The corresponding string.
 */
std::string boolToString(const bool &boolean)
{
  if (boolean) {
    return "true";
  }
  else {
    return "false";
  }
}

/**
 * \brief Write the WECO's rules used in the Shewhart's test in human readable format.
 *
 * \param[in] rules The array indicating which WECO's rules are used.
 * \param[in] prefix The first character(s) delimiting the array in the string.
 * \param[in] suffix The last character(s) delimiting the array in the string.
 * \param[in] sep The separator character(s).
 * \return std::string The corresponding string.
 */
std::string wecoRulesToString(const bool rules[vpStatisticalTestShewhart::COUNT_WECO - 1], const std::string &prefix = "[", const std::string &suffix = "]", const std::string &sep = " , ")
{
  std::string rulesAsString = prefix;
  for (unsigned int i = 0; i < vpStatisticalTestShewhart::COUNT_WECO - 2; ++i) {
    if (rules[i]) {
      rulesAsString += "ON";
    }
    else {
      rulesAsString += "OFF";
    }
    rulesAsString += sep;
  }
  if (rules[vpStatisticalTestShewhart::COUNT_WECO - 2]) {
    rulesAsString += "ON";
  }
  else {
    rulesAsString += "OFF";
  }
  rulesAsString += suffix;
  return rulesAsString;
}

/**
 * \brief Array that sets all the types of mean drift to deactivated.
 */
const bool CONST_ALL_ALARM_OFF[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT] = { false, false, false, false };

/**
 * \brief Array that sets all the types of mean drift to activated.
 */
const bool CONST_ALL_ALARM_ON[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT] = { true, true, true, true };

/**
 * \brief Cast  a vector of string into an array of boolean activating / deactivating
 * the mean drift alarms.
 *
 * \param[in] names The names of the alarms to set.
 * \param[out] array The corresponding array of boolean.
 */
void vectorOfStringToMeanDriftTypeArray(const std::vector<std::string> &names, bool array[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT])
{
  std::memcpy(array, CONST_ALL_ALARM_OFF, vpStatisticalTestAbstract::MEAN_DRIFT_COUNT * sizeof(bool));
  size_t nbNames = names.size();
  for (size_t i = 0; i < nbNames; ++i) {
    vpStatisticalTestAbstract::vpMeanDriftType alarmToActivate = vpStatisticalTestAbstract::vpMeanDriftTypeFromString(names[i]);
    std::cout << "alarm[" << names[i] << "] (i.e. " << static_cast<unsigned int>(alarmToActivate) << ") set to true"  << std::endl;
    array[static_cast<unsigned int>(alarmToActivate)] = true;
    if (alarmToActivate == vpStatisticalTestAbstract::MEAN_DRIFT_BOTH) {
      array[vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD] = true;
      array[vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD] = true;
    }
  }
  if (array[vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD] || array[vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD]) {
    array[vpStatisticalTestAbstract::MEAN_DRIFT_BOTH] = true;
  }
}

/**
 * \brief Cast an array of boolean (de)activating the mean drift alarms into
 * the corresponding vector of strings.
 *
 * \param[in] array The array of boolean indicating which alarm are set.
 * \return std::vector<std::string> The corresponding vector of names of alarms.
 */
std::vector<std::string> meanDriftArrayToVectorOfString(const bool array[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT])
{
  std::vector<std::string> listActivatedAlarms;
  unsigned int nbTypeTests = static_cast<unsigned int>(vpStatisticalTestAbstract::MEAN_DRIFT_COUNT);
  for (unsigned int id = 0; id < nbTypeTests; ++id) {
    if (array[id]) {
      vpStatisticalTestAbstract::vpMeanDriftType test = static_cast<vpStatisticalTestAbstract::vpMeanDriftType>(id);
      std::string testName = vpStatisticalTestAbstract::vpMeanDriftTypeToString(test);
      listActivatedAlarms.push_back(testName);
    }
  }
  return listActivatedAlarms;
}

/**
 * \brief Cast an array of boolean (de)activating the mean drift alarms into
 * a single string listing all the alarms.
 *
 * \param[in] array The array of boolean indicating which alarm are set.
 * \param[in] prefix The returned string prefix.
 * \param[in] sep The returned string separator.
 * \param[in] suffix The returned string suffix.
 * \return std::string The corresponding string listing the names of alarms.
 */
std::string meanDriftArrayToString(const bool array[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT],
                                  const std::string &prefix = "[", const std::string &sep = " , ",
                                  const std::string &suffix = "]")
{
  std::vector<std::string> listActivatedAlarms = meanDriftArrayToVectorOfString(array);
  std::string result = prefix;
  size_t nbTestActivated = listActivatedAlarms.size();
  if (nbTestActivated == 0) {
    return prefix + " " + suffix;
  }
  for (size_t i = 0; i < nbTestActivated - 1; ++i) {
    result += listActivatedAlarms[i] + sep;
  }
  result += listActivatedAlarms[nbTestActivated - 1] + suffix;
  return result;
}

/**
 * \brief Indicate how many alarms are set.
 *
 * \param[in] array The array of boolean indicating which alarms are set.
 * \return unsigned int The number of alarms that are set.
 */
unsigned int meanDriftArrayToNbActivated(const bool array[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT])
{
  unsigned int nbActivated = 0;
  unsigned int nbTypeAlarms = static_cast<unsigned int>(vpStatisticalTestAbstract::MEAN_DRIFT_COUNT);
  for (unsigned int id = 0; id < nbTypeAlarms; ++id) {
    if (array[id]) {
      ++nbActivated;
    }
  }
  return nbActivated;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
//! [Structure_Parameters]
/**
 * \brief Structure that contains the parameters of the different algorithms.
 */
typedef struct ParametersForAlgo
{
  unsigned int m_test_nbsamples; /*!< Number of samples to compute the mean and stdev, common to all the algorithms.*/
  bool m_test_activatedalarms[vpStatisticalTestAbstract::MEAN_DRIFT_COUNT]; /*!< Flag is true for a type of alarm that must be considered, false otherwise.*/
  unsigned int m_test_nbactivatedalarms; /*!< Number of activated alarms.*/
  float m_cusum_h; /*!< Alarm factor for the mean adjusted CUSUM test.*/
  float m_cusum_k; /*!< Detection factor for the mean adjusted CUSUM test.*/
  float m_ewma_alpha; /*!< Forgetting factor for the EWMA test.*/
  float m_hinkley_alpha; /*!< Alarm threshold for the Hinkley's test. */
  float m_hinkley_delta; /*!< Detection threshold for the Hinkley's test. */
  bool m_hinkley_computealphadelta; /*!< If true, compute alpha and delta of the Hinkley's using the stdev of the signal.*/
  float m_hinkley_h; /*!< Alarm factor permitting to compute alpha from the standard deviation of the signal.*/
  float m_hinkley_k; /*!< Detection factor permitting to compute delta from the standard deviation of the signal.*/
  bool m_shewhart_useWECO; /*!< If true, use the WECO rules for additionnal subtests for Shewhart's test.*/
  bool m_shewhart_rules[vpStatisticalTestShewhart::COUNT_WECO - 1]; /*!< Rules for the Shewart's test. True activate a WECO rule, false deactivate it.*/
  float m_sigma_h; /*!< Alarm factor for the sigma test.*/

  ParametersForAlgo()
    : m_test_nbsamples(30)
    , m_cusum_h(4.76f)
    , m_cusum_k(1.f)
    , m_ewma_alpha(0.1f)
    , m_hinkley_alpha(4.76f)
    , m_hinkley_delta(1.f)
    , m_hinkley_computealphadelta(false)
    , m_hinkley_h(4.76f)
    , m_hinkley_k(1.f)
    , m_shewhart_useWECO(false)
    , m_sigma_h(3.f)
  {
    std::memcpy(m_shewhart_rules, vpStatisticalTestShewhart::CONST_ALL_WECO_ACTIVATED, (vpStatisticalTestShewhart::COUNT_WECO - 1) * sizeof(bool));
    memcpy(m_test_activatedalarms, CONST_ALL_ALARM_ON, vpStatisticalTestAbstract::MEAN_DRIFT_COUNT * sizeof(bool));
    m_test_activatedalarms[vpStatisticalTestAbstract::MEAN_DRIFT_NONE] = false;
    m_test_nbactivatedalarms = meanDriftArrayToNbActivated(m_test_activatedalarms);
  }
}ParametersForAlgo;
//! [Structure_Parameters]
}
#endif // DOXYGEN_SHOULD_SKIP_THIS

int testOnSynthetic(const TutorialMeanDrift::TypeTest &type, const TutorialMeanDrift::ParametersForAlgo parameters,
                    const float &mean, const float &mean_drift, const float &stdev)
{
  const float dt = 10.f; // Emulate a 10ms period

  //! [Plot_Init]
  vpPlot plotter(1);
  plotter.initGraph(0, 1);
  plotter.setTitle(0, "Evolution of the signal");
  plotter.setUnitX(0, "Frame ID");
  plotter.setUnitY(0, "No units");
  //! [Plot_Init]

  //! [Test_Creat]
  unsigned int idFrame = 0;
  vpStatisticalTestAbstract *p_test = nullptr;
  switch (type) {
  case TutorialMeanDrift::EWMA_TYPE_TEST:
    p_test = new vpStatisticalTestEWMA(parameters.m_ewma_alpha);
    break;
  case TutorialMeanDrift::HINLKEY_TYPE_TEST:
    p_test = new vpStatisticalTestHinkley(parameters.m_hinkley_alpha, parameters.m_hinkley_delta, parameters.m_test_nbsamples);
    break;
  case TutorialMeanDrift::MEAN_ADJUSTED_CUSUM_TYPE_TEST:
    p_test = new vpStatisticalTestMeanAdjustedCUSUM(parameters.m_cusum_h, parameters.m_cusum_k, parameters.m_test_nbsamples);
    break;
  case TutorialMeanDrift::SHEWHART_TYPE_TEST:
    p_test = new vpStatisticalTestShewhart(parameters.m_shewhart_useWECO, parameters.m_shewhart_rules, parameters.m_test_nbsamples);
    break;
  case TutorialMeanDrift::SIGMA_TYPE_TEST:
    p_test = new vpStatisticalTestSigma(parameters.m_sigma_h, parameters.m_test_nbsamples);
    break;
  default:
    throw(vpException(vpException::badValue, TutorialMeanDrift::typeTestToString(type) + " is not handled."));
    break;
  }

  if ((type == TutorialMeanDrift::HINLKEY_TYPE_TEST) && parameters.m_hinkley_computealphadelta) {
    // Initialization of Hinkley's test in automatic mode
    delete p_test;
    p_test = new vpStatisticalTestHinkley(parameters.m_hinkley_h, parameters.m_hinkley_k, true, parameters.m_test_nbsamples);
  }
  //! [Test_Creat]

  float signal;

  //! [Test_Init]
  // Initial computation of the mean and stdev of the input signal
  for (unsigned int i = 0; i < parameters.m_test_nbsamples; ++i) {
    vpGaussRand rndGen(stdev, mean, static_cast<long>(idFrame * dt));
    signal = static_cast<float>(rndGen());
    p_test->testDownUpwardMeanDrift(signal);
    ++idFrame;
  }
  //! [Test_Init]

  std::cout << "Estimated mean of the input signal: " << p_test->getMean() << std::endl;
  std::cout << "Estimated stdev of the input signal: " << p_test->getStdev() << std::endl;

  //! [Loop_Monitor]
  float mean_eff = mean;
  bool hasToRun = true;
  vpStatisticalTestAbstract::vpMeanDriftType drift_type = vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  while (hasToRun) {
    vpGaussRand rndGen(stdev, mean_eff, static_cast<long>(idFrame * dt));
    signal = static_cast<float>(rndGen());
    plotter.plot(0, 0, idFrame - parameters.m_test_nbsamples, signal);
    drift_type = p_test->testDownUpwardMeanDrift(signal);
    if ((drift_type != vpStatisticalTestAbstract::MEAN_DRIFT_NONE) && (parameters.m_test_activatedalarms[drift_type])) {
      hasToRun = false;
    }
    else {
      mean_eff += mean_drift;
      ++idFrame;
    }
  }
  //! [Loop_Monitor]

  //! [Failure_Debrief]
  std::cout << "Test failed at frame: " << idFrame - parameters.m_test_nbsamples << std::endl;
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
  else if (type==TutorialMeanDrift::SHEWHART_TYPE_TEST) {
    vpStatisticalTestShewhart *p_testShewhart = dynamic_cast<vpStatisticalTestShewhart *>(p_test);
    std::vector<float> signal = p_testShewhart->getSignals();
    size_t nbSignal = signal.size();
    std::cout << "Signal history = [ ";
    for (size_t i = 0; i < nbSignal; ++i) {
      std::cout << signal[i] << " ";
    }
    std::cout << "]" << std::endl;
    std::cout << "\tWECO alarm type = " << vpStatisticalTestShewhart::vpWecoRulesAlarmToString(p_testShewhart->getAlarm()) << std::endl;
  }
  else if (type == TutorialMeanDrift::HINLKEY_TYPE_TEST) {
    vpStatisticalTestHinkley *p_hinkley = dynamic_cast<vpStatisticalTestHinkley *>(p_test);
    float Mk = p_hinkley->getMk();
    float Nk = p_hinkley->getNk();
    float Sk = p_hinkley->getSk();
    float Tk = p_hinkley->getTk();
    std::cout << "S+(t) = " << Tk - Nk <<std::endl;
    std::cout << "S-(t) = " << Mk - Sk <<std::endl;
  }
  float limitDown = 0.f, limitUp = 0.f;
  p_test->getLimits(limitDown, limitUp);
  std::cout << "\tLimit down = " << limitDown << std::endl;
  std::cout << "\tLimit up = " << limitUp << std::endl;
  //! [Failure_Debrief]
  std::cout << "End of test on synthetic data. Press enter to leave." << std::endl;
  std::cin.get();
  delete p_test;
  return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
  TutorialMeanDrift::TypeTest opt_typeTest = TutorialMeanDrift::MEAN_ADJUSTED_CUSUM_TYPE_TEST;
  TutorialMeanDrift::ParametersForAlgo parameters;
  float opt_mean = 6.f;
  float opt_meandrift = 0.f;
  float opt_stdev = 2.f;

  int i = 1;
  while (i < argc) {
    if ((std::string(argv[i]) == "--test") && ((i + 1) < argc)) {
      opt_typeTest = TutorialMeanDrift::typeTestFromString(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--nb-samples") && ((i + 1) < argc)) {
      parameters.m_test_nbsamples = std::atoi(argv[i + 1]);
      ++i;
    }
    else if ((std::string(argv[i]) == "--mean") && ((i + 1) < argc)) {
      opt_mean = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--mean-drift") && ((i + 1) < argc)) {
      opt_meandrift = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--stdev") && ((i + 1) < argc)) {
      opt_stdev = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--alarms")) {
      unsigned int nbArguments = 0;
      std::vector<std::string> alarmNames;
      bool hasNotFoundNextParams = true;
      for (int j = 1; ((i + j) < argc) && hasNotFoundNextParams; ++j) {
        std::string candidate(argv[i+j]);
        if (candidate.find("--") != std::string::npos) {
          // This is the next command line parameter
          hasNotFoundNextParams = false;
        }
        else {
          // This is a name
          alarmNames.push_back(candidate);
          ++nbArguments;
        }
      }
      TutorialMeanDrift::vectorOfStringToMeanDriftTypeArray(alarmNames, parameters.m_test_activatedalarms);
      parameters.m_test_nbactivatedalarms = TutorialMeanDrift::meanDriftArrayToNbActivated(parameters.m_test_activatedalarms);
      i += nbArguments;
    }
    else if ((std::string(argv[i]) == "--cusum-h") && ((i + 1) < argc)) {
      parameters.m_cusum_h = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--cusum-k") && ((i + 1) < argc)) {
      parameters.m_cusum_k = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--ewma-alpha") && ((i + 1) < argc)) {
      parameters.m_ewma_alpha = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--hinkley-alpha") && ((i + 1) < argc)) {
      parameters.m_hinkley_alpha = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--hinkley-delta") && ((i + 1) < argc)) {
      parameters.m_hinkley_delta = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if (std::string(argv[i]) == "--hinkley-compute") {
      parameters.m_hinkley_computealphadelta = true;
    }
    else if ((std::string(argv[i]) == "--hinkley-h") && ((i + 1) < argc)) {
      parameters.m_hinkley_h = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--hinkley-k") && ((i + 1) < argc)) {
      parameters.m_hinkley_k = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--shewhart-rules") && (i + vpStatisticalTestShewhart::COUNT_WECO - 1 < argc)) {
      for (int j = 0; j < vpStatisticalTestShewhart::COUNT_WECO - 1; ++j) {
        std::string argument = std::string(argv[i + 1 + j]);
        if ((argument.find("on") != std::string::npos) || (argument.find("ON") != std::string::npos)) {
          parameters.m_shewhart_rules[j] = true;
        }
        else {
          parameters.m_shewhart_rules[j] = false;
        }
      }
      i += vpStatisticalTestShewhart::COUNT_WECO - 1;
    }
    else if (std::string(argv[i]) == "--shewhart-weco") {
      parameters.m_shewhart_useWECO = true;
    }
    else if ((std::string(argv[i]) == "--sigma-h") && ((i + 1) < argc)) {
      parameters.m_sigma_h = static_cast<float>(std::atof(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--test <type>]"
        << " [--nb-samples <value>]"
        << " [--alarms <name_1 ... name_n>]"
        << " [--mean <value>]"
        << " [--mean-drift <value>]"
        << " [--stdev <value>]"
        << " [--cusum-h <value>]"
        << " [--cusum-k <value>]"
        << " [--ewma-alpha <value ]0; 1[>]"
        << " [--hinkley-alpha <]0; inf[>]"
        << " [--hinkley-delta <]0; inf[>]"
        << " [--hinkley-compute]"
        << " [--hinkley-h <]0; inf[>]"
        << " [--hinkley-k <]0; inf[>]"
        << " [--shewhart-rules <3-sigma:{on|off} 2-sigma:{on|off} 1-sigma:{on|off} same-side:{on|off}>"
        << " [--shewhart-weco]"
        << " [--sigma-h <value>]"
        << " [--help,-h]" << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --test <type-name>" << std::endl
        << "      Type of test to perform on the data." << std::endl
        << "      Available values: " << TutorialMeanDrift::getAvailableTypeTest() << std::endl
        << std::endl
        << "  --nb-samples <value>" << std::endl
        << "      Number of samples to compute the mean and standard deviation of the monitored signal." << std::endl
        << "      Default: " << parameters.m_test_nbsamples << std::endl
        << std::endl
        << "  --alarms <name_1 .. name_n>" << std::endl
        << "      Set the mean drift alarms to monitor." << std::endl
        << "      Default: " << TutorialMeanDrift::meanDriftArrayToString(parameters.m_test_activatedalarms) << std::endl
        << "      Available: " << vpStatisticalTestAbstract::getAvailableMeanDriftType() << std::endl
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
        << "  --hinkley-compute" << std::endl
        << "      If set, the Hinkley's test will compute the alarm and detection thresholds" << std::endl
        << "      from the standard deviation of the input signal." << std::endl
        << "      Default: disabled" << std::endl
        << std::endl
        << "  --hinkley-h <value>" << std::endl
        << "      Alarm factor permitting to compute the alarm threshold for the Hinkley's test." << std::endl
        << "      Default: " << parameters.m_hinkley_h << std::endl
        << std::endl
        << "  --hinkley-k <value>" << std::endl
        << "      Detection factor permitting to compute the Detection threshold for the Hinkley's test." << std::endl
        << "      Default: " << parameters.m_hinkley_k << std::endl
        << std::endl
        << "  --shewhart-rules <3-sigma:{on|off} 2-sigma:{on|off} 1-sigma:{on|off} same-side:{on|off}>" << std::endl
        << "      Choose the WECO additionnal tests for the Shewhart's test to use. To activate them, --shewart-weco must be used." << std::endl
        << "      Default: ON ON ON ON" << std::endl
        << std::endl
        << "  --shewhart-weco" << std::endl
        << "      Activate the WECO additionnal tests for the Shewhart's test." << std::endl
        << "      Default: deactivated" << std::endl
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

  if (parameters.m_test_nbactivatedalarms == 0) {
    throw(vpException(vpException::badValue, "Error, at least one type of alarm must be monitored. See " + std::string(argv[0]) + " --help"));
    return EXIT_FAILURE;
  }

  std::cout << "  Activated statistical test           : " << TutorialMeanDrift::typeTestToString(opt_typeTest) << std::endl;
  std::cout << "  Activated alarms                     : " << TutorialMeanDrift::meanDriftArrayToString(parameters.m_test_activatedalarms) << std::endl;
  std::cout << "  Nb samples for statistics computation: " << parameters.m_test_nbsamples << std::endl;
  std::cout << "  Alarm factor CUSUM test              : " << (opt_typeTest == TutorialMeanDrift::MEAN_ADJUSTED_CUSUM_TYPE_TEST ? TutorialMeanDrift::numberToString(parameters.m_cusum_h) : "N/A")  << std::endl;
  std::cout << "  Detection factor CUSUM test          : " << (opt_typeTest == TutorialMeanDrift::MEAN_ADJUSTED_CUSUM_TYPE_TEST ? TutorialMeanDrift::numberToString(parameters.m_cusum_k) : "N/A") << std::endl;
  std::cout << "  Forgetting factor EWMA test          : " << (opt_typeTest == TutorialMeanDrift::EWMA_TYPE_TEST ? TutorialMeanDrift::numberToString(parameters.m_ewma_alpha) : "N/A") << std::endl;
  std::cout << "  Alarm threshold Hinkley's test       : " << ((opt_typeTest == TutorialMeanDrift::HINLKEY_TYPE_TEST) && (!parameters.m_hinkley_computealphadelta) ? TutorialMeanDrift::numberToString(parameters.m_hinkley_alpha) : "N/A") << std::endl;
  std::cout << "  Detection threshold Hinkley's test   : " << ((opt_typeTest == TutorialMeanDrift::HINLKEY_TYPE_TEST) && (!parameters.m_hinkley_computealphadelta) ? TutorialMeanDrift::numberToString(parameters.m_hinkley_delta) : "N/A") << std::endl;
  std::cout << "  Alarm factor Hinkley's test          : " << ((opt_typeTest == TutorialMeanDrift::HINLKEY_TYPE_TEST) &&   parameters.m_hinkley_computealphadelta ? TutorialMeanDrift::numberToString(parameters.m_hinkley_h) : "N/A") << std::endl;
  std::cout << "  Detection factor Hinkley's test      : " << ((opt_typeTest == TutorialMeanDrift::HINLKEY_TYPE_TEST) &&   parameters.m_hinkley_computealphadelta ? TutorialMeanDrift::numberToString(parameters.m_hinkley_k) : "N/A") << std::endl;
  std::cout << "  Shewhart's test set of WECO rules    : " << (parameters.m_shewhart_useWECO && (opt_typeTest == TutorialMeanDrift::SHEWHART_TYPE_TEST) ? TutorialMeanDrift::wecoRulesToString(parameters.m_shewhart_rules) : "N/A") << std::endl;
  std::cout << "  Shewhart's test use WECO rules       : " << (opt_typeTest == TutorialMeanDrift::SHEWHART_TYPE_TEST ? TutorialMeanDrift::boolToString(parameters.m_shewhart_useWECO && (opt_typeTest == TutorialMeanDrift::SHEWHART_TYPE_TEST)) : "N/A") << std::endl;
  std::cout << "  Alarm factor Sigma test              : " << (opt_typeTest == TutorialMeanDrift::SIGMA_TYPE_TEST ? TutorialMeanDrift::numberToString(parameters.m_sigma_h) : "N/A") << std::endl;
  std::cout << "  Actual mean of the input signal: " << opt_mean << std::endl;
  std::cout << "  Actual stdev of the input signal: " << opt_stdev << std::endl;
  std::cout << "  Mean drift of the input signal: " << opt_meandrift << std::endl;

  return testOnSynthetic(opt_typeTest, parameters, opt_mean, opt_meandrift, opt_stdev);
}
#else
int main()
{
  std::cerr << "Recompile ViSP with display capabilities in order to use this tutorial." << std::endl;
  return EXIT_FAILURE;
}
#endif
