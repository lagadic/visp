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
 * Test for vpImagePoint::getValue().
 */
/*!
  \example testImageMeanAndStdev.cpp

  \brief Test for vpImage::getMeanValue() and vpImage::getStdev().
*/

#include <iostream>
#include <visp3/core/vpImage.h>

void printHelp(const std::string &progName)
{
  std::cout << "SYNOPSIS: " << std::endl;
  std::cout << "  " << progName << " [-v, --verbose] [-h, --help]" << std::endl;
  std::cout << "DETAILS:" << std::endl;
  std::cout << "  -v, --verbose" << std::endl;
  std::cout << "    Activate verbose mode to have some logs in the console." << std::endl;
  std::cout << std::endl;
  std::cout << "  -h, --help" << std::endl;
  std::cout << "    Display the help about the program." << std::endl;
  std::cout << std::endl;
}

int main(const int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  bool opt_verbose = false;
  for (int i = 1; i < argc; ++i) {
    std::string argName(argv[i]);
    if ((argName == "-v") || (argName == "--verbose")) {
      opt_verbose = true;
    }
    else if ((argName == "-h") || (argName == "--help")) {
      printHelp(std::string(argv[0]));
      return EXIT_FAILURE;
    }
  }

  const unsigned int nbRows = 4, nbCols = 4;
  vpImage<unsigned char> I_uchar_ref(nbRows, nbCols);
  vpImage<vpRGBa> I_rgba_ref(nbRows, nbCols);
  vpImage<vpRGBf> I_rgbf_ref(nbRows, nbCols);
  double sum_uchar_ref = 0.;
  double sum_rgba_ref = 0.;
  double sum_rgbf_ref = 0.;
  unsigned int count_ref = 0;
  // Initialization of the images
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      unsigned int val = r * nbCols + c;
      I_uchar_ref[r][c] = val;
      I_rgba_ref[r][c].R = val;
      I_rgba_ref[r][c].G = 2*val;
      I_rgba_ref[r][c].B = 3*val;
      I_rgbf_ref[r][c].R = I_rgba_ref[r][c].R;
      I_rgbf_ref[r][c].G = I_rgba_ref[r][c].G;
      I_rgbf_ref[r][c].B = I_rgba_ref[r][c].B;
      sum_uchar_ref += static_cast<double>(val);
      double val_rgb = static_cast<double>(I_rgba_ref[r][c].R) + static_cast<double>(I_rgba_ref[r][c].G) + static_cast<double>(I_rgba_ref[r][c].B);
      sum_rgba_ref += val_rgb;
      sum_rgbf_ref += val_rgb;
      ++count_ref;
    }
  }

  // Computation of the means
  double mean_uchar_ref = sum_uchar_ref / static_cast<double>(count_ref);
  double mean_rgba_ref = sum_rgba_ref / static_cast<double>(count_ref);
  double mean_rgbf_ref = sum_rgbf_ref / static_cast<double>(count_ref);

  // Computation of the standard deviations
  double stdev_uchar_ref = 0.;
  double stdev_rgba_ref = 0.;
  double stdev_rgbf_ref = 0.;
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      stdev_uchar_ref += std::pow(static_cast<double>(I_uchar_ref[r][c]) - mean_uchar_ref, 2);
      stdev_rgba_ref += std::pow(static_cast<double>(I_rgba_ref[r][c].R) + static_cast<double>(I_rgba_ref[r][c].G) + static_cast<double>(I_rgba_ref[r][c].B) - mean_rgba_ref, 2);
      stdev_rgbf_ref += std::pow(static_cast<double>(I_rgbf_ref[r][c].R) + static_cast<double>(I_rgbf_ref[r][c].G) + static_cast<double>(I_rgbf_ref[r][c].B) - mean_rgbf_ref, 2);
    }
  }
  stdev_uchar_ref = std::sqrt((1./static_cast<double>(nbRows * nbCols))* stdev_uchar_ref);
  stdev_rgba_ref = std::sqrt((1./static_cast<double>(nbRows * nbCols))* stdev_rgba_ref);
  stdev_rgbf_ref = std::sqrt((1./static_cast<double>(nbRows * nbCols))* stdev_rgbf_ref);
  if (opt_verbose) {
    std::cout << "----- Input data-----" << std::endl;
    std::cout << "I_uchar_ref = \n" << I_uchar_ref << std::endl;
    std::cout << "sum_uchar_ref(I_uchar_ref) = " << sum_uchar_ref << std::endl;
    std::cout << "mean_uchar_ref(I_uchar_ref) = " << mean_uchar_ref << std::endl;
    std::cout << "stdev_uchar_ref(I_uchar_ref) = " << stdev_uchar_ref << std::endl;
    std::cout << std::endl;
    std::cout << "I_rgba_ref = \n" << I_rgba_ref << std::endl;
    std::cout << "sum_rgba_ref(I_uchar_ref) = " << sum_rgba_ref << std::endl;
    std::cout << "mean_rgba_ref(I_rgba_ref) = " << mean_rgba_ref << std::endl;
    std::cout << "stdev_rgba_ref(I_rgba_ref) = " << stdev_rgba_ref << std::endl;
    std::cout << std::endl;
    std::cout << "I_rgbf_ref = \n" << I_rgbf_ref << std::endl;
    std::cout << "sum_rgbf_ref(I_rgbf_ref) = " << sum_rgbf_ref << std::endl;
    std::cout << "mean_rgbf_ref(I_rgbf_ref) = " << mean_rgbf_ref << std::endl;
    std::cout << "stdev_rgbf_ref(I_rgbf_ref) = " << stdev_rgbf_ref << std::endl;
    std::cout << std::endl;
  }

  vpImage<bool> I_mask(nbRows, nbCols);
  unsigned int count_true = 0;
  double sum_uchar_true = 0.;
  double sum_rgba_true = 0.;
  double sum_rgbf_true = 0.;
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      bool isTrue = ((r + c) % 2) == 0;
      I_mask[r][c] = isTrue;
      if (isTrue) {
        ++count_true;
        sum_uchar_true += static_cast<double>(I_uchar_ref[r][c]);
        double val_rgba = static_cast<double>(I_rgba_ref[r][c].R) + static_cast<double>(I_rgba_ref[r][c].G) + static_cast<double>(I_rgba_ref[r][c].B);
        sum_rgba_true += val_rgba;
        double val_rgbf = static_cast<double>(I_rgbf_ref[r][c].R) + static_cast<double>(I_rgbf_ref[r][c].G) + static_cast<double>(I_rgbf_ref[r][c].B);
        sum_rgbf_true += val_rgbf;
      }
    }
  }
  // Computation of the means when a boolean mask is used
  double mean_uchar_true = sum_uchar_true / static_cast<double>(count_true);
  double mean_rgba_true = sum_rgba_true / static_cast<double>(count_true);
  double mean_rgbf_true = sum_rgbf_true / static_cast<double>(count_true);
  // Computation of the standard deviations when a boolean mask is used
  double stdev_uchar_true = 0.;
  double stdev_rgba_true = 0.;
  double stdev_rgbf_true = 0.;
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      if (I_mask[r][c]) {
        stdev_uchar_true += (static_cast<double>(I_uchar_ref[r][c]) - mean_uchar_true) * (static_cast<double>(I_uchar_ref[r][c]) - mean_uchar_true);
        double val_rgba = static_cast<double>(I_rgba_ref[r][c].R) + static_cast<double>(I_rgba_ref[r][c].G) + static_cast<double>(I_rgba_ref[r][c].B);
        stdev_rgba_true += (val_rgba - mean_rgba_true) * (val_rgba - mean_rgba_true);
        double val_rgbf = static_cast<double>(I_rgbf_ref[r][c].R) + static_cast<double>(I_rgbf_ref[r][c].G) + static_cast<double>(I_rgbf_ref[r][c].B);
        stdev_rgbf_true += (val_rgbf - mean_rgbf_true) * (val_rgbf - mean_rgbf_true);
      }
    }
  }
  stdev_uchar_true = std::sqrt((1./static_cast<double>(count_true)) * stdev_uchar_true);
  stdev_rgba_true = std::sqrt((1./static_cast<double>(count_true)) * stdev_rgba_true);
  stdev_rgbf_true = std::sqrt((1./static_cast<double>(count_true)) * stdev_rgbf_true);
  if (opt_verbose) {
    std::cout << "I_mask = \n";
    for (unsigned int r = 0; r < nbRows; ++r) {
      for (unsigned int c = 0; c < nbCols; ++c) {
        std::cout << (I_mask[r][c] ? "true" : "false") << " ";
      }
      std::cout << "\n";
    }
    std::cout << std::endl;
    std::cout << "nb_true(I_uchar_ref, I_mask) = " << count_true << std::endl;
    std::cout << "sum_uchar_true(I_uchar_ref, I_mask) = " << sum_uchar_true << std::endl;
    std::cout << "mean_uchar_true(I_uchar_ref, I_mask) = " << mean_uchar_true << std::endl;
    std::cout << "stdev_uchar_true(I_uchar_ref, I_mask) = " << stdev_uchar_true << std::endl;
    std::cout << "sum_rgba_true(I_rgba_ref, I_mask) = " << sum_rgba_true << std::endl;
    std::cout << "mean_rgba_true(I_rgba_ref, I_mask) = " << mean_rgba_true << std::endl;
    std::cout << "stdev_rgba_true(I_rgba_ref, I_mask) = " << stdev_rgba_true << std::endl;
    std::cout << "sum_rgbf_true(I_rgbf_ref, I_mask) = " << sum_rgbf_true << std::endl;
    std::cout << "mean_rgbf_true(I_rgbf_ref, I_mask) = " << mean_rgbf_true << std::endl;
    std::cout << "stdev_rgbf_true(I_rgbf_ref, I_mask) = " << stdev_rgbf_true << std::endl;
    std::cout << std::endl;
  }

  bool areTestOK = true;
  unsigned int nbFailedTests = 0;
  std::vector<std::string> failedTestsNames;
  if (opt_verbose) {
    std::cout << "----- BEGIN tests-----" << std::endl;
  }

  // Tests on the sum
  {
    if (opt_verbose) {
      std::cout << "Tests on the sum" << std::endl;
    }
    std::string nameTest("vpImage<uchar>::getSum()");
    double sum = I_uchar_ref.getSum();
    bool success = vpMath::equal(sum, sum_uchar_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical sum = " << sum_uchar_ref << " | returned value = " << sum << std::endl;
      }
    }

    unsigned int nbValidPoints = 0;
    nameTest = ("vpImage<uchar>::getSum( const vpImage<bool> *, unsigned int * )");
    sum = I_uchar_ref.getSum(&I_mask, &nbValidPoints);
    success = vpMath::equal(sum, sum_uchar_true) && (nbValidPoints == count_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << count_true << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical sum = " << sum_uchar_true << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<uchar>::getSum( vpImage<bool> * = nullptr, unsigned int * )");
    sum = I_uchar_ref.getSum(nullptr, &nbValidPoints);
    success = vpMath::equal(sum, sum_uchar_ref) && (nbValidPoints == (nbCols * nbRows));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbCols * nbRows << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical sum = " << sum_uchar_ref << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getSum()");
    sum = I_rgba_ref.getSum();
    success = vpMath::equal(sum, sum_rgba_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical sum = " << sum_rgba_ref << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getSum( vpImage<bool> *, unsigned int * )");
    sum = I_rgba_ref.getSum(&I_mask, &nbValidPoints);
    success = vpMath::equal(sum, sum_rgba_true) && (nbValidPoints == count_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << count_true << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical sum = " << sum_rgba_true << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getSum( vpImage<bool> * = nullptr, unsigned int * )");
    sum = I_rgba_ref.getSum(nullptr, &nbValidPoints);
    success = vpMath::equal(sum, sum_rgba_ref) && (nbValidPoints == (nbCols * nbRows));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbCols * nbRows << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical sum = " << sum_rgba_ref << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getSum()");
    sum = I_rgbf_ref.getSum();
    success = vpMath::equal(sum, sum_rgbf_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical sum = " << sum_rgbf_ref << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getSum( vpImage<bool> *, unsigned int * )");
    sum = I_rgbf_ref.getSum(&I_mask, &nbValidPoints);
    success = vpMath::equal(sum, sum_rgbf_true) && (nbValidPoints == count_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << count_true << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical sum = " << sum_rgbf_true << " | returned value = " << sum << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getSum( vpImage<bool> * = nullptr, unsigned int * )");
    sum = I_rgbf_ref.getSum(nullptr, &nbValidPoints);
    success = vpMath::equal(sum, sum_rgbf_ref) && (nbValidPoints == (nbCols * nbRows));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbCols * nbRows << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical sum = " << sum_rgbf_ref << " | returned value = " << sum << std::endl;
      }
    }
  }

  // Tests on the mean
  {
    if (opt_verbose) {
      std::cout << "Tests on the mean" << std::endl;
    }
    std::string nameTest("vpImage<uchar>::getMeanValue()");
    double mean = I_uchar_ref.getMeanValue();
    bool success = vpMath::equal(mean, mean_uchar_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_uchar_ref << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<uchar>::getMeanValue(vpImage<bool> *)";
    mean = I_uchar_ref.getMeanValue(&I_mask);
    success = vpMath::equal(mean, mean_uchar_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_uchar_true << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<uchar>::getMeanValue(vpImage<bool> * = nullptr)";
    mean = I_uchar_ref.getMeanValue(nullptr);
    success = vpMath::equal(mean, mean_uchar_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_uchar_ref << " | returned value = " << mean << std::endl;
      }
    }

    unsigned int nbValidPoints = 0;
    nameTest = "vpImage<uchar>::getMeanValue(vpImage<bool> *, unsigned int &)";
    mean = I_uchar_ref.getMeanValue(&I_mask, &nbValidPoints);
    success = vpMath::equal(mean, mean_uchar_true) && (nbValidPoints == count_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << count_true << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical mean = " << mean_uchar_true << " | returned value = " << mean << std::endl;
      }
    }

    nbValidPoints = 0;
    nameTest = "vpImage<uchar>::getMeanValue(vpImage<bool> * = nullptr, unsigned int &)";
    mean = I_uchar_ref.getMeanValue(nullptr, &nbValidPoints);
    success = vpMath::equal(mean, mean_uchar_ref) && (nbValidPoints == (nbCols * nbRows));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbCols * nbRows << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical mean = " << mean_uchar_ref << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBa>::getMeanValue()";
    mean = I_rgba_ref.getMeanValue();
    success = vpMath::equal(mean, mean_rgba_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_rgba_ref << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBa>::getMeanValue(vpImage<bool> *)";
    mean = I_rgba_ref.getMeanValue(&I_mask);
    success = vpMath::equal(mean, mean_rgba_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_rgba_true << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBa>::getMeanValue(vpImage<bool> * = nullptr)";
    mean = I_rgba_ref.getMeanValue(nullptr);
    success = vpMath::equal(mean, mean_rgba_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_rgba_ref << " | returned value = " << mean << std::endl;
      }
    }

    nbValidPoints = 0;
    nameTest = "vpImage<vpRGBa>::getMeanValue(vpImage<bool> *, unsigned int &)";
    mean = I_rgba_ref.getMeanValue(&I_mask, &nbValidPoints);
    success = vpMath::equal(mean, mean_rgba_true) && (nbValidPoints == count_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << count_true << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical mean = " << mean_rgba_true << " | returned value = " << mean << std::endl;
      }
    }

    nbValidPoints = 0;
    nameTest = "vpImage<vpRGBa>::getMeanValue(vpImage<bool> * = nullptr, unsigned int &)";
    mean = I_rgba_ref.getMeanValue(nullptr, &nbValidPoints);
    success = vpMath::equal(mean, mean_rgba_ref) && (nbValidPoints == (nbRows * nbCols));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbRows * nbCols << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical mean = " << mean_rgba_ref << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBf>::getMeanValue()";
    mean = I_rgbf_ref.getMeanValue();
    success = vpMath::equal(mean, mean_rgbf_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_rgbf_ref << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBf>::getMeanValue(vpImage<bool> *)";
    mean = I_rgbf_ref.getMeanValue(&I_mask);
    success = vpMath::equal(mean, mean_rgbf_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_rgbf_true << " | returned value = " << mean << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBf>::getMeanValue(vpImage<bool> * = nullptr)";
    mean = I_rgbf_ref.getMeanValue(nullptr);
    success = vpMath::equal(mean, mean_rgbf_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical mean = " << mean_rgbf_ref << " | returned value = " << mean << std::endl;
      }
    }

    nbValidPoints = 0;
    nameTest = "vpImage<vpRGBf>::getMeanValue(vpImage<bool> *, unsigned int &)";
    mean = I_rgbf_ref.getMeanValue(&I_mask, &nbValidPoints);
    success = vpMath::equal(mean, mean_rgbf_true) && (nbValidPoints == count_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << count_true << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical mean = " << mean_rgbf_true << " | returned value = " << mean << std::endl;
      }
    }

    nbValidPoints = 0;
    nameTest = "vpImage<vpRGBf>::getMeanValue(vpImage<bool> * = nullptr, unsigned int &)";
    mean = I_rgbf_ref.getMeanValue(nullptr, &nbValidPoints);
    success = vpMath::equal(mean, mean_rgbf_ref) && (nbValidPoints == (nbRows * nbCols));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbRows * nbCols << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical mean = " << mean_rgbf_ref << " | returned value = " << mean << std::endl;
      }
    }
  }

  // Tests on the stdev
  {
    if (opt_verbose) {
      std::cout << "Tests on the stdev" << std::endl;
    }
    std::string nameTest("vpImage<uchar>::getStdev()");
    double stdev = I_uchar_ref.getStdev();
    bool success = vpMath::equal(stdev, stdev_uchar_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_uchar_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<uchar>::getStdev(const vpImage<bool> *)");
    stdev = I_uchar_ref.getStdev(&I_mask);
    success = vpMath::equal(stdev, stdev_uchar_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_uchar_true << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<uchar>::getStdev(const vpImage<bool> * = nullptr)");
    stdev = I_uchar_ref.getStdev(nullptr);
    success = vpMath::equal(stdev, stdev_uchar_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_uchar_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<uchar>::getStdev(const double &)");
    double mean = I_uchar_ref.getMeanValue();
    stdev = I_uchar_ref.getStdev(mean);
    success = vpMath::equal(stdev, stdev_uchar_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_uchar_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<uchar>::getStdev(const double &, vpImage<bool> *, unsigned int *)");
    unsigned int nbValidPoints = 0;
    mean = I_uchar_ref.getMeanValue(&I_mask, &nbValidPoints);
    stdev = I_uchar_ref.getStdev(mean, &I_mask, &nbValidPoints);
    success = vpMath::equal(stdev, stdev_uchar_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_uchar_true << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<uchar>::getStdev(const double &, vpImage<bool> *, unsigned int * = nullptr)");
    nbValidPoints = 0;
    mean = I_uchar_ref.getMeanValue(nullptr, &nbValidPoints);
    stdev = I_uchar_ref.getStdev(mean, nullptr, &nbValidPoints);
    success = vpMath::equal(stdev, stdev_uchar_ref) && (nbValidPoints == (nbRows * nbCols));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbRows * nbCols << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical stdev = " << stdev_uchar_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBa>::getStdev()";
    stdev = I_rgba_ref.getStdev();
    success = vpMath::equal(stdev, stdev_rgba_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgba_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getStdev(const vpImage<bool> *)");
    stdev = I_rgba_ref.getStdev(&I_mask);
    success = vpMath::equal(stdev, stdev_rgba_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgba_true << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getStdev(const vpImage<bool> * = nullptr)");
    stdev = I_rgba_ref.getStdev(nullptr);
    success = vpMath::equal(stdev, stdev_rgba_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgba_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getStdev(const double &)");
    mean = I_rgba_ref.getMeanValue();
    stdev = I_rgba_ref.getStdev(mean);
    success = vpMath::equal(stdev, stdev_rgba_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgba_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getStdev(const double &, vpImage<bool> *, unsigned int *)");
    nbValidPoints = 0;
    mean = I_rgba_ref.getMeanValue(&I_mask, &nbValidPoints);
    stdev = I_rgba_ref.getStdev(mean, &I_mask, &nbValidPoints);
    success = vpMath::equal(stdev, stdev_rgba_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgba_true << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBa>::getStdev(const double &, vpImage<bool> *, unsigned int * = nullptr)");
    nbValidPoints = 0;
    mean = I_rgba_ref.getMeanValue(nullptr, &nbValidPoints);
    stdev = I_rgba_ref.getStdev(mean, nullptr, &nbValidPoints);
    success = vpMath::equal(stdev, stdev_rgba_ref) && (nbValidPoints == (nbRows * nbCols));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbRows * nbCols << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical stdev = " << stdev_rgba_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = "vpImage<vpRGBf>::getStdev()";
    stdev = I_rgbf_ref.getStdev();
    success = vpMath::equal(stdev, stdev_rgbf_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgbf_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getStdev(const vpImage<bool> *)");
    stdev = I_rgbf_ref.getStdev(&I_mask);
    success = vpMath::equal(stdev, stdev_rgbf_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgbf_true << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getStdev(const vpImage<bool> * = nullptr)");
    stdev = I_rgbf_ref.getStdev(nullptr);
    success = vpMath::equal(stdev, stdev_rgbf_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgbf_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getStdev(const double &)");
    mean = I_rgbf_ref.getMeanValue();
    stdev = I_rgbf_ref.getStdev(mean);
    success = vpMath::equal(stdev, stdev_rgbf_ref);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgbf_ref << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getStdev(const double &, vpImage<bool> *, unsigned int *)");
    nbValidPoints = 0;
    mean = I_rgbf_ref.getMeanValue(&I_mask, &nbValidPoints);
    stdev = I_rgbf_ref.getStdev(mean, &I_mask, &nbValidPoints);
    success = vpMath::equal(stdev, stdev_rgbf_true);
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical stdev = " << stdev_rgbf_true << " | returned value = " << stdev << std::endl;
      }
    }

    nameTest = ("vpImage<vpRGBf>::getStdev(const double &, vpImage<bool> *, unsigned int * = nullptr)");
    nbValidPoints = 0;
    mean = I_rgbf_ref.getMeanValue(nullptr, &nbValidPoints);
    stdev = I_rgbf_ref.getStdev(mean, nullptr, &nbValidPoints);
    success = vpMath::equal(stdev, stdev_rgbf_ref) && (nbValidPoints == (nbRows * nbCols));
    if (!success) {
      ++nbFailedTests;
      failedTestsNames.push_back(nameTest);
      areTestOK = false;
    }
    if (opt_verbose) {
      std::cout << "\tTest " << nameTest << ": " << (success ? "OK" : "failure") << std::endl;
      if (!success) {
        std::cout << "Theoretical count = " << nbRows * nbCols << " | returned value = " << nbValidPoints << std::endl;
        std::cout << "Theoretical stdev = " << stdev_rgbf_ref << " | returned value = " << stdev << std::endl;
      }
    }
  }

  if (areTestOK) {
    std::cout << "All tests succeeded" << std::endl;
    return EXIT_SUCCESS;
  }
  else {
    std::cerr << nbFailedTests << " tests failed: " << std::endl;
    for (unsigned int i = 0; i < nbFailedTests; ++i) {
      std::cerr << "  - " << failedTestsNames[i] << std::endl;
    }
    return EXIT_FAILURE;
  }
}
