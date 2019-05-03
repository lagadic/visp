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
 * Test for vpImagePoint::getValue().
 *
 *****************************************************************************/
 /*!
   \example testImageGetValue.cpp

   \brief Test for vpImagePoint::getValue().
 */

#include <iostream>
#include <visp3/core/vpImage.h>

namespace
{
template<typename PixelType> PixelType checkPixelAccess(unsigned int height, unsigned int width, double v, double u) {
  vpImage<PixelType> I(height, width);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      I[i][j] = static_cast<PixelType>(i * I.getWidth() + j);
    }
  }

  return I.getValue(v,u);
}

template<> vpRGBa checkPixelAccess(unsigned int height, unsigned int width, double v, double u) {
  vpImage<vpRGBa> I(height, width);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      I[i][j] = vpRGBa(static_cast<unsigned char>(i * I.getWidth() + j),
                       static_cast<unsigned char>(i * I.getWidth() + j),
                       static_cast<unsigned char>(i * I.getWidth() + j));
    }
  }

  return I.getValue(v,u);
}

double randomDouble(double a, double b) {
  double random = (static_cast<double>(rand())) / static_cast<double>(RAND_MAX);
  double diff = b - a;
  double r = random * diff;
  return a + r;
}

unsigned char randomPixelValue() {
  const int min = 0, max = 255;
  return static_cast<unsigned char>((rand() % (max - min + 1) + min));
}

template <class PixelType> PixelType getValue(const vpImage<PixelType> &I, double i, double j, bool roundValue) {
  if (i < 0 || j < 0 || i+1 > I.getHeight() || j+1 > I.getWidth()) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if (I.getHeight() * I.getWidth() == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = (std::min)(I.getHeight() - 1, iround + 1);
  unsigned int jround_1 = (std::min)(I.getWidth() - 1, jround + 1);

  double value = (static_cast<double>(I[iround][jround]) * rfrac + static_cast<double>(I[iround_1][jround]) * rratio) * cfrac +
                 (static_cast<double>(I[iround][jround_1]) * rfrac + static_cast<double>(I[iround_1][jround_1]) * rratio) * cratio;

  return static_cast<PixelType>(roundValue ? vpMath::round(value) : value);
}
} // namespace

int main() {
  //Test out of image memory access
  //vpImage::getValue(double, double)
  {
    //unsigned char
    std::cout << "checkPixelAccess<unsigned char>(3, 4, 2, 3): "
              << static_cast<unsigned int>(checkPixelAccess<unsigned char>(3, 4, 2, 3)) << std::endl;
    try {
      std::cout << "checkPixelAccess<unsigned char>(3, 4, -2, -3): "
                << static_cast<unsigned int>(checkPixelAccess<unsigned char>(3, 4, -2, -3)) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }
    try {
      std::cout << "checkPixelAccess<unsigned char>(3, 4, 3, 4): "
                << static_cast<unsigned int>(checkPixelAccess<unsigned char>(3, 4, 3, 4)) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }

    //vpRGBa
    std::cout << "checkPixelAccess<vpRGBa>(3, 4, 2, 3): " << checkPixelAccess<vpRGBa>(3, 4, 2, 3) << std::endl;
    try {
      std::cout << "checkPixelAccess<vpRGBa>(3, 4, -2, -3): " << checkPixelAccess<vpRGBa>(3, 4, -2, -3) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }
    try {
      std::cout << "checkPixelAccess<vpRGBa>(3, 4, 3, 4): " << checkPixelAccess<vpRGBa>(3, 4, 3, 4) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }

    //int
    std::cout << "checkPixelAccess<int>(3, 4, 2, 3): " << checkPixelAccess<int>(3, 4, 2, 3) << std::endl;
    try {
      std::cout << "checkPixelAccess<int>(3, 4, -2, -3): " << checkPixelAccess<int>(3, 4, -2, -3) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }
    try {
      std::cout << "checkPixelAccess<int>(3, 4, 3, 4): " << checkPixelAccess<int>(3, 4, 3, 4) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }

    //double
    std::cout << "checkPixelAccess<double>(3, 4, 2, 3): " << checkPixelAccess<double>(3, 4, 2, 3) << std::endl;
    try {
      std::cout << "checkPixelAccess<double>(3, 4, -2, -3): " << checkPixelAccess<double>(3, 4, -2, -3) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }
    try {
      std::cout << "checkPixelAccess<double>(3, 4, 3, 4): " << checkPixelAccess<double>(3, 4, 3, 4) << std::endl;
      std::cerr << "Out of image access exception should have been thrown" << std::endl;
      return EXIT_FAILURE;
    } catch (...) { std::cout << "\n"; }
  }

  //Test difference between double bilinear interpolation and fixed-point interpolation
  srand(0);

  {
    vpImage<unsigned char> I(480, 640);
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        I[i][j] = randomPixelValue();
      }
    }

    double diff_round = 0.0, diff = 0.0;
    vpImage<unsigned char> I1(480, 640);
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        double idx1 = randomDouble(0, I.getHeight() - 1);
        double idx2 = randomDouble(0, I.getWidth() - 1);
        unsigned char val1 = I.getValue(idx1, idx2);
        unsigned char val2 = getValue<unsigned char>(I, idx1, idx2, true);
        unsigned char val3 = getValue<unsigned char>(I, idx1, idx2, false);

        diff_round += std::fabs(val1 - val2);
        diff += std::fabs(val1 - val3);
      }
    }

    double meanDiffRound = diff_round / I.getSize();
    double meanDiff = diff / I.getSize();
    std::cout << "diff_round: " << diff_round << " ; meanDiffRound: " << meanDiffRound << std::endl;
    std::cout << "diff: " << diff << " ; meanDiff: " << meanDiff << std::endl;
    const double maxInterpolationErrorDiff = 1.0;
    if (std::fabs(meanDiffRound) > maxInterpolationErrorDiff) {
      std::cerr << "Too much pixel difference between fixed-point vpImage::getValue(double, double) and old method."
                << std::endl;
      return EXIT_FAILURE;
    }
  }

  //Test performance double bilinear interpolation + round vs fixed-point interpolation
  {
    vpImage<unsigned char> I(1080, 1920);
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        I[i][j] = randomPixelValue();
      }
    }

    std::vector<std::pair<double, double> > indexes;
    for (int cpt = 0; cpt < 1000000; cpt++) {
      double idx1 = randomDouble(0, I.getHeight() - 1);
      double idx2 = randomDouble(0, I.getWidth() - 1);
      indexes.push_back(std::pair<double, double>(idx1, idx2));
    }

    int sum1 = 0;
    double t_optim = vpTime::measureTimeMs();
    for (size_t cpt = 0; cpt < indexes.size(); cpt++) {
      double idx1 = indexes[cpt].first;
      double idx2 = indexes[cpt].second;
      sum1 += I.getValue(idx1, idx2);
    }
    t_optim = vpTime::measureTimeMs() - t_optim;
    std::cout << "\nFixed-point vpImage::getValue(double, double), sum1: " << sum1 << " in " << t_optim << " ms" << std::endl;

    int sum2 = 0;
    double t_old = vpTime::measureTimeMs();
    for (size_t cpt = 0; cpt < indexes.size(); cpt++) {
      double idx1 = indexes[cpt].first;
      double idx2 = indexes[cpt].second;
      sum2 += getValue(I, idx1, idx2, true);
    }
    t_old = vpTime::measureTimeMs() - t_old;
    std::cout << "Old method, sum2: " << sum2 << " in " << t_old << " ms" << std::endl;
    std::cout << "Speed-up: " << t_old / t_optim << "X" << std::endl;
  }

  //Test performance double bilinear interpolation + round vs fixed-point interpolation
  {
    vpImage<unsigned char> I(1080, 1920);
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        I[i][j] = randomPixelValue();
      }
    }

    std::vector<std::pair<double, double> > indexes;
    for (int cpt = 0; cpt < 1000000; cpt++) {
      double idx1 = randomDouble(0, I.getHeight() - 1);
      double idx2 = randomDouble(0, I.getWidth() - 1);
      indexes.push_back(std::pair<double, double>(idx1, idx2));
    }

    int sum1 = 0;
    double t_optim = vpTime::measureTimeMs();
    for (size_t cpt = 0; cpt < indexes.size(); cpt++) {
      double idx1 = indexes[cpt].first;
      double idx2 = indexes[cpt].second;
      sum1 += I.getValue(idx1, idx2);
    }
    t_optim = vpTime::measureTimeMs() - t_optim;
    std::cout << "\nFixed-point vpImage::getValue(double, double), sum1: " << sum1 << " in " << t_optim << " ms" << std::endl;

    int sum2 = 0;
    double t_old = vpTime::measureTimeMs();
    for (size_t cpt = 0; cpt < indexes.size(); cpt++) {
      double idx1 = indexes[cpt].first;
      double idx2 = indexes[cpt].second;
      sum2 += getValue(I, idx1, idx2, false);
    }
    t_old = vpTime::measureTimeMs() - t_old;
    std::cout << "Old method (without vpMath::round()), sum2: " << sum2 << " in " << t_old << " ms" << std::endl;
    std::cout << "Speed-up: " << t_old / t_optim << "X" << std::endl;
  }

  //Check that getValue() still returns correct values
  {
    vpImage<unsigned char> I(480, 640);
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        I[i][j] = randomPixelValue();
      }
    }

    vpImage<unsigned char> I_copy(480, 640);
    for (unsigned int i = 0; i < I_copy.getHeight(); i++) {
      double y = static_cast<double>(i);

      for (unsigned int j = 0; j < I_copy.getWidth(); j++) {
        double x = static_cast<double>(j);

        I_copy[i][j] = I.getValue(y, x);
      }
    }

    bool same = (I == I_copy);
    std::cout << "\nCheck that getValue returns correct results for integer coordinates\n(I == I_copy)? " << same << std::endl;
    if (!same) {
      std::cerr << "Issue with vpImage::getValue(double, double)!" << std::endl;
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
