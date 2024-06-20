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
 * Image difference.
 */

#include <iostream>
#include <visp3/core/vpImageTools.h>

/*!
  \example testImageDifference.cpp

  \brief Test vpImageTools::imageDifference()
*/
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void regularImageDifference(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            vpImage<unsigned char> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images have not the same size"));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diff = I1.bitmap[b] - I2.bitmap[b] + 128u;
    Idiff.bitmap[b] = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diff, 255), 0));
  }
}

void regularImageDifference(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, vpImage<vpRGBa> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute image difference. The two images "
                      "(%ux%u) and (%ux%u) have not the same size",
                      I1.getWidth(), I1.getHeight(), I2.getWidth(), I2.getHeight()));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diffR = I1.bitmap[b].R - I2.bitmap[b].R + 128;
    int diffG = I1.bitmap[b].G - I2.bitmap[b].G + 128;
    int diffB = I1.bitmap[b].B - I2.bitmap[b].B + 128;
    int diffA = I1.bitmap[b].A - I2.bitmap[b].A + 128;
    Idiff.bitmap[b].R = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffR, 255), 0));
    Idiff.bitmap[b].G = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffG, 255), 0));
    Idiff.bitmap[b].B = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffB, 255), 0));
    Idiff.bitmap[b].A = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffA, 255), 0));
  }
}
} // namespace

int main()
{
  unsigned int width = 501, height = 447;
  vpImage<unsigned char> I1(height, width), I2(height, width), Idiff_regular(height, width), Idiff_sse(height, width);
  vpImage<vpRGBa> I1_color(height, width), I2_color(height, width), Idiff_regular_color(height, width),
    Idiff_sse_color(height, width);
  for (unsigned int i = 0; i < I1.getRows(); i++) {
    for (unsigned int j = 0; j < I1.getCols(); j++) {
      I1[i][j] = static_cast<unsigned char>(i * I1.getCols() + j);
      I1_color[i][j] = vpRGBa(static_cast<unsigned char>(i * I1.getCols() + j));
    }
  }

  {
    std::cout << "Grayscale:" << std::endl;

    double t_regular = 0.0, t_sse = 0.0;
    for (unsigned int cpt = 0; cpt < 256; cpt++) {
      for (unsigned int i = 0; i < I2.getRows(); i++) {
        for (unsigned int j = 0; j < I2.getCols(); j++) {
          I2[i][j] = static_cast<unsigned char>(i * I2.getCols() + j + cpt);
        }
      }

      double t = vpTime::measureTimeMs();
      regularImageDifference(I1, I2, Idiff_regular);
      t_regular += vpTime::measureTimeMs() - t;

      t = vpTime::measureTimeMs();
      vpImageTools::imageDifference(I1, I2, Idiff_sse);
      t_sse += vpTime::measureTimeMs() - t;

      if (Idiff_regular != Idiff_sse) {
        std::cerr << "Problem with vpImageTools::imageDifference()" << std::endl;
        return EXIT_FAILURE;
      }
    }

    std::cout << "(Idiff_regular == Idiff_sse)" << std::endl;
    std::cout << "t_regular: " << t_regular << " ms ; mean t_regular: " << t_regular / 256 << " ms" << std::endl;
    std::cout << "t_sse: " << t_sse << " ms ; mean t_sse: " << t_sse / 256 << " ms" << std::endl;
    std::cout << "speed-up: " << t_regular / t_sse << " times" << std::endl;
  }

  {
    std::cout << "\nColor:" << std::endl;

    double t_regular = 0.0, t_sse = 0.0;
    for (unsigned int cpt = 0; cpt < 256; cpt++) {
      for (unsigned int i = 0; i < I2.getRows(); i++) {
        for (unsigned int j = 0; j < I2.getCols(); j++) {
          I2_color[i][j] = vpRGBa(static_cast<unsigned char>(i * I2.getCols() + j + cpt));
        }
      }

      double t = vpTime::measureTimeMs();
      regularImageDifference(I1_color, I2_color, Idiff_regular_color);
      t_regular += vpTime::measureTimeMs() - t;

      t = vpTime::measureTimeMs();
      vpImageTools::imageDifference(I1_color, I2_color, Idiff_sse_color);
      t_sse += vpTime::measureTimeMs() - t;

      if (Idiff_regular_color != Idiff_sse_color) {
        std::cerr << "Problem with vpImageTools::imageDifference()" << std::endl;
        return EXIT_FAILURE;
      }
    }

    std::cout << "(Idiff_regular_color == Idiff_sse_color)" << std::endl;
    std::cout << "t_regular: " << t_regular << " ms ; mean t_regular: " << t_regular / 256 << " ms" << std::endl;
    std::cout << "t_sse: " << t_sse << " ms ; mean t_sse: " << t_sse / 256 << " ms" << std::endl;
    std::cout << "speed-up: " << t_regular / t_sse << " times" << std::endl;
  }

  {
    std::cout << "Test vpRGBa" << std::endl;
    vpRGBa rgba_1(10, 20, 30);
    vpRGBa rgba_2(10, 20, 30);

    if (rgba_1 == rgba_2) {
      std::cout << "Test ok: same rgba" << std::endl;
    }
    else {
      std::cerr << "Error in rgba operator==" << std::endl;
      return EXIT_FAILURE;
    }
    if (rgba_1 != rgba_2) {
      std::cerr << "Error in rgba operator!=" << std::endl;
      return EXIT_FAILURE;
    }
    {
      vpRGBa rgba_3(1, 0, 0);
      if (rgba_1 == (rgba_2 + rgba_3)) {
        std::cerr << "Error in rgba operator==" << std::endl;
        return EXIT_FAILURE;
      }
      if (rgba_1 != (rgba_2 + rgba_3)) {
        std::cerr << "Test ok: R value differ" << std::endl;
      }
    }
    {
      vpRGBa rgba_3(0, 1, 0);
      if (rgba_1 == (rgba_2 + rgba_3)) {
        std::cerr << "Error in rgba operator==" << std::endl;
        return EXIT_FAILURE;
      }
      if (rgba_1 != (rgba_2 + rgba_3)) {
        std::cerr << "Test ok: G value differ" << std::endl;
      }
    }
    {
      vpRGBa rgba_3(0, 0, 1);
      if (rgba_1 == (rgba_2 + rgba_3)) {
        std::cerr << "Error in rgba operator==" << std::endl;
        return EXIT_FAILURE;
      }
      if (rgba_1 != (rgba_2 + rgba_3)) {
        std::cerr << "Test ok: B value differ" << std::endl;
      }
    }
  }

  {
    std::cout << "Test vpRGBf" << std::endl;
    vpRGBf rgbf_1(10.f, 20.f, 30.f);
    vpRGBf rgbf_2(10.f, 20.f, 30.f);

    if (rgbf_1 == rgbf_2) {
      std::cout << "Test ok: same rgbf" << std::endl;
    }
    else {
      std::cerr << "Error in rgbf operator==" << std::endl;
      return EXIT_FAILURE;
    }
    if (rgbf_1 != rgbf_2) {
      std::cerr << "Error in rgbf operator!=" << std::endl;
      return EXIT_FAILURE;
    }
    {
      vpRGBf rgbf_3(1e-6f, 0.f, 0.f);
      if (rgbf_1 == (rgbf_2 + rgbf_3)) {
        std::cerr << "Rf Error in rgbf operator==" << std::endl;
        return EXIT_FAILURE;
      }
      if (rgbf_1 != (rgbf_2 + rgbf_3)) {
        std::cerr << "Test ok: Rf value differ" << std::endl;
      }
    }
    {
      vpRGBf rgbf_3(0.f, 1e-6f, 0.f);
      if (rgbf_1 == (rgbf_2 + rgbf_3)) {
        std::cerr << "Gf Error in rgbf operator==" << std::endl;
        return EXIT_FAILURE;
      }
      if (rgbf_1 != (rgbf_2 + rgbf_3)) {
        std::cerr << "Test ok: Gf value differ" << std::endl;
      }
    }
    {
      vpRGBf rgbf_3(0.f, 0.f, 1e-6f);
      if (rgbf_1 == (rgbf_2 + rgbf_3)) {
        std::cerr << "Bf Error in rgbf operator==" << std::endl;
        return EXIT_FAILURE;
      }
      if (rgbf_1 != (rgbf_2 + rgbf_3)) {
        std::cerr << "Test ok: Bf value differ" << std::endl;
      }
    }
  }

  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
