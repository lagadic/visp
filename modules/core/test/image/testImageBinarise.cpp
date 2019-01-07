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
 * Test for vpImageTools::binarise() function.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
/*!
  \example testImageBinarise.cpp

  \brief Test vpImageTools::binarise() function.

*/

#include <visp3/core/vpImageTools.h>

int main()
{
  std::cout << "Test vpImageTools::binarise() with different data types." << std::endl;

  unsigned int width = 5, height = 4;
  unsigned char *uchar_array = new unsigned char[width * height];
  double *double_array = new double[width * height];
  vpRGBa *rgba_array = new vpRGBa[width * height];
  for (unsigned char i = 0; i < width * height; i++) {
    uchar_array[i] = i;
    double_array[i] = i;
    rgba_array[i] = vpRGBa(i, i, i, i);
  }

  vpImage<unsigned char> I(uchar_array, height, width);
  vpImage<double> I_double(double_array, height, width);
  vpImage<vpRGBa> I_rgba(rgba_array, height, width);

  std::cout << "I:" << std::endl;
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      std::cout << static_cast<unsigned>(I[i][j]) << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "\nI_double:" << std::endl;
  for (unsigned int i = 0; i < I_double.getHeight(); i++) {
    for (unsigned int j = 0; j < I_double.getWidth(); j++) {
      std::cout << I_double[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "\nI_rgba:" << std::endl;
  for (unsigned int i = 0; i < I_rgba.getHeight(); i++) {
    for (unsigned int j = 0; j < I_rgba.getWidth(); j++) {
      std::cout << static_cast<unsigned>(I_rgba[i][j].R) << " ; " << static_cast<unsigned>(I_rgba[i][j].G) << " ; "
                << static_cast<unsigned>(I_rgba[i][j].B) << " ; " << static_cast<unsigned int>(I_rgba[i][j].A)
                << std::endl;
    }
    std::cout << std::endl;
  }

  vpImageTools::binarise(I, (unsigned char)5, (unsigned char)12, (unsigned char)0, (unsigned char)127,
                         (unsigned char)255);
  vpImageTools::binarise(I_double, 5.0, 12.0, 0.0, 127.0, 255.0);
  vpImageTools::binarise(I_rgba, vpRGBa(5), vpRGBa(12), vpRGBa(0), vpRGBa(127), vpRGBa(255));

  std::cout << "\nI binarise:" << std::endl;
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      std::cout << static_cast<unsigned>(I[i][j]) << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "\nI_double binarise:" << std::endl;
  for (unsigned int i = 0; i < I_double.getHeight(); i++) {
    for (unsigned int j = 0; j < I_double.getWidth(); j++) {
      std::cout << I_double[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "\nI_rgba binarise:" << std::endl;
  for (unsigned int i = 0; i < I_rgba.getHeight(); i++) {
    for (unsigned int j = 0; j < I_rgba.getWidth(); j++) {
      std::cout << static_cast<unsigned>(I_rgba[i][j].R) << " ; " << static_cast<unsigned>(I_rgba[i][j].G) << " ; "
                << static_cast<unsigned>(I_rgba[i][j].B) << " ; " << static_cast<unsigned>(I_rgba[i][j].A) << std::endl;
    }
    std::cout << std::endl;
  }

  // Check if results are the same between iterate and LUT methods
  width = 32;
  height = 8;
  unsigned char *uchar_array1 = new unsigned char[width * height];
  unsigned char *uchar_array2 = new unsigned char[width * height];
  for (unsigned int i = 0; i < 256; i++) {
    uchar_array1[i] = (unsigned char)i;
    uchar_array2[i] = (unsigned char)i;
  }

  vpImage<unsigned char> I_uchar1(uchar_array1, height, width);
  vpImage<unsigned char> I_uchar2(uchar_array2, height, width);

  unsigned char threshold1 = 50, threshold2 = 200;
  unsigned char value1 = 4, value2 = 127, value3 = 250;
  vpImageTools::binarise(I_uchar1, threshold1, threshold2, value1, value2, value3, false);
  vpImageTools::binarise(I_uchar2, threshold1, threshold2, value1, value2, value3, true);

  for (unsigned int i = 0; i < height; i++) {
    for (unsigned int j = 0; j < width; j++) {
      if (I_uchar1[i][j] != I_uchar2[i][j]) {
        std::cerr << "Results are different between iterate and LUT methods !" << std::endl;
        return -1;
      }
    }
  }

  // Test performance between iterate and LUT methods
  width = 640;
  height = 480;
  unsigned char *uchar_array_perf_lut = new unsigned char[width * height];
  unsigned char *uchar_array_perf_iterate = new unsigned char[width * height];
  for (unsigned int i = 0; i < width * height; i++) {
    uchar_array_perf_lut[i] = (unsigned char)i;
    uchar_array_perf_iterate[i] = (unsigned char)i;
  }

  vpImage<unsigned char> I_perf_lut(uchar_array_perf_lut, height, width);
  vpImage<unsigned char> I_perf_iterate(uchar_array_perf_iterate, height, width);

  unsigned int nbIterations = 100;
  double t1 = vpTime::measureTimeMs();
  for (unsigned int cpt = 0; cpt < nbIterations; cpt++) {
    vpImageTools::binarise(I_perf_iterate, threshold1, threshold2, value1, value2, value3, false);
  }
  t1 = vpTime::measureTimeMs() - t1;
  std::cout << "Iterate: " << t1 << " ms for " << nbIterations << " iterations." << std::endl;

  double t2 = vpTime::measureTimeMs();
  for (unsigned int cpt = 0; cpt < nbIterations; cpt++) {
    vpImageTools::binarise(I_perf_lut, threshold1, threshold2, value1, value2, value3, true);
  }
  t2 = vpTime::measureTimeMs() - t2;
  std::cout << "LUT: " << t2 << " ms for " << nbIterations << " iterations." << std::endl;

  std::cout << "\ntestImageBinarise ok !" << std::endl;
  return 0;
}
