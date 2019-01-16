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
 * Test image print.
 *
 *****************************************************************************/

#include <iostream>
#include <visp3/core/vpImage.h>

/*!
  \example testImagePrint.cpp

  \brief Test image print.
*/
int main()
{
  unsigned int size = 16;
  vpImage<int> I_int(size, size);
  vpImage<unsigned char> I_uchar(size, size);
  vpImage<char> I_char(size, size);

  for (unsigned int i = 0, cpt = 0; i < size; i++) {
    for (unsigned int j = 0; j < size; j++, cpt++) {
      I_int[i][j] = (int)cpt;
      I_uchar[i][j] = (unsigned char)cpt;
      I_char[i][j] = (char)cpt;
    }
  }

  size = 5;
  vpImage<float> I_float(size, size);
  vpImage<double> I_double(size, size);
  vpImage<vpRGBa> I_rgba(size, size);

  for (unsigned int i = 0, cpt = 0; i < size; i++) {
    for (unsigned int j = 0; j < size; j++, cpt++) {
      I_float[i][j] = (float)sqrt((double)cpt);
      I_double[i][j] = sqrt((double)cpt);
      I_rgba[i][j] = vpRGBa((unsigned char)cpt);
    }
  }

  std::cout << "I_int:\n" << I_int << std::endl;
  std::cout << "\nI_uchar:\n" << I_uchar << std::endl;
  std::cout << "\nI_char:\n" << I_char << std::endl;
  std::cout << "\nI_float:\n" << I_float << std::endl;
  std::cout << "\nI_double:\n" << I_double << std::endl;
  std::cout << "\nI_rgba:\n" << I_rgba << std::endl;

  return EXIT_SUCCESS;
}
