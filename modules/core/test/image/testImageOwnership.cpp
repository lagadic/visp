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
 * Test vpImage ownership
 */

#include <visp3/core/vpImage.h>

/*!
  \example testImageOwnership.cpp

  \brief Test vpImage ownership.
*/

int main(int /* argc */, const char ** /* argv */)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    {
      unsigned char bitmap[4];
      bitmap[0] = 0;
      bitmap[1] = 1;
      bitmap[2] = 2;
      bitmap[3] = 3;

      vpImage<unsigned char> I(bitmap, 2, 2, true);
      std::cout << "I:\n" << I << std::endl;
      I.destroy();
    }
    {
      unsigned char bitmap[4];
      bitmap[0] = 0;
      bitmap[1] = 1;
      bitmap[2] = 2;
      bitmap[3] = 3;

      vpImage<unsigned char> I(bitmap, 2, 2, false);
      std::cout << "\nI:\n" << I << std::endl;
    }
    {
      unsigned char bitmap[4];
      bitmap[0] = 0;
      bitmap[1] = 1;
      bitmap[2] = 2;
      bitmap[3] = 3;

      vpImage<unsigned char> I(bitmap, 2, 2, false);
      {
        vpImage<unsigned char> I2(I);
        std::cout << "\nI2:\n" << I2 << std::endl;
      }
      {
        vpImage<unsigned char> I2 = I;
        std::cout << "I2:\n" << I2 << std::endl;
      }
    }
    {
      unsigned char bitmap[12];
      for (unsigned char i = 0; i < 12; i++) {
        bitmap[i] = i;
      }

      vpImage<unsigned char> I(bitmap, 3, 4, false);
      std::cout << "\nI:\n" << I << std::endl;
      I.init(2, 2, 0);
      std::cout << "I:\n" << I << std::endl;
    }
    {
      unsigned char bitmap[12];
      for (unsigned char i = 0; i < 12; i++) {
        bitmap[i] = i;
      }
      vpImage<unsigned char> I(bitmap, 3, 4, false);
      std::cout << "\nI:\n" << I << std::endl;
      I.init(bitmap, 4, 3, true);
      std::cout << "I:\n" << I << std::endl;
    }
    {
      unsigned char *bitmap = new unsigned char[12];
      {
        vpImage<unsigned char> I(bitmap, 3, 4, true);
      }
      delete[] bitmap;
    }
    {
      unsigned char *bitmap = new unsigned char[12];
      {
        vpImage<unsigned char> I(bitmap, 3, 4, false);
      }
      vpImage<unsigned char> I(bitmap, 3, 4, false);

      delete[] bitmap;

      bitmap = new unsigned char[16];
      I.init(bitmap, 4, 4, false);

      delete[] bitmap;
    }
    {
      unsigned char *bitmap = new unsigned char[12];
      vpImage<unsigned char> I(bitmap, 3, 4, false);
      I.destroy();
      I.init(bitmap, 3, 4, false);

      delete[] bitmap;

      I.destroy();
      bitmap = new unsigned char[16];
      I.init(bitmap, 4, 4, false);

      delete[] bitmap;
    }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    {
      unsigned char *bitmap = new unsigned char[12];
      vpImage<unsigned char> I = std::move(vpImage<unsigned char>(bitmap, 3, 4, false));
      if (bitmap != I.bitmap) {
        std::cout << "std::move(vpImage) failed" << std::endl;
        return EXIT_FAILURE;
      }
      delete[] bitmap;
    }
    {
      unsigned char *bitmap = new unsigned char[12];
      vpImage<unsigned char> I(std::move(vpImage<unsigned char>(bitmap, 3, 4, false)));
      if (bitmap != I.bitmap) {
        std::cout << "vpImage(td::move(vpImage)) failed" << std::endl;
        return EXIT_FAILURE;
      }
      delete[] bitmap;
    }
#endif
  }
  catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
