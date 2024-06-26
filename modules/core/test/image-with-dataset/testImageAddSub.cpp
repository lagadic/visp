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
 * Test image addition / subtraction.
 */

#include <visp3/core/vpConfig.h>

/*!
  \example testImageAddSub.cpp

  \brief Test images addition / subtraction.
*/

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include "common.hpp"
#include <catch.hpp>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Test vpImageTools::imageAdd()", "[image_add]")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  SECTION("I + Inull = I")
  {
    vpImage<unsigned char> Inull(I.getHeight(), I.getWidth(), 0);
    vpImage<unsigned char> Iadd;
    vpImageTools::imageAdd(I, Inull, Iadd);
    CHECK((Iadd == I));
  }

  SECTION("I + I without saturation")
  {
    const bool saturation = false;
    vpImage<unsigned char> Iref;
    common_tools::imageAddRef(I, I, Iref, saturation);

    vpImage<unsigned char> Iadd;
    vpImageTools::imageAdd(I, I, Iadd, saturation);
    CHECK((Iadd == Iref));
  }

  SECTION("I + I with saturation")
  {
    const bool saturation = true;
    vpImage<unsigned char> Iref;
    common_tools::imageAddRef(I, I, Iref, saturation);

    vpImage<unsigned char> Iadd;
    vpImageTools::imageAdd(I, I, Iadd, saturation);
    CHECK((Iadd == Iref));
  }
}

TEST_CASE("Test vpImageTools::imageDifference()", "[image_difference]")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  SECTION("I - Inull = I")
  {
    vpImage<unsigned char> Inull(I.getHeight(), I.getWidth(), 0);
    vpImage<unsigned char> Isub;
    vpImageTools::imageSubtract(I, Inull, Isub);
    CHECK((Isub == I));
  }

  SECTION("I - I2 without saturation")
  {
    const bool saturation = false;
    vpImage<unsigned char> Iref;
    vpImage<unsigned char> I2(I.getHeight(), I.getWidth());
    common_tools::fill(I2);

    common_tools::imageSubtractRef(I, I2, Iref, saturation);

    vpImage<unsigned char> Isub;
    vpImageTools::imageSubtract(I, I2, Isub, saturation);
    CHECK((Isub == Iref));
  }

  SECTION("I - I2 with saturation ")
  {
    const bool saturation = true;
    vpImage<unsigned char> Iref;
    vpImage<unsigned char> I2(I.getHeight(), I.getWidth());
    common_tools::fill(I2);

    common_tools::imageSubtractRef(I, I2, Iref, saturation);

    vpImage<unsigned char> Isub;
    vpImageTools::imageSubtract(I, I2, Isub, saturation);
    CHECK((Isub == Iref));
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
