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
 * Test image I/O for EXR file format.
 */

/*!
  \example testIoEXR.cpp

  Test image I/O for EXR file format.
*/

#include <visp3/core/vpConfig.h>

// EXR format is only supported when OpenCV or TinyEXR 3rd parties are available
#if defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030600) && \
    defined(HAVE_OPENCV_IMGCODECS) && defined(VISP_HAVE_TINYEXR)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <iostream>
#include <limits>
#include <visp3/core/vpEndian.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
#ifdef VISP_LITTLE_ENDIAN
void checkColorImages(const vpImage<vpRGBf> &I1, const vpImage<vpRGBf> &I2,
                      float epsilon = std::numeric_limits<float>::epsilon())
{
  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      REQUIRE(vpMath::equal(I1[i][j].R, I2[i][j].R, epsilon));
      REQUIRE(vpMath::equal(I1[i][j].G, I2[i][j].G, epsilon));
      REQUIRE(vpMath::equal(I1[i][j].B, I2[i][j].B, epsilon));
    }
  }
}

void checkGrayImages(const vpImage<float> &I1, const vpImage<float> &I2,
                     float epsilon = std::numeric_limits<float>::epsilon())
{
  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      REQUIRE(vpMath::equal(I1[i][j], I2[i][j], epsilon));
    }
  }
}
#endif
} // namespace

TEST_CASE("EXR image read", "[exr_image_io]")
{
// Disable the tests if big endian for now.
// See: https://github.com/syoyo/tinyexr/issues/189#issuecomment-1465174904
#ifdef VISP_LITTLE_ENDIAN
  SECTION("Color")
  {
    const std::string imgPathRef = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_LSB.pfm");
    REQUIRE(vpIoTools::checkFilename(imgPathRef));

    vpImage<vpRGBf> I_ref;
    vpImageIo::readPFM_HDR(I_ref, imgPathRef);
    CHECK(I_ref.getSize() > 0);

    SECTION("32-bits")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_32bits.exr");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<vpRGBf> I;
      vpImageIo::readEXR(I, imgPath);
      CHECK(I.getSize() > 0);
      checkColorImages(I_ref, I);
    }

    SECTION("16-bits")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_16bits.exr");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<vpRGBf> I;
      vpImageIo::readEXR(I, imgPath);
      CHECK(I.getSize() > 0);
      checkColorImages(I_ref, I, 0.00097656f);
    }
  }

  SECTION("Gray")
  {
    const std::string imgPathRef =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_LSB.pfm");
    REQUIRE(vpIoTools::checkFilename(imgPathRef));

    vpImage<float> I_ref;
    vpImageIo::readPFM_HDR(I_ref, imgPathRef);
    CHECK(I_ref.getSize() > 0);

    SECTION("32-bits")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_32bits.exr");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<float> I;
      vpImageIo::readEXR(I, imgPath);
      CHECK(I.getSize() > 0);
      checkGrayImages(I_ref, I);
    }

    SECTION("16-bits")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_16bits.exr");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<float> I;
      vpImageIo::readEXR(I, imgPath);
      CHECK(I.getSize() > 0);
      checkGrayImages(I_ref, I, 0.00097656f);
    }
  }
#endif
}

TEST_CASE("EXR image write", "[exr_image_io]")
{
#ifdef VISP_LITTLE_ENDIAN
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp = tmp_dir + "/testIoEXR_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  SECTION("Color")
  {
    const std::string imgPath =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_32bits.exr");
    REQUIRE(vpIoTools::checkFilename(imgPath));

    vpImage<vpRGBf> I;
    vpImageIo::readEXR(I, imgPath);
    REQUIRE(I.getSize() > 0);

    vpImageIo::writeEXR(I, vpIoTools::createFilePath(directory_filename_tmp, "write_color_exr.exr"));
    vpImage<vpRGBf> I_write;
    vpImageIo::readEXR(I_write, vpIoTools::createFilePath(directory_filename_tmp, "write_color_exr.exr"));

    REQUIRE(I.getHeight() == I_write.getHeight());
    REQUIRE(I.getWidth() == I_write.getWidth());

    checkColorImages(I, I_write);
  }

  SECTION("Gray")
  {
    const std::string imgPath =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_32bits.exr");
    REQUIRE(vpIoTools::checkFilename(imgPath));

    vpImage<float> I;
    vpImageIo::readEXR(I, imgPath);
    REQUIRE(I.getSize() > 0);

    vpImageIo::writeEXR(I, vpIoTools::createFilePath(directory_filename_tmp, "write_gray_exr.exr"));
    vpImage<float> I_write;
    vpImageIo::readEXR(I_write, vpIoTools::createFilePath(directory_filename_tmp, "write_gray_exr.exr"));

    REQUIRE(I.getHeight() == I_write.getHeight());
    REQUIRE(I.getWidth() == I_write.getWidth());

    checkGrayImages(I, I_write);
  }

  CHECK(vpIoTools::remove(directory_filename_tmp));
  CHECK(vpIoTools::remove(tmp_dir));
#endif
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
