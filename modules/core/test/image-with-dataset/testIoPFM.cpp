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
 * Test image I/O for PFM file format.
 */

/*!
  \example testIoPFM.cpp

  Test image I/O for PFM file format.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030600)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <iostream>
#include <limits>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void checkColorImages(const vpImage<vpRGBf> &I1, const vpImage<vpRGBf> &I2)
{
  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      REQUIRE(vpMath::equal(I1[i][j].R, I2[i][j].R, std::numeric_limits<float>::epsilon()));
      REQUIRE(vpMath::equal(I1[i][j].G, I2[i][j].G, std::numeric_limits<float>::epsilon()));
      REQUIRE(vpMath::equal(I1[i][j].B, I2[i][j].B, std::numeric_limits<float>::epsilon()));
    }
  }
}

void checkGrayImages(const vpImage<float> &I1, const vpImage<float> &I2)
{
  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      REQUIRE(vpMath::equal(I1[i][j], I2[i][j], std::numeric_limits<float>::epsilon()));
    }
  }
}
} // namespace

TEST_CASE("HDR PFM image read", "[hdr_pfm_image_io]")
{
  SECTION("Little-endian (LSB)")
  {
    SECTION("Color")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_LSB.pfm");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<vpRGBf> I;
      vpImageIo::readPFM_HDR(I, imgPath);
      CHECK(I.getSize() > 0);
    }
    SECTION("Gray")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_LSB.pfm");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<float> I;
      vpImageIo::readPFM_HDR(I, imgPath);
      CHECK(I.getSize() > 0);
    }
  }

  SECTION("Big-endian (MSB)")
  {
    SECTION("Color")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_MSB.pfm");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<vpRGBf> I;
      vpImageIo::readPFM_HDR(I, imgPath);
      CHECK(I.getSize() > 0);
    }
    SECTION("Gray")
    {
      const std::string imgPath =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_MSB.pfm");
      REQUIRE(vpIoTools::checkFilename(imgPath));

      vpImage<float> I;
      vpImageIo::readPFM_HDR(I, imgPath);
      CHECK(I.getSize() > 0);
    }
  }

  SECTION("Endianness")
  {
    SECTION("Color")
    {
      const std::string imgPathLSB =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_LSB.pfm");
      const std::string imgPathMSB =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_MSB.pfm");
      REQUIRE(vpIoTools::checkFilename(imgPathLSB));
      REQUIRE(vpIoTools::checkFilename(imgPathMSB));

      vpImage<vpRGBf> I_LSB;
      vpImageIo::readPFM_HDR(I_LSB, imgPathLSB);
      REQUIRE(I_LSB.getSize() > 0);

      vpImage<vpRGBf> I_MSB;
      vpImageIo::readPFM_HDR(I_MSB, imgPathMSB);
      REQUIRE(I_MSB.getSize() > 0);

      REQUIRE(I_LSB.getHeight() == I_MSB.getHeight());
      REQUIRE(I_LSB.getWidth() == I_MSB.getWidth());

      checkColorImages(I_LSB, I_MSB);
    }
    SECTION("Gray")
    {
      const std::string imgPathLSB =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_LSB.pfm");
      const std::string imgPathMSB =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_MSB.pfm");
      REQUIRE(vpIoTools::checkFilename(imgPathLSB));
      REQUIRE(vpIoTools::checkFilename(imgPathMSB));

      vpImage<float> I_LSB;
      vpImageIo::readPFM_HDR(I_LSB, imgPathLSB);
      REQUIRE(I_LSB.getSize() > 0);

      vpImage<float> I_MSB;
      vpImageIo::readPFM_HDR(I_MSB, imgPathMSB);
      REQUIRE(I_MSB.getSize() > 0);

      REQUIRE(I_LSB.getHeight() == I_MSB.getHeight());
      REQUIRE(I_LSB.getWidth() == I_MSB.getWidth());

      checkGrayImages(I_LSB, I_MSB);
    }
  }
}

TEST_CASE("HDR PFM image write", "[hdr_pfm_image_io]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp = tmp_dir + "/testIoPFM_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  SECTION("Color")
  {
    const std::string imgPath =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_color_LSB.pfm");
    REQUIRE(vpIoTools::checkFilename(imgPath));

    vpImage<vpRGBf> I;
    vpImageIo::readPFM_HDR(I, imgPath);
    REQUIRE(I.getSize() > 0);

    vpImageIo::writePFM_HDR(I, vpIoTools::createFilePath(directory_filename_tmp, "write_color_pfm.pfm"));
    vpImage<vpRGBf> I_write;
    vpImageIo::readPFM_HDR(I_write, vpIoTools::createFilePath(directory_filename_tmp, "write_color_pfm.pfm"));

    REQUIRE(I.getHeight() == I_write.getHeight());
    REQUIRE(I.getWidth() == I_write.getWidth());

    checkColorImages(I, I_write);
  }
  SECTION("Gray")
  {
    const std::string imgPath =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "memorial/memorial_gray_LSB.pfm");
    REQUIRE(vpIoTools::checkFilename(imgPath));

    vpImage<float> I;
    vpImageIo::readPFM_HDR(I, imgPath);
    REQUIRE(I.getSize() > 0);

    vpImageIo::writePFM_HDR(I, vpIoTools::createFilePath(directory_filename_tmp, "write_gray_pfm.pfm"));
    vpImage<float> I_write;
    vpImageIo::readPFM_HDR(I_write, vpIoTools::createFilePath(directory_filename_tmp, "write_gray_pfm.pfm"));

    REQUIRE(I.getHeight() == I_write.getHeight());
    REQUIRE(I.getWidth() == I_write.getWidth());

    checkGrayImages(I, I_write);
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
  REQUIRE(vpIoTools::remove(tmp_dir));
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
