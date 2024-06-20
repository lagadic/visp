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
 * Check that the different image I/O backends work correctly.
 */

/*!
  \example testImageLoadSave.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030500) && defined(VISP_HAVE_THREADS)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <thread>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static const std::string ipath = vpIoTools::getViSPImagesDataPath();
static const std::string path = ipath + "/Solvay/Solvay_conference_1927_Version2_640x440";

static const double ccThreshPNG = 1.0;
static const double ccThreshJPG = 0.99;

static const std::vector<vpImageIo::vpImageIoBackendType> backends
{
#if defined(VISP_HAVE_JPEG) && defined(VISP_HAVE_PNG)
  vpImageIo::IO_SYSTEM_LIB_BACKEND,
#endif
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
  vpImageIo::IO_OPENCV_BACKEND,
#endif
#if defined VISP_HAVE_SIMDLIB
  vpImageIo::IO_SIMDLIB_BACKEND,
#endif
#if defined VISP_HAVE_STBIMAGE
  vpImageIo::IO_STB_IMAGE_BACKEND
#endif
};
static const std::vector<std::string> backendNamesJpeg
{
#if defined(VISP_HAVE_JPEG)
  "libjpeg",
#endif
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
      "OpenCV",
#endif
      "simd", "stb"
};
static std::vector<std::string> backendNamesPng
{
#if defined(VISP_HAVE_PNG)
  "libpng",
#endif
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
      "OpenCV",
#endif
      "simd", "stb"
};

static const std::vector<vpImageIo::vpImageIoBackendType> backendsInMemory
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
  vpImageIo::IO_OPENCV_BACKEND,
#endif
#if defined VISP_HAVE_STBIMAGE
  vpImageIo::IO_STB_IMAGE_BACKEND
#endif
};
static std::vector<std::string> backendNamesPngInMemory
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
  "OpenCV",
#endif
#if defined VISP_HAVE_STBIMAGE
  "stb"
#endif
};

static const unsigned int imgWidth = 640;
static const unsigned int imgHeight = 440;

namespace
{
double computePearsonCC(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2)
{
  double m1 = I1.getSum() / I1.getSize();
  double m2 = I2.getSum() / I2.getSize();

  double num = 0, den1 = 0, den2 = 0;
  for (unsigned int i = 0; i < I1.getSize(); i++) {
    double x1 = I1.bitmap[i];
    double x2 = I2.bitmap[i];
    double x1_m = x1 - m1;
    double x2_m = x2 - m2;
    num += x1_m * x2_m;
    den1 += x1_m * x1_m;
    den2 += x2_m * x2_m;
  }

  return num / (std::sqrt(den1) * std::sqrt(den2));
}

double computePearsonCC(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2)
{
  double m1 = I1.getSum() / (3 * I1.getSize());
  double m2 = I2.getSum() / (3 * I2.getSize());

  double num = 0, den1 = 0, den2 = 0;
  for (unsigned int i = 0; i < I1.getSize(); i++) {
    double x1 = I1.bitmap[i].R + I1.bitmap[i].G + I1.bitmap[i].B;
    double x2 = I2.bitmap[i].R + I2.bitmap[i].G + I2.bitmap[i].B;
    double x1_m = x1 - m1;
    double x2_m = x2 - m2;
    num += x1_m * x2_m;
    den1 += x1_m * x1_m;
    den2 += x2_m * x2_m;
  }

  return num / (std::sqrt(den1) * std::sqrt(den2));
}
} // namespace

TEST_CASE("Test grayscale JPEG image loading", "[image_I/O]")
{
  vpImage<unsigned char> I_ref;
  vpImageIo::read(I_ref, path + ".jpg");

  for (size_t j = 0; j < backends.size(); j++) {
    vpImage<unsigned char> I;

    SECTION(backendNamesJpeg[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::read(I, path + ".jpg", backends[j]));
      double cc = computePearsonCC(I_ref, I);
      std::cout << backendNamesJpeg[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc >= Approx(ccThreshJPG));
      CHECK(I.getWidth() == imgWidth);
      CHECK(I.getHeight() == imgHeight);
    };
  }
}

TEST_CASE("Test RGBA JPEG image loading", "[image_I/O]")
{
  vpImage<vpRGBa> I_ref;
  vpImageIo::read(I_ref, path + ".jpg");

  for (size_t j = 0; j < backends.size(); j++) {
    vpImage<vpRGBa> I;

    SECTION(backendNamesJpeg[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::read(I, path + ".jpg", backends[j]));
      double cc = computePearsonCC(I_ref, I);
      std::cout << backendNamesJpeg[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc >= Approx(ccThreshJPG));
      CHECK(I.getWidth() == imgWidth);
      CHECK(I.getHeight() == imgHeight);
    };
  }
}

TEST_CASE("Test grayscale PNG image loading", "[image_I/O]")
{
  vpImage<unsigned char> I_ref;
  vpImageIo::read(I_ref, path + ".png");

  for (size_t j = 0; j < backends.size(); j++) {
    vpImage<unsigned char> I;

    SECTION(backendNamesPng[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::read(I, path + ".png", backends[j]));
      double cc = computePearsonCC(I_ref, I);
      std::cout << backendNamesPng[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc == Approx(ccThreshPNG));
      CHECK(I.getWidth() == imgWidth);
      CHECK(I.getHeight() == imgHeight);
    };
  }
}

TEST_CASE("Test RGBA PNG image loading", "[image_I/O]")
{
  vpImage<vpRGBa> I_ref;
  vpImageIo::read(I_ref, path + ".png");

  for (size_t j = 0; j < backends.size(); j++) {
    vpImage<vpRGBa> I;

    SECTION(backendNamesPng[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::read(I, path + ".png", backends[j]));
      double cc = computePearsonCC(I_ref, I);
      std::cout << backendNamesPng[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc == Approx(ccThreshPNG));
      CHECK(I.getWidth() == imgWidth);
      CHECK(I.getHeight() == imgHeight);
    };
  }
}

TEST_CASE("Test grayscale JPEG image saving", "[image_I/O]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  vpImage<unsigned char> I;
  vpImageIo::read(I, path + ".png");

  for (size_t j = 0; j < backends.size(); j++) {
    SECTION(backendNamesJpeg[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.jpg", backends[j]));
      vpImage<unsigned char> I_read;
      vpImageIo::read(I_read, directory_filename_tmp + "/ViSP_tmp_perf_write.jpg");
      double cc = computePearsonCC(I_read, I);
      std::cout << backendNamesPng[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc >= Approx(ccThreshJPG));
    };
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Test RGBA JPEG image saving", "[image_I/O]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  vpImage<vpRGBa> I;
  vpImageIo::read(I, path + ".png");

  for (size_t j = 0; j < backends.size(); j++) {
    SECTION(backendNamesJpeg[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.jpg", backends[j]));
      vpImage<vpRGBa> I_read;
      vpImageIo::read(I_read, directory_filename_tmp + "/ViSP_tmp_perf_write.jpg");
      double cc = computePearsonCC(I_read, I);
      std::cout << backendNamesPng[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc >= Approx(ccThreshJPG));
    };
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Test grayscale PNG image saving", "[image_I/O]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  vpImage<unsigned char> I;
  vpImageIo::read(I, path + ".png");

  for (size_t j = 0; j < backends.size(); j++) {
    SECTION(backendNamesPng[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.png", backends[j]));
      vpImage<unsigned char> I_read;
      vpImageIo::read(I_read, directory_filename_tmp + "/ViSP_tmp_perf_write.png");
      double cc = computePearsonCC(I_read, I);
      std::cout << backendNamesPng[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc == Approx(ccThreshPNG));
    };
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Test RGBA PNG image saving", "[image_I/O]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  vpImage<vpRGBa> I;
  vpImageIo::read(I, path + ".png");

  for (size_t j = 0; j < backends.size(); j++) {
    SECTION(backendNamesPng[j] + " backend")
    {
      CHECK_NOTHROW(vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.png", backends[j]));
      vpImage<vpRGBa> I_read;
      vpImageIo::read(I_read, directory_filename_tmp + "/ViSP_tmp_perf_write.png");
      double cc = computePearsonCC(I_read, I);
      std::cout << backendNamesPng[j] << " backend Pearson correlation coefficient: " << cc << std::endl;
      CHECK(cc == Approx(ccThreshPNG));
    };
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Test grayscale in-memory PNG image encoding/decoding", "[image_I/O]")
{
  vpImage<unsigned char> I_ref;
  vpImageIo::read(I_ref, path + ".png");
  REQUIRE(I_ref.getSize() > 0);

  for (size_t i = 0; i < backendsInMemory.size(); i++) {
    SECTION(backendNamesPngInMemory[i] + " backend")
    {
      std::vector<unsigned char> buffer;
      vpImageIo::writePNGtoMem(I_ref, buffer, backendsInMemory[i]);
      REQUIRE(!buffer.empty());

      for (size_t j = 0; j < backendsInMemory.size(); j++) {
        SECTION(backendNamesPngInMemory[i] + " backend")
        {
          vpImage<unsigned char> I;
          vpImageIo::readPNGfromMem(buffer, I, backendsInMemory[j]);
          CHECK(I_ref == I);
        };
      }
    };
  }
}

TEST_CASE("Test color in-memory PNG image encoding/decoding", "[image_I/O]")
{
  vpImage<vpRGBa> I_ref;
  vpImageIo::read(I_ref, path + ".png");
  REQUIRE(I_ref.getSize() > 0);

  for (size_t i = 0; i < backendsInMemory.size(); i++) {
    SECTION(backendNamesPngInMemory[i] + " backend")
    {
      {
        std::vector<unsigned char> buffer;
        const bool saveAlpha = false;
        vpImageIo::writePNGtoMem(I_ref, buffer, backendsInMemory[i], saveAlpha);
        REQUIRE(!buffer.empty());

        for (size_t j = 0; j < backendsInMemory.size(); j++) {
          SECTION(backendNamesPngInMemory[i] + " backend")
          {
            vpImage<vpRGBa> I;
            vpImageIo::readPNGfromMem(buffer, I, backendsInMemory[j]);
            CHECK(I_ref == I);
          };
        }
      }
      {
        std::vector<unsigned char> buffer;
        const bool saveAlpha = true;
        vpImageIo::writePNGtoMem(I_ref, buffer, backendsInMemory[i], saveAlpha);
        REQUIRE(!buffer.empty());

        for (size_t j = 0; j < backendsInMemory.size(); j++) {
          SECTION(backendNamesPngInMemory[i] + " backend")
          {
            vpImage<vpRGBa> I;
            vpImageIo::readPNGfromMem(buffer, I, backendsInMemory[j]);
            CHECK(I_ref == I);
          };
        }
      }
    };
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
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
