/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Test image warping.
 */

/*!
  \example testImageWarp.cpp

  Test image warping.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030300)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <iostream>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
static const double g_threshold_value = 0.5;
static const double g_threshold_percentage = 0.9;
static const double g_threshold_percentage_bilinear = 0.75;
static const double g_threshold_percentage_pers = 0.75;
static const double g_threshold_percentage_pers_bilinear = 0.65;

static const std::vector<vpImageTools::vpImageInterpolationType> interp_methods = { vpImageTools::INTERPOLATION_NEAREST,
                                                                                   vpImageTools::INTERPOLATION_LINEAR };
static const std::vector<std::string> interp_names = { "Nearest Neighbor", "Bilinear" };
static const std::vector<std::string> suffixes = { "_NN.png", "_bilinear.png" };

bool almostEqual(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, double threshold_val,
                 double threshold_percentage, double &percentage)
{
  double nb_valid = 0;

  if (I1.getHeight() != I2.getHeight() || I1.getWidth() != I2.getWidth()) {
    return false;
  }

  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      nb_valid += vpMath::abs(I1[i][j] - I2[i][j]) < threshold_val ? 1 : 0;
    }
  }

  percentage = nb_valid / I1.getSize();
  return percentage >= threshold_percentage;
}

bool almostEqual(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, double threshold_val,
                 double threshold_percentage, double &percentage)
{
  double nb_valid = 0;

  if (I1.getHeight() != I2.getHeight() || I1.getWidth() != I2.getWidth()) {
    return false;
  }

  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      if (vpMath::abs(I1[i][j].R - I2[i][j].R) < threshold_val) {
        nb_valid++;
      }
      if (vpMath::abs(I1[i][j].G - I2[i][j].G) < threshold_val) {
        nb_valid++;
      }
      if (vpMath::abs(I1[i][j].B - I2[i][j].B) < threshold_val) {
        nb_valid++;
      }
    }
  }

  percentage = nb_valid / (3 * I1.getSize());
  return percentage >= threshold_percentage;
}
} // namespace

TEST_CASE("Affine warp on grayscale", "[warp_image]")
{
  const std::string imgPath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<unsigned char> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);

  SECTION("Identity")
  {
    vpMatrix M(2, 3);
    M.eye();

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Empty destination")
        {
          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);
          CHECK((I == I_affine));
        }

        SECTION("Initialized destination")
        {
          vpImage<unsigned char> I_affine(I.getHeight(), I.getWidth(), 0);
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);
          CHECK((I == I_affine));
        }
      }
    }
  }

  SECTION("Rotation 45 deg")
  {
    vpMatrix M(2, 3);
    M.eye();

    const double theta = vpMath::rad(45);
    M[0][0] = cos(theta);
    M[0][1] = -sin(theta);
    M[0][2] = I.getWidth() / 2.0;
    M[1][0] = sin(theta);
    M[1][1] = cos(theta);
    M[1][2] = I.getHeight() / 2.0;

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Against reference implementation")
        {
          vpImage<unsigned char> I_ref;
          vpImageTools::warpImage(I, M, I_ref, interp_methods[i], false);

          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref, I_affine, g_threshold_value, g_threshold_percentage, percentage);
          std::cout << "Percentage valid pixels (45 deg " << interp_names[i] << " Ref): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against OpenCV")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/cv_warp_affine_rot_45_gray" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<unsigned char> I_ref_opencv;

          vpImageIo::read(I_ref_opencv, refImgPath);

          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_opencv, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (45 deg " << interp_names[i] << " OpenCV): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against PIL")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/pil_warp_affine_rot_45_gray" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<unsigned char> I_ref_pil;
          vpImageIo::read(I_ref_pil, refImgPath);

          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i], false, true);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_pil, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (45 deg " << interp_names[i] << " PIL): " << percentage << std::endl;
          CHECK(equal);
        }
      }
    }
  }

  SECTION("SRT")
  {
    vpMatrix M(2, 3);
    M.eye();

    const double theta = vpMath::rad(-67);
    const double scale = 0.83;
    M[0][0] = scale * cos(theta);
    M[0][1] = -scale * sin(theta);
    M[0][2] = I.getWidth() / 2.0 + 17;
    M[1][0] = scale * sin(theta);
    M[1][1] = scale * cos(theta);
    M[1][2] = I.getHeight() / 2.0 - 23;

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Against reference implementation")
        {
          vpImage<unsigned char> I_ref;
          vpImageTools::warpImage(I, M, I_ref, interp_methods[i], false);

          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref, I_affine, g_threshold_value, g_threshold_percentage, percentage);
          std::cout << "Percentage valid pixels (SRT " << interp_names[i] << " Ref): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against OpenCV")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/cv_warp_affine_SRT_gray" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<unsigned char> I_ref_opencv;
          vpImageIo::read(I_ref_opencv, refImgPath);

          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_opencv, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (SRT " << interp_names[i] << " OpenCV): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against PIL")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/pil_warp_affine_SRT_gray" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<unsigned char> I_ref_pil;
          vpImageIo::read(I_ref_pil, refImgPath);

          vpImage<unsigned char> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i], false, true);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_pil, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (SRT " << interp_names[i] << " PIL): " << percentage << std::endl;
          CHECK(equal);
        }
      }
    }
  }
}

TEST_CASE("Affine warp on color", "[warp_image]")
{
  const std::string imgPath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.ppm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<vpRGBa> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);

  SECTION("Identity")
  {
    vpMatrix M(2, 3);
    M.eye();

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Empty destination")
        {
          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);
          CHECK((I == I_affine));
        }

        SECTION("Initialized destination")
        {
          vpImage<vpRGBa> I_affine(I.getHeight(), I.getWidth(), vpRGBa(0));
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);
          CHECK((I == I_affine));
        }
      }
    }
  }

  SECTION("Rotation 45 deg")
  {
    vpMatrix M(2, 3);
    M.eye();

    const double theta = vpMath::rad(45);
    M[0][0] = cos(theta);
    M[0][1] = -sin(theta);
    M[0][2] = I.getWidth() / 2.0;
    M[1][0] = sin(theta);
    M[1][1] = cos(theta);
    M[1][2] = I.getHeight() / 2.0;

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Against reference implementation")
        {
          vpImage<vpRGBa> I_ref;
          vpImageTools::warpImage(I, M, I_ref, interp_methods[i], false);

          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref, I_affine, g_threshold_value, g_threshold_percentage, percentage);
          std::cout << "Percentage valid pixels (45 deg " << interp_names[i] << " Ref): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against OpenCV")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/cv_warp_affine_rot_45_color" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<vpRGBa> I_ref_opencv;
          vpImageIo::read(I_ref_opencv, refImgPath);

          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_opencv, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (45 deg " << interp_names[i] << " OpenCV): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against PIL")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/pil_warp_affine_rot_45_color" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<vpRGBa> I_ref_pil;
          vpImageIo::read(I_ref_pil, refImgPath);

          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i], false, true);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_pil, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (45 deg " << interp_names[i] << " PIL): " << percentage << std::endl;
          CHECK(equal);
        }
      }
    }
  }

  SECTION("SRT")
  {
    vpMatrix M(2, 3);
    M.eye();

    const double theta = vpMath::rad(-67);
    const double scale = 0.83;
    M[0][0] = scale * cos(theta);
    M[0][1] = -scale * sin(theta);
    M[0][2] = I.getWidth() / 2.0 + 17;
    M[1][0] = scale * sin(theta);
    M[1][1] = scale * cos(theta);
    M[1][2] = I.getHeight() / 2.0 - 23;

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Against reference implementation")
        {
          vpImage<vpRGBa> I_ref;
          vpImageTools::warpImage(I, M, I_ref, interp_methods[i], false);

          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (SRT " << interp_names[i] << " Ref): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against OpenCV")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/cv_warp_affine_SRT_color" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<vpRGBa> I_ref_opencv;
          vpImageIo::read(I_ref_opencv, refImgPath);

          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i]);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_opencv, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (SRT " << interp_names[i] << " OpenCV): " << percentage << std::endl;
          CHECK(equal);
        }

        SECTION("Against PIL")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/pil_warp_affine_SRT_color" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<vpRGBa> I_ref_pil;
          vpImageIo::read(I_ref_pil, refImgPath);

          vpImage<vpRGBa> I_affine;
          vpImageTools::warpImage(I, M, I_affine, interp_methods[i], false, true);

          double percentage = 0.0;
          bool equal = almostEqual(I_ref_pil, I_affine, g_threshold_value,
                                   (i == 0) ? g_threshold_percentage : g_threshold_percentage_bilinear, percentage);
          std::cout << "Percentage valid pixels (SRT " << interp_names[i] << " PIL): " << percentage << std::endl;
          CHECK(equal);
        }
      }
    }
  }
}

TEST_CASE("Perspective warp on grayscale", "[warp_image]")
{
  const std::string imgPath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<unsigned char> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);

  SECTION("Identity")
  {
    vpMatrix M(3, 3);
    M.eye();

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Empty destination")
        {
          vpImage<unsigned char> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);
          CHECK((I == I_perspective));
        }

        SECTION("Initialized destination")
        {
          vpImage<unsigned char> I_perspective(I.getHeight(), I.getWidth(), 0);
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);
          CHECK((I == I_perspective));
        }
      }
    }
  }

  SECTION("Rotation 45 deg")
  {
    vpMatrix M(3, 3);
    M.eye();

    const double theta = vpMath::rad(45);
    M[0][0] = cos(theta);
    M[0][1] = -sin(theta);
    M[0][2] = I.getWidth() / 2.0;
    M[1][0] = sin(theta);
    M[1][1] = cos(theta);
    M[1][2] = I.getHeight() / 2.0;

    SECTION("Nearest Neighbor")
    {
      vpImage<unsigned char> I_ref;
      vpImageTools::warpImage(I, M, I_ref, vpImageTools::INTERPOLATION_NEAREST, false);

      vpImage<unsigned char> I_perspective;
      vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_NEAREST);

      double percentage = 0.0;
      bool equal = almostEqual(I_ref, I_perspective, g_threshold_value, g_threshold_percentage, percentage);
      std::cout << "Percentage valid pixels (persp 45 deg): " << percentage << std::endl;
      CHECK(equal);
    }
  }

  SECTION("Homography")
  {
    vpMatrix M(3, 3);
    M.eye();

    M[0][0] = 1.8548;
    M[0][1] = -0.0402;
    M[0][2] = 114.9;
    M[1][0] = 1.1209;
    M[1][1] = 4.0106;
    M[1][2] = 111;
    M[2][0] = 0.0022;
    M[2][1] = 0.0064;

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Against reference implementation")
        {
          vpImage<unsigned char> I_ref;
          vpImageTools::warpImage(I, M, I_ref, interp_methods[i], false);

          vpImage<unsigned char> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);

          double percentage = 0.0;
          bool equal =
            almostEqual(I_ref, I_perspective, g_threshold_value,
                        (i == 0) ? g_threshold_percentage_pers : g_threshold_percentage_pers_bilinear, percentage);
          std::cout << "Percentage valid pixels (Homography " << interp_names[i] << " Ref): " << percentage
            << std::endl;
          CHECK(equal);
        }

        SECTION("Against OpenCV")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/cv_warp_perspective_gray" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<unsigned char> I_ref_opencv;
          vpImageIo::read(I_ref_opencv, refImgPath);

          vpImage<unsigned char> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);

          double percentage = 0.0;
          bool equal =
            almostEqual(I_ref_opencv, I_perspective, g_threshold_value,
                        (i == 0) ? g_threshold_percentage_pers : g_threshold_percentage_pers_bilinear, percentage);
          std::cout << "Percentage valid pixels (Homography " << interp_names[i] << " OpenCV): " << percentage
            << std::endl;
          CHECK(equal);
        }

        SECTION("Against PIL")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/pil_warp_perspective_gray" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<unsigned char> I_ref_pil;
          vpImageIo::read(I_ref_pil, refImgPath);

          vpImage<unsigned char> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i], false, true);

          double percentage = 0.0;
          bool equal =
            almostEqual(I_ref_pil, I_perspective, g_threshold_value,
                        (i == 0) ? g_threshold_percentage_pers : g_threshold_percentage_pers_bilinear, percentage);
          std::cout << "Percentage valid pixels (Homography " << interp_names[i] << " PIL): " << percentage
            << std::endl;
          CHECK(equal);
        }
      }
    }
  }
}

TEST_CASE("Perspective warp on color", "[warp_image]")
{
  const std::string imgPath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.ppm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<vpRGBa> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);

  SECTION("Identity")
  {
    vpMatrix M(3, 3);
    M.eye();

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Empty destination")
        {
          vpImage<vpRGBa> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);
          CHECK((I == I_perspective));
        }

        SECTION("Initialized destination")
        {
          vpImage<vpRGBa> I_perspective(I.getHeight(), I.getWidth(), vpRGBa(0));
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);
          CHECK((I == I_perspective));
        }
      }
    }
  }

  SECTION("Homography")
  {
    vpMatrix M(3, 3);
    M.eye();

    M[0][0] = 1.8548;
    M[0][1] = -0.0402;
    M[0][2] = 114.9;
    M[1][0] = 1.1209;
    M[1][1] = 4.0106;
    M[1][2] = 111;
    M[2][0] = 0.0022;
    M[2][1] = 0.0064;

    for (size_t i = 0; i < interp_methods.size(); i++) {
      SECTION(interp_names[i])
      {
        SECTION("Against reference implementation")
        {
          vpImage<vpRGBa> I_ref;
          vpImageTools::warpImage(I, M, I_ref, interp_methods[i], false);

          vpImage<vpRGBa> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);

          double percentage = 0.0;
          bool equal =
            almostEqual(I_ref, I_perspective, g_threshold_value,
                        (i == 0) ? g_threshold_percentage_pers : g_threshold_percentage_pers_bilinear, percentage);
          std::cout << "Percentage valid pixels (Homography " << interp_names[i] << " Ref): " << percentage
            << std::endl;
          CHECK(equal);
        }

        SECTION("Against OpenCV")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/cv_warp_perspective_color" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<vpRGBa> I_ref_opencv;
          vpImageIo::read(I_ref_opencv, refImgPath);

          vpImage<vpRGBa> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i]);

          double percentage = 0.0;
          bool equal =
            almostEqual(I_ref_opencv, I_perspective, g_threshold_value,
                        (i == 0) ? g_threshold_percentage_pers : g_threshold_percentage_pers_bilinear, percentage);
          std::cout << "Percentage valid pixels (Homography " << interp_names[i] << " OpenCV): " << percentage
            << std::endl;
          CHECK(equal);
        }

        SECTION("Against PIL")
        {
          const std::string refImgPath = vpIoTools::createFilePath(
              vpIoTools::getViSPImagesDataPath(), std::string("warp/pil_warp_perspective_color" + suffixes[i]));
          REQUIRE(vpIoTools::checkFilename(refImgPath));
          vpImage<vpRGBa> I_ref_pil;
          vpImageIo::read(I_ref_pil, refImgPath);

          vpImage<vpRGBa> I_perspective;
          vpImageTools::warpImage(I, M, I_perspective, interp_methods[i], false, true);

          double percentage = 0.0;
          bool equal =
            almostEqual(I_ref_pil, I_perspective, g_threshold_value,
                        (i == 0) ? g_threshold_percentage_pers : g_threshold_percentage_pers_bilinear, percentage);
          std::cout << "Percentage valid pixels (Homography " << interp_names[i] << " PIL): " << percentage
            << std::endl;
          CHECK(equal);
        }
      }
    }
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
