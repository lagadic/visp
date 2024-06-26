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
 * Test image morphology.
 */

/*!
  \example testImageMorphology.cpp

  \brief Test image morphology.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include "common.hpp"
#include <catch.hpp>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpImageMorphology.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
TEST_CASE("Binary image morphology", "[image_morphology]")
{
  unsigned char image_data[8 * 16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0,
                                      0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1,
                                      0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                                      1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                      1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                                      1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };

  vpImage<unsigned char> I(image_data, 8, 16, true);

  SECTION("Dilatation")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho_tpl = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageDilatationRef(I_morpho_ref, connexity);
      vpImageMorphology::dilatation(I_morpho_tpl, (unsigned char)1, (unsigned char)0, connexity);
      vpImageMorphology::dilatation<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho_tpl));
      CHECK((I_morpho_ref == I_morpho));
    }
    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho_tpl = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageDilatationRef(I_morpho_ref, connexity);
      vpImageMorphology::dilatation(I_morpho_tpl, (unsigned char)1, (unsigned char)0, connexity);
      vpImageMorphology::dilatation<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho_tpl));
      CHECK((I_morpho_ref == I_morpho));
    }
  }

  SECTION("Erosion")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho_tpl = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageErosionRef(I_morpho_ref, connexity);
      vpImageMorphology::erosion(I_morpho_tpl, (unsigned char)1, (unsigned char)0, connexity);
      vpImageMorphology::erosion<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho_tpl));
      CHECK((I_morpho_ref == I_morpho));
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho_tpl = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageErosionRef(I_morpho_ref, connexity);
      vpImageMorphology::erosion(I_morpho_tpl, (unsigned char)1, (unsigned char)0, connexity);
      vpImageMorphology::erosion<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho_tpl));
      CHECK((I_morpho_ref == I_morpho));
    }

    SECTION("8-connexity-size5")
    {
      vpImage<unsigned char> I_dilatation_ref(8, 16, 1);
      I_dilatation_ref[0][0] = 0;
      I_dilatation_ref[0][1] = 0;
      I_dilatation_ref[0][2] = 0;
      I_dilatation_ref[6][12] = 0;
      I_dilatation_ref[7][12] = 0;
      vpImage<unsigned char> I_dilatation = I;
      vpImage<unsigned char> I_erosion_ref(8, 16, 0);
      vpImage<unsigned char> I_erosion = I;

      const int size = 5;
      vpImageMorphology::dilatation(I_dilatation, size);
      vpImageMorphology::erosion(I_erosion, size);

      CHECK((I_dilatation_ref == I_dilatation));
      CHECK((I_erosion_ref == I_erosion));
    }
  }

  SECTION("Matlab reference")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      vpImage<unsigned char> I_dilatation = I;
      vpImageMorphology::dilatation<unsigned char>(I_dilatation, connexity);

      unsigned char image_data_dilatation[8 * 16] = {
          0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0,
          0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
          1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1,
          1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1 };
      vpImage<unsigned char> I_dilatation_ref(image_data_dilatation, 8, 16, true);
      CHECK((I_dilatation_ref == I_dilatation));

      vpImage<unsigned char> I_erosion = I_dilatation;
      vpImageMorphology::erosion<unsigned char>(I_erosion, connexity);

      unsigned char image_data_erosion[8 * 16] = {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1,
          0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
          1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
      vpImage<unsigned char> I_erosion_ref(image_data_erosion, 8, 16, true);
      CHECK((I_erosion_ref == I_erosion));
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      vpImage<unsigned char> I_dilatation = I;
      vpImageMorphology::dilatation<unsigned char>(I_dilatation, connexity);

      unsigned char image_data_dilatation[8 * 16] = {
          0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
          0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1,
          1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1 };
      vpImage<unsigned char> I_dilatation_ref(image_data_dilatation, 8, 16, true);
      CHECK((I_dilatation_ref == I_dilatation));

      vpImage<unsigned char> I_erosion = I_dilatation;
      vpImageMorphology::erosion<unsigned char>(I_erosion, connexity);

      unsigned char image_data_erosion[8 * 16] = {
          0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1,
          0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
          1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1 };
      vpImage<unsigned char> I_erosion_ref(image_data_erosion, 8, 16, true);
      CHECK((I_erosion_ref == I_erosion));
    }
  }
}

TEST_CASE("Gray image morphology", "[image_morphology]")
{
  vpImage<unsigned char> I;
  common_tools::magicSquare(I, 17);

  SECTION("Dilatation")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageDilatationRef(I_morpho_ref, connexity);
      vpImageMorphology::dilatation<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho));
    }
    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageDilatationRef(I_morpho_ref, connexity);
      vpImageMorphology::dilatation<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho));
    }

    SECTION("8-connexity-size5")
    {
      const int size = 5;
      vpImage<unsigned char> I_morpho(12, 12);
      unsigned char count = 1;
      for (int r = 0; r < 12; r++) {
        for (int c = 0; c < 12; c++) {
          I_morpho[r][c] = count;
          count++;
        }
      }
      unsigned char image_data_dilatation[12 * 12] = {
         27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  36,  36,
         39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  48,  48,
         51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  60,  60,
         63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  72,  72,
         75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  84,  84,
         87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  96,  96,
         99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 108, 108,
        111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 120, 120,
        123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 132, 132,
        135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 144, 144,
        135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 144, 144,
        135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 144, 144 };
      vpImage<unsigned char> I_dilatation_ref(image_data_dilatation, 12, 12, true);

      vpImageMorphology::dilatation<unsigned char>(I_morpho, size);

      CHECK((I_dilatation_ref == I_morpho));
    }
  }

  SECTION("Erosion")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageErosionRef(I_morpho_ref, connexity);
      vpImageMorphology::erosion<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho));
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      vpImage<unsigned char> I_morpho_ref = I;
      vpImage<unsigned char> I_morpho = I;

      common_tools::imageErosionRef(I_morpho_ref, connexity);
      vpImageMorphology::erosion<unsigned char>(I_morpho, connexity);

      CHECK((I_morpho_ref == I_morpho));
    }

    SECTION("8-connexity-size5")
    {
      const int size = 5;
      vpImage<unsigned char> I_morpho(12, 12);
      unsigned char count = 1;
      for (int r = 0; r < 12; r++) {
        for (int c = 0; c < 12; c++) {
          I_morpho[r][c] = count;
          count++;
        }
      }
      unsigned char image_data_erosion[12 * 12] = {
          1,   1,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,
          1,   1,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,
          1,   1,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,
         13,  13,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,
         25,  25,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,
         37,  37,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,
         49,  49,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,
         61,  61,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
         73,  73,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,
         85,  85,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,
         97,  97,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106,
        109, 109, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118 };
      vpImage<unsigned char> I_erosion_ref(image_data_erosion, 12, 12, true);

      vpImageMorphology::erosion<unsigned char>(I_morpho, size);

      CHECK((I_erosion_ref == I_morpho));
    }
  }

  SECTION("Matlab reference")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      vpImage<unsigned char> I_dilatation = I;
      vpImageMorphology::dilatation<unsigned char>(I_dilatation, connexity);

      unsigned char image_data_dilatation[17 * 17] = {
          174, 193, 212, 231, 250, 255, 255, 255, 255, 39,  58,  77,  96,  115, 134, 153, 154, 192, 211, 230, 249,
          255, 255, 255, 255, 38,  57,  76,  95,  114, 133, 152, 170, 172, 210, 229, 248, 255, 255, 255, 255, 37,
          56,  75,  94,  113, 132, 151, 170, 172, 190, 228, 247, 255, 255, 255, 255, 36,  55,  74,  93,  112, 131,
          150, 169, 187, 190, 208, 246, 255, 255, 255, 255, 51,  54,  73,  92,  111, 130, 149, 168, 187, 189, 208,
          226, 255, 255, 255, 255, 51,  53,  72,  91,  110, 129, 148, 167, 186, 204, 207, 226, 244, 255, 255, 255,
          50,  68,  71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 255, 255, 255, 49,  68,  70,  89,  108,
          127, 146, 165, 184, 203, 221, 224, 243, 255, 255, 255, 48,  67,  85,  88,  107, 126, 145, 164, 183, 202,
          221, 223, 242, 255, 255, 255, 47,  66,  85,  87,  106, 125, 144, 163, 182, 201, 220, 238, 241, 255, 255,
          255, 255, 65,  84,  102, 105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 255, 255, 45,  83,  102,
          104, 123, 142, 161, 180, 199, 218, 237, 255, 255, 255, 255, 255, 45,  63,  101, 119, 122, 141, 160, 179,
          198, 217, 236, 255, 255, 255, 255, 255, 44,  63,  81,  119, 121, 140, 159, 178, 197, 216, 235, 254, 255,
          255, 255, 255, 43,  62,  81,  99,  136, 139, 158, 177, 196, 215, 234, 253, 255, 255, 255, 255, 42,  61,
          80,  99,  117, 138, 157, 176, 195, 214, 233, 252, 255, 255, 255, 255, 41,  60,  79,  98,  117, 135, 156,
          175, 194, 213, 232, 251, 255, 255, 255, 255, 40,  59,  78,  97,  116, 135, 135 };
      vpImage<unsigned char> I_dilatation_ref(image_data_dilatation, 17, 17, true);
      CHECK((I_dilatation_ref == I_dilatation));

      vpImage<unsigned char> I_erosion = I_dilatation;
      vpImageMorphology::erosion<unsigned char>(I_erosion, connexity);

      unsigned char image_data_erosion[17 * 17] = {
          174, 174, 193, 212, 231, 250, 255, 255, 38,  39,  39,  58,  77,  96,  115, 134, 153, 174, 192, 211, 230,
          249, 255, 255, 37,  38,  38,  57,  76,  95,  114, 133, 152, 154, 192, 210, 229, 248, 255, 255, 36,  37,
          37,  56,  75,  94,  113, 132, 151, 170, 172, 210, 228, 247, 255, 255, 36,  36,  36,  55,  74,  93,  112,
          131, 150, 169, 172, 190, 228, 246, 255, 255, 51,  51,  36,  54,  73,  92,  111, 130, 149, 168, 187, 189,
          208, 246, 255, 255, 50,  51,  51,  53,  72,  91,  110, 129, 148, 167, 186, 189, 207, 226, 255, 255, 49,
          50,  50,  53,  71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 255, 48,  49,  49,  68,  70,  89,
          108, 127, 146, 165, 184, 203, 206, 224, 243, 255, 47,  48,  48,  67,  70,  88,  107, 126, 145, 164, 183,
          202, 221, 223, 242, 255, 255, 47,  47,  66,  85,  87,  106, 125, 144, 163, 182, 201, 220, 223, 241, 255,
          255, 45,  47,  65,  84,  87,  105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 45,  45,  65,  83,
          102, 104, 123, 142, 161, 180, 199, 218, 237, 240, 255, 255, 44,  45,  45,  83,  101, 104, 122, 141, 160,
          179, 198, 217, 236, 255, 255, 255, 43,  44,  44,  63,  101, 119, 121, 140, 159, 178, 197, 216, 235, 254,
          255, 255, 42,  43,  43,  62,  81,  119, 121, 139, 158, 177, 196, 215, 234, 253, 255, 255, 41,  42,  42,
          61,  80,  99,  136, 138, 157, 176, 195, 214, 233, 252, 255, 255, 40,  41,  41,  60,  79,  98,  117, 138,
          156, 175, 194, 213, 232, 251, 255, 255, 40,  40,  40,  59,  78,  97,  116, 135 };
      vpImage<unsigned char> I_erosion_ref(image_data_erosion, 17, 17, true);
      CHECK((I_erosion_ref == I_erosion));
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      vpImage<unsigned char> I_dilatation = I;
      vpImageMorphology::dilatation<unsigned char>(I_dilatation, connexity);

      unsigned char image_data_dilatation[17 * 17] = {
          192, 211, 230, 249, 255, 255, 255, 255, 255, 57,  76,  95,  114, 133, 152, 154, 154, 210, 229, 248, 255,
          255, 255, 255, 255, 255, 75,  94,  113, 132, 151, 170, 172, 172, 228, 247, 255, 255, 255, 255, 255, 255,
          74,  93,  112, 131, 150, 169, 171, 190, 190, 246, 255, 255, 255, 255, 255, 255, 73,  92,  111, 130, 149,
          168, 187, 189, 208, 208, 255, 255, 255, 255, 255, 255, 72,  91,  110, 129, 148, 167, 186, 188, 207, 226,
          226, 255, 255, 255, 255, 255, 71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 244, 255, 255, 255,
          255, 70,  89,  108, 127, 146, 165, 184, 203, 205, 224, 243, 255, 255, 255, 255, 255, 69,  88,  107, 126,
          145, 164, 183, 202, 221, 223, 242, 255, 255, 255, 255, 255, 85,  87,  106, 125, 144, 163, 182, 201, 220,
          222, 241, 255, 255, 255, 255, 65,  84,  86,  105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 255,
          255, 255, 83,  102, 104, 123, 142, 161, 180, 199, 218, 237, 239, 255, 255, 255, 255, 255, 255, 101, 103,
          122, 141, 160, 179, 198, 217, 236, 255, 255, 255, 255, 255, 255, 255, 63,  119, 121, 140, 159, 178, 197,
          216, 235, 254, 255, 255, 255, 255, 255, 255, 81,  81,  120, 139, 158, 177, 196, 215, 234, 253, 255, 255,
          255, 255, 255, 255, 80,  99,  99,  138, 157, 176, 195, 214, 233, 252, 255, 255, 255, 255, 255, 255, 79,
          98,  117, 117, 156, 175, 194, 213, 232, 251, 255, 255, 255, 255, 255, 255, 78,  97,  116, 135, 135, 156,
          175, 194, 213, 232, 251, 255, 255, 255, 255, 255, 59,  78,  97,  116, 135, 135 };
      vpImage<unsigned char> I_dilatation_ref(image_data_dilatation, 17, 17, true);
      CHECK((I_dilatation_ref == I_dilatation));

      vpImage<unsigned char> I_erosion = I_dilatation;
      vpImageMorphology::erosion<unsigned char>(I_erosion, connexity);

      unsigned char image_data_erosion[17 * 17] = {
          192, 192, 211, 230, 249, 255, 255, 255, 57,  57,  57,  76,  95,  114, 133, 152, 154, 192, 192, 211, 230,
          249, 255, 255, 74,  57,  57,  57,  76,  95,  114, 133, 152, 154, 210, 210, 229, 248, 255, 255, 73,  73,
          73,  74,  75,  94,  113, 132, 151, 170, 172, 228, 228, 247, 255, 255, 72,  72,  72,  73,  74,  93,  112,
          131, 150, 169, 171, 190, 246, 246, 255, 255, 71,  71,  71,  72,  73,  92,  111, 130, 149, 168, 187, 189,
          208, 255, 255, 255, 70,  70,  70,  71,  72,  91,  110, 129, 148, 167, 186, 188, 207, 226, 255, 255, 69,
          69,  69,  70,  71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 255, 85,  69,  69,  69,  70,  89,
          108, 127, 146, 165, 184, 203, 205, 224, 243, 255, 65,  65,  69,  69,  69,  88,  107, 126, 145, 164, 183,
          202, 221, 223, 242, 255, 255, 65,  65,  84,  85,  87,  106, 125, 144, 163, 182, 201, 220, 222, 241, 255,
          255, 255, 65,  65,  84,  86,  105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 63,  63,  83,  83,
          102, 104, 123, 142, 161, 180, 199, 218, 237, 239, 255, 255, 81,  63,  63,  101, 101, 103, 122, 141, 160,
          179, 198, 217, 236, 255, 255, 255, 80,  80,  63,  63,  119, 119, 121, 140, 159, 178, 197, 216, 235, 254,
          255, 255, 79,  79,  79,  80,  81,  120, 120, 139, 158, 177, 196, 215, 234, 253, 255, 255, 78,  78,  78,
          79,  80,  99,  138, 138, 157, 176, 195, 214, 233, 252, 255, 255, 59,  59,  59,  78,  79,  98,  117, 156,
          156, 175, 194, 213, 232, 251, 255, 255, 255, 59,  59,  59,  78,  97,  116, 135 };
      vpImage<unsigned char> I_erosion_ref(image_data_erosion, 17, 17, true);
      CHECK((I_erosion_ref == I_erosion));
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
