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
 * Test vpEndian.
 */

/*!
  \example catchEndian.cpp
 */
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEndian.h>

#if defined(VISP_HAVE_EIGEN3) && defined(VISP_HAVE_CATCH2)

#include <catch_amalgamated.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Test reinterpret_cast_uchar_to_uint16_LE", "[vpEndian_test]")
{
  unsigned char bitmap[] = { 100, 110, 120, 130 };
  uint16_t val01 = vpEndian::reinterpret_cast_uchar_to_uint16_LE(bitmap);
  uint16_t val12 = vpEndian::reinterpret_cast_uchar_to_uint16_LE(bitmap + 2);

  CHECK((val01 & 0x00FF) == bitmap[0]);
  CHECK(((val01 & 0xFF00) >> 8) == bitmap[1]);
  CHECK((val01 >> 8) == bitmap[1]);

  CHECK((val12 & 0x00FF) == bitmap[2]);
  CHECK(((val12 & 0xFF00) >> 8) == bitmap[3]);
  CHECK((val12 >> 8) == bitmap[3]);
}

TEST_CASE("Test bitwise shift operators and zero fill", "[vpEndian_test]")
{
  // https://docs.microsoft.com/en-us/cpp/cpp/left-shift-and-right-shift-operators-input-and-output?view=vs-2019
  // https://devblogs.microsoft.com/cppblog/hello-arm-exploring-undefined-unspecified-and-implementation-defined-behavior-in-c/
  for (uint16_t i = 0; i < 60000; i += 500) {
    REQUIRE(((i >> 8) & 0x00FF) == (i >> 8));
  }
}

TEST_CASE("Test endianness conversion", "[vpEndian_test]")
{
  SECTION("LE --> BE --> LE / BE --> LE --> BE")
  {
    SECTION("uint16_t")
    {
      uint16_t val_LE = 123;
      uint16_t val_LE_2_BE = vpEndian::swap16bits(val_LE);
      CHECK(val_LE == vpEndian::swap16bits(val_LE_2_BE));
    }
    SECTION("int16_t")
    {
      int16_t val_LE = -123;
      int16_t val_LE_2_BE = static_cast<int16_t>(vpEndian::swap16bits(static_cast<uint16_t>(val_LE)));
      CHECK(val_LE == static_cast<int16_t>(vpEndian::swap16bits(static_cast<uint16_t>(val_LE_2_BE))));
    }

    SECTION("uint32_t")
    {
      uint32_t val_LE = 123456;
      uint32_t val_LE_2_BE = vpEndian::swap32bits(val_LE);
      CHECK(val_LE == vpEndian::swap32bits(val_LE_2_BE));
    }
    SECTION("int32_t")
    {
      int32_t val_LE = -123456;
      int32_t val_LE_2_BE = static_cast<int32_t>(vpEndian::swap32bits(static_cast<uint32_t>(val_LE)));
      CHECK(val_LE == static_cast<int32_t>(vpEndian::swap32bits(static_cast<uint32_t>(val_LE_2_BE))));
    }

    SECTION("uint64_t")
    {
      uint64_t val_LE = 12345678900;
      uint64_t val_LE_2_BE = vpEndian::swap64bits(val_LE);
      CHECK(val_LE == vpEndian::swap64bits(val_LE_2_BE));
    }
    SECTION("int64_t")
    {
      int64_t val_LE = -12345678900;
      int64_t val_LE_2_BE = static_cast<int64_t>(vpEndian::swap64bits(static_cast<uint64_t>(val_LE)));
      CHECK(val_LE == static_cast<int64_t>(vpEndian::swap64bits(static_cast<uint64_t>(val_LE_2_BE))));
    }

    SECTION("float")
    {
      float val_LE = 3.14f;
      float val_LE_2_BE = vpEndian::swapFloat(val_LE);
      CHECK(val_LE == Catch::Approx((vpEndian::swapFloat(val_LE_2_BE))).epsilon(std::numeric_limits<float>::epsilon()));
    }
    SECTION("double")
    {
      double val_LE = 3.14;
      double val_LE_2_BE = vpEndian::swapDouble(val_LE);
      CHECK(val_LE == Catch::Approx((vpEndian::swapDouble(val_LE_2_BE))).epsilon(std::numeric_limits<double>::epsilon()));
    }
  }
}

TEST_CASE("Test endianness detection", "[vpEndian_test]")
{
  bool is_big_endian = vpEndian::isBigEndian();
#ifdef VISP_BIG_ENDIAN
  CHECK(is_big_endian);
#else
  CHECK_FALSE(is_big_endian);
#endif
}

int main(int argc, char *argv[])
{
#if defined(VISP_LITTLE_ENDIAN)
  std::cout << "Detected endianess: LE" << std::endl;
#elif defined(VISP_BIG_ENDIAN)
  std::cout << "Detected endianess: BE" << std::endl;
#elif defined(VISP_PDB_ENDIAN)
  std::cout << "Detected endianess: PDB" << std::endl;
#else
  std::cout << "Detected endianess: unknown" << std::endl;
#endif

  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  return numFailed;
}

#else
int main() { return EXIT_SUCCESS; }
#endif
