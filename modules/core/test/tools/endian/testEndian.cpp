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
  \example testEndian.cpp
 */
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEndian.h>

#if defined(VISP_HAVE_EIGEN3) && defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

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
