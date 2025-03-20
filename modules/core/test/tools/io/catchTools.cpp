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
 * Test visp::cnpy::npz_load() / visp::cnpy::npy_save() functions.
 */

/*!
  \example catchTools.cpp
 */

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#include <catch_amalgamated.hpp>

#include <visp3/core/vpIoTools.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

std::string toString(const std::string &input, unsigned int val)
{
  char input_[FILENAME_MAX];
  snprintf(input_, FILENAME_MAX, input.c_str(), val);
  return std::string(input_);
}

SCENARIO("Test vpIoTools::formatString()", "[toString]")
{
  GIVEN("int values")
  {
    std::string input("test-%04d.png");
    THEN("1 digit")
    {
      unsigned int val = 1;
      CHECK(vpIoTools::formatString(input, val) == toString(input, val));
    }
    THEN("2 digits")
    {
      unsigned int val = 23;
      CHECK(vpIoTools::formatString(input, val) == toString(input, val));
    }
  }
};

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  return numFailed;
}

#else
int main() { return EXIT_SUCCESS; }
#endif
