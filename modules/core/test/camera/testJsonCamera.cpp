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
 * Test vpCameraParameters JSON parse / save.
 */

/*!
  \example testJsonCamera.cpp

  Test saving and parsing JSON configuration for vpCameraParameters.
*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpIoTools.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)
#include <nlohmann/json.hpp>
using json = nlohmann::json; //! json namespace shortcut

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#include <random>
namespace
{
// This class shows how to implement a simple generator for Catch tests
class vpRandomCamGenerator : public Catch::Generators::IGenerator<vpCameraParameters>
{
private:
  std::minstd_rand m_rand;
  std::uniform_int_distribution<> m_count_dist;
  std::uniform_int_distribution<> m_type_dist;
  std::uniform_real_distribution<> m_dist;

  vpCameraParameters current;

public:
  vpRandomCamGenerator(double low, double high)
    : m_rand(std::random_device {}()), m_count_dist(1, 5), m_type_dist(0, 2), m_dist(low, high)
  {
    static_cast<void>(next());
  }

  const vpCameraParameters &get() const VP_OVERRIDE { return current; }
  bool next() VP_OVERRIDE
  {
    const double px = m_dist(m_rand);
    const double py = m_dist(m_rand);
    const double u0 = m_dist(m_rand);
    const double v0 = m_dist(m_rand);

    const int type = m_type_dist(m_rand);
    switch (type) {
    case 0: {
      current.initPersProjWithoutDistortion(px, py, u0, v0);
      break;
    }
    case 1: {
      current.initPersProjWithDistortion(px, py, u0, v0, m_dist(m_rand), m_dist(m_rand));
      break;
    }
    case 2: {
      std::vector<double> coeffs;
      const int count = m_count_dist(m_rand);
      for (int i = 0; i < count; ++i) {
        coeffs.push_back(m_dist(m_rand));
      }
      current.initProjWithKannalaBrandtDistortion(px, py, u0, v0, coeffs);
      break;
    }
    default: {
      throw vpException(vpException::badValue, "Shouldn't happen");
    }
    }
    return true;
  }
};

Catch::Generators::GeneratorWrapper<vpCameraParameters> randomCam(double low, double high)
{
  return Catch::Generators::GeneratorWrapper<vpCameraParameters>(
    std::unique_ptr<Catch::Generators::IGenerator<vpCameraParameters> >(new vpRandomCamGenerator(low, high)));
}
} // namespace

SCENARIO("Serializing and deserializing a single vpCameraParameters", "[json]")
{
  GIVEN("Some camera intrinsics")
  {
    vpCameraParameters cam = GENERATE(take(100, randomCam(50.0, 500.0)));
    THEN("Serializing and deserializing does not modify the object")
    {
      const json j = cam;
      const vpCameraParameters otherCam = j;
      REQUIRE(cam == otherCam);
    }
  }
}

SCENARIO("Serializing two cameras", "[json]")
{
  GIVEN("Some camera intrinsics")
  {
    vpCameraParameters cam = GENERATE(take(10, randomCam(50.0, 500.0)));
    vpCameraParameters cam2 = GENERATE(take(10, randomCam(50.0, 500.0)));
    if (cam != cam2) {
      WHEN("serializing and deserializing two different cameras")
      {
        THEN("The deserialized cams are still different")
        {
          const json j1 = cam;
          const json j2 = cam2;

          const vpCameraParameters resCam = j1;
          const vpCameraParameters resCam2 = j2;
          REQUIRE(resCam != resCam2);
        }
      }
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

int main() { return EXIT_SUCCESS; }

#endif
