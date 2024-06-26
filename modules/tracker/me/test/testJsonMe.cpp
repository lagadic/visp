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
  \file testJsonMe.cpp

  Test test saving and parsing JSON configuration for vpCameraParameters
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)

#include <random>
#include <visp3/core/vpIoTools.h>
#include <visp3/me/vpMe.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json; //! json namespace shortcut

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

template <typename T, typename C> void checkProperties(const T &t1, const T &t2, C fn, const std::string &message)
{
  THEN(message) { REQUIRE((t1.*fn)() == (t2.*fn)()); }
}

template <typename T, typename C, typename... Fns>
void checkProperties(const T &t1, const T &t2, C fn, const std::string &message, Fns... fns)
{
  checkProperties(t1, t2, fn, message);
  checkProperties(t1, t2, fns...);
}

template <typename C>
void testOptionalProperty(json &j, const std::vector<std::string> &keys, vpMe &me,
  std::function<void(vpMe *, C)> setter, std::function<C(vpMe *)> getter,
  std::function<C(C)> valueFn)
{
  THEN("Removing keys does not modify the value")
  {
    const C v = valueFn(getter(&me));
    setter(&me, v);
    for (const std::string &k : keys) {
      if (!j.contains(k)) {
        FAIL();
      }
      j.erase(k);
    }
    from_json(j, me);
    REQUIRE(getter(&me) == v);
  }
}

namespace
{
class RandomMeGenerator : public Catch::Generators::IGenerator<vpMe>
{
private:
  std::minstd_rand m_rand;
  std::uniform_real_distribution<> m_dist;
  std::uniform_int_distribution<> m_int_dist;

  vpMe current;

public:
  RandomMeGenerator() : m_rand(std::random_device {}()), m_dist(0.0, 1.0), m_int_dist(1, 10)
  {
    static_cast<void>(next());
  }

  vpMe const &get() const VP_OVERRIDE { return current; }
  bool next() VP_OVERRIDE
  {
    current.setThreshold(m_dist(m_rand) * 255);
    current.setMaskNumber(m_int_dist(m_rand) * 10);
    current.setMaskSign(m_int_dist(m_rand) > 5 ? 1 : 0);
    current.setMu1(m_dist(m_rand));
    current.setMu2(current.getMu1() + m_dist(m_rand));
    current.setNbTotalSample(m_int_dist(m_rand) * 2);
    current.setPointsToTrack(m_int_dist(m_rand));
    current.setRange(m_int_dist(m_rand));
    current.setStrip(m_int_dist(m_rand));
    return true;
  }
};
Catch::Generators::GeneratorWrapper<vpMe> randomMe()
{
  return Catch::Generators::GeneratorWrapper<vpMe>(
    std::unique_ptr<Catch::Generators::IGenerator<vpMe> >(new RandomMeGenerator()));
}
} // namespace

SCENARIO("Serializing and deserializing a single vpMe", "[json]")
{
  GIVEN("Some random vpMe object")
  {
    vpMe me = GENERATE(take(10, randomMe()));
    WHEN("Serializing and deserializing an object")
    {
      const json j = me;
      const vpMe otherMe = j;
      THEN("The object's properties are the same")
      {
        checkProperties(me, otherMe, &vpMe::getThreshold, "Threshold should be equal", &vpMe::getAngleStep,
          "Angle step should be equal", &vpMe::getMaskNumber, "Mask number should be equal",
          &vpMe::getMaskSign, "Mask sign should be equal", &vpMe::getMinSampleStep,
          "Min sample step should be equal", &vpMe::getSampleStep, "Sample step should be equal",
          &vpMe::getMu1, "Mu 1 should be equal", &vpMe::getMu2, "Mu 2 should be equal",
          &vpMe::getNbTotalSample, "Nb total sample should be equal", &vpMe::getPointsToTrack,
          "Number of points to track should be equal", &vpMe::getRange, "Range should be equal",
          &vpMe::getStrip, "Strip should be equal");
      }
    }
    WHEN("Removing optional properties in JSON object")
    {
      json j = me;

      const auto testInt = [&j, &me](const std::string &key, std::function<void(vpMe *, int)> setter,
        std::function<int(vpMe *)> getter) -> void {
          testOptionalProperty<int>(j, { key }, me, setter, getter, [](int v) -> int { return v - 1; });
        };
      const auto testDouble = [&j, &me](const std::string &key, std::function<void(vpMe *, double)> setter,
        std::function<double(vpMe *)> getter) -> void {
          testOptionalProperty<double>(j, { key }, me, setter, getter, [](double v) -> double { return v + 1.0; });
        };

      WHEN("Removing threshold") { testDouble("threshold", &vpMe::setThreshold, &vpMe::getThreshold); }
      WHEN("Removing mu1 and mu2")
      {
        testDouble("mu", &vpMe::setMu1, &vpMe::getMu1);
        testDouble("mu", &vpMe::setMu2, &vpMe::getMu2);
      }
      WHEN("Removing nMask") { testInt("nMask", &vpMe::setMaskNumber, &vpMe::getMaskNumber); }
      WHEN("Removing maskSize") { testInt("maskSize", &vpMe::setMaskSize, &vpMe::getMaskSize); }
      WHEN("Removing minSampleStep") { testDouble("minSampleStep", &vpMe::setMinSampleStep, &vpMe::getMinSampleStep); }
      WHEN("Removing sampleStep") { testDouble("sampleStep", &vpMe::setSampleStep, &vpMe::getSampleStep); }
      WHEN("Removing maskSign") { testInt("maskSign", &vpMe::setMaskSign, &vpMe::getMaskSign); }
      WHEN("Removing ntotalSample") { testInt("ntotalSample", &vpMe::setNbTotalSample, &vpMe::getNbTotalSample); }
      WHEN("Removing pointsToTrack") { testInt("pointsToTrack", &vpMe::setPointsToTrack, &vpMe::getPointsToTrack); }
      WHEN("Removing range") { testInt("range", &vpMe::setRange, &vpMe::getRange); }
      WHEN("Removing strip") { testInt("strip", &vpMe::setStrip, &vpMe::getStrip); }
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
