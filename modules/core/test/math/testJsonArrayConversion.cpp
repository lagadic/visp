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
 * Test vpArray2D and children JSON parse / save.
 */

/*!
  \file testJsonArrayConversion.cpp

  Test test saving and parsing JSON configuration for vpArray2D and children classes.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)

#include <random>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpIoTools.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json; //! json namespace shortcut

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
using StringMatcherBase = Catch::Matchers::StdString::StringMatcherBase;
class vpExceptionMatcher : public Catch::Matchers::Impl::MatcherBase<vpException>
{
  vpException::generalExceptionEnum m_type;
  const StringMatcherBase &m_messageMatcher;

public:
  vpExceptionMatcher(vpException::generalExceptionEnum type, const StringMatcherBase &messageMatcher)
    : m_type(type), m_messageMatcher(messageMatcher)
  { }

  bool match(vpException const &in) const VP_OVERRIDE
  {
    return m_type == in.getCode() && m_messageMatcher.match(in.getStringMessage());
  }

  std::string describe() const VP_OVERRIDE
  {
    std::ostringstream ss;
    ss << "vpException has type " << m_type << " and message " << m_messageMatcher.describe();
    return ss.str();
  }
};

class vpRandomArray2DGenerator : public Catch::Generators::IGenerator<vpArray2D<double> >
{
private:
  std::minstd_rand m_rand;
  std::uniform_real_distribution<> m_val_dist;
  std::uniform_int_distribution<> m_dim_dist;

  vpArray2D<double> current;

public:
  vpRandomArray2DGenerator(double valueRange, int minSize, int maxSize)
    : m_rand(std::random_device {}()), m_val_dist(-valueRange, valueRange), m_dim_dist(minSize, maxSize)

  {
    static_cast<void>(next());
  }

  vpArray2D<double> const &get() const VP_OVERRIDE { return current; }
  bool next() VP_OVERRIDE
  {
    const unsigned nCols = m_dim_dist(m_rand);
    const unsigned nRows = m_dim_dist(m_rand);
    current.resize(nRows, nCols);
    for (unsigned i = 0; i < nRows; ++i) {
      for (unsigned j = 0; j < nCols; ++j) {
        current[i][j] = m_val_dist(m_rand);
      }
    }
    return true;
  }
};
class vpRandomColVectorGenerator : public Catch::Generators::IGenerator<vpColVector>
{
private:
  std::minstd_rand m_rand;
  std::uniform_real_distribution<> m_val_dist;
  std::uniform_int_distribution<> m_dim_dist;

  vpColVector current;

public:
  vpRandomColVectorGenerator(double valueRange, int minSize, int maxSize)
    : m_rand(std::random_device {}()), m_val_dist(-valueRange, valueRange), m_dim_dist(minSize, maxSize)

  {
    static_cast<void>(next());
  }

  const vpColVector &get() const VP_OVERRIDE { return current; }
  bool next() VP_OVERRIDE
  {
    const unsigned nRows = m_dim_dist(m_rand);
    current.resize(nRows);
    for (unsigned i = 0; i < nRows; ++i) {
      current[i] = m_val_dist(m_rand);
    }
    return true;
  }
};
Catch::Generators::GeneratorWrapper<vpArray2D<double> > randomArray(double v, int minSize, int maxSize)
{
  return Catch::Generators::GeneratorWrapper<vpArray2D<double> >(
    std::unique_ptr<Catch::Generators::IGenerator<vpArray2D<double> > >(
      new vpRandomArray2DGenerator(v, minSize, maxSize)));
}
Catch::Generators::GeneratorWrapper<vpColVector> randomColVector(double v, int minSize, int maxSize)
{
  return Catch::Generators::GeneratorWrapper<vpColVector>(
    std::unique_ptr<Catch::Generators::IGenerator<vpColVector> >(new vpRandomColVectorGenerator(v, minSize, maxSize)));
}
vpExceptionMatcher matchVpException(vpException::generalExceptionEnum type, const StringMatcherBase &matcher)
{
  return vpExceptionMatcher(type, matcher);
}
} // namespace
SCENARIO("Serializing a vpArray2D", "[json]")
{
  GIVEN("A random vpArray2D<double>")
  {
    const vpArray2D<double> array = GENERATE(take(10, randomArray(50.0, 1, 10)));
    WHEN("Serializing to a JSON object")
    {
      const json j = array;
      THEN("JSON object is a dictionary") { REQUIRE(j.is_object()); }
      THEN("JSON object contains correct number of columns")
      {
        REQUIRE(j.at("cols").get<unsigned int>() == array.getCols());
      }
      THEN("JSON object contains correct number of rows")
      {
        REQUIRE(j.at("rows").get<unsigned int>() == array.getRows());
      }
      THEN("JSON object contains the array values")
      {
        const json jData = j.at("data");
        THEN("The data field is an array") { REQUIRE(jData.is_array()); }
        THEN("The data field contains the correct number of values") { REQUIRE(jData.size() == array.size()); }
        THEN("The data field contains the correct number of values")
        {
          const double *const start = array[0];
          const std::vector<double> vec(start, start + array.size());
          REQUIRE(vec == jData.get<std::vector<double> >());
        }
      }
    }
  }
}

SCENARIO("Trying to instantiate a vpArray with a wrong type of object", "[json]")
{
  GIVEN("A random scalar converted to a JSON representation")
  {
    vpArray2D<double> array;
    std::minstd_rand rand;
    std::uniform_real_distribution<> dist;
    const json j = GENERATE(take(50, random(-200.0, 200.0)));
    THEN("An exception is thrown")
    {
      const auto matcher = Catch::Matchers::Contains("is not an array or object");
      REQUIRE_THROWS_MATCHES(from_json(j, array), vpException, matchVpException(vpException::badValue, matcher));
    }
  }
}

SCENARIO("Recovering a vpArray2D from a JSON array", "[json]")
{
  GIVEN("An empty array")
  {
    const json j = json::array_t();
    WHEN("Converting to a vpArray2D")
    {
      vpArray2D<double> array = j;
      THEN("The resulting array is empty") { REQUIRE(array.size() == 0); }
    }
  }
  GIVEN("A 1D array")
  {
    const json j = { 10.0, 20.0, 30.0 };
    WHEN("Converting to a vpArray2D")
    {
      THEN("An exception is thrown, since this is an ambiguous array")
      {
        vpArray2D<double> array;
        const auto matcher = Catch::Matchers::Contains("is not an array of array");
        REQUIRE_THROWS_MATCHES(from_json(j, array), vpException, matchVpException(vpException::badValue, matcher));
      }
    }
  }
  GIVEN("A vpArray2D converted to a json 2D array")
  {
    vpArray2D<double> array = GENERATE(take(10, randomArray(50.0, 2, 10)));
    json j;
    for (unsigned i = 0; i < array.getRows(); ++i) {
      json jRow;
      for (unsigned j = 0; j < array.getCols(); ++j) {
        jRow.push_back(array[i][j]);
      }
      j.push_back(jRow);
    }
    WHEN("Converting back to a vpArray2D")
    {
      vpArray2D<double> array2 = j;
      THEN("The values are correct") { REQUIRE(array == array2); }
    }
    WHEN("Removing elements from rows so that they do not have the same size")
    {
      j[0].erase(0); // Generating at least a 2x2 array
      THEN("An exception is thrown")
      {
        const auto matcher = Catch::Matchers::Contains("row arrays that are not of the same size");
        REQUIRE_THROWS_MATCHES(from_json(j, array), vpException, matchVpException(vpException::badValue, matcher));
      }
    }
  }
}

SCENARIO("Recovering a vpArray2D from a JSON object as serialized by ViSP", "[json]")
{
  GIVEN("A vpArray2D converted to JSON format")
  {
    const vpArray2D<double> array = GENERATE(take(10, randomArray(50.0, 1, 10)));
    json j = array;
    WHEN("Converting back to a vpArray2D")
    {
      const vpArray2D<double> array2 = j;
      THEN("The 2 arrays are equal") { REQUIRE(array == array2); }
    }
    WHEN("Removing or adding some values from the data field so that its size does not match the expected array size")
    {
      j.at("data").erase(0);
      THEN("An exception is thrown")
      {
        vpArray2D<double> array2;
        const auto matcher = Catch::Matchers::Contains("must be an array of size");
        REQUIRE_THROWS_MATCHES(from_json(j, array2), vpException, matchVpException(vpException::badValue, matcher));
      }
    }
  }
}

SCENARIO("Serializing and deserializing a vpColVector", "[json]")
{
  GIVEN("A random vpColVector")
  {
    const vpColVector v = GENERATE(take(100, randomColVector(100.0, 1, 50)));
    WHEN("Serializing to JSON")
    {
      const json j = v;
      THEN("There is only one column") { REQUIRE(j.at("cols") == 1); }
      THEN("The type is vpColVector") { REQUIRE(j.at("type") == "vpColVector"); }
      WHEN("Deserializing back to a vpColVector")
      {
        const vpColVector v2 = j;
        THEN("The 2 vectors are the same") { REQUIRE(v == v2); }
      }
    }
  }
  GIVEN("A random 2D array with number of cols > 1")
  {
    const vpArray2D<double> array = GENERATE(take(10, randomArray(50.0, 2, 5)));
    WHEN("Serializing this array to JSON")
    {
      const json j = array;
      THEN("Serializing back to a vpColVector throws an exception")
      {
        vpColVector v;
        const auto matcher = Catch::Matchers::Contains("tried to read a 2D array into a vpColVector");
        REQUIRE_THROWS_MATCHES(from_json(j, v), vpException, matchVpException(vpException::badValue, matcher));
      }
    }
  }
  GIVEN("A random 2D array with number of cols = 1")
  {
    const vpColVector v = GENERATE(take(10, randomColVector(100.0, 1, 50)));
    const vpArray2D<double> array = (vpArray2D<double>)v;
    WHEN("Serializing this array to JSON")
    {
      const json j = array;
      THEN("Serializing back to a vpColVector is ok and gives the same vector")
      {
        const vpColVector v2 = j;
        REQUIRE(v == v2);
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
