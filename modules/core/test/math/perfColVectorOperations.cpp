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
 * Benchmark column vector operations.
 */

/*!
  \example perfColVectorOperations.cpp
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpColVector.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
static bool g_runBenchmark = false;
static const std::vector<unsigned int> g_sizes = { 23, 127, 233, 419, 1153, 2749 };

double getRandomValues(double min, double max) { return (max - min) * ((double)rand() / (double)RAND_MAX) + min; }

vpColVector generateRandomVector(unsigned int rows, double min = -100, double max = 100)
{
  vpColVector v(rows);

  for (unsigned int i = 0; i < v.getRows(); i++) {
    v[i] = getRandomValues(min, max);
  }

  return v;
}

double stddev(const std::vector<double> &vec)
{
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  double mean = sum / vec.size();

  std::vector<double> diff(vec.size());
  std::transform(vec.begin(), vec.end(), diff.begin(), [mean](double x) {
    return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / vec.size());
}

double computeRegularSum(const vpColVector &v)
{
  double sum = 0.0;

  for (unsigned int i = 0; i < v.getRows(); i++) {
    sum += v[i];
  }

  return sum;
}

double computeRegularSumSquare(const vpColVector &v)
{
  double sum_square = 0.0;

  for (unsigned int i = 0; i < v.getRows(); i++) {
    sum_square += v[i] * v[i];
  }

  return sum_square;
}

double computeRegularStdev(const vpColVector &v)
{
  double mean_value = computeRegularSum(v) / v.getRows();
  double sum_squared_diff = 0.0;

  for (unsigned int i = 0; i < v.size(); i++) {
    sum_squared_diff += (v[i] - mean_value) * (v[i] - mean_value);
  }

  double divisor = (double)v.size();

  return std::sqrt(sum_squared_diff / divisor);
}

std::vector<double> computeHadamard(const std::vector<double> &v1, const std::vector<double> &v2)
{
  std::vector<double> result;
  std::transform(v1.begin(), v1.end(), v2.begin(), std::back_inserter(result), std::multiplies<double>());
  return result;
}

bool almostEqual(const vpColVector &v1, const vpColVector &v2, double tol = 1e-9)
{
  if (v1.getRows() != v2.getRows()) {
    return false;
  }

  for (unsigned int i = 0; i < v1.getRows(); i++) {
    if (!vpMath::equal(v1[i], v2[i], tol)) {
      return false;
    }
  }

  return true;
}
} // namespace

TEST_CASE("Benchmark vpColVector::sum()", "[benchmark]")
{
  // Sanity checks
  {
    const double val = 11;
    vpColVector v(1, val);
    CHECK(v.sum() == Approx(val).epsilon(std::numeric_limits<double>::epsilon()));
  }
  {
    const unsigned int size = 11;
    std::vector<double> vec(size);
    vpColVector v(size);
    for (size_t i = 0; i < 11; i++) {
      vec[i] = 2. * i;
      v[static_cast<unsigned int>(i)] = vec[i];
    }
    CHECK(v.sum() ==
          Approx(std::accumulate(vec.begin(), vec.end(), 0.0)).epsilon(std::numeric_limits<double>::epsilon()));
  }

  if (g_runBenchmark) {
    for (auto size : g_sizes) {
      vpColVector v = generateRandomVector(size);
      std::vector<double> vec = v.toStdVector();

      std::ostringstream oss;
      oss << "Benchmark vpColVector::sum() with size: " << size << " (ViSP)";
      double vp_sum = 0;
      BENCHMARK(oss.str().c_str())
      {
        vp_sum = v.sum();
        return vp_sum;
      };

      oss.str("");
      oss << "Benchmark std::accumulate() with size: " << size << " (C++)";
      double std_sum = 0;
      BENCHMARK(oss.str().c_str())
      {
        std_sum = std::accumulate(vec.begin(), vec.end(), 0.0);
        return std_sum;
      };
      CHECK(vp_sum == Approx(std_sum));

      oss.str("");
      oss << "Benchmark naive sum() with size: " << size;
      double naive_sum = 0;
      BENCHMARK(oss.str().c_str())
      {
        naive_sum = computeRegularSum(v);
        return naive_sum;
      };
      CHECK(naive_sum == Approx(std_sum));
    }
  }
}

TEST_CASE("Benchmark vpColVector::sumSquare()", "[benchmark]")
{
  // Sanity checks
  {
    const double val = 11;
    vpColVector v(1, val);
    CHECK(v.sumSquare() == Approx(val * val).epsilon(std::numeric_limits<double>::epsilon()));
  }
  {
    const unsigned int size = 11;
    std::vector<double> vec(size);
    vpColVector v(size);
    for (size_t i = 0; i < 11; i++) {
      vec[i] = 2. * i;
      v[static_cast<unsigned int>(i)] = vec[i];
    }
    CHECK(v.sumSquare() == Approx(std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0))
                               .epsilon(std::numeric_limits<double>::epsilon()));
  }

  if (g_runBenchmark) {
    for (auto size : g_sizes) {
      vpColVector v = generateRandomVector(size);
      std::vector<double> vec = v.toStdVector();

      std::ostringstream oss;
      oss << "Benchmark vpColVector::sumSquare() with size: " << size << " (ViSP)";
      double vp_sq_sum = 0;
      BENCHMARK(oss.str().c_str())
      {
        vp_sq_sum = v.sumSquare();
        return vp_sq_sum;
      };

      oss.str("");
      oss << "Benchmark std::inner_product with size: " << size << " (C++)";
      double std_sq_sum = 0;
      BENCHMARK(oss.str().c_str())
      {
        std_sq_sum = std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0);
        return std_sq_sum;
      };
      CHECK(vp_sq_sum == Approx(std_sq_sum));

      oss.str("");
      oss << "Benchmark naive sumSquare() with size: " << size;
      double naive_sq_sum = 0;
      BENCHMARK(oss.str().c_str())
      {
        naive_sq_sum = computeRegularSumSquare(v);
        return naive_sq_sum;
      };
      CHECK(naive_sq_sum == Approx(std_sq_sum));
    }
  }
}

TEST_CASE("Benchmark vpColVector::stdev()", "[benchmark]")
{
  // Sanity checks
  {
    vpColVector v(2);
    v[0] = 11;
    v[1] = 16;
    std::vector<double> vec = v.toStdVector();
    CHECK(vpColVector::stdev(v) == Approx(stddev(vec)).epsilon(std::numeric_limits<double>::epsilon()));
  }
  {
    const unsigned int size = 11;
    std::vector<double> vec(size);
    vpColVector v(size);
    for (size_t i = 0; i < 11; i++) {
      vec[i] = 2. * i;
      v[static_cast<unsigned int>(i)] = vec[i];
    }
    CHECK(vpColVector::stdev(v) == Approx(stddev(vec)).epsilon(std::numeric_limits<double>::epsilon()));
  }

  if (g_runBenchmark) {
    for (auto size : g_sizes) {
      vpColVector v = generateRandomVector(size);
      std::vector<double> vec = v.toStdVector();

      std::ostringstream oss;
      oss << "Benchmark vpColVector::stdev() with size: " << size << " (ViSP)";
      double vp_stddev = 0;
      BENCHMARK(oss.str().c_str())
      {
        vp_stddev = vpColVector::stdev(v);
        return vp_stddev;
      };

      oss.str("");
      oss << "Benchmark C++ stddev impl with size: " << size << " (C++)";
      double std_stddev = 0;
      BENCHMARK(oss.str().c_str())
      {
        std_stddev = stddev(vec);
        return std_stddev;
      };
      CHECK(vp_stddev == Approx(std_stddev));

      oss.str("");
      oss << "Benchmark naive stdev() with size: " << size;
      double naive_stddev = 0;
      BENCHMARK(oss.str().c_str())
      {
        naive_stddev = computeRegularStdev(v);
        return naive_stddev;
      };
      CHECK(naive_stddev == Approx(std_stddev));
    }
  }
}

TEST_CASE("Benchmark vpColVector::hadamard()", "[benchmark]")
{
  // Sanity checks
  {
    vpColVector v1(2), v2(2);
    v1[0] = 11;
    v1[1] = 16;
    v2[0] = 8;
    v2[1] = 23;
    vpColVector res1 = v1.hadamard(v2);
    vpColVector res2(computeHadamard(v1.toStdVector(), v2.toStdVector()));
    CHECK(almostEqual(res1, res2));
  }
  {
    const unsigned int size = 11;
    std::vector<double> vec1(size), vec2(size);
    for (size_t i = 0; i < 11; i++) {
      vec1[i] = 2. * i;
      vec2[i] = 3. * i + 5.;
    }
    vpColVector v1(vec1), v2(vec2);
    vpColVector res1 = v1.hadamard(v2);
    vpColVector res2(computeHadamard(v1.toStdVector(), v2.toStdVector()));
    CHECK(almostEqual(res1, res2));
  }

  if (g_runBenchmark) {
    for (auto size : g_sizes) {
      vpColVector v1 = generateRandomVector(size);
      vpColVector v2 = generateRandomVector(size);
      std::vector<double> vec1 = v1.toStdVector();
      std::vector<double> vec2 = v2.toStdVector();

      std::ostringstream oss;
      oss << "Benchmark vpColVector::hadamard() with size: " << size << " (ViSP)";
      vpColVector vp_res;
      BENCHMARK(oss.str().c_str())
      {
        vp_res = v1.hadamard(v2);
        return vp_res;
      };

      oss.str("");
      oss << "Benchmark C++ element wise multiplication with size: " << size << " (C++)";
      std::vector<double> std_res;
      BENCHMARK(oss.str().c_str())
      {
        std_res = computeHadamard(vec1, vec2);
        return std_res;
      };
      CHECK(almostEqual(vp_res, vpColVector(std_res)));
    }
  }
}

int main(int argc, char *argv[])
{
  // Set random seed explicitly to avoid confusion
  // See: https://en.cppreference.com/w/cpp/numeric/random/srand
  // If rand() is used before any calls to srand(), rand() behaves as if it was seeded with srand(1).
  srand(1);

  Catch::Session session; // There must be exactly one instance

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()         // Get Catch's composite command line parser
    | Opt(g_runBenchmark) // bind variable to a new option, with a hint string
    ["--benchmark"] // the option names it will respond to
    ("run benchmark?");   // description string for the help output

// Now pass the new composite back to Catch so it uses that
  session.cli(cli);

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
