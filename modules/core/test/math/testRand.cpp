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
 * Test pseudo random number generator.
 */

/*!
  \example testRand.cpp
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
class vpUniRandOld
{
  long a;
  long m;            // 2^31-1
  long q;            // integer part of m/a
  long r;            // r=m mod a
  double normalizer; // we use a normalizer > m to ensure ans will never be 1
                     // (it is the case if x = 739806647)

private:
  inline void draw0()
  {
    long k = x / q; // temp value for computing without overflow
    x = a * (x - k * q) - k * r;
    if (x < 0)
      x += m; // compute x without overflow
  }

protected:
  long x;
  double draw1()
  {
    const long ntab = 33; // we work on a 32 elements array.
                          // the 33rd one is actually the first value of y.
    const long modulo = ntab - 2;

    static long y = 0;
    static long T[ntab];

    long j; // index of T

    // step 0
    if (!y) { // first time
      for (j = 0; j < ntab; j++) {
        draw0();
        T[j] = x;
      } // compute table T
      y = T[ntab - 1];
    }

    // step 1
    j = y & modulo; // compute modulo ntab+1 (the first element is the 0th)

    // step 3
    y = T[j];
    double ans = (double)y / normalizer;

    // step 4
    // generate x(k+i) and set y=x(k+i)
    draw0();

    // refresh T[j];
    T[j] = x;

    return ans;
  }

public:
  //! Default constructor.
  VP_EXPLICIT vpUniRandOld(const long seed = 0)
    : a(16807), m(2147483647), q(127773), r(2836), normalizer(2147484721.0), x((seed) ? seed : 739806647)
  { }

  //! Default destructor.
  virtual ~vpUniRandOld() { };

  //! Operator that allows to get a random value.
  double operator()() { return draw1(); }
};
} // namespace

TEST_CASE("Check Gaussian draw", "[visp_rand]")
{
  std::vector<double> vec(100000);
  const double sigma = 5.0, mean = -7.5;
  vpGaussRand rng(sigma, mean);

  vpChrono chrono;
  chrono.start();
  for (size_t i = 0; i < vec.size(); i++) {
    vec[i] = rng();
  }
  chrono.stop();

  std::cout << vec.size() << " Gaussian draw performed in " << chrono.getDurationMs() << " ms" << std::endl;
  double calculated_sigma = vpMath::getStdev(vec);
  double calculated_mean = vpMath::getMean(vec);
  std::cout << "Calculated sigma: " << calculated_sigma << std::endl;
  std::cout << "Calculated mean: " << calculated_mean << std::endl;

  CHECK(calculated_sigma == Approx(sigma).epsilon(0.01));
  CHECK(calculated_mean == Approx(mean).epsilon(0.01));
}

TEST_CASE("Check Gaussian draw independance", "[visp_rand]")
{
  const double sigma = 5.0, mean = -7.5;

  SECTION("Two simultaneous vpGaussRand instances with the same seed should produce the same results")
  {
    vpGaussRand rng1(sigma, mean), rng2(sigma, mean);

    for (int i = 0; i < 10; i++) {
      CHECK(rng1() == Approx(rng2()).margin(1e-6));
    }
  }
  SECTION("Two vpGaussRand instances with the same seed should produce the same results")
  {
    std::vector<double> vec1, vec2;
    {
      vpGaussRand rng(sigma, mean);
      for (int i = 0; i < 10; i++) {
        vec1.push_back(rng());
      }
    }
    {
      vpGaussRand rng(sigma, mean);
      for (int i = 0; i < 10; i++) {
        vec2.push_back(rng());
      }
    }
    REQUIRE(vec1.size() == vec2.size());

    for (size_t i = 0; i < vec1.size(); i++) {
      CHECK(vec1[i] == Approx(vec2[i]).margin(1e-6));
    }
  }
}

TEST_CASE("Check uniform draw", "[visp_rand]")
{
  const int niters = 500000;

  SECTION("vpUniRand")
  {
    vpUniRand rng;
    int inside = 0;

    vpChrono chrono;
    chrono.start();
    for (int i = 0; i < niters; i++) {
      double x = rng();
      double y = rng();

      if (sqrt(x * x + y * y) <= 1.0) {
        inside++;
      }
    }
    double pi = 4.0 * inside / niters;
    chrono.stop();

    double pi_error = pi - M_PI;
    std::cout << "vpUniRand calculated pi: " << pi << " in " << chrono.getDurationMs() << " ms" << std::endl;
    std::cout << "pi error: " << pi_error << std::endl;

    CHECK(pi == Approx(M_PI).margin(0.005));
  }

  SECTION("C++ rand()")
  {
    srand(0);
    int inside = 0;

    vpChrono chrono;
    chrono.start();
    for (int i = 0; i < niters; i++) {
      double x = static_cast<double>(rand()) / RAND_MAX;
      double y = static_cast<double>(rand()) / RAND_MAX;

      if (sqrt(x * x + y * y) <= 1.0) {
        inside++;
      }
    }
    double pi = 4.0 * inside / niters;
    chrono.stop();

    double pi_error = pi - M_PI;
    std::cout << "C++ rand() calculated pi: " << pi << " in " << chrono.getDurationMs() << " ms" << std::endl;
    std::cout << "pi error: " << pi_error << std::endl;

    CHECK(pi == Approx(M_PI).margin(0.01));
  }

  SECTION("Old ViSP vpUniRand implementation")
  {
    vpUniRand rng;
    int inside = 0;

    vpChrono chrono;
    chrono.start();
    for (int i = 0; i < niters; i++) {
      double x = rng();
      double y = rng();

      if (sqrt(x * x + y * y) <= 1.0) {
        inside++;
      }
    }
    double pi = 4.0 * inside / niters;
    chrono.stop();

    double pi_error = pi - M_PI;
    std::cout << "Old ViSP vpUniRand implementation calculated pi: " << pi << " in " << chrono.getDurationMs() << " ms"
      << std::endl;
    std::cout << "pi error: " << pi_error << std::endl;

    CHECK(pi == Approx(M_PI).margin(0.005));
  }
}

TEST_CASE("Check uniform draw range", "[visp_rand]")
{
  const int niters = 1000;
  vpUniRand rng;

  SECTION("Check[0.0, 1.0) range")
  {
    for (int i = 0; i < niters; i++) {
      double r = rng();
      CHECK(r >= 0.0);
      CHECK(r < 1.0);
    }
  }

  SECTION("Check[-7, 10) range")
  {
    const int a = -7, b = 10;
    for (int i = 0; i < niters; i++) {
      int r = rng.uniform(a, b);
      CHECK(r >= a);
      CHECK(r < b);
    }
  }

  SECTION("Check[-4.5f, 105.3f) range")
  {
    const float a = -4.5f, b = 105.3f;
    for (int i = 0; i < niters; i++) {
      float r = rng.uniform(a, b);
      CHECK(r >= a);
      CHECK(r < b);
    }
  }

  SECTION("Check[14.6, 56.78) range")
  {
    const double a = 14.6, b = 56.78;
    for (int i = 0; i < niters; i++) {
      double r = rng.uniform(a, b);
      CHECK(r >= a);
      CHECK(r < b);
    }
  }
}

TEST_CASE("Check uniform draw independance", "[visp_rand]")
{
  SECTION("Two simultaneous vpUniRand instances with the same seed should produce the same results")
  {
    {
      vpUniRand rng1, rng2;

      for (int i = 0; i < 10; i++) {
        CHECK(rng1.next() == rng2.next());
      }
    }
    {
      vpUniRand rng1, rng2;

      for (int i = 0; i < 10; i++) {
        CHECK(rng1.uniform(-1.0, 1.0) == Approx(rng2.uniform(-1.0, 1.0)).margin(1e-6));
      }
    }
  }
  SECTION("Two vpUniRand instances with the same seed should produce the same results")
  {
    {
      std::vector<uint32_t> vec1, vec2;
      {
        vpUniRand rng;
        for (int i = 0; i < 10; i++) {
          vec1.push_back(rng.next());
        }
      }
      {
        vpUniRand rng;
        for (int i = 0; i < 10; i++) {
          vec2.push_back(rng.next());
        }
      }
      REQUIRE(vec1.size() == vec2.size());

      for (size_t i = 0; i < vec1.size(); i++) {
        CHECK(vec1[i] == vec2[i]);
      }
    }
    {
      std::vector<double> vec1, vec2;
      {
        vpUniRand rng;
        for (int i = 0; i < 10; i++) {
          vec1.push_back(rng.uniform(-1.0, 2.0));
        }
      }
      {
        vpUniRand rng;
        for (int i = 0; i < 10; i++) {
          vec2.push_back(rng.uniform(-1.0, 2.0));
        }
      }
      REQUIRE(vec1.size() == vec2.size());

      for (size_t i = 0; i < vec1.size(); i++) {
        CHECK(vec1[i] == Approx(vec2[i]).margin(1e-6));
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
