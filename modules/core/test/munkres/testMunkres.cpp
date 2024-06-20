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
 * Test Munkres assignment algorithm.
 */
/*!
  \example testMunkres.cpp

  \brief Test Munkres assignment algorithm.
*/

#include <visp3/core/vpMunkres.h>

// Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

// System
#include <iostream>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace std
{

// Helper to output a Munkres pair
ostream &operator<<(ostream &os, const pair<unsigned int, unsigned int> &val)
{
  os << "[" << val.first << "," << val.second << "]";
  return os;
}
} // namespace std
#endif // DOXYGEN_SHOULD_SKIP_THIS

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Check Munkres-based assignment", "[visp_munkres]")
{
  auto testMunkres = [](const std::vector<std::vector<double> > &cost_matrix,
                        const std::vector<std::pair<unsigned int, unsigned int> > &expected_pairs) {
                          const auto munkres_pairs = vpMunkres::run(cost_matrix);
                          REQUIRE(expected_pairs.size() == munkres_pairs.size());
                          for (auto i = 0u; i < munkres_pairs.size(); i++) {
                            REQUIRE(expected_pairs.at(i) == munkres_pairs.at(i));
                          }
    };

  SECTION("Square cost matrix")
  {
    std::vector<std::vector<double> > costs {};
    costs.push_back(std::vector<double>{3, 1, 2});
    costs.push_back(std::vector<double>{2, 3, 1});
    costs.push_back(std::vector<double>{1, 2, 3});

    std::vector<std::pair<unsigned int, unsigned int> > expected_pairs {};
    expected_pairs.emplace_back(0, 1);
    expected_pairs.emplace_back(1, 2);
    expected_pairs.emplace_back(2, 0);

    testMunkres(costs, expected_pairs);
  }

  SECTION("Horizontal cost matrix")
  {
    std::vector<std::vector<double> > costs {};
    costs.push_back(std::vector<double>{4, 1, 2, 3});
    costs.push_back(std::vector<double>{3, 4, 1, 2});
    costs.push_back(std::vector<double>{2, 3, 4, 1});

    std::vector<std::pair<unsigned int, unsigned int> > expected_pairs {};
    expected_pairs.emplace_back(0, 1);
    expected_pairs.emplace_back(1, 2);
    expected_pairs.emplace_back(2, 3);

    testMunkres(costs, expected_pairs);
  }

  SECTION("Vertical cost matrix")
  {
    std::vector<std::vector<double> > costs {};
    costs.push_back(std::vector<double>{4, 1, 2});
    costs.push_back(std::vector<double>{3, 4, 1});
    costs.push_back(std::vector<double>{2, 3, 4});
    costs.push_back(std::vector<double>{1, 2, 3});

    std::vector<std::pair<unsigned int, unsigned int> > expected_pairs {};
    expected_pairs.emplace_back(0, 1);
    expected_pairs.emplace_back(1, 2);
    expected_pairs.emplace_back(3, 0);

    testMunkres(costs, expected_pairs);
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);

  return session.run();
}
#else
// Fallback to classic tests

bool testMunkres(const std::vector<std::vector<double> > &costs_matrix,
                 const std::vector<std::pair<unsigned int, unsigned int> > &expected_pairs)
{
  const auto pairs = vpMunkres::run(costs_matrix);

  if (pairs.size() != expected_pairs.size()) {
    // clang-format off
    std::cerr << "Expected nb of association | Munkres nb of association: "
      << expected_pairs.size() << " | " << pairs.size()
      << std::endl;
// clang-format on
    return false;
  }

  for (auto i = 0u; i < pairs.size(); i++) {
    if (expected_pairs.at(i) != pairs.at(i)) {

      // Output the cost matrix
      std::cout << "Cost matrix:" << std::endl;
      for (const auto &cost_row : costs_matrix) {
        std::cout << "| ";
        for (const auto &cost : cost_row) {
          std::cout << cost << " | ";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;

      // Output the pair which fails
      std::cerr << "FAIL: "
        << "Expected association | Munkres association: " << expected_pairs.at(i) << " | " << pairs.at(i)
        << std::endl;

      return false;
    }
  }

  return true;
}

bool testSquareMat()
{
  std::vector<std::vector<double> > costs {};
  costs.push_back(std::vector<double>{3, 4, 1, 2});
  costs.push_back(std::vector<double>{3, 4, 2, 1});
  costs.push_back(std::vector<double>{1, 2, 3, 4});
  costs.push_back(std::vector<double>{2, 1, 4, 3});

  std::vector<std::pair<unsigned int, unsigned int> > pairs {};
  pairs.emplace_back(0, 2);
  pairs.emplace_back(1, 3);
  pairs.emplace_back(2, 0);
  pairs.emplace_back(3, 1);

  return testMunkres(costs, pairs);
}

bool testVertMat()
{
  std::vector<std::vector<double> > costs {};
  costs.push_back(std::vector<double>{3, 2, 1});
  costs.push_back(std::vector<double>{4, 3, 2});
  costs.push_back(std::vector<double>{1, 4, 3});
  costs.push_back(std::vector<double>{2, 1, 4});

  std::vector<std::pair<unsigned int, unsigned int> > pairs {};
  pairs.emplace_back(0, 2);
  pairs.emplace_back(2, 0);
  pairs.emplace_back(3, 1);

  return testMunkres(costs, pairs);
}

bool testHorMat()
{
  std::vector<std::vector<double> > costs {};
  costs.push_back(std::vector<double>{2, 3, 4, 1});
  costs.push_back(std::vector<double>{4, 1, 2, 3});
  costs.push_back(std::vector<double>{1, 2, 3, 4});

  std::vector<std::pair<unsigned int, unsigned int> > pairs {};
  pairs.emplace_back(0, 3);
  pairs.emplace_back(1, 1);
  pairs.emplace_back(2, 0);

  return testMunkres(costs, pairs);
}

int main()
{
  if (not testSquareMat()) {
    return EXIT_FAILURE;
  }

  if (not testVertMat()) {
    return EXIT_FAILURE;
  }

  if (not testHorMat()) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
#endif

#else
int main() { return EXIT_SUCCESS; }
#endif
