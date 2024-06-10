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
 * Class for Munkres Assignment Algorithm.
 */

#pragma once

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher.
// Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) in the declaration of the class
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

// System
#include <optional>
#include <tuple>

// Internal
#include "vpMath.h"

BEGIN_VISP_NAMESPACE
/*!
  \class vpMunkres
  \ingroup group_core_munkres

  Implements the Munkres Assignment Algorithm described [here](https://en.wikipedia.org/wiki/Hungarian_algorithm).

  \note This class is only available with c++17 enabled.
*/
class VISP_EXPORT vpMunkres
{
public:
  template <typename Type>
  static std::vector<std::pair<unsigned int, unsigned int> > run(std::vector<std::vector<Type> > costs);

private:
  enum ZERO_T : unsigned int;
  enum STEP_T : unsigned int;

  // Init
  template <typename Type> static void padCostMatrix(std::vector<std::vector<Type> > &costs);

  // Global helpers
  template <typename Type>
  static std::optional<std::pair<unsigned int, unsigned int> > findAZero(const std::vector<std::vector<Type> > &costs,
                                                                         const std::vector<bool> &row_cover,
                                                                         const std::vector<bool> &col_cover);
  static std::optional<unsigned int> findStarInRow(const std::vector<std::vector<ZERO_T> > &mask,
                                                   const unsigned int &row);
  static std::optional<unsigned int> findStarInCol(const std::vector<std::vector<ZERO_T> > &mask,
                                                   const unsigned int &col);
  static std::optional<unsigned int> findPrimeInRow(const std::vector<std::vector<ZERO_T> > &mask,
                                                    const unsigned int &row);
  template <typename Type>
  static Type findSmallest(const std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                           const std::vector<bool> &col_cover);

  // FSM helpers
  static void augmentPath(std::vector<std::vector<ZERO_T> > &mask,
                          const std::vector<std::pair<unsigned int, unsigned int> > &path);
  static void clearCovers(std::vector<bool> &row_cover, std::vector<bool> &col_cover);
  static void erasePrimes(std::vector<std::vector<ZERO_T> > &mask);

  // FSM
  template <typename Type> static STEP_T stepOne(std::vector<std::vector<Type> > &costs);
  template <typename Type>
  static STEP_T stepTwo(std::vector<std::vector<Type> > &costs, std::vector<std::vector<ZERO_T> > &mask,
                        std::vector<bool> &row_cover, std::vector<bool> &col_cover);
  static STEP_T stepThree(const std::vector<std::vector<ZERO_T> > &mask, std::vector<bool> &col_cover);
  template <typename Type>
  static std::tuple<STEP_T, std::optional<std::pair<unsigned int, unsigned int> > >
    stepFour(const std::vector<std::vector<Type> > &costs, std::vector<std::vector<ZERO_T> > &mask,
             std::vector<bool> &row_cover, std::vector<bool> &col_cover);
  static STEP_T stepFive(std::vector<std::vector<ZERO_T> > &mask, const std::pair<unsigned int, unsigned int> &path_0,
                         std::vector<bool> &row_cover, std::vector<bool> &col_cover);
  template <typename Type>
  static STEP_T stepSix(std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                        const std::vector<bool> &col_cover);

private:
  static constexpr auto ZeroEpsilon { 1e-6 };
};

enum vpMunkres::ZERO_T : unsigned int { NA = 0, STARRED = 1, PRIMED = 2 };

enum vpMunkres::STEP_T : unsigned int { ENTRY = 0, ONE = 1, TWO = 2, THREE = 3, FOUR = 4, FIVE = 5, SIX = 6, DONE };

/*!
 * Ensure that the cost matrix is square by the addition of dummy rows/columns.
 *
 * \param[in,out] costs: Cost matrix.
 */
template <typename Type> inline void vpMunkres::padCostMatrix(std::vector<std::vector<Type> > &costs)
{
  const auto row_input_size = costs.size();
  const auto col_input_size = costs.at(0).size();

  if (row_input_size > col_input_size) {
    for (auto &vec : costs)
      vec.resize(row_input_size, 0);
  }

  while (costs.size() < col_input_size) {
    costs.emplace_back(col_input_size, 0);
  }
}

/*!
 * Find a zero in the cost matrix.
 *
 * \param[in] costs: Cost matrix.
 * \param[in] row_cover: Row coverage array.
 * \param[in] col_cover: Col coverage array.
 * \return Index of the Zero [<row,col>] or std::nullopt if the cost matrix does not contain a zero.
 */
template <typename Type>
inline std::optional<std::pair<unsigned int, unsigned int> >
vpMunkres::findAZero(const std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                     const std::vector<bool> &col_cover)
{
  for (auto row = 0u; row < costs.size(); row++)
    for (auto col = 0u; col < costs.size(); col++)
      if (vpMath::equal(costs.at(row).at(col), static_cast<Type>(vpMunkres::ZeroEpsilon)) && !row_cover.at(row) &&
          !col_cover.at(col)) {
        return std::make_optional<std::pair<unsigned int, unsigned int> >(row, col);
      }

  return std::nullopt;
}

/*!
 * Find the smallest value of the cost matrix.
 *
 * \param[in] costs: Cost matrix.
 * \param[in] row_cover: Row coverage array.
 * \param[in] col_cover: Col coverage array.
 * \return Smallest value of the cost matrix.
 */
template <typename Type>
inline Type vpMunkres::findSmallest(const std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                                    const std::vector<bool> &col_cover)
{
  auto minval = std::numeric_limits<Type>::max();
  for (auto row = 0u; row < costs.size(); row++)
    for (auto col = 0u; col < costs.size(); col++)
      if (minval > costs.at(row).at(col) && !row_cover.at(row) && !col_cover.at(col)) {
        minval = costs.at(row).at(col);
      }

  return minval;
}

/*!
 * For each row of the cost matrix, find the smallest element and subtract it from every element in its row.
 * For each col of the cost matrix, find the smallest element and subtract it from every element in its col.
 * When finished, Go to Step 2.
 *
 * \param[in,out] costs: Cost matrix.
 * \return Next step.
 */
template <typename Type> inline vpMunkres::STEP_T vpMunkres::stepOne(std::vector<std::vector<Type> > &costs)
{
  // process rows
  std::for_each(begin(costs), end(costs), [](auto &cost_row) {
    const auto min_in_row = *std::min_element(begin(cost_row), end(cost_row));
    std::transform(begin(cost_row), end(cost_row), begin(cost_row),
                   [&min_in_row](auto &cost) { return cost - min_in_row; });
  });

  // process cols
  for (auto col = 0u; col < costs.size(); ++col) {
    auto minval = std::numeric_limits<Type>::max();
    for (const auto &cost_row : costs) {
      minval = std::min<Type>(minval, cost_row.at(col));
    }

    for (auto &cost_row : costs) {
      cost_row.at(col) -= minval;
    }
  }

  return vpMunkres::STEP_T(2);
}

/*!
 * Find a zero (Z) in the cost matrix. If there is no starred zero in its row or column, star Z. Repeat for each
 * element in the cost matrix.
 * When finished, Go to Step 3.
 *
 * \param[in,out] costs: Cost matrix.
 * \param[in] mask: Mask matrix.
 * \param[in,out] row_cover: Row coverage array.
 * \param[in,out] col_cover: Col coverage array.
 * \return Next step.
 */
template <typename Type>
inline vpMunkres::STEP_T vpMunkres::stepTwo(std::vector<std::vector<Type> > &costs,
                                            std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                                            std::vector<bool> &row_cover, std::vector<bool> &col_cover)
{
  for (auto row = 0u; row < costs.size(); row++) {
    for (auto col = 0u; col < costs.size(); col++) {
      if (vpMath::equal(costs.at(row).at(col), static_cast<Type>(vpMunkres::ZeroEpsilon)) && !row_cover.at(row) &&
          !col_cover.at(col)) {
        mask.at(row).at(col) = vpMunkres::ZERO_T::STARRED;
        row_cover.at(row) = true;
        col_cover.at(col) = true;
        break;
      }
    }
  }

  clearCovers(row_cover, col_cover);
  return vpMunkres::STEP_T(3);
}

/*!
 * Find a noncovered zero and prime it.
 * If there is no starred zero in the row containing this primed zero, Go to Step 5.
 * Otherwise, cover this row and uncover the column containing the starred zero. Continue in this manner until there
 * are no uncovered zeros left. Go to Step 6.
 *
 * \param[in] costs: Cost matrix.
 * \param[in,out] mask: Mask matrix.
 * \param[in,out] row_cover: Row coverage array.
 * \param[in,out] col_cover: Col coverage array.
 * \return Tuple(Next step, pair(path_row_0 path_col_0)).
 */
template <typename Type>
inline std::tuple<vpMunkres::STEP_T, std::optional<std::pair<unsigned int, unsigned int> > >
vpMunkres::stepFour(const std::vector<std::vector<Type> > &costs, std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                    std::vector<bool> &row_cover, std::vector<bool> &col_cover)
{
  if (const auto zero = findAZero(costs, row_cover, col_cover)) {
    const auto [row, col] = *zero; // convenient zero.value() is not working on iOS
    mask.at(row).at(col) = vpMunkres::ZERO_T::PRIMED;

    if (const auto star_in_row = findStarInRow(mask, row)) {
      row_cover.at(row) = true;
      col_cover.at(*star_in_row) = false;
      return { vpMunkres::STEP_T(4), std::nullopt }; // Repeat
    }
    else {
      return { vpMunkres::STEP_T(5), std::make_optional<std::pair<unsigned int, unsigned int> >(row, col) };
    }
  }
  else {
    return { vpMunkres::STEP_T(6), std::nullopt };
  }
}

/*!
 * Add the smallest value of the cost matrix to every element of each covered row, and subtract it from every element
 * of each uncovered column. Return to Step 4 without altering any stars, primes, or covered lines.
 *
 * \param[in,out] costs: Cost matrix.
 * \param[in] row_cover: Row coverage array.
 * \param[in] col_cover: Col coverage array.
 * \return Next step.
 */
template <typename Type>
inline vpMunkres::STEP_T vpMunkres::stepSix(std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                                            const std::vector<bool> &col_cover)
{
  const auto minval = findSmallest(costs, row_cover, col_cover);
  for (auto row = 0u; row < costs.size(); row++) {
    for (auto col = 0u; col < costs.size(); col++) {
      if (row_cover.at(row)) {
        costs.at(row).at(col) += minval;
      }

      if (!col_cover.at(col)) {
        costs.at(row).at(col) -= minval;
      }
    }
  }

  return vpMunkres::STEP_T(4);
}

/*!
 * Munkres FSM.
 *
 * \param[in] costs: Cost matrix.
 * \return List of associated pairs [<row_idx,col_idx>].
 */
template <typename Type>
inline std::vector<std::pair<unsigned int, unsigned int> > vpMunkres::run(std::vector<std::vector<Type> > costs)
{
  const auto original_row_size = static_cast<Type>(costs.size());
  const auto original_col_size = static_cast<Type>(costs.front().size());
  const size_t sq_size = static_cast<size_t>(std::max<Type>(original_row_size, original_col_size));

  auto mask = std::vector<std::vector<vpMunkres::ZERO_T> >(sq_size, std::vector<vpMunkres::ZERO_T>(sq_size, vpMunkres::ZERO_T::NA));
  auto row_cover = std::vector<bool>(sq_size, false);
  auto col_cover = std::vector<bool>(sq_size, false);

  std::optional<std::pair<unsigned int, unsigned int> > path_0 { std::nullopt };

  auto step { vpMunkres::STEP_T::ENTRY };
  while (step != vpMunkres::STEP_T::DONE) {
    switch (step) {
    case vpMunkres::STEP_T::ENTRY:
      padCostMatrix(costs);
      step = vpMunkres::STEP_T(1);
      break;
    case 1:
      step = stepOne(costs);
      break;
    case 2:
      step = stepTwo(costs, mask, row_cover, col_cover);
      break;
    case 3:
      step = stepThree(mask, col_cover);
      break;
    case 4:
      std::tie(step, path_0) = stepFour(costs, mask, row_cover, col_cover);
      break;
    case 5:
      step = stepFive(mask, *path_0, row_cover, col_cover);
      break;
    case 6:
      step = stepSix(costs, row_cover, col_cover);
      break;
    case vpMunkres::STEP_T::DONE:
    default:
      break;
    }
  }

  // Compute the pairs
  std::vector<std::pair<unsigned int, unsigned int> > ret {};
  for (auto i = 0u; i < original_row_size; i++) {
    if (const auto it = std::find(begin(mask.at(i)), end(mask.at(i)), vpMunkres::ZERO_T::STARRED);
        it != end(mask.at(i))) {
      if (const unsigned int j = static_cast<unsigned int>(std::distance(begin(mask.at(i)), it));
          j < original_col_size) {
        ret.emplace_back(i, j);
      }
    }
  }

  return ret;
}
END_VISP_NAMESPACE
#endif
