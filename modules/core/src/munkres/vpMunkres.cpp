/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 *
 * Authors:
 * Souriya Trinh
 * Julien Dufour
 *
 *****************************************************************************/

#include <visp3/core/vpMunkres.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)

// System
#include <algorithm>

// Internal
#include <visp3/core/vpMath.h>

// Local helper
namespace
{
constexpr auto ZeroEpsilon{1e-6};
} // namespace

namespace munkres
{

enum vpMunkres::ZERO_T : unsigned int { NA = 0, STARRED = 1, PRIMED = 2 };

enum vpMunkres::STEP_T : unsigned int { ENTRY = 0, ONE = 1, TWO = 2, THREE = 3, FOUR = 4, FIVE = 5, SIX = 6, DONE };

/*!
 * Ensure that the cost matrix is square by the addition of dummy rows/columns.
 *
 * \param[in,out] costs: Cost matrix.
 */
template <typename Type> void vpMunkres::padCostMatrix(std::vector<std::vector<Type> > &costs)
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
std::optional<std::pair<unsigned int, unsigned int> > vpMunkres::findAZero(const std::vector<std::vector<Type> > &costs,
                                                                           const std::vector<bool> &row_cover,
                                                                           const std::vector<bool> &col_cover)
{
  for (auto row = 0u; row < costs.size(); row++)
    for (auto col = 0u; col < costs.size(); col++)
      if (vpMath::equal(costs.at(row).at(col), static_cast<Type>(ZeroEpsilon)) && not row_cover.at(row) &&
          not col_cover.at(col)) {
        return std::make_optional<std::pair<unsigned int, unsigned int> >(row, col);
      }

  return std::nullopt;
}

/*!
 * Find a starred zero in a specific mask matrix row.
 *
 * \param[in] mask: Mask matrix.
 * \param[in] row: Row index.
 * \return Index of the starred zero [col] or std::nullopt if the mask matrix row does not contain a starred zero.
 */
std::optional<unsigned int> vpMunkres::findStarInRow(const std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                                                     const unsigned int &row)
{
  const auto it = std::find(begin(mask.at(row)), end(mask.at(row)), vpMunkres::ZERO_T::STARRED);
  return it != end(mask.at(row)) ? std::make_optional<unsigned int>(std::distance(begin(mask.at(row)), it))
                                 : std::nullopt;
}

/*!
 * Find a starred zero in a specific mask matrix col.
 *
 * \param[in] mask: Mask matrix.
 * \param[in] col : Col index.
 * \return Index of the starred zero [row] or std::nullopt if the mask matrix col does not contain a starred zero.
 */
std::optional<unsigned int> vpMunkres::findStarInCol(const std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                                                     const unsigned int &col)
{
  const auto it = std::find_if(begin(mask), end(mask),
                               [&col](const auto &row) { return row.at(col) == vpMunkres::ZERO_T::STARRED; });
  return it != end(mask) ? std::make_optional<unsigned int>(std::distance(begin(mask), it)) : std::nullopt;
}

/*!
 * Find a primed zero in a specific mask matrix row.
 *
 * \param[in] mask: Mask matrix.
 * \param[in] row : Row index.
 * \return Index of the primed zero [col] or std::nullopt if the mask matrix row does not contain a primed zero.
 */
std::optional<unsigned int> vpMunkres::findPrimeInRow(const std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                                                      const unsigned int &row)
{
  const auto it = std::find(begin(mask.at(row)), end(mask.at(row)), vpMunkres::ZERO_T::PRIMED);
  return it != end(mask.at(row)) ? std::make_optional<unsigned int>(std::distance(begin(mask.at(row)), it))
                                 : std::nullopt;
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
Type vpMunkres::findSmallest(const std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                             const std::vector<bool> &col_cover)
{
  auto minval = std::numeric_limits<Type>::max();
  for (auto row = 0u; row < costs.size(); row++)
    for (auto col = 0u; col < costs.size(); col++)
      if (minval > costs.at(row).at(col) && not row_cover.at(row) && not col_cover.at(col)) {
        minval = costs.at(row).at(col);
      }

  return minval;
}

/*!
 * Unstar each starred zero of the series, star each primed zero of the series.
 *
 * \param[in,out] mask: Mask matrix.
 * \param[in] path: Series.
 */
void vpMunkres::augmentPath(std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                            const std::vector<std::pair<unsigned int, unsigned int> > &path)
{
  for (const auto &[row, col] : path) {
    mask.at(row).at(col) =
        mask.at(row).at(col) == vpMunkres::ZERO_T::STARRED ? vpMunkres::ZERO_T::NA : vpMunkres::ZERO_T::STARRED;
  }
}

/*!
 * Clear coverage matrices (uncover every line in the matrix).
 *
 * \param[in,out] row_cover: Row coverage array.
 * \param[in,out] col_cover: Col coverage array.
 */
void vpMunkres::clearCovers(std::vector<bool> &row_cover, std::vector<bool> &col_cover)
{
  row_cover = std::vector<bool>(row_cover.size(), false);
  col_cover = std::vector<bool>(col_cover.size(), false);
}

/*!
 * Erase primed zeros in the mask matrix.
 *
 * \param[in,out] mask: Mask matrix.
 */
void vpMunkres::erasePrimes(std::vector<std::vector<vpMunkres::ZERO_T> > &mask)
{
  std::for_each(begin(mask), end(mask), [](auto &mask_row) {
    std::for_each(begin(mask_row), end(mask_row), [](auto &val) {
      if (val == vpMunkres::ZERO_T::PRIMED)
        val = vpMunkres::ZERO_T::NA;
    });
  });
}

/*!
 * For each row of the cost matrix, find the smallest element and subtract it from every element in its row.
 * For each col of the cost matrix, find the smallest element and subtract it from every element in its col.
 * When finished, Go to Step 2.
 *
 * \param[in,out] costs: Cost matrix.
 * \return Next step.
 */
template <typename Type> vpMunkres::STEP_T vpMunkres::stepOne(std::vector<std::vector<Type> > &costs)
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
      minval = std::min(minval, cost_row.at(col));
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
vpMunkres::STEP_T vpMunkres::stepTwo(std::vector<std::vector<Type> > &costs,
                                     std::vector<std::vector<vpMunkres::ZERO_T> > &mask, std::vector<bool> &row_cover,
                                     std::vector<bool> &col_cover)
{
  for (auto row = 0u; row < costs.size(); row++) {
    for (auto col = 0u; col < costs.size(); col++) {
      if (vpMath::equal(costs.at(row).at(col), static_cast<Type>(ZeroEpsilon)) && not row_cover.at(row) &&
          not col_cover.at(col)) {
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
 * Cover each column containing a starred zero.
 * If all columns are covered, the starred zeros describe a complete set of unique assignments.
 * In this case, Go to DONE, otherwise, Go to Step 4.
 *
 * \param[in] mask: Mask matrix.
 * \param[in,out] col_cover: Col coverage array.
 * \return Next step.
 */
vpMunkres::STEP_T vpMunkres::stepThree(const std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                                       std::vector<bool> &col_cover)
{
  for (const auto &mask_row : mask) {
    for (auto col = 0u; col < mask_row.size(); col++) {
      if (mask_row.at(col) == vpMunkres::ZERO_T::STARRED) {
        col_cover.at(col) = true;
      }
    }
  }

  const unsigned int col_count = std::count(begin(col_cover), end(col_cover), true);
  return col_count >= mask.size() ? vpMunkres::STEP_T::DONE : vpMunkres::STEP_T(4);
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
 * \return <Next step, <path_row_0 path_col_0>>.
 */
template <typename Type>
std::tuple<vpMunkres::STEP_T, std::optional<std::pair<unsigned int, unsigned int> > >
vpMunkres::stepFour(const std::vector<std::vector<Type> > &costs, std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                    std::vector<bool> &row_cover, std::vector<bool> &col_cover)
{
  if (const auto zero = findAZero(costs, row_cover, col_cover)) {
    const auto [row, col] = zero.value();
    mask.at(row).at(col) = vpMunkres::ZERO_T::PRIMED;

    if (const auto star_in_row = findStarInRow(mask, row)) {
      row_cover.at(row) = true;
      col_cover.at(*star_in_row) = false;
      return {vpMunkres::STEP_T(4), std::nullopt}; // Repeat
    } else {
      return {vpMunkres::STEP_T(5), std::make_optional<std::pair<uint, uint> >(row, col)};
    }
  } else {
    return {vpMunkres::STEP_T(6), std::nullopt};
  }
}

/*!
 * Construct a series (path) of alternating primed and starred zeros as follows.
 * Let Z0 represent the uncovered primed zero found in Step 4.
 * Let Z1 denote the starred zero in the column of Z0 (if any).
 * Let Z2 denote the primed zero in the row of Z1 (there will always be one).
 * Continue until the series terminates at a primed zero that has no starred zero in its column.
 * Unstar each starred zero of the series, star each primed zero of the series, erase all primes and uncover every
 * line in the cost matrix. Return to Step 3.
 *
 * \param[in,out] mask: Mask matrix.
 * \param[in] path_0: Initial path index [<row,col>].
 * \param[in,out] row_cover: Row coverage array.
 * \param[in,out] col_cover: Col coverage array.
 * \return Next step.
 */
vpMunkres::STEP_T vpMunkres::stepFive(std::vector<std::vector<vpMunkres::ZERO_T> > &mask,
                                      const std::pair<unsigned int, unsigned int> &path_0, std::vector<bool> &row_cover,
                                      std::vector<bool> &col_cover)
{
  std::vector<std::pair<unsigned int, unsigned int> > path{path_0}; // Z0

  while (true) {
    if (const auto star_in_col = findStarInCol(mask, path.back().second)) {
      path.emplace_back(*star_in_col, path.back().second); // Z1
    } else {
      augmentPath(mask, path);
      erasePrimes(mask);
      clearCovers(row_cover, col_cover);

      return vpMunkres::STEP_T(3);
    }

    if (const auto prime_in_row = findPrimeInRow(mask, path.back().first)) {
      path.emplace_back(path.back().first, *prime_in_row); // Z2
    }
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
vpMunkres::STEP_T vpMunkres::stepSix(std::vector<std::vector<Type> > &costs, const std::vector<bool> &row_cover,
                                     const std::vector<bool> &col_cover)
{
  const auto minval = findSmallest(costs, row_cover, col_cover);
  for (auto row = 0u; row < costs.size(); row++) {
    for (auto col = 0u; col < costs.size(); col++) {
      if (row_cover.at(row)) {
        costs.at(row).at(col) += minval;
      }

      if (not col_cover.at(col)) {
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
std::vector<std::pair<unsigned int, unsigned int> > vpMunkres::run(std::vector<std::vector<Type> > costs)
{
  const auto original_row_size = costs.size();
  const auto original_col_size = costs.front().size();
  const auto sq_size = std::max(original_row_size, original_col_size);

  auto mask = std::vector<std::vector<vpMunkres::ZERO_T> >(
      sq_size, std::vector<vpMunkres::ZERO_T>(sq_size, vpMunkres::ZERO_T::NA));
  auto row_cover = std::vector<bool>(sq_size, false);
  auto col_cover = std::vector<bool>(sq_size, false);

  std::optional<std::pair<unsigned int, unsigned int> > path_0{std::nullopt};

  auto step{vpMunkres::STEP_T::ENTRY};
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
  std::vector<std::pair<unsigned int, unsigned int> > ret{};
  for (auto i = 0u; i < original_row_size; i++) {
    if (const auto it = std::find(begin(mask.at(i)), end(mask.at(i)), vpMunkres::ZERO_T::STARRED);
        it != end(mask.at(i))) {
      if (const unsigned int j = std::distance(begin(mask.at(i)), it); j < original_col_size) {
        ret.emplace_back(i, j);
      }
    }
  }

  return ret;
}

// Quick fix link issue
using input_data_type = double;

template std::vector<std::pair<unsigned int, unsigned int> >
    vpMunkres::run<input_data_type>(std::vector<std::vector<input_data_type> >);
template void vpMunkres::padCostMatrix<input_data_type>(std::vector<std::vector<input_data_type> > &);
template std::optional<std::pair<unsigned int, unsigned int> >
vpMunkres::findAZero<input_data_type>(const std::vector<std::vector<input_data_type> > &, const std::vector<bool> &,
                                      const std::vector<bool> &);
template input_data_type vpMunkres::findSmallest<input_data_type>(const std::vector<std::vector<input_data_type> > &,
                                                                  const std::vector<bool> &, const std::vector<bool> &);
template vpMunkres::STEP_T vpMunkres::stepOne<input_data_type>(std::vector<std::vector<input_data_type> > &);
template vpMunkres::STEP_T vpMunkres::stepTwo<input_data_type>(std::vector<std::vector<input_data_type> > &,
                                                               std::vector<std::vector<vpMunkres::ZERO_T> > &,
                                                               std::vector<bool> &, std::vector<bool> &);
template std::tuple<vpMunkres::STEP_T, std::optional<std::pair<unsigned int, unsigned int> > >
vpMunkres::stepFour<input_data_type>(const std::vector<std::vector<input_data_type> > &,
                                     std::vector<std::vector<vpMunkres::ZERO_T> > &, std::vector<bool> &,
                                     std::vector<bool> &);
template vpMunkres::STEP_T vpMunkres::stepSix<input_data_type>(std::vector<std::vector<input_data_type> > &,
                                                               const std::vector<bool> &, const std::vector<bool> &);

} // namespace munkres

#endif
