/****************************************************************************
 *
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
 *
 * Authors:
 * Souriya Trinh
 * Julien Dufour
 *
*****************************************************************************/

#include <visp3/core/vpMunkres.h>

// Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

BEGIN_VISP_NAMESPACE
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

  const unsigned int col_count = static_cast<unsigned int>(std::count(begin(col_cover), end(col_cover), true));
  return col_count >= mask.size() ? vpMunkres::STEP_T::DONE : vpMunkres::STEP_T(4);
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
  std::vector<std::pair<unsigned int, unsigned int> > path { path_0 }; // Z0

  while (true) {
    if (const auto star_in_col = findStarInCol(mask, path.back().second)) {
      path.emplace_back(*star_in_col, path.back().second); // Z1
    }
    else {
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
END_VISP_NAMESPACE
#endif
