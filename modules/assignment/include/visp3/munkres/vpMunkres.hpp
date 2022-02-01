/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Fabien Spindler
 *
 *****************************************************************************/

// System
#include <optional>
#include <vector>

namespace munkres
{

class vpMunkres
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
};

} // namespace munkres
