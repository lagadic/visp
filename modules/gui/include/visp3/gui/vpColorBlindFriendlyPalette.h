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
 * Palette of colors that is said to be color-blind people friendly.
 * This palette has been found on https://jfly.uni-koeln.de/color/#see
 *
*****************************************************************************/

#ifndef _vpColorBlindFliendlyPalette_h_
#define _vpColorBlindFliendlyPalette_h_

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
/**
 * \brief Class that furnishes a set of colors that color blind people
 * should be able to distinguish one from another.
 */
BEGIN_VISP_NAMESPACE
class VISP_EXPORT vpColorBlindFriendlyPalette
{
public:
  /**
   * \brief Enum that list the different available colors.
   */
  typedef enum class Palette
  {
    Black = 0,     /*!< Color that looks like black for non-colorblind people.*/
    Orange = 1,    /*!< Color that looks like orange for non-colorblind people.*/
    SkyBlue = 2,   /*!< Color that looks like sky blue for non-colorblind people.*/
    Green = 3,     /*!< Color that looks like green for non-colorblind people.*/
    Yellow = 4,    /*!< Color that looks like yellow for non-colorblind people.*/
    Blue = 5,      /*!< Color that looks like blue for non-colorblind people.*/
    Vermillon = 6, /*!< Color that looks like vermilion for non-colorblind people.*/
    Purple = 7,    /*!< Color that looks like purple for non-colorblind people.*/
    COUNT = 8      /*!< Number of colors the palette proposes.*/
  }Palette;

  static std::vector<std::string> s_paletteNames; /*!< Vector that lists the names of the different available colors.*/

  /**
   * \brief Construct a new vp Color Blind Friendly Palette.
   * The default value \b vpColorBlindFriendlyPalette::Palette::COUNT is affected.
   *
   */
  vpColorBlindFriendlyPalette();

  /**
   * \brief Construct a new vp Color Blind Friendly Palette object.
   *
   * \param colorID \b vpColorBlindFriendlyPalette::Palette that permits to determine the RGB values
   * that we must affect to it.
   */
  vpColorBlindFriendlyPalette(const Palette &colorID);

  /**
   * \brief Construct a new vp Color Blind Friendly Palette object from the name of the color.
   * \b WARNING: if the color is not found, it will be set to \b vpColorBlindFriendlyPalette::Palette::COUNT.
   *
   * \param nameColor
   */
  vpColorBlindFriendlyPalette(const std::string &nameColor);

  /**
   * @brief Get the \b vpColorBlindFriendlyPalette::Palette the object corresponds to.
   *
   * @return Palette Return \b vpColorBlindFriendlyPalette::_colorID.
   */
  Palette get_colorID() const;

  /**
   * \brief Cast a \b vpColorBlindFriendlyPalette in a \b vpColor object.
   * A \b vpColorBlindFriendlyPalette::Palette::COUNT object is set as white (255, 255, 255).
   *
   * \return vpColor The \b vpColor corresponding to the RGB values of the \b vpColorBlindFriendlyPalette .
   */
  vpColor to_vpColor() const;

  /**
   * \brief Cast a \b vpColorBlindFriendlyPalette in a vector {R, G, B}.
   * A \b vpColorBlindFriendlyPalette::Palette::COUNT object is set as white, i.e. {255, 255, 255}.
   *
   * \return std::vector<unsigned char> A vector containing the R, g, B components (in his order), expressed
   * as an unsigned char (so between 0 and 255).
   */
  std::vector<unsigned char> to_RGB() const;

  /**
   * \brief Cast the object in a vector of doubles that belong to the range [0; 1]. The
   * initial R, G, B values are divided by 255.
   * For instance a pixel whose RGB values would be {0; 127.5; 255.} would be transformed in
   * a vector {0.; 0.5; 1.0}.
   *
   * \return std::vector<double> R, G, B values normalized in doubles in the range [0; 1.] by
   * dividing the original values by 255.
   */
  std::vector<double> to_colorRatio() const;

  /**
   * \brief Set the fromString object
   *
   * \param nameColor
   * \return true if a match was found and it was possible to set \b _colorID correctly.
   * \return false if the input string \b nameColor did not match any known names.
   */
  bool set_fromString(const std::string &nameColor);

  /**
   * \brief Get the name of the \b vpColorBlindFriendlyPalette object.
   *
   * \return std::string The name of the color.
   */
  std::string to_string() const;

  /**
   * \brief Get the list of available colors names.
   *
   * \param prefix Optional prefix that will be written before starting the list.
   * \param separator Optional separator between each member of the list.
   * \param suffix Optional suffix that will be written after ending the list.
   * \return std::string The list of names of the available colors.
   */
  static std::string getAvailableColorsNames(const std::string &prefix = "", const std::string &separator = " ", const std::string &suffix = "");
private:
  static std::vector<vpColor> s_palette; /*!< A vector that contains the \b vpColor corresponding to the \b vpColorBlindFriendlyPalette in terms of R, g, B values.*/

  /**
   * \brief Cast \b vpColorBlindFriendlyPalette::Palette enum value into an unsigned int.
   *
   * \param colorID A \b vpColorBlindFriendlyPalette::Palette enum
   * value we are interested in knowing the value, expressed as unsigned int.
   * \return unsigned int that matches the value of \b colorID .
   */
  static unsigned int to_uint(const Palette &colorID);

  /**
   * \brief Get the name that corresponds to \b colorID .
   *
   * \param colorID A \b vpColorBlindFriendlyPalette::Palette enum
   * value we are interested in knowing the name.
   * \return std::string The corresponding name.
   */
  static std::string to_string(const Palette &colorID);

  Palette m_colorID; /*!< The ID of the color in the \b vpColorBlindFriendlyPalette::Palette.*/
};
END_VISP_NAMESPACE

/**
 * \brief Permit to display in a \b std::ostream the name of the \b color.
 *
 * \param os The stream in which we want to write the name of the palette.
 * \param color The color we are interested in displaying the name.
 * \return std::ostream& The stream, in which the name of the \b color has been written.
 */
  std::ostream &operator<<(std::ostream &os, const VISP_NAMESPACE_ADDRESSING vpColorBlindFriendlyPalette &color);

  /**
   * \brief Permits to initialize a \b vpColorBlindFriendlyPalette by reading its name in
   * an \b std::istream.
   *
   * \param is A stream from which we will read the name of \b color in order to initialized it.
   * \param color The color we want to initialized.
   * \return std::istream& The \b is input, from which we have read the name of \b color.
   */
std::istream &operator>>(std::istream &is, VISP_NAMESPACE_ADDRESSING vpColorBlindFriendlyPalette &color);

#endif
#endif // _vpColorBlindFliendlyPalette_h_
