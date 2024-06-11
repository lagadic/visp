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
 * Description:
 * Real-time 3D point clouds plotter based on the PCL library.
 *
*****************************************************************************/

#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/core/vpIoTools.h>

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)

BEGIN_VISP_NAMESPACE
std::vector<std::string> vpColorBlindFriendlyPalette::s_paletteNames =
{
  "black"     ,
  "orange"    ,
  "sky-blue"  ,
  "green"     ,
  "yellow"    ,
  "blue"      ,
  "vermillon" ,
  "purple"    ,
  "unknown"
};

std::vector<vpColor> vpColorBlindFriendlyPalette::s_palette = {
  vpColor(0,0,0),       // Black = 0,
  vpColor(230,159,0),   // Orange = 1,
  vpColor(86,180,233),  // SkyBlue = 2,
  vpColor(0,158,115),   // Green = 3,
  vpColor(240,228,66),  // Yellow = 4,
  vpColor(0,114,178),   // Blue = 5,
  vpColor(213,94,0),    // Vermillon = 6,
  vpColor(204,121,167), // Purple = 7,
  vpColor(255,255,255)  // COUNT = 8
};

vpColorBlindFriendlyPalette::vpColorBlindFriendlyPalette()
  : m_colorID(Palette::COUNT)
{

}

vpColorBlindFriendlyPalette::vpColorBlindFriendlyPalette(const vpColorBlindFriendlyPalette::Palette &colorID)
  : m_colorID(colorID)
{

}

vpColorBlindFriendlyPalette::vpColorBlindFriendlyPalette(const std::string &nameColor)
  : m_colorID(Palette::COUNT)
{
  set_fromString(nameColor);
}

vpColorBlindFriendlyPalette::Palette vpColorBlindFriendlyPalette::get_colorID() const
{
  return m_colorID;
}

vpColor vpColorBlindFriendlyPalette::to_vpColor() const
{
  return s_palette[to_uint(m_colorID)];
}

std::vector<unsigned char> vpColorBlindFriendlyPalette::to_RGB() const
{
  vpColor color = s_palette[to_uint(m_colorID)];
  std::vector<unsigned char> v_rgb;
  v_rgb.push_back(color.R);
  v_rgb.push_back(color.G);
  v_rgb.push_back(color.B);
  return v_rgb;
}

std::vector<double> vpColorBlindFriendlyPalette::to_colorRatio() const
{
  vpColor color = s_palette[to_uint(m_colorID)];
  std::vector<double> v_rgb;
  v_rgb.push_back((double)color.R / 255.0);
  v_rgb.push_back((double)color.G / 255.0);
  v_rgb.push_back((double)color.B / 255.0);
  return v_rgb;
}

bool vpColorBlindFriendlyPalette::set_fromString(const std::string &nameColor)
{
  m_colorID = Palette::COUNT;
  std::string nameLowerCase = nameColor; // vpIoTools::toLowerCase(nameColor);
  bool wasFound(false);
  for (unsigned int i = 0; i < to_uint(Palette::COUNT) && !wasFound; i++) {
    vpColorBlindFriendlyPalette::Palette candidate = (Palette)i;
    if (to_string(candidate) == nameLowerCase) {
      m_colorID = candidate;
      wasFound = true;
    }
  }
  return wasFound;
}

std::string vpColorBlindFriendlyPalette::to_string() const
{
  std::string nameColor = to_string(m_colorID);
  return nameColor;
}

std::string vpColorBlindFriendlyPalette::getAvailableColorsNames(const std::string &prefix, const std::string &separator, const std::string &suffix)
{
  std::string list(prefix);
  const unsigned int nbAvailableColors = (unsigned int)Palette::COUNT;
  for (unsigned int i = 0; i < nbAvailableColors - 1; i++) {
    std::string nameCandidateID = s_paletteNames[i];
    list += nameCandidateID + separator;
  }
  list += s_paletteNames[nbAvailableColors - 1] + suffix;
  return list;
}

unsigned int vpColorBlindFriendlyPalette::to_uint(const Palette &colorID)
{
  const unsigned int nbAvailableColors = (unsigned int)Palette::COUNT;
  unsigned int ID = nbAvailableColors;
  std::string nameSearchedColor = to_string(colorID);
  bool wasFound = false;
  for (unsigned int i = 0; i < nbAvailableColors && !wasFound; i++) {
    Palette candidate = (Palette)i;
    if (to_string(candidate) == nameSearchedColor) {
      ID = i;
      wasFound = true;
    }
  }
  return ID;
}

std::string vpColorBlindFriendlyPalette::to_string(const vpColorBlindFriendlyPalette::Palette &colorID)
{
  std::string nameColor;
  switch (colorID) {
  case Palette::Black:
    nameColor = s_paletteNames[0];
    break;
  case Palette::Orange:
    nameColor = s_paletteNames[1];
    break;
  case Palette::SkyBlue:
    nameColor = s_paletteNames[2];
    break;
  case Palette::Green:
    nameColor = s_paletteNames[3];
    break;
  case Palette::Yellow:
    nameColor = s_paletteNames[4];
    break;
  case Palette::Blue:
    nameColor = s_paletteNames[5];
    break;
  case Palette::Vermillon:
    nameColor = s_paletteNames[6];
    break;
  case Palette::Purple:
    nameColor = s_paletteNames[7];
    break;
  default:
    nameColor = s_paletteNames[8];
  }
  return nameColor;
}

END_VISP_NAMESPACE

std::ostream &operator<<(std::ostream &os, const VISP_NAMESPACE_ADDRESSING vpColorBlindFriendlyPalette &color)
{
  os << color.to_string();
  return os;
}

std::istream &operator>>(std::istream &is, VISP_NAMESPACE_ADDRESSING vpColorBlindFriendlyPalette &color)
{
  std::string nameColor;
  is >> nameColor;
  color.set_fromString(nameColor);
  return is;
}
#else
void dummy_vpColorBlindFriendlyPalette() { }
#endif
