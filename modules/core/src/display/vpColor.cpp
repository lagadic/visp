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
 * Color definition.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpColor.h>

// FS: Sould be improved to avoid the #if preprocessor line. Not a good idea
// to define colors in static.
//     See also vpColor.h where things need to be improved.
//#if !defined(visp_EXPORTS)
#if !defined(VISP_USE_MSVC) || (defined(VISP_USE_MSVC) && !defined(VISP_BUILD_SHARED_LIBS))
/*!< Predefined black color with R=G=B=0 and identifier vpColor::id_black. */
vpColor const vpColor::black = vpColor(0, 0, 0, id_black);
/*!< Predefined white color with R=G=B=255 and identifier vpColor::id_white.
 */
vpColor const vpColor::white = vpColor(255, 255, 255, id_white);
/*!< Predefined light gray color with R=G=B=64 and identifier
 * vpColor::id_lightGray. */
vpColor const vpColor::lightGray = vpColor(192, 192, 192, id_lightGray);
/*!< Predefined gray color with R=G=B=128 and identifier vpColor::id_gray. */
vpColor const vpColor::gray = vpColor(128, 128, 128, id_gray);
/*!< Predefined dark gray color with R=G=B=192 and identifier
 * vpColor::id_darkGray. */
vpColor const vpColor::darkGray = vpColor(64, 64, 64, id_darkGray);
/*!< Predefined light red color with R= 255 and G=B=140 and identifier
   vpColor::id_lightRed. */
vpColor const vpColor::lightRed = vpColor(255, 140, 140, id_lightRed);
/*!< Predefined red color with R=255 and G=B=0 and identifier
   vpColor::id_red. */
vpColor const vpColor::red = vpColor(255, 0, 0, id_red);
/*!< Predefined dark red color with R= 128 and G=B=0 and identifier
   vpColor::id_darkRed. */
vpColor const vpColor::darkRed = vpColor(128, 0, 0, id_darkRed);
/*!< Predefined light green color with G= 255 and R=B=140 and identifier
   vpColor::id_lightGreen. */
vpColor const vpColor::lightGreen = vpColor(140, 255, 140, id_lightGreen);
/*!< Predefined green color with G=255 and R=B=0 and identifier
   vpColor::id_green. */
vpColor const vpColor::green = vpColor(0, 255, 0, id_green);
/*!< Predefined dark green color with G= 128 and R=B=0 and identifier
   vpColor::id_darkGreen. */
vpColor const vpColor::darkGreen = vpColor(0, 128, 0, id_darkGreen);
/*!< Predefined light blue color with B= 255 and R=G=140 and identifier
   vpColor::id_lightBlue. */
vpColor const vpColor::lightBlue = vpColor(140, 140, 255, id_lightBlue);
/*!< Predefined blue color with R=G=0 and B=255 and identifier
   vpColor::id_blue. */
vpColor const vpColor::blue = vpColor(0, 0, 255, id_blue);
/*!< Predefined dark blue color with B= 128 and R=G=0 and identifier
   vpColor::id_darkBlue. */
vpColor const vpColor::darkBlue = vpColor(0, 0, 128, id_darkBlue);
/*!< Predefined yellow color with R=G=255 and B=0 and identifier
   vpColor::id_yellow. */
vpColor const vpColor::yellow = vpColor(255, 255, 0, id_yellow);
/*!< Predefined cyan color with R=0 and G=B=255 and identifier
   vpColor::id_cyan. */
vpColor const vpColor::cyan = vpColor(0, 255, 255, id_cyan);
/*!< Predefined orange color with R=255, G=165 and B=0 and identifier
   vpColor::id_orange. */
vpColor const vpColor::orange = vpColor(255, 165, 0, id_orange);
/*!< Predefined purple color with R=128, G=0 and B=128 and identifier
   vpColor::id_purple. */
vpColor const vpColor::purple = vpColor(128, 0, 128, id_purple);
/*!< Predefined none color with R=G=B=0 and identifier vpColor::id_unknown. */
vpColor const vpColor::none = vpColor(0, 0, 0, id_unknown);

const unsigned int vpColor::nbColors = 18;

/*!< Array of available colors. */
vpColor const vpColor::allColors[vpColor::nbColors] = {vpColor::blue,       // 12
                                                       vpColor::green,      // 9
                                                       vpColor::red,        // 6
                                                       vpColor::cyan,       // 15
                                                       vpColor::purple,     // 4
                                                       vpColor::yellow,     // 14
                                                       vpColor::orange,     // 16
                                                       vpColor::lightBlue,  // 11
                                                       vpColor::lightGreen, // 8
                                                       vpColor::lightRed,   // 5
                                                       vpColor::darkBlue,   // 13
                                                       vpColor::darkGreen,  // 10
                                                       vpColor::darkRed,    // 7
                                                       vpColor::lightGray,  // 2
                                                       vpColor::gray,       // 3
                                                       vpColor::darkGray,   // 4
                                                       vpColor::black,      // 0
                                                       vpColor::white};     // 17
#endif

vpColor colors[6] = {vpColor::blue, vpColor::green, vpColor::red, vpColor::cyan, vpColor::orange, vpColor::purple};

/*!
  Compare two colors.

  Return true if the R,G,B components are the same.

  \param c1,c2 : Color to compare.
*/
VISP_EXPORT bool operator==(const vpColor &c1, const vpColor &c2)
{
  return ((c1.R == c2.R) && (c1.G == c2.G) && (c1.B == c2.B));
}

/*!

  Compare two colors.

  Return true if the R,G,B components are different.

  \param c1,c2 : Color to compare.
*/
VISP_EXPORT bool operator!=(const vpColor &c1, const vpColor &c2)
{
  return ((c1.R != c2.R) || (c1.G != c2.G) || (c1.B == c2.B));
}
