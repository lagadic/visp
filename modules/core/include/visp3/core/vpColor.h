/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpColor_hh
#define vpColor_hh

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRGBa.h>

/*!

  \class vpColor

  \ingroup group_core_gui

  \brief Class to define colors available for display functionnalities.

  You may use predefined colors (black, white, red, green, blue, cyan
  and orange) or specify your own color by settings its R,G,B values.

  An identifier vpColorIdentifier is associated to each color. This
  identifier is useful to determine if a color is predefined or
  specified by it R,G,B values. In that last case, the identifier is
  always set to vpColor::id_unknown.

  The example below shows how to display geometric features in a
  display overlay using predefined colors (here the blue color to draw
  a circle) and a specific brown color (used to draw a rectangle).

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
  vpImage<unsigned char> I(240, 320); // Create a black grey level image

  vpDisplay *d;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a filled circle with the predefined blue color
  vpDisplay::displayCircle(I, 100, 200, 30, vpColor::blue, true);

  // Creation of a new brown color with its RGB values
  vpColor color(128, 100, 50);

  // Draw a brown rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, color, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  delete d;
}
  \endcode

*/
class VISP_EXPORT vpColor : public vpRGBa
{
public:
  /*! Predefined colors identifier. */
  typedef enum {
    id_black = 0,  /*!< Identifier associated to the predefined vpColor::black
                      color. */
    id_white,      /*!< Identifier associated to the predefined vpColor::white
                      color. */
    id_lightGray,  /*!< Identifier associated to the predefined
                      vpColor::lightGray color. */
    id_gray,       /*!< Identifier associated to the predefined vpColor::gray color.
                    */
    id_darkGray,   /*!< Identifier associated to the predefined
                      vpColor::darkGray color. */
    id_lightRed,   /*!< Identifier associated to the predefined
                      vpColor::lightRed color. */
    id_red,        /*!< Identifier associated to the predefined vpColor::red color.
                    */
    id_darkRed,    /*!< Identifier associated to the predefined vpColor::darkRed
                      color. */
    id_lightGreen, /*!< Identifier associated to the predefined
                      vpColor::lightGreen color. */
    id_green,      /*!< Identifier associated to the predefined vpColor::green
                      color. */
    id_darkGreen,  /*!< Identifier associated to the predefined
                      vpColor::darkGreen color. */
    id_lightBlue,  /*!< Identifier associated to the predefined
                      vpColor::lightBlue color. */
    id_blue,       /*!< Identifier associated to the predefined vpColor::blue color.
                    */
    id_darkBlue,   /*!< Identifier associated to the predefined
                      vpColor::darkBlue color. */
    id_yellow,     /*!< Identifier associated to the predefined vpColor::yellow
                      color. */
    id_cyan,       /*!< Identifier associated to the predefined vpColor::cyan color.
                    */
    id_orange,     /*!< Identifier associated to the predefined vpColor::orange
                      color. */
    id_purple,     /*!< Identifier associated to the predefined vpColor::purple
                      color. */

    id_unknown /*!< Identifier associated to unknowned
                  colors. By unknowned, we mean not a predefined
                  color. This identifier can also be used to know
                  the number of predefined colors. */

  } vpColorIdentifier;

  vpColorIdentifier id; /*!< Color identifier to indicate if a color
                           is predefined or set by the user using its
                           RGB values. */

  /* Predefined colors. */
  static const vpColor black;
  static const vpColor white;
  static const vpColor lightGray;
  static const vpColor gray;
  static const vpColor darkGray;
  static const vpColor lightRed;
  static const vpColor red;
  static const vpColor darkRed;
  static const vpColor lightGreen;
  static const vpColor green;
  static const vpColor darkGreen;
  static const vpColor lightBlue;
  static const vpColor blue;
  static const vpColor darkBlue;
  static const vpColor yellow;
  static const vpColor cyan;
  static const vpColor orange;
  static const vpColor purple;
  static const vpColor none;

  static const unsigned int nbColors;
  static const vpColor allColors[];

  /*!
    Default constructor. All the colors components are set to zero.

    The color identifier is set to vpColor::id_unknown to indicate
    that this color is not a predefined one.

  */
  inline vpColor() : vpRGBa(), id(id_unknown){};
  /*!
    Construct a color from its RGB values.

    \param r : Red component.
    \param g : Green component.
    \param b : Blue component.

    \param cid : The color identifier to indicate if this color is or
    not a predefined one.
  */
  inline vpColor(unsigned char r, unsigned char g, unsigned char b,
                 vpColor::vpColorIdentifier cid = vpColor::id_unknown)
    : vpRGBa(r, g, b), id(cid){};
  /*! Default destructor. */
  inline virtual ~vpColor(){};

  friend VISP_EXPORT bool operator==(const vpColor &c1, const vpColor &c2);
  friend VISP_EXPORT bool operator!=(const vpColor &c1, const vpColor &c2);
  /*!
    Set a color from its RGB values.

    \param r : Red component.
    \param g : Green component.
    \param b : Blue component.

    The color identifier is set to vpColor::id_unknown to indicate
    that this color is not a predefined one.

  */
  inline void setColor(unsigned char r, unsigned char g, unsigned char b)
  {
    this->R = r;
    this->G = g;
    this->B = b;
    this->A = 0;
    id = id_unknown;
  };

  /*!

   Get a predefined color

   \param i : color indice
   */
  static inline vpColor getColor(const unsigned int &i) { return vpColor::allColors[i % vpColor::nbColors]; };
};

// In this file if windows
#if defined(VISP_USE_MSVC) && defined(visp_EXPORTS)
/*!< Predefined black color with R=G=B=0 and identifier vpColor::id_black. */
vpColor const __declspec(selectany) vpColor::black = vpColor(0, 0, 0, id_black);
/*!< Predefined white color with R=G=B=255 and identifier vpColor::id_white.
 */
vpColor const __declspec(selectany) vpColor::white = vpColor(255, 255, 255, id_white);
/*!< Predefined light gray color with R=G=B=64 and identifier
 * vpColor::id_lightGray. */
vpColor const __declspec(selectany) vpColor::lightGray = vpColor(192, 192, 192, id_lightGray);
/*!< Predefined gray color with R=G=B=128 and identifier vpColor::id_gray. */
vpColor const __declspec(selectany) vpColor::gray = vpColor(128, 128, 128, id_gray);
/*!< Predefined dark gray color with R=G=B=192 and identifier
 * vpColor::id_darkGray. */
vpColor const __declspec(selectany) vpColor::darkGray = vpColor(64, 64, 64, id_darkGray);
/*!< Predefined light red color with R= 255 and G=B=140 and identifier
   vpColor::id_lightRed. */
vpColor const __declspec(selectany) vpColor::lightRed = vpColor(255, 140, 140, id_lightRed);
/*!< Predefined red color with R=255 and G=B=0 and identifier
   vpColor::id_red. */
vpColor const __declspec(selectany) vpColor::red = vpColor(255, 0, 0, id_red);
/*!< Predefined dark red color with R= 128 and G=B=0 and identifier
   vpColor::id_darkRed. */
vpColor const __declspec(selectany) vpColor::darkRed = vpColor(128, 0, 0, id_darkRed);
/*!< Predefined light green color with G= 255 and R=B=140 and identifier
   vpColor::id_lightGreen. */
vpColor const __declspec(selectany) vpColor::lightGreen = vpColor(140, 255, 140, id_lightGreen);
/*!< Predefined green color with G=255 and R=B=0 and identifier
   vpColor::id_green. */
vpColor const __declspec(selectany) vpColor::green = vpColor(0, 255, 0, id_green);
/*!< Predefined dark green color with G= 128 and R=B=0 and identifier
   vpColor::id_darkGreen. */
vpColor const __declspec(selectany) vpColor::darkGreen = vpColor(0, 128, 0, id_darkGreen);
/*!< Predefined light blue color with B= 255 and R=G=140 and identifier
   vpColor::id_lightBlue. */
vpColor const __declspec(selectany) vpColor::lightBlue = vpColor(140, 140, 255, id_lightBlue);
/*!< Predefined blue color with R=G=0 and B=255 and identifier
   vpColor::id_blue. */
vpColor const __declspec(selectany) vpColor::blue = vpColor(0, 0, 255, id_blue);
/*!< Predefined dark blue color with B= 128 and R=G=0 and identifier
   vpColor::id_darkBlue. */
vpColor const __declspec(selectany) vpColor::darkBlue = vpColor(0, 0, 128, id_darkBlue);
/*!< Predefined yellow color with R=G=255 and B=0 and identifier
   vpColor::id_yellow. */
vpColor const __declspec(selectany) vpColor::yellow = vpColor(255, 255, 0, id_yellow);
/*!< Predefined cyan color with R=0 and G=B=255 and identifier
   vpColor::id_cyan. */
vpColor const __declspec(selectany) vpColor::cyan = vpColor(0, 255, 255, id_cyan);
/*!< Predefined orange color with R=255, G=165 and B=0 and identifier
   vpColor::id_orange. */
vpColor const __declspec(selectany) vpColor::orange = vpColor(255, 165, 0, id_orange);
/*!< Predefined purple color with R=128, G=0 and B=128 and identifier
   vpColor::id_purple. */
vpColor const __declspec(selectany) vpColor::purple = vpColor(128, 0, 128, id_purple);
/*!< Predefined none color with R=G=B=0 and identifier vpColor::id_unknown. */
vpColor const __declspec(selectany) vpColor::none = vpColor(0, 0, 0, id_unknown);

const __declspec(selectany) unsigned int vpColor::nbColors = 18;

/*!< Array of available colors. */
vpColor const __declspec(selectany) vpColor::allColors[vpColor::nbColors] = {vpColor::blue,       // 12
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

#endif
