/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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


#include <visp/vpConfig.h>
#include <visp/vpRGBa.h>

/*!

  \class vpColor

  \ingroup ImageGUI

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
#include <visp/vpConfig.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayOpenCV.h>

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
    id_black=0,    /*!< Identifier associated to the predefined vpColor::black color. */
    id_white,      /*!< Identifier associated to the predefined vpColor::white color. */
    id_lightGray,  /*!< Identifier associated to the predefined vpColor::lightGray color. */
    id_gray,       /*!< Identifier associated to the predefined vpColor::gray color. */
    id_darkGray,   /*!< Identifier associated to the predefined vpColor::darkGray color. */
    id_lightRed,   /*!< Identifier associated to the predefined vpColor::lightRed color. */
    id_red,        /*!< Identifier associated to the predefined vpColor::red color. */
    id_darkRed,    /*!< Identifier associated to the predefined vpColor::darkRed color. */
    id_lightGreen, /*!< Identifier associated to the predefined vpColor::lightGreen color. */
    id_green,      /*!< Identifier associated to the predefined vpColor::green color. */
    id_darkGreen,  /*!< Identifier associated to the predefined vpColor::darkGreen color. */
    id_lightBlue,  /*!< Identifier associated to the predefined vpColor::lightBlue color. */
    id_blue,       /*!< Identifier associated to the predefined vpColor::blue color. */
    id_darkBlue,   /*!< Identifier associated to the predefined vpColor::darkBlue color. */
    id_yellow,     /*!< Identifier associated to the predefined vpColor::yellow color. */
    id_cyan,       /*!< Identifier associated to the predefined vpColor::cyan color. */
    id_orange,     /*!< Identifier associated to the predefined vpColor::orange color. */
    id_purple,     /*!< Identifier associated to the predefined vpColor::purple color. */

    id_unknown   /*!< Identifier associated to unknowned
                    colors. By unknowned, we mean not a predefined
                    color. This identifier can also be used to know
                    the number of predefined colors. */

  } vpColorIdentifier;

  vpColorIdentifier id; /*!< Color identifier to indicate if a color
                           is predefined or set by the user using its
                           RGB values. */

  /* Predefined colors. */
  static const vpColor black ;
  static const vpColor  white ;
  static const vpColor  lightGray;
  static const vpColor  gray;
  static const vpColor  darkGray;
  static const vpColor  lightRed;
  static const vpColor  red ;
  static const vpColor  darkRed;
  static const vpColor  lightGreen;
  static const vpColor  green;
  static const vpColor  darkGreen;
  static const vpColor  lightBlue;
  static const vpColor  blue ;  
  static const vpColor  darkBlue;
  static const vpColor  yellow ;
  static const vpColor  cyan;   
  static const vpColor  orange;
  static const vpColor  purple;
  static const vpColor  none;

  static const unsigned int nbColors;
  static const vpColor allColors[];

  /*!
    Default constructor. All the colors components are set to zero. 

    The color identifier is set to vpColor::id_unknown to indicate
    that this color is not a predefined one.

  */  
  inline vpColor() 
    : vpRGBa(), id(id_unknown)
  {};
  /*!
    Construct a color from its RGB values.
    
    \param r : Red component.
    \param g : Green component.
    \param b : Blue component.

    \param cid : The color identifier to indicate if this color is or
    not a predefined one.
  */  
  inline vpColor(unsigned char r, unsigned char g, unsigned char b,
     vpColor::vpColorIdentifier cid=vpColor::id_unknown)
    : vpRGBa(r, g, b), id(cid)
  {};

  friend VISP_EXPORT bool operator==( const vpColor &c1, const vpColor &c2 );
  friend VISP_EXPORT bool operator!=( const vpColor &c1, const vpColor &c2 );
  /*!
    Set a color from its RGB values.
    
    \param r : Red component.
    \param g : Green component.
    \param b : Blue component.

    The color identifier is set to vpColor::id_unknown to indicate
    that this color is not a predefined one.

  */  
  inline void setColor(unsigned char r, unsigned char g, unsigned char b) {
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
  static inline vpColor getColor(const unsigned int &i) {
    return vpColor::allColors[i % vpColor::nbColors];
  };

};

#endif
