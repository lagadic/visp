/****************************************************************************
 *
 * $Id: vpColor.h,v 1.6 2008-09-26 15:20:54 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
  d->init(I);

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
    id_black=0,  /*!< Identifier associated to the predefined vpColor::black color. */
    id_white,    /*!< Identifier associated to the predefined vpColor::white color. */
    id_red,      /*!< Identifier associated to the predefined vpColor::red color. */
    id_green,    /*!< Identifier associated to the predefined vpColor::green color. */
    id_blue,     /*!< Identifier associated to the predefined vpColor::blue color. */
    id_yellow,   /*!< Identifier associated to the predefined vpColor::yellow color. */
    id_cyan,     /*!< Identifier associated to the predefined vpColor::cyan color. */
    id_orange,   /*!< Identifier associated to the predefined vpColor::orange color. */

    id_unknown   /*!< Identifier associated to unknowned
                    colors. By unknowned, we mean not a predefined
                    color. This identifier can also be used to know
                    the number of predefined colors. */

  } vpColorIdentifier;

  vpColorIdentifier id; /*!< Color identifier to indicate if a color
                           is predefined or set by the user using its
                           RGB values. */

  /* Predefined colors. */
  static const vpColor 
    black,  
    white,  
    red,    
    green,  
    blue,   
    yellow, 
    cyan,   
    orange, 
    none;   

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
    
    \param R : Red component.
    \param G : Green component.
    \param B : Blue component.

    \param id : The color identifier to indicate if this color is or
    not a predefined one.
  */  
  inline vpColor(unsigned char R, unsigned char G, unsigned char B, 
		 vpColor::vpColorIdentifier id=vpColor::id_unknown) 
    : vpRGBa(R, G, B), id(id)
  {};

  /*!
    Set a color from its RGB values.
    
    \param R : Red component.
    \param G : Green component.
    \param B : Blue component.

    The color identifier is set to vpColor::id_unknown to indicate
    that this color is not a predefined one.

  */  
  inline void setColor(unsigned char R, unsigned char G, unsigned char B) {
    this->R = R;
    this->G = G;
    this->B = B;
    this->A = 0;
    id = id_unknown;
  };
};

/*!
  Compare two colors.

  Return true if the R,G,B components are the same.

  \param c1,c2 : Color to compare.
*/
VISP_EXPORT inline bool operator==( const vpColor &c1, const vpColor &c2 ) {
  return ( ( c1.R == c2.R ) && ( c1.G == c2.G ) && ( c1.B == c2.B) );
}
 
/*!

  Compare two colors.

  Return true if the R,G,B components are different.

  \param c1,c2 : Color to compare.
*/
VISP_EXPORT inline bool operator!=( const vpColor &c1, const vpColor &c2 ) {
  return ( ( c1.R != c2.R ) || ( c1.G != c2.G ) || ( c1.B == c2.B) );
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
