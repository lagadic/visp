/****************************************************************************
 *
 * $Id: vpRect.h,v 1.4 2007-07-02 16:35:02 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Defines a rectangle in the plane.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpRect_h
#define vpRect_h




/*!
  \class vpRect

  \brief Defines a rectangle in the plane.

  A rectangle is internally represented as an upper-left corner and a
  width and height, but it is normally expressed as an upper-left
  corner and bottom-right corner.

  Note that the size (width and height) of a rectangle might be
  different from what you are used to. If the top-left corner and the
  bottom-right corner are the same, the height and the width of the
  rectangle will both be 1.

  Generally, width = right - left + 1 and height = bottom - top +
  1. We designed it this way to make it correspond to rectangular
  spaces used by drawing functions in which the width and height
  denote a number of pixels. For example, drawing a rectangle with
  width and height 1 draws a single pixel.

  The default coordinate system has origin (0, 0) in the top-left
  corner. The positive direction of the y axis is down, and the
  positive x axis is from left to right.

  A vpRect can be constructed with a set of left, top, width and
  height double. After creation the dimensions can be changed,
  e.g. with setLeft(), setRight(), setTop() and setBottom(), or by
  setting sizes, e.g. setWidth(), setHeight()

*/

#include <visp/vpConfig.h>

class VISP_EXPORT vpRect
{
public:

  vpRect();
  vpRect(double left, double top, double width, double height);
  vpRect(const vpRect& r);
  
  vpRect &operator=(const vpRect& r);

  /*!
    Returns the left coordinate of the rectangle. 
  */
  inline double getLeft() const { return this->left;   };

  /*!
    Returns the right coordinate of the rectangle. 
  */
  inline double getRight() const { return (this->left + this->width - 1.0); };

  /*!
    Returns the top coordinate of the rectangle. 

  */
  inline double getTop() const { return this->top;  };
  /*!
    Returns the bottom coordinate of the rectangle. 
  */
  inline double getBottom() const { return (this->top + this->height - 1.0); };
  /*!
   Returns the width of the rectangle. The width includes both the
   left and right edges, i.e. width = right - left + 1.

  */
  inline double getWidth() const { return this->width;  };
  /*!
 
   Returns the height of the rectangle. The height includes both the
   top and bottom edges, i.e. height = bottom - top + 1.

  */
  inline double getHeight() const { return this->height; };

  /*!

    Sets the coordinates of the rectangle's top left corner to
    (left, top), and its size to (width, height).

  */
  inline void setRect(double left, double top, double width, double height) {
     this->left   = left; 
     this->top    = top; 
     this->width  = width;
     this->height = height;
  };
  /*!

    Sets the left edge position of the rectangle to pos. May change the right
    edge  position of the rectangle, but will never change the width of the
    rectangle.

  */
  inline void setLeft(double pos) { this->left = pos; };
  /*!

    Sets the right edge position of the rectangle to pos. May change
    the width of the rectangle, but will never change the left edge
    position of the rectangle.

  */
  inline void setRight(double pos) { this->width = pos - this->left + 1.0; };
  /*!

    Sets the top edge position of the rectangle to pos. May change the bottom
    edge position of the rectangle, but will never change the height of the
    rectangle.

  */
  inline void setTop(double pos) { this->top = pos; };
  /*!

    Sets the bottom edge position of the rectangle to pos. May change
    the height of the rectangle, but will never change the top edge
    position the rectangle.

  */
  inline void setBottom(double pos) { this->height = pos - this->top + 1.0; };
  /*!

    Sets the width of the rectangle to \e w. The right edge is changed,
    but not the left edge.

  */
  inline void setWidth(double w) { this->width = w; };
  /*!

    Sets the height of the rectangle to \e h. The top edge is not moved,
    but the bottom edge may be moved.

  */
  inline void setHeight(double h) { this->height = h; };
 
  /*!

    Returns the center point of the rectangle. The center point
    coordinates are (\e x, \e y)

    The default coordinate system has origin (0, 0) in the top-left
    corner. The positive direction of the y axis is down, and the
    positive x axis is from left to right.

  */
  inline void getCenter(double & x, double & y) const { 
    x = this->left + this->width  / 2.0 - 0.5; 
    y = this->top  + this->height / 2.0 - 0.5; 
  };
 
  /*!

    Sets the center point of the rectangle to (\e x, \e y), leaving
    the size unchanged.

  */
  inline void moveCenter(double x, double y) {
    this->left = x - this->width/2  + 0.5;
    this->top  = y - this->height/2 + 0.5;
  };
  
private:
  double left;   // Upper left corner position along the columns axis
  double top;    // Upper left corner position along the rows axis
  double width;  // Rectangle width
  double height; // Rectangle height
};


#endif
