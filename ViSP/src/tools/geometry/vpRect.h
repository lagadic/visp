/****************************************************************************
 *
 * $Id: vpRect.h,v 1.4 2007-07-02 16:35:02 fspindle Exp $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
#include <visp/vpImagePoint.h>

class VISP_EXPORT vpRect
{
public:

  vpRect();
  vpRect(double left, double top, double width, double height);
  vpRect(const vpImagePoint &topLeft, double width, double height);
  vpRect(const vpImagePoint &topLeft, const vpImagePoint &bottomRight);
  vpRect(const vpRect& r);
  
  vpRect &operator=(const vpRect& r);

  /*!
    Returns the bottom coordinate of the rectangle. 
    \sa getRight()
  */
  inline double getBottom() const { return (this->top + this->height - 1.0); };
  /*!
    Returns the bottom-right coordinate of the rectangle. 
    \sa getTopLeft(), getBottom(), getRight()
  */
  inline vpImagePoint getBottomRight() const {
    vpImagePoint bottomRight;
    bottomRight.set_u( getRight() );
    bottomRight.set_v( getBottom() );

    return bottomRight;
  };
  /*!

    Returns the center point of the rectangle. The center point
    coordinates are (\e x, \e y)

    The default coordinate system has origin (0, 0) in the top-left
    corner. The positive direction of the y axis is down, and the
    positive x axis is from left to right.

    \sa moveCenter()
  */
  inline void getCenter(double & x, double & y) const { 
    x = this->left + this->width  / 2.0 - 0.5; 
    y = this->top  + this->height / 2.0 - 0.5; 
  };
  /*!

    Returns the center point of the rectangle. The center point
    coordinates are (\e x, \e y)

    The default coordinate system has origin (0, 0) in the top-left
    corner. The positive direction of the y axis is down, and the
    positive x axis is from left to right.

    \sa moveCenter()
  */
  inline vpImagePoint getCenter() const { 
    vpImagePoint center;
    center.set_u( this->left + this->width  / 2.0 - 0.5 ); 
    center.set_v( this->top  + this->height / 2.0 - 0.5 ); 
    return center;
  };
  /*!
 
   Returns the height of the rectangle. The height includes both the
   top and bottom edges, i.e. height = bottom - top + 1.

   \sa getWidth()

  */
  inline double getHeight() const { return this->height; };
  /*!
    Returns the left coordinate of the rectangle. 

    \sa getTopLeft(), getRight()
  */
  inline double getLeft() const { return this->left;   };

  /*!
    Returns the right coordinate of the rectangle. 
    \sa getLeft()
  */
  inline double getRight() const { return (this->left + this->width - 1.0); };

  /*!
    Returns the top coordinate of the rectangle. 

    \sa getTopLeft(), getBottom()
  */
  inline double getTop() const { return this->top;  };
  /*!
    Returns the top-left position of the rectangle. 

    \sa getBottomRight(), getTop(), getLeft()
  */
  inline vpImagePoint getTopLeft() const { 
    vpImagePoint topLeft;
    topLeft.set_u( this->left );
    topLeft.set_v( this->top );
    return topLeft; 
  };
  /*!
   Returns the width of the rectangle. The width includes both the
   left and right edges, i.e. width = right - left + 1.
   
   \sa getHeight()

  */
  inline double getWidth() const { return this->width;  };

  /*!

    Sets the bottom edge position of the rectangle to pos. May change
    the height of the rectangle, but will never change the top edge
    position the rectangle.

    \sa setTop()
  */
  inline void setBottom(double pos) { this->height = pos - this->top + 1.0; };
  /*!

    Sets the bottom-right position of the rectangle. Will never change
    the top-left position the rectangle.

    \sa setTopLeft()
  */
  inline void setBottomRight(const vpImagePoint &bottomRight) {
    this->height = bottomRight.get_v() - this->top + 1.0; 
    this->width = bottomRight.get_u() - this->left + 1.0;
  };
  /*!

    Sets the height of the rectangle to \e h. The top edge is not moved,
    but the bottom edge may be moved.

    \sa setWidth()
  */
  inline void setHeight(double h) { this->height = h; };
  /*!

    Sets the left edge position of the rectangle to pos. May change the right
    edge  position of the rectangle, but will never change the width of the
    rectangle.

    \sa setRight()

  */
  inline void setLeft(double pos) { this->left = pos; };
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

    Sets the right edge position of the rectangle to pos. May change
    the width of the rectangle, but will never change the left edge
    position of the rectangle.

    \sa setLeft()

  */
  inline void setRight(double pos) { this->width = pos - this->left + 1.0; };
  /*!

    Sets the top edge position of the rectangle to pos. May change the bottom
    edge position of the rectangle, but will never change the height of the
    rectangle.

    \sa setBottom()
  */
  inline void setTop(double pos) { this->top = pos; };
  /*!

    Sets the top-left position of the rectangle. May change the bottom
    edge position of the rectangle, but will never change the height of the
    rectangle.

    \sa setBottomRight()
  */
  inline void setTopLeft(const vpImagePoint &topLeft) { 
    this->left = topLeft.get_u(); 
    this->top  = topLeft.get_v(); 
  };
  /*!

    Sets the width of the rectangle to \e w. The right edge is changed,
    but not the left edge.

    \sa setHeight()
  */
  inline void setWidth(double w) { this->width = w; }
 
  /*!

    Sets the center point of the rectangle to (\e x, \e y), leaving
    the size unchanged.

    \sa getCenter()

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


/*!

  Check if an image point belongs to a rectangle.
  
  \param ip : the image point.
  \param rect : the rectangle.
  
  \return Returns true if the point belongs to the rectangle.

*/
VISP_EXPORT inline bool inRectangle( const vpImagePoint &ip, const vpRect &rect ) {
  return ( ip.get_i() <= rect.getBottom() && ip.get_i() >= rect.getTop() && ip.get_j() <= rect.getRight() && ip.get_j() >= rect.getLeft());
}


#endif
