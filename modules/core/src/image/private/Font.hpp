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
 * Point and Rectangle structures for text drawing.
 *
 *****************************************************************************/
// Contains code from:
/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2017 Yermalayeu Ihar.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef _Font_hpp_
#define _Font_hpp_

#include <visp3/core/vpImageTools.h>

namespace Font
{
template <typename T>
struct Point
{
  typedef T Type; /*!< Type definition. */

  T x; /*!< \brief Specifies the x-coordinate of a point. */
  T y; /*!< \brief Specifies the y-coordinate of a point. */

  /*!
    Creates a new Point structure that contains the default (0, 0) coordinates.
  */
  Point();

  /*!
    Creates a new Point structure that contains the specified coordinates.

    \param [in] tx - initial X value.
    \param [in] ty - initial Y value.
  */
  template <typename TX, typename TY> Point(TX tx, TY ty);

  /*!
    Creates a new Point structure on the base of another point of arbitrary type.

    \param [in] p - a point of arbitrary type.
  */
  template <class TP, template<class> class TPoint> Point(const TPoint<TP> & p);

  /*!
    Performs shift bit left for value of point coordinates.

    \note It function is actual for integer types of Point.

    \param [in] shift - a shift value.
    \return a new point with shifted coordinates.
  */
  Point operator << (int shift) const;
};

inline int Round(double value)
{
  return (int)(value + (value >= 0 ? 0.5 : -0.5));
}

template <class TD, class TS>
inline TD Convert(TS src)
{
  return (TD)src;
}

template <>
inline int Convert<int, double>(double src)
{
  return Round(src);
}

template <>
inline int Convert<int, float>(float src)
{
  return Round(src);
}

template <typename T>
inline Point<T>::Point()
  : x(0)
  , y(0)
{
}

template <typename T> template <typename TX, typename TY>
inline Point<T>::Point(TX tx, TY ty)
  : x(Convert<T, TX>(tx))
  , y(Convert<T, TY>(ty))
{
}

template <typename T> template <class TP, template<class> class TPoint>
inline Point<T>::Point(const TPoint<TP> & p)
  : x(Convert<T, TP>(p.x))
  , y(Convert<T, TP>(p.y))
{
}

template <typename T>
inline Point<T> Point<T>::operator << (int shift) const
{
  return Point<T>(x << shift, y << shift);
}

// Point<T> utilities implementation:

template <typename T>
inline Point<T> operator + (const Point<T> & p1, const Point<T> & p2)
{
  return Point<T>(p1.x + p2.x, p1.y + p2.y);
}

template <typename T>
inline Point<T> operator - (const Point<T> & p)
{
  return Point<T>(-p.x, -p.y);
}

template <typename TP, typename TA>
inline Point<TP> operator * (const TA & a, const Point<TP> & p)
{
  return Point<TP>(p.x*a, p.y*a);
}

template <typename T>
struct Rectangle
{
  typedef T Type; /*!< Type definition. */

  T left;   /*!< \brief Specifies the position of left side of a rectangle. */
  T top;    /*!< \brief Specifies the position of top side of a rectangle. */
  T right;  /*!< \brief Specifies the position of right side of a rectangle. */
  T bottom; /*!< \brief Specifies the position of bottom side of a rectangle. */

  /*!
    Creates a new Rectangle structure that contains the default (0, 0, 0, 0) positions of its sides.
  */
  Rectangle();

  /*!
    Creates a new Rectangle structure that contains the specified positions of its sides.

    \param [in] l - initial left value.
    \param [in] t - initial top value.
    \param [in] r - initial right value.
    \param [in] b - initial bottom value.
  */
  template <typename TL, typename TT, typename TR, typename TB> Rectangle(TL l, TT t, TR r, TB b);

  /*!
    Creates a new Rectangular structure that contains the specified coordinates of its left-top and right-bottom corners.

    \param [in] lt - initial coordinates of left-top corner.
    \param [in] rb - initial coordinates of right-bottom corner.
  */
  template <typename TLT, typename TRB> Rectangle(const Point<TLT> & lt, const Point<TRB> & rb);

  /*!
    Creates a new Rectangular structure that contains the specified coordinates of its right-bottom corner.
    The coordinates of left-top corner is set to (0, 0).

    \param [in] rb - initial coordinates of right-bottom corner.
  */
  template <typename TRB> Rectangle(const Point<TRB> & rb);

  /*!
    Creates a new Rectangle structure on the base of another rectangle of arbitrary type.

    \param [in] r - a rectangle of arbitrary type.
  */
  template <class TR, template<class> class TRectangle> Rectangle(const TRectangle<TR> & r);

  /*!
    Converts itself to rectangle of arbitrary type.

    \return a rectangle of arbitrary type.
  */
  template <class TR, template<class> class TRectangle> operator TRectangle<TR>() const;

  /*!
    Performs copying from rectangle of arbitrary type.

    \param [in] r - a rectangle of arbitrary type.
    \return a reference to itself.
  */
  template <typename TR> Rectangle<T> & operator = (const Rectangle<TR> & r);

  /*!
    Gets position of left side.

    \return a position of left side.
  */
  T Left() const;

  /*!
    Gets position of top side.

    \return a position of top side.
  */
  T Top() const;

  /*!
    Gets position of right side.

    \return a position of right side.
  */
  T Right() const;

  /*!
    Gets position of bottom side.

    \return a position of bottom side.
  */
  T Bottom() const;

  /*!
    Gets coordinates of top-left corner.

    \return a point with coordinates of top-left corner.
  */
  Point<T> TopLeft() const;

  /*!
    Gets coordinates of top-right corner.

    \return a point with coordinates of top-right corner.
  */
  Point<T> TopRight() const;

  /*!
    Gets coordinates of bottom-left corner.

    \return a point with coordinates of bottom-left corner.
  */
  Point<T> BottomLeft() const;

  /*!
    Gets coordinates of bottom-right corner.

    \return a point with coordinates of bottom-right corner.
  */
  Point<T> BottomRight() const;

  /*!
    Gets rectangle width.

    \return a rectangle width.
  */
  T Width() const;

  /*!
    Gets rectangle height.

    \return a rectangle height.
  */
  T Height() const;

  /*!
    Gets rectangle area.

    \return a rectangle area.
  */
  T Area() const;

  /*!
    Returns true if rectangle area is equal to zero.

    \return a boolean value.
  */
  bool Empty() const;

  /*!
    Gets size (width and height) of the rectangle.

    \return a point with rectangle size.
  */
  Point<T> Size() const;

  /*!
    Checks on the rectangle to belonging to the rectangle.

    \param [in] r - a checked rectangle.
    \return a result of checking.
  */
  template <typename TR> bool Contains(const Rectangle <TR> & r) const;

  /*!
    Shifts a rectangle on the specific value.

    \param [in] shift - a point with shift value.
    \return a reference to itself.
  */
  template <typename TP> Rectangle<T> & Shift(const Point<TP> & shift);

  /*!
    Shifts a rectangle on the specific value.

    \param [in] shiftX - x-coordinate of the shift.
    \param [in] shiftY - y-coordinate of the shift.
    \return a reference to itself.
  */
  template <typename TX, typename TY> Rectangle<T> & Shift(TX shiftX, TY shiftY);

  /*!
    Gets a rectangle with shifted coordinates.

    \param [in] shift - a point with shift value.
    \return a shifted rectangle.
  */
  template <typename TP> Rectangle<T> Shifted(const Point<TP> & shift) const;

  /*!
    Gets a rectangle with shifted coordinates.

    \param [in] shiftX - x-coordinate of the shift.
    \param [in] shiftY - y-coordinate of the shift.
    \return a shifted rectangle.
  */
  template <typename TX, typename TY> Rectangle<T> Shifted(TX shiftX, TY shiftY) const;

  /*!
    Gets an intersection of the two rectangles (current and specified).

    \param [in] r - specified rectangle.
    \return a rectangle with result of intersection.
  */
  template <typename TR> Rectangle<T> Intersection(const Rectangle<TR> & r) const;

  /*!
    Sets to the rectangle results of the intersection of the rectangle and specified rectangle.

    \param [in] r - specified rectangle.
    \return a reference to itself.
  */
  template <typename TR> Rectangle<T> & operator &= (const Rectangle<TR> & r);

  /*!
    Sets to the rectangle results of the union of the rectangle and specified rectangle.

    \param [in] r - specified rectangle.
    \return a reference to itself.
  */
  template <typename TR> Rectangle<T> & operator |= (const Rectangle<TR> & r);
};

// struct Rectangle<T> implementation:

template <typename T>
inline Rectangle<T>::Rectangle()
  : left(0)
  , top(0)
  , right(0)
  , bottom(0)
{
}

template <typename T> template <typename TL, typename TT, typename TR, typename TB>
inline Rectangle<T>::Rectangle(TL l, TT t, TR r, TB b)
  : left(Convert<T, TL>(l))
  , top(Convert<T, TT>(t))
  , right(Convert<T, TR>(r))
  , bottom(Convert<T, TB>(b))
{
}

template <typename T> template <typename TLT, typename TRB>
inline Rectangle<T>::Rectangle(const Point<TLT> & lt, const Point<TRB> & rb)
  : left(Convert<T, TLT>(lt.x))
  , top(Convert<T, TLT>(lt.y))
  , right(Convert<T, TRB>(rb.x))
  , bottom(Convert<T, TRB>(rb.y))
{
}

template <typename T> template <typename TRB>
inline Rectangle<T>::Rectangle(const Point<TRB> & rb)
  : left(0)
  , top(0)
  , right(Convert<T, TRB>(rb.x))
  , bottom(Convert<T, TRB>(rb.y))
{
}

template <typename T> template <class TR, template<class> class TRectangle>
inline Rectangle<T>::Rectangle(const TRectangle<TR> & r)
  : left(Convert<T, TR>(r.left))
  , top(Convert<T, TR>(r.top))
  , right(Convert<T, TR>(r.right))
  , bottom(Convert<T, TR>(r.bottom))
{
}

template <typename T> template <class TR, template<class> class TRectangle>
inline Rectangle<T>::operator TRectangle<TR>() const
{
  return TRectangle<TR>(Convert<TR, T>(left), Convert<TR, T>(top),
                        Convert<TR, T>(right), Convert<TR, T>(bottom));
}

template <typename T> template <typename TR>
inline Rectangle<T> & Rectangle<T>::operator = (const Rectangle<TR> & r)
{
  left = Convert<T, TR>(r.left);
  top = Convert<T, TR>(r.top);
  right = Convert<T, TR>(r.right);
  bottom = Convert<T, TR>(r.bottom);
  return *this;
}

template <typename T>
inline T Rectangle<T>::Left() const
{
  return left;
}

template <typename T>
inline T Rectangle<T>::Top() const
{
  return top;
}

template <typename T>
inline T Rectangle<T>::Right() const
{
  return right;
}

template <typename T>
inline T Rectangle<T>::Bottom() const
{
  return bottom;
}

template <typename T>
inline Point<T> Rectangle<T>::TopLeft() const
{
  return Point<T>(left, top);
}

template <typename T>
inline Point<T> Rectangle<T>::TopRight() const
{
  return Point<T>(right, top);
}

template <typename T>
inline Point<T> Rectangle<T>::BottomLeft() const
{
  return Point<T>(left, bottom);
}

template <typename T>
inline Point<T> Rectangle<T>::BottomRight() const
{
  return Point<T>(right, bottom);
}

template <typename T>
inline T Rectangle<T>::Width() const
{
  return right - left;
}

template <typename T>
inline T Rectangle<T>::Height() const
{
  return bottom - top;
}

template <typename T>
inline T Rectangle<T>::Area() const
{
  return Width()*Height();
}

template <typename T>
inline bool Rectangle<T>::Empty() const
{
  return Area() == 0;
}

template <typename T>
inline Point<T> Rectangle<T>::Size() const
{
  return Point<T>(Width(), Height());
}

template <typename T> template <typename TR>
inline bool Rectangle<T>::Contains(const Rectangle <TR> & r) const
{
  return Contains(r.left, r.top, r.right, r.bottom);
}

template <typename T> template <typename TP>
inline Rectangle<T> & Rectangle<T>::Shift(const Point<TP> & shift)
{
  return Shift(shift.x, shift.y);
}

template <typename T> template <typename TX, typename TY>
inline Rectangle<T> & Rectangle<T>::Shift(TX shiftX, TY shiftY)
{
  Point<T> shift(shiftX, shiftY);
  left += shift.x;
  top += shift.y;
  right += shift.x;
  bottom += shift.y;
  return *this;
}

template <typename T> template <typename TP>
inline Rectangle<T> Rectangle<T>::Shifted(const Point<TP> & shift) const
{
  return Shifted(shift.x, shift.y);
}

template <typename T> template <typename TX, typename TY>
inline Rectangle<T> Rectangle<T>::Shifted(TX shiftX, TY shiftY) const
{
  Point<T> shift(shiftX, shiftY);
  return Rectangle<T>(left + shift.x, top + shift.y, right + shift.x, bottom + shift.y);
}

template <typename T> template <typename TR>
inline Rectangle<T> Rectangle<T>::Intersection(const Rectangle<TR> & rect) const
{
  Rectangle<T> _rect(rect);
  T l = std::max(left, _rect.left);
  T t = std::max(top, _rect.top);
  T r = std::max(l, std::min(right, _rect.right));
  T b = std::max(t, std::min(bottom, _rect.bottom));
  return Rectangle(l, t, r, b);
}

template <typename T> template <typename TR>
inline Rectangle<T> & Rectangle<T>::operator &= (const Rectangle<TR> & r)
{
  if (Empty())
    return *this;
  if (r.Empty())
    return this->operator=(r);

  Rectangle<T> _r(r);
  if (left < _r.left)
    left = std::min(_r.left, right);
  if (top < _r.top)
    top = std::min(_r.top, bottom);
  if (right > _r.right)
    right = std::max(_r.right, left);
  if (bottom > _r.bottom)
    bottom = std::max(_r.bottom, top);
  return *this;
}

template <typename T> template <typename TR>
inline Rectangle<T> & Rectangle<T>::operator |= (const Rectangle<TR> & r)
{
  if (Empty())
    return this->operator=(r);
  if (r.Empty())
    return *this;

  Rectangle<T> _r(r);
  left = std::min(left, _r.left);
  top = std::min(top, _r.top);
  right = std::max(right, _r.right);
  bottom = std::max(bottom, _r.bottom);
  return *this;
}
}

#endif
