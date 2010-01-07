/****************************************************************************
 *
 * $Id$
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
 * RGBA pixel.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpRGBa.cpp
  \brief Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used - yet -)
*/


#include <visp/vpRGBa.h>
#include <visp/vpColor.h>
#include <visp/vpDebug.h>
#include <visp/vpException.h>


/*!
  Copy operator (from an unsigned char value)

  \param v : Input color ( R = G = B = v )
*/
void
vpRGBa::operator=(const unsigned char &v)
{
  this->R = v;
  this->G = v;
  this->B = v;
  this->A = v;
}

/*!
  Copy operator.
*/
void
vpRGBa::operator=(const vpRGBa &v)
{
  this->R = v.R;
  this->G = v.G;
  this->B = v.B;
  this->A = v.A;
}

/*!
  Cast a vpColVector in a vpRGBa

  \param v : Input vector. v[0], v[1], v[2], v[3] are to make into
  relation with respectively R, G, B and A.

  \exception vpException::dimensionError : If v is not a 4 four
  dimention vector.
*/
void
vpRGBa::operator=(const vpColVector &v)
{
  if (v.getRows() != 4) {
    vpERROR_TRACE("Bad vector dimension ") ;
    throw(vpException(vpException::dimensionError, "Bad vector dimension "));
  }
  R = (unsigned char)v[0];
  G = (unsigned char)v[1];
  B = (unsigned char)v[2];
  A = (unsigned char)v[3];
}

/*!
  Substraction operator : "this" - v.
  \param v : Color to substract to the current object "this".
  \return "this" - v
*/
vpColVector
vpRGBa::operator-(const vpRGBa &v) const
{
  vpColVector n(4); // new color
  n[0] = (double)R - (double)v.R;
  n[1] = (double)G - (double)v.G;
  n[2] = (double)B - (double)v.B;
  n[3] = (double)A - (double)v.A;
  return n;
}

/*!
  Addition operator : "this" + v.
  \param v : Color to add to the current object "this".
  \return "this" + v
*/
vpRGBa
vpRGBa::operator+(const vpRGBa &v) const
{
  vpRGBa n; // new color
  n.R = R + v.R ;
  n.G = G + v.G ;
  n.B = B + v.B ;
  n.A = A + v.A ;
  return n;
}

/*!
  Substraction operator : "this" - v.
  \param v : Color to substract to the current object "this".
  \return "this" - v
*/
vpColVector
vpRGBa::operator-(const vpColVector &v) const
{
  vpColVector n(4); // new color
  n[0] = R - v[0];
  n[1] = G - v[1];
  n[2] = B - v[2];
  n[3] = A - v[3];
  return n;
}

/*!
  Addition operator : "this" + v.
  \param v : Color to add to the current object "this".
  \return "this" + v
*/
vpColVector
vpRGBa::operator+(const vpColVector &v) const
{
  vpColVector n(4); // new color
  n[0] = R + v[0];
  n[1] = G + v[1];
  n[2] = B + v[2];
  n[3] = A + v[3];
  return n;
}

/*!
  Multiplication operator : v * "this".
  \param v : Value to multiply.
  \return v * "this"
*/
vpColVector
vpRGBa::operator*(const float &v) const
{
  vpColVector n(4);
  n[0] = R * v;
  n[1] = G * v;
  n[2] = B * v;
  n[3] = A * v;
  return n;
}

/*!
  Multiplication operator : v * "this".
  \param v : Value to multiply.
  \return v * "this"
*/
vpColVector
vpRGBa::operator*(const double &v) const
{
  vpColVector n(4);
  n[0] = R * v;
  n[1] = G * v;
  n[2] = B * v;
  n[3] = A * v;
  return n;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
