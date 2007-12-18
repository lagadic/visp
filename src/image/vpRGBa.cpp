/****************************************************************************
 *
 * $Id: vpRGBa.cpp,v 1.4 2007-12-18 14:34:47 fspindle Exp $
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
 * RGBA pixel.
 *
 * Authors:
 * Eric Marchand
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
  Basic constructor.

  Build a black value.

*/
vpRGBa::vpRGBa()
{
  R=0 ;
  G=0 ;
  B=0 ;
  A=0 ;
}

/*!
  Constructor.

  Initilialize the color with R, G, B, A values.

  \param R : Red value.
  \param G : Green value.
  \param B : Blue value.
  \param A : Additional value.

*/
vpRGBa::vpRGBa(const unsigned char &R, const unsigned char &G, 
	       const unsigned char &B, const unsigned char &A)
{
  this->R = R;
  this->G = G;
  this->B = B;
  this->A = A;
}

/*!
  Constructor.

  Initilialization from a color identifier.

  \param v : Color to initialize.

*/
vpRGBa::vpRGBa(const vpColorType &v)
{
  R = G = B = A = (unsigned char)0 ;
  switch(v)
  {
  case none:
  case black:
    R = G = B = (unsigned char)0 ;
    break ;
  case white:
    R = G = B = (unsigned char)255;
    break ;
  case red   :
    R = (unsigned char)255 ;
    break ;
  case green :
    G = (unsigned char)255 ;
    break ;
  case blue :
    B = (unsigned char)255 ;
    break ;
  case yellow :
    R = G = (unsigned char)255 ;
    B = (unsigned char)0 ;
    break ;
  case cyan  :
    G = B = (unsigned char)255 ;
    R = (unsigned char)0 ;
    break ;
  case orange  :
    R = (unsigned char)244 ;
    R = (unsigned char)121 ;
    R = (unsigned char)7 ;
    break ;
  }
}
/*!
  Copy constructor.
*/
vpRGBa::vpRGBa(const vpRGBa &v)
{
  *this = v ;
}

/*!
  Copy constructor.
*/
vpRGBa::vpRGBa(const vpColVector &v)
{
  *this = v ;
}

/*!
  Copy operator (from an unsigned char value)

  \param v : Input color ( R = G = B = v )
*/
void
vpRGBa::operator=(const unsigned char &v)
{
  R = v ; G = v ;  B = v ; A = v;
}

/*!
  Copy operator.
*/
void
vpRGBa::operator=(const vpRGBa &v)
{
  R = v.R ;
  G = v.G ;
  B = v.B ;
  A = v.A ;
}

/*!
  \brief Cast a vpColVector in a vpRGBa

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
