/****************************************************************************
 *
 * $Id: vpRGBa.cpp,v 1.2 2006-05-30 08:40:43 fspindle Exp $
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

/*!
  \class vpRGBa

  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes


  Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)

  \sa vpImage
*/

/*!
  \brief basic constructor

  build a black value
*/
vpRGBa::vpRGBa()
{
  R=0 ;
  G=0 ;
  B=0 ;
  A=0 ;
}

/*!
  \brief constructor

  initilialization with R1, G1, B1
*/
vpRGBa::vpRGBa(unsigned char R1, unsigned char G1, unsigned char B1, unsigned char  A1)
{
  R=R1 ;
  G=G1 ;
  B=B1 ;
  A=A1 ;
}

/*!
  \brief constructor

  initilialization from a color identifier

  are allowd the following identifier
    RED, GREEN,  BLUE, WHITE, BLACK, NOIR, YELLOW, CYAN

  \sa defs_color.h

*/
vpRGBa::vpRGBa(int color)
{
  R = G = B = A = (unsigned char)0 ;
  switch(color)
  {
  case red   :
    R = (unsigned char)255 ;
    break ;
  case green :
    G = (unsigned char)255 ;
    break ;
  case blue :
    B = (unsigned char)255 ;
    break ;
  case white :
    R = G = B = (unsigned char)255 ;
    break ;
  case black :
    R = G = B = (unsigned char)0 ;
    break ;
  case yellow :
    R = G = (unsigned char)255 ;
    B = (unsigned char)0 ;
    break ;
  case cyan  :
    G = B = (unsigned char)255 ;
    R = (unsigned char)0 ;
    break ;
  }
}
/*!
  \brief Copy constructor
*/
vpRGBa::vpRGBa(const vpRGBa &c)
{
  *this = c ;
}

/*!
  \brief copy operator (from an unsigned char value)

  \param x : input color ( R = G = B = x )
*/
void
vpRGBa::operator=(const unsigned char x)
{
  R = x ; G = x ;  B = x ;
}

/*!
  \brief copy operator
*/
void
vpRGBa::operator=(const vpRGBa &c)
{
  R = c.R ;
  G = c.G ;
  B = c.B ;
  A = c.A ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
