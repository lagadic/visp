/****************************************************************************
 *
 * $Id: vpRGBa.h,v 1.8 2008-09-26 15:20:54 fspindle Exp $
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
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpRGBa_h
#define vpRGBa_h

/*!
  \file vpRGBa.h
  \brief Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)
*/

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>


/*!
  \class vpRGBa

  \ingroup ImageContainer

  \brief Class that defines a RGB 32 bits structure.

  Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)

  \sa vpImage
*/
class VISP_EXPORT vpRGBa
{
public:
  /*!
    Basic constructor.
    
    Build a black value.
    
  */
  inline vpRGBa() 
    : R(0), G(0), B(0), A(0) 
    {
    };
  
  /*!
    Constructor.
    
    Initialize the color with R, G, B, A values.
    
    \param R : Red value.
    \param G : Green value.
    \param B : Blue value.
    \param A : Additional value.
    
  */
  inline vpRGBa(const unsigned char &R, const unsigned char &G,
		const unsigned char &B, const unsigned char &A=0) 
    : R(R), G(G), B(B), A(A)
  {
  };


  /*!
    Constructor.
    
    Initialize all the R, G, B, A components to \e v.
    
    \param v : Value to set.
    
  */
  inline vpRGBa(const unsigned char &v) 
    : R(v), G(v), B(v), A(v)
  {
  };


  /*!
    Copy constructor.
  */
  inline vpRGBa(const vpRGBa &v)
  {
    *this = v ;
  };

  /*!
    Create a RGBa value from a 4 dimension column vector.

    R=v[0]
    G=v[1]
    B=v[2]
    A=v[3]
    
  */
  inline vpRGBa(const vpColVector &v)
  {
    *this = v ;
  }

  void operator=(const unsigned char &v) ;
  void operator=(const vpRGBa &v) ;
  void operator=(const vpColVector &v) ;
  vpColVector operator-(const vpRGBa &v) const;
  vpRGBa operator+(const vpRGBa &v) const;
  vpColVector operator-(const vpColVector &v) const;
  vpColVector operator+(const vpColVector &v) const;
  vpColVector operator*(const float &v) const;
  vpColVector operator*(const double &v) const;

 public:
  unsigned char R ; //!< Red component.
  unsigned char G ; //!< Green component.
  unsigned char B ; //!< Blue component.
  unsigned char A ; //!< Additionnal component.

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
