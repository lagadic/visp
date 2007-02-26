/****************************************************************************
 *
 * $Id: vpImage.h,v 1.9 2007-02-26 16:39:17 fspindle Exp $
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
 * Image handling.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpImage.h
  \brief  image handling
*/

#ifndef vpImage_H
#define vpImage_H


#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpImageException.h>

class vpDisplay;

/*!
  \class vpImage

  \brief  definition of the vpImage class member functions

  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  <h3> Data structure </h3>

  Each image is build using two structure (an array bitmap which size
  is [width*height]) and an array of pointer row (which size is [nrow])
  the ith element in the row array row[i] is pointer toward the ith
  "line" of the image (ie, bitmap +i*width )

  \image html image-data-structure.gif
  \image latex image-data-structure.ps  width=10cm

  Such a structure allows a fast acces to each element of the image.
  if i is the ith rows and j the jth columns the value of this pixel
  is given by I[i][j] (that is equivalent to row[i][j]).

  This is a template class, therefore the type of each  element of the
  array is not a priori defined.


*/
template<class Type>
class vpImage
{

private:
  unsigned npixels ; //<! number of pixel in the image
  unsigned width ;   //<! number of columns
  unsigned height ;   //<! number of rows

public:
  Type *bitmap ;  //!< points toward the bitmap
private:
  Type **row ;    //!< points the row pointer array
public:
  /*!
    Get the number of rows in the image.

    \return The image number of rows, or image height.

    \warning getRows() is obsolete and preserved for the moment for
    compatibility with previous releases. You should use getHeight()
    instead.

    \sa getHeight()
  */
  inline  unsigned getRows() const { return height ; }

  /*! 
    Get the number of columns in the image.

    \return The image number of column, or image width.

    \warning getCols() is obsolete and preserved for the moment for
    compatibility with previous releases. You should use getWidth()
    instead.

    \sa getHeight()
   */
  inline  unsigned getCols() const { return width ; }
  /*! 
    Get the image height.

    \return The image height.

    \sa getWidth()

  */
  inline  unsigned getHeight() const { return height; }
  /*!
    Get the image width.

    \return The image width.

    \sa getHeight()
    
  */
  inline  unsigned getWidth() const { return width; }

  /*! 

    Get the image number of pixels which corresponds to the image
    width multiplied by the image height.

    \return The image number of pixels or image size.


    \sa getWidth(), getHeight()
   */
  inline unsigned getNumberOfPixel() const{ return npixels; }

  //------------------------------------------------------------------
  //         Acces to the image


  //! operator[] allows operation like I[i] = x
  inline Type *operator[]( const unsigned n)   { return row[n];}

  //! operator[] allows operation like x = I[i]
  inline const  Type *operator[](unsigned n) const { return row[n];}

  /*!
    Get a pixel value.

    \param i, j: Pixel coordinates; i for the row position, j for the
    column position.

    \return Pixel value for pixel (i, j)

  */
  inline  Type operator()(const unsigned i, const  unsigned j)
  {
    return bitmap[i*width+j] ;
  }
  /*!
    Set a pixel value.

    \param i, j: Pixel coordinates; i for the row position, j for the
    column position.

    \param v : Value to set for pixel (i, j)

  */
  inline  void  operator()(const unsigned i, const  unsigned j, const Type &v)
  {
    bitmap[i*width+j] = v ;
  }

  //! bilinear interpolation acces
  double get(double i, double j) ;

  //------------------------------------------------------------------
  //         build the image

  //! set the size of the image
  void  resize(const unsigned height, const unsigned width) ;

  //! set the size of the image
  void init(unsigned height, unsigned width) ;
  //! set the size of the image
  void init(unsigned height, unsigned width, Type value) ;

  //! constructor  set the size of the image
  vpImage(unsigned height, unsigned width) ;
  //! constructor  set the size of the image and init all the pixel
  vpImage(unsigned height, unsigned width, Type value) ;
  //! constructor
  vpImage() ;
  //! destructor
  ~vpImage() ;
  //! destructor
  void destroy() ;


  //! copy constructor
  vpImage(const vpImage<Type>&);

  //! Copy operator
  void operator=(const vpImage<Type> &m) ;

  void operator=(const Type &x);

  //! Return the maximum value within the bitmap
  Type maxValue() const ;
  //! Return the minumum value within the bitmap
  Type minValue() const ;


  vpDisplay *display ;
  bool initDisplay  ;
} ;

#include <visp/vpImageBase.t.cpp>


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
