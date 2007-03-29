/****************************************************************************
 *
 * $Id: vpImageBase.t.cpp,v 1.9 2007-03-29 13:49:02 hatran Exp $
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
  \file vpImageBase.t.cpp
  \brief vpImage member functions dealing with basic image manipulations
  \ingroup libimage
  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/


//debug
#include <visp/vpDebug.h>

// exception
#include <visp/vpException.h>
#include <visp/vpImageException.h>

/*!
  \brief Image initialisation

  Allocate memory for an [height x width] image

  Set all the element of the bitmap to value

  \exception vpException::memoryAllocationError

  \sa vpImage::init(height, width)
*/
template<class Type>
void
vpImage<Type>::init(unsigned int height, unsigned int width, Type value)
{
  try
  {
    init(height,width) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  for (unsigned int i=0  ; i < npixels ;  i++)
    bitmap[i] = value ;
}


/*!
  \brief Image initialization

  Allocate memory for an [height x width] image

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template<class Type>
void
vpImage<Type>::init(unsigned int height, unsigned int width)
{

  if (height != this->height) {
    if (row != NULL)  {
      vpDEBUG_TRACE(10,"Destruction row[]");
      delete [] row;
      row = NULL;
    }
  }

  if ((height != this->height) || (width != this->width))
  {
    if (bitmap != NULL) {
      vpDEBUG_TRACE(10,"Destruction bitmap[]") ;
      delete [] bitmap;
      bitmap = NULL;
    }
  }



  this->width = width ;
  this->height = height ;

  npixels=width*height;


  if (bitmap == NULL)  bitmap = new  Type[npixels] ;

  //  vpERROR_TRACE("Allocate bitmap %p",bitmap) ;
  if (bitmap == NULL)
  {
        vpERROR_TRACE("cannot allocate bitmap ") ;
    throw(vpException(vpException::memoryAllocationError,
		      "cannot allocate bitmap ")) ;
  }

  if (row == NULL)  row = new  Type*[height] ;
//  vpERROR_TRACE("Allocate row %p",row) ;
  if (row == NULL)
  {
    vpERROR_TRACE("cannot allocate row ") ;
    throw(vpException(vpException::memoryAllocationError,
		      "cannot allocate row ")) ;
  }

  unsigned int i ;
  for ( i =0  ; i < height ; i++)
    row[i] = bitmap + i*width ;
}

/*!
  \brief Constructor

  Allocate memory for an [height x width] image

  Element of the bitmap are set to zero

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa vpImage::init(height, width)
*/
template<class Type>
vpImage<Type>::vpImage(unsigned int height, unsigned int width)
{
  bitmap = NULL ;
  row = NULL ;

  display =  NULL ;
  this->height = this->width = 0 ;
  initDisplay = false;

  try
  {
    init(height,width,0) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Constructor

  Allocate memory for an [height x width] image

  set all the element of the bitmap to value

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return MEMORY_FAULT if memory allocation is impossible, else OK

  \sa vpImage::init(height, width, value)
*/
template<class Type>
vpImage<Type>::vpImage (unsigned int height, unsigned int width, Type value)
{
  bitmap = NULL ;
  row = NULL ;

  display =  NULL ;
  this->height = this->width = 0 ;
  initDisplay = false;
  try
  {
    init(height,width,value) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Constructor

  No memory allocation is done

  set all the element of the bitmap to value

  \sa vpImage::resize(height, width) for memory allocation
*/
template<class Type>
vpImage<Type>::vpImage()
{
  bitmap = NULL ;
  row = NULL ;

  display =  NULL ;

  this->height = this->width = 0 ;

  initDisplay = false;

}

/*!
  \brief resize the image : Image initialization

  Allocate memory for an [height x width] image

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa init(unsigned int, unsigned int)
*/
template<class Type>
void
vpImage<Type>::resize(unsigned int height, unsigned int width)
{
  try
  {
    init(height, width) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Destructor : Memory de-allocation

  \warning does not deallocate memory for display and video

*/
template<class Type>
void
vpImage<Type>::destroy()
{
 //   vpERROR_TRACE("Deallocate ") ;


  if (bitmap!=NULL)
  {
  //  vpERROR_TRACE("Deallocate bitmap memory %p",bitmap) ;
//    vpDEBUG_TRACE(20,"Deallocate bitmap memory %p",bitmap) ;
    delete [] bitmap ;
    bitmap = NULL;
  }


  if (row!=NULL)
  {
 //   vpERROR_TRACE("Deallocate row memory %p",row) ;
//    vpDEBUG_TRACE(20,"Deallocate row memory %p",row) ;
    delete [] row ;
    row = NULL;
  }

}

/*!
  \brief Destructor : Memory de-allocation

  \warning does not deallocate memory for display and video

*/
template<class Type>
vpImage<Type>::~vpImage()
{
  destroy() ;
}



/*!
  Copy constructor
*/
template<class Type>
vpImage<Type>::vpImage(const vpImage<Type>& I)
{
  bitmap = NULL ;
  row = NULL ;
  initDisplay = false;
  try
  {
    if (I.bitmap!=NULL)
    {
      resize(I.getHeight(),I.getWidth());
      unsigned int i;
      memcpy(bitmap, I.bitmap, I.npixels*sizeof(Type)) ;
      for (i =0  ; i < this->height ; i++) row[i] = bitmap + i*this->width ;
    }
  }
  catch(vpException me)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Return the maximum value within the bitmap
*/
template<class Type>
Type vpImage<Type>::maxValue() const
{
  Type m = bitmap[0] ;
  for (unsigned int i=0 ; i < npixels ; i++)
  {
    if (bitmap[i]>m) m = bitmap[i] ;
  }
  return m ;
}

/*!
  \brief Return the minimum value within the bitmap
*/
template<class Type>
Type vpImage<Type>::minValue() const
{
  Type m =  bitmap[0];
  for (unsigned int i=0 ; i < npixels ; i++)
    if (bitmap[i]<m) m = bitmap[i] ;
  return m ;
}

/*!
  \brief Copy operator
*/
template<class Type>
void vpImage<Type>::operator=(const vpImage<Type> &m)
{
  try
  {
    resize(m.getHeight(),m.getWidth()) ;

    memcpy(bitmap, m.bitmap, m.npixels*sizeof(Type)) ;

    //for (unsigned int i=0; i<this->height; i++) row[i] = bitmap + i*this->width;
  }
  catch(vpException me)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  \brief = operator : set   all the element of the bitmap to a given  value x
   \f$ A = x <=> A[i][j] = x \f$

   \warning = must be defined for \f$ <\f$ Type \f$ > \f$
*/
template<class Type>
void vpImage<Type>::operator=(const Type &x)
{
  for (unsigned int i=0 ; i < npixels ; i++)
    bitmap[i] = x ;
}


/*!

  \brief Get the value of pixel at coordinates [i][j] using a bilinear
  interpolation.

  \param i,j : There are float value

  \return Value of the pixel obtained by bilinear interpolation.

  Bilinear interpolation explores four points neighboring the point
  (i, j), and assumes that the brightness function is bilinear in this
  neighborhood

  Bilinear interpolation is given by:

  f[i][j] = (1-t)(1-u)I[l][k] + t(1-u)I[l+1][k] + u(1-t)I[l][k+1] + t u I[l+1][k+1]

   where
      l < i < l+1 and k < j < k+1
   and
      t = i - l,
      u = j - k

  Source : W.H. Press, S.A. Teukolsky, W.T. Vetterling, and B.P. Flannery.
    Numerical Recipes in C.  Cambridge University Press, Cambridge, NY, USA,
    1992. (Section 3.6, p123)

*/

template<class Type>
double
vpImage<Type>::get(double i, double j)  // bilinear interpolation
{
  unsigned int l = (unsigned int) i ;
  unsigned int k = (unsigned int) j ;

  double t =  i - l ;
  double u =  j - k ;
  return (1-t)*(1-u)*(*this)[l][k] + t*(1-u)*(*this)[l+1][k] +
    (1-t)*u*(*this)[l][k+1] + t*u*(*this)[l+1][k+1] ;

}



/*!
  used for building pyramid of the image
   \warning = must be defined for <Type>
 */

template<class Type>
void
vpImage<Type>::halfSizeImage(vpImage<Type>* res)
{
  int r = height/2;
  int c = width/2;
  if(res == NULL)
    res = new vpImage<Type>(r, c);
  else if((res->getCols() != c) || (res->getRows()!= r))
    res->resize(r,c);
  for(int y = 0; y < r; y++)
    for(int x = 0; x < c; x++)
      (*res)[y][x] = (*this)[y*2][x*2];
}

/*!
  used (eg. in case of keypoints extraction, we might
  double size of the image in order to have more keypoints)
   \warning = must be defined for <Type>
 */
template<class Type>
void
vpImage<Type>::doubleSizeImage(vpImage<Type>* res)
{
  int h = height*2;
  int w = width*2;

  if(res == NULL)
    res = new vpImage<Type>(h, w);
  else if((res->getCols() != w) || (res->getRows()!= h))
    res->resize(h, w);

  for(int j = 0; j < h; j++)
    for(int i = 0; i < w; i++)
      (*res)[j][i] = (*this)[j/2][i/2];

  /*
    A B C
    E F G
    H I J
    A C H J are pixels from original image
    B E G I are interpolated pixels
  */

  //interpolate pixels B and I
  for(int j = 0; j < h; j += 2)
    for(int i = 1; i < w - 1; i += 2)
      (*res)[j][i] = (Type)(0.5 * ((*this)[j/2][i/2] + (*this)[j/2][i/2 + 1]));

  //interpolate pixels E and G
  for(int j = 1; j < h - 1; j += 2)
    for(int i = 0; i < w; i += 2)
      (*res)[j][i] = (Type)(0.5 * ((*this)[j/2][i/2] + (*this)[j/2+1][i/2]));

  //interpolate pixel F
  for(int j = 1; j < h - 1; j += 2)
    for(int i = 1; i < w - 1; i += 2)
      (*res)[j][i] = (Type)(0.25 * ((*this)[j/2][i/2] + (*this)[j/2][i/2+1] +
			     (*this)[j/2+1][i/2] + (*this)[j/2+1][i/2+1]));
}

template<class Type>
void
vpImage<Type>::sub(vpImage<Type>* im2, vpImage<Type>* dst)
{
  if(dst == NULL)
    dst = new vpImage<Type>(height, width);
  else if((dst->getRows()!=height) || (dst->getCols()!=width))
    dst->resize(height, width);

  for(int i  = 0; i < height * width; i++)
    dst->bitmap[i] = this->bitmap[i] - im2->bitmap[i];
}

/*!
  used (eg. to make keypoint patch (normalization)
   \warning = must be defined for <Type>
 */

template<class Type>
Type vpImage<Type>::getPixelBI(float col0, float row0)
{
  int irow, icol;
  float rfrac, cfrac;
  Type row1 = 0, row2 = 0;

  irow = (int) row0;
  icol = (int) col0;

  if (irow < 0 || irow >= height
      || icol < 0 || icol >= width)
    return 0;

  if (row0 > height - 1)
    row0 = height - 1;

  if (col0 > width - 1)
    col0 = width - 1;

  rfrac = 1.0 - (row0 - (float) irow);
  cfrac = 1.0 - (col0 - (float) icol);

  if (cfrac < 1)
  {
    row1 = cfrac * row[irow][icol] + (1.0 - cfrac) * row[irow][icol + 1];
  }
  else
  {
    row1 = row[irow][icol];
  }

  if (rfrac < 1)
  {
    if (cfrac < 1)
    {
      row2 = cfrac * row[irow+1][icol] + (1.0 - cfrac) * row[irow+1][icol + 1];
    }
    else
    {
      row2 = row[irow+1][icol];
    }
  }
  return (Type)(rfrac * row1 + (1.0 - rfrac) * row2);
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
