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
 * Image handling.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpImage.h
  \brief Image handling.
*/

#ifndef vpImage_H
#define vpImage_H


#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpException.h>
#include <visp/vpImageException.h>
#include <visp/vpImagePoint.h>


class vpDisplay;

/*!
  \class vpImage

  \ingroup ImageContainer ImageDataRepresentation

  \brief Definition of the vpImage class member functions.

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
  friend class vpImageConvert;

public:
  Type *bitmap ;  //!< points toward the bitmap
  vpDisplay *display ;

  //! constructor
  vpImage() ;
  //! copy constructor
  vpImage(const vpImage<Type>&);
  //! constructor  set the size of the image
  vpImage(unsigned int height, unsigned int width) ;
  //! constructor  set the size of the image and init all the pixel
  vpImage(unsigned int height, unsigned int width, Type value) ;
  //! destructor
  virtual ~vpImage() ;
  //! set the size of the image
  void init(unsigned int height, unsigned int width) ;
  //! set the size of the image
  void init(unsigned int height, unsigned int width, Type value) ;
  //! set the size of the image
  void resize(const unsigned int height, const unsigned int width) ;
  //! destructor
  void destroy() ;

  /*!
    Get the image height.

    \return The image height.

    \sa getWidth()

  */
  inline  unsigned int getHeight() const { return height; }
  /*!
    Get the image width.

    \return The image width.

    \sa getHeight()

  */
  inline  unsigned int getWidth() const { return width; }


  // Return the maximum value within the bitmap
  Type getMaxValue() const ;
  // Return the minumum value within the bitmap
  Type getMinValue() const ;

  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(double i, double j) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(const vpImagePoint &ip) const;

  /*!

    Get the image number of pixels which corresponds to the image
    width multiplied by the image height.

    \return The image number of pixels or image size.


    \sa getWidth(), getHeight()
   */
  inline unsigned int getNumberOfPixel() const{ return npixels; }

  //------------------------------------------------------------------
  //         Acces to the image


  //! operator[] allows operation like I[i] = x
  inline Type *operator[]( const unsigned int n)   { return row[n];}

  //! operator[] allows operation like x = I[i]
  inline const  Type *operator[](unsigned int n) const { return row[n];}

  /*!
    Get the value of an image point.

    \param i,j : Image point coordinates; i for the row position, j for the
    column position.

    \return Value of the image point (i, j).

  */
  inline  Type operator()(const unsigned int i, const  unsigned int j) const
  {
    return bitmap[i*width+j] ;
  }
  /*!
    Set the value of an image point.

    \param i, j: Image point coordinates; i for the row position, j for the
    column position.

    \param v : Value to set for image point (i, j).

  */
  inline  void  operator()(const unsigned int i, const  unsigned int j,
			   const Type &v)
  {
    bitmap[i*width+j] = v ;
  }
  /*!
    Get the value of an image point.

    \param ip : An image point with sub-pixel coordinates. Sub-pixel 
	coordinates are roughly transformed to insigned int coordinates by cast.

    \return Value of the image point \e ip.

    \sa getValue(const vpImagePoint &)

  */
  inline  Type operator()(const vpImagePoint &ip) const
  {
    unsigned int i = (unsigned int) ip.get_i();
    unsigned int j = (unsigned int) ip.get_j();

    return bitmap[i*width+j] ;
  }
  /*!
    Set the value of an image point.

    \param ip : An image point with sub-pixel coordinates. Sub-pixel 
	coordinates are roughly transformed to insigned int coordinates by cast.

    \param v : Value to set for the image point.

  */
  inline  void  operator()(const vpImagePoint &ip,
			   const Type &v)
  {
    unsigned int i = (unsigned int) ip.get_i();
    unsigned int j = (unsigned int) ip.get_j();

    bitmap[i*width+j] = v ;
  }

  vpImage<Type> operator-(const vpImage<Type> &B);

  //! Copy operator
  void operator=(const vpImage<Type> &m) ;

  void operator=(const Type &x);

  void sub(const vpImage<Type> &A, const vpImage<Type> &B, vpImage<Type> &C);
  //! Substract two images: dst = this - im2;
  void sub(vpImage<Type>* im2, vpImage<Type>* dst);

  // Returns a new image that's half size of the current image
  void halfSizeImage(vpImage<Type> &res);
  // Returns a new image that's half size of the current image
  void halfSizeImage(vpImage<Type>* res);

  // Returns a new image that's a quarter size of the current image
  void quarterSizeImage(vpImage<Type> &res);
  // Returns a new image that's a quarter size of the current image
  void quarterSizeImage(vpImage<Type>* res);

  // Returns a new image that's double size of the current image
  void doubleSizeImage(vpImage<Type> &res);
  // Returns a new image that's double size of the current image
  void doubleSizeImage(vpImage<Type>* res);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  // Gets the value of a pixel at a location with bilinear interpolation.
  vp_deprecated double get(double i, double j) ;

  // Gets the value of a pixel at a location with bilinear interpolation.
  vp_deprecated Type getPixelBI(double j, double i) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  vp_deprecated Type getSubPix(double i, double j) const;
  /*!

    \deprecated This method is deprecated. You should use
    vpImage::getHeight() instead.

    Get the number of rows in the image.

    \return The image number of rows, or image height.

    \warning getRows() is obsolete and preserved for the moment for
    compatibility with previous releases. You should use getHeight()
    instead.

    \sa getHeight()
  */
  vp_deprecated inline  unsigned int getRows() const { return height ; }

  /*!  
    \deprecated This method is deprecated. You should use
    vpImage::getWidth() instead.

    Get the number of columns in the image.

    \return The image number of column, or image width.

    \warning getCols() is obsolete and preserved for the moment for
    compatibility with previous releases. You should use getWidth()
    instead.

    \sa getWidth()
   */
  vp_deprecated inline  unsigned int getCols() const { return width ; }
  // Return the maximum value within the bitmap
  vp_deprecated Type maxValue() const ;
  // Return the minumum value within the bitmap
  vp_deprecated Type minValue() const ;
#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

private:
  unsigned int npixels ; //<! number of pixel in the image
  unsigned int width ;   //<! number of columns
  unsigned int height ;   //<! number of rows
  Type **row ;    //!< points the row pointer array
  } ;


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

  \sa getMinValue()
*/
template<class Type>
Type vpImage<Type>::getMaxValue() const
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

  \sa getMaxValue()
*/
template<class Type>
Type vpImage<Type>::getMinValue() const
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
  Operation  A - B (A is unchanged).

  \sa sub(const vpImage<Type> &, const vpImage<Type> &, vpImage<Type> &) to
  avoid matrix allocation for each use.
*/
template<class Type>
vpImage<Type> vpImage<Type>::operator-(const vpImage<Type> &B)
{
  vpImage<Type> C;
  sub(*this,B,C);
  return C;
}

/*!
  Operation C = A - B.

  The Result is placed in the third parameter C and not returned.
  A new image won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \sa operator-()
*/
template<class Type>
void vpImage<Type>::sub(const vpImage<Type> &A, const vpImage<Type> &B,
			vpImage<Type> &C)
{

  try
  {
    if ((A.getHeight() != C.getHeight())
	|| (A.getWidth() != C.getWidth()))
      C.resize(A.getHeight(), A.getWidth());
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }

  if ( (A.getWidth() != B.getWidth())||(A.getHeight() != B.getHeight()))
  {
    vpERROR_TRACE("\n\t\t vpImage mismatch in vpImage/vpImage substraction") ;
    throw(vpException(vpException::memoryAllocationError,
		      "vpImage mismatch in vpImage/vpImage substraction ")) ;
  }

  for (unsigned int i=0;i<A.getWidth()*getHeight();i++)
  {
    *(C.bitmap + i) = *(A.bitmap + i) - *(B.bitmap + i) ;
  }
}

/*!
  Returns a new image that's half size of the current image.
  No filtering is used during the sub sampling.
  Used for building pyramid of the image.
  \warning Operator = must be defined for Type.

 */

template<class Type>
void
vpImage<Type>::halfSizeImage(vpImage<Type>* res)
{
  unsigned int r = height/2;
  unsigned int c = width/2;
  if(res == NULL)
    res = new vpImage<Type>(r, c);
  else if((res->getWidth() != c) || (res->getHeight()!= r))
    res->resize(r,c);
  for(unsigned int y = 0; y < r; y++)
    for(unsigned int x = 0; x < c; x++)
      (*res)[y][x] = (*this)[y*2][x*2];
}

/*!
  Returns a new image that's half size of the current image.
  No filtering is used during the sub sampling.

  Used for building pyramid of the image.
  \warning Operator = must be defined for Type.

  \param res [out] : Subsampled image that is half size of the current image.

  The example below shows how to use this method:
  \code
  vpImage<unsigned char> I; // original image
  vpImageIo::readPGM(I, "myImage.pgm");
  vpImage<unsigned char> I2; // half size image
  I.halfSizeImage(I2);
  vpImageIo::writePGM(I2, "myHalfSizeImage.pgm");
  \endcode

  This other example shows how to construct a pyramid of the image:
  \code
  vpImage<unsigned char> I[4];   // pyramid with 4 levels
  vpImageIo::readPGM(I[1], "myImage.pgm"); // Original image at level 1
  // compute the other levels
  I5[1].doubleSizeImage(I5[0]);  // double size image at level 0
  I5[1].halfSizeImage(I5[2]);    // half size image at level 2
  I5[1].quarterSizeImage(I5[3]); // quarter size image at level 3
  \endcode

*/
template<class Type>
void
vpImage<Type>::halfSizeImage(vpImage<Type> &res)
{
  unsigned int h = height/2;
  unsigned int w = width/2;
  res.resize(h, w);
  for(unsigned int i = 0; i < h; i++)
    for(unsigned int j = 0; j < w; j++)
      res[i][j] = (*this)[i<<1][j<<1];
}

/*!
  Returns a new image that's a quarter size of the current image.
  Used for building a quarter of the image.
  \warning Operator = must be defined for Type.
 */

template<class Type>
void
vpImage<Type>::quarterSizeImage(vpImage<Type>* res)
{
  unsigned int r = height/4;
  unsigned int c = width/4;
  if(res == NULL)
    res = new vpImage<Type>(r, c);
  else if((res->getWidth() != c) || (res->getHeight()!= r))
    res->resize(r,c);
  for(unsigned int y = 0; y < r; y++)
    for(unsigned int x = 0; x < c; x++)
      (*res)[y][x] = (*this)[y*4][x*4];
}

/*!
  Returns a new image that's a quarter size of the current image.
  No filtering is used during the sub sampling.
  Used for building a quarter of the image.
  \warning Operator = must be defined for Type.

  \param res [out] : Subsampled image that is quarter size of the
  current image.

  The example below shows how to use this method:
  \code
  vpImage<unsigned char> I; // original image
  vpImageIo::readPGM(I, "myImage.pgm");
  vpImage<unsigned char> I4; // quarter size image
  I.halfSizeImage(I4);
  vpImageIo::writePGM(I4, "myQuarterSizeImage.pgm");
  \endcode

  See halfSizeImage(vpImage<Type> &) for an example of pyramid construction.

*/

template<class Type>
void
vpImage<Type>::quarterSizeImage(vpImage<Type> &res)
{
  unsigned int h = height/4;
  unsigned int w = width/4;
  res.resize(h, w);
  for(unsigned int i = 0; i < h; i++)
    for(unsigned int j = 0; j < w; j++)
      res[i][j] = (*this)[i<<2][j<<2];
}

/*!
  Returns a new image that's double size of the current image.
  Used (eg. in case of keypoints extraction, we might
  double size of the image in order to have more keypoints)
  \warning Operator = must be defined for Type.
 */
template<class Type>
void
vpImage<Type>::doubleSizeImage(vpImage<Type>* res)
{
  int h = height*2;
  int w = width*2;

  if(res == NULL)
    res = new vpImage<Type>(h, w);
  else if((res->getWidth() != w) || (res->getHeight()!= h))
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

/*!
  Returns a new image that's double size of the current image.
  Used (eg. in case of keypoints extraction, we might
  double size of the image in order to have more keypoints).
  The double size image is computed by nearest-neighbour interpolation:

  \code
  A B C
  E F G
  H I J

  where
  A C H J are pixels from original image
  B E G I are interpolated pixels
  \endcode


  \warning Operator = must be defined for Type.

  \param res [out] : Image that is double size of the current image.

  The example below shows how to use this method:
  \code
  vpImage<unsigned char> I; // original image
  vpImageIo::readPGM(I, "myImage.pgm");
  vpImage<unsigned char> I2; // double size image
  I.doubleSizeImage(I2);
  vpImageIo::writePGM(I2, "myDoubleSizeImage.pgm");
  \endcode
 
  See halfSizeImage(vpImage<Type> &) for an example of pyramid construction.

*/
template<class Type>
void
vpImage<Type>::doubleSizeImage(vpImage<Type> &res)
{
  int h = height*2;
  int w = width*2;

  res.resize(h, w);

  for(int i = 0; i < h; i++)
    for(int j = 0; j < w; j++)
      res[i][j] = (*this)[i>>1][j>>1];

  /*
    A B C
    E F G
    H I J
    A C H J are pixels from original image
    B E G I are interpolated pixels
  */

  //interpolate pixels B and I
  for(int i = 0; i < h; i += 2)
    for(int j = 1; j < w - 1; j += 2)
      res[i][j] = (Type)(0.5 * ((*this)[i>>1][j>>1] 
				+ (*this)[i>>1][(j>>1) + 1]));

  //interpolate pixels E and G
  for(int i = 1; i < h - 1; i += 2)
    for(int j = 0; j < w; j += 2)
      res[i][j] = (Type)(0.5 * ((*this)[i>>1][j>>1] 
				+ (*this)[(i>>1)+1][j>>1]));

  //interpolate pixel F
  for(int i = 1; i < h - 1; i += 2)
    for(int j = 1; j < w - 1; j += 2)
      res[i][j] = (Type)(0.25 * ((*this)[i>>1][j>>1] 
				 + (*this)[i>>1][(j>>1)+1] 
				 + (*this)[(i>>1)+1][j>>1] 
				 + (*this)[(i>>1)+1][(j>>1)+1]));
}

template<class Type>
void
vpImage<Type>::sub(vpImage<Type>* im2, vpImage<Type>* dst)
{
  if(dst == NULL)
    dst = new vpImage<Type>(height, width);
  else if((dst->getHeight()!=height) || (dst->getWidth()!=width))
    dst->resize(height, width);

  for(unsigned int i  = 0; i < height * width; i++)
    dst->bitmap[i] = this->bitmap[i] - im2->bitmap[i];
}

/*! 

  Retrieves pixel value from image with sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation. If location is out of bounds, then return value of
  closest pixel.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.

*/

template<class Type>
Type vpImage<Type>::getValue(double i, double j) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;
  Type row1;
  Type row2= (Type)(row[0][0]); // The compiler request an initialisation

  iround = (int) i;
  jround = (int) j;

  if (/*iround < 0 || */ iround >= height
      /* || jround < 0 */ || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
		      "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  rfrac = 1.0f - (i - (double) iround);
  cfrac = 1.0f - (j - (double) jround);

  if (cfrac < 1)
  {
    row1 = (Type)(row[iround][jround] * cfrac + row[iround][jround + 1]*(1.0 - cfrac));
  }
  else
  {
    row1 = row[iround][jround];
  }

  if (rfrac < 1)
  {
    if (cfrac < 1)
    {
      row2 = (Type)(row[iround+1][jround] * cfrac
		    + row[iround+1][jround + 1]*(1.0 - cfrac) );
    }
    else
    {
      row2 = row[iround+1][jround];
    }
  }
  return (Type)( row1 * rfrac +  row2 * (1.0 - rfrac));
}

/*! 

  Retrieves pixel value from image with sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation. If location is out of bounds, then return value of
  closest pixel.

  \param ip : An image point.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If the image point \e ip is out
  of the image.

*/

template<class Type>
Type vpImage<Type>::getValue(const vpImagePoint &ip) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;
  Type row1;
  Type row2= (Type)(row[0][0]); // The compiler request an initialisation

  double i = ip.get_i();
  double j = ip.get_j();

  iround = (int) i;
  jround = (int) j;

  if (/*iround < 0 || */ iround >= height
      /* || jround < 0 */ || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
		      "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  rfrac = 1.0f - (i - (double) iround);
  cfrac = 1.0f - (j - (double) jround);

  if (cfrac < 1)
  {
    row1 = (Type)(row[iround][jround] * cfrac + row[iround][jround + 1]*(1.0 - cfrac));
  }
  else
  {
    row1 = row[iround][jround];
  }

  if (rfrac < 1)
  {
    if (cfrac < 1)
    {
      row2 = (Type)(row[iround+1][jround] * cfrac
		    + row[iround+1][jround + 1]*(1.0 - cfrac) );
    }
    else
    {
      row2 = row[iround+1][jround];
    }
  }
  return (Type)( row1 * rfrac +  row2 * (1.0 - rfrac));
}



/****************************************************************

           Deprecated functions

*****************************************************************/

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!

  \deprecated This method is deprecated. You should use
  vpImage::getSubPix() instead.

  Get the value of pixel at coordinates [i][j] using a bilinear
  interpolation.

  \param i : Sub pixel coordinate along the rows.
  \param j : Sub pixel coordinate along the colums.

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

  \sa getSubPix()

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

  \deprecated This method is deprecated. You should use
  vpImage::getValue() instead.

  Retrieves pixel value from image with sub-pixel accuracy. 


  Gets the value of a pixel at a location with bilinear
  interpolation. If location is out of bounds, then return value of
  closest pixel.

  \param i : Sub pixel coordinate along the rows.
  \param j : Sub pixel coordinate along the colums.

  \exception vpImageException::notInTheImage : If (i, j) are out
  of the image.

  \sa getSubPix()

*/
template<class Type>
Type vpImage<Type>::getPixelBI(double j, double i) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;
  Type row1;
  Type row2= (Type)(row[0][0]); // The compiler request an initialisation

  iround = (int) i;
  jround = (int) j;

  if (/*iround < 0 || */ iround >= height
      /* || jround < 0 */ || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
		      "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  rfrac = 1.0f - (i - (double) iround);
  cfrac = 1.0f - (j - (double) jround);

  if (cfrac < 1)
  {
    row1 = (Type)(row[iround][jround] * cfrac + row[iround][jround + 1]*(1.0 - cfrac));
  }
  else
  {
    row1 = row[iround][jround];
  }

  if (rfrac < 1)
  {
    if (cfrac < 1)
    {
      row2 = (Type)(row[iround+1][jround] * cfrac
		    + row[iround+1][jround + 1]*(1.0 - cfrac) );
    }
    else
    {
      row2 = row[iround+1][jround];
    }
  }
  return (Type)( row1 * rfrac +  row2 * (1.0 - rfrac));
}
/*! 

  \deprecated This method is deprecated. You should use
  vpImage::getValue() instead.

  Retrieves pixel value from image with sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation. If location is out of bounds, then return value of
  closest pixel.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.

*/

template<class Type>
Type vpImage<Type>::getSubPix(double i, double j) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;
  Type row1;
  Type row2= (Type)(row[0][0]); // The compiler request an initialisation

  iround = (int) i;
  jround = (int) j;

  if (/*iround < 0 || */ iround >= height
      /* || jround < 0 */ || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
		      "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  rfrac = 1.0f - (i - (double) iround);
  cfrac = 1.0f - (j - (double) jround);

  if (cfrac < 1)
  {
    row1 = (Type)(row[iround][jround] * cfrac + row[iround][jround + 1]*(1.0 - cfrac));
  }
  else
  {
    row1 = row[iround][jround];
  }

  if (rfrac < 1)
  {
    if (cfrac < 1)
    {
      row2 = (Type)(row[iround+1][jround] * cfrac
		    + row[iround+1][jround + 1]*(1.0 - cfrac) );
    }
    else
    {
      row2 = row[iround+1][jround];
    }
  }
  return (Type)( row1 * rfrac +  row2 * (1.0 - rfrac));
}

/*!

  \deprecated This method is deprecated. You should use
  vpImage::getMaxValue() instead.

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

  \deprecated This method is deprecated. You should use
  vpImage::getMaxValue() instead.

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

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS


// For template instantiation with Visual Studio
#if defined(VISP_BUILD_SHARED_LIBS) && defined(VISP_USE_MSVC)
template class VISP_EXPORT vpImage<unsigned char>;
#endif

#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
