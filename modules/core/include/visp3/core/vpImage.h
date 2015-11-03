/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRGBa.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

class vpDisplay;

/*!
  \class vpImage

  \ingroup group_core_image

  \brief Definition of the vpImage class member functions.

  This is a template class, therefore the type of each  element of the
  array is not a priori defined.

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

  <h3>Example</h3>
  The following example available in tutorial-image-manipulation.cpp shows how
  to create gray level and color images and how to access to the pixels.
  \include tutorial-image-manipulation.cpp

  <h3>Important remark</h3> To provide high-performance access there
  is no verification to ensure that 0 \f$\le\f$ i < height and 0
  \f$\le\f$ j < width. Since the memory allocated in the bitmap array
  is continuous, that means that if (i, j) is outside the image you
  will manipulate a pixel that is not as expected. To highlight this
  remark, we provide hereafter an example where the considered pixel
  is outside the image:

\code
unsigned int width = 320;
unsigned int height = 240;
vpImage<unsigned char> I(height, width); // Create an 320x240 image
// Set pixel coordinates that is outside the image
unsigned int i = 100;
unsigned int j = 400;
unsigned char value;
value = I[i][j]; // Here we will get the pixel value at position (101, 80)
\endcode

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
  //! constructor from an image stored as a continuous array in memory
  vpImage(Type * const array, const unsigned int height, const unsigned int width, const bool copyData=false) ;
  //! destructor
  virtual ~vpImage() ;
  //! set the size of the image
  void init(unsigned int height, unsigned int width) ;
  //! set the size of the image
  void init(unsigned int height, unsigned int width, Type value) ;
  //! init from an image stored as a continuous array in memory
  void init(Type * const array, const unsigned int height, const unsigned int width, const bool copyData=false);
  //! set the size of the image without initializing it.
  void resize(const unsigned int h, const unsigned int w) ;
  //! set the size of the image and initialize it.
  void resize(const unsigned int h, const unsigned int w, const Type val) ;
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

 /*!

    Get the number of rows in the image.

    \return The image number of rows, or image height.

    \sa getHeight()
  */
  inline  unsigned int getRows() const { return height ; }

  /*!
    Get the number of columns in the image.

    \return The image number of column, or image width.

    \sa getWidth()
   */
   inline  unsigned int getCols() const { return width ; }

  /*!
    Get the image size.

    \return The image size = width * height.

    \sa getWidth(), getHeight()
   */
   inline  unsigned int getSize() const { return width*height ; }


  // Return the maximum value within the bitmap
  Type getMaxValue() const ;
  // Return the minumum value within the bitmap
  Type getMinValue() const ;
  //Look for the minumum and the maximum value within the bitmap
  void getMinMaxValue(Type &min, Type &max) const;

  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(double i, double j) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(vpImagePoint &ip) const;

  /*!

    Get the image number of pixels which corresponds to the image
    width multiplied by the image height.

    \return The image number of pixels or image size.


    \sa getWidth(), getHeight()
   */
  inline unsigned int getNumberOfPixel() const{ return npixels; }

  //------------------------------------------------------------------
  //         Acces to the image


  //! operator[] allows operation like I[i] = x.
  inline Type *operator[]( const unsigned int i)   { return row[i];}
  inline Type *operator[]( const int i)   { return row[i];}

  //! operator[] allows operation like x = I[i]
  inline const  Type *operator[](unsigned int i) const { return row[i];}
  inline const  Type *operator[](int i) const { return row[i];}

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
  vpImage<Type>&  operator=(const vpImage<Type> &I) ;

  vpImage<Type>& operator=(const Type &v);
  bool operator==(const vpImage<Type> &I);
  bool operator!=(const vpImage<Type> &I);

  void insert(const vpImage<Type> &src, const vpImagePoint topLeft);


  // Returns a new image that's half size of the current image
  void halfSizeImage(vpImage<Type> &res);

  // Returns a new image that's a quarter size of the current image
  void quarterSizeImage(vpImage<Type> &res);

  // Returns a new image that's double size of the current image
  void doubleSizeImage(vpImage<Type> &res);

  void sub(const vpImage<Type> &B, vpImage<Type> &C);
  void sub(const vpImage<Type> &A, const vpImage<Type> &B, vpImage<Type> &C);

  // Perform a look-up table transformation
  void performLut(const Type (&lut)[256]);

private:
  unsigned int npixels ; //<! number of pixel in the image
  unsigned int width ;   //<! number of columns
  unsigned int height ;   //<! number of rows
  Type **row ;    //!< points the row pointer array
  } ;


/*!
  \brief Image initialisation

  Allocate memory for an [h x w] image.

  \param w : Image width.
  \param h : Image height.
  \param value : Set all the element of the bitmap to \e value.

  \exception vpException::memoryAllocationError

  \sa vpImage::init(h, w)
*/
template<class Type>
void
vpImage<Type>::init(unsigned int h, unsigned int w, Type value)
{
  try
  {
    init(h,w) ;
  }
  catch(vpException &)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  for (unsigned int i=0  ; i < npixels ;  i++)
    bitmap[i] = value ;
}


/*!
  \brief Image initialization

  Allocate memory for an [h x w] image.

  \param w : Image width.
  \param h : Image height.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template<class Type>
void
vpImage<Type>::init(unsigned int h, unsigned int w)
{
  if (h != this->height) {
    if (row != NULL)  {
      vpDEBUG_TRACE(10,"Destruction row[]");
      delete [] row;
      row = NULL;
    }
  }

  if ((h != this->height) || (w != this->width))
  {
    if (bitmap != NULL) {
      vpDEBUG_TRACE(10,"Destruction bitmap[]") ;
      delete [] bitmap;
      bitmap = NULL;
    }
  }

  this->width = w ;
  this->height = h;

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
  \brief Image initialization

  Init from image data stored as a continuous array in memory.

  \param array : Image data stored as a continuous array in memory
  \param h : Image height.
  \param w : Image width.
  \param copyData : If false (by default) only the memory address is copied, otherwise the data are copied.

  \exception vpException::memoryAllocationError
*/
template<class Type>
void
vpImage<Type>::init(Type * const array, const unsigned int h, const unsigned int w, const bool copyData)
{
  if (h != this->height) {
    if (row != NULL)  {
      delete [] row;
      row = NULL;
    }
  }

  //Delete bitmap if copyData==false, otherwise only if the dimension differs
  if ( (copyData && ((h != this->height) || (w != this->width))) || !copyData ) {
    if (bitmap != NULL) {
      delete [] bitmap;
      bitmap = NULL;
    }
  }

  this->width = w ;
  this->height = h;

  npixels = width*height;

  if(copyData) {
    if (bitmap == NULL)  bitmap = new  Type[npixels];

    if (bitmap == NULL) {
      throw(vpException(vpException::memoryAllocationError,
            "cannot allocate bitmap ")) ;
    }

    //Copy the image data
    memcpy(bitmap, array, (size_t) (npixels * sizeof(Type)));
  } else {
    //Copy the address of the array in the bitmap
    bitmap = array;
  }

  if (row == NULL)  row = new Type*[height];
  if (row == NULL) {
    throw(vpException(vpException::memoryAllocationError,
          "cannot allocate row ")) ;
  }

  for (unsigned int i = 0  ; i < height ; i++) {
    row[i] = bitmap + i*width;
  }
}

/*!
  \brief Constructor

  Allocate memory for an [h x w] image.

  \param w : Image width.
  \param h : Image height.

  Element of the bitmap are set to zero.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa vpImage::init(height, width)
*/
template<class Type>
vpImage<Type>::vpImage(unsigned int h, unsigned int w)
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  try
  {
    init(h,w,0) ;
  }
  catch(...)
  {
    throw ;
  }
}

/*!
  \brief Constructor

  Allocate memory for an [height x width] image.

  \param w : Image width.
  \param h : Image height.
  \param value : Set all the element of the bitmap to value.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return MEMORY_FAULT if memory allocation is impossible, else OK

  \sa vpImage::init(height, width, value)
*/
template<class Type>
vpImage<Type>::vpImage (unsigned int h, unsigned int w, Type value)
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  try
  {
    init(h,w,value) ;
  }
  catch(vpException &)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Constructor

  Construct a vpImage from a continuous array in memory.

  \param array : Image data stored as a continuous array in memory.
  \param h : Image height.
  \param w : Image width.
  \param copyData : If false (by default) only the memory address is copied, otherwise the data are copied.

  \return MEMORY_FAULT if memory allocation is impossible, else OK

  \sa vpImage::init(array, height, width)
*/
template<class Type>
vpImage<Type>::vpImage (Type * const array, const unsigned int h, const unsigned int w, const bool copyData)
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  try
  {
    init(array, h, w, copyData);
  }
  catch(vpException &)
  {
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
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
}

/*!
  \brief resize the image : Image initialization

  Allocate memory for an [height x width] image.

  \warning The image is not initialized.

  \param w : Image width.
  \param h : Image height.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa init(unsigned int, unsigned int)
*/
template<class Type>
void
vpImage<Type>::resize(unsigned int h, unsigned int w)
{
  try
  {
    init(h, w) ;
  }
  catch(vpException &)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief resize the image : Image initialization

  Allocate memory for an [height x width] image and initialize the image.

  \param w : Image width.
  \param h : Image height.
  \param val : Pixels value.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa init(unsigned int, unsigned int)
*/
template<class Type>
void
vpImage<Type>::resize(unsigned int h, unsigned int w, const Type val)
{
  try
  {
    init(h, w, val) ;
  }
  catch(vpException &)
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
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  try
  {
    resize(I.getHeight(),I.getWidth());
    memcpy(bitmap, I.bitmap, I.npixels*sizeof(Type)) ;
    for (unsigned int i =0  ; i < this->height ; i++) row[i] = bitmap + i*this->width ;
  }
  catch(vpException &)
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
  \brief Look for the minimum and the maximum value within the bitmap

  \sa getMaxValue()
  \sa getMinValue()
*/
template<class Type>
void vpImage<Type>::getMinMaxValue(Type &min, Type &max) const
{
  min = max =  bitmap[0];
  for (unsigned int i=0 ; i < npixels ; i++)
  {
    if (bitmap[i]<min) min = bitmap[i] ;
    if (bitmap[i]>max) max = bitmap[i] ;
  }
}

/*!
  \brief Copy operator
*/
template<class Type>
vpImage<Type> & vpImage<Type>::operator=(const vpImage<Type> &I)
{
    /* we first have to set the initial values of the image because resize function calls init function that test the actual size of the image */
  if(bitmap != NULL){
    delete[] bitmap;
    bitmap = NULL ;
  }

  if(row != NULL){
    delete[] row;
    row = NULL ;
  }
  this->width = I.width;
  this->height = I.height;
  this->npixels = I.npixels;
  try
  {
    if(I.npixels != 0)
    {
      if (bitmap == NULL){
        bitmap = new  Type[npixels] ;
      }

      if (bitmap == NULL){
            vpERROR_TRACE("cannot allocate bitmap ") ;
        throw(vpException(vpException::memoryAllocationError,
              "cannot allocate bitmap ")) ;
      }

      if (row == NULL){
        row = new  Type*[height] ;
      }
      if (row == NULL){
        vpERROR_TRACE("cannot allocate row ") ;
        throw(vpException(vpException::memoryAllocationError,
              "cannot allocate row ")) ;
      }

      memcpy(bitmap, I.bitmap, I.npixels*sizeof(Type)) ;

      for (unsigned int i=0; i<this->height; i++){
        row[i] = bitmap + i*this->width;
      }
    }
  }
  catch(vpException &)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
  return (* this);
}


/*!
  \brief = operator : Set all the element of the bitmap to a given  value \e v.
   \f$ A = v <=> A[i][j] = v \f$

   \warning = must be defined for \f$ <\f$ Type \f$ > \f$
*/
template<class Type>
vpImage<Type>& vpImage<Type>::operator=(const Type &v)
{
  for (unsigned int i=0 ; i < npixels ; i++)
    bitmap[i] = v ;

  return *this;
}

/*!
  Compare two images.

  \return true if the images are the same, false otherwise.
*/
template<class Type>
bool vpImage<Type>::operator==(const vpImage<Type> &I)
{
  if (this->width != I.getWidth())
    return false;
  if (this->height != I.getHeight())
    return false;

  for (unsigned int i=0 ; i < npixels ; i++)
  {
    if (bitmap[i] != I.bitmap[i])
      return false;
  }
  return true ;
}
/*!
  Compare two images.

  \return true if the images are different, false if they are the same.
*/
template<class Type>
bool vpImage<Type>::operator!=(const vpImage<Type> &I)
{
  if (this->width != I.getWidth())
    return true;
  if (this->height != I.getHeight())
    return true;

  for (unsigned int i=0 ; i < npixels ; i++)
  {
    if (bitmap[i] == I.bitmap[i])
      return false;
  }
  return true ;
}

/*!
  Operation  A - B (A is unchanged).

  \code
#include <visp3/core/vpImage.h>

int main()
{
  vpImage<unsigned char> A(288, 384);
  vpImage<unsigned char> B(288, 384);
  vpImage<unsigned char> C;

  A = 128;
  B = 120;

  // operator-() : C = A - B
  C = A - B;

  return 0;
}
  \endcode

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
  Insert an image into another one.

  It is possible to insert the image \f$ src \f$ into the calling vpImage.
  You can set the point in the destination image where the top left corner of the \f$ src \f$ image will belocated.

  \param src : Image to insert
  \param topLeft : Coordinates of the \f$ src \f$ image's top left corner in the destination image.
*/
template<class Type>
void vpImage<Type>::insert(const vpImage<Type> &src,
      const vpImagePoint topLeft)
{
  Type* srcBitmap;
  Type* destBitmap;

  int itl = (int)topLeft.get_i();
  int jtl = (int)topLeft.get_j();

  int dest_ibegin = 0;
  int dest_jbegin = 0;
  int src_ibegin = 0;
  int src_jbegin = 0;
  int dest_w = (int)this->getWidth();
  int dest_h = (int)this->getHeight();
  int src_w = (int)src.getWidth();
  int src_h = (int)src.getHeight();
  int wsize = (int)src.getWidth();
  int hsize = (int)src.getHeight();

  if (itl >= dest_h || jtl >= dest_w)
    return;

  if (itl < 0)
    src_ibegin = -itl;
  else
    dest_ibegin = itl;

  if (jtl < 0)
    src_jbegin = -jtl;
  else
    dest_jbegin = jtl;

  if (src_w - src_jbegin > dest_w - dest_jbegin)
    wsize = dest_w - dest_jbegin;
  else
    wsize = src_w - src_jbegin;

  if (src_h - src_ibegin > dest_h - dest_ibegin)
    hsize = dest_h - dest_ibegin;
  else
    hsize = src_h - src_ibegin;

  for (int i = 0; i < hsize; i++)
  {
    srcBitmap = src.bitmap + ((src_ibegin+i)*src_w+src_jbegin);
    destBitmap = this->bitmap + ((dest_ibegin+i)*dest_w+dest_jbegin);

    memcpy(destBitmap,srcBitmap,wsize*sizeof(Type));
  }
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
  vpImageIo::read(I, "myImage.pgm");
  vpImage<unsigned char> I2; // half size image
  I.halfSizeImage(I2);
  vpImageIo::write(I2, "myHalfSizeImage.pgm");
  \endcode

  This other example shows how to construct a pyramid of the image:
  \code
  vpImage<unsigned char> I[4];   // pyramid with 4 levels
  vpImageIo::read(I[1], "myImage.pgm"); // Original image at level 1
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
  No filtering is used during the sub sampling.
  Used for building a quarter of the image.
  \warning Operator = must be defined for Type.

  \param res [out] : Subsampled image that is quarter size of the
  current image.

  The example below shows how to use this method:
  \code
  vpImage<unsigned char> I; // original image
  vpImageIo::read(I, "myImage.pgm");
  vpImage<unsigned char> I4; // quarter size image
  I.halfSizeImage(I4);
  vpImageIo::write(I4, "myQuarterSizeImage.pgm");
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
  vpImageIo::read(I, "myImage.pgm");
  vpImage<unsigned char> I2; // double size image
  I.doubleSizeImage(I2);
  vpImageIo::write(I2, "myDoubleSizeImage.pgm");
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

/*!

  \warning This generic method is not implemented. You should rather use the
  instantiated methods for double, unsigned char and vpRGBa images.

  \sa vpImage<double>::getValue(double, double)
  \sa vpImage<unsigned char>::getValue(double, double)
  \sa vpImage<vpRGBa>::getValue(double, double)

*/
template<class Type>
Type vpImage<Type>::getValue(double /* i */, double /* j */) const
{
  vpTRACE("Not implemented");
}

/*!

  Retrieves pixel value from an image of unsigned char with sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation. If location is out of bounds, then return value of
  closest pixel.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.

*/
template<>
inline unsigned char vpImage<unsigned char>::getValue(double i, double j) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;

  iround = (unsigned int)floor(i);
  jround = (unsigned int)floor(j);

  if (iround >= height || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
          "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  double rratio = i - (double) iround;
  if(rratio < 0)
    rratio=-rratio;
  double cratio = j - (double) jround;
  if(cratio < 0)
    cratio=-cratio;

  rfrac = 1.0f - rratio;
  cfrac = 1.0f - cratio;


  double value = ((double)row[iround][jround] * rfrac + (double)row[iround+1][jround] * rratio)*cfrac
             + ((double)row[iround][jround+1]*rfrac + (double)row[iround+1][jround+1] * rratio)*cratio;
  return (unsigned char)vpMath::round(value);
}


/*!

  Retrieves pixel value from an image of double with sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation. If location is out of bounds, then return value of
  closest pixel.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.

*/
template<>
inline double vpImage<double>::getValue(double i, double j) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;

  iround = (unsigned int)floor(i);
  jround = (unsigned int)floor(j);

  if (iround >= height || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
          "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  double rratio = i - (double) iround;
  if(rratio < 0)
    rratio=-rratio;
  double cratio = j - (double) jround;
  if(cratio < 0)
    cratio=-cratio;

  rfrac = 1.0f - rratio;
  cfrac = 1.0f - cratio;


  double value = ((double)row[iround][jround] * rfrac + (double)row[iround+1][jround] * rratio)*cfrac
             + ((double)row[iround][jround+1]*rfrac + (double)row[iround+1][jround+1] * rratio)*cratio;
  return value;
}

template<>
inline vpRGBa vpImage<vpRGBa>::getValue(double i, double j) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;

  iround = (unsigned int)floor(i);
  jround = (unsigned int)floor(j);

  if (iround >= height || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
          "Pixel outside the image"));
  }

  if (i > height - 1)
    i = (double)(height - 1);

  if (j > width - 1)
    j = (double)(width - 1);

  double rratio = i - (double) iround;
  if(rratio < 0)
    rratio=-rratio;
  double cratio = j - (double) jround;
  if(cratio < 0)
    cratio=-cratio;

  rfrac = 1.0f - rratio;
  cfrac = 1.0f - cratio;

  double valueR = ((double)row[iround][jround].R * rfrac + (double)row[iround+1][jround].R * rratio)*cfrac
             + ((double)row[iround][jround+1].R * rfrac + (double)row[iround+1][jround+1].R * rratio)*cratio;
  double valueG = ((double)row[iround][jround].G * rfrac + (double)row[iround+1][jround].G * rratio)*cfrac
             + ((double)row[iround][jround+1].G* rfrac + (double)row[iround+1][jround+1].G * rratio)*cratio;
  double valueB = ((double)row[iround][jround].B * rfrac + (double)row[iround+1][jround].B * rratio)*cfrac
             + ((double)row[iround][jround+1].B*rfrac + (double)row[iround+1][jround+1].B * rratio)*cratio;
  return vpRGBa((unsigned char)vpMath::round(valueR),(unsigned char)vpMath::round(valueG),(unsigned char)vpMath::round(valueB));
}

/*!

  \warning This generic method is not implemented. You should rather use the
  instantiated methods for double, unsigned char and vpRGBa images.

  \sa vpImage<double>::getValue(vpImagePoint &)
  \sa vpImage<unsigned char>::getValue(vpImagePoint &)
  \sa vpImage<vpRGBa>::getValue(vpImagePoint &)

*/
template<class Type>
Type vpImage<Type>::getValue(vpImagePoint & /* ip */) const
{
  vpTRACE("Not implemented");
}

template<>
inline unsigned char vpImage<unsigned char>::getValue(vpImagePoint &ip) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;

  iround = (unsigned int)floor(ip.get_i());
  jround = (unsigned int)floor(ip.get_j());

  if (iround >= height || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
          "Pixel outside the image"));
  }

  if (ip.get_i() > height - 1)
    ip.set_i((double)(height - 1));

  if (ip.get_j() > width - 1)
    ip.set_j((double)(width - 1));

  double rratio = ip.get_i() - (double) iround;
  if(rratio < 0)
    rratio=-rratio;
  double cratio = ip.get_j() - (double) jround;
  if(cratio < 0)
    cratio=-cratio;

  rfrac = 1.0f - rratio;
  cfrac = 1.0f - cratio;


  double value = ((double)row[iround][jround] * rfrac + (double)row[iround+1][jround] * rratio)*cfrac
             + ((double)row[iround][jround+1]*rfrac + (double)row[iround+1][jround+1] * rratio)*cratio;
  return (unsigned char)vpMath::round(value);
}


template<>
inline double vpImage<double>::getValue(vpImagePoint &ip) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;

  iround = (unsigned int)floor(ip.get_i());
  jround = (unsigned int)floor(ip.get_j());

  if (iround >= height || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
          "Pixel outside the image"));
  }

  if (ip.get_i() > height - 1)
    ip.set_i((double)(height - 1));

  if (ip.get_j() > width - 1)
    ip.set_j((double)(width - 1));

  double rratio = ip.get_i() - (double) iround;
  if(rratio < 0)
    rratio=-rratio;
  double cratio = ip.get_j() - (double) jround;
  if(cratio < 0)
    cratio=-cratio;

  rfrac = 1.0f - rratio;
  cfrac = 1.0f - cratio;


  double value = ((double)row[iround][jround] * rfrac + (double)row[iround+1][jround] * rratio)*cfrac
             + ((double)row[iround][jround+1]*rfrac + (double)row[iround+1][jround+1] * rratio)*cratio;
  return value;
}

template<>
inline vpRGBa vpImage<vpRGBa>::getValue(vpImagePoint &ip) const
{
  unsigned int iround, jround;
  double rfrac, cfrac;

  iround = (unsigned int)floor(ip.get_i());
  jround = (unsigned int)floor(ip.get_j());

  if (iround >= height || jround >= width) {
    vpERROR_TRACE("Pixel outside the image") ;
    throw(vpException(vpImageException::notInTheImage,
          "Pixel outside the image"));
  }

  if (ip.get_i() > height - 1)
    ip.set_i((double)(height - 1));

  if (ip.get_j() > width - 1)
    ip.set_j((double)(width - 1));

  double rratio = ip.get_i() - (double) iround;
  if(rratio < 0)
    rratio=-rratio;
  double cratio = ip.get_j() - (double) jround;
  if(cratio < 0)
    cratio=-cratio;

  rfrac = 1.0f - rratio;
  cfrac = 1.0f - cratio;

  double valueR = ((double)row[iround][jround].R * rfrac + (double)row[iround+1][jround].R * rratio)*cfrac
             + ((double)row[iround][jround+1].R * rfrac + (double)row[iround+1][jround+1].R * rratio)*cratio;
  double valueG = ((double)row[iround][jround].G * rfrac + (double)row[iround+1][jround].G * rratio)*cfrac
             + ((double)row[iround][jround+1].G* rfrac + (double)row[iround+1][jround+1].G * rratio)*cratio;
  double valueB = ((double)row[iround][jround].B * rfrac + (double)row[iround+1][jround].B * rratio)*cfrac
             + ((double)row[iround][jround+1].B*rfrac + (double)row[iround+1][jround+1].B * rratio)*cratio;
  return vpRGBa((unsigned char)vpMath::round(valueR),(unsigned char)vpMath::round(valueG),(unsigned char)vpMath::round(valueB));
}

/*!
  Operation C = *this - B.

  \code
#include <visp3/core/vpImage.h>

int main()
{
  vpImage<unsigned char> A(288, 384);
  vpImage<unsigned char> B(288, 384);
  vpImage<unsigned char> C;

  A = 128;
  B = 120;

  A.sub(B, C); // C = A - B

  return 0;
}
  \endcode

  The result is placed in the third parameter C and not returned.
  A new image won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \exception vpException::memoryAllocationError If the images size differ.

  \sa operator-()
*/
template<class Type>
void vpImage<Type>::sub(const vpImage<Type> &B, vpImage<Type> &C)
{

  try
  {
    if ((this->getHeight() != C.getHeight())
      || (this->getWidth() != C.getWidth()))
      C.resize(this->getHeight(), this->getWidth());
  }
  catch(vpException &me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }

  if ( (this->getWidth() != B.getWidth())||(this->getHeight() != B.getHeight()))
  {
    vpERROR_TRACE("\n\t\t vpImage mismatch in vpImage/vpImage substraction") ;
    throw(vpException(vpException::memoryAllocationError,
          "vpImage mismatch in vpImage/vpImage substraction ")) ;
  }

  for (unsigned int i=0;i<this->getWidth()*this->getHeight();i++)
  {
    *(C.bitmap + i) = *(bitmap + i) - *(B.bitmap + i) ;
  }
}

/*!
  Operation C = A - B.

  The result is placed in the third parameter C and not returned.
  A new image won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \exception vpException::memoryAllocationError If the images size differ.

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
  catch(vpException &me)
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

  for (unsigned int i=0;i<A.getWidth()*A.getHeight();i++)
  {
    *(C.bitmap + i) = *(A.bitmap + i) - *(B.bitmap + i) ;
  }
}

/*!

  \warning This generic method is not implemented. You should rather use the
  instantiated methods for unsigned char and vpRGBa images.

  \sa vpImage<unsigned char>::performLut(const unsigned char (&)[256])
  \sa vpImage<vpRGBa char>::performLut(const vpRGBa (&)[256])

*/
template<class Type>
void vpImage<Type>::performLut(const Type (&)[256])
{
//  vpTRACE("Not implemented");
  std::cerr << "Not implemented !" << std::endl;
}

/*!
  Modify the intensities of a grayscale image using the look-up table passed in parameter.

  \param lut : Look-up table (unsigned char array of size=256) which maps each intensity to his new value.
*/
template<>
inline void vpImage<unsigned char>::performLut(const unsigned char (&lut)[256]) {
  unsigned int size = getWidth()*getHeight();
  unsigned char *ptrStart = (unsigned char*) bitmap;
  unsigned char *ptrEnd = ptrStart + size;
  unsigned char *ptrCurrent = ptrStart;

  while(ptrCurrent != ptrEnd) {
    *ptrCurrent = lut[*ptrCurrent];
    ++ptrCurrent;
  }
}

/*!
  Modify the intensities of a color image using the look-up table passed in parameter.

  \param lut : Look-up table (vpRGBa array of size=256) which maps each intensity to his new value.
*/
template<>
inline void vpImage<vpRGBa>::performLut(const vpRGBa (&lut)[256]) {
  unsigned int size = getWidth()*getHeight();
  unsigned char *ptrStart = (unsigned char*) bitmap;
  unsigned char *ptrEnd = ptrStart + size*4;
  unsigned char *ptrCurrent = ptrStart;

  while(ptrCurrent != ptrEnd) {
    *ptrCurrent = lut[*ptrCurrent].R;
    ++ptrCurrent;

    *ptrCurrent = lut[*ptrCurrent].G;
    ++ptrCurrent;

    *ptrCurrent = lut[*ptrCurrent].B;
    ++ptrCurrent;

    *ptrCurrent = lut[*ptrCurrent].A;
    ++ptrCurrent;
  }
}

#endif
