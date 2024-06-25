/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpImage.h
 * \brief Image handling.
 */

#ifndef VP_IMAGE_H
#define VP_IMAGE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEndian.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpRGBf.h>

#if defined(VISP_HAVE_THREADS)
#include <thread>
#endif

#include <fstream>
#include <iomanip> // std::setw
#include <iostream>
#include <math.h>
#include <string.h>

// Visual Studio 2010 or previous is missing inttypes.h
#if defined(_MSC_VER) && (_MSC_VER < 1700)
typedef long long int64_t;
typedef unsigned short uint16_t;
#else
#include <inttypes.h>
#endif

BEGIN_VISP_NAMESPACE
class vpDisplay;
// Ref: http://en.cppreference.com/w/cpp/language/friend#Template_friends
template <class Type> class vpImage; // forward declare to make function declaration possible

// declarations
template <class Type> std::ostream &operator<<(std::ostream &s, const vpImage<Type> &I);

std::ostream &operator<<(std::ostream &s, const vpImage<unsigned char> &I);
std::ostream &operator<<(std::ostream &s, const vpImage<char> &I);
std::ostream &operator<<(std::ostream &s, const vpImage<float> &I);
std::ostream &operator<<(std::ostream &s, const vpImage<double> &I);

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

  Such a structure allows a fast access to each element of the image.
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
template <class Type> class vpImage
{
  friend class vpImageConvert;

public:
  Type *bitmap; //!< points toward the bitmap
  vpDisplay *display;

  //! constructor
  vpImage();
  //! copy constructor
  vpImage(const vpImage<Type> &img);
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  //! move constructor
  vpImage(vpImage<Type> &&img);
#endif
  //! constructor  set the size of the image
  vpImage(unsigned int height, unsigned int width);
  //! constructor  set the size of the image and init all the pixel
  vpImage(unsigned int height, unsigned int width, Type value);
  //! constructor from an image stored as a continuous array in memory
  vpImage(Type *const array, unsigned int height, unsigned int width, bool copyData = false);
  //! destructor
  virtual ~vpImage();

  /** @name Inherited functionalities from vpImage */
  //@{

  // destructor
  void destroy();

  // Returns a new image that's double size of the current image
  void doubleSizeImage(vpImage<Type> &res);

  /*!
    Get the number of columns in the image.

    \return The image number of column, or image width.

    \sa getWidth()
   */
  inline unsigned int getCols() const { return width; }

  /*!
    Get the image height.

    \return The image height.

    \sa getWidth()

  */
  inline unsigned int getHeight() const { return height; }

  // Return the maximum value within the bitmap
  Type getMaxValue(bool onlyFiniteVal = true) const;
  // Return the mean value of the bitmap
  double getMeanValue(const vpImage<bool> *p_mask = nullptr, unsigned int *nbValidPoints = nullptr) const;

  // Return the minumum value within the bitmap
  Type getMinValue(bool onlyFiniteVal = true) const;
  // Look for the minumum and the maximum value within the bitmap
  void getMinMaxValue(Type &min, Type &max, bool onlyFiniteVal = true) const;
  // Look for the minumum and the maximum value within the bitmap and get their location
  void getMinMaxLoc(vpImagePoint *minLoc, vpImagePoint *maxLoc, Type *minVal = nullptr, Type *maxVal = nullptr) const;

  /*!
    Get the image number of pixels which corresponds to the image
    width multiplied by the image height.

    \return The image number of pixels or image size.

    \sa getWidth(), getHeight()
   */
  inline unsigned int getNumberOfPixel() const { return npixels; }

  /*!
    Get the number of rows in the image.

    \return The image number of rows, or image height.

    \sa getHeight()
  */
  inline unsigned int getRows() const { return height; }

  /*!
    Get the image size.

    \return The image size = width * height.

    \sa getWidth(), getHeight()
   */
  inline unsigned int getSize() const { return width * height; }

  double getStdev(const vpImage<bool> *p_mask = nullptr, unsigned int *nbValidPoints = nullptr) const;
  double getStdev(const double &mean, const vpImage<bool> *p_mask = nullptr, unsigned int *nbValidPoints = nullptr) const;

  double getSum(const vpImage<bool> *p_mask = nullptr, unsigned int *nbValidPoints = nullptr) const;

  // Gets the value of a pixel at a location.
  Type getValue(unsigned int i, unsigned int j) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(double i, double j) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(const vpImagePoint &ip) const;

  /*!
    Get the image width.

    \return The image width.

    \sa getHeight()
  */
  inline unsigned int getWidth() const { return width; }

  // Returns a new image that's half size of the current image
  void halfSizeImage(vpImage<Type> &res) const;

  //! Set the size of the image
  void init(unsigned int height, unsigned int width);
  //! Set the size of the image and initialize all the elements to 'value'
  void init(unsigned int height, unsigned int width, Type value);
  //! Initialization from an image stored as a continuous array in memory
  void init(Type *const array, unsigned int height, unsigned int width, bool copyData = false);
  void insert(const vpImage<Type> &src, const vpImagePoint &topLeft);

  //------------------------------------------------------------------
  // Access to the image

  //! operator[] allows operation like I[i] = x.
  inline Type *operator[](unsigned int i) { return row[i]; }
  inline Type *operator[](int i) { return row[i]; }

  //! operator[] allows operation like x = I[i]
  inline const Type *operator[](unsigned int i) const { return row[i]; }
  inline const Type *operator[](int i) const { return row[i]; }

  /*!
    Get the value of an image point with coordinates (i, j), with i the row
    position and j the column position.

    \return Value of the image point (i, j).
  */
  inline Type operator()(unsigned int i, unsigned int j) const { return bitmap[(i * width) + j]; }

  /*!
    Set the value \e v of an image point with coordinates (i, j), with i the
    row position and j the column position.
  */
  inline void operator()(unsigned int i, unsigned int j, const Type &v) { bitmap[(i * width) + j] = v; }

  /*!
    Get the value of an image point.

    \param ip : An image point with sub-pixel coordinates. Sub-pixel
    coordinates are roughly transformed to insigned int coordinates by cast.

    \return Value of the image point \e ip.

    \sa getValue(const vpImagePoint &)
  */
  inline Type operator()(const vpImagePoint &ip) const
  {
    unsigned int i = static_cast<unsigned int>(ip.get_i());
    unsigned int j = static_cast<unsigned int>(ip.get_j());

    return bitmap[(i * width) + j];
  }

  /*!
    Set the value of an image point.

    \param ip : An image point with sub-pixel coordinates. Sub-pixel
    coordinates are roughly transformed to insigned int coordinates by cast.

    \param v : Value to set for the image point.
  */
  inline void operator()(const vpImagePoint &ip, const Type &v)
  {
    unsigned int i = static_cast<unsigned int>(ip.get_i());
    unsigned int j = static_cast<unsigned int>(ip.get_j());

    bitmap[(i * width) + j] = v;
  }

  vpImage<Type> operator-(const vpImage<Type> &B) const;

  //! Copy operator
  vpImage<Type> &operator=(vpImage<Type> other);

  vpImage<Type> &operator=(const Type &v);
  bool operator==(const vpImage<Type> &I) const;
  bool operator!=(const vpImage<Type> &I) const;
  friend std::ostream &operator<< <>(std::ostream &s, const vpImage<Type> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<unsigned char> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<char> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<float> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<double> &I);

  // Perform a look-up table transformation
  void performLut(const Type(&lut)[256], unsigned int nbThreads = 1);

  // Returns a new image that's a quarter size of the current image
  void quarterSizeImage(vpImage<Type> &res) const;

  // set the size of the image without initializing it.
  void resize(unsigned int h, unsigned int w);
  // set the size of the image and initialize it.
  void resize(unsigned int h, unsigned int w, const Type &val);

  void sub(const vpImage<Type> &B, vpImage<Type> &C) const;
  void sub(const vpImage<Type> &A, const vpImage<Type> &B, vpImage<Type> &C) const;
  void subsample(unsigned int v_scale, unsigned int h_scale, vpImage<Type> &sampled) const;

  // See https://stackoverflow.com/questions/11562/how-to-overload-stdswap to understand why swap is in visp namespace
  friend void swap(vpImage<Type> &first, vpImage<Type> &second)
  {
    using std::swap;
    swap(first.bitmap, second.bitmap);
    swap(first.display, second.display);
    swap(first.npixels, second.npixels);
    swap(first.width, second.width);
    swap(first.height, second.height);
    swap(first.row, second.row);
  }

  //@}

private:
  unsigned int npixels; ///! number of pixel in the image
  unsigned int width;   ///! number of columns
  unsigned int height;  ///! number of rows
  Type **row;           ///! points the row pointer array
  bool hasOwnership;    ///! true if this instance owns the bitmap, false otherwise (e.g. copyData=false)
};

#include <visp3/core/vpImage_operators.h>
#include <visp3/core/vpImage_lut.h>
#include <visp3/core/vpImage_getters.h>

/*!
  \relates vpImage
*/
template <class Type> void vpImage<Type>::init(unsigned int h, unsigned int w, Type value)
{
  init(h, w);
  std::fill(bitmap, bitmap + npixels, value);
}

/*!
  \relates vpImage
*/
template <class Type> void vpImage<Type>::init(unsigned int h, unsigned int w)
{
  if (h != this->height) {
    if (row != nullptr) {
      delete[] row;
      row = nullptr;
    }
  }

  if ((h != this->height) || (w != this->width)) {
    if (bitmap != nullptr) {
      if (hasOwnership) {
        delete[] bitmap;
      }
      bitmap = nullptr;
    }
  }

  this->width = w;
  this->height = h;

  npixels = width * height;

  if (bitmap == nullptr) {
    bitmap = new Type[npixels];
    hasOwnership = true;
  }
  if (bitmap == nullptr) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }
  if (row == nullptr) {
    row = new Type *[height];
  }
  if (row == nullptr) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate row "));
  }

  for (unsigned int i = 0; i < height; ++i) {
    row[i] = bitmap + (i * width);
  }
}

/*!
  \relates vpImage
*/
template <class Type> void vpImage<Type>::init(Type *const array, unsigned int h, unsigned int w, bool copyData)
{
  if (h != this->height) {
    if (row != nullptr) {
      delete[] row;
      row = nullptr;
    }
  }

  // Delete bitmap if copyData==false, otherwise only if the dimension differs
  if ((copyData && ((h != this->height) || (w != this->width))) || (!copyData)) {
    if (bitmap != nullptr) {
      if (hasOwnership) {
        delete[] bitmap;
      }
      bitmap = nullptr;
    }
  }

  hasOwnership = copyData;
  this->width = w;
  this->height = h;

  npixels = width * height;

  if (copyData) {
    if (bitmap == nullptr) {
      bitmap = new Type[npixels];
    }

    if (bitmap == nullptr) {
      throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
    }

    // Copy the image data
    memcpy(static_cast<void *>(bitmap), static_cast<void *>(array), static_cast<size_t>(npixels * sizeof(Type)));
  }
  else {
    // Copy the address of the array in the bitmap
    bitmap = array;
  }

  if (row == nullptr) {
    row = new Type *[height];
  }
  if (row == nullptr) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate row "));
  }

  for (unsigned int i = 0; i < height; ++i) {
    row[i] = bitmap + (i * width);
  }
}

/*!
  \relates vpImage
*/
template <class Type>
vpImage<Type>::vpImage(unsigned int h, unsigned int w)
  : bitmap(nullptr), display(nullptr), npixels(0), width(0), height(0), row(nullptr), hasOwnership(true)
{
  Type val(0);
  init(h, w, val);
}

/*!
  \relates vpImage
*/
template <class Type>
vpImage<Type>::vpImage(unsigned int h, unsigned int w, Type value)
  : bitmap(nullptr), display(nullptr), npixels(0), width(0), height(0), row(nullptr), hasOwnership(true)
{
  init(h, w, value);
}

/*!
  \relates vpImage
*/
template <class Type>
vpImage<Type>::vpImage(Type *const array, unsigned int h, unsigned int w, bool copyData)
  : bitmap(nullptr), display(nullptr), npixels(0), width(0), height(0), row(nullptr), hasOwnership(true)
{
  init(array, h, w, copyData);
}

/*!
  \relates vpImage
*/
template <class Type>
vpImage<Type>::vpImage() : bitmap(nullptr), display(nullptr), npixels(0), width(0), height(0), row(nullptr), hasOwnership(true)
{ }

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
template <class Type> void vpImage<Type>::resize(unsigned int h, unsigned int w) { init(h, w); }

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
template <class Type> void vpImage<Type>::resize(unsigned int h, unsigned int w, const Type &val) { init(h, w, val); }

/*!
  \brief Destructor : Memory de-allocation

  \warning does not deallocate memory for display and video

*/
template <class Type> void vpImage<Type>::destroy()
{
  if (bitmap != nullptr) {
    if (hasOwnership) {
      delete[] bitmap;
    }
    bitmap = nullptr;
  }

  if (row != nullptr) {
    delete[] row;
    row = nullptr;
  }
}

/*!
  \brief Destructor : Memory de-allocation

  \warning does not deallocate memory for display and video

*/
template <class Type> vpImage<Type>::~vpImage() { destroy(); }

/*!
  \relates vpImage
*/
template <class Type>
vpImage<Type>::vpImage(const vpImage<Type> &I)
  : bitmap(nullptr), display(nullptr), npixels(0), width(0), height(0), row(nullptr), hasOwnership(true)
{
  resize(I.getHeight(), I.getWidth());
  if (bitmap) {
    memcpy(static_cast<void *>(bitmap), static_cast<void *>(I.bitmap), I.npixels * sizeof(Type));
  }
}

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
/*!
  \relates vpImage
*/
template <class Type>
vpImage<Type>::vpImage(vpImage<Type> &&I)
  : bitmap(I.bitmap), display(I.display), npixels(I.npixels), width(I.width), height(I.height), row(I.row),
  hasOwnership(I.hasOwnership)
{
  I.bitmap = nullptr;
  I.display = nullptr;
  I.npixels = 0;
  I.width = 0;
  I.height = 0;
  I.row = nullptr;
  I.hasOwnership = false;
}
#endif

/*!
  Insert an image into another one.

  It is possible to insert the image \f$ src \f$ into the calling vpImage.
  You can set the point in the destination image where the top left corner of
  the \f$ src \f$ image will be located.

  \param src : Image to insert
  \param topLeft : Upper/left coordinates in the image where the image \e src
  is inserted in the destination image.
*/
template <class Type> void vpImage<Type>::insert(const vpImage<Type> &src, const vpImagePoint &topLeft)
{
  int itl = static_cast<int>(topLeft.get_i());
  int jtl = static_cast<int>(topLeft.get_j());

  int dest_ibegin = 0;
  int dest_jbegin = 0;
  int src_ibegin = 0;
  int src_jbegin = 0;
  int dest_w = static_cast<int>(this->getWidth());
  int dest_h = static_cast<int>(this->getHeight());
  int src_w = static_cast<int>(src.getWidth());
  int src_h = static_cast<int>(src.getHeight());
  int wsize = static_cast<int>(src.getWidth());
  int hsize = static_cast<int>(src.getHeight());

  if ((itl >= dest_h) || (jtl >= dest_w)) {
    return;
  }

  if (itl < 0) {
    src_ibegin = -itl;
  }
  else {
    dest_ibegin = itl;
  }

  if (jtl < 0) {
    src_jbegin = -jtl;
  }
  else {
    dest_jbegin = jtl;
  }

  if ((src_w - src_jbegin) >(dest_w - dest_jbegin)) {
    wsize = dest_w - dest_jbegin;
  }
  else {
    wsize = src_w - src_jbegin;
  }

  if ((src_h - src_ibegin) > (dest_h - dest_ibegin)) {
    hsize = dest_h - dest_ibegin;
  }
  else {
    hsize = src_h - src_ibegin;
  }

  for (int i = 0; i < hsize; ++i) {
    Type *srcBitmap = src.bitmap + (((src_ibegin + i) * src_w) + src_jbegin);
    Type *destBitmap = this->bitmap + (((dest_ibegin + i) * dest_w) + dest_jbegin);

    memcpy(static_cast<void *>(destBitmap), static_cast<void *>(srcBitmap), static_cast<size_t>(wsize) * sizeof(Type));
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

  \sa subsample()
*/
template <class Type> void vpImage<Type>::halfSizeImage(vpImage<Type> &res) const
{
  unsigned int h = height / 2;
  unsigned int w = width / 2;
  res.resize(h, w);
  for (unsigned int i = 0; i < h; ++i) {
    for (unsigned int j = 0; j < w; ++j) {
      res[i][j] = (*this)[i << 1][j << 1];
    }
  }
}

/*!
  Computes a subsampled image.
  No filtering is used during the sub sampling.

  \param v_scale [in] : Vertical subsampling factor.
  \param h_scale [in] : Horizontal subsampling factor.
  \param sampled [out] : Subsampled image.

  The example below shows how to use this method:
  \code
  vpImage<unsigned char> I; // original image
  vpImageIo::read(I, "myImage.pgm");
  vpImage<unsigned char> I2; // half size image
  I.subsample(2, 2, I2);
  vpImageIo::write(I2, "myHalfSizeImage.pgm");
  \endcode
*/
template <class Type>
void vpImage<Type>::subsample(unsigned int v_scale, unsigned int h_scale, vpImage<Type> &sampled) const
{
  if ((v_scale == 1) && (h_scale == 1)) {
    sampled = *this;
    return;
  }
  unsigned int h = height / v_scale;
  unsigned int w = width / h_scale;
  sampled.resize(h, w);
  for (unsigned int i = 0; i < h; ++i) {
    for (unsigned int j = 0; j < w; ++j) {
      sampled[i][j] = (*this)[i * v_scale][j * h_scale];
    }
  }
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

  \sa subsample()
*/
template <class Type> void vpImage<Type>::quarterSizeImage(vpImage<Type> &res) const
{
  unsigned int h = height / 4;
  unsigned int w = width / 4;
  const unsigned int magic_2 = 2;
  res.resize(h, w);
  for (unsigned int i = 0; i < h; ++i) {
    for (unsigned int j = 0; j < w; ++j) {
      res[i][j] = (*this)[i << magic_2][j << magic_2];
    }
  }
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
template <class Type> void vpImage<Type>::doubleSizeImage(vpImage<Type> &res)
{
  const unsigned int magic_2 = 2;
  unsigned int h = height * magic_2;
  unsigned int w = width * magic_2;

  res.resize(h, w);

  for (unsigned int i = 0; i < h; ++i) {
    for (unsigned int j = 0; j < w; ++j) {
      res[i][j] = (*this)[i >> 1][j >> 1];
    }
  }

  /*
    A B C
    E F G
    H I J
    A C H J are pixels from original image
    B E G I are interpolated pixels
  */

  // interpolate pixels B and I
  for (unsigned int i = 0; i < h; i += magic_2) {
    for (unsigned int j = 1; j < (w - 1); j += magic_2) {
      res[i][j] = static_cast<Type>(0.5 * ((*this)[i >> 1][j >> 1] + (*this)[i >> 1][(j >> 1) + 1]));
    }
  }

  // interpolate pixels E and G
  for (unsigned int i = 1; i < (h - 1); i += magic_2) {
    for (unsigned int j = 0; j < w; j += magic_2) {
      res[i][j] = static_cast<Type>(0.5 * ((*this)[i >> 1][j >> 1] + (*this)[(i >> 1) + 1][j >> 1]));
    }
  }

  // interpolate pixel F
  for (unsigned int i = 1; i < (h - 1); i += magic_2) {
    for (unsigned int j = 1; j < (w - 1); j += magic_2) {
      res[i][j] = static_cast<Type>(0.25 * ((*this)[i >> 1][j >> 1] + (*this)[i >> 1][(j >> 1) + 1] +
                                            (*this)[(i >> 1) + 1][j >> 1] + (*this)[(i >> 1) + 1][(j >> 1) + 1]));
    }
  }
}

/*!
  Operation C = *this - B.

  \code
  #include <visp3/core/vpImage.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
template <class Type> void vpImage<Type>::sub(const vpImage<Type> &B, vpImage<Type> &C) const
{

  try {
    if ((this->getHeight() != C.getHeight()) || (this->getWidth() != C.getWidth())) {
      C.resize(this->getHeight(), this->getWidth());
    }
  }
  catch (const vpException &me) {
    std::cout << me << std::endl;
    throw;
  }

  if ((this->getWidth() != B.getWidth()) || (this->getHeight() != B.getHeight())) {
    throw(vpException(vpException::memoryAllocationError, "vpImage mismatch in vpImage/vpImage subtraction"));
  }

  unsigned int this_width = this->getWidth();
  unsigned int this_height = this->getHeight();
  for (unsigned int i = 0; i < (this_width * this_height); ++i) {
    *(C.bitmap + i) = *(bitmap + i) - *(B.bitmap + i);
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
template <class Type> void vpImage<Type>::sub(const vpImage<Type> &A, const vpImage<Type> &B, vpImage<Type> &C) const
{

  try {
    if ((A.getHeight() != C.getHeight()) || (A.getWidth() != C.getWidth())) {
      C.resize(A.getHeight(), A.getWidth());
    }
  }
  catch (const vpException &me) {
    std::cout << me << std::endl;
    throw;
  }

  if ((A.getWidth() != B.getWidth()) || (A.getHeight() != B.getHeight())) {
    throw(vpException(vpException::memoryAllocationError, "vpImage mismatch in vpImage/vpImage subtraction "));
  }

  unsigned int a_width = A.getWidth();
  unsigned int a_height = A.getHeight();
  for (unsigned int i = 0; i < (a_width * a_height); ++i) {
    *(C.bitmap + i) = *(A.bitmap + i) - *(B.bitmap + i);
  }
}

END_VISP_NAMESPACE
#endif
