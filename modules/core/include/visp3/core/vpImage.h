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
#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
#include <visp3/core/vpThread.h>
#endif

#include <fstream>
#include <iomanip> // std::setw
#include <iostream>
#include <math.h>
#include <string.h>
#include <inttypes.h>

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

// Ref: http://en.cppreference.com/w/cpp/language/friend#Template_friends
template <class Type> class vpImage; // forward declare to make function declaration possible

// declarations
template <class Type> std::ostream &operator<<(std::ostream &, const vpImage<Type> &);

std::ostream &operator<<(std::ostream &, const vpImage<unsigned char> &);
std::ostream &operator<<(std::ostream &, const vpImage<char> &);
std::ostream &operator<<(std::ostream &, const vpImage<float> &);
std::ostream &operator<<(std::ostream &, const vpImage<double> &);

template <class Type> void swap(vpImage<Type> &first, vpImage<Type> &second);

template <class Type> class vpImage
{
  friend class vpImageConvert;

public:
  Type *bitmap; //!< points toward the bitmap
  vpDisplay *display;

  //! constructor
  vpImage();
  //! copy constructor
  vpImage(const vpImage<Type> &);
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  //! move constructor
  vpImage(vpImage<Type> &&);
#endif
  //! constructor  set the size of the image
  vpImage(unsigned int height, unsigned int width);
  //! constructor  set the size of the image and init all the pixel
  vpImage(unsigned int height, unsigned int width, Type value);
  //! constructor from an image stored as a continuous array in memory
  vpImage(Type *const array, const unsigned int height, const unsigned int width, const bool copyData = false);
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
  Type getMaxValue() const;
  // Return the mean value of the bitmap
  Type getMeanValue() const;
  // Return the minumum value within the bitmap
  Type getMinValue() const;
  // Look for the minumum and the maximum value within the bitmap
  void getMinMaxValue(Type &min, Type &max) const;
  // Look for the minumum and the maximum value within the bitmap and get their location
  void getMinMaxLoc(vpImagePoint *minLoc, vpImagePoint *maxLoc, Type *minVal = NULL, Type *maxVal = NULL) const;

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

  // Gets the value of a pixel at a location.
  Type getValue(unsigned int i, unsigned int j) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(double i, double j) const;
  // Gets the value of a pixel at a location with bilinear interpolation.
  Type getValue(const vpImagePoint &ip) const;

  // Get image pixels sum
  double getSum() const;

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
  //! Set the size of the image
  void init(unsigned int height, unsigned int width, Type value);
  //! init from an image stored as a continuous array in memory
  void init(Type *const array, const unsigned int height, const unsigned int width, const bool copyData = false);
  void insert(const vpImage<Type> &src, const vpImagePoint &topLeft);

  //------------------------------------------------------------------
  //         Acces to the image

  //! operator[] allows operation like I[i] = x.
  inline Type *operator[](const unsigned int i) { return row[i]; }
  inline Type *operator[](const int i) { return row[i]; }

  //! operator[] allows operation like x = I[i]
  inline const Type *operator[](unsigned int i) const { return row[i]; }
  inline const Type *operator[](int i) const { return row[i]; }

  /*!
    Get the value of an image point with coordinates (i, j), with i the row
    position and j the column position.

    \return Value of the image point (i, j).
  */
  inline Type operator()(const unsigned int i, const unsigned int j) const { return bitmap[i * width + j]; }

  /*!
    Set the value \e v of an image point with coordinates (i, j), with i the
    row position and j the column position.
  */
  inline void operator()(const unsigned int i, const unsigned int j, const Type &v) { bitmap[i * width + j] = v; }

  /*!
    Get the value of an image point.

    \param ip : An image point with sub-pixel coordinates. Sub-pixel
    coordinates are roughly transformed to insigned int coordinates by cast.

    \return Value of the image point \e ip.

    \sa getValue(const vpImagePoint &)
  */
  inline Type operator()(const vpImagePoint &ip) const
  {
    unsigned int i = (unsigned int)ip.get_i();
    unsigned int j = (unsigned int)ip.get_j();

    return bitmap[i * width + j];
  }

  /*!
    Set the value of an image point.

    \param ip : An image point with sub-pixel coordinates. Sub-pixel
    coordinates are roughly transformed to insigned int coordinates by cast.

    \param v : Value to set for the image point.
  */
  inline void operator()(const vpImagePoint &ip, const Type &v)
  {
    unsigned int i = (unsigned int)ip.get_i();
    unsigned int j = (unsigned int)ip.get_j();

    bitmap[i * width + j] = v;
  }

  vpImage<Type> operator-(const vpImage<Type> &B);

  //! Copy operator
  vpImage<Type> &operator=(vpImage<Type> other);

  vpImage<Type> &operator=(const Type &v);
  bool operator==(const vpImage<Type> &I);
  bool operator!=(const vpImage<Type> &I);
  friend std::ostream &operator<< <>(std::ostream &s, const vpImage<Type> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<unsigned char> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<char> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<float> &I);
  friend std::ostream &operator<<(std::ostream &s, const vpImage<double> &I);

  // Perform a look-up table transformation
  void performLut(const Type (&lut)[256], const unsigned int nbThreads = 1);

  // Returns a new image that's a quarter size of the current image
  void quarterSizeImage(vpImage<Type> &res) const;

  // set the size of the image without initializing it.
  void resize(const unsigned int h, const unsigned int w);
  // set the size of the image and initialize it.
  void resize(const unsigned int h, const unsigned int w, const Type &val);

  void sub(const vpImage<Type> &B, vpImage<Type> &C);
  void sub(const vpImage<Type> &A, const vpImage<Type> &B, vpImage<Type> &C);
  void subsample(unsigned int v_scale, unsigned int h_scale, vpImage<Type> &sampled) const;

  friend void swap<>(vpImage<Type> &first, vpImage<Type> &second);

  //@}

private:
  unsigned int npixels; ///! number of pixel in the image
  unsigned int width;   ///! number of columns
  unsigned int height;  ///! number of rows
  Type **row;           ///! points the row pointer array
};

template <class Type> std::ostream &operator<<(std::ostream &s, const vpImage<Type> &I)
{
  if (I.bitmap == NULL) {
    return s;
  }

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth() - 1; j++) {
      s << I[i][j] << " ";
    }

    // We don't add "  " after the last column element
    s << I[i][I.getWidth() - 1];

    // We don't add a \n character at the end of the last row line
    if (i < I.getHeight() - 1) {
      s << std::endl;
    }
  }

  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<unsigned char> &I)
{
  if (I.bitmap == NULL) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth() - 1; j++) {
      s << std::setw(3) << static_cast<unsigned>(I[i][j]) << " ";
    }

    // We don't add "  " after the last column element
    s << std::setw(3) << static_cast<unsigned>(I[i][I.getWidth() - 1]);

    // We don't add a \n character at the end of the last row line
    if (i < I.getHeight() - 1) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<char> &I)
{
  if (I.bitmap == NULL) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth() - 1; j++) {
      s << std::setw(4) << static_cast<int>(I[i][j]) << " ";
    }

    // We don't add "  " after the last column element
    s << std::setw(4) << static_cast<int>(I[i][I.getWidth() - 1]);

    // We don't add a \n character at the end of the last row line
    if (i < I.getHeight() - 1) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<float> &I)
{
  if (I.bitmap == NULL) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();
  s.precision(9); // http://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth() - 1; j++) {
      s << I[i][j] << " ";
    }

    // We don't add "  " after the last column element
    s << I[i][I.getWidth() - 1];

    // We don't add a \n character at the end of the last row line
    if (i < I.getHeight() - 1) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<double> &I)
{
  if (I.bitmap == NULL) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();
  s.precision(17); // http://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth() - 1; j++) {
      s << I[i][j] << " ";
    }

    // We don't add "  " after the last column element
    s << I[i][I.getWidth() - 1];

    // We don't add a \n character at the end of the last row line
    if (i < I.getHeight() - 1) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
namespace
{
struct ImageLut_Param_t {
  unsigned int m_start_index;
  unsigned int m_end_index;

  unsigned char m_lut[256];
  unsigned char *m_bitmap;

  ImageLut_Param_t() : m_start_index(0), m_end_index(0), m_lut(), m_bitmap(NULL) {}

  ImageLut_Param_t(const unsigned int start_index, const unsigned int end_index, unsigned char *bitmap)
    : m_start_index(start_index), m_end_index(end_index), m_lut(), m_bitmap(bitmap)
  {
  }
};

vpThread::Return performLutThread(vpThread::Args args)
{
  ImageLut_Param_t *imageLut_param = static_cast<ImageLut_Param_t *>(args);
  unsigned int start_index = imageLut_param->m_start_index;
  unsigned int end_index = imageLut_param->m_end_index;

  unsigned char *bitmap = imageLut_param->m_bitmap;

  unsigned char *ptrStart = bitmap + start_index;
  unsigned char *ptrEnd = bitmap + end_index;
  unsigned char *ptrCurrent = ptrStart;

  //    while(ptrCurrent != ptrEnd) {
  //      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
  //      ++ptrCurrent;
  //    }

  if (end_index - start_index >= 8) {
    // Unroll loop version
    for (; ptrCurrent <= ptrEnd - 8;) {
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;
    }
  }

  for (; ptrCurrent != ptrEnd; ++ptrCurrent) {
    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
  }

  return 0;
}

struct ImageLutRGBa_Param_t {
  unsigned int m_start_index;
  unsigned int m_end_index;

  vpRGBa m_lut[256];
  unsigned char *m_bitmap;

  ImageLutRGBa_Param_t() : m_start_index(0), m_end_index(0), m_lut(), m_bitmap(NULL) {}

  ImageLutRGBa_Param_t(const unsigned int start_index, const unsigned int end_index, unsigned char *bitmap)
    : m_start_index(start_index), m_end_index(end_index), m_lut(), m_bitmap(bitmap)
  {
  }
};

vpThread::Return performLutRGBaThread(vpThread::Args args)
{
  ImageLutRGBa_Param_t *imageLut_param = static_cast<ImageLutRGBa_Param_t *>(args);
  unsigned int start_index = imageLut_param->m_start_index;
  unsigned int end_index = imageLut_param->m_end_index;

  unsigned char *bitmap = imageLut_param->m_bitmap;

  unsigned char *ptrStart = bitmap + start_index * 4;
  unsigned char *ptrEnd = bitmap + end_index * 4;
  unsigned char *ptrCurrent = ptrStart;

  if (end_index - start_index >= 4 * 2) {
    // Unroll loop version
    for (; ptrCurrent <= ptrEnd - 4 * 2;) {
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].R;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].G;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].B;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].A;
      ptrCurrent++;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].R;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].G;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].B;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].A;
      ptrCurrent++;
    }
  }

  while (ptrCurrent != ptrEnd) {
    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].R;
    ptrCurrent++;

    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].G;
    ptrCurrent++;

    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].B;
    ptrCurrent++;

    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].A;
    ptrCurrent++;
  }

  return 0;
}
}
#endif

/*!
  \brief Image initialisation

  Allocate memory for an [h x w] image.

  \param w : Image width.
  \param h : Image height.
  \param value : Set all the element of the bitmap to \e value.

  \exception vpException::memoryAllocationError

  \sa vpImage::init(h, w)
*/
template <class Type> void vpImage<Type>::init(unsigned int h, unsigned int w, Type value)
{
  init(h, w);

  //  for (unsigned int i = 0; i < npixels; i++)
  //    bitmap[i] = value;
  std::fill(bitmap, bitmap + npixels, value);
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
template <class Type> void vpImage<Type>::init(unsigned int h, unsigned int w)
{
  if (h != this->height) {
    if (row != NULL) {
      vpDEBUG_TRACE(10, "Destruction row[]");
      delete[] row;
      row = NULL;
    }
  }

  if ((h != this->height) || (w != this->width)) {
    if (bitmap != NULL) {
      vpDEBUG_TRACE(10, "Destruction bitmap[]");
      delete[] bitmap;
      bitmap = NULL;
    }
  }

  this->width = w;
  this->height = h;

  npixels = width * height;

  if (bitmap == NULL)
    bitmap = new Type[npixels];

  if (bitmap == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
  }

  if (row == NULL)
    row = new Type *[height];
  if (row == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate row "));
  }

  for (unsigned int i = 0; i < height; i++)
    row[i] = bitmap + i * width;
}

/*!
  \brief Image initialization

  Init from image data stored as a continuous array in memory.

  \param array : Image data stored as a continuous array in memory
  \param h : Image height.
  \param w : Image width.
  \param copyData : If false (by default) only the memory address is copied,
  otherwise the data are copied.

  \exception vpException::memoryAllocationError
*/
template <class Type>
void vpImage<Type>::init(Type *const array, const unsigned int h, const unsigned int w, const bool copyData)
{
  if (h != this->height) {
    if (row != NULL) {
      delete[] row;
      row = NULL;
    }
  }

  // Delete bitmap if copyData==false, otherwise only if the dimension differs
  if ((copyData && ((h != this->height) || (w != this->width))) || !copyData) {
    if (bitmap != NULL) {
      delete[] bitmap;
      bitmap = NULL;
    }
  }

  this->width = w;
  this->height = h;

  npixels = width * height;

  if (copyData) {
    if (bitmap == NULL)
      bitmap = new Type[npixels];

    if (bitmap == NULL) {
      throw(vpException(vpException::memoryAllocationError, "cannot allocate bitmap "));
    }

    // Copy the image data
    memcpy(static_cast<void*>(bitmap), static_cast<void*>(array), (size_t)(npixels * sizeof(Type)));
  } else {
    // Copy the address of the array in the bitmap
    bitmap = array;
  }

  if (row == NULL)
    row = new Type *[height];
  if (row == NULL) {
    throw(vpException(vpException::memoryAllocationError, "cannot allocate row "));
  }

  for (unsigned int i = 0; i < height; i++) {
    row[i] = bitmap + i * width;
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
template <class Type>
vpImage<Type>::vpImage(unsigned int h, unsigned int w)
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  init(h, w, 0);
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
template <class Type>
vpImage<Type>::vpImage(unsigned int h, unsigned int w, Type value)
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  init(h, w, value);
}

/*!
  \brief Constructor

  Construct a vpImage from a continuous array in memory.

  \param array : Image data stored as a continuous array in memory.
  \param h : Image height.
  \param w : Image width.
  \param copyData : If false (by default) only the memory address is copied,
  otherwise the data are copied.

  \return MEMORY_FAULT if memory allocation is impossible, else OK

  \sa vpImage::init(array, height, width)
*/
template <class Type>
vpImage<Type>::vpImage(Type *const array, const unsigned int h, const unsigned int w, const bool copyData)
  : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  init(array, h, w, copyData);
}

/*!
  \brief Constructor

  No memory allocation is done

  set all the element of the bitmap to value

  \sa vpImage::resize(height, width) for memory allocation
*/
template <class Type> vpImage<Type>::vpImage() : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
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
  //   vpERROR_TRACE("Deallocate ");

  if (bitmap != NULL) {
    //  vpERROR_TRACE("Deallocate bitmap memory %p",bitmap);
    //    vpDEBUG_TRACE(20,"Deallocate bitmap memory %p",bitmap);
    delete[] bitmap;
    bitmap = NULL;
  }

  if (row != NULL) {
    //   vpERROR_TRACE("Deallocate row memory %p",row);
    //    vpDEBUG_TRACE(20,"Deallocate row memory %p",row);
    delete[] row;
    row = NULL;
  }
}

/*!
  \brief Destructor : Memory de-allocation

  \warning does not deallocate memory for display and video

*/
template <class Type> vpImage<Type>::~vpImage() { destroy(); }

/*!
  Copy constructor
*/
template <class Type>
vpImage<Type>::vpImage(const vpImage<Type> &I) : bitmap(NULL), display(NULL), npixels(0), width(0), height(0), row(NULL)
{
  resize(I.getHeight(), I.getWidth());
  memcpy(static_cast<void*>(bitmap), static_cast<void*>(I.bitmap), I.npixels * sizeof(Type));
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
/*!
  Move constructor
*/
template <class Type>
vpImage<Type>::vpImage(vpImage<Type> &&I)
  : bitmap(I.bitmap), display(I.display), npixels(I.npixels), width(I.width), height(I.height), row(I.row)
{
  I.bitmap = NULL;
  I.display = NULL;
  I.npixels = 0;
  I.width = 0;
  I.height = 0;
  I.row = NULL;
}
#endif

/*!
  \brief Return the maximum value within the bitmap

  \sa getMinValue()
*/
template <class Type> Type vpImage<Type>::getMaxValue() const
{
  if (npixels == 0)
    throw(vpException(vpException::fatalError, "Cannot compute maximum value of an empty image"));
  Type m = bitmap[0];
  for (unsigned int i = 0; i < npixels; i++) {
    if (bitmap[i] > m)
      m = bitmap[i];
  }
  return m;
}

/*!
  \brief Return the mean value of the bitmap
*/
template <class Type> Type vpImage<Type>::getMeanValue() const
{
  if ((height == 0) || (width == 0))
    return 0.0;

  return getSum() / (height * width);
}

/*!
  \brief Return the minimum value within the bitmap

  \sa getMaxValue()
*/
template <class Type> Type vpImage<Type>::getMinValue() const
{
  if (npixels == 0)
    throw(vpException(vpException::fatalError, "Cannot compute minimum value of an empty image"));
  Type m = bitmap[0];
  for (unsigned int i = 0; i < npixels; i++)
    if (bitmap[i] < m)
      m = bitmap[i];
  return m;
}

/*!
  \brief Look for the minimum and the maximum value within the bitmap

  \sa getMaxValue()
  \sa getMinValue()
  \sa getMinMaxLoc()
*/
template <class Type> void vpImage<Type>::getMinMaxValue(Type &min, Type &max) const
{
  if (npixels == 0)
    throw(vpException(vpException::fatalError, "Cannot get minimum/maximum values of an empty image"));

  min = max = bitmap[0];
  for (unsigned int i = 0; i < npixels; i++) {
    if (bitmap[i] < min)
      min = bitmap[i];
    if (bitmap[i] > max)
      max = bitmap[i];
  }
}

/*!
  \brief Get the position of the minimum and/or the maximum pixel value within the bitmap and
  the corresponding value.
  Following code allows retrieving only minimum value and position:
  \code
  vpImage<double> I(h, w);
  //[...] Fill I
  vpImagePoint min_loc;
  double min_val = 0.0;
  I.getMinMaxLoc(&min_loc, NULL, &min_val, NULL);
  \endcode

  \param minLoc : Position of the pixel with minimum value if not NULL.
  \param maxLoc : Position of the pixel with maximum value if not NULL.
  \param minVal : Minimum pixel value if not NULL.
  \param maxVal : Maximum pixel value if not NULL.

  \sa getMaxValue()
  \sa getMinValue()
  \sa getMinMaxValue()
*/
template <class Type>
void vpImage<Type>::getMinMaxLoc(vpImagePoint *minLoc, vpImagePoint *maxLoc, Type *minVal, Type *maxVal) const
{
  if (npixels == 0)
    throw(vpException(vpException::fatalError, "Cannot get location of minimum/maximum "
                                               "values of an empty image"));

  Type min = bitmap[0], max = bitmap[0];
  vpImagePoint minLoc_, maxLoc_;
  for (unsigned int i = 0; i < height; i++) {
    for (unsigned int j = 0; j < width; j++) {
      if (row[i][j] < min) {
        min = row[i][j];
        minLoc_.set_ij(i, j);
      }

      if (row[i][j] > max) {
        max = row[i][j];
        maxLoc_.set_ij(i, j);
      }
    }
  }

  if (minLoc != NULL)
    *minLoc = minLoc_;

  if (maxLoc != NULL)
    *maxLoc = maxLoc_;

  if (minVal != NULL)
    *minVal = min;

  if (maxVal != NULL)
    *maxVal = max;
}

/*!
  \brief Copy operator
*/
template <class Type> vpImage<Type> &vpImage<Type>::operator=(vpImage<Type> other)
{
  swap(*this, other);
  // Swap back display pointer if it was not null
  // vpImage<unsigned char> I2(480, 640);
  // vpDisplayX d(I2);
  // I2 = I1; //copy only the data
  if (other.display != NULL)
    display = other.display;

  return *this;
}

/*!
  \brief = operator : Set all the element of the bitmap to a given  value \e
  v. \f$ A = v <=> A[i][j] = v \f$

   \warning = must be defined for \f$ <\f$ Type \f$ > \f$
*/
template <class Type> vpImage<Type> &vpImage<Type>::operator=(const Type &v)
{
  for (unsigned int i = 0; i < npixels; i++)
    bitmap[i] = v;

  return *this;
}

/*!
  Compare two images.

  \return true if the images are the same, false otherwise.
*/
template <class Type> bool vpImage<Type>::operator==(const vpImage<Type> &I)
{
  if (this->width != I.getWidth())
    return false;
  if (this->height != I.getHeight())
    return false;

  //  printf("wxh: %dx%d bitmap: %p I.bitmap %p\n", width, height, bitmap,
  //  I.bitmap);
  for (unsigned int i = 0; i < npixels; i++) {
    if (bitmap[i] != I.bitmap[i]) {
      //      std::cout << "differ for pixel " << i << " (" << i%this->height
      //      << ", " << i - i%this->height << ")" << std::endl;
      return false;
    }
  }
  return true;
}
/*!
  Compare two images.

  \return true if the images are different, false if they are the same.
*/
template <class Type> bool vpImage<Type>::operator!=(const vpImage<Type> &I)
{
  return !(*this == I);
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
template <class Type> vpImage<Type> vpImage<Type>::operator-(const vpImage<Type> &B)
{
  vpImage<Type> C;
  sub(*this, B, C);
  return C;
}

/*!
  Insert an image into another one.

  It is possible to insert the image \f$ src \f$ into the calling vpImage.
  You can set the point in the destination image where the top left corner of
  the \f$ src \f$ image will belocated.

  \param src : Image to insert
  \param topLeft : Upper/left coordinates in the image where the image \e src
  is inserted in the destination image.
*/
template <class Type> void vpImage<Type>::insert(const vpImage<Type> &src, const vpImagePoint &topLeft)
{
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

  for (int i = 0; i < hsize; i++) {
    Type *srcBitmap = src.bitmap + ((src_ibegin + i) * src_w + src_jbegin);
    Type *destBitmap = this->bitmap + ((dest_ibegin + i) * dest_w + dest_jbegin);

    memcpy(static_cast<void*>(destBitmap), static_cast<void*>(srcBitmap), (size_t)wsize * sizeof(Type));
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
  for (unsigned int i = 0; i < h; i++)
    for (unsigned int j = 0; j < w; j++)
      res[i][j] = (*this)[i << 1][j << 1];
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
  unsigned int h = height / v_scale;
  unsigned int w = width / h_scale;
  sampled.resize(h, w);
  for (unsigned int i = 0; i < h; i++)
    for (unsigned int j = 0; j < w; j++)
      sampled[i][j] = (*this)[i * v_scale][j * h_scale];
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
  res.resize(h, w);
  for (unsigned int i = 0; i < h; i++)
    for (unsigned int j = 0; j < w; j++)
      res[i][j] = (*this)[i << 2][j << 2];
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
  int h = height * 2;
  int w = width * 2;

  res.resize(h, w);

  for (int i = 0; i < h; i++)
    for (int j = 0; j < w; j++)
      res[i][j] = (*this)[i >> 1][j >> 1];

  /*
    A B C
    E F G
    H I J
    A C H J are pixels from original image
    B E G I are interpolated pixels
  */

  // interpolate pixels B and I
  for (int i = 0; i < h; i += 2)
    for (int j = 1; j < w - 1; j += 2)
      res[i][j] = (Type)(0.5 * ((*this)[i >> 1][j >> 1] + (*this)[i >> 1][(j >> 1) + 1]));

  // interpolate pixels E and G
  for (int i = 1; i < h - 1; i += 2)
    for (int j = 0; j < w; j += 2)
      res[i][j] = (Type)(0.5 * ((*this)[i >> 1][j >> 1] + (*this)[(i >> 1) + 1][j >> 1]));

  // interpolate pixel F
  for (int i = 1; i < h - 1; i += 2)
    for (int j = 1; j < w - 1; j += 2)
      res[i][j] = (Type)(0.25 * ((*this)[i >> 1][j >> 1] + (*this)[i >> 1][(j >> 1) + 1] +
                                 (*this)[(i >> 1) + 1][j >> 1] + (*this)[(i >> 1) + 1][(j >> 1) + 1]));
}

/*!
  Retrieves pixel value from an image containing values of type \e Type

  Gets the value of a sub-pixel with coordinates (i,j).

  \param i : Pixel coordinate along the rows.
  \param j : Pixel coordinate along the columns.

  \return Pixel value.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.
*/
template <class Type> inline Type vpImage<Type>::getValue(unsigned int i, unsigned int j) const
{
  if (i >= height || j >= width) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside the image"));
  }

  return row[i][j];
}

/*!
  Retrieves pixel value from an image containing values of type \e Type with
  sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation.

  See also vpImageTools::interpolate() for a similar result, but with a choice of the interpolation method.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.
*/
template <class Type> Type vpImage<Type>::getValue(double i, double j) const
{
  if (i < 0 || j < 0 || i+1 > height || j+1 > width) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if (height * width == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = (std::min)(height - 1, iround + 1);
  unsigned int jround_1 = (std::min)(width - 1, jround + 1);

  double value = (static_cast<double>(row[iround][jround]) * rfrac + static_cast<double>(row[iround_1][jround]) * rratio) * cfrac +
                 (static_cast<double>(row[iround][jround_1]) * rfrac + static_cast<double>(row[iround_1][jround_1]) * rratio) * cratio;

  return static_cast<Type>(vpMath::round(value));
}

/*!
  Retrieves pixel value from an image of double with sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation.

  See also vpImageTools::interpolate() for a similar result, but with a choice of the interpolation method.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out
  of the image.
*/
template <> inline double vpImage<double>::getValue(double i, double j) const
{
  if (i < 0 || j < 0 || i+1 > height || j+1 > width) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if (height * width == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = (std::min)(height - 1, iround + 1);
  unsigned int jround_1 = (std::min)(width - 1, jround + 1);

  return (row[iround][jround] * rfrac + row[iround_1][jround] * rratio) * cfrac +
         (row[iround][jround_1] * rfrac + row[iround_1][jround_1] * rratio) * cratio;
}

template <> inline unsigned char vpImage<unsigned char>::getValue(double i, double j) const {
  if (i < 0 || j < 0 || i+1 > height || j+1 > width) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if (height * width == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  //Fixed-point arithmetic
  const int precision = 1 << 16;
  int64_t y = static_cast<int64_t>(i * precision);
  int64_t x = static_cast<int64_t>(j * precision);

  int64_t iround = y & (~0xFFFF);
  int64_t jround = x & (~0xFFFF);

  int64_t rratio = y - iround;
  int64_t cratio = x - jround;

  int64_t rfrac = precision - rratio;
  int64_t cfrac = precision - cratio;

  int64_t x_ = x >> 16;
  int64_t y_ = y >> 16;

  if (y_ + 1 < height && x_ + 1 < width) {
    uint16_t up = *reinterpret_cast<uint16_t *>(bitmap + y_ * width + x_);
    uint16_t down = *reinterpret_cast<uint16_t *>(bitmap + (y_ + 1) * width + x_);

    return static_cast<unsigned char>((((up & 0x00FF) * rfrac + (down & 0x00FF) * rratio) * cfrac +
                                       ((up >> 8) * rfrac + (down >> 8) * rratio) * cratio) >> 32);
  } else if (y_ + 1 < height) {
    return static_cast<unsigned char>(((row[y_][x_] * rfrac + row[y_ + 1][x_] * rratio)) >> 16);
  } else if (x_ + 1 < width) {
    uint16_t up = *reinterpret_cast<uint16_t *>(bitmap + y_ * width + x_);
    return static_cast<unsigned char>(((up & 0x00FF) * cfrac + (up >> 8) * cratio) >> 16);
  } else {
    return row[y_][x_];
  }
}

template <> inline vpRGBa vpImage<vpRGBa>::getValue(double i, double j) const
{
  if (i < 0 || j < 0 || i+1 > height || j+1 > width) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if (height * width == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = (std::min)(height - 1, iround + 1);
  unsigned int jround_1 = (std::min)(width - 1, jround + 1);

  double valueR = (static_cast<double>(row[iround][jround].R) * rfrac + static_cast<double>(row[iround_1][jround].R) * rratio) * cfrac +
                  (static_cast<double>(row[iround][jround_1].R) * rfrac + static_cast<double>(row[iround_1][jround_1].R) * rratio) * cratio;
  double valueG = (static_cast<double>(row[iround][jround].G) * rfrac + static_cast<double>(row[iround_1][jround].G) * rratio) * cfrac +
                  (static_cast<double>(row[iround][jround_1].G) * rfrac + static_cast<double>(row[iround_1][jround_1].G) * rratio) * cratio;
  double valueB = (static_cast<double>(row[iround][jround].B) * rfrac + static_cast<double>(row[iround_1][jround].B) * rratio) * cfrac +
                  (static_cast<double>(row[iround][jround_1].B) * rfrac + static_cast<double>(row[iround_1][jround_1].B) * rratio) * cratio;

  return vpRGBa(static_cast<unsigned char>(vpMath::round(valueR)),
                static_cast<unsigned char>(vpMath::round(valueG)),
                static_cast<unsigned char>(vpMath::round(valueB)));
}

/*!
  Retrieves pixel value from an image containing values of type \e Type with
  sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation.

  See also vpImageTools::interpolate() for a similar result, but with a choice of the interpolation method.

  \param ip : Sub-pixel coordinates of a point in the image.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If the image point \e ip is out
  of the image.
*/
template <class Type> inline Type vpImage<Type>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

template <> inline double vpImage<double>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

template <> inline unsigned char vpImage<unsigned char>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

template <> inline vpRGBa vpImage<vpRGBa>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

/**
* Compute the sum of image intensities.
*/
template <class Type> inline double vpImage<Type>::getSum() const
{
  if ((height == 0) || (width == 0))
    return 0.0;

  double res = 0.0;
  for (unsigned int i = 0; i < height * width; ++i) {
    res += static_cast<double>(bitmap[i]);
  }
  return res;
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
template <class Type> void vpImage<Type>::sub(const vpImage<Type> &B, vpImage<Type> &C)
{

  try {
    if ((this->getHeight() != C.getHeight()) || (this->getWidth() != C.getWidth()))
      C.resize(this->getHeight(), this->getWidth());
  } catch (const vpException &me) {
    std::cout << me << std::endl;
    throw;
  }

  if ((this->getWidth() != B.getWidth()) || (this->getHeight() != B.getHeight())) {
    throw(vpException(vpException::memoryAllocationError, "vpImage mismatch in vpImage/vpImage substraction "));
  }

  for (unsigned int i = 0; i < this->getWidth() * this->getHeight(); i++) {
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
template <class Type> void vpImage<Type>::sub(const vpImage<Type> &A, const vpImage<Type> &B, vpImage<Type> &C)
{

  try {
    if ((A.getHeight() != C.getHeight()) || (A.getWidth() != C.getWidth()))
      C.resize(A.getHeight(), A.getWidth());
  } catch (const vpException &me) {
    std::cout << me << std::endl;
    throw;
  }

  if ((A.getWidth() != B.getWidth()) || (A.getHeight() != B.getHeight())) {
    throw(vpException(vpException::memoryAllocationError, "vpImage mismatch in vpImage/vpImage substraction "));
  }

  for (unsigned int i = 0; i < A.getWidth() * A.getHeight(); i++) {
    *(C.bitmap + i) = *(A.bitmap + i) - *(B.bitmap + i);
  }
}

/*!

  \warning This generic method is not implemented. You should rather use the
  instantiated methods for unsigned char and vpRGBa images.

  \sa vpImage<unsigned char>::performLut(const unsigned char (&)[256], const
  unsigned int) \sa vpImage<vpRGBa char>::performLut(const vpRGBa (&)[256],
  const unsigned int)

*/
template <class Type> void vpImage<Type>::performLut(const Type (&)[256], const unsigned int)
{
  std::cerr << "Not implemented !" << std::endl;
}

/*!
  Modify the intensities of a grayscale image using the look-up table passed
  in parameter.

  \param lut : Look-up table (unsigned char array of size=256) which maps each
  intensity to his new value. \param nbThreads : Number of threads to use for
  the computation.
*/
template <>
inline void vpImage<unsigned char>::performLut(const unsigned char (&lut)[256], const unsigned int nbThreads)
{
  unsigned int size = getWidth() * getHeight();
  unsigned char *ptrStart = (unsigned char *)bitmap;
  unsigned char *ptrEnd = ptrStart + size;
  unsigned char *ptrCurrent = ptrStart;

  bool use_single_thread = (nbThreads == 0 || nbThreads == 1);
#if !defined(VISP_HAVE_PTHREAD) && !defined(_WIN32)
  use_single_thread = true;
#endif

  if (!use_single_thread && getSize() <= nbThreads) {
    use_single_thread = true;
  }

  if (use_single_thread) {
    // Single thread

    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = lut[*ptrCurrent];
      ++ptrCurrent;
    }
  } else {
#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
    // Multi-threads

    std::vector<vpThread *> threadpool;
    std::vector<ImageLut_Param_t *> imageLutParams;

    unsigned int image_size = getSize();
    unsigned int step = image_size / nbThreads;
    unsigned int last_step = image_size - step * (nbThreads - 1);

    for (unsigned int index = 0; index < nbThreads; index++) {
      unsigned int start_index = index * step;
      unsigned int end_index = (index + 1) * step;

      if (index == nbThreads - 1) {
        end_index = start_index + last_step;
      }

      ImageLut_Param_t *imageLut_param = new ImageLut_Param_t(start_index, end_index, bitmap);
      memcpy(imageLut_param->m_lut, lut, 256 * sizeof(unsigned char));

      imageLutParams.push_back(imageLut_param);

      // Start the threads
      vpThread *imageLut_thread = new vpThread((vpThread::Fn)performLutThread, (vpThread::Args)imageLut_param);
      threadpool.push_back(imageLut_thread);
    }

    for (size_t cpt = 0; cpt < threadpool.size(); cpt++) {
      // Wait until thread ends up
      threadpool[cpt]->join();
    }

    // Delete
    for (size_t cpt = 0; cpt < threadpool.size(); cpt++) {
      delete threadpool[cpt];
    }

    for (size_t cpt = 0; cpt < imageLutParams.size(); cpt++) {
      delete imageLutParams[cpt];
    }
#endif
  }
}

/*!
  Modify the intensities of a color image using the look-up table passed in
  parameter.

  \param lut : Look-up table (vpRGBa array of size=256) which maps each
  intensity to his new value. \param nbThreads : Number of threads to use for
  the computation.
*/
template <> inline void vpImage<vpRGBa>::performLut(const vpRGBa (&lut)[256], const unsigned int nbThreads)
{
  unsigned int size = getWidth() * getHeight();
  unsigned char *ptrStart = (unsigned char *)bitmap;
  unsigned char *ptrEnd = ptrStart + size * 4;
  unsigned char *ptrCurrent = ptrStart;

  bool use_single_thread = (nbThreads == 0 || nbThreads == 1);
#if !defined(VISP_HAVE_PTHREAD) && !defined(_WIN32)
  use_single_thread = true;
#endif

  if (!use_single_thread && getSize() <= nbThreads) {
    use_single_thread = true;
  }

  if (use_single_thread) {
    // Single thread
    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = lut[*ptrCurrent].R;
      ++ptrCurrent;

      *ptrCurrent = lut[*ptrCurrent].G;
      ++ptrCurrent;

      *ptrCurrent = lut[*ptrCurrent].B;
      ++ptrCurrent;

      *ptrCurrent = lut[*ptrCurrent].A;
      ++ptrCurrent;
    }
  } else {
#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
    // Multi-threads
    std::vector<vpThread *> threadpool;
    std::vector<ImageLutRGBa_Param_t *> imageLutParams;

    unsigned int image_size = getSize();
    unsigned int step = image_size / nbThreads;
    unsigned int last_step = image_size - step * (nbThreads - 1);

    for (unsigned int index = 0; index < nbThreads; index++) {
      unsigned int start_index = index * step;
      unsigned int end_index = (index + 1) * step;

      if (index == nbThreads - 1) {
        end_index = start_index + last_step;
      }

      ImageLutRGBa_Param_t *imageLut_param = new ImageLutRGBa_Param_t(start_index, end_index, (unsigned char *)bitmap);
      memcpy(imageLut_param->m_lut, lut, 256 * sizeof(vpRGBa));

      imageLutParams.push_back(imageLut_param);

      // Start the threads
      vpThread *imageLut_thread = new vpThread((vpThread::Fn)performLutRGBaThread, (vpThread::Args)imageLut_param);
      threadpool.push_back(imageLut_thread);
    }

    for (size_t cpt = 0; cpt < threadpool.size(); cpt++) {
      // Wait until thread ends up
      threadpool[cpt]->join();
    }

    // Delete
    for (size_t cpt = 0; cpt < threadpool.size(); cpt++) {
      delete threadpool[cpt];
    }

    for (size_t cpt = 0; cpt < imageLutParams.size(); cpt++) {
      delete imageLutParams[cpt];
    }
#endif
  }
}

template <class Type> void swap(vpImage<Type> &first, vpImage<Type> &second)
{
  using std::swap;
  swap(first.bitmap, second.bitmap);
  swap(first.display, second.display);
  swap(first.npixels, second.npixels);
  swap(first.width, second.width);
  swap(first.height, second.height);
  swap(first.row, second.row);
}

#endif
