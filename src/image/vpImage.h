


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImage.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpImage.h,v 1.1.1.1 2005-06-08 07:08:08 fspindle Exp $
 *
 * Description
 * ============
 *   image handling
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpImage_H
#define vpImage_H

/*!
  \file vpImage.h
  \brief  image handling
  \ingroup
*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpImageException.h>

/*!
  \class vpImage

  \brief  definition of the vpImage class member functions

  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  <h3> Data structure </h3>

  Each image is build using two structure (an array bitmap which size
  is [ncol*nrow]) and an array of pointer row (which size is [nrow])
  the ith element in the row array row[i] is pointer toward the ith
  "line" of the image (ie, bitmap +i*bcol )

  \image html image-data-structure.gif
  \image latex image-data-structure.ps  width=10cm

  \if 0
  bitmap --------------------|
                             v
               -----         --------------------------------------------
  row -------> | x-|-------->| | | | | | | | |  ...     | | | | | | | | |
               -----         --------------------------------------------
	       | x-|-------->| | | | | | | | |  ...     | | | | | | | | |
               -----         --------------------------------------------
	       |   |
               -----
	       |   |
               -----
	       |   |
               -----
                 :
  \endif

  Such a structure allows a fast acces to each element of the image.
  if i is the ith rows and j the jth columns the value of this pixel
  is given by I[i][j] (that is equivalent to row[i][j]).

  This is a template class, therefore the type of each  element of the
  array is not a priori defined.


*/
template<class Type>
class vpImage
{

  friend class vpDisplay ;

private:
  int npixels ; //<! number of pixel in the image
  int ncols ;   //<! number of columns
  int nrows ;   //<! number of rows

public:
  Type *bitmap ;  //!< points toward the bitmap
private:
  Type **row ;    //!< points the row pointer array
public:
  //! get the number of rows in the image
  inline  int getRows() const { return nrows ; }
  //! get the number of columns in the image
  inline  int getCols() const { return ncols ; }

  inline int getNumberOfPixel() const{ return npixels; }

  //------------------------------------------------------------------
  //         Acces to the image


  //! operator[] allows operation like I[i] = x
  inline Type *operator[]( const int n)   { return row[n];}

  //! operator[] allows operation likex = I[i]
  inline const  Type *operator[](int n) const { return row[n];}

  inline  Type operator()(const int i, const  int j)
  {
    return bitmap[i*ncols+j] ;
  }
  inline  void  operator()(const int i, const  int j, const Type &a)
  {
    bitmap[i*ncols+j] = a ;
  }

  //! bilinear interpolation acces
  double get(double i, double j) ;

  //------------------------------------------------------------------
  //         build the image

  //! set the size of the image
  void  resize(const int nbl, const int nbc) ;

  //! set the size of the image
  void init(int nbl, int nbc) ;
  //! set the size of the image
  void init(int nbl, int nbc, Type value) ;

  //! constructor  set the size of the image
  vpImage(int nbl, int nbc) ;
  //! constructor  set the size of the image and init all the pixel
  vpImage(int nbl, int nbc, Type value) ;
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
  Type MaxValue() const ;
  //! Return the minumum value within the bitmap
  Type MinValue() const ;


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
