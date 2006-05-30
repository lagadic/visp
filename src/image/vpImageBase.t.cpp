/****************************************************************************
 *
 * $Id: vpImageBase.t.cpp,v 1.4 2006-05-30 08:40:43 fspindle Exp $
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

  Allocate memory for an [_nrows x _ncols] image

  Set all the element of the bitmap to value

  \exception vpException::memoryAllocationError

  \sa vpImage::init(_nrows, _ncols)
*/
template<class Type>
void
vpImage<Type>::init(int _nrows, int _ncols, Type value)
{
  try
  {
    init(_nrows,_ncols) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  for (int i=0  ; i < npixels ;  i++)
    bitmap[i] = value ;
}


/*!
  \brief Image initialization

  Allocate memory for an [_nrows x _ncols] image

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template<class Type>
void
vpImage<Type>::init(int _nrows, int _ncols)
{

  if (_nrows != nrows) {
    if (row != NULL)  {
      DEBUG_TRACE(10,"Destruction row[]");
      delete [] row;
      row = NULL;
    }
  }

  if ((_nrows != nrows) || (_ncols != ncols))
  {
    if (bitmap != NULL) {
      DEBUG_TRACE(10,"Destruction bitmap[]") ;
      delete [] bitmap;
      bitmap = NULL;
    }
  }



  ncols = _ncols ;
  nrows = _nrows ;

  npixels=ncols*nrows;


  if (bitmap == NULL)  bitmap = new  Type[npixels] ;

  //  ERROR_TRACE("Allocate bitmap %p",bitmap) ;
  if (bitmap == NULL)
  {
        ERROR_TRACE("cannot allocate bitmap ") ;
    throw(vpException(vpException::memoryAllocationError,
		      "cannot allocate bitmap ")) ;
  }

  if (row == NULL)  row = new  Type*[nrows] ;
//  ERROR_TRACE("Allocate row %p",row) ;
  if (row == NULL)
  {
    ERROR_TRACE("cannot allocate row ") ;
    throw(vpException(vpException::memoryAllocationError,
		      "cannot allocate row ")) ;
  }

  int i ;
  for ( i =0  ; i < nrows ; i++)
    row[i] = bitmap + i*ncols ;
}

/*!
  \brief Constructor

  Allocate memory for an [_nrows x _ncols] image

  Element of the bitmap are set to zero

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa vpImage::init(_nrows, _ncols)
*/
template<class Type>
vpImage<Type>::vpImage(int _nrows, int _ncols)
{
  bitmap = NULL ;
  row = NULL ;

  display =  NULL ;
  nrows = ncols = 0 ;
  initDisplay = false;

  try
  {
    init(_nrows,_ncols,0) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Constructor

  Allocate memory for an [_nrows x _ncols] image

  set all the element of the bitmap to value

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \return MEMORY_FAULT if memory allocation is impossible, else OK

  \sa vpImage::init(_nrows, _ncols, value)
*/
template<class Type>
vpImage<Type>::vpImage (int _nrows, int _ncols, Type value)
{
  bitmap = NULL ;
  row = NULL ;

  display =  NULL ;
  nrows = ncols = 0 ;
  initDisplay = false;
  try
  {
    init(_nrows,_ncols,value) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief Constructor

  No memory allocation is done

  set all the element of the bitmap to value

  \sa vpImage::resize(_nrows, _ncols) for memory allocation
*/
template<class Type>
vpImage<Type>::vpImage()
{
  bitmap = NULL ;
  row = NULL ;

  display =  NULL ;

  nrows = ncols = 0 ;

  initDisplay = false;

}

/*!
  \brief resize the image : Image initialization

  Allocate memory for an [_nrows x _ncols] image

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

  \sa vpImage::init(_nrows, _ncols)
*/
template<class Type>
void
vpImage<Type>::resize(int n_rows, int n_cols)
{
  try
  {
    init(n_rows,n_cols) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE(" ") ;
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
 //   ERROR_TRACE("Deallocate ") ;


  if (bitmap!=NULL)
  {
  //  ERROR_TRACE("Deallocate bitmap memory %p",bitmap) ;
//    DEBUG_TRACE(20,"Deallocate bitmap memory %p",bitmap) ;
    delete [] bitmap ;
    bitmap = NULL;
  }


  if (row!=NULL)
  {
 //   ERROR_TRACE("Deallocate row memory %p",row) ;
//    DEBUG_TRACE(20,"Deallocate row memory %p",row) ;
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
      resize(I.getRows(),I.getCols());
      int i;
      memcpy(bitmap, I.bitmap, I.npixels*sizeof(Type)) ;
      for (i =0  ; i < nrows ; i++) row[i] = bitmap + i*ncols ;
    }
  }
  catch(vpException me)
  {
    ERROR_TRACE(" ") ;
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
  for (int i=0 ; i < npixels ; i++)
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
  for (int i=0 ; i < npixels ; i++)
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
    resize(m.getRows(),m.getCols()) ;

    memcpy(bitmap, m.bitmap, m.npixels*sizeof(Type)) ;

    //  for (int i=0; i<nrows; i++) row[i]   = bitmap + i*ncols ;
  }
  catch(vpException me)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  \brief = operator : set   all the element of the bitmap to a given  value x
   A = x <=> A[i][j] = x

   \warning = must be defined for <Type>
*/
template<class Type>
void vpImage<Type>::operator=(const Type &x)
{
  for (int i=0 ; i < npixels ; i++)
    bitmap[i] = x ;
}


/*!
  \brief get the value of pixel at coordinates [i][j] using a biliner interpolation

  \param i,j : There are float value

  \return value of the pixel obtained by bilinear interpolation

  Bilinear interpolation explores four points neighboring the point (i, j), and
  assumes that the brightness function is bilinear in this neighborhood

   bilinear interpolation is given by

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
  int l = (int) i ;
  int k = (int) j ;

  double t =  i - l ;
  double u =  j - k ;
  return (1-t)*(1-u)*(*this)[l][k] + t*(1-u)*(*this)[l+1][k] +
    (1-t)*u*(*this)[l][k+1] + t*u*(*this)[l+1][k+1] ;

}





/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
