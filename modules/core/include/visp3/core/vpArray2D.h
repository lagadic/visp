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
 * This class implements an 2D array as a template class.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef __vpArray2D_h_
#define __vpArray2D_h_

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

/*!
  \class vpArray2D
  \ingroup group_core_matrices

  This class implements a 2D array as a template class and all the basic functionalities common to matrices and vectors,
  but also specific containers such as twist, homogeneous or rotation matrices.
*/
template<class Type>
class vpArray2D
{
protected:
  //! Number of rows in the array
  unsigned int rowNum;
  //! Number of columns in the array
  unsigned int colNum;
  //! Address of the first element of each rows
  Type **rowPtrs;
  //! Current array size (rowNum * colNum)
  unsigned int dsize;

public:
  //! Address of the first element of the data array
  Type *data;

public:
  /*!
  Basic constructor of a 2D array.
  Number of columns and rows are set to zero.
  */
  vpArray2D<Type>()
    : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {}
  /*!
  Copy constructor of a 2D array.
  */
  vpArray2D<Type>(const vpArray2D<Type> & A)
    : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {
    resize(A.rowNum, A.colNum);
    memcpy(data, A.data, rowNum*colNum*sizeof(Type));
  }
  /*!
  Constructor that initializes a 2D array with 0.

  \param r : Array number of rows.
  \param c : Array number of columns.
  */
  vpArray2D<Type>(unsigned int r, unsigned int c)
    : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {
    resize(r, c);
  }
  /*!
  Constructor that initialize a 2D array with \e val.

  \param r : Array number of rows.
  \param c : Array number of columns.
  \param val : Each element of the array is set to \e val.
  */
  vpArray2D<Type>(unsigned int r, unsigned int c, Type val)
    : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {
    resize(r, c);
    *this = val;
  }
  /*!
  Destructor that desallocate memory.
  */
  virtual ~vpArray2D<Type>()
  {
    if (data != NULL ) {
      free(data);
      data=NULL;
    }

    if (rowPtrs!=NULL) {
      free(rowPtrs);
      rowPtrs=NULL ;
    }
    rowNum = colNum = dsize = 0;
  }

  /** @name Common inherited functionalities  */
  //@{

  Type getMinValue() const;

  Type getMaxValue() const;

  //! Return the number of rows of the 2D array
  inline unsigned int getRows() const { return rowNum ;}
  //! Return the number of columns of the 2D array
  inline unsigned int getCols() const { return colNum; }
  //! Return the number of elements of the 2D array.
  inline unsigned int size() const { return colNum*rowNum; }
  /*!
  Set the size of the array and initialize all the values to zero.

  \param nrows : number of rows.
  \param ncols : number of column.
  \param flagNullify : if true, then the array is re-initialized to 0
  after resize. If false, the initial values from the common part of the
  array (common part between old and new version of the array) are kept.
  Default value is true.
  */
  void resize(const unsigned int nrows, const unsigned int ncols,
              const bool flagNullify = true)
  {
    if ((nrows == rowNum) && (ncols == colNum)) {
      if (flagNullify && this->data != NULL) {
        memset(this->data, 0, this->dsize*sizeof(Type));
      }
    }
    else {
      const bool recopyNeeded = (ncols != this ->colNum);
      Type * copyTmp = NULL;
      unsigned int rowTmp = 0, colTmp=0;

      // Recopy case per case is required if number of cols has changed;
      // structure of Type array is not the same in this case.
      if (recopyNeeded && this->data != NULL) {
        copyTmp = new Type[this->dsize];
        memcpy (copyTmp, this ->data, sizeof(Type)*this->dsize);
        rowTmp=this->rowNum; colTmp=this->colNum;
      }

      // Reallocation of this->data array
      this->dsize = nrows*ncols;
      this->data = (Type*)realloc(this->data, this->dsize*sizeof(Type));
      if ((NULL == this->data) && (0 != this->dsize)) {
        if (copyTmp != NULL) delete [] copyTmp;
        throw(vpException(vpException::memoryAllocationError,
          "Memory allocation error when allocating 2D array data")) ;
      }

      this->rowPtrs = (Type**)realloc (this->rowPtrs, nrows*sizeof(Type*));
      if ((NULL == this->rowPtrs) && (0 != this->dsize)) {
        if (copyTmp != NULL) delete [] copyTmp;
        throw(vpException(vpException::memoryAllocationError,
          "Memory allocation error when allocating 2D array rowPtrs")) ;
      }

      // Update rowPtrs
      {
        Type **t_= rowPtrs;
        for (unsigned int i=0; i<dsize; i+=ncols)  {
          *t_++ = this->data + i;
        }
      }

      this->rowNum = nrows; this->colNum = ncols;

      // Recopy of this->data array values or nullify
      if (flagNullify) {
        memset(this->data,0,this->dsize*sizeof(Type));
      }
      else if (recopyNeeded && this->rowPtrs != NULL) {
        // Recopy...
        const unsigned int minRow = (this->rowNum<rowTmp)?this->rowNum:rowTmp;
        const unsigned int minCol = (this->colNum<colTmp)?this->colNum:colTmp;
        for (unsigned int i=0; i<this->rowNum; ++i) {
          for (unsigned int j=0; j<this->colNum; ++j) {
            if ((minRow > i) && (minCol > j)) {
              (*this)[i][j] = copyTmp [i*colTmp+j];
            }
            else {
              (*this)[i][j] = 0;
            }
          }
        }
      }

      if (copyTmp != NULL)
        delete [] copyTmp;
    }
  }
  //! Set all the elements of the array to \e x.
  vpArray2D<Type> & operator=(Type x)
  {
    for (unsigned int i=0;i<rowNum;i++)
      for(unsigned int j=0;j<colNum;j++)
        rowPtrs[i][j] = x;

    return *this;
  }

  //! Set element \f$A_{ij} = x\f$ using A[i][j] = x
  inline Type *operator[](unsigned int i) { return rowPtrs[i]; }
  //! Get element \f$x = A_{ij}\f$ using x = A[i][j]
  inline Type *operator[](unsigned int i) const {return rowPtrs[i];}

  /*!
    \relates vpArray2D
    Writes the given array to the output stream and returns a reference to the output stream.
    */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
  {
    if (A.data == NULL)
      return s;
    std::ios_base::fmtflags original_flags = s.flags();

    s.precision(10) ;
    for (unsigned int i=0;i<A.getRows();i++) {
      for (unsigned int j=0;j<A.getCols();j++){
        s <<  A[i][j] << "  ";
      }
      // We don't add a \n char on the end of the last array line
      if (i < A.getRows()-1)
        s << std::endl;
    }

    s.flags(original_flags); // restore s to standard state

    return s;
  }
  //@}
};

/*!
 Return the array min value.
 */
template<class Type>
Type
vpArray2D<Type>::getMinValue() const
{
  Type *dataptr = data;
  Type min = *dataptr;
  dataptr++;
  for (unsigned int i = 0; i < dsize-1; i++)
  {
    if (*dataptr < min) min = *dataptr;
    dataptr++;
  }
  return min;
}

/*!
 Return the array max value.
 */
template<class Type>
Type
vpArray2D<Type>::getMaxValue() const
{
  Type *dataptr = data;
  Type max = *dataptr;
  dataptr++;
  for (unsigned int i = 0; i < dsize-1; i++)
  {
    if (*dataptr > max) max = *dataptr;
    dataptr++;
  }
  return max;
}

#endif
