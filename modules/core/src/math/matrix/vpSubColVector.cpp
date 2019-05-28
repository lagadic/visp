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
 * Mask on a vpColVector.
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#include <stdlib.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpSubColVector.h>

//! Default constructor that creates an empty vector.
vpSubColVector::vpSubColVector() : vpColVector(), pRowNum(0), parent(NULL) {}

/*!
  Construct a sub-column vector from a parent column vector.
  \param v : parent column vector.
  \param offset : offset where the sub-column vector starts in the parent
  column vector. \param nrows : size of the sub-column vector.
*/
vpSubColVector::vpSubColVector(vpColVector &v, const unsigned int &offset, const unsigned int &nrows)
  : vpColVector(), pRowNum(0), parent(NULL)
{
  init(v, offset, nrows);
}

/*!
  Initialize a sub-column vector from a parent column vector.
  \param v : parent column vector.
  \param offset : offset where the sub-column vector starts in the parent
  column vector. \param nrows : size of the sub-column vector.
*/
void vpSubColVector::init(vpColVector &v, const unsigned int &offset, const unsigned int &nrows)
{
  if (!v.data) {
    throw(vpException(vpException::fatalError, "Cannot initialize a "
                                               "sub-column vector from an "
                                               "empty parent column vector"));
  }

  if (offset + nrows <= v.getRows()) {
    data = v.data + offset;

    rowNum = nrows;
    colNum = 1;

    pRowNum = v.getRows();
    parent = &v;

    if (rowPtrs) {
      free(rowPtrs);
    }

    rowPtrs = (double **)malloc(parent->getRows() * sizeof(double *));
    for (unsigned int i = 0; i < nrows; i++)
      rowPtrs[i] = v.data + i + offset;

    dsize = rowNum;
  } else {
    throw(vpException(vpException::dimensionError, "Cannot create a sub-column vector that is not "
                                                   "completely containt in the parrent column vector"));
  }
}

//! Destructor that set the pointer to the parrent column vector to NULL.
vpSubColVector::~vpSubColVector() { data = NULL; }

/*!
  This method can be used to detect if the parent column vector
  always exits or its size have not changed.
  If this not the case an exception is thrown.
*/
void vpSubColVector::checkParentStatus() const
{
  if (!data) {
    throw(vpException(vpException::fatalError, "The parent of the current sub-column vector has been destroyed"));
  }
  if (pRowNum != parent->getRows()) {
    throw(vpException(vpException::dimensionError, "The size of the parent sub-column vector has changed"));
  }
}

/*!
  Allow to initialize a sub-column vector from an other one using operation A
  = B. Notice that the sub-column vector is not resized to the dimension of \e
  B.

  \param B : a sub-column vector.
*/
vpSubColVector &vpSubColVector::operator=(const vpSubColVector &B)
{
  if (rowNum != B.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot initialize (%dx1) sub-column vector from "
                      "(%dx1) sub-column vector",
                      rowNum, B.getRows()));
  }
  pRowNum = B.pRowNum;
  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = B[i];
  return *this;
}

/*!
  Allow to initialize a sub-column vector from a column vector using operation
  A = B. Notice that the sub-column vector is not resized to the dimension of
  \e B. \param B : a column vector.
*/
vpSubColVector &vpSubColVector::operator=(const vpColVector &B)
{
  if (rowNum != B.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot initialize (%dx1) sub-column vector from "
                      "(%dx1) column vector",
                      rowNum, B.getRows()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = B[i];

  return *this;
}

/*!
  Allow to initialize a sub-column vector from a m-by-1 matrix using operation
  A = B. Notice that the sub-column vector is not resized to the dimension of
  \e B. \param B : a matrix of size m-by-1.
*/
vpSubColVector &vpSubColVector::operator=(const vpMatrix &B)
{
  if ((B.getCols() != 1) || (rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot initialize (%dx1) sub-column vector from (%dx%d) matrix",
                      rowNum, B.getRows(), B.getCols()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = B[i][1];
  return *this;
}

/*!
  Set all the elements of the sub-column vector to \e x.
  \param x : a scalar value.
*/
vpSubColVector &vpSubColVector::operator=(const double &x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = x;
  return *this;
}

/*!
   Operator that allows to convert a translation vector into a column vector.
 */
vpSubColVector &vpSubColVector::operator=(const vpTranslationVector &tv)
{
  unsigned int k = tv.getRows();
  if (rowNum != k) {
    try {
      resize(k);
    } catch (...) {
      throw;
    }
  }

  memcpy(data, tv.data, rowNum * sizeof(double));
  return *this;
}
/*!
   Operator that allows to convert a rotation vector into a column vector.
 */
vpSubColVector &vpSubColVector::operator=(const vpRotationVector &rv)
{
  unsigned int k = rv.getRows();
  if (rowNum != k) {
    try {
      resize(k);
    } catch (...) {
      throw;
    }
  }

  memcpy(data, rv.data, rowNum * sizeof(double));
  return *this;
}
/*!
   Operator that allows to convert a pose vector into a column vector.
 */
vpSubColVector &vpSubColVector::operator=(const vpPoseVector &p)
{
  unsigned int k = p.getRows();
  if (rowNum != k) {
    try {
      resize(k);
    } catch (...) {
      throw;
    }
  }

  memcpy(data, p.data, rowNum * sizeof(double));
  return *this;
}
