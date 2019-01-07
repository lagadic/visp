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
 * Mask on a vpRowVector .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#include <stdlib.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpSubRowVector.h>

//! Default constructor that creates an empty vector.
vpSubRowVector::vpSubRowVector() : vpRowVector(), pColNum(0), parent(NULL) {}

/*!
  Construct a sub-row vector from a parent row vector.
  \param v : parent row vector.
  \param offset : offset where the sub-row vector starts in the parent row
  vector. \param ncols : size of the sub-row vector.
*/
vpSubRowVector::vpSubRowVector(vpRowVector &v, const unsigned int &offset, const unsigned int &ncols)
  : vpRowVector(), pColNum(0), parent(NULL)
{
  init(v, offset, ncols);
}

/*!
  Initialize a sub-row vector from a parent row vector.
  \param v : parent row vector.
  \param offset : offset where the sub-row vector starts in the parent row
  vector. \param ncols : size of the sub-row vector.
*/
void vpSubRowVector::init(vpRowVector &v, const unsigned int &offset, const unsigned int &ncols)
{
  if (!v.data) {
    throw(vpException(vpException::fatalError, "Cannot initialize a sub-row "
                                               "vector from an empty parent "
                                               "row vector"));
  }

  if (offset + ncols <= v.getCols()) {
    data = v.data + offset;

    rowNum = 1;
    colNum = ncols;

    pColNum = v.getCols();
    parent = &v;

    if (rowPtrs)
      free(rowPtrs);

    rowPtrs = (double **)malloc(1 * sizeof(double *));
    for (unsigned int i = 0; i < 1; i++)
      rowPtrs[i] = v.data + i + offset;

    dsize = colNum;
  } else {
    throw(vpException(vpException::dimensionError, "Cannot create a sub-row vector that is not completely "
                                                   "containt in the parrent row vector"));
  }
}

//! Destructor that set the pointer to the parrent row vector to NULL.
vpSubRowVector::~vpSubRowVector() { data = NULL; }

/*!
  This method can be used to detect if the parent row vector
  always exits or its size have not changed.
  If this not the case an exception is thrown.
*/
void vpSubRowVector::checkParentStatus() const
{
  if (!data) {
    throw(vpException(vpException::fatalError, "The parent of the current sub-row vector has been destroyed"));
  }
  if (pColNum != parent->getCols()) {
    throw(vpException(vpException::dimensionError, "The size of the parent sub-row vector has changed"));
  }
}

/*!
  Allow to initialize a sub-row vector from an other one using operation A =
  B. Notice that the sub-row vector is not resized to the dimension of \e B.

  \param B : a sub-row vector.
*/
vpSubRowVector &vpSubRowVector::operator=(const vpSubRowVector &B)
{
  if (colNum != B.getCols()) {
    throw(vpException(vpException::dimensionError, "Cannot initialize (1x%d) sub-row vector from (1x%d) sub-row vector",
                      colNum, B.getCols()));
  }
  pColNum = B.pColNum;
  parent = B.parent;
  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = B[i];

  return *this;
}

/*!
  Allow to initialize a sub-row vector from a row vector using operation A =
  B. Notice that the sub-row vector is not resized to the dimension of \e B.

  \param B : a row vector.
*/
vpSubRowVector &vpSubRowVector::operator=(const vpRowVector &B)
{
  if (colNum != B.getCols()) {
    throw(vpException(vpException::dimensionError, "Cannot initialize (1x%d) sub-row vector from (1x%d) row vector",
                      colNum, B.getCols()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = B[i];

  return *this;
}

/*!
  Allow to initialize a sub-row vector from a matrix using operation A = B.
  Notice that the sub-row vector is not resized to the dimension of \e B.

  \param B : a matrix of size 1-by-n.
*/
vpSubRowVector &vpSubRowVector::operator=(const vpMatrix &B)
{
  if ((B.getRows() != 1) || (colNum != B.getCols())) {
    throw(vpException(vpException::dimensionError, "Cannot initialize (1x%d) sub-column vector from (%dx%d) matrix",
                      colNum, B.getRows(), B.getCols()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = B[i][1];
  return *this;
}
/*!
  Set all the elements of the sub-row vector to \e x.
  \param x : a scalar value.
*/
vpSubRowVector &vpSubRowVector::operator=(const double &x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    data[i] = x;
  return *this;
}
