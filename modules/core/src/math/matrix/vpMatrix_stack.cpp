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
 * Stack matrix.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>

BEGIN_VISP_NAMESPACE

/*!
  Stacks columns of a matrix in a vector.
  \param out : a vpColVector.
*/
void vpMatrix::stackColumns(vpColVector &out)
{
  if ((out.rowNum != (colNum * rowNum)) || (out.colNum != 1)) {
    out.resize(colNum * rowNum, false, false);
  }

  double *optr = out.data;
  for (unsigned int j = 0; j < colNum; ++j) {
    for (unsigned int i = 0; i < rowNum; ++i) {
      *(optr++) = rowPtrs[i][j];
    }
  }
}

/*!
  Stacks columns of a matrix in a vector.
  \return a vpColVector.
*/
vpColVector vpMatrix::stackColumns()
{
  vpColVector out(colNum * rowNum);
  stackColumns(out);
  return out;
}

/*!
  Stacks rows of a matrix in a vector
  \param out : a vpRowVector.
*/
void vpMatrix::stackRows(vpRowVector &out)
{
  if ((out.getRows() != 1) || (out.getCols() != (colNum * rowNum))) {
    out.resize(colNum * rowNum, false, false);
  }

  memcpy(out.data, data, sizeof(double) * out.getCols());
}

/*!
  Stacks rows of a matrix in a vector.
  \return a vpRowVector.
*/
vpRowVector vpMatrix::stackRows()
{
  vpRowVector out(colNum * rowNum);
  stackRows(out);
  return out;
}

/*!
  Stack matrix \e B to the end of matrix \e A and return the resulting matrix
  [ A B ]^T

  \param A : Upper matrix.
  \param B : Lower matrix.
  \return Stacked matrix [ A B ]^T

  \warning A and B must have the same number of columns.
*/
vpMatrix vpMatrix::stack(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C;

  vpMatrix::stack(A, B, C);

  return C;
}

/*!
  Stack matrix \e B to the end of matrix \e A and return the resulting matrix
  in \e C.

  \param  A : Upper matrix.
  \param  B : Lower matrix.
  \param  C : Stacked matrix C = [ A B ]^T

  \warning A and B must have the same number of columns. A and C, B and C must
  be two different objects.
*/
void vpMatrix::stack(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  unsigned int nra = A.getRows();
  unsigned int nrb = B.getRows();

  if (nra != 0) {
    if (A.getCols() != B.getCols()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                        A.getCols(), B.getRows(), B.getCols()));
    }
  }

  if ((A.data != nullptr) && (A.data == C.data)) {
    std::cerr << "A and C must be two different objects!" << std::endl;
    return;
  }

  if ((B.data != nullptr) && (B.data == C.data)) {
    std::cerr << "B and C must be two different objects!" << std::endl;
    return;
  }

  C.resize(nra + nrb, B.getCols(), false, false);

  if ((C.data != nullptr) && (A.data != nullptr) && (A.size() > 0)) {
    // Copy A in C
    memcpy(C.data, A.data, sizeof(double) * A.size());
  }

  if ((C.data != nullptr) && (B.data != nullptr) && (B.size() > 0)) {
    // Copy B in C
    memcpy(C.data + A.size(), B.data, sizeof(double) * B.size());
  }
}

/*!
  Stack row vector \e r to matrix \e A and return the resulting matrix [ A r ]^T

  \param A : Upper matrix.
  \param r : Lower row vector.
  \return Stacked matrix [ A r ]^T

  \warning \e A and \e r must have the same number of columns.
*/
vpMatrix vpMatrix::stack(const vpMatrix &A, const vpRowVector &r)
{
  vpMatrix C;
  vpMatrix::stack(A, r, C);

  return C;
}

/*!
  Stack row vector \e r to the end of matrix \e A and return the resulting
  matrix in \e C.

  \param  A : Upper matrix.
  \param  r : Lower row vector.
  \param  C : Stacked matrix C = [ A r ]^T

  \warning A and r must have the same number of columns. A and C must be two
  different objects.
*/
void vpMatrix::stack(const vpMatrix &A, const vpRowVector &r, vpMatrix &C)
{
  if ((A.data != nullptr) && (A.data == C.data)) {
    std::cerr << "A and C must be two different objects!" << std::endl;
    return;
  }

  C = A;
  C.stack(r);
}

/*!
  Stack column vector \e c to matrix \e A and return the resulting matrix [ A c ]

  \param A : Left matrix.
  \param c : Right column vector.
  \return Stacked matrix [ A c ]

  \warning \e A and \e c must have the same number of rows.
*/
vpMatrix vpMatrix::stack(const vpMatrix &A, const vpColVector &c)
{
  vpMatrix C;
  vpMatrix::stack(A, c, C);

  return C;
}

/*!
  Stack column vector \e c to the end of matrix \e A and return the resulting
  matrix in \e C.

  \param  A : Left matrix.
  \param  c : Right column vector.
  \param  C : Stacked matrix C = [ A c ]

  \warning A and c must have the same number of rows. A and C must be two
  different objects.
*/
void vpMatrix::stack(const vpMatrix &A, const vpColVector &c, vpMatrix &C)
{
  if ((A.data != nullptr) && (A.data == C.data)) {
    std::cerr << "A and C must be two different objects!" << std::endl;
    return;
  }

  C = A;
  C.stack(c);
}

/*!
  Stack A at the end of the current matrix, or copy if the matrix has no
  dimensions : this = [ this A ]^T.
*/
void vpMatrix::stack(const vpMatrix &A)
{
  if (rowNum == 0) {
    *this = A;
  }
  else if (A.getRows() > 0) {
    if (colNum != A.getCols()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (%dx%d) matrix", rowNum, colNum,
                        A.getRows(), A.getCols()));
    }

    unsigned int rowNumOld = rowNum;
    resize(rowNum + A.getRows(), colNum, false, false);
    insert(A, rowNumOld, 0);
  }
}

/*!
  Stack row vector \e r at the end of the current matrix, or copy if the
  matrix has no dimensions: this = [ this r ]^T.

  Here an example for a robot velocity log :
  \code
  vpMatrix A;
  vpColVector v(6);
  for(unsigned int i = 0;i<100;i++)
  {
    robot.getVelocity(vpRobot::ARTICULAR_FRAME, v);
    Velocities.stack(v.t());
  }
  \endcode
*/
void vpMatrix::stack(const vpRowVector &r)
{
  if (rowNum == 0) {
    *this = r;
  }
  else {
    if (colNum != r.getCols()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (1x%d) row vector", rowNum,
                        colNum, r.getCols()));
    }

    if (r.size() == 0) {
      return;
    }

    unsigned int oldSize = size();
    resize(rowNum + 1, colNum, false, false);

    if ((data != nullptr) && (r.data != nullptr) && (data != r.data)) {
      // Copy r in data
      memcpy(data + oldSize, r.data, sizeof(double) * r.size());
    }
  }
}

/*!
  Stack column vector \e c at the right of the current matrix, or copy if the
  matrix has no dimensions: this = [ this c ].

  Here an example for a robot velocity log matrix:
  \code
  vpMatrix log;
  vpColVector v(6);
  for(unsigned int i = 0; i<100;i++)
  {
    robot.getVelocity(vpRobot::ARTICULAR_FRAME, v);
    log.stack(v);
  }
  \endcode
  Here the log matrix has size 6 rows by 100 columns.
*/
void vpMatrix::stack(const vpColVector &c)
{
  if (colNum == 0) {
    *this = c;
  }
  else {
    if (rowNum != c.getRows()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (%dx1) column vector", rowNum,
                        colNum, c.getRows()));
    }

    if (c.size() == 0) {
      return;
    }

    vpMatrix tmp = *this;
    unsigned int oldColNum = colNum;
    resize(rowNum, colNum + 1, false, false);

    if ((data != nullptr) && (tmp.data != nullptr) && (data != tmp.data)) {
      // Copy c in data
      for (unsigned int i = 0; i < rowNum; ++i) {
        memcpy(data + (i * colNum), tmp.data + (i * oldColNum), sizeof(double) * oldColNum);
        rowPtrs[i][oldColNum] = c[i];
      }
    }
  }
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
vpMatrix vpMatrix::stackMatrices(const vpColVector &A, const vpColVector &B)
{
  return (vpMatrix)(vpColVector::stack(A, B));
}

void vpMatrix::stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  vpColVector::stack(A, B, C);
}

vpMatrix vpMatrix::stackMatrices(const vpMatrix &A, const vpRowVector &B) { return vpMatrix::stack(A, B); }

void vpMatrix::stackMatrices(const vpMatrix &A, const vpRowVector &B, vpMatrix &C) { vpMatrix::stack(A, B, C); }
#endif

END_VISP_NAMESPACE
