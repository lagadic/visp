/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * This class implements an 2D array as a template class.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef __vpArray2D_h_
#define __vpArray2D_h_

#include <fstream>
#include <iostream>
#include <limits>
#include <math.h>
#include <ostream>
#include <sstream>
#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

/*!
  \class vpArray2D
  \ingroup group_core_matrices

  \brief Implementation of a generic 2D array used as vase class of matrices
  and vectors.

  This class implements a 2D array as a template class and all the basic
  functionalities common to matrices and vectors. More precisely:
  - concerning matrices, vpMatrix but also specific containers such as twist
    (vpVelocityTwistMatrix and vpForceTwistMatrix), homogeneous
  (vpHomogeneousMatrix), rotation (vpRotationMatrix) and homography
  (vpHomography) matrices inherit from vpArray2D<double>.
  - concerning vectors, vpColVector, vpRowVector but also specific containers
  describing the pose (vpPoseVector) and the rotation (vpRotationVector)
  inherit also from vpArray2D<double>.
*/
template <class Type> class vpArray2D
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
  vpArray2D<Type>() : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL) {}
  /*!
  Copy constructor of a 2D array.
  */
  vpArray2D<Type>(const vpArray2D<Type> &A) : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {
    resize(A.rowNum, A.colNum, false, false);
    memcpy(data, A.data, rowNum * colNum * sizeof(Type));
  }
  /*!
  Constructor that initializes a 2D array with 0.

  \param r : Array number of rows.
  \param c : Array number of columns.
  */
  vpArray2D<Type>(unsigned int r, unsigned int c) : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {
    resize(r, c);
  }
  /*!
  Constructor that initialize a 2D array with \e val.

  \param r : Array number of rows.
  \param c : Array number of columns.
  \param val : Each element of the array is set to \e val.
  */
  vpArray2D<Type>(unsigned int r, unsigned int c, Type val) : rowNum(0), colNum(0), rowPtrs(NULL), dsize(0), data(NULL)
  {
    resize(r, c, false, false);
    *this = val;
  }
  /*!
  Destructor that desallocate memory.
  */
  virtual ~vpArray2D<Type>()
  {
    if (data != NULL) {
      free(data);
      data = NULL;
    }

    if (rowPtrs != NULL) {
      free(rowPtrs);
      rowPtrs = NULL;
    }
    rowNum = colNum = dsize = 0;
  }

  /** @name Inherited functionalities from vpArray2D */
  //@{

  /*!
   * Return the number of columns of the 2D array.
   * \sa getRows(), size()
   */
  inline unsigned int getCols() const { return colNum; }

  Type getMaxValue() const;

  Type getMinValue() const;

  /*!
   * Return the number of rows of the 2D array.
   * \sa getCols(), size()
   */
  inline unsigned int getRows() const { return rowNum; }
  //! Return the number of elements of the 2D array.
  inline unsigned int size() const { return colNum * rowNum; }
  /*!
  Set the size of the array and initialize all the values to zero.

  \param nrows : number of rows.
  \param ncols : number of column.
  \param flagNullify : if true, then the array is re-initialized to 0
  after resize. If false, the initial values from the common part of the
  array (common part between old and new version of the array) are kept.
  Default value is true.
  \param recopy_ : if true, will perform an explicit recopy of the old data
  if needed and if flagNullify is set to false.
  */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify = true,
              const bool recopy_ = true)
  {
    if ((nrows == rowNum) && (ncols == colNum)) {
      if (flagNullify && this->data != NULL) {
        memset(this->data, 0, this->dsize * sizeof(Type));
      }
    } else {
      bool recopy = !flagNullify && recopy_; // priority to flagNullify
      const bool recopyNeeded = (ncols != this->colNum && this->colNum > 0 && ncols > 0 && (!flagNullify || recopy));
      Type *copyTmp = NULL;
      unsigned int rowTmp = 0, colTmp = 0;

      // Recopy case per case is required if number of cols has changed;
      // structure of Type array is not the same in this case.
      if (recopyNeeded && this->data != NULL) {
        copyTmp = new Type[this->dsize];
        memcpy(copyTmp, this->data, sizeof(Type) * this->dsize);
        rowTmp = this->rowNum;
        colTmp = this->colNum;
      }

      // Reallocation of this->data array
      this->dsize = nrows * ncols;
      this->data = (Type *)realloc(this->data, this->dsize * sizeof(Type));
      if ((NULL == this->data) && (0 != this->dsize)) {
        if (copyTmp != NULL)
          delete[] copyTmp;
        throw(vpException(vpException::memoryAllocationError, "Memory allocation error when allocating 2D array data"));
      }

      this->rowPtrs = (Type **)realloc(this->rowPtrs, nrows * sizeof(Type *));
      if ((NULL == this->rowPtrs) && (0 != this->dsize)) {
        if (copyTmp != NULL)
          delete[] copyTmp;
        throw(vpException(vpException::memoryAllocationError,
                          "Memory allocation error when allocating 2D array rowPtrs"));
      }

      // Update rowPtrs
      {
        Type **t_ = rowPtrs;
        for (unsigned int i = 0; i < dsize; i += ncols) {
          *t_++ = this->data + i;
        }
      }

      this->rowNum = nrows;
      this->colNum = ncols;

      // Recopy of this->data array values or nullify
      if (flagNullify) {
        memset(this->data, 0, this->dsize * sizeof(Type));
      } else if (recopyNeeded && this->rowPtrs != NULL) {
        // Recopy...
        const unsigned int minRow = (this->rowNum < rowTmp) ? this->rowNum : rowTmp;
        const unsigned int minCol = (this->colNum < colTmp) ? this->colNum : colTmp;
        for (unsigned int i = 0; i < this->rowNum; ++i) {
          for (unsigned int j = 0; j < this->colNum; ++j) {
            if ((minRow > i) && (minCol > j)) {
              (*this)[i][j] = copyTmp[i * colTmp + j];
            } else {
              (*this)[i][j] = 0;
            }
          }
        }
      }

      if (copyTmp != NULL)
        delete[] copyTmp;
    }
  }
  //! Set all the elements of the array to \e x.
  vpArray2D<Type> &operator=(Type x)
  {
    for (unsigned int i = 0; i < rowNum; i++)
      for (unsigned int j = 0; j < colNum; j++)
        rowPtrs[i][j] = x;

    return *this;
  }

  /*!
    Copy operator of a 2D array.
  */
  vpArray2D<Type> &operator=(const vpArray2D<Type> &A)
  {
    resize(A.rowNum, A.colNum, false, false);
    memcpy(data, A.data, rowNum * colNum * sizeof(Type));
    return *this;
  }

  //! Set element \f$A_{ij} = x\f$ using A[i][j] = x
  inline Type *operator[](unsigned int i) { return rowPtrs[i]; }
  //! Get element \f$x = A_{ij}\f$ using x = A[i][j]
  inline Type *operator[](unsigned int i) const { return rowPtrs[i]; }

  /*!
    \relates vpArray2D
    Writes the given array to the output stream and returns a reference to the
    output stream.
    */
  friend std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
  {
    if (A.data == NULL || A.size() == 0)
      return s;
    std::ios_base::fmtflags original_flags = s.flags();

    s.precision(10);
    for (unsigned int i = 0; i < A.getRows(); i++) {
      for (unsigned int j = 0; j < A.getCols() - 1; j++) {
        s << A[i][j] << "  ";
      }
      // We don't add "  " after the last row element
      s << A[i][A.getCols() - 1];
      // We don't add a \n char on the end of the last array line
      if (i < A.getRows() - 1)
        s << std::endl;
    }

    s.flags(original_flags); // restore s to standard state

    return s;
  }

  vpArray2D<Type> hadamard(const vpArray2D<Type> &m) const;
  //@}

  //---------------------------------
  // Inherited array I/O  Static Public Member Functions
  //---------------------------------
  /** @name Inherited I/O from vpArray2D with Static Public Member Functions
   */
  //@{
  /*!
    Load a matrix from a file.

    \param filename : Absolute file name.
    \param A : Array to be loaded
    \param binary : If true the matrix is loaded from a binary file, else from
    a text file. \param header : Header of the file is loaded in this
    parameter.

    \return Returns true if success.

    \sa save()
  */
  static bool load(const std::string &filename, vpArray2D<Type> &A, const bool binary = false, char *header = NULL)
  {
    std::fstream file;

    if (!binary)
      file.open(filename.c_str(), std::fstream::in);
    else
      file.open(filename.c_str(), std::fstream::in | std::fstream::binary);

    if (!file) {
      file.close();
      return false;
    }

    if (!binary) {
      std::string h;
      bool headerIsDecoded = false;
      do {
        std::streampos pos = file.tellg();
        char line[256];
        file.getline(line, 256);
        std::string prefix("# ");
        std::string line_(line);
        if (line_.compare(0, 2, prefix.c_str()) == 0) {
          // Line is a comment
          // If we are not on the first line, we should add "\n" to the end of
          // the previous line
          if (pos)
            h += "\n";
          h += line_.substr(2); // Remove "# "
        } else {
          // rewind before the line
          file.seekg(pos, file.beg);
          headerIsDecoded = true;
        }
      } while (!headerIsDecoded);

      if (header != NULL) {
#if defined(__MINGW32__) ||                                                                                            \
    !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
        sprintf(header, "%s", h.c_str());
#else
        _snprintf_s(header, h.size() + 1, _TRUNCATE, "%s", h.c_str());
#endif
      }

      unsigned int rows, cols;
      file >> rows;
      file >> cols;

      if (rows >= (std::numeric_limits<unsigned int>::max)() || cols >= (std::numeric_limits<unsigned int>::max)())
        throw vpException(vpException::badValue, "Array exceed the max size.");

      A.resize(rows, cols);

      Type value;
      for (unsigned int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < cols; j++) {
          file >> value;
          A[i][j] = value;
        }
      }
    } else {
      char c = '0';
      std::string h;
      // Decode header until '\0' char that ends the header string
      while ((c != '\0')) {
        file.read(&c, 1);
        h += c;
      }
      if (header != NULL) {
#if defined(__MINGW32__) ||                                                                                            \
    !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
        sprintf(header, "%s", h.c_str());
#else
        _snprintf_s(header, h.size() + 1, _TRUNCATE, "%s", h.c_str());
#endif
      }

      unsigned int rows, cols;
      file.read((char *)&rows, sizeof(unsigned int));
      file.read((char *)&cols, sizeof(unsigned int));
      A.resize(rows, cols);

      Type value;
      for (unsigned int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < cols; j++) {
          file.read((char *)&value, sizeof(Type));
          A[i][j] = value;
        }
      }
    }

    file.close();
    return true;
  }
  /*!
    Load an array from a YAML-formatted file.

    \param filename : absolute file name.
    \param A : array to be loaded from the file.
    \param header : header of the file is loaded in this parameter.

    \return Returns true on success.

    \sa saveYAML()

  */
  static bool loadYAML(const std::string &filename, vpArray2D<Type> &A, char *header = NULL)
  {
    std::fstream file;

    file.open(filename.c_str(), std::fstream::in);

    if (!file) {
      file.close();
      return false;
    }

    unsigned int rows = 0, cols = 0;
    std::string h;
    std::string line, subs;
    bool inheader = true;
    unsigned int i = 0, j;
    unsigned int lineStart = 0;

    while (getline(file, line)) {
      if (inheader) {
        if (rows == 0 && line.compare(0, 5, "rows:") == 0) {
          std::stringstream ss(line);
          ss >> subs;
          ss >> rows;
        } else if (cols == 0 && line.compare(0, 5, "cols:") == 0) {
          std::stringstream ss(line);
          ss >> subs;
          ss >> cols;
        } else if (line.compare(0, 5, "data:") == 0)
          inheader = false;
        else
          h += line + "\n";
      } else {
        // if i == 0, we just got out of the header: initialize matrix
        // dimensions
        if (i == 0) {
          if (rows == 0 || cols == 0) {
            file.close();
            return false;
          }
          A.resize(rows, cols);
          // get indentation level which is common to all lines
          lineStart = (unsigned int)line.find("[") + 1;
        }
        std::stringstream ss(line.substr(lineStart, line.find("]") - lineStart));
        j = 0;
        while (getline(ss, subs, ','))
          A[i][j++] = atof(subs.c_str());
        i++;
      }
    }

    if (header != NULL) {
      std::string h_ = h.substr(0, h.size() - 1); // Remove last '\n' char
#if defined(__MINGW32__) ||                                                                                            \
    !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
      sprintf(header, "%s", h_.c_str());
#else
      _snprintf_s(header, h_.size() + 1, _TRUNCATE, "%s", h_.c_str());
#endif
    }

    file.close();
    return true;
  }

  /*!
    Save a matrix to a file.

    \param filename : Absolute file name.
    \param A : Array to be saved.
    \param binary : If true the matrix is saved in a binary file, else a text
    file. \param header : Optional line that will be saved at the beginning of
    the file.

    \return Returns true if success.

    Warning : If you save the matrix as in a text file the precision is
    less than if you save it in a binary file.

    \sa load()
  */
  static bool save(const std::string &filename, const vpArray2D<Type> &A, const bool binary = false,
                   const char *header = "")
  {
    std::fstream file;

    if (!binary)
      file.open(filename.c_str(), std::fstream::out);
    else
      file.open(filename.c_str(), std::fstream::out | std::fstream::binary);

    if (!file) {
      file.close();
      return false;
    }

    if (!binary) {
      unsigned int i = 0;
      file << "# ";
      while (header[i] != '\0') {
        file << header[i];
        if (header[i] == '\n')
          file << "# ";
        i++;
      }
      file << std::endl;
      file << A.getRows() << "\t" << A.getCols() << std::endl;
      file << A << std::endl;
    } else {
      int headerSize = 0;
      while (header[headerSize] != '\0')
        headerSize++;
      file.write(header, headerSize + 1);
      unsigned int matrixSize;
      matrixSize = A.getRows();
      file.write((char *)&matrixSize, sizeof(unsigned int));
      matrixSize = A.getCols();
      file.write((char *)&matrixSize, sizeof(unsigned int));
      Type value;
      for (unsigned int i = 0; i < A.getRows(); i++) {
        for (unsigned int j = 0; j < A.getCols(); j++) {
          value = A[i][j];
          file.write((char *)&value, sizeof(Type));
        }
      }
    }

    file.close();
    return true;
  }
  /*!
    Save an array in a YAML-formatted file.

    \param filename : absolute file name.
    \param A : array to be saved in the file.
    \param header : optional lines that will be saved at the beginning of the
  file. Should be YAML-formatted and will adapt to the indentation if any.

    \return Returns true if success.

    Here is an example of outputs.
  \code
  vpArray2D<double> M(3,4);
  vpArray2D::saveYAML("matrix.yml", M, "example: a YAML-formatted header");
  vpArray2D::saveYAML("matrixIndent.yml", M, "example:\n    - a YAML-formatted
  header\n    - with inner indentation"); \endcode Content of matrix.yml:
  \code
  example: a YAML-formatted header
  rows: 3
  cols: 4
  data:
    - [0, 0, 0, 0]
    - [0, 0, 0, 0]
    - [0, 0, 0, 0]
  \endcode
  Content of matrixIndent.yml:
  \code
  example:
      - a YAML-formatted header
      - with inner indentation
  rows: 3
  cols: 4
  data:
      - [0, 0, 0, 0]
      - [0, 0, 0, 0]
      - [0, 0, 0, 0]
  \endcode

    \sa loadYAML()
  */
  static bool saveYAML(const std::string &filename, const vpArray2D<Type> &A, const char *header = "")
  {
    std::fstream file;

    file.open(filename.c_str(), std::fstream::out);

    if (!file) {
      file.close();
      return false;
    }

    unsigned int i = 0;
    bool inIndent = false;
    std::string indent = "";
    bool checkIndent = true;
    while (header[i] != '\0') {
      file << header[i];
      if (checkIndent) {
        if (inIndent) {
          if (header[i] == ' ')
            indent += " ";
          else if (indent.length() > 0)
            checkIndent = false;
        }
        if (header[i] == '\n' || (inIndent && header[i] == ' '))
          inIndent = true;
        else
          inIndent = false;
      }
      i++;
    }

    if (i != 0)
      file << std::endl;
    file << "rows: " << A.getRows() << std::endl;
    file << "cols: " << A.getCols() << std::endl;

    if (indent.length() == 0)
      indent = "  ";

    file << "data: " << std::endl;
    unsigned int j;
    for (i = 0; i < A.getRows(); ++i) {
      file << indent << "- [";
      for (j = 0; j < A.getCols() - 1; ++j)
        file << A[i][j] << ", ";
      file << A[i][j] << "]" << std::endl;
    }

    file.close();
    return true;
  }
  //@}
};

/*!
 Return the array min value.
 */
template <class Type> Type vpArray2D<Type>::getMinValue() const
{
  Type *dataptr = data;
  Type min = *dataptr;
  dataptr++;
  for (unsigned int i = 0; i < dsize - 1; i++) {
    if (*dataptr < min)
      min = *dataptr;
    dataptr++;
  }
  return min;
}

/*!
 Return the array max value.
 */
template <class Type> Type vpArray2D<Type>::getMaxValue() const
{
  Type *dataptr = data;
  Type max = *dataptr;
  dataptr++;
  for (unsigned int i = 0; i < dsize - 1; i++) {
    if (*dataptr > max)
      max = *dataptr;
    dataptr++;
  }
  return max;
}

/*!
  Compute the Hadamard product (element wise matrix multiplication).
  \param m : Second matrix;
  \return m1.hadamard(m2) The Hadamard product : \f$ m1 \circ m2 = (m1 \circ
  m2)_{i,j} = (m1)_{i,j} (m2)_{i,j} \f$
*/
template <class Type> vpArray2D<Type> vpArray2D<Type>::hadamard(const vpArray2D<Type> &m) const
{
  if (m.getRows() != rowNum || m.getCols() != colNum) {
    throw(vpException(vpException::dimensionError, "Hadamard product: bad dimensions!"));
  }

  vpArray2D<Type> out;
  out.resize(rowNum, colNum, false);

  for (unsigned int i = 0; i < dsize; i++) {
    out.data[i] = data[i] * m.data[i];
  }

  return out;
}

#endif
