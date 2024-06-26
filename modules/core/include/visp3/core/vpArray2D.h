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
 * This class implements an 2D array as a template class.
 */
#ifndef VP_ARRAY2D_H
#define VP_ARRAY2D_H

#include <fstream>
#include <iostream>
#include <limits>
#include <math.h>
#include <ostream>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \class vpArray2D
 *  \ingroup group_core_matrices
 *
 *  \brief Implementation of a generic 2D array used as base class for matrices
 *  and vectors.
 *
 *  This class implements a 2D array as a template class and all the basic
 *  functionalities common to matrices and vectors. More precisely:
 *  - concerning matrices, vpMatrix but also specific containers such as twist
 *    (vpVelocityTwistMatrix and vpForceTwistMatrix), homogeneous
 *    (vpHomogeneousMatrix), rotation (vpRotationMatrix) and homography
 *    (vpHomography) matrices inherit from vpArray2D<double>.
 *  - concerning vectors, vpColVector, vpRowVector but also specific containers
 *   describing the pose (vpPoseVector) and the rotation (vpRotationVector)
 *   inherit also from vpArray2D<double>.
 *
 * The code below shows how to create a 2-by-3 array of doubles, set the element values and access them:
 * \code
 * #include <visp3/code/vpArray2D.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpArray2D<float> a(2, 3);
 *   a[0][0] = -1; a[0][1] =  -2; a[0][2] = -3;
 *   a[1][0] =  4; a[1][1] = 5.5; a[1][2] =  6;
 *
 *   std::cout << "a:" << std::endl;
 *   for (unsigned int i = 0; i < a.getRows(); ++i) {
 *     for (unsigned int j = 0; j < a.getCols(); ++j) {
 *       std::cout << a[i][j] << " ";
 *     }
 *     std::cout << std::endl;
 *   }
 * }
 * \endcode
 * Once build, this previous code produces the following output:
 * \code
 * a:
 * -1 -2 -3
 * 4 5.5 6
 * \endcode
 *  If ViSP is build with c++11 enabled, you can do the same using:
 * \code
 * #include <visp3/code/vpArray2D.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpArray2D<float> a{ {-1, -2, -3}, {4, 5.5, 6.0f} };
 *   std::cout << "a:\n" << a << std::endl;
 * }
 * \endcode
 * The array could also be initialized using operator=(const std::initializer_list< std::initializer_list< Type > > &)
 * \code
 * #include <visp3/code/vpArray2D.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpArray2D<float> a;
 *   a = { {-1, -2, -3}, {4, 5.5, 6.0f} };
 * }
 * \endcode
 *
 * You can also use reshape() function:
 * \code
 * #include <visp3/code/vpArray2D.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpArray2D<float> a{ -1, -2, -3, 4, 5.5, 6.0f };
 *   a.reshape(2, 3);
 * }
 * \endcode
*/
template <class Type> class vpArray2D
{
public:
  //! Address of the first element of the data array
  Type *data;

  /*!
   * Basic constructor of a 2D array.
   * Number of columns and rows are set to zero.
   */
  vpArray2D<Type>() : data(nullptr), rowNum(0), colNum(0), rowPtrs(nullptr), dsize(0) { }

  /*!
    Copy constructor of a 2D array.
  */
  vpArray2D<Type>(const vpArray2D<Type> &A)
    :
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
    vpArray2D<Type>()
#else
    data(nullptr), rowNum(0), colNum(0), rowPtrs(nullptr), dsize(0)
#endif
  {
    resize(A.rowNum, A.colNum, false, false);
    memcpy(data, A.data, static_cast<size_t>(rowNum) * static_cast<size_t>(colNum) * sizeof(Type));
  }

  /*!
    Constructor that initializes a 2D array with 0.

    \param r : Array number of rows.
    \param c : Array number of columns.
  */
  vpArray2D<Type>(unsigned int r, unsigned int c)
    :
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
    vpArray2D<Type>()
#else
    data(nullptr), rowNum(0), colNum(0), rowPtrs(nullptr), dsize(0)
#endif
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
    :
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
    vpArray2D<Type>()
#else
    data(nullptr), rowNum(0), colNum(0), rowPtrs(nullptr), dsize(0)
#endif
  {
    resize(r, c, false, false);
    *this = val;
  }

  /*!
    Constructor that initialize a 2D array from a std::vector.

    - When c = 0, create a colum vector with dimension (data.size, 1)
    - When r = 0, create a row vector with dimension (1, data.size)
    - Otherwise create an array with dimension (r, c)

    \param r : Array number of rows.
    \param c : Array number of columns.
    \param vec : Data used to initialize the 2D array.
  */
  vpArray2D<Type>(const std::vector<Type> &vec, unsigned int r = 0, unsigned int c = 0)
    :
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
    vpArray2D<Type>()
#else
    data(nullptr), rowNum(0), colNum(0), rowPtrs(nullptr), dsize(0)
#endif
  {
    if ((r > 0) && (c > 0)) {
      if ((r * c) != vec.size()) {
        throw(vpException(vpException::dimensionError,
                          "Cannot initialize vpArray(%d, %d) from std::vector(%d). Wrong dimension", r, c, vec.size()));
      }
      resize(r, c, false, false);
    }
    else if ((c == 0) && (r == 0)) {
      throw(vpException(vpException::dimensionError,
                        "Cannot initialize vpArray(%d, %d) from std::vector(%d). Using rows = 0 and cols = 0 is ambiguous", r, c, vec.size()));
    }
    else if (c == 0) {
      if (r != vec.size()) {
        throw(vpException(vpException::dimensionError,
                          "Cannot initialize vpArray(%d, %d) from std::vector(%d). Wrong dimension", r, c, vec.size()));
      }
      resize(static_cast<unsigned int>(vec.size()), 1, false, false);
    }
    else if (r == 0) {
      if (c != vec.size()) {
        throw(vpException(vpException::dimensionError,
                          "Cannot initialize vpArray(%d, %d) from std::vector(%d). Wrong dimension", r, c, vec.size()));
      }
      resize(1, static_cast<unsigned int>(vec.size()), false, false);
    }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
    std::copy(vec.begin(), vec.end(), data);
#else
    memcpy(data, vec.data(), vec.size() * sizeof(Type));
#endif
  }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  vpArray2D<Type>(vpArray2D<Type> &&A) noexcept
  {
    rowNum = A.rowNum;
    colNum = A.colNum;
    rowPtrs = A.rowPtrs;
    dsize = A.dsize;
    data = A.data;

    A.rowNum = 0;
    A.colNum = 0;
    A.rowPtrs = nullptr;
    A.dsize = 0;
    A.data = nullptr;
  }

  VP_EXPLICIT vpArray2D<Type>(const std::initializer_list<Type> &list) : vpArray2D<Type>()
  {
    resize(1, static_cast<unsigned int>(list.size()), false, false);
    std::copy(list.begin(), list.end(), data);
  }

  VP_EXPLICIT vpArray2D<Type>(unsigned int nrows, unsigned int ncols, const std::initializer_list<Type> &list)
    : data(nullptr), rowNum(0), colNum(0), rowPtrs(nullptr), dsize(0)
  {
    if ((nrows * ncols) != static_cast<unsigned int>(list.size())) {
      std::ostringstream oss;
      oss << "Cannot create a vpArray2D of size (" << nrows << ", " << ncols << ") with a list of size " << list.size();
      throw vpException(vpException::dimensionError, oss.str());
    }

    resize(nrows, ncols, false, false);
    std::copy(list.begin(), list.end(), data);
  }

  VP_EXPLICIT vpArray2D<Type>(const std::initializer_list<std::initializer_list<Type> > &lists) : vpArray2D<Type>()
  {
    unsigned int nrows = static_cast<unsigned int>(lists.size()), ncols = 0;
    for (auto &l : lists) {
      if (static_cast<unsigned int>(l.size()) > ncols) {
        ncols = static_cast<unsigned int>(l.size());
      }
    }

    resize(nrows, ncols, false, false);
    auto it = lists.begin();
    for (unsigned int i = 0; i < rowNum; ++i, ++it) {
      std::copy(it->begin(), it->end(), rowPtrs[i]);
    }
  }
#endif

  /*!
   * Destructor that deallocate memory.
   */
  virtual ~vpArray2D<Type>()
  {
    if (data != nullptr) {
      free(data);
      data = nullptr;
    }

    if (rowPtrs != nullptr) {
      free(rowPtrs);
      rowPtrs = nullptr;
    }
    rowNum = 0;
    colNum = 0;
    dsize = 0;
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
  \param recopy_ : if true, will perform an explicit recopy of the old data.
  */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify = true, bool recopy_ = true)
  {
    if ((nrows == rowNum) && (ncols == colNum)) {
      if (flagNullify && (this->data != nullptr)) {
        memset(this->data, 0, this->dsize * sizeof(Type));
      }
    }
    else {
      bool recopy = (!flagNullify) && recopy_; // priority to flagNullify
      bool colcond = (ncols != this->colNum) && (this->colNum > 0) && (ncols > 0);
      const bool recopyNeeded = colcond && ((!flagNullify) || recopy);
      Type *copyTmp = nullptr;
      unsigned int rowTmp = 0, colTmp = 0;

      // Recopy case per case is required if number of cols has changed;
      // structure of Type array is not the same in this case.
      if (recopyNeeded && (this->data != nullptr)) {
        copyTmp = new Type[this->dsize];
        memcpy(copyTmp, this->data, sizeof(Type) * this->dsize);
        rowTmp = this->rowNum;
        colTmp = this->colNum;
      }

      // Reallocation of this->data array
      this->dsize = nrows * ncols;
      Type *tmp_data = reinterpret_cast<Type *>(realloc(this->data, this->dsize * sizeof(Type)));
      if (tmp_data) {
        this->data = tmp_data;
      }
      else {
        this->data = nullptr;
      }

      if ((nullptr == this->data) && (0 != this->dsize)) {
        if (copyTmp != nullptr) {
          delete[] copyTmp;
        }
        throw(vpException(vpException::memoryAllocationError, "Memory allocation error when allocating 2D array data"));
      }

      Type **tmp_rowPtrs = reinterpret_cast<Type **>(realloc(this->rowPtrs, nrows * sizeof(Type *)));
      if (tmp_rowPtrs) {
        this->rowPtrs = tmp_rowPtrs;
      }
      else {
        this->rowPtrs = nullptr;
      }
      if ((nullptr == this->rowPtrs) && (0 != this->dsize)) {
        if (copyTmp != nullptr) {
          delete[] copyTmp;
        }
        throw(vpException(vpException::memoryAllocationError,
                          "Memory allocation error when allocating 2D array rowPtrs"));
      }

      // Update rowPtrs

      Type **t_ = rowPtrs;
      for (unsigned int i = 0; i < dsize; i += ncols) {
        *t_++ = this->data + i;
      }

      this->rowNum = nrows;
      this->colNum = ncols;

      // Recopy of this->data array values or nullify
      if (flagNullify) {
        memset(this->data, 0, static_cast<size_t>(this->dsize) * sizeof(Type));
      }
      else if (recopyNeeded && (this->rowPtrs != nullptr)) {
        // Recopy...
        unsigned int minRow = (this->rowNum < rowTmp) ? this->rowNum : rowTmp;
        unsigned int minCol = (this->colNum < colTmp) ? this->colNum : colTmp;
        for (unsigned int i = 0; i < this->rowNum; ++i) {
          for (unsigned int j = 0; j < this->colNum; ++j) {
            if ((minRow > i) && (minCol > j)) {
              (*this)[i][j] = copyTmp[(i * colTmp) + j];
            }
            else {
              (*this)[i][j] = 0;
            }
          }
        }
      }

      if (copyTmp != nullptr) {
        delete[] copyTmp;
      }
    }
  }

  void reshape(unsigned int nrows, unsigned int ncols)
  {
    if (dsize == 0) {
      resize(nrows, ncols);
      return;
    }

    if ((nrows * ncols) != dsize) {
      std::ostringstream oss;
      oss << "Cannot reshape array of total size " << dsize << " into shape (" << nrows << ", " << ncols << ")";
      throw vpException(vpException::dimensionError, oss.str());
    }

    rowNum = nrows;
    colNum = ncols;
    if (rowPtrs) {
      Type **tmp = reinterpret_cast<Type **>(realloc(rowPtrs, nrows * sizeof(Type *)));
      if (tmp) {
        this->rowPtrs = tmp;
      }
    }
    if (rowPtrs) {
      // Update rowPtrs
      Type **t_ = rowPtrs;
      for (unsigned int i = 0; i < dsize; i += ncols) {
        *t_++ = data + i;
      }
    }
  }


  /*!
    Insert array A at the given position in the current array.

    \warning Throw vpException::dimensionError if the
    dimensions of the matrices do not allow the operation.

    \param A : The array to insert.
    \param r : The index of the row to begin to insert data.
    \param c : The index of the column to begin to insert data.
  */
  void insert(const vpArray2D<Type> &A, unsigned int r, unsigned int c)
  {
    if (((r + A.getRows()) <= rowNum) && ((c + A.getCols()) <= colNum)) {
      if ((A.colNum == colNum) && (data != nullptr) && (A.data != nullptr) && (A.data != data)) {
        memcpy(data + (r * colNum), A.data, sizeof(Type) * A.size());
      }
      else if ((data != nullptr) && (A.data != nullptr) && (A.data != data)) {
        unsigned int a_rows = A.getRows();
        for (unsigned int i = r; i < (r + a_rows); ++i) {
          memcpy(data + (i * colNum) + c, A.data + ((i - r) * A.colNum), sizeof(Type) * A.colNum);
        }
      }
    }
    else {
      throw vpException(vpException::dimensionError, "Cannot insert (%dx%d) array in (%dx%d) array at position (%d,%d)",
                        A.getRows(), A.getCols(), rowNum, colNum, r, c);
    }
  }

  /*!
    Equal to comparison operator of a 2D array.
  */
  bool operator==(const vpArray2D<Type> &A) const;
  /*!
    Not equal to comparison operator of a 2D array.
  */
  bool operator!=(const vpArray2D<Type> &A) const;

  //! Set all the elements of the array to \e x.
  vpArray2D<Type> &operator=(Type x)
  {
    std::fill(data, data + dsize, x);
    return *this;
  }

  /*!
    Copy operator of a 2D array.
  */
  vpArray2D<Type> &operator=(const vpArray2D<Type> &A)
  {
    resize(A.rowNum, A.colNum, false, false);
    if ((data != nullptr) && (A.data != nullptr) && (data != A.data)) {
      memcpy(data, A.data, static_cast<size_t>(rowNum) * static_cast<size_t>(colNum) * sizeof(Type));
    }
    return *this;
  }

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  vpArray2D<Type> &operator=(vpArray2D<Type> &&other) noexcept
  {
    if (this != &other) {
      if (data) {
        free(data);
      }
      if (rowPtrs) {
        free(rowPtrs);
      }

      rowNum = other.rowNum;
      colNum = other.colNum;
      rowPtrs = other.rowPtrs;
      dsize = other.dsize;
      data = other.data;

      other.rowNum = 0;
      other.colNum = 0;
      other.rowPtrs = nullptr;
      other.dsize = 0;
      other.data = nullptr;
    }

    return *this;
  }

  vpArray2D<Type> &operator=(const std::initializer_list<Type> &list)
  {
    if (dsize != static_cast<unsigned int>(list.size())) {
      resize(1, static_cast<unsigned int>(list.size()), false, false);
    }
    std::copy(list.begin(), list.end(), data);

    return *this;
  }

  vpArray2D<Type> &operator=(const std::initializer_list<std::initializer_list<Type> > &lists)
  {
    unsigned int nrows = static_cast<unsigned int>(lists.size()), ncols = 0;
    for (auto &l : lists) {
      if (static_cast<unsigned int>(l.size()) > ncols) {
        ncols = static_cast<unsigned int>(l.size());
      }
    }

    resize(nrows, ncols, false, false);
    auto it = lists.begin();
    for (unsigned int i = 0; i < rowNum; ++i, ++it) {
      std::copy(it->begin(), it->end(), rowPtrs[i]);
    }

    return *this;
  }

#ifdef VISP_HAVE_NLOHMANN_JSON
  vpArray2D<Type> &operator=(const nlohmann::json &j) = delete;
#endif
#endif

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
    if ((A.data == nullptr) || (A.size() == 0)) {
      return s;
    }
    std::ios_base::fmtflags original_flags = s.flags();
    const unsigned int precision = 10;
    s.precision(precision);
    unsigned int a_rows = A.getRows();
    unsigned int a_cols = A.getCols();
    for (unsigned int i = 0; i < a_rows; ++i) {
      for (unsigned int j = 0; j < (a_cols - 1); ++j) {
        s << A[i][j] << "  ";
      }
      // We don't add "  " after the last row element
      s << A[i][a_cols - 1];
      // We don't add a \n char on the end of the last array line
      if (i < (a_rows - 1)) {
        s << std::endl;
      }
    }

    s.flags(original_flags); // restore s to standard state

    return s;
  }

  vpArray2D<Type> hadamard(const vpArray2D<Type> &m) const;

  /**
   * \brief  Compute the transpose of the array
   *
   * @return vpArray2D<Type> C = A^T
   */
  vpArray2D<Type> t() const;
  //@}

  //---------------------------------
  // Inherited array I/O  Static Public Member Functions
  //---------------------------------
  /** @name Inherited I/O from vpArray2D with Static Public Member Functions */
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
  static bool load(const std::string &filename, vpArray2D<Type> &A, bool binary = false, char *header = nullptr)
  {
    std::fstream file;

    if (!binary) {
      file.open(filename.c_str(), std::fstream::in);
    }
    else {
      file.open(filename.c_str(), std::fstream::in | std::fstream::binary);
    }

    if (!file) {
      file.close();
      return false;
    }

    if (!binary) {
      std::string h;
      bool headerIsDecoded = false;
      do {
        std::streampos pos = file.tellg();
        char line[FILENAME_MAX];
        file.getline(line, FILENAME_MAX);
        std::string prefix("# ");
        std::string line_(line);
        if (line_.compare(0, prefix.size(), prefix.c_str()) == 0) {
          // Line is a comment
          // If we are not on the first line, we should add "\n" to the end of
          // the previous line
          if (pos) {
            h += "\n";
          }
          h += line_.substr(prefix.size()); // Remove "# "
        }
        else {
          // rewind before the line
          file.seekg(pos, file.beg);
          headerIsDecoded = true;
        }
      } while (!headerIsDecoded);

      if (header != nullptr) {
#if defined(__MINGW32__) ||                                                                                            \
    !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
        snprintf(header, h.size() + 1, "%s", h.c_str());
#else
        _snprintf_s(header, h.size() + 1, _TRUNCATE, "%s", h.c_str());
#endif
      }

      unsigned int rows, cols;
      file >> rows;
      file >> cols;

      if ((rows >= std::numeric_limits<unsigned int>::max()) || (cols >= std::numeric_limits<unsigned int>::max())) {
        throw vpException(vpException::badValue, "Array exceed the max size.");
      }

      A.resize(rows, cols);

      Type value;
      for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
          file >> value;
          A[i][j] = value;
        }
      }
    }
    else {
      char c = '0';
      std::string h;
      // Decode header until '\0' char that ends the header string
      while (c != '\0') {
        file.read(&c, 1);
        h += c;
      }
      if (header != nullptr) {
#if defined(__MINGW32__) || \
    !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
        snprintf(header, h.size() + 1, "%s", h.c_str());
#else
        _snprintf_s(header, h.size() + 1, _TRUNCATE, "%s", h.c_str());
#endif
      }

      unsigned int rows, cols;
      file.read(reinterpret_cast<char *>(&rows), sizeof(unsigned int));
      file.read(reinterpret_cast<char *>(&cols), sizeof(unsigned int));
      A.resize(rows, cols);

      Type value;
      for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
          file.read(reinterpret_cast<char *>(&value), sizeof(Type));
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
  static bool loadYAML(const std::string &filename, vpArray2D<Type> &A, char *header = nullptr)
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
        const std::string str_rows("rows:");
        const std::string str_cols("cols:");
        const std::string str_data("data:");
        if ((rows == 0) && (line.compare(0, str_rows.size(), str_rows.c_str()) == 0)) {
          std::stringstream ss(line);
          ss >> subs;
          ss >> rows;
        }
        else if ((cols == 0) && (line.compare(0, str_cols.size(), str_cols.c_str()) == 0)) {
          std::stringstream ss(line);
          ss >> subs;
          ss >> cols;
        }
        else if (line.compare(0, str_data.size(), str_data.c_str()) == 0) {
          inheader = false;
        }
        else {
          h += line + "\n";
        }
      }
      else {
        // if i == 0, we just got out of the header: initialize matrix
        // dimensions
        if (i == 0) {
          if ((rows == 0) || (cols == 0)) {
            file.close();
            return false;
          }
          A.resize(rows, cols);
          // get indentation level which is common to all lines
          lineStart = static_cast<unsigned int>(line.find("[")) + 1;
        }
        std::stringstream ss(line.substr(lineStart, line.find("]") - lineStart));
        j = 0;
        while (getline(ss, subs, ',')) {
          A[i][j++] = atof(subs.c_str());
        }
        ++i;
      }
    }

    if (header != nullptr) {
      std::string h_ = h.substr(0, h.size() - 1); // Remove last '\n' char
#if defined(__MINGW32__) || \
    !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
      snprintf(header, h_.size() + 1, "%s", h_.c_str());
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
  static bool save(const std::string &filename, const vpArray2D<Type> &A, bool binary = false, const char *header = "")
  {
    std::fstream file;

    if (!binary) {
      file.open(filename.c_str(), std::fstream::out);
    }
    else {
      file.open(filename.c_str(), std::fstream::out | std::fstream::binary);
    }

    if (!file) {
      file.close();
      return false;
    }

    if (!binary) {
      unsigned int i = 0;
      file << "# ";
      while (header[i] != '\0') {
        file << header[i];
        if (header[i] == '\n') {
          file << "# ";
        }
        ++i;
      }
      file << std::endl;
      file << A.getRows() << "\t" << A.getCols() << std::endl;
      file << A << std::endl;
    }
    else {
      int headerSize = 0;
      while (header[headerSize] != '\0') {
        ++headerSize;
      }
      file.write(header, static_cast<size_t>(headerSize)+static_cast<size_t>(1));
      unsigned int matrixSize;
      matrixSize = A.getRows();
      file.write(reinterpret_cast<char *>(&matrixSize), sizeof(unsigned int));
      matrixSize = A.getCols();
      file.write(reinterpret_cast<char *>(&matrixSize), sizeof(unsigned int));
      Type value;
      unsigned int a_rows = A.getRows();
      unsigned int a_cols = A.getCols();
      for (unsigned int i = 0; i < a_rows; ++i) {
        for (unsigned int j = 0; j < a_cols; ++j) {
          value = A[i][j];
          file.write(reinterpret_cast<char *>(&value), sizeof(Type));
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
      vpArray2D::saveYAML("matrixIndent.yml", M, "example:\n    - a YAML-formatted \
      header\n    - with inner indentation");
      \endcode
      Content of matrix.yml:
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
          if (header[i] == ' ') {
            indent += " ";
          }
          else if (indent.length() > 0) {
            checkIndent = false;
          }
        }
        if ((header[i] == '\n') || (inIndent && (header[i] == ' '))) {
          inIndent = true;
        }
        else {
          inIndent = false;
        }
      }
      ++i;
    }

    if (i != 0) {
      file << std::endl;
    }
    file << "rows: " << A.getRows() << std::endl;
    file << "cols: " << A.getCols() << std::endl;

    if (indent.length() == 0) {
      indent = "  ";
    }

    file << "data: " << std::endl;
    unsigned int j;
    unsigned int a_rows = A.getRows();
    unsigned int a_cols = A.getCols();
    for (i = 0; i < a_rows; ++i) {
      file << indent << "- [";
      for (j = 0; j < (a_cols - 1); ++j) {
        file << A[i][j] << ", ";
      }
      file << A[i][j] << "]" << std::endl;
    }

    file.close();
    return true;
  }
#ifdef VISP_HAVE_NLOHMANN_JSON
  //template<typename Type>
  template<class T>
  friend void from_json(const nlohmann::json &j, vpArray2D<T> &array);
  //template<typename Type>
  template<class T>
  friend void to_json(nlohmann::json &j, const vpArray2D<T> &array);
#endif

  /*!
    Perform a 2D convolution similar to Matlab conv2 function: \f$ M \star kernel \f$.

    \param M : First matrix.
    \param kernel : Second matrix.
    \param mode : Convolution mode: "full" (default), "same", "valid".

    \image html vpMatrix-conv2-mode.jpg "Convolution mode: full, same, valid (image credit: Theano doc)."

    \note This is a very basic implementation that does not use FFT.
  */
  static vpArray2D<Type> conv2(const vpArray2D<Type> &M, const vpArray2D<Type> &kernel, const std::string &mode);

  /*!
    Perform a 2D convolution similar to Matlab conv2 function: \f$ M \star kernel \f$.

    \param M : First array.
    \param kernel : Second array.
    \param res : Result.
    \param mode : Convolution mode: "full" (default), "same", "valid".

    \image html vpMatrix-conv2-mode.jpg "Convolution mode: full, same, valid (image credit: Theano doc)."

    \note This is a very basic implementation that does not use FFT.
  */
  static void conv2(const vpArray2D<Type> &M, const vpArray2D<Type> &kernel, vpArray2D<Type> &res, const std::string &mode);

  /*!
    Insert array B in array A at the given position.

    \param A : Main array.
    \param B : Array to insert.
    \param r : Index of the row where to add the array.
    \param c : Index of the column where to add the array.
    \return Array with B insert in A.

    \warning Throw exception if the sizes of the arrays do not allow the
    insertion.
  */
  vpArray2D<Type> insert(const vpArray2D<Type> &A, const vpArray2D<Type> &B, unsigned int r, unsigned int c);

  /*!
    \relates vpArray2D
    Insert array B in array A at the given position.

    \param A : Main array.
    \param B : Array to insert.
    \param C : Result array.
    \param r : Index of the row where to insert array B.
    \param c : Index of the column where to insert array B.

    \warning Throw exception if the sizes of the arrays do not
    allow the insertion.
  */
  static void insert(const vpArray2D<Type> &A, const vpArray2D<Type> &B, vpArray2D<Type> &C, unsigned int r, unsigned int c);
  //@}

protected:
  //! Number of rows in the array
  unsigned int rowNum;
  //! Number of columns in the array
  unsigned int colNum;
  //! Address of the first element of each rows
  Type **rowPtrs;
  //! Current array size (rowNum * colNum)
  unsigned int dsize;
};

/*!
  Return the array min value.
 */
template <class Type> Type vpArray2D<Type>::getMinValue() const
{
  Type *dataptr = data;
  Type min = *dataptr;
  ++dataptr;
  for (unsigned int i = 0; i < (dsize - 1); ++i) {
    if (*dataptr < min) {
      min = *dataptr;
    }
    ++dataptr;
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
  ++dataptr;
  for (unsigned int i = 0; i < (dsize - 1); ++i) {
    if (*dataptr > max) {
      max = *dataptr;
    }
    ++dataptr;
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
  if ((m.getRows() != rowNum) || (m.getCols() != colNum)) {
    throw(vpException(vpException::dimensionError, "Hadamard product: bad dimensions!"));
  }

  vpArray2D<Type> out;
  out.resize(rowNum, colNum, false);

  for (unsigned int i = 0; i < dsize; ++i) {
    out.data[i] = data[i] * m.data[i];
  }

  return out;
}

template <class Type> vpArray2D<Type> vpArray2D<Type>::t() const
{
  vpArray2D<Type> At(colNum, rowNum);
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      At[j][i] = (*this)[i][j];
    }
  }
  return At;
}

template <class Type> vpArray2D<Type> vpArray2D<Type>::conv2(const vpArray2D<Type> &M, const vpArray2D<Type> &kernel, const std::string &mode)
{
  vpArray2D<Type> res;
  conv2(M, kernel, res, mode);
  return res;
}

template <class Type> void vpArray2D<Type>::conv2(const vpArray2D<Type> &M, const vpArray2D<Type> &kernel, vpArray2D<Type> &res, const std::string &mode)
{
  if (((M.getRows() * M.getCols()) == 0) || ((kernel.getRows() * kernel.getCols()) == 0)) {
    return;
  }

  if (mode == "valid") {
    if ((kernel.getRows() > M.getRows()) || (kernel.getCols() > M.getCols())) {
      return;
    }
  }

  vpArray2D<Type> M_padded, res_same;

  if ((mode == "full") || (mode == "same")) {
    const unsigned int pad_x = kernel.getCols() - 1;
    const unsigned int pad_y = kernel.getRows() - 1;
    const unsigned int pad = 2;
    M_padded.resize(M.getRows() + (pad * pad_y), M.getCols() + (pad * pad_x), true, false);
    M_padded.insert(M, pad_y, pad_x);

    if (mode == "same") {
      res.resize(M.getRows(), M.getCols(), false, false);
      res_same.resize(M.getRows() + pad_y, M.getCols() + pad_x, true, false);
    }
    else {
      res.resize(M.getRows() + pad_y, M.getCols() + pad_x, true, false);
    }
  }
  else if (mode == "valid") {
    M_padded = M;
    res.resize((M.getRows() - kernel.getRows()) + 1, (M.getCols() - kernel.getCols()) + 1);
  }
  else {
    return;
  }

  if (mode == "same") {
    unsigned int res_same_rows = res_same.getRows();
    unsigned int res_same_cols = res_same.getCols();
    unsigned int kernel_rows = kernel.getRows();
    unsigned int kernel_cols = kernel.getCols();
    for (unsigned int i = 0; i < res_same_rows; ++i) {
      for (unsigned int j = 0; j < res_same_cols; ++j) {
        for (unsigned int k = 0; k < kernel_rows; ++k) {
          for (unsigned int l = 0; l < kernel_cols; ++l) {
            res_same[i][j] += M_padded[i + k][j + l] * kernel[kernel.getRows() - k - 1][kernel.getCols() - l - 1];
          }
        }
      }
    }

    const unsigned int start_i = kernel.getRows() / 2;
    const unsigned int start_j = kernel.getCols() / 2;
    unsigned int m_rows = M.getRows();
    for (unsigned int i = 0; i < m_rows; ++i) {
      memcpy(res.data + (i * M.getCols()), res_same.data + ((i + start_i) * res_same.getCols()) + start_j,
             sizeof(Type) * M.getCols());
    }
  }
  else {
    unsigned int res_rows = res.getRows();
    unsigned int res_cols = res.getCols();
    unsigned int kernel_rows = kernel.getRows();
    unsigned int kernel_cols = kernel.getCols();
    for (unsigned int i = 0; i < res_rows; ++i) {
      for (unsigned int j = 0; j < res_cols; ++j) {
        for (unsigned int k = 0; k < kernel_rows; ++k) {
          for (unsigned int l = 0; l < kernel_cols; ++l) {
            res[i][j] += M_padded[i + k][j + l] * kernel[kernel.getRows() - k - 1][kernel.getCols() - l - 1];
          }
        }
      }
    }
  }
}

template<class Type> vpArray2D<Type> vpArray2D<Type>::insert(const vpArray2D<Type> &A, const vpArray2D<Type> &B, unsigned int r, unsigned int c)
{
  vpArray2D<Type> C;

  insert(A, B, C, r, c);

  return C;
}

template<class Type> void vpArray2D<Type>::insert(const vpArray2D<Type> &A, const vpArray2D<Type> &B, vpArray2D<Type> &C, unsigned int r, unsigned int c)
{
  if (((r + B.getRows()) <= A.getRows()) && ((c + B.getCols()) <= A.getCols())) {
    C.resize(A.getRows(), A.getCols(), false, false);

    unsigned int a_rows = A.getRows();
    unsigned int a_cols = A.getCols();
    for (unsigned int i = 0; i < a_rows; ++i) {
      for (unsigned int j = 0; j < a_cols; ++j) {
        if ((i >= r) && (i < (r + B.getRows())) && (j >= c) && (j < (c + B.getCols()))) {
          C[i][j] = B[i - r][j - c];
        }
        else {
          C[i][j] = A[i][j];
        }
      }
    }
  }
  else {
    throw vpException(vpException::dimensionError, "Cannot insert (%dx%d) array in (%dx%d) array at position (%d,%d)",
                      B.getRows(), B.getCols(), A.getCols(), A.getRows(), r, c);
  }
}

template <class Type> bool vpArray2D<Type>::operator==(const vpArray2D<Type> &A) const
{
  if ((A.rowNum != rowNum) || (A.colNum != colNum)) {
    return false;
  }

  unsigned int a_size = A.size();
  for (unsigned int i = 0; i < a_size; ++i) {
    if (data[i] != A.data[i]) {
      return false;
    }
  }

  return true;
}

/*!
 * \relates vpArray2D
 */
template <> inline bool vpArray2D<double>::operator==(const vpArray2D<double> &A) const
{
  if ((A.rowNum != rowNum) || (A.colNum != colNum)) {
    return false;
  }

  unsigned int a_size = A.size();
  for (unsigned int i = 0; i < a_size; ++i) {
    if (fabs(data[i] - A.data[i]) > std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }

  return true;
}

/*!
 * \relates vpArray2D
 */
template <> inline bool vpArray2D<float>::operator==(const vpArray2D<float> &A) const
{
  if ((A.rowNum != rowNum) || (A.colNum != colNum)) {
    return false;
  }

  unsigned int a_size = A.size();
  for (unsigned int i = 0; i < a_size; ++i) {
    if (fabsf(data[i] - A.data[i]) > std::numeric_limits<float>::epsilon()) {
      return false;
    }
  }

  return true;
}

/*!
 * \relates vpArray2D
 */
template <class Type> bool vpArray2D<Type>::operator!=(const vpArray2D<Type> &A) const { return !(*this == A); }

#ifdef VISP_HAVE_NLOHMANN_JSON
template <class Type>
inline void from_json(const nlohmann::json &j, vpArray2D<Type> &array)
{
  if (j.is_array()) {
    const unsigned int nrows = static_cast<unsigned int>(j.size());
    if (nrows == 0) { // Initialize an empty array, Finished
      array.resize(0, 0);
      return;
    }
    unsigned int ncols = 0;
    bool first = true;
    for (const auto &item : j) { // Find number of columns, validate that all rows have same number of cols
      if (!item.is_array()) {
        throw vpException(vpException::badValue, "Trying to instantiate a 2D array with a JSON object that is not an array of array");
      }
      if (first) {
        first = false;
        ncols = static_cast<unsigned int>(item.size());
      }
      else if (ncols != item.size()) {
        throw vpException(vpException::badValue, "Trying to instantiate a 2D array with JSON row arrays that are not of the same size");
      }
    }
    array.resize(nrows, ncols);
    unsigned i = 0;
    for (const auto &item : j) {
      std::vector<Type> row = item;
      std::copy(row.begin(), row.end(), array.rowPtrs[i]);
      ++i;
    }
  }
  else if (j.is_object()) {
    const unsigned ncols = j.at("cols");
    const unsigned nrows = j.at("rows");
    array.resize(nrows, ncols);
    const nlohmann::json jData = j.at("data");
    if (!jData.is_array() || jData.size() != nrows * ncols) {
      std::stringstream ss;
      ss << "JSON \"data\" field must be an array of size " << nrows * ncols;
      throw vpException(vpException::badValue, ss.str());
    }
    unsigned i = 0;
    for (const auto &jValue : jData) {
      array.data[i] = jValue;
      ++i;
    }
  }
  else {
    throw vpException(vpException::badValue, "Trying to read a vpArray2D from something that is not an array or object");
  }
}


template <class Type>
inline void to_json(nlohmann::json &j, const vpArray2D<Type> &array)
{
  j = {
    {"cols", array.colNum},
    {"rows", array.rowNum},
    {"type", "vpArray2D"}
  };

  nlohmann::json::array_t data;
  data.reserve(array.size());
  for (unsigned i = 0; i < array.size(); ++i) {
    data.push_back(array.data[i]);
  }
  j["data"] = data;
}
#endif

END_VISP_NAMESPACE

#endif
