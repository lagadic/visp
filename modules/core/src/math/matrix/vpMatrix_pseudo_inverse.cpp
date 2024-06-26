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
 * Pseudo inverse computation.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif

#ifdef VISP_HAVE_LAPACK
#ifdef VISP_HAVE_GSL
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#elif defined(VISP_HAVE_MKL)
#include <mkl.h>
#endif
#endif

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS

void compute_pseudo_inverse(const vpMatrix &U, const vpColVector &sv, const vpMatrix &V, unsigned int nrows,
                            unsigned int ncols, double svThreshold, vpMatrix &Ap, int &rank_out, int *rank_in,
                            vpMatrix *imA, vpMatrix *imAt, vpMatrix *kerAt)
{
  Ap.resize(ncols, nrows, true, false);

  // compute the highest singular value and the rank of h
  double maxsv = sv[0];

  rank_out = 0;

  unsigned int sv_size = sv.size();
  for (unsigned int i = 0; i < sv_size; ++i) {
    if (sv[i] >(maxsv * svThreshold)) {
      ++rank_out;
    }
  }

  unsigned int rank = static_cast<unsigned int>(rank_out);
  if (rank_in) {
    rank = static_cast<unsigned int>(*rank_in);
  }

  for (unsigned int i = 0; i < ncols; ++i) {
    for (unsigned int j = 0; j < nrows; ++j) {
      for (unsigned int k = 0; k < rank; ++k) {
        Ap[i][j] += (V[i][k] * U[j][k]) / sv[k];
      }
    }
  }

  // Compute im(A)
  if (imA) {
    imA->resize(nrows, rank);

    for (unsigned int i = 0; i < nrows; ++i) {
      for (unsigned int j = 0; j < rank; ++j) {
        (*imA)[i][j] = U[i][j];
      }
    }
  }

  // Compute im(At)
  if (imAt) {
    imAt->resize(ncols, rank);
    for (unsigned int i = 0; i < ncols; ++i) {
      for (unsigned int j = 0; j < rank; ++j) {
        (*imAt)[i][j] = V[i][j];
      }
    }
  }

  // Compute ker(At)
  if (kerAt) {
    kerAt->resize(ncols - rank, ncols);
    if (rank != ncols) {
      unsigned int v_rows = V.getRows();
      for (unsigned int k = 0; k < (ncols - rank); ++k) {
        unsigned j = k + rank;
        for (unsigned int i = 0; i < v_rows; ++i) {
          (*kerAt)[k][i] = V[i][j];
        }
      }
    }
  }
}

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    unsigned int rank = A.pseudoInverse(A_p);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank: " << rank << std::endl;
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank: 2
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, double svThreshold) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, svThreshold);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, svThreshold);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, svThreshold);
#else
  (void)Ap;
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    // This matrix rank is 2
    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    int rank_in = 2;
    int rank_out = A.pseudoInverse(A_p, rank_in);
    if (rank_out != rank_in) {
      std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
      std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
    }

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank in : " << rank_in << std::endl;
    std::cout << "Rank out: " << rank_out << std::endl;
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank in : 2
  Rank out: 2
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, int rank_in) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, rank_in);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, rank_in);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, rank_in);
#else
  (void)Ap;
  (void)rank_in;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p = A.pseudoInverse();

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  \endcode

*/
vpMatrix vpMatrix::pseudoInverse(double svThreshold) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(svThreshold);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(svThreshold);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(svThreshold);
#else
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[in] rank_in : Known rank of the matrix.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    // This matrix rank is 2
    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    int rank_in = 2;
    vpMatrix A_p = A.pseudoInverseLapack(rank_in);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  \endcode

*/
vpMatrix vpMatrix::pseudoInverse(int rank_in) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(rank_in);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(rank_in);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(rank_in);
#else
  (void)rank_in;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    vpColVector sv;
    unsigned int rank = A.pseudoInverse(A_p, sv);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

    std::cout << "Rank: " << rank << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank: 2
  Singular values: 6.874359351  4.443330227
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, svThreshold);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, svThreshold);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, svThreshold);
#else
  (void)Ap;
  (void)sv;
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    vpColVector sv;
    int rank_in = 2;
    int rank_out = A.pseudoInverse(A_p, sv, rank_in);
    if (rank_out != rank_in) {
      std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
      std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
    }

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

    std::cout << "Rank in : " << rank_in << std::endl;
    std::cout << "Rank out: " << rank_out << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank in : 2
  Rank out: 2
  Singular values: 6.874359351  4.443330227
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, rank_in);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, rank_in);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, rank_in);
#else
  (void)Ap;
  (void)sv;
  (void)rank_in;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}
/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$ and \f$\mbox{Im}(A^T)\f$
  and return the rank of the matrix.

  See pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
  for a complete description of this function.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    vpColVector sv;
    vpMatrix imA, imAt;
    unsigned int rank = A.pseudoInverse(A_p, sv, 1e-6, imA, imAt);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank: " << rank << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
    imA.print(std::cout, 10, "Im(A): ");
    imAt.print(std::cout, 10, "Im(A^T): ");
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank: 2
  Singular values: 6.874359351  4.443330227
  Im(A): [2,2]=
    0.81458 -0.58003
    0.58003  0.81458
  Im(A^T): [3,2]=
    -0.100515 -0.994397
    0.524244 -0.024967
    0.845615 -0.102722
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt) const
{
  vpMatrix kerAt;
  return pseudoInverse(Ap, sv, svThreshold, imA, imAt, kerAt);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$ and \f$\mbox{Im}(A^T)\f$
  and return the rank of the matrix.

  See pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
  for a complete description of this function.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    vpColVector sv;
    vpMatrix imA, imAt;
    int rank_in = 2;
    int rank_out = A.pseudoInverse(A_p, sv, rank_in, imA, imAt);
    if (rank_out != rank_in) {
      std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
      std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
    }

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank in : " << rank_in << std::endl;
    std::cout << "Rank out: " << rank_in << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
    imA.print(std::cout, 10, "Im(A): ");
    imAt.print(std::cout, 10, "Im(A^T): ");
  }
  \endcode

  Once build, the previous example produces the following output:
  \code
  A: [2,3]=
    2  3  5
    -4  2  3
  A^+ (pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank: 2
  Singular values: 6.874359351  4.443330227
  Im(A): [2,2]=
    0.81458 -0.58003
    0.58003  0.81458
  Im(A^T): [3,2]=
    -0.100515 -0.994397
    0.524244 -0.024967
    0.845615 -0.102722
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt) const
{
  vpMatrix kerAt;
  return pseudoInverse(Ap, sv, rank_in, imA, imAt, kerAt);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv : Vector corresponding to matrix A singular values. The size of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values.If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA : \f$\mbox { Im }({ \bf A })\f$ that is a m - by - r matrix.

  \param imAt : \f$\mbox { Im }({ \bf A } ^ T)\f$ that is n - by - r matrix.

  \param kerAt : The matrix that contains the null space(kernel) of \f$\bf
  A\f$ defined by the matrix \f$ { \bf X } ^ T\f$.If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is(0, n), otherwise the dimension is (n - r, n).
  This matrix is thus the transpose of \f$\mbox { Ker }({ \bf A })\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo - inverse of a 2 - by - 3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpColVector sv;
    vpMatrix A_p, imA, imAt, kerAt;
    unsigned int rank = A.pseudoInverse(A_p, sv, 1e-6, imA, imAt, kerAt);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank: " << rank << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
    imA.print(std::cout, 10, "Im(A): ");
    imAt.print(std::cout, 10, "Im(A^T): ");

    if (kerAt.size()) {
      kerAt.t().print(std::cout, 10, "Ker(A): ");
    }
    else {
      std::cout << "Ker(A) empty " << std::endl;
    }

    // Reconstruct matrix A from ImA, ImAt, KerAt
    vpMatrix S(rank, A.getCols());
    for (unsigned int i = 0; i< rank; i++)
      S[i][i] = sv[i];
    vpMatrix Vt(A.getCols(), A.getCols());
    Vt.insert(imAt.t(), 0, 0);
    Vt.insert(kerAt, rank, 0);
    (imA *S *Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
  }
  \endcode

    Once build, the previous example produces the following output :

  \code{.sh}
  A: [2,3] =
    2  3  5
    -4  2  3
  A^+(pseudo-inverse): [3,2]=
    0.117899 -0.190782
    0.065380  0.039657
    0.113612  0.052518
  Rank: 2
  Singular values: 6.874359351  4.443330227
  Im(A): [2,2]=
    0.81458 -0.58003
    0.58003  0.81458
  Im(A^T): [3,2] =
    -0.100515 -0.994397
    0.524244 -0.024967
    0.845615 -0.102722
  Ker(A): [3,1]=
    -0.032738
    -0.851202
    0.523816
  Im(A) * S * [Im(A^T) | Ker(A)]^T: [2,3]=
    2  3  5
    -4  2  3
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt,
                                     vpMatrix &kerAt) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, svThreshold, imA, imAt, kerAt);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, svThreshold, imA, imAt, kerAt);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, svThreshold, imA, imAt, kerAt);
#else
  (void)Ap;
  (void)sv;
  (void)svThreshold;
  (void)imA;
  (void)imAt;
  (void)kerAt;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox { Ker }(A)\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party.It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names(Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n - by - n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  \param Ap : The Moore - Penros pseudo inverse \f$ { \bf A } ^ +\f$.

  \param sv : Vector corresponding to matrix \f$A\f$ singular values.The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA : \f$\mbox { Im }({ \bf A })\f$ that is a m - by - r matrix.

  \param imAt : \f$\mbox { Im }({ \bf A } ^T)\f$ that is n - by - r matrix.

  \param kerAt : The matrix that contains the null space(kernel) of \f$\bf
  A\f$ defined by the matrix \f$ { \bf X } ^ T\f$.If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is(0, n), otherwise the dimension is(n - r,
  n).This matrix is thus the transpose of \f$\mbox { Ker }({ \bf A })\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo - inverse of a 2 - by - 3 matrix that is rank 2.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(2, 3);

    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpColVector sv;
    vpMatrix A_p, imA, imAt, kerAt;
    int rank_in = 2;
    int rank_out = A.pseudoInverse(A_p, sv, rank_in, imA, imAt, kerAt);
    if (rank_out != rank_in) {
      std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
      std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
    }

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank in : " << rank_in << std::endl;
    std::cout << "Rank out: " << rank_out << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
    imA.print(std::cout, 10, "Im(A): ");
    imAt.print(std::cout, 10, "Im(A^T): ");

    if (kerAt.size()) {
      kerAt.t().print(std::cout, 10, "Ker(A): ");
    }
    else {
      std::cout << "Ker(A) empty " << std::endl;
    }

    // Reconstruct matrix A from ImA, ImAt, KerAt
    vpMatrix S(rank, A.getCols());
    for (unsigned int i = 0; i < rank_in; i++)
      S[i][i] = sv[i];
    vpMatrix Vt(A.getCols(), A.getCols());
    Vt.insert(imAt.t(), 0, 0);
    Vt.insert(kerAt, rank, 0);
    (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
  }
  \endcode

  Once build, the previous example produces the following output :
  \code
  A : [2, 3] =
  2  3  5
  - 4  2  3
  A ^ +(pseudo - inverse) : [3, 2] =
  0.117899 - 0.190782
  0.065380  0.039657
  0.113612  0.052518
  Rank in : 2
  Rank out : 2
  Singular values : 6.874359351  4.443330227
  Im(A) : [2, 2] =
  0.81458 - 0.58003
  0.58003  0.81458
  Im(A ^ T) : [3, 2] =
  -0.100515 - 0.994397
  0.524244 - 0.024967
  0.845615 - 0.102722
  Ker(A) : [3, 1] =
  -0.032738
  - 0.851202
  0.523816
  Im(A) * S *[Im(A ^ T) | Ker(A)] ^ T : [2, 3] =
  2  3  5
  - 4  2  3
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt,
                            vpMatrix &kerAt) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, rank_in, imA, imAt, kerAt);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, rank_in, imA, imAt, kerAt);
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, rank_in, imA, imAt, kerAt);
#else
  (void)Ap;
  (void)sv;
  (void)rank_in;
  (void)imA;
  (void)imAt;
  (void)kerAt;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

END_VISP_NAMESPACE
