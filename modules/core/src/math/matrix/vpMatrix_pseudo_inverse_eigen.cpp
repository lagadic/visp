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

#include "private/vpMatrix_pseudo_inverse.h"

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#if defined(VISP_HAVE_EIGEN3)
/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using Eigen3 3rd party.

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

    vpMatrix A_p = A.pseudoInverseEigen3();

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  }
  \endcode

  \sa pseudoInverse(double)
*/
vpMatrix vpMatrix::pseudoInverseEigen3(double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V, Ap;
  vpColVector sv;

  U = *this;
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, nullptr, nullptr, nullptr, nullptr);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using Eigen3 3rd party.

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
    unsigned int rank = A.pseudoInverseEigen3(A_p);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank: " << rank << std::endl;
  }
  \endcode

  \sa pseudoInverse(vpMatrix &, double) const
*/
unsigned int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V;
  vpColVector sv;

  U = *this;
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, nullptr, nullptr, nullptr, nullptr);

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  Eigen3 3rd party.

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
    unsigned int rank = A.pseudoInverseEigen3(A_p, sv);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

    std::cout << "Rank: " << rank << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
  }
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double) const
*/
unsigned int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V;

  Ap.resize(ncols, nrows, false, false);

  U = *this;
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, nullptr, nullptr, nullptr, nullptr);

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using Eigen3 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

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

    vpColVector sv;
    vpMatrix A_p, imA, imAt, kerAt;
    unsigned int rank = A.pseudoInverseEigen3(A_p, sv, 1e-6, imA, imAt, kerAt);

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
    for(unsigned int i = 0; i< rank; i++)
      S[i][i] = sv[i];
    vpMatrix Vt(A.getCols(), A.getCols());
    Vt.insert(imAt.t(), 0, 0);
    Vt.insert(kerAt, rank, 0);
    (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
  }
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
unsigned int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA,
                                           vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    U.insert(*this, 0, 0);
  }
  else {
    U = *this;
  }

  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, nullptr, &imA, &imAt, &kerAt);

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using Eigen3 3rd party.

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
    vpMatrix A_p = A.pseudoInverseEigen3(rank_in);

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    return 0;
  }
  \endcode

  \sa pseudoInverse(int) const
*/
vpMatrix vpMatrix::pseudoInverseEigen3(int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V, Ap;
  vpColVector sv;

  U = *this;
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, nullptr, nullptr, nullptr);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[out] Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param[in] rank_in : Known rank of the matrix.

  \return The computed rank of the matrix.

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
    int rank_out = A.pseudoInverseEigen3(A_p, rank_in);
    if (rank_out != rank_in) {
      std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
      std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
    }

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank in : " << rank_in << std::endl;
    std::cout << "Rank out: " << rank_out << std::endl;
    return 0;
  }
  \endcode

  \sa pseudoInverse(vpMatrix &, int) const
*/
int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  U = *this;
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, nullptr, nullptr, nullptr);

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  Eigen3 3rd party.

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

    // This matrix rank is 2
    A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
    A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

    A.print(std::cout, 10, "A: ");

    vpMatrix A_p;
    vpColVector sv;
    int rank_in = 2;

    int rank_out = A.pseudoInverseEigen3(A_p, sv, rank_in);
    if (rank_out != rank_in) {
      std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
      std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
    }

    A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
    std::cout << "Rank in : " << rank_in << std::endl;
    std::cout << "Rank out: " << rank_out << std::endl;
    std::cout << "Singular values: " << sv.t() << std::endl;
    return 0;
  }
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int) const
*/
int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V;

  Ap.resize(ncols, nrows, false, false);

  U = *this;
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, nullptr, nullptr, nullptr);

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using Eigen3 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

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

    vpColVector sv;
    vpMatrix A_p, imA, imAt, kerAt;
    int rank_in = 2;
    int rank_out = A.pseudoInverseEigen3(A_p, sv, rank_in, imA, imAt, kerAt);

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
    vpMatrix S(rank_in, A.getCols());
    for(unsigned int i = 0; i< rank_in; i++)
      S[i][i] = sv[i];
    vpMatrix Vt(A.getCols(), A.getCols());
    Vt.insert(imAt.t(), 0, 0);
    Vt.insert(kerAt, rank_in, 0);
    (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
  }
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt,
                                  vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    U.insert(*this, 0, 0);
  }
  else {
    U = *this;
  }
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, &imA, &imAt, &kerAt);

  return rank_out;
}
#endif

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

END_VISP_NAMESPACE
