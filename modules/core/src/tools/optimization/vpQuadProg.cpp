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
 * Quadratic Programming
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/

#include <algorithm>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpQuadProg.h>

#ifdef VISP_HAVE_CPP11_COMPATIBILITY

/*!
Changes a canonical quadratic cost \f$\min \frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{c}^T\mathbf{x}\f$
to the formulation used by this class \f$ \min ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2\f$.

Computes \f$(\mathbf{Q}, \mathbf{r})\f$ such that \f$\mathbf{H} = \mathbf{Q}^T\mathbf{Q}\f$ and \f$\mathbf{c} = -\mathbf{Q}^T\mathbf{r}\f$.

  \param H : canonical symmetric positive cost matrix (dimension n x n)
  \param c : canonical cost vector (dimension n)
  \param Q : custom cost matrix (dimension rank(H) x n)
  \param r : custom cost vector (dimension rank(H))
  \param tol : tolerance to test ranks

  \warning This method is only available if the Gnu Scientific Library
  (GSL) is detected as a third party library.

  Here is an example:

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & 2x_1^2 + x_2^2 + x_1x_2 + x_1 + x_2\\
              & \text{s.t.}& x_1 + x_2 = 1
              \end{array}
\Leftrightarrow
\begin{array}{lll}
  \mathbf{x} = &  \arg\min & \frac{1}{2}\mathbf{x}^T\left[\begin{array}{cc}4 & 1 \\ 1 & 2\end{array}\right]\mathbf{x} + [1~1]\mathbf{x}\\
               & \text{s.t.}& [1~1]\mathbf{x} = 1
                \end{array}
\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix H(2,2), A(1,2);
    vpColVector c(2), b(1);

    H[0][0] = 4;
    H[0][1] = H[1][0] = 1;
    H[1][1] = 2;
    c[0] = c[1] = 1;
    A[0][0] = A[0][1] = 1;
    b[0] = 1;

    vpMatrix Q;vpColVector r;
    vpQuadProg::fromCanonicalCost(H, c, Q, r);

    vpColVector x;
    vpQuadProg::solveQPe(Q, r, A, b, x);
    std::cout << x.t() << std::endl;  // prints (0.25 0.75)
  }
  \endcode
*/
#ifdef VISP_HAVE_GSL /* be careful of the copy below */
void vpQuadProg::fromCanonicalCost(const vpMatrix &H, const vpColVector &c, vpMatrix &Q, vpColVector &r, const double &tol)
#else
void vpQuadProg::fromCanonicalCost(const vpMatrix &/*H*/, const vpColVector &/*c*/,
                                   vpMatrix &/*Q*/, vpColVector &/*r*/, const double &/*tol*/)
#endif

{
#ifdef VISP_HAVE_GSL
  const unsigned int n = H.getCols();
  if(H.getRows() != n || c.getRows() != n)
  {
    throw(vpException(vpMatrixException::dimensionError, "vpQuadProg::fromCanonicalCost: H is not square or not the same dimension as c"));
  }

  vpColVector d(n);
  vpMatrix    V(n,n);
  // Compute the eigenvalues and eigenvectors
  H.eigenValues(d, V);
  // find first non-null eigen value
  unsigned int k = 0;
  for(unsigned int i = 0; i < n; ++i)
  {
    if(d[i] > tol)
      d[i] = sqrt(d[i]);
    else if(d[i] < tol)
      throw(vpException(vpMatrixException::matrixError, "vpQuadProg::fromCanonicalCost: H is not positive"));
    else
      k = i+1;
  }
  // build (Q,r) such that H = Q.^T.Q and c = -Q^T.r
  vpMatrix D(n-k,n-k);
  vpMatrix P(n-k,n);
  D.diag(d.extract(k,n-k));
  for(unsigned int i = 0; i < n-k; ++i)
    P[i][k+i] = 1;

  Q = D*P*V.transpose();
  r = -Q.t().pseudoInverse()*c;
#else
  throw(vpException(vpException::functionNotImplementedError, "Symmetric matrix decomposition is not implemented. You "
                                                              "should install GSL 3rd party"));
#endif
}

/*!
  Saves internally the column reduction of the passed equality constraint:

  \f$\mathbf{A}\mathbf{x} = \mathbf{b} \Leftrightarrow \mathbf{x} = \mathbf{x}_1 + \mathbf{Z}\mathbf{z}
  \f$
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param tol : tolerance to test the ranks

  \return True if \f$\mathbf{A}\mathbf{x} = \mathbf{b}\f$ has a solution.

  \sa solveQPi(), solveQP()
*/
bool vpQuadProg::setEqualityConstraint(const vpMatrix &A, const vpColVector &b, const double &tol)
{
  x1 = b;
  Z = A;
  if(A.getRows() == b.getRows() && vpLinProg::colReduction(Z, x1, false, tol))
    return true;

  std::cout << "vpQuadProg::setEqualityConstraint: equality constraint infeasible" << std::endl;
  return false;
}

/*!
  Solves a Quadratic Program under equality constraints.

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2 \\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b}\end{array}
\f$
  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param x : solution (dimension n)
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  This function is for internal use, no dimension check is performed and A and b may be modified.
*/
bool vpQuadProg::solveByProjection(const vpMatrix &Q, const vpColVector &r,
                                   vpMatrix &A, vpColVector &b,
                                   vpColVector &x, const double &tol)
{
  if(A.getRows())
  {
    if(!vpLinProg::colReduction(A, b, false, tol))
      return false;

    if(A.getCols() && (Q*A).infinityNorm() > tol)
      x = b + A*(Q*A).solveBySVD(r - Q*b);
    else
      x = b;
  }
  else
    x = Q.solveBySVD(r);
  return true;
}

/*!
  Solves a Quadratic Program under previously stored equality constraints (setEqualityConstraint())

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2 \\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b}\end{array}
  \f$
  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param x : solution (dimension n)
  \param tol : Tolerance.

  \return True if the solution was found.

  Here is an example where the two following QPs are solved:

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min  & x_1^2 + x_2^2  \\
               & \text{s.t.}& x_1 + x_2 = 1\end{array}\f$

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & (x_1-1)^2 + x_2^2  \\
               & \text{s.t.}& x_1 + x_2 = 1\end{array}\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix Q(2,2), A(1,2);
    vpColVector r(2), b(1);

    Q[0][0] = Q[1][1] = 1;
    A[0][0] = A[0][1] = b[0] = 1;

    vpQuadProg qp;
    qp.setEqualityConstraint(A, b);
    vpColVector x;

    // solve x_1^2 + x_2^2
    qp.solveQPe(Q, r, x);
    std::cout << x.t() << std::endl;  // prints (0.5 0.5)

    // solve (x_1-1)^2 + x_2^2
    r[0] = 1;
    qp.solveQPe(Q, r, x);
    std::cout << x.t() << std::endl;  // prints (1 0)
  }
  \endcode

  \sa setEqualityConstraint(), solveQPe()
*/
bool vpQuadProg::solveQPe (const vpMatrix &Q, const vpColVector &r,
                           vpColVector &x, const double &tol) const
{
  const unsigned int n = Q.getCols();
  if(Q.getRows() != r.getRows() ||
     Z.getRows() != n ||
     x1.getRows() != n)
  {
    std::cout << "vpQuadProg::solveQPe: wrong dimension\n" <<
                 "Q: " << Q.getRows() << "x" << Q.getCols() << " - r: " << r.getRows() << std::endl;
    std::cout << "Z: " << Z.getRows() << "x" << Z.getCols() << " - x1: " << x1.getRows() << std::endl;
    throw vpMatrixException::dimensionError;
  }
  if(Z.getCols())
  {
    if((Q*Z).infinityNorm() > tol)
      x = x1 + Z*(Q*Z).solveBySVD(r - Q*x1);
    else
      x = x1;
  }
  else
    x = Q.solveBySVD(r);
  return true;
}

/*!
  Solves a Quadratic Program under equality constraints.

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2 \\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b}\end{array}\f$

  If the equality constraints \f$(\mathbf{A}, \mathbf{b})\f$ are the same between two calls, it is more efficient to use setEqualityConstraint() and solveQPe().

  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param x : solution (dimension n)
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  Here is an example:

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & (x_1-1)^2 + x_2^2  \\
               & \text{s.t.}& x_1 + x_2 = 1\end{array}\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix Q(2,2), A(1,2);
    vpColVector r(2), b(1);

    Q[0][0] = Q[1][1] = 1;
    r[0] = 1;
    A[0][0] = A[0][1] = b[0] = 1;

    vpColVector x;

    vpQuadProg::solveQPe(Q, r, A, b, x);
    std::cout << x.t() << std::endl;  // prints (1 0)
  }
  \endcode

  \sa setEqualityConstraint(), solveQP(), solveQPi()
*/
bool vpQuadProg::solveQPe (const vpMatrix &Q, const vpColVector &r, vpMatrix A, vpColVector b,
                           vpColVector &x, const double &tol)
{
  checkDimensions(Q, r, &A, &b, NULL, NULL, "solveQPe");

  if(!solveByProjection(Q, r, A, b, x, tol))
  {
    std::cout << "vpQuadProg::solveQPe: equality constraint infeasible" << std::endl;
    return false;
  }
  return true;
}

/*!
  Solves a Quadratic Program under equality and inequality constraints.

  If the equality constraints \f$(\mathbf{A}, \mathbf{b})\f$ are the same between two calls, it is more efficient to use setEqualityConstraint() and solveQPi().

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2 \\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b} \\
               & \text{s.t.}& \mathbf{C}\mathbf{x} \leq \mathbf{d}
\end{array}
\f$
  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param C : inequality matrix (dimension p x n)
  \param d : inequality vector (dimension p)
  \param x : solution (dimension n)
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  Here is an example:

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & (x_1-1)^2 + x_2^2  \\
               & \text{s.t.}& x_1 + x_2 = 1  \\
                & \text{s.t.} & x_2 \geq 1\end{array}\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix Q(2,2), A(1,2), C(1,2);
    vpColVector r(2), b(1), d(1);

    Q[0][0] = Q[1][1] = 1;    r[0] = 1;
    A[0][0] = A[0][1] = 1;    b[0] = 1;
    C[0][1] = -1; d[0] = -1;

    vpColVector x;
    vpQuadProg qp;

    qp.solveQP(Q, r, A, b, C, d, x);
    std::cout << x.t() << std::endl;  // prints (0 1)
  }
  \endcode

  \sa resetActiveSet()
*/
bool vpQuadProg::solveQP(const vpMatrix &Q, const vpColVector &r,
                         vpMatrix A, vpColVector b,
                         const vpMatrix &C, const vpColVector &d,
                         vpColVector &x,
                         const double &tol)
{
  if(A.getRows() == 0)
    return solveQPi(Q, r, C, d, x, false, tol);

  checkDimensions(Q, r, &A, &b, &C, &d, "solveQP");

  if(!vpLinProg::colReduction(A, b, false, tol))
  {
    std::cout << "vpQuadProg::solveQP: equality constraint infeasible" << std::endl;
    return false;
  }

  if(A.getCols() && solveQPi(Q*A, r - Q*b, C*A, d - C*b, x, false, tol))
  {
    x = b + A*x;
    return true;
  }
  else if(vpLinProg::allLesser(C, b, d, tol))  // Ax = b has only 1 solution
  {
    x = b;
    return true;
  }
  std::cout << "vpQuadProg::solveQP: inequality constraint infeasible" << std::endl;
  return false;
}

/*!
  Solves a Quadratic Program under inequality constraints

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2 \\
               & \text{s.t.}& \mathbf{C}\mathbf{x} \leq \mathbf{d}
\end{array}
\f$
  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param C : inequality matrix (dimension p x n)
  \param d : inequality vector (dimension p)
  \param x : solution (dimension n)
  \param use_equality : if a previously saved equality constraint (setEqualityConstraint()) should be considered
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  Here is an example:

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & (x_1-1)^2 + x_2^2  \\
               & \text{s.t.}& x_1 + x_2 \leq 1  \\
                & \text{s.t.} & x_1, x_2 \geq 0\end{array}\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix Q(2,2), C(3,2);
    vpColVector r(2), d(1);

    Q[0][0] = Q[1][1] = 1;    r[0] = 1;
    C[0][0] = C[0][1] = 1;    d[0] = 1;
    C[1][0] = C[2][1] = -1;

    vpColVector x;
    vpQuadProg qp;

    qp.solveQPi(Q, r, C, d, x);
    std::cout << x.t() << std::endl;  // prints (1 0)
  }
  \endcode
*/
bool vpQuadProg::solveQPi(const vpMatrix &Q, const vpColVector &r,
                          const vpMatrix &C, const vpColVector &d,
                          vpColVector &x, const bool use_equality,
                          const double &tol)
{
  const unsigned int n = checkDimensions(Q, r, NULL, NULL, &C, &d, "solveQPi");

  if(use_equality)
  {
    if(Z.getRows() == n)
    {
      if(Z.getCols() && solveQPi(Q*Z, r - Q*x1, C*Z, d - C*x1, x, false, tol))
      {
        // back to initial solution
        x = x1 + Z*x;
        return true;
      }
      else if(vpLinProg::allLesser(C, x1, d, tol))
      {
        x = x1;
        return true;
      }
      std::cout << "vpQuadProg::solveQPi: inequality constraint infeasible" << std::endl;
      return false;
    }
    else
      std::cout << "vpQuadProg::solveQPi: use_equality before setEqualityConstraint" << std::endl;
  }

  const unsigned int p = C.getRows();

  // look for trivial solution
  // r = 0 and d > 0 -> x = 0
  if(vpLinProg::allZero(r, tol) &&
     (d.getRows() == 0 || vpLinProg::allGreater(d, -tol)))
  {
    x.resize(n);
    return true;
  }

  // go for solver
  // build feasible point
  vpMatrix A;
  vpColVector b;
  // check active set - all values should be < rows of C
  for(auto v: active)
  {
    if(v >= p)
    {
      active.clear();
      std::cout << "vpQuadProg::solveQPi: some constraints have been removed since last call\n";
      break;
    }
  }

  // warm start from previous active set
  A.resize((unsigned int)active.size(), n);
  b.resize((unsigned int)active.size());
  for(unsigned int i = 0; i < active.size(); ++i)
  {
    for(unsigned int j = 0; j < n; ++j)
      A[i][j] = C[active[i]][j];
    b[i] = d[active[i]];
  }
  if(!solveByProjection(Q, r, A, b, x, tol))
    x.resize(n);

  // or from simplex if we really have no clue
  if(!vpLinProg::allLesser(C, x, d, tol))
  {
    // feasible point with simplex:
    //  min r
    //   st C.(x + z1 - z2) + y - r = d
    //   st z1, z2, y, r >= 0
    // dim r = violated constraints
    // feasible if r can be minimized to 0

    // count how many violated constraints
    vpColVector e = d - C*x;
    unsigned int k = 0;
    for(unsigned int i = 0; i < p; ++i)
    {
      if(e[i] < -tol)
        k++;
    }
    // cost vector
    vpColVector c(2*n+p+k);
    for(unsigned int i = 0; i < k; ++i)
      c[2*n+p+i] = 1;

    vpColVector xc(2*n+p+k);

    vpMatrix A_lp(p, 2*n+p+k);
    unsigned int l = 0;
    for(unsigned int i = 0; i < p; ++i)
    {
      // copy [C -C] part
      for(unsigned int j = 0; j < n; ++j)
      {
        A_lp[i][j] = C[i][j];
        A_lp[i][n+j] = -C[i][j];
      }
      // y-part
      A_lp[i][2*n+i] = 1;
      if(e[i] < -tol)
      {
        // r-part
        A_lp[i][2*n+p+l] = -1;
        xc[2*n+p+l] = -e[i];
        l++;
      }
      else
        xc[2*n+i] = e[i];
    }
    vpLinProg::simplex(c, A_lp, e, xc);

    // r-part should be 0
    if(!vpLinProg::allLesser(xc.extract(2*n+p, k), tol))
    {
      std::cout << "vpQuadProg::solveQPi: inequality constraints not feasible" << std::endl;
      return false;
    }

    // update x to feasible point
    x += xc.extract(0,n) - xc.extract(n,n);
    // init active/inactive sets from y-part of x
    active.clear();
    active.reserve(p);
    inactive.clear();
    for(unsigned int i = 0; i < p; ++i)
    {
      if(C.getRow(i)*x - d[i] < -tol)
        inactive.push_back(i);
      else
        active.push_back(i);
    }
  }
  else  // warm start feasible
  {
    // using previous active set, check that inactive is sync
    if(active.size() + inactive.size() != p)
    {
      inactive.clear();
      for(unsigned int i = 0; i < p; ++i)
      {
        if(std::find(active.begin(), active.end(), i) == active.end())
          inactive.push_back(i);
      }
    }
  }

  vpMatrix Ap;
  bool update_Ap = true;
  unsigned int last_active = C.getRows();

  vpColVector u, g = r - Q*x, mu;

  // solve at one iteration
  while (true)
  {
    A.resize((unsigned int)active.size(), n);
    b.resize((unsigned int)active.size());
    for(unsigned int i = 0; i < active.size(); ++i)
    {
      for(unsigned int j = 0; j < n; ++j)
        A[i][j] = C[active[i]][j];
    }

    if(update_Ap && active.size())
      Ap = A.pseudoInverse(); // to get Lagrange multipliers if needed

    if(!solveByProjection(Q, g, A, b, u, tol))
    {
      std::cout << "vpQuadProg::solveQPi: QP seems infeasible, too many constraints activated\n";
      return false;
    }

    // 0-update = optimal or useless activated constraints
    if(vpLinProg::allZero(u, tol))
    {
      // compute multipliers if any
      unsigned int ineqInd = (unsigned int)active.size();
      if(active.size())
      {
        mu = -Ap.transpose() * Q.transpose() * (Q*u - g);
        // find most negative one if any - except last activated in case of degeneracy
        double ineqMax = -tol;
        for(unsigned int i = 0; i < mu.getRows(); ++i)
        {
          if(mu[i] < ineqMax && active[i] != last_active)
          {
            ineqInd = i;
            ineqMax = mu[i];
          }
        }
      }

      if(ineqInd == active.size())   // KKT condition no useless constraint
        return true;

      // useless inequality, deactivate
      inactive.push_back(active[ineqInd]);
      if(active.size() == 1)
        active.clear();
      else
        active.erase(active.begin()+ineqInd);
      update_Ap = true;
    }
    else    // u != 0, can improve xc
    {
      unsigned int ineqInd = 0;
      // step length to next constraint
      double alpha = 1;
      for(unsigned int i = 0; i < inactive.size(); ++i)
      {
        const double Cu = C.getRow(inactive[i])*u;
        if(Cu > tol)
        {
          const double a = (d[inactive[i]] - C.getRow(inactive[i])*x)/Cu;
          if(a < alpha)
          {
            alpha = a;
            ineqInd = i;
          }
        }
      }
      if(alpha < 1)
      {
        last_active = inactive[ineqInd];
        if(active.size())
        {
          auto it = active.begin();
          while(it != active.end() && *it < inactive[ineqInd])
            it++;
          active.insert(it, inactive[ineqInd]);
        }
        else
          active.push_back(inactive[ineqInd]);
        inactive.erase(inactive.begin()+ineqInd);
        update_Ap = true;
      }
      else
        update_Ap = false;
      // update x for next iteration
      x += alpha * u;
      g -= alpha*Q*u;
    }
  }
}
#else
void dummy_vpQuadProg(){};
#endif
