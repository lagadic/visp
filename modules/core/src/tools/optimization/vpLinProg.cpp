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
 * Linear Programming
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/

#include <visp3/core/vpLinProg.h>

/*!
  Reduces the search space induced by an equality constraint.

  Changes A and b so that the constraint \f$\mathbf{A}\mathbf{x} = \mathbf{b}\f$
  can be written \f$\exists\mathbf{z}, \mathbf{x} = \mathbf{b} + \mathbf{A}\mathbf{z}\f$

  This method is destructive for A and b.

  \param A : in = equality matrix (dimension m x n), out = projector to kernel (dimension n x (n-rank(A)))
  \param b : in = equality vector (dimension m), out = particular solution (dimension n)
  \param full_rank : if we think A is full rank, leads to a faster result but a slower one if A is actually not full rank.
  \param tol : tolerance to test the ranks

  \return True if \f$\mathbf{A}\mathbf{x} = \mathbf{b}\f$ has a solution.

  Here is an example with \f$\mathbf{A} =
\left[\begin{array}{cccc}1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 1 & 1& 0 & 0\end{array}\right]\f$
  and \f$\mathbf{b} = \left[\begin{array}{c}1\\2\\3\end{array}\right]\f$ that become \f$\mathbf{A} =
\left[\begin{array}{cc}0 & 0\\ 0 & 0\\ 1& 0\\ 0& 1\end{array}\right]\f$ and
 \f$\mathbf{b} = \left[\begin{array}{c}1\\2\\0\\0\end{array}\right]\f$.

 We indeed have \f$\forall \mathbf{z}\in\mathbb{R}^2, \left[\begin{array}{cccc}1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 1 & 1& 0 & 0\end{array}\right]
 \left(\left[\begin{array}{c}1\\2\\0\\0\end{array}\right] +
\left[\begin{array}{cc}0 & 0\\ 0 & 0\\ 1& 0\\ 0& 1\end{array}\right]\mathbf{z}\right) = \left[\begin{array}{c}1\\2\\3\end{array}\right]\f$

  \code
  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix A(3,4);
    vpColVector b(3);
    A[0][0] = A[1][1] = 1;
    A[2][0] = A[2][1] = 1;
    b[0] = 1; b[1] = 2; b[2] = 3;

    vpMatrix Z = A;
    vpColVector x1 = b;

    if(vpLinProg::colReduction(Z, x1))
    {
      // Z is now 4 x 2
      // try with random z-vector
      vpColVector z(2);
      for(int i = 0; i < 10; ++i)
      {
        z[0] = (10.*rand())/RAND_MAX;
        z[1] = (10.*rand())/RAND_MAX;
        std::cout << "A.(x1 + Z.z): " << (A*(x1 + Z*z)).t() << std::endl;
      }
    }
    else
      std::cout << "Ax = b not feasible\n";
  }
  \endcode
*/
bool vpLinProg::colReduction(vpMatrix &A, vpColVector &b, bool full_rank, const double &tol)
{
  const unsigned int m = A.getRows();
  const unsigned int n = A.getCols();

  // degeneracy if A is actually null
  if(A.infinityNorm() < tol)
  {
    if(b.infinityNorm() < tol)
    {
      b.resize(n);
      A.eye(n);
      return true;
    }
    else
      return false;
  }

  // try with standard QR
  vpMatrix Q, R;
  unsigned int r;

  if(full_rank) // caller thinks rank is n or m, can use basic QR
  {
    r = A.transpose().qr(Q, R, false, true);
    // degenerate but easy case - rank is number of columns
    if(r == n)
    {
      // full rank a priori not feasible (A is high thin matrix)
      const vpColVector x = Q * R.inverseTriangular().t() * b.extract(0, r);
      if(allClose(A, x, b, tol))
      {
        b = x;
        A.resize(n,0);
        return true;
      }
      return false;
    }
    else if (r == m)  // most common use case - rank is number of rows
    {
      b = Q * R.inverseTriangular().t() * b;
      // build projection to kernel of Q^T, pick n-m independent columns of I - Q.Q^T
      vpMatrix IQQt = -Q*Q.t();
      for(unsigned int j = 0; j < n; ++j)
        IQQt[j][j] += 1;
      // most of the time the first n-m columns are just fine
      A = IQQt.extract(0,0,n,n-m);
      if(A.qr(Q, R, false, false, tol) != n-m)
      {
        // rank deficiency, manually find n-m independent columns
        unsigned int j0;
        for(j0 = 0; j0 < n; ++j0)
        {
          if(!allZero(IQQt.getCol(j0)))
          {
            A = IQQt.getCol(j0);
            break;
          }
        }
        // fill up
        unsigned int j = j0+1;
        while(A.getCols() < n-m)
        {
          // add next column and check rank of A^T.A
          if(!allZero(IQQt.getCol(j)))
          {
            A = vpMatrix::juxtaposeMatrices(A, IQQt.getCol(j));
            if(A.qr(Q, R, false, false, tol) != A.getCols())
              A.resize(n,A.getCols()-1, false);
          }
          j++;
        }
      }
      return true;
    }
  }

  // A may be non full rank, go for QR+Pivot
  vpMatrix P;
  r = A.transpose().qrPivot(Q, R, P, false, true);
  Q = Q.extract(0,0,n,r);
  const vpColVector x = Q
      * R.inverseTriangular().t()
      * P * b;
  if(allClose(A, x, b, tol))
  {
    b = x;
    if(r == n)  // no dimension left
    {
      A.resize(n,0);
      return true;
    }
    // build projection to kernel of Q, pick n-r independent columns of I - Q.Q^T
    vpMatrix IQQt = -Q*Q.t();
    for(unsigned int j = 0; j < n; ++j)
      IQQt[j][j] += 1;
    // most of the time the first n-r columns are just fine
    A = IQQt.extract(0,0,n,n-r);
    if(A.qr(Q, R, false, false, tol) != n-r)
    {
      // rank deficiency, manually find n-r independent columns
      unsigned int j0;
      for(j0 = 0; j0 < n; ++j0)
      {
        if(!allZero(IQQt.getCol(j0)))
        {
          A = IQQt.getCol(j0);
          break;
        }
      }
      // fill up
      unsigned int j = j0+1;
      while(A.getCols() < n-r)
      {
        // add next column and check rank of A^T.A
        if(!allZero(IQQt.getCol(j)))
        {
          A = vpMatrix::juxtaposeMatrices(A, IQQt.getCol(j));
          if(A.qr(Q, R, false, false, tol) != A.getCols())
            A.resize(n,A.getCols()-1, false);
        }
        j++;
      }
    }
    return true;
  }
  return false;
}

/*!
  Reduces the number of equality constraints.

  Changes A and b so that the constraint \f$\mathbf{A}\mathbf{x} = \mathbf{b}\f$
  is written with minimal rows.

  This method is destructive for A and b.

  \param A : equality matrix (dimension in = (m x n), out = (rank(A) x n)
  \param b : equality vector (dimension in = (m), out = (rank(A)))
  \param tol : tolerance to test the ranks

  \return True if \f$\mathbf{A}\mathbf{x} = \mathbf{b}\f$ has a solution.

  Here is an example with \f$\mathbf{A} =
\left[\begin{array}{cccc}1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 1 & 1& 0 & 0\end{array}\right]\f$
  and \f$\mathbf{b} = \left[\begin{array}{c}0\\0\\0\end{array}\right]\f$ (feasible)
or \f$\mathbf{b} = \left[\begin{array}{c}0\\0\\1\end{array}\right]\f$ (not feasible).
  \code
  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpMatrix A(3,4);
    vpColVector b(3);

    A[0][0] = A[1][1] = 1;
    A[2][0] = A[2][1] = 1;

    // b[2] = 1;    // uncomment to make it unfeasible

    if(vpLinProg::rowReduction(A, b))
      std::cout << A << std::endl; // A is now 2x4
    else
      std::cout << "Ax = b not feasible\n";
  }
  \endcode
*/
bool vpLinProg::rowReduction(vpMatrix &A, vpColVector &b, const double &tol)
{
  const unsigned int m = A.getRows();
  const unsigned int n = A.getCols();

  vpMatrix Q, R, P;
  const unsigned int r = A.qrPivot(Q, R, P, false, false, tol);
  const vpColVector x = P.transpose() *
      vpMatrix::stack(R.extract(0, 0, r, r).inverseTriangular(), vpMatrix(n-r, r))
      * Q.extract(0, 0, m, r).transpose() * b;

  if(allClose(A, x, b, tol))
  {
    if(r < m) // if r == m then (A,b) is not changed
    {
      A = R.extract(0, 0, r, n)*P;
      b = Q.extract(0, 0, m, r).transpose() * b;
    }
    return true;
  }
  return false;
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
/*!
  Solves a Linear Program under various constraints

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & \mathbf{c}^T\mathbf{x}\\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b}\\
               & \text{s.t.}& \mathbf{C}\mathbf{x} \leq \mathbf{d}\\
               & \text{s.t.}& \mathbf{x}_i \geq \mathbf{l}_i \text{~for some i}\\
               & \text{s.t.}& \mathbf{x}_j \leq \mathbf{u}_j \text{~for some j}
\end{array}
\f$
  \param c : cost vector (dimension n)
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param C : inequality matrix (dimension p x n)
  \param d : inequality vector (dimension p)
  \param x : in: feasible point if any, out: solution (dimension n)
  \param l : lower bounds (if any)
  \param u : upper bounds (if any)
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  Lower and upper bounds may be passed as a list of (index, bound) with C++11's braced initialization.

  \warning This function is only available if C++11 is activated during compilation. Configure ViSP using cmake -DUSE_CPP11=ON.

  Here is an example:

  \f$\begin{array}{lll}
  (x,y,z) = &  \arg\min & -2x -3y -4z\\
               & \text{s.t.}& 3x + 2y + z \leq 10\\
               & \text{s.t.}& 2x + 5y + 3z \leq 15\\
                & \text{s.t.}& x, y, z \geq 0\\
                & \text{s.t.}& z \leq 6\end{array}\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpColVector c(3), x;
    vpMatrix C(2, 3);
    vpColVector d(2);
    c[0] = -2; c[1] = -3; c[2] = -4;
    C[0][0] = 3;    C[0][1] = 2; C[0][2] = 1; d[0] = 10;
    C[1][0] = 2; C[1][1] = 5; C[1][2] = 3;  d[1] = 15;

    if(vpLinProg::solveLP(c, vpMatrix(0,0), vpColVector(0), C, d, x,
                          {{0,0},{1,0},{2,0}},
                          {{2,6}}))
    {
        std::cout << "x: " << x.t() << std::endl;
        std::cout << "cost: " << c.t()*x << std::endl;
    }
  }
  \endcode

  \sa BoundedIndex
*/
bool vpLinProg::solveLP(const vpColVector &c, vpMatrix A, vpColVector b,
                        const vpMatrix &C, const vpColVector &d, vpColVector &x,
                        std::vector<BoundedIndex> l, std::vector<BoundedIndex> u,
                        const double &tol)
{
  const unsigned int n = c.getRows();
  const unsigned int m = A.getRows();
  const unsigned int p = C.getRows();

  // check if we should forward a feasible point to the next solver
  const bool feasible =
      (x.getRows() == c.getRows())
      && (A.getRows() == 0 || allClose(A, x, b, tol))
      && (C.getRows() == 0 || allLesser(C, x, d, tol))
      && (find_if(l.begin(), l.end(),
                  [&](BoundedIndex &i){return x[i.first] < i.second-tol;})
      == l.end())
      && (find_if(u.begin(), u.end(),
                  [&](BoundedIndex &i){return x[i.first] > i.second+tol;})
      == u.end());

  // shortcut for unbounded variables with equality
  if(!feasible && m && l.size() == 0 && u.size() == 0)
  {
    // changes A.x = b to x = b + A.z
    if(colReduction(A, b, false, tol))
    {
      if(A.getCols())
      {
        if(solveLP(A.transpose()*c, vpMatrix(0, n), vpColVector(0), C*A, d - C*b, x, {}, {}, tol))
        {
          x = b + A*x;
          return true;
        }
      }
      else if(C.getRows() && allLesser(C, b, d, tol))
      {   // A.x = b has only 1 solution (the new b) and C.b <= d
        x = b;
        return true;
      }
    }
    std::cout << "vpLinProg::simplex: equality constraints not feasible" << std::endl;
    return false;
  }

  // count how many additional variables are needed to deal with bounds
  unsigned int s1 = 0, s2 = 0;
  for(unsigned int i = 0; i < n; ++i)
  {
    const auto cmp = [&](const BoundedIndex &p){return p.first == i;};
    // look for lower bound
    const bool has_low = find_if(l.begin(), l.end(), cmp) != l.end();
    // look for upper bound
    const bool has_up = find_if(u.begin(), u.end(), cmp) != u.end();
    if(has_low == has_up)
    {
      s1++;       // additional variable (double-bounded or unbounded variable)
      if(has_low)
        s2++;   // additional equality constraint (double-bounded)
    }
  }

  // build equality matrix with slack variables
  A.resize(m+p+s2, n+p+s1, false);
  b.resize(A.getRows(),false);
  if(feasible)
    x.resize(n+p+s1, false);

  // deal with slack variables for inequality
  // Cx <= d <=> Cx + y = d
  for(unsigned int i = 0; i < p; ++i)
  {
    A[m+i][n+i] = 1;
    b[m+i] = d[i];
    for(unsigned int j = 0; j < n; ++j)
      A[m+i][j] = C[i][j];
    if(feasible)
      x[n+i] = d[i] - C.getRow(i)*x.extract(0,n);
  }
  // x = P.(z - z0)
  vpMatrix P;P.eye(n, n+p+s1);
  vpColVector z0(n+p+s1);

  // slack variables for bounded terms
  // base slack variable is z1 (replaces x)
  // additional is z2
  unsigned int k1 = 0, k2 = 0;
  for(unsigned int i = 0; i < n; ++i)
  {
    // lambda to find a bound for this index
    const auto cmp = [&](const BoundedIndex &p)
    {return p.first == i;};

    // look for lower bound
    const auto low = find_if(l.begin(), l.end(), cmp);
    // look for upper bound
    const auto up = find_if(u.begin(), u.end(), cmp);

    if(low == l.end())  // no lower bound
    {
      if(up == u.end())   // no bounds, x = z1 - z2
      {
        P[i][n+p+k1] = -1;
        for(unsigned int j = 0; j < m+p; ++j)
          A[j][n+p+k1] = -A[j][i];
        if(feasible)
        {
          x[i] = std::max(x[i], 0.);
          x[n+p+k1] = std::max(-x[i], 0.);
        }
        k1++;
      }
      else                // upper bound x <= u <=> z1 = -x + u >= 0
      {
        z0[i] = up->second;
        P[i][i] = -1;
        for(unsigned int j = 0; j < m+p; ++j)
          A[j][i] *= -1;
        if(feasible)
          x[i] = up->second - x[i];
        u.erase(up);
      }
    }
    else                    // lower bound  x >= l <=> z1 = x - l >= 0
    {
      z0[i] = -low->second;
      if(up != u.end())   // both bounds  z1 + z2 = u - l
      {
        A[m+p+k2][i] = A[m+p+k2][n+p+k1] = 1;
        b[m+p+k2] = up->second - low->second;
        if(feasible)
        {
          x[i] = up->second - x[i];
          x[n+p+k1] = x[i] - low->second;
        }
        k1++;
        k2++;
        u.erase(up);
      }
      else if(feasible)             // only lower bound
        x[i] = x[i] - low->second;
      l.erase(low);
    }
  }

  // x = P.(z-z0)
  // c^T.x = (P^T.c)^T.z
  // A.x - b = A.P.Z - (b + A.P.z0)
  // A is already A.P
  if(vpLinProg::simplex(P.transpose()*c, A, b+A*z0, x, tol))
  {
    x = P*(x - z0);
    return true;
  }
  return false;
}

/*!
  Solves a Linear Program under simplex canonical form

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & \mathbf{c}^T\mathbf{x}\\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b}\\
                & \text{s.t.}& \mathbf{x} \geq 0
\end{array}
\f$
  \param c : cost vector (dimension n)
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param x : in: feasible point if any, out: solution (dimension n)
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  \warning This function is only available if C++11 is activated during compilation. Configure ViSP using cmake -DUSE_CPP11=ON.

  Here is an example:

  \f$\begin{array}{lll}
  (x,y,z) = &  \arg\min & -2x -3y -4z\\
               & \text{s.t.}& 3x + 2y + z \leq 10\\
               & \text{s.t.}& 2x + 5y + 3z \leq 15\\
                & \text{s.t.}& x, y, z \geq 0\end{array}\f$

  That can be re-written as:

  \f$\begin{array}{lll}
  (x,y,z), (s_1, s_2) = &  \arg\min & -2x -3y -4z\\
               & \text{s.t.}& 3x + 2y + z + s_1 = 10\\
               & \text{s.t.}& 2x + 5y + 3z + s_2 = 15\\
                & \text{s.t.}& x, y, z, s_1, s_2 \geq 0\end{array}\f$
  \code

  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpColVector c(5), x;
    vpMatrix A(2, 5);
    vpColVector b(2);
    c[0] = -2; c[1] = -3; c[2] = -4;
    A[0][0] = 3; A[0][1] = 2; A[0][2] = 1;  b[0] = 10;
    A[1][0] = 2; A[1][1] = 5; A[1][2] = 3;  b[1] = 15;
    A[0][3] = A[1][4] = 1;

    if(vpLinProg::simplex(c, A, b, x))
    {
        std::cout << "x: " << x.t().extract(0,3) << std::endl;
        std::cout << "cost: " << c.t()*x << std::endl;
    }
  }
  \endcode

  \sa solveLP
*/
bool vpLinProg::simplex(const vpColVector &c, vpMatrix A, vpColVector b, vpColVector &x, const double &tol)
{
  const unsigned int n = c.getRows();
  unsigned int m = A.getRows();

  // find a feasible point is passed x is not
  if(    (x.getRows() != c.getRows()) ||
         !allGreater(x, -tol) ||
         (m != 0 && !allClose(A, x, b, tol)) ||
         [&x,&n](){unsigned int non0 = 0; // count non-0 in x, feasible if <= m
         for(unsigned int i = 0; i < n; ++i)
         if(x[i] > 0) non0++;
         return non0;}() > m
         )
  {
    // min sum(z)
    //  st A.x + D.z =  with diag(D) = sign(b)
    // feasible = (0, |b|)
    // z should be minimized to 0 to get a feasible point for this simplex
    vpColVector cz(n+m);
    vpMatrix AD(m,n+m);
    x.resize(n+m);
    for(unsigned int i = 0; i < m; ++i)
    {
      // build AD and x
      if(b[i] > -tol)
      {
        AD[i][n+i] = 1;
        x[n+i] = b[i];
      }
      else
      {
        AD[i][n+i] = -1;
        x[n+i] = -b[i];
      }
      cz[n+i] = 1;

      // copy A
      for(unsigned int j = 0; j < n; ++j)
        AD[i][j] = A[i][j];
    }
    // solve to get feasibility
    vpLinProg::simplex(cz, AD, b, x, tol);
    if(!allLesser(x.extract(n,m), tol))    // no feasible starting point found
    {
      std::cout << "vpLinProg::simplex: constraints not feasible" << std::endl;
      x.resize(n);
      return false;
    }
    // extract feasible x
    x = x.extract(0, n);
  }
  // feasible part  x is >= 0 and Ax = b

  // try to reduce number of rows to remove rank deficiency
  if(!rowReduction(A, b, tol))
  {
    std::cout << "vpLinProg::simplex: equality constraint not feasible" << std::endl;
    return false;
  }
  m = A.getRows();
  if(m == 0)
  {
    // no constraints, solution is either unbounded or 0
    x = 0;
    return true;
  }

  // build base index from feasible point
  vpMatrix Ab(m,m), An(m,n-m), Abi;
  vpColVector cb(m), cn(n-m);
  std::vector<unsigned int> B, N;
  // add all non-0 indices to B
  for(unsigned int i = 0; i < n; ++i)
  {
    if(std::abs(x[i]) > tol)
      B.push_back(i);
  }
  // if B not full then try to get Ab without null rows
  // get null rows of current Ab = A[B]
  std::vector<unsigned int> null_rows;
  for(unsigned int i = 0; i < m; ++i)
  {
    bool is_0 = true;
    for(unsigned int j = 0; j < B.size(); ++j)
    {
      if(std::abs(A[i][B[j]]) > tol)
      {
        is_0 = false;
        break;
      }
    }
    if(is_0)
      null_rows.push_back(i);
  }

  // add columns to B only if make Ab non-null
  // start from the end as it makes vpQuadProg faster
  for(unsigned int j = n; j-- > 0; )
  {
    if(std::abs(x[j]) < tol)
    {
      bool in_N = true;
      if(B.size() != m)
      {
        in_N = false;
        for(unsigned int i = 0; i < null_rows.size(); ++i)
        {
          in_N = true;
          if(std::abs(A[null_rows[i]][j]) > tol)
          {
            null_rows.erase(null_rows.begin()+i);
            in_N = false;
            break;
          }
        }
        // update empty for next time
        if(!in_N && null_rows.size())
        {
          bool redo = true;
          while(redo)
          {
            redo = false;
            for(unsigned int i = 0; i < null_rows.size(); ++i)
            {
              if(std::abs(A[null_rows[i]][j]) > tol)
              {
                redo = true;
                null_rows.erase(null_rows.begin()+i);
                break;
              }
            }
          }
        }
      }
      if(in_N)
        N.push_back(j);
      else
        B.push_back(j);
    }
  }
  // split A into (Ab, An) and c into (cb, cn)
  for(unsigned int j = 0; j < m; ++j)
  {
    cb[j] = c[B[j]];
    for(unsigned int i = 0; i < m; ++i)
      Ab[i][j] = A[i][B[j]];
  }
  for(unsigned int j = 0; j < n-m; ++j)
  {
    cn[j] = c[N[j]];
    for(unsigned int i = 0; i < m; ++i)
      An[i][j] = A[i][N[j]];
  }

  std::vector<std::pair<double, unsigned int>> a;
  a.reserve(n);

  vpColVector R(n-m), db(m);

  while(true)
  {
    Abi = Ab.inverseByQR();
    // find negative residual
    R  = cn - An.t()*Abi.t()*cb;
    unsigned int j;
    for(j = 0; j < N.size(); ++j)
    {
      if(R[j] < -tol)
        break;
    }

    // no negative residual -> optimal point
    if(j == N.size())
      return true;

    // j will be added as a constraint, find the one to remove
    db = -Abi * An.getCol(j);

    if(allGreater(db, -tol))  // unbounded
      return true;

    // compute alpha / step length to next constraint
    a.resize(0);
    for(unsigned int k = 0; k < m; ++k)
    {
      if(db[k] < -tol)
        a.push_back({-x[B[k]]/db[k], k});
    }
    // get smallest index for smallest alpha
    const auto amin = std::min_element(a.begin(), a.end());
    const double alpha = amin->first;
    const unsigned int k = amin->second;

    // pivot between B[k] and N[j]
    // update x
    for(unsigned int i = 0; i < B.size(); ++i)
      x[B[i]] += alpha * db[i];
    // N part of x
    x[N[j]] = alpha;

    // update A and c
    std::swap(cb[k], cn[j]);
    for(unsigned int i = 0; i < m; ++i)
      std::swap(Ab[i][k], An[i][j]);

    // update B and N
    std::swap(B[k],N[j]);
  }
}
#endif
