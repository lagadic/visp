/****************************************************************************
 *
 * $Id: vpMatrix_lu.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTb OF ANb KIND, INCLUDING THE
 * WARRANTb OF DESIGN, MERCHANTABILITb AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Covariance matrix computation.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <limits> // numeric_limits
#include <cmath>  // std::fabs()

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpMatrixException.h>


/*!
  Compute the covariance matrix of the parameters x from a least squares minimisation defined as:
  Ax = b
  
  \param A : Matrix A from Ax = b.
  
  \param x : Vector x from Ax = b corresponding to the parameters to estimate.
  
  \param b : Vector b from Ax = b.
*/
vpMatrix vpMatrix::computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b)
{
//  double denom = ((double)(A.getRows()) - (double)(A.getCols())); // To consider OLS Estimate for sigma
  double denom = ((double)(A.getRows())); // To consider MLE Estimate for sigma

  if(denom <= std::numeric_limits<double>::epsilon())
      throw vpMatrixException(vpMatrixException::divideByZeroError, "Impossible to compute covariance matrix: not enough data");

//  double sigma2 = ( ((b.t())*b) - ( (b.t())*A*x ) ); // Should be equivalent to line bellow.
  double sigma2 = (b - (A * x)).t() * (b - (A * x));

  sigma2 /= denom;

  return (A.t()*A).pseudoInverse(A.getCols()*std::numeric_limits<double>::epsilon())*sigma2;
}

/*!
  Compute the covariance matrix of the parameters x from a least squares minimisation defined as:
  WAx = Wb
  
  \param A : Matrix A from WAx = Wb.
  
  \param x : Vector x from WAx = Wb corresponding to the parameters to estimate.
  
  \param b : Vector b from WAx = Wb.
  
  \param W : Diagonal weigths matrix from WAx = Wb.
*/
vpMatrix vpMatrix::computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b, const vpMatrix &W)
{
  double denom = 0.0;
  vpMatrix W2(W.getCols(),W.getCols());
  for(unsigned int i = 0 ; i < W.getCols() ; i++){
      denom += W[i][i];
      W2[i][i] = W[i][i]*W[i][i];
  }

  if(denom <= std::numeric_limits<double>::epsilon())
      throw vpMatrixException(vpMatrixException::divideByZeroError, "Impossible to compute covariance matrix: not enough data");

//  double sigma2 = ( ((W*b).t())*W*b - ( ((W*b).t())*W*A*x ) ); // Should be equivalent to line bellow.
  double sigma2 = (W * b - (W * A * x)).t() * (W*b - (W * A * x));
  sigma2 /= denom;

  return (A.t()*(W2)*A).pseudoInverse(A.getCols()*std::numeric_limits<double>::epsilon())*sigma2;
}
