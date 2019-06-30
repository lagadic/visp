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
 * REPPnP / EPPnP pose computation methods.
 *
 *****************************************************************************/
/*
% Copyright (C) <2014>  <Luis Ferraz, Xavier Binefa, Francesc Moreno-Noguer>
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the version 3 of the GNU General Public License
% as published by the Free Software Foundation.
%
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <visp3/vision/vpPose.h>

namespace
{
static vpMatrix defineControlPoints()
{
  vpMatrix M(4, 3, 0.0);
  for (unsigned int i = 0; i < 3; i++) {
    M[i][i] = 1.0;
  }

  return M;
}

static vpMatrix computeM(const vpMatrix& U, const vpMatrix& Alpha)
{
  vpMatrix m(2, 3, { 1, 0, -1, 0, 1, -1 });
  vpMatrix M;
  vpMatrix::kron(Alpha, m, M);

  vpMatrix tmp(M.getRows(), 4);
  for (unsigned int i = 0; i < M.getRows(); i++) {
    tmp[i][0] = M[i][2];
    tmp[i][1] = M[i][5];
    tmp[i][2] = M[i][8];
    tmp[i][3] = M[i][11];
  }

  vpMatrix tmp2 = tmp.hadamard(U * vpMatrix(1, 4, 1.0));

  for (unsigned int i = 0; i < M.getRows(); i++) {
    M[i][2] = tmp2[i][0];
    M[i][5] = tmp2[i][1];
    M[i][8] = tmp2[i][2];
    M[i][11] = tmp2[i][3];
  }

  return M;
}

static vpMatrix computeAlphas(const vpMatrix& Xw, const vpMatrix& Cw)
{
  unsigned int nbPts = Xw.getRows();

  //generate auxiliar matrix to compute alphas
  vpMatrix C(4, 4, 1.0);
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      C[i][j] = Cw[j][i];
    }
  }

  vpMatrix X(4, nbPts, 1.0);
  vpMatrix Xwt = Xw.t();
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < nbPts; j++) {
      X[i][j] = Xwt[i][j];
    }
  }

  vpMatrix Alpha = C.inverseByLU() * X;

  return Alpha.t();
}

static vpMatrix reshape1DMatlab(const vpMatrix& src)
{
  vpMatrix dst(src.getRows()*src.getCols(), 1);
  for (unsigned int j = 0; j < src.getCols(); j++) {
    for (unsigned int i = 0; i < src.getRows(); i++) {
      dst[j*src.getRows() + i][0] = src[i][j];
    }
  }

  return dst;
}

static void prepareData(const std::vector<vpPoint>& points, vpMatrix& M, vpMatrix& Cw, vpMatrix& Alpha)
{
  vpMatrix Cwt = defineControlPoints();
  Cw = Cwt.t();

  unsigned int nbPts = static_cast<unsigned int>(points.size());
  vpMatrix Xw(nbPts, 3);
  vpMatrix U(2, nbPts);

  for (unsigned int i = 0; i < nbPts; i++) {
    const vpPoint& pt = points[i];
    Xw[i][0] = pt.get_oX();
    Xw[i][1] = pt.get_oY();
    Xw[i][2] = pt.get_oZ();

    U[0][i] = pt.get_x();
    U[1][i] = pt.get_y();
  }

  Alpha = computeAlphas(Xw, Cwt);
  M = computeM(reshape1DMatlab(U), Alpha);
}

static vpMatrix kernelNoise(const vpMatrix& M, int dimKer)
{
  vpMatrix MtM = M.AtA();
  vpColVector W;
  vpMatrix eigenVectors;
  MtM.svd(W, eigenVectors);

  const unsigned int sz0 = eigenVectors.getRows(), sz1 = eigenVectors.getCols();
  vpMatrix K(sz0, static_cast<unsigned int>(dimKer));
  for (unsigned int i = 0, i2 = 0; i < sz0; i++, i2++) {
    for (unsigned int j = sz1-static_cast<unsigned int>(dimKer), j2 = 0; j < sz1; j++, j2++) {
      K[i2][j2] = eigenVectors[i][j];
    }
  }

  return K;
}

static vpMatrix reshapeMatlab(const vpMatrix& V, unsigned int rows, unsigned int cols)
{
  if (V.getCols() > 1) {
    throw vpException(vpException::dimensionError, "V must be a column vector!");
  }
  vpMatrix Mreshape(rows, cols);
  for (unsigned int j = 0; j < cols; j++)
  {
    for (unsigned int i = 0; i < rows; i++)
    {
      Mreshape[i][j] = V[j*rows + i][0];
    }
  }

  return Mreshape;
}

struct Procrustes
{
  vpMatrix P;
  vpMatrix mP;
  vpMatrix cP;
  double norm;
  vpMatrix nP;
};

static vpMatrix reduceRowAvg(const vpMatrix& M)
{
  vpMatrix avg;

  if (M.getRows()*M.getCols() > 0) {
    avg.resize(M.getRows(), 1);

    for (unsigned int i = 0; i < M.getRows(); i++) {
      for (unsigned int j = 0; j < M.getCols(); j++) {
        avg[i][0] += M[i][j];
      }
      avg[i][0] /= M.getCols();
    }
  }

  return  avg;
}

static vpMatrix reduceColSum(const vpMatrix& M)
{
  vpMatrix sum;

  if (M.getRows()*M.getCols() > 0) {
    sum.resize(M.getCols(), 1);

    for (unsigned int j = 0; j < M.getCols(); j++) {
      for (unsigned int i = 0; i < M.getRows(); i++) {
        sum[j][0] += M[i][j];
      }
    }
  }

  return  sum;
}

static std::vector<unsigned int> findIdxZero(const vpMatrix& M)
{
  std::vector<unsigned int> idx;

  for (unsigned int i = 0; i < M.getRows(); i++) {
    if (vpMath::nul(M[i][0], 1e-9)) {
      idx.push_back(i);
    }
  }

  return idx;
}

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

static void EPPnPProcrustes(const Procrustes& X, const vpMatrix& Y, vpMatrix& R, double& b, vpMatrix& mc)
{
  const unsigned int dims = Y.getCols();
  vpMatrix mY = reduceRowAvg(Y);
  vpMatrix cY = Y - mY*vpMatrix(1, dims, 1.0);
  double ncY = sqrt(cY.sumSquare());
  vpMatrix tcY = cY / ncY;

  vpMatrix A = X.nP * tcY.t();
  vpColVector w;
  vpMatrix M;
  A.svd(w, M);

  vpColVector vec_diag(3, 1.0);
  vec_diag[2] = sgn((M*A.t()).det());
  vpMatrix M_diag;
  M_diag.diag(vec_diag);
  R = M * M_diag * A.t();

  b = (w.sum() * X.norm/ncY);
  vpMatrix c = X.mP - b*R.t()*mY;
  mc = c*vpMatrix(1, dims, 1.0);
}

static void KernelPnP(const vpMatrix& Cw, const vpMatrix& Km, unsigned int dims, vpMatrix& R, vpMatrix& T, double& err, bool solIter=true)
{
  vpMatrix Km_lastCol(Km.getRows(), 1);
  for (unsigned int i = 0; i < Km.getRows(); i++) {
    Km_lastCol[i][0] = Km[i][Km.getCols()-1];
  }
  vpMatrix vK = reshapeMatlab(Km_lastCol, 3, dims);

  Procrustes X;
  X.P = Cw;
  X.mP = reduceRowAvg(X.P);

  vpMatrix ones(1, dims, 1.0);
  X.cP = X.P - X.mP*ones;

  vpColVector cP = X.cP.stackColumns();
  X.norm = sqrt(cP.sumSquare());

  double inv_norm = 1/X.norm;
  X.nP = X.cP*inv_norm;

  double mean_vK_lastRows = vK.getRow(vK.getRows()-1).sum() / vK.getCols();
  if (mean_vK_lastRows < 0) {
    vK = -vK;
  }

  vpMatrix mc;
  double b;
  EPPnPProcrustes(X, vK, R, b, mc);

  vpMatrix solV = b*vK;
  vpMatrix solR = R;
  vpMatrix solmc = mc;

  if (solIter) {
    err = std::numeric_limits<double>::max();
    const int nIter = 10;

    // procrustes solution using 4 kernel eigenvectors
    for (int iter = 0; iter < nIter; iter++)
    {
      // projection of previous solution into the null space
      vpMatrix A = R * (-mc + X.P);
      vpMatrix abcd = Km.pseudoInverse() * reshape1DMatlab(A);
      vpMatrix newV = reshapeMatlab(Km*abcd, 3, dims);

      // euclidean error
      vpColVector w;
      vpMatrix V;
      (R.t()*newV + mc-X.P).svd(w, V);
      double newErr = w[0];

      if (newErr > err && iter >= 2) {
        break;
      } else {
        // procrustes solution
        EPPnPProcrustes(X, newV, R, b, mc);

        solV = b * newV;
        solmc = mc;
        solR = R;
        err = newErr;
      }
    }
  }

  R = solR;
  vpMatrix mV = reduceRowAvg(solV);
  T = mV - R*X.mP;
}

static std::vector<unsigned int> setDiff(unsigned int srcSize, const std::vector<unsigned int>& removeIdx)
{
  std::vector<unsigned int> srcIdx(srcSize);
  for (unsigned int i = 0; i < srcSize; i++) {
    srcIdx[i] = i;
  }

  std::vector<unsigned int> diffIdx;
  std::set_difference(srcIdx.begin(), srcIdx.end(),
                      removeIdx.begin(), removeIdx.end(),
                      std::back_inserter(diffIdx));
  return diffIdx;
}

static vpMatrix copyCols(const vpMatrix& M, const std::vector<unsigned int>& colIdx)
{
  vpMatrix M_out(M.getRows(), static_cast<unsigned int>(colIdx.size()));

  for (size_t idx = 0; idx < colIdx.size(); idx++) {
    for (unsigned int i = 0; i < M.getRows(); i++) {
      M_out[i][idx] = M[i][colIdx[idx]];
    }
  }

  return  M_out;
}

static vpMatrix copyRows(const vpMatrix& M, const std::vector<unsigned int>& rowIdx)
{
  vpMatrix M_out(static_cast<unsigned int>(rowIdx.size()), M.getCols());

  for (unsigned int idx = 0; idx < static_cast<unsigned int>(rowIdx.size()); idx++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_out[idx][j] = M[rowIdx[idx]][j];
    }
  }

  return  M_out;
}

static vpMatrix squareRoot(const vpMatrix& M)
{
  vpMatrix M_out(M.getRows(), M.getCols());

  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_out[i][j] = sqrt(M[i][j]);
    }
  }

  return M_out;
}

static void sort(const vpColVector& V, vpColVector& Vsorted, std::vector<unsigned int>& idx)
{
  std::vector<std::pair<double, unsigned int> > pairs;
  pairs.reserve(V.getRows());
  for (unsigned int i = 0; i < V.getRows(); i++) {
    pairs.push_back(std::pair<double, unsigned int>(V[i], i));
  }
  std::sort(pairs.begin(), pairs.end());

  Vsorted.resize(V.getRows(), false);
  idx.resize(pairs.size());
  for (unsigned int i = 0; i < static_cast<unsigned int>(pairs.size()); i++) {
    Vsorted[i] = pairs[i].first;
    idx[i] = pairs[i].second;
  }
}

static int countInliers(const vpColVector& V, double med, double minError)
{
  int sum = 0;
  const double threshold = std::max(med, minError);

  for (unsigned int i = 0; i < V.getRows(); i++) {
    if (V[i] < threshold) {
      sum++;
    } else {
      break;
    }
  }

  return sum;
}

static std::vector<unsigned int> getNewInliersIdx(const std::vector<unsigned int>& idx)
{
  std::vector<unsigned int> newIdx(2*idx.size());

  for (size_t i = 0; i < idx.size(); i++) {
    newIdx[2*i] = 2*idx[i];
    newIdx[2*i+1] = 2*idx[i]+1;
  }

  return  newIdx;
}

static vpMatrix robustKernelNoise(const vpMatrix& M, unsigned int dimKer, double minError, std::vector<unsigned int>& idxInliers)
{
  const unsigned int m = M.getRows();
  std::vector<unsigned int> idx(m);
  for (unsigned int i = 0; i < m; i++) {
    idx[i] = i;
  }

  vpMatrix N = M;
  //SVD
  vpColVector w;
  vpMatrix V, resV;
  vpMatrix M_even(M.getRows()/2, M.getCols()), M_odd(M.getRows()/2, M.getCols());
  for (unsigned int i = 0; i < M_even.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_even[i][j] = M[2*i][j];
      M_odd[i][j] = M[2*i+1][j];
    }
  }

  double prev_sv = std::numeric_limits<double>::max();
  const int maxIters = 50;
  for (int iter = 0; iter < maxIters; iter++) {
    N = copyRows(M, idx);
    N.AtA().svd(w, V);

    vpColVector error10 = M_even * V.getCol(V.getCols()-1);
    vpColVector error11 = M_odd * V.getCol(V.getCols()-1);
    vpColVector error1 = squareRoot(error10.hadamard(error10) + error11.hadamard(error11));

    vpColVector sv;
    std::vector<unsigned int> tidx;
    sort(error1, sv, tidx);

    double med = sv[static_cast<unsigned int>(std::floor(m/8.0)) > 0 ?
          static_cast<unsigned int>(std::floor(m/8.0))-1 :
          static_cast<unsigned int>(std::floor(m/8.0))];

    int nInliers = countInliers(sv, med, minError);

    if (med >= prev_sv) {
      break;
    } else {
      prev_sv = med;
      resV = V;
      idxInliers = std::vector<unsigned int>(tidx.begin(), tidx.begin()+nInliers);
    }

    idx = getNewInliersIdx(idxInliers);
  }

  vpMatrix K(resV.getRows(), dimKer);
  for (unsigned int i = 0; i < resV.getRows(); i++) {
    for (unsigned int j = resV.getCols()-dimKer, j2 = 0; j < resV.getCols(); j++, j2++) {
      K[i][j2] = resV[i][j];
    }
  }

  return K;
}
} //namespace

void vpPose::poseEPPnPNonPlan(vpHomogeneousMatrix& cMo)
{
  vpMatrix M, Cw, Alpha;
  std::vector<vpPoint> vecPoints{std::begin(listP), std::end(listP)};
  prepareData(vecPoints, M, Cw, Alpha);
  const unsigned int dimKer = 4;
  vpMatrix Km = kernelNoise(M, dimKer);

  double err = std::numeric_limits<double>::max();
  vpMatrix R, tvec;
  KernelPnP(Cw, Km, dimKer, R, tvec, err);

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      cMo[i][j] = R[i][j];
    }
    cMo[i][3] = tvec[i][0];
  }
}

void vpPose::poseEPPnPPlan(vpHomogeneousMatrix& cMo)
{
  vpMatrix M, Cw, Alpha;
  std::vector<vpPoint> vecPoints{std::begin(listP), std::end(listP)};
  prepareData(vecPoints, M, Cw, Alpha);

  std::vector<unsigned int> idx = findIdxZero( reduceColSum( reshapeMatlab(reduceColSum(M), 3, 4) ) );

  if (!idx.empty()) {
    std::vector<unsigned int> removeIdx, removeIdxCw;
    for (size_t i = 0; i < idx.size(); i++) {
      removeIdx.push_back(3*idx[i]);
      removeIdx.push_back(3*idx[i]+1);
      removeIdx.push_back(3*idx[i]+2);

      removeIdxCw.push_back(idx[i]);
    }
    std::vector<unsigned int> diffIdx = setDiff(M.getCols(), removeIdx);
    M = copyCols(M, diffIdx);

    diffIdx = setDiff(Cw.getCols(), removeIdxCw);
    Cw = copyCols(Cw, diffIdx);

    Alpha = copyCols(Alpha, diffIdx);
  }

  const unsigned int dimKer = 3;
  vpMatrix Km = kernelNoise(M, dimKer);

  double err = std::numeric_limits<double>::max();
  vpMatrix R, tvec;
  KernelPnP(Cw, Km, dimKer, R, tvec, err);

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      cMo[i][j] = R[i][j];
    }
    cMo[i][3] = tvec[i][0];
  }
}

void vpPose::poseREPPnPNonPlan(vpHomogeneousMatrix& cMo)
{
  vpMatrix M, Cw, Alpha;
  std::vector<vpPoint> vecPoints{std::begin(listP), std::end(listP)};
  prepareData(vecPoints, M, Cw, Alpha);
  const unsigned int dimKer = 4;
  vpMatrix Km = robustKernelNoise(M, dimKer, m_REPPnPMinError, m_REPPnPInlierIndices);

  double err = std::numeric_limits<double>::max();
  vpMatrix R, tvec;
  KernelPnP(Cw, Km, dimKer, R, tvec, err);

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      cMo[i][j] = R[i][j];
    }
    cMo[i][3] = tvec[i][0];
  }
}

void vpPose::poseREPPnPPlan(vpHomogeneousMatrix& cMo)
{
  vpMatrix M, Cw, Alpha;
  std::vector<vpPoint> vecPoints{std::begin(listP), std::end(listP)};
  prepareData(vecPoints, M, Cw, Alpha);

  std::vector<unsigned int> idx = findIdxZero( reduceColSum( reshapeMatlab(reduceColSum(M), 3, 4) ) );

  if (!idx.empty()) {
    std::vector<unsigned int> removeIdx, removeIdxCw;
    for (size_t i = 0; i < idx.size(); i++) {
      removeIdx.push_back(3*idx[i]);
      removeIdx.push_back(3*idx[i]+1);
      removeIdx.push_back(3*idx[i]+2);

      removeIdxCw.push_back(idx[i]);
    }
    std::vector<unsigned int> diffIdx = setDiff(M.getCols(), removeIdx);
    M = copyCols(M, diffIdx);

    diffIdx = setDiff(Cw.getCols(), removeIdxCw);
    Cw = copyCols(Cw, diffIdx);

    Alpha = copyCols(Alpha, diffIdx);
  }

  const unsigned int dimKer = 3;
  vpMatrix Km = robustKernelNoise(M, dimKer, m_REPPnPMinError, m_REPPnPInlierIndices);

  double err = std::numeric_limits<double>::max();
  vpMatrix R, tvec;
  KernelPnP(Cw, Km, dimKer, R, tvec, err);

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      cMo[i][j] = R[i][j];
    }
    cMo[i][3] = tvec[i][0];
  }
}
