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
 * Luminance dimensionality reduction features
 */

#include <visp3/visual_features/vpFeatureLuminanceMapping.h>
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifdef VISP_HAVE_MODULE_IO
#include <visp3/io/vpImageIo.h>
#endif
BEGIN_VISP_NAMESPACE

// vpLuminanceMapping

void vpLuminanceMapping::imageAsVector(const vpImage<unsigned char> &I, vpColVector &Ivec, unsigned border)
{
  const unsigned h = I.getHeight();
  const unsigned w = I.getWidth();
  if (h < 2 * border || w < 2 * border) {
    throw vpException(vpException::dimensionError, "Image is smaller than required border crop");
  }
  Ivec.resize((h - 2 * border) * (w - 2 * border));
  unsigned l = 0;
  for (unsigned i = border; i < h - border; ++i) {
    for (unsigned j = border; j < w - border; ++j) {
      Ivec[l++] = (double)I[i][j];
    }
  }
}

void vpLuminanceMapping::imageAsMatrix(const vpImage<unsigned char> &I, vpMatrix &Imat, unsigned border)
{
  const unsigned h = I.getHeight();
  const unsigned w = I.getWidth();
  if (h < 2 * border || w < 2 * border) {
    throw vpException(vpException::dimensionError, "Image is smaller than required border crop");
  }
  Imat.resize((h - 2 * border), (w - 2 * border), false, false);
  for (unsigned i = border; i < h - border; ++i) {
    for (unsigned j = border; j < w - border; ++j) {
      Imat[i - border][j - border] = (double)I[i][j];
    }
  }
}

// vpLuminancePCA

vpLuminancePCA::vpLuminancePCA(const std::shared_ptr<vpMatrix> &basis, const std::shared_ptr<vpColVector> &mean, const vpColVector &explainedVariance)
  : vpLuminanceMapping(basis->getRows())
{
  m_Ih = m_Iw = 0;
  init(basis, mean, explainedVariance);
}

void vpLuminancePCA::init(const std::shared_ptr<vpMatrix> &basis, const std::shared_ptr<vpColVector> &mean, const vpColVector &variance)
{
  if (basis->getCols() != mean->getRows()) {
    throw vpException(vpException::dimensionError, "PCA mean and basis should have the same number of inputs");
  }
  if (variance.getRows() != basis->getRows()) {
    throw vpException(vpException::dimensionError, "PCA explained variance should have the same size as the subspace");
  }
  m_mappingSize = basis->getRows();
  m_basis = basis;
  m_mean = mean;
  m_explainedVariance = variance;
  m_border = vpFeatureLuminance::DEFAULT_BORDER;
}

vpLuminancePCA::vpLuminancePCA(const vpLuminancePCA &other) : vpLuminanceMapping(other.m_mappingSize)
{
  *this = other;
}

vpLuminancePCA &vpLuminancePCA::operator=(const vpLuminancePCA &other)
{
  m_basis = other.m_basis;
  m_mean = other.m_mean;
  m_explainedVariance = other.m_explainedVariance;
  m_mappingSize = other.m_mappingSize;
  m_border = other.m_border;
  m_Ivec = other.m_Ivec;
  m_Ih = other.m_Ih;
  m_Iw = other.m_Iw;

  return *this;
}

void vpLuminancePCA::map(const vpImage<unsigned char> &I, vpColVector &s)
{
  m_Ih = I.getHeight() - 2 * m_border;
  m_Iw = I.getWidth() - 2 * m_border;
  imageAsVector(I, m_Ivec, m_border);

  m_Ivec -= *m_mean;
  s = (*m_basis) * m_Ivec;
}

void vpLuminancePCA::inverse(const vpColVector &s, vpImage<unsigned char> &I)
{
  const vpColVector vI = ((*m_basis).transpose() * s + (*m_mean));
  I.resize(m_Ih, m_Iw);
  // Vector to image
  for (unsigned int i = 0; i < m_Ih; ++i) {
    for (unsigned int j = 0; j < m_Iw; ++j) {
      I[i][j] = static_cast<unsigned char>(vI[i * m_Iw + j]);
    }
  }
}

void vpLuminancePCA::interaction(const vpImage<unsigned char> &, const vpMatrix &LI, const vpColVector &, vpMatrix &L)
{
  L = (*m_basis) * LI;
}

vpLuminancePCA vpLuminancePCA::load(const std::string &basisFilename, const std::string &meanFilename, const std::string &explainedVarianceFile)
{
  std::shared_ptr<vpMatrix> basis = std::make_shared<vpMatrix>();
  std::shared_ptr<vpColVector> mean = std::make_shared<vpColVector>();
  vpColVector explainedVariance;
  vpMatrix::loadMatrix(basisFilename, *basis, true);
  vpMatrix::loadMatrix(meanFilename, *mean, true);
  vpMatrix::loadMatrix(explainedVarianceFile, explainedVariance, true);

  if (mean->getCols() > 1) {
    throw vpException(vpException::dimensionError,
    "Read something that was not a column vector when trying to load the PCA mean vector");
  }
  if (explainedVariance.getCols() > 1) {
    throw vpException(vpException::dimensionError,
    "Read something that was not a column vector when trying to load the PCA components explained variance");
  }
  if (basis->getCols() != mean->getRows()) {
    std::stringstream ss;
    ss << "Error when loading PCA from binary files";
    ss << "The basis matrix had dimensions (" << basis->getRows() << ", " << basis->getCols() << ")";
    ss << " and the mean vector had size " << mean->getRows() << ".";
    ss << "You may be loading data from two different PCAs";
    throw vpException(vpException::dimensionError, ss.str());
  }

  return vpLuminancePCA(basis, mean, explainedVariance);
}

void vpLuminancePCA::save(const std::string &basisFilename, const std::string &meanFilename, const std::string &explainedVarianceFile) const
{
  if (m_basis.get() == nullptr || m_mean.get() == nullptr) {
    throw vpException(vpException::notInitialized,
    "Tried to save a PCA projection that was uninitialized");
  }
  if (m_basis->size() == 0 || m_mean->getCols() == 0 || m_basis->getCols() != m_mean->getRows()) {
    throw vpException(vpException::dimensionError,
    "Tried to save a PCA projection but there are issues with the basis and mean dimensions");
  }
  vpMatrix::saveMatrix(basisFilename, *m_basis, true);
  vpMatrix::saveMatrix(meanFilename, *m_mean, true);
  vpMatrix::saveMatrix(explainedVarianceFile, m_explainedVariance, true);
}

vpLuminancePCA vpLuminancePCA::learn(const std::vector<vpImage<unsigned char>> &images, const unsigned int projectionSize, const unsigned int border)
{
  vpMatrix matrix;
  for (unsigned i = 0; i < images.size(); ++i) {
    const vpImage<unsigned char> &I = images[i];
    if (i == 0) {
      matrix.resize(images.size(), (I.getHeight() - 2 * border) * (I.getWidth() - 2 * border));
    }
    if ((I.getHeight() - 2 * border) * (I.getWidth() - 2 * border) != matrix.getCols()) {
      throw vpException(vpException::badValue, "Not all images have the same dimensions when learning pca");
    }
    for (unsigned j = border; j < I.getHeight() - border; ++j) {
      for (unsigned k = border; k < I.getWidth() - border; ++k) {
        matrix[i][(j - border) * (I.getWidth() - 2 * border) + k - border] = I[j][k];
      }
    }
  }

  return vpLuminancePCA::learn(matrix.transpose(), projectionSize);
}

#ifdef VISP_HAVE_MODULE_IO
vpLuminancePCA vpLuminancePCA::learn(const std::vector<std::string> &imageFiles, const unsigned int projectionSize, const unsigned int border)
{
  vpMatrix matrix;
  vpImage<unsigned char> I;
  for (unsigned i = 0; i < imageFiles.size(); ++i) {
    vpImageIo::read(I, imageFiles[i]);
    if (i == 0) {
      matrix.resize(imageFiles.size(), (I.getHeight() - 2 * border) * (I.getWidth() - 2 * border));
    }
    if ((I.getHeight() - 2 * border) * (I.getWidth() - 2 * border) != matrix.getCols()) {
      throw vpException(vpException::badValue, "Not all images have the same dimensions when learning pca");
    }
    for (unsigned j = border; j < I.getHeight() - border; ++j) {
      for (unsigned k = border; k < I.getWidth() - border; ++k) {
        matrix[i][(j - border) * (I.getWidth() - 2 * border) + k - border] = I[j][k];
      }
    }
  }
  return vpLuminancePCA::learn(matrix.transpose(), projectionSize);
}
#endif

vpLuminancePCA vpLuminancePCA::learn(const vpMatrix &images, const unsigned int projectionSize)
{
  if (projectionSize > images.getRows() || projectionSize > images.getCols()) {
    throw vpException(vpException::badValue, "Cannot use a subspace greater than the data dimensions (number of pixels or images)");
  }
  if (images.getRows() < images.getCols()) {
    throw vpException(vpException::badValue, "Cannot compute SVD when there are more images (columns) than pixels (rows)");
  }
  // Mean computation
  vpColVector mean(images.getRows(), 0.0);
  for (unsigned i = 0; i < images.getCols(); ++i) {
    mean += images.getCol(i);
  }
  mean /= images.getCols();

  // Before SVD, center data
  vpMatrix centered(images.getRows(), images.getCols());
  for (unsigned i = 0; i < centered.getRows(); ++i) {
    for (unsigned j = 0; j < centered.getCols(); ++j) {
      centered[i][j] = images[i][j] - mean[i];
    }
  }

  vpColVector eigenValues;
  vpMatrix V;
  centered.svd(eigenValues, V);
  vpMatrix U(centered.getRows(), projectionSize);
  for (unsigned i = 0; i < centered.getRows(); ++i) {
    for (unsigned j = 0; j < projectionSize; ++j) {
      U[i][j] = centered[i][j];
    }
  }
  double cumEigenValues = eigenValues.sum();
  vpColVector componentsExplainedVar(eigenValues, 0, projectionSize);
  componentsExplainedVar /= cumEigenValues;
  std::shared_ptr<vpMatrix> basis = std::make_shared<vpMatrix>(U.t());
  std::shared_ptr<vpColVector> meanPtr = std::make_shared<vpColVector>(mean);
  return vpLuminancePCA(basis, meanPtr, componentsExplainedVar);
}

//vpMatrixZigZagIndex
vpLuminanceDCT::vpMatrixZigZagIndex::vpMatrixZigZagIndex() { }

void vpLuminanceDCT::vpMatrixZigZagIndex::init(unsigned rows, unsigned cols)
{
  // Adapted from  https://www.geeksforgeeks.org/print-matrix-in-zig-zag-fashion/
  m_colIndex.resize(rows * cols);
  m_rowIndex.resize(rows * cols);
  m_rows = rows;
  m_cols = cols;
  int rowCount = static_cast<int>(rows);
  int colCount = static_cast<int>(cols);

  unsigned int index = 0;
  int row = 0, col = 0;

  bool row_inc = 0;

  int mindim = std::min(rowCount, colCount);
  for (int len = 1; len <= mindim; ++len) {
    for (int i = 0; i < len; ++i) {
      m_rowIndex[index] = row;
      m_colIndex[index] = col;
      ++index;
      if (i + 1 == len) {
        break;
      }

      if (row_inc) {
        ++row;
        --col;
      }
      else {
        --row;
        ++col;
      }
    }

    if (len == mindim) {
      break;
    }

    if (row_inc)
      ++row, row_inc = false;
    else
      ++col, row_inc = true;
  }

  // Update the indexes of row and col variable
  if (row == 0) {
    if (col == rowCount - 1) {
      ++row;
    }
    else {
      ++col;
    }
    row_inc = 1;
  }
  else {
    if (row == colCount - 1) {
      ++col;
    }
    else {
      ++row;
    }
    row_inc = 0;
  }

  // Print the next half zig-zag pattern
  int maxdim = std::max(rowCount, rowCount) - 1;
  for (int len, diag = maxdim; diag > 0; --diag) {

    if (diag > mindim) {
      len = mindim;
    }
    else {
      len = diag;
    }

    for (int i = 0; i < len; ++i) {
      m_rowIndex[index] = row;
      m_colIndex[index] = col;
      ++index;

      if (i + 1 == len) {
        break;
      }

      if (row_inc) {
        ++row;
        --col;
      }
      else {
        ++col;
        --row;
      }
    }

    if (row == 0 || col == rowCount - 1) {
      if (col == rowCount - 1) {
        ++row;
      }
      else {
        ++col;
      }
      row_inc = true;
    }

    else if (col == 0 || row == colCount - 1) {
      if (row == colCount - 1) {
        ++col;
      }
      else {
        ++row;
      }
      row_inc = false;
    }
  }
}

void vpLuminanceDCT::vpMatrixZigZagIndex::getValues(const vpMatrix &m, unsigned int start, unsigned int end, vpColVector &s) const
{
  if (m.getRows() != m_rows || m.getCols() != m_cols) {
    throw vpException(vpException::dimensionError, "Input matrix has wrong dimensions");
  }

  if (end <= start) {
    throw vpException(vpException::dimensionError, "End index should be > to the start index");
  }

  s.resize(end - start, false);

  for (unsigned index = start; index < end; ++index) {
    s[index - start] = m[m_rowIndex[index]][m_colIndex[index]];
  }
}

void vpLuminanceDCT::vpMatrixZigZagIndex::setValues(const vpColVector &s, unsigned int start, vpMatrix &m) const
{
  if (m.getRows() != m_rows || m.getCols() != m_cols) {
    throw vpException(vpException::dimensionError, "Input matrix has wrong dimensions");
  }

  if (start + s.size() > m.size()) {
    throw vpException(vpException::dimensionError, "Start index combined to vector size exceeds matrix size");
  }

  for (unsigned index = start; index < start + s.size(); ++index) {
    m[m_rowIndex[index]][m_colIndex[index]] = s[index - start];
  }
}

// vpLuminanceDCT

vpLuminanceDCT::vpLuminanceDCT(const vpLuminanceDCT &other) : vpLuminanceMapping(other.getProjectionSize())
{
  *this = other;
}

void vpLuminanceDCT::map(const vpImage<unsigned char> &I, vpColVector &s)
{
  m_Ih = I.getHeight() - 2 * m_border;
  m_Iw = I.getWidth() - 2 * m_border;
  if (m_Imat.getCols() != m_Ih || m_Imat.getRows() != m_Iw) {
    computeDCTMatrices(m_Ih, m_Iw);
    m_zigzag.init(m_Ih, m_Iw);
  }
  imageAsMatrix(I, m_Imat, m_border);
  m_dct = m_Dcols * m_Imat * m_Drows;
  m_zigzag.getValues(m_dct, 0, m_mappingSize, s);
}

void vpLuminanceDCT::computeDCTMatrix(vpMatrix &D, unsigned int n) const
{
  D.resize(n, n, false, false);
  for (unsigned i = 0; i < n; i++) {
    D[0][i] = 1.0 / sqrt(n);
  }
  double alpha = sqrt(2./(n));
  for (unsigned int i = 1; i < n; i++) {
    for (unsigned int j = 0; j < n; j++) {
      D[i][j] = alpha*cos((2 * j + 1) * i * M_PI / (2.0 * n));
    }
  }
}

void vpLuminanceDCT::computeDCTMatrices(unsigned int rows, unsigned int cols)
{
  computeDCTMatrix(m_Dcols, rows);
  computeDCTMatrix(m_Drows, cols);
  m_Drows = m_Drows.transpose();
}

void vpLuminanceDCT::inverse(const vpColVector &s, vpImage<unsigned char> &I)
{
  vpMatrix dctCut(m_dct.getRows(), m_dct.getCols(), 0.0);
  m_zigzag.setValues(s, 0, dctCut);
  const vpMatrix Ir = m_Dcols.t() * dctCut * m_Drows.t();
  I.resize(Ir.getRows(), Ir.getCols());
  for (unsigned int i = 0; i < I.getRows(); ++i) {
    for (unsigned int j = 0; j < I.getCols(); ++j) {
      I[i][j] = std::max(0.0, std::min(Ir[i][j], 255.0));
    }
  }
}

void vpLuminanceDCT::interaction(const vpImage<unsigned char> &, const vpMatrix &LI, const vpColVector &, vpMatrix &L)
{
  const vpMatrix LIT = LI.t();
  for (unsigned int dof = 0; dof < 6; ++dof) {
    m_dIdrPlanes[dof].resize(m_Ih, m_Iw, false, false);
    memcpy(m_dIdrPlanes[dof].data, LIT[dof], m_Ih * m_Iw * sizeof(double));
  }

  L.resize(m_mappingSize, 6, false, false);
  vpMatrix dTddof(m_Ih, m_Iw);
  vpColVector column;
  for (unsigned int dof = 0; dof < 6; ++dof) {
    dTddof = m_Dcols * m_dIdrPlanes[dof] * m_Drows;
    m_zigzag.getValues(dTddof, 0, m_mappingSize, column);
    for (unsigned int row = 0; row < L.getRows(); ++row) {
      L[row][dof] = column[row];
    }
  }
}

// Feature luminance mapping

vpFeatureLuminanceMapping::vpFeatureLuminanceMapping(const vpCameraParameters &cam,
unsigned int h, unsigned int w, double Z, std::shared_ptr<vpLuminanceMapping> mapping)
{
  init(cam, h, w, Z, mapping);
}

vpFeatureLuminanceMapping::vpFeatureLuminanceMapping(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping)
{
  init(luminance, mapping);
}
vpFeatureLuminanceMapping::vpFeatureLuminanceMapping(const vpFeatureLuminanceMapping &f) : vpBasicFeature()
{
  *this = f;
}

void vpFeatureLuminanceMapping::init()
{
  dim_s = 0;
  m_featI.init(0, 0, 0.0);
  m_mapping = nullptr;
}

void vpFeatureLuminanceMapping::init(
  const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z,
  std::shared_ptr<vpLuminanceMapping> mapping)
{
  m_featI.init(h, w, Z);
  m_featI.setCameraParameters(cam);
  m_mapping = mapping;
  m_mapping->setBorder(m_featI.getBorder());
  dim_s = m_mapping->getProjectionSize();
  s.resize(dim_s, true);
}
void vpFeatureLuminanceMapping::init(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping)
{
  m_featI = luminance;
  m_mapping = mapping;
  dim_s = m_mapping->getProjectionSize();
  m_mapping->setBorder(m_featI.getBorder());
  s.resize(dim_s, true);
}

vpFeatureLuminanceMapping &vpFeatureLuminanceMapping::operator=(const vpFeatureLuminanceMapping &f)
{
  dim_s = f.dim_s;
  s = f.s;
  m_mapping = f.m_mapping;
  m_featI = f.m_featI;
  return *this;
}
vpFeatureLuminanceMapping *vpFeatureLuminanceMapping::duplicate() const
{
  return new vpFeatureLuminanceMapping(*this);
}

void vpFeatureLuminanceMapping::build(vpImage<unsigned char> &I)
{
  m_featI.build(I);
  m_featI.interaction(m_LI);
  m_mapping->map(I, s);
}

vpColVector vpFeatureLuminanceMapping::error(const vpBasicFeature &s_star, unsigned int select)
{
  if (select != FEATURE_ALL) {
    throw vpException(vpException::notImplementedError, "cannot compute error on subset of a mapping");
  }
  vpColVector e(dim_s);
  error(s_star, e);
  return e;
}

void vpFeatureLuminanceMapping::error(const vpBasicFeature &s_star, vpColVector &e)
{
  e.resize(dim_s, false);
  for (unsigned int i = 0; i < dim_s; i++) {
    e[i] = s[i] - s_star[i];
  }
}

vpMatrix vpFeatureLuminanceMapping::interaction(unsigned int select)
{
  if (select != FEATURE_ALL) {
    throw vpException(vpException::notImplementedError, "cannot compute interaction matrix for a subset of a mapping");
  }
  vpMatrix dsdr(dim_s, 6);
  interaction(dsdr);
  return dsdr;
}
void vpFeatureLuminanceMapping::interaction(vpMatrix &L)
{
  L.resize(dim_s, 6, false, false);
  m_featI.interaction(m_LI);
  m_mapping->interaction(I, m_LI, s, L);
}

void vpFeatureLuminanceMapping::print(unsigned int /*select*/) const
{
  std::cout << s << std::endl;
}
END_VISP_NAMESPACE
#endif // C++11
