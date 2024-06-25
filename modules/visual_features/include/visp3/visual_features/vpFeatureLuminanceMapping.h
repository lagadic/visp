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
 * Luminance based feature.
 */

#ifndef VP_FEATURE_LUMINANCE_MAPPING_H
#define VP_FEATURE_LUMINANCE_MAPPING_H
#include <visp3/core/vpConfig.h>
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <array>
#include <memory>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureLuminance.h>

BEGIN_VISP_NAMESPACE

/*!
 * \brief Base class for functions that map an image and its interaction matrix to a different domain.
 *
 * \ingroup group_visual_features
 *
 * - The mapping\f$ \mathbf{I} \rightarrow \mathbf{z}\f$ is done via vpLuminanceMapping::map
 * - The projection of the interaction matrix \f$ \mathbf{L_I} \rightarrow \mathbf{L_z}\f$ is performed in vpLuminanceMapping::interaction
 * - If possible the inverse mapping (i.e., image reconstruction) is available throug vpLuminanceMapping::inverse
*/
class VISP_EXPORT vpLuminanceMapping
{
public:
  /**
   * \brief Construct a new vp Luminance Mapping object
   *
   * @param mappingSize The size of the space that this transformation maps to.
   */
  vpLuminanceMapping(unsigned int mappingSize) : m_mappingSize(mappingSize) { }

  /**
   * Destructor.
   */
  virtual ~vpLuminanceMapping() { }

  /**
   * \brief Map an image \p I to a representation \p s.
   * This representation s has getProjectionSize() rows.
   *
   * Note that when combined with vpFeatureLuminanceMapping,
   * The image \p I does not have the same size as the image input of vpFeatureLuminanceMapping::build.
   * \p I is the center crop of this image.
   * @param I The input image
   * @param s The resulting representation that will serve as visual servoing features.
   */
  virtual void map(const vpImage<unsigned char> &I, vpColVector &s) = 0;

  /**
   * \brief Compute the interaction matrix associated with the representation \p s
   *
   * @param I input image used to compute s
   * @param LI Photometric interaction matrix associated to \p I (see vpFeatureLuminance)
   * @param s the already computed representation
   * @param L The output interaction matrix, of dimensions getProjectionSize() x 6
   */
  virtual void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) = 0;

  /**
   * \brief Reconstruct \p I from a representation \p s
   *
   * @param s the representation
   * @param I Output lossy reconstruction
   */
  virtual void inverse(const vpColVector &s, vpImage<unsigned char> &I) = 0;

  /**
   * \brief Returns the size of the space to which an image is mapped to.
   *
   * @return space size
   */
  unsigned int getProjectionSize() const { return m_mappingSize; }

  /**
   * \brief Returns the number of pixels that are removed by the photometric VS computation
   *
   * @return space size
   */
  unsigned int getBorder() const { return m_border; }

  /**
   * \brief Set the number of pixels that are removed by the photometric VS computation
   * This function should be called by vpFeatureLuminanceMapping
   *
   * @param border
   */
  void setBorder(unsigned border) { m_border = border; }

  static void imageAsVector(const vpImage<unsigned char> &I, vpColVector &Ivec, unsigned border);
  static void imageAsMatrix(const vpImage<unsigned char> &I, vpMatrix &Imat, unsigned border);

protected:
  unsigned m_mappingSize; //! Final vector size
  unsigned m_border; //! Borders that were removed during raw photometric VS computation
};

/**
 * \brief Implementation of \cite Marchand19a.
 *
 * \ingroup group_visual_features
 *
 * Projects an image onto an orthogonal subspace,
 * obtained via Principal Component Analysis (PCA).
 *
 * The orthogonal basis is obtained through Singular Value Decomposition of a dataset of images (see vpLuminancePCA::learn)
 * where the \f$ k \f$ first basis vectors that explain the most variance are kept.
 *
 * an image \f$ I \f$ is projected to the representation \f$ \mathbf{s} \f$ with:
 * \f[ \mathbf{s} = \mathbf{U}^\top (vec(\mathbf{I}) - vec(\mathbf{\bar I})) \f]
 *
 * with \f$ \mathbf{U} \f$ the subspace projection matrix (\f$ dim(\mathbf{I}) \times k \f$) and \f$ \mathbf{\bar I} \f$ is the average image computed from the dataset.
 *
 *
 */
class VISP_EXPORT vpLuminancePCA : public vpLuminanceMapping
{
public:
  vpLuminancePCA() : vpLuminanceMapping(0), m_basis(nullptr), m_mean(nullptr), m_Ivec(0), m_Ih(0), m_Iw(0) { }

  /**
   * Destructor.
   */
  virtual ~vpLuminancePCA() { }

  /**
   * \brief Build a new PCA object
   *
   * @param basis \f$ \mathbf{U}^\top \f$ a k x dim(I) matrix
   * @param mean  \f$ vec(\mathbf{\bar I}) \f$ the mean image represented as a vector
   * @param explainedVariance The explained variance for each of the k vectors.
   */
  vpLuminancePCA(const std::shared_ptr<vpMatrix> &basis, const std::shared_ptr<vpColVector> &mean, const vpColVector &explainedVariance);

  /**
   * \brief Copy constructor: does not make a deep copy of the basis and mean
   */
  vpLuminancePCA(const vpLuminancePCA &other);

  vpLuminancePCA &operator=(const vpLuminancePCA &other);

  /**
   * \brief Initialize the PCA object with a basis, mean and explained variance vector
   *
   * \sa vpLuminancePCA()
   * @param basis
   * @param mean
   * @param variance
   */
  void init(const std::shared_ptr<vpMatrix> &basis, const std::shared_ptr<vpColVector> &mean, const vpColVector &variance);

  /**
   * \brief Get \f$ \mathbf{U}^\top \f$, the subspace projection matrix (\f$ k \times dim(\mathbf{I}) \f$)
   *
   * @return std::shared_ptr<vpColVector>
   */
  std::shared_ptr<vpMatrix> getBasis() const { return m_basis; }
  /**
   * \brief Get \f$ vec(\mathbf{\bar I}) \f$, the mean image computed from the dataset.
   * @return std::shared_ptr<vpColVector>
   */
  std::shared_ptr<vpColVector> getMean() const { return m_mean; }

  /**
   * \brief Get the values of explained variance by each of the eigen vectors.
   *
   * When all eigenvectors of the dataset are considered, the explained variance total is 1.
   * When they are not all considered (as should be the case), their sum should be below 1.
   * @return vpColVector
   */
  vpColVector getExplainedVariance() const { return m_explainedVariance; }

  void map(const vpImage<unsigned char> &I, vpColVector &s) VP_OVERRIDE;
  void inverse(const vpColVector &s, vpImage<unsigned char> &I) VP_OVERRIDE;
  void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) VP_OVERRIDE;

  /**
   * \brief Save the PCA basis to multiple text files, for later use via the \ref load function.
   *
   * @param basisFilename The file in which \f$ \mathbf{U}^\top \f$ is stored
   * @param meanFileName The file in which \f$ \mathbf{\bar I} \f$ is stored
   * @param explainedVarianceFile The file containing the explained variance.
   *
   * \throws if the basis is null or mean is null
   */
  void save(const std::string &basisFilename, const std::string &meanFileName, const std::string &explainedVarianceFile) const;

  /**
   * \brief Save the PCA basis to multiple text files, for later use via the \ref load function.
   *
   * @param basisFilename The file in which \f$ \mathbf{U}^\top \f$ is stored
   * @param meanFileName The file in which \f$ \mathbf{\bar I} \f$ is stored
   * @param explainedVarianceFile The file containing the explained variance.
   *
   * \throws if files cannot be read, or if basis and mean dimensions are incorrect.
   */
  static vpLuminancePCA load(const std::string &basisFilename, const std::string &meanFileName, const std::string &explainedVarianceFile);

#ifdef VISP_HAVE_MODULE_IO
  /**
   * \brief Compute a new Principal Component Analysis on set of images, stored on disk.
   *
   * @param imageFiles The list of image paths to load and use to compute the PCA
   * @param projectionSize the number of eigenvectors that are kept for the final projection
   * @param imageBorder The number of pixels to crop on each side of the image before adding it to the image set, effectively taking the center crop.
   * Useful when the stored images do not have the correct dimensions
   * @return the PCA computed on the imageFiles
   *
   * \throws if the images do not have the same dimensions
   */
  static vpLuminancePCA learn(const std::vector<std::string> &imageFiles, const unsigned int projectionSize, const unsigned int imageBorder = 0);
#endif

  /**
   * \brief Compute a new Principal Component Analysis on set of images.
   *
   * @param images The list of images used to compute the PCA
   * @param projectionSize the number of eigenvectors that are kept for the final projection
   * @param imageBorder The number of pixels to crop on each side of the image before adding it to the image set, effectively taking the center crop.
   * Useful when the images do not have the correct dimensions. Typically, the input image to PCA is smaller than the one used when computing luminance features, since the latter step requires crops the image.
   * @return the PCA computed on the images
   *
   * \throws if the images do not have the same dimensions
   */
  static vpLuminancePCA learn(const std::vector<vpImage<unsigned char>> &images, const unsigned int projectionSize, const unsigned int imageBorder = 0);
  /**
   * \brief Compute a new Principal Component Analysis on dataset
   *
   * @param images The data matrix, where each column represents a single data point (image)
   * @param projectionSize the number of eigenvectors that are kept for the final projection
   * @return the PCA computed on the images
   */
  static vpLuminancePCA learn(const vpMatrix &images, const unsigned int projectionSize);


private:
  std::shared_ptr<vpMatrix> m_basis; //! \f$ \mathbf{U}^\top \f$ a K by dim(I) orthogonal matrix
  std::shared_ptr<vpColVector> m_mean; //! \f$ \mathbf{\bar I} \f$ The mean image
  vpColVector m_explainedVariance; //! The explained variance
  vpColVector m_Ivec; //! Vector representation of the image
  unsigned int m_Ih, m_Iw; //! Input image dimensions (without borders);
};

/**
 * \brief Implementation of \cite Marchand20a.
 *
 * \ingroup group_visual_features
 *
 * Computes the Discrete Cosine Transform (DCT) representation of the image.
 * Only the K first components are preserved and stored into a vector when calling map. These components correspond to the lowest frequencies of the input image.
 */
class VISP_EXPORT vpLuminanceDCT : public vpLuminanceMapping
{
public:
  /**
   * \brief Helper class to iterate and get/set the values from a matrix, following a zigzag pattern.
   *
   */
  class VISP_EXPORT vpMatrixZigZagIndex
  {
  public:
    vpMatrixZigZagIndex();
    /**
     * \brief Initalize the ZigZag object. Computes and stores the zigzag indexing for a given matrix size
     *
     * @param rows the matrix's number of rows
     * @param cols the matrix's number of cols
     */
    void init(unsigned rows, unsigned cols);
    /**
     * \brief Fill the vector s with (end - start) values, according to the zigzag matrix indexing strategy
     *
     * @param m the matrix
     * @param start The first value. Use 0 to start with the matrix's top left value
     * @param end The last value to store in the vector. (exclusive)
     * @param s The vector in which to store the values
     */
    void getValues(const vpMatrix &m, unsigned int start, unsigned int end, vpColVector &s) const;

    /**
     * \brief set the values in the matrix, according to the values stored in the vector s and the zigzag indexing strategy
     *
     * @param s The vector from which to set the values
     * @param start the zigzag index at which to start filling values
     * @param m The matrix in which the values will be replaced
     */
    void setValues(const vpColVector &s, unsigned int start, vpMatrix &m) const;

  private:
    std::vector<unsigned> m_rowIndex; // Contains the row index of the nth value of the zigzag indexing
    std::vector<unsigned> m_colIndex; // Contains the row index of the nth value of the zigzag indexing
    unsigned m_rows;
    unsigned m_cols;
  };

  /**
   * \brief Build a new DCT object
   *
   * @param k the number of components to keep from the DCT matrix and use as servoing features
   */
  vpLuminanceDCT(const unsigned int k) : vpLuminanceMapping(k)
  {
    init(k);
  }

  /**
   * \brief Initialize the DCT object with the number of required components
   */
  void init(const unsigned int k)
  {
    m_mappingSize = k;
    m_border = vpFeatureLuminance::DEFAULT_BORDER;
    m_Ih = m_Iw = 0;
  }

  /**
   * \brief Copy constructor
   */
  vpLuminanceDCT(const vpLuminanceDCT &other);

  vpLuminanceDCT &operator=(const vpLuminanceDCT &other) = default;

  void map(const vpImage<unsigned char> &I, vpColVector &s) VP_OVERRIDE;
  void inverse(const vpColVector &s, vpImage<unsigned char> &I) VP_OVERRIDE;
  void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) VP_OVERRIDE;

private:
  void computeDCTMatrix(vpMatrix &D, unsigned int n) const;
  void computeDCTMatrices(unsigned int rows, unsigned int cols);

protected:
  unsigned m_Ih, m_Iw; //! image dimensions (without borders)
  vpMatrix m_Imat; //! Image as a matrix
  vpMatrix m_dct; //! DCT representation of the image
  vpMatrix m_Dcols, m_Drows; //! the computed DCT matrices. The separable property of DCt is used so that a 1D DCT is computed on rows and another on columns of the result of the first dct;
  std::array<vpMatrix, 6> m_dIdrPlanes; //! Luminance interaction matrix, seen as six image planes
  vpLuminanceDCT::vpMatrixZigZagIndex m_zigzag; //! zigzag indexing helper
};

/**
 * \brief Class to combine luminance features (photometric servoing)
 *
 * \ingroup group_visual_features
 *
 * with a mapping \f$ f(\mathbf{I}) \f$ that projects an image to a low dimensional representation \f$ \mathbf{s} \f$ (see vpLuminanceMapping::map).
 * The interaction matrix of \f$ \mathbf{s} \f$ is computed as a function of \f$ \mathbf{I}, \mathbf{L_I} \f$ (see vpLuminanceMapping::interaction)
 *
 * The mapping \f$ f \f$ is applied to the center crop of the image,
 * where the interaction matrix of the pixels can be computed (see vpFeatureLuminance::getBorder).
 *
 * \see vpLuminanceDCT, vpLuminancePCA, vpFeatureLuminance
 */
class VISP_EXPORT vpFeatureLuminanceMapping : public vpBasicFeature
{
public:
  vpFeatureLuminanceMapping(const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z, const std::shared_ptr<vpLuminanceMapping> mapping);
  vpFeatureLuminanceMapping(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping);
  void init() VP_OVERRIDE;
  void init(const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z, std::shared_ptr<vpLuminanceMapping> mapping);
  void init(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping);

  vpFeatureLuminanceMapping(const vpFeatureLuminanceMapping &f);
  vpFeatureLuminanceMapping &operator=(const vpFeatureLuminanceMapping &f);
  vpFeatureLuminanceMapping *duplicate() const VP_OVERRIDE;

  virtual ~vpFeatureLuminanceMapping() = default;

  void build(vpImage<unsigned char> &I);

  void display(const vpCameraParameters &, const vpImage<unsigned char> &, const vpColor & = vpColor::green,
               unsigned int = 1) const VP_OVERRIDE
  { }
  void display(const vpCameraParameters &, const vpImage<vpRGBa> &, const vpColor & = vpColor::green,
               unsigned int = 1) const VP_OVERRIDE
  { }

  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) VP_OVERRIDE;
  void error(const vpBasicFeature &s_star, vpColVector &e);

  vpMatrix interaction(unsigned int select = FEATURE_ALL) VP_OVERRIDE;
  void interaction(vpMatrix &L);

  void print(unsigned int select = FEATURE_ALL) const VP_OVERRIDE;

  vpFeatureLuminance &getLuminanceFeature() { return m_featI; }
  std::shared_ptr<vpLuminanceMapping> &getMapping() { return m_mapping; }

private:
  std::shared_ptr<vpLuminanceMapping> m_mapping;
  vpFeatureLuminance m_featI;
  vpMatrix m_LI; //! Photometric interaction matrix
  vpImage<unsigned char> I;
};
END_VISP_NAMESPACE
#endif
#endif
