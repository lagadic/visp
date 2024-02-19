/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

#ifndef vpFeatureLuminanceMapping_h
#define vpFeatureLuminanceMapping_h

#include <memory>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureLuminance.h>

/**
 * @brief Base class for functions that map an image and its interaction matrix to a different domain.
 * * The mapping\f$ \mathbf{I} \rightarrow \mathbf{z}\f$ is done via vpLuminanceMapping::map
 * * The projection of the interaction matrix \f$ \mathbf{L_I} \rightarrow \mathbf{L_z}\f$ is performed in vpLuminanceMapping::interaction
 * * If possible the inverse mapping (i.e., image reconstruction) is available throug vpLuminanceMapping::inverse
 */
class VISP_EXPORT vpLuminanceMapping
{
public:
  /**
   * @brief Construct a new vp Luminance Mapping object
   *
   * @param mappingSize The size of the space that this transformation maps to.
   */
  vpLuminanceMapping(unsigned int mappingSize) : m_mappingSize(mappingSize) { }
  /**
   * @brief Map an image \ref I to a representation \ref s.
   * This representation s has getProjectionSize() rows.
   *
   * Note that when combined with vpFeatureLuminanceMapping,
   * The image \ref I does not have the same size as the image input of vpFeatureLuminanceMapping::buildFrom.
   * \ref I is the center crop of this image.
   * @param I The input image
   * @param s The resulting representation that will serve as visual servoing features.
   */
  virtual void map(const vpImage<unsigned char> &I, vpColVector &s) = 0;
  /**
   * @brief Compute the interaction matrix associated with the representation \ref s
   *
   * @param I input image used to compute s
   * @param LI Photometric interaction matrix associated to \ref I (see vpFeatureLuminance)
   * @param s the already computed representation
   * @param L The output interaction matrix, of dimensions getProjectionSize() x 6
   */
  virtual void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) = 0;
  /**
   * @brief Reconstruct \ref I from a representation \ref s
   *
   * @param s the representation
   * @param I Output lossy reconstruction
   */
  virtual void inverse(const vpColVector &s, vpImage<unsigned char> &I) = 0;

  /**
   * @brief Returns the size of the space to which an image is mapped to.
   *
   * @return space size
   */
  unsigned int getProjectionSize() const { return m_mappingSize; }

  static void imageAsVector(const vpImage<unsigned char> &I, vpColVector &Ivec, unsigned border);

protected:
  unsigned m_mappingSize;
};


/**
 * @brief Implementation of \cite{Marchand19a}.
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

  vpLuminancePCA() : vpLuminanceMapping(0), m_basis(nullptr), m_mean(nullptr) { }

  /**
   * @brief Build a new PCA object
   *
   * @param basis \f$ \mathbf{U}^\top \f$ a k x dim(I) matrix
   * @param mean  \f$ vec(\mathbf{\bar I}) \f$ the mean image represented as a vector
   * @param explainedVariance The explained variance for each of the k vectors.
   */
  vpLuminancePCA(const std::shared_ptr<vpMatrix> &basis, const std::shared_ptr<vpColVector> &mean, const vpColVector &explainedVariance);

  /**
   * @brief Initialize the PCA object with a basis, mean and explained variance vector
   *
   * \sa vpLuminancePCA()
   * @param basis
   * @param mean
   * @param variance
   */
  void init(const std::shared_ptr<vpMatrix> &basis, const std::shared_ptr<vpColVector> &mean, const vpColVector &variance);

  /**
   * @brief Get \f$ \mathbf{U}^\top \f$, the subspace projection matrix (\f$ k \times dim(\mathbf{I}) \f$)
   *
   * @return std::shared_ptr<vpColVector>
   */
  std::shared_ptr<vpMatrix> getBasis() const { return m_basis; }
  /**
   * @brief Get \f$ vec(\mathbf{\bar I}) \f$, the mean image computed from the dataset.
   * @return std::shared_ptr<vpColVector>
   */
  std::shared_ptr<vpColVector> getMean() const { return m_mean; }

  /**
   * @brief Get the values of explained variance by each of the eigen vectors.
   *
   * When all eigenvectors of the dataset are considered, the explained variance total is 1.
   * When they are not all considered (as should be the case), their sum should be below 1.
   * @return vpColVector
   */
  vpColVector getExplainedVariance() const { return m_explainedVariance; }

  void map(const vpImage<unsigned char> &I, vpColVector &s) vp_override;
  void inverse(const vpColVector &s, vpImage<unsigned char> &I) vp_override;
  void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) vp_override;

  /**
   * @brief Save the PCA basis to multiple text files, for later use via the \ref load function.
   *
   * @param basisFilename The file in which \f$ \mathbf{U}^\top \f$ is stored
   * @param meanFileName The file in which \f$ \mathbf{\bar I} \f$ is stored
   * @param explainedVarianceFile The file containing the explained variance.
   *
   * \throws if the basis is null or mean is null
   */
  void save(const std::string &basisFilename, const std::string &meanFileName, const std::string &explainedVarianceFile) const;

  /**
   * @brief Save the PCA basis to multiple text files, for later use via the \ref load function.
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
   * @brief Compute a new Principal Component Analysis on set of images, stored on disk.
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
   * @brief Compute a new Principal Component Analysis on set of images.
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
   * @brief Compute a new Principal Component Analysis on dataset
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
  vpColVector m_Ivec;
};

class VISP_EXPORT vpFeatureLuminanceMapping : public vpBasicFeature
{

public:

  vpFeatureLuminanceMapping(const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z, const std::shared_ptr<vpLuminanceMapping> mapping);
  vpFeatureLuminanceMapping(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping);
  void init() vp_override;
  void init(const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z, std::shared_ptr<vpLuminanceMapping> mapping);
  void init(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping);

  vpFeatureLuminanceMapping(const vpFeatureLuminanceMapping &f);
  vpFeatureLuminanceMapping &operator=(const vpFeatureLuminanceMapping &f);
  vpFeatureLuminanceMapping *duplicate() const vp_override;

  virtual ~vpFeatureLuminanceMapping() = default;

  void buildFrom(vpImage<unsigned char> &I);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const vp_override;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const vp_override;


  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) vp_override;
  void error(const vpBasicFeature &s_star, vpColVector &e);

  vpMatrix interaction(unsigned int select = FEATURE_ALL) vp_override;
  void interaction(vpMatrix &L);

  void print(unsigned int select = FEATURE_ALL) const vp_override;

  vpFeatureLuminance &getLuminanceFeature() { return m_featI; }
  std::shared_ptr<vpLuminanceMapping> &getMapping() { return m_mapping; }

private:

  std::shared_ptr<vpLuminanceMapping> m_mapping;
  vpFeatureLuminance m_featI;
  vpMatrix m_LI; //! Photometric interaction matrix
  vpImage<unsigned char> I;

};

#endif
