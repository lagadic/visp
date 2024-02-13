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


class VISP_EXPORT vpLuminanceMapping
{
public:
  vpLuminanceMapping(unsigned int mappingSize) : m_mappingSize(mappingSize) { }
  virtual void map(const vpImage<unsigned char> &I, vpColVector &s) = 0;
  virtual void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) = 0;
  virtual void inverse(const vpColVector &s, vpImage<unsigned char> &I) = 0;

  unsigned int getProjectionSize() const { return m_mappingSize; }

private:
  const unsigned m_mappingSize;
};


/**
 * @brief
 *
 */
class VISP_EXPORT vpLuminancePCA : public vpLuminanceMapping
{
public:
  vpLuminancePCA() : vpLuminanceMapping(0) { }
  vpLuminancePCA(std::shared_ptr<vpMatrix> basis, std::shared_ptr<vpColVector> mean);
  vpMatrix getBasis() const { return *m_basis; }
  vpColVector getMean() const { return *m_mean; }
  void map(const vpImage<unsigned char> &I, vpColVector &s) override;
  void inverse(const vpColVector &s, vpImage<unsigned char> &I) override;
  void interaction(const vpImage<unsigned char> &I, const vpMatrix &LI, const vpColVector &s, vpMatrix &L) override;

  void save(const std::string &basisFilename, const std::string &meanFileName) const;
  static vpLuminancePCA load(const std::string &basisFilename, const std::string &meanFileName);
#ifdef VISP_HAVE_MODULE_IO
  static vpLuminancePCA learn(const std::vector<std::string> &imageFiles, const unsigned int projectionSize, const unsigned int imageBorder = 0);
#endif
  static vpLuminancePCA learn(const std::vector<vpImage<unsigned char>> &images, const unsigned int projectionSize, const unsigned int imageBorder = 0);
  static vpLuminancePCA learn(const vpMatrix &images, const unsigned int projectionSize);


private:
  std::shared_ptr<vpMatrix> m_basis;
  std::shared_ptr<vpColVector> m_mean;
};

class VISP_EXPORT vpFeatureLuminanceMapping : public vpBasicFeature
{

public:

  vpFeatureLuminanceMapping(const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z, const std::shared_ptr<vpLuminanceMapping> mapping);
  vpFeatureLuminanceMapping(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping);
  void init() override;
  void init(const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z, std::shared_ptr<vpLuminanceMapping> mapping);
  void init(const vpFeatureLuminance &luminance, std::shared_ptr<vpLuminanceMapping> mapping);

  vpFeatureLuminanceMapping(const vpFeatureLuminanceMapping &f);
  vpFeatureLuminanceMapping &operator=(const vpFeatureLuminanceMapping &f);
  vpFeatureLuminanceMapping *duplicate() const override;

  virtual ~vpFeatureLuminanceMapping() = default;

  void buildFrom(vpImage<unsigned char> &I);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const override;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const override;


  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) override;
  void error(const vpBasicFeature &s_star, vpColVector &e);

  vpMatrix interaction(unsigned int select = FEATURE_ALL) override;
  void interaction(vpMatrix &L);


  void print(unsigned int select = FEATURE_ALL) const override;

  vpFeatureLuminance &getLuminanceFeature() { return m_featI; }
  std::shared_ptr<vpLuminanceMapping> &getMapping() { return m_mapping; }




private:

  std::shared_ptr<vpLuminanceMapping> m_mapping;
  vpFeatureLuminance m_featI;
  vpMatrix m_LI; //! Photometric interaction matrix

};

#endif
