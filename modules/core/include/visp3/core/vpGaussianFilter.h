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
 * Gaussian filter class.
 *
 *****************************************************************************/

/*!
  \file vpGaussianFilter.h
  \brief Gaussian filter class
*/

#ifndef vpGaussianFilter_H
#define vpGaussianFilter_H

#include <visp3/core/vpImage.h>

/*!
  \class vpGaussianFilter

  \ingroup group_core_image

  \brief Gaussian filter class
*/
class VISP_EXPORT vpGaussianFilter
{
public:
  vpGaussianFilter(unsigned int width, unsigned int height, float sigma, bool deinterleave=false);

  void apply(const vpImage<unsigned char>& I, vpImage<unsigned char>& I_blur);

  void apply(const vpImage<vpRGBa>& I, vpImage<vpRGBa>& I_blur);

  virtual ~vpGaussianFilter();

private:
  vpGaussianFilter(const vpGaussianFilter&);
  vpGaussianFilter& operator=(const vpGaussianFilter&);

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};

#endif
