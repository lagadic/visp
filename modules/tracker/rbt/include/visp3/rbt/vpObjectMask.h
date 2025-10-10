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
 */

/*!
  \file vpObjectMask.h
  \brief Object mask segmentation for the render-based tracker
*/
#ifndef VP_OBJECT_MASK_H
#define VP_OBJECT_MASK_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE

template <typename T>
class vpImage;
class vpRBFeatureTrackerInput;

/**
 * \brief
 *
 * \ingroup group_rbt_mask
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you want to have an in-depth presentation of the Render-Based Tracker (RBT), you may have a look at:</span><br>
 *
 * - \ref tutorial-tracking-rbt
*/
class VISP_EXPORT vpObjectMask
{
public:
  virtual ~vpObjectMask() = default;
  virtual void updateMask(const vpRBFeatureTrackerInput &frame,
                          const vpRBFeatureTrackerInput &previousFrame,
                          vpImage<float> &mask) = 0;

  virtual void display(const vpImage<float> &mask, vpImage<unsigned char> &Imask) const;

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j) = 0;
#endif
};

END_VISP_NAMESPACE

#endif
