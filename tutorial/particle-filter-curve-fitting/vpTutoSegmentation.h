/****************************************************************************
 *
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
*****************************************************************************/
#ifndef VP_TUTO_SEGMENTATION_H
#define VP_TUTO_SEGMENTATION_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageMorphology.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpImageTools.h>

#include "vpTutoCommonData.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace tutorial
{
/**
 * \brief Perform a segmentation of the original image based on the HSV color encoding.
 *
 * \param[in] data Data common to the whole program.
 */
void performSegmentationHSV(vpTutoCommonData &data);

/**
 * \brief Extract the skeleton of the segmented image.
 *
 * \param[in] data Data common to the whole program.
 * \return std::vector< VISP_NAMESPACE_ADDRESSING vpImagePoint > List of the noise-free skeletonized image points.
 */
std::vector< VISP_NAMESPACE_ADDRESSING vpImagePoint > extractSkeleton(vpTutoCommonData &data);

/**
 * \brief Add salt and pepper noise to the skeletonized image.
 *
 * \param[in] noisefreePts List of the noise-free skeletonized image points.
 * \param[in] data Data common to the whole program.
 * \return std::vector< VISP_NAMESPACE_ADDRESSING vpImagePoint > List of the noise-free points + additional noisy points.
 */
std::vector< VISP_NAMESPACE_ADDRESSING vpImagePoint > addSaltAndPepperNoise(const std::vector< VISP_NAMESPACE_ADDRESSING vpImagePoint > &noisefreePts, vpTutoCommonData &data);
}
#endif
#endif
