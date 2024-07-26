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

#include "vpTutoSegmentation.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace tutorial
{
void performSegmentationHSV(vpTutoCommonData &data)
{
  const unsigned int height = data.m_I_orig.getHeight(), width = data.m_I_orig.getWidth();
  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);
  vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(data.m_I_orig.bitmap),
                                reinterpret_cast<unsigned char *>(H.bitmap),
                                reinterpret_cast<unsigned char *>(S.bitmap),
                                reinterpret_cast<unsigned char *>(V.bitmap), data.m_I_orig.getSize());

  vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                        reinterpret_cast<unsigned char *>(S.bitmap),
                        reinterpret_cast<unsigned char *>(V.bitmap),
                        data.m_hsv_values,
                        data.m_mask.bitmap,
                        data.m_mask.getSize());

  vpImageTools::inMask(data.m_I_orig, data.m_mask, data.m_I_segmented);
}
}
