/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTrackerZNCCInverseCompositional.h
 \brief
*/

#ifndef vpTemplateTrackerZNCCInverseCompositional_hh
#define vpTemplateTrackerZNCCInverseCompositional_hh

#include <vector>

#include <visp3/tt/vpTemplateTrackerZNCC.h>

/*!
  \class vpTemplateTrackerZNCCInverseCompositional
  \ingroup group_tt_tracker
  The algorithm implemented in this class is described in \cite Irani98a and
  \cite Marchand16a.
 */
class VISP_EXPORT vpTemplateTrackerZNCCInverseCompositional : public vpTemplateTrackerZNCC
{
protected:
  bool compoInitialised;
  // pour eval evolRMS
  double evolRMS;
  std::vector<double> x_pos;
  std::vector<double> y_pos;
  double threshold_RMS;
  vpColVector moydIrefdp;

protected:
  void initCompInverse(const vpImage<unsigned char> &I);
  void initHessienDesired(const vpImage<unsigned char> &I);
  void trackNoPyr(const vpImage<unsigned char> &I);
  void deletePosEvalRMS();
  void computeEvalRMS(const vpColVector &p);
  void initPosEvalRMS(const vpColVector &p);

public:
  explicit vpTemplateTrackerZNCCInverseCompositional(vpTemplateTrackerWarp *warp);

  void setThresholdRMS(double threshold) { threshold_RMS = threshold; }
};
#endif
