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
 * Template tracker.
 */
/*!
 \file vpTemplateTrackerSSD.h
 \brief
*/

#ifndef VP_TEMPLATE_TRACKER_SSD_H
#define VP_TEMPLATE_TRACKER_SSD_H

#include <visp3/core/vpConfig.h>         // for BEGIN_VISP_NAMESPACE, END_VI...
#include <visp3/tt/vpTemplateTracker.h>  // for vpTemplateTracker
#include <visp3/core/vpColVector.h>      // for vpColVector
#include <visp3/core/vpRowVector.h>      // for vpRowVector

BEGIN_VISP_NAMESPACE

class vpTemplateTrackerWarp;
template <class Type> class vpImage;

/*!
  \class vpTemplateTrackerSSD
  \ingroup group_tt_tracker
*/
class VISP_EXPORT vpTemplateTrackerSSD : public vpTemplateTracker
{
protected:
  vpRowVector DI;
  vpRowVector temp;

protected:
  double getCost(const vpImage<unsigned char> &I, const vpColVector &tp);
  double getCost(const vpImage<unsigned char> &I) { return getCost(I, p); }
  virtual void initHessienDesired(const vpImage<unsigned char> &I) = 0;
  virtual void trackNoPyr(const vpImage<unsigned char> &I) = 0;

public:
  VP_EXPLICIT vpTemplateTrackerSSD(vpTemplateTrackerWarp *warp);

  double getSSD(const vpImage<unsigned char> &I, const vpColVector &tp);
  void setGain(double g) { gain = g; }
};
END_VISP_NAMESPACE
#endif
