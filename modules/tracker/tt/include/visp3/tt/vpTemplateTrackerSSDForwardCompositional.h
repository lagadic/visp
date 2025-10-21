/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 */
/*!
 \file vpTemplateTrackerSSDForwardCompositional.h
 \brief
*/

#ifndef vpTemplateTrackerSSDForwardCompositional_hh
#define vpTemplateTrackerSSDForwardCompositional_hh

#include <visp3/core/vpConfig.h>
#include <visp3/tt/vpTemplateTrackerSSD.h>

BEGIN_VISP_NAMESPACE
/*!
  \ingroup group_tt_tracker
  The algorithm implemented in this class is described in \cite Baker04a and
  \cite Marchand16a.

  <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>

  <b>Tutorials</b><br>
  <span style="margin-left:2em"> If you are interested in the Template Tracker(TT), you may have a look at:</span><br>

  - \ref tutorial-tracking-tt
*/
class VISP_EXPORT vpTemplateTrackerSSDForwardCompositional : public vpTemplateTrackerSSD
{
protected:
  bool compoInitialised;

protected:
  void initHessienDesired(const vpImage<unsigned char> &I);
  void initCompo(const vpImage<unsigned char> &I);
  void trackNoPyr(const vpImage<unsigned char> &I);

public:
  VP_EXPLICIT vpTemplateTrackerSSDForwardCompositional(vpTemplateTrackerWarp *warp);
};
END_VISP_NAMESPACE
#endif
