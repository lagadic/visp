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

#include <visp3/rbt/vpRBFeatureTrackerFactory.h>

#include <visp3/rbt/vpRBKltTracker.h>
#include <visp3/rbt/vpRBSilhouetteCCDTracker.h>
#include <visp3/rbt/vpRBSilhouetteMeTracker.h>
#include <visp3/rbt/vpRBDenseDepthTracker.h>

BEGIN_VISP_NAMESPACE

template class VISP_EXPORT vpDynamicFactory<vpRBFeatureTracker>;

vpRBFeatureTrackerFactory::vpRBFeatureTrackerFactory()
{
#ifdef VISP_HAVE_NLOHMANN_JSON
  setJsonKeyFinder([](const nlohmann::json &j) -> std::string {
    return j.at("type");
  });

  registerType("silhouetteMe", [](const nlohmann::json &j) {
    std::shared_ptr<vpRBSilhouetteMeTracker> p(new vpRBSilhouetteMeTracker());
    p->loadJsonConfiguration(j);
    return p;
  });
  registerType("silhouetteColor", [](const nlohmann::json &j) {
    std::shared_ptr<vpRBSilhouetteCCDTracker> p(new vpRBSilhouetteCCDTracker());
    p->loadJsonConfiguration(j);
    return p;
  });
  registerType("depth", [](const nlohmann::json &j) {
    std::shared_ptr<vpRBDenseDepthTracker> p(new vpRBDenseDepthTracker());
    p->loadJsonConfiguration(j);
    return p;
  });
#if defined(VP_HAVE_RB_KLT_TRACKER)
  registerType("klt", [](const nlohmann::json &j) {
    std::shared_ptr<vpRBKltTracker> p(new vpRBKltTracker());
    p->loadJsonConfiguration(j);
    return p;
  });
#endif
#endif
}

END_VISP_NAMESPACE
