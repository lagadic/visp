/****************************************************************************
 *
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
 * Tracker based on MegaPose.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_THREADS)

#include <visp3/dnn_tracker/vpMegaPoseTracker.h>
#include <future>

BEGIN_VISP_NAMESPACE
std::future<vpMegaPoseEstimate> vpMegaPoseTracker::init(const vpImage<vpRGBa> &I, const vpRect &bb)
{
  return std::async(std::launch::async, [&I, &bb, this]() -> vpMegaPoseEstimate {
    std::vector<vpRect> bbs = { bb };
    m_poseEstimate = m_megapose->estimatePoses(I, { m_objectLabel }, nullptr, 0.0, &bbs, nullptr)[0];
    m_initialized = true;
    return m_poseEstimate;
  });
}
std::future<vpMegaPoseEstimate> vpMegaPoseTracker::init(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cTo)
{
  return std::async(std::launch::async, [&I, &cTo, this]() -> vpMegaPoseEstimate {
    std::vector<vpHomogeneousMatrix> poses = { cTo };
    m_poseEstimate = m_megapose->estimatePoses(I, { m_objectLabel }, nullptr, 0.0, nullptr, &poses)[0];
    m_initialized = true;
    return m_poseEstimate;
  });
}

std::future<vpMegaPoseEstimate> vpMegaPoseTracker::track(const vpImage<vpRGBa> &I)
{
  if (!m_initialized) {
    throw vpException(vpException::notInitialized, "MegaPose tracker was not initialized. Call init before calling track.");
  }
  return std::async(std::launch::async, [&I, this]() -> vpMegaPoseEstimate {
    std::vector<vpHomogeneousMatrix> poses = { m_poseEstimate.cTo };
    m_poseEstimate = m_megapose->estimatePoses(I, { m_objectLabel }, nullptr, 0.0, nullptr, &poses, m_refinerIterations)[0];
    return m_poseEstimate;
  });
}

void vpMegaPoseTracker::updatePose(const vpHomogeneousMatrix &cTo)
{
  m_poseEstimate.cTo = cTo;
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_dnn_tracker.a(vpMegaPoseTracker.cpp.o) has no symbols
void dummy_vpMegaPoseTracker() { };

#endif
