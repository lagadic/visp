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

#include <visp3/rbt/vpObjectCentricRenderer.h>

#include <visp3/core/vpRect.h>

#include "boundingSphere.h"
#include "boundingBox.h"
#include "graphicsOutput.h"
#include "graphicsEngine.h"
#include "windowFramework.h"
#include "load_prc_file.h"

BEGIN_VISP_NAMESPACE

vpObjectCentricRenderer::vpObjectCentricRenderer(const vpPanda3DRenderParameters &renderParameters)
  : vpPanda3DRendererSet(renderParameters), m_enableCrop(true), m_shouldComputeBBPoints(true)
{
  m_renderParameters = renderParameters;
}

void vpObjectCentricRenderer::beforeFrameRendered()
{
  if (m_shouldComputeBBPoints) {
    computeBoundingBox3DPoints();
    m_shouldComputeBBPoints = false;
  }
  m_bb = computeBoundingBox();

  double delta = 0.0;
  m_bb.setTop(std::max(m_bb.getTop() - delta, 0.0));
  m_bb.setLeft(std::max(m_bb.getLeft() - delta, 0.0));
  m_bb.setBottom(std::min(m_bb.getBottom() + delta, (double)m_renderParameters.getImageHeight()));
  m_bb.setRight(std::min(m_bb.getRight() + delta, (double)m_renderParameters.getImageWidth()));

  if (m_enableCrop) {
    vpPanda3DRenderParameters subParams = m_renderParameters;

    unsigned width = static_cast<unsigned int>(m_bb.getWidth());
    unsigned height = static_cast<unsigned int>(m_bb.getHeight());
    subParams.setImageResolution(height, width);
    subParams.setClippingDistance(subParams.getNearClippingDistance(), subParams.getFarClippingDistance());
    const vpCameraParameters cam = subParams.getCameraIntrinsics();
    subParams.setCameraIntrinsics(vpCameraParameters(cam.get_px(), cam.get_py(), cam.get_u0() - m_bb.getLeft(), cam.get_v0() - m_bb.getTop()));
    for (std::shared_ptr<vpPanda3DBaseRenderer> &subrenderer : m_subRenderers) {
      subrenderer->setRenderParameters(subParams);
    }
  }
}

void vpObjectCentricRenderer::computeBoundingBox3DPoints()
{
  if (m_subRenderers.size() == 0) {
    throw vpException(vpException::fatalError, "Cannot compute bounding box with no subrender");
  }
  std::shared_ptr<vpPanda3DBaseRenderer> subrenderer = m_subRenderers[0];
  NodePath object = subrenderer->getRenderRoot().find(m_focusedObject);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Focused node %s was not found", m_focusedObject.c_str());
  }
  m_bb3DPoints.clear();
  LPoint3 minP, maxP;
  object.calc_tight_bounds(minP, maxP);
  const BoundingBox box(minP, maxP);

  for (unsigned int i = 0; i < 8; ++i) {
    const LPoint3 p = box.get_point(i);
    m_bb3DPoints.push_back(vpColVector({ p.get_x(), -p.get_z(), p.get_y(), 1.0 }));
  }
}

void vpObjectCentricRenderer::computeClipping(float &nearV, float &farV)
{
  if (m_subRenderers.size() == 0) {
    throw vpException(vpException::fatalError, "Cannot compute clpping distances with no subrenderer");
  }
  std::shared_ptr<vpPanda3DBaseRenderer> subrenderer = m_subRenderers[0];
  NodePath object = subrenderer->getRenderRoot().find(m_focusedObject);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Node %s was not found", m_focusedObject.c_str());
  }
  if (m_shouldComputeBBPoints) {
    computeBoundingBox3DPoints();
    m_shouldComputeBBPoints = false;
  }
  const vpHomogeneousMatrix wTcam = getCameraPose();
  const vpHomogeneousMatrix wTobj = getNodePose(m_focusedObject) * vpPanda3DBaseRenderer::PANDA_T_VISP;
  const vpHomogeneousMatrix camTobj = wTcam.inverse() * wTobj;
  float minZ = std::numeric_limits<float>::max(), maxZ = 0.f;
  for (unsigned int i = 0; i < m_bb3DPoints.size(); ++i) {
    vpColVector cpV = camTobj * m_bb3DPoints[i];
    cpV /= cpV[3];
    float Z = cpV[2];
    if (Z > maxZ) {
      maxZ = Z;
    }
    if (Z < minZ) {
      minZ = Z;
    }
  }
  nearV = minZ;
  farV = maxZ;
}

vpRect vpObjectCentricRenderer::computeBoundingBox()
{
  if (m_subRenderers.size() == 0) {
    throw vpException(vpException::fatalError, "Cannot compute bounding box with no subrenderer");
  }
  std::shared_ptr<vpPanda3DBaseRenderer> subrenderer = m_subRenderers[0];
  NodePath object = subrenderer->getRenderRoot().find(m_focusedObject);
  if (object.is_empty()) {
    throw vpException(vpException::badValue, "Node %s was not found", m_focusedObject.c_str());
  }
  if (m_shouldComputeBBPoints) {
    computeBoundingBox3DPoints();
    m_shouldComputeBBPoints = false;
  }
  const auto pointToPixel = [this](const vpHomogeneousMatrix &camTobj, const vpColVector &point) -> vpImagePoint {
    vpColVector cpV = camTobj * point;
    cpV /= cpV[3];
    double x = cpV[0] / cpV[2];
    double y = cpV[1] / cpV[2];
    vpImagePoint ip;
    vpMeterPixelConversion::convertPoint(m_renderParameters.getCameraIntrinsics(), x, y, ip);
    ip.set_j(vpMath::clamp(ip.get_j(), 0.0, m_renderParameters.getImageWidth() - 1.0));
    ip.set_i(vpMath::clamp(ip.get_i(), 0.0, m_renderParameters.getImageHeight() - 1.0));
    return ip;
    };

  const vpHomogeneousMatrix wTcam = getCameraPose();
  const vpHomogeneousMatrix wTobj = getNodePose(m_focusedObject) * vpPanda3DBaseRenderer::PANDA_T_VISP;
  const vpHomogeneousMatrix camTobj = wTcam.inverse() * wTobj;

  double minu = m_renderParameters.getImageWidth(), maxu = 0.0, minv = m_renderParameters.getImageHeight(), maxv = 0.0;
  for (unsigned int i = 0; i < m_bb3DPoints.size(); ++i) {
    const vpImagePoint ip = pointToPixel(camTobj, m_bb3DPoints[i]);
    double u = ip.get_u(), v = ip.get_v();
    if (u < minu) {
      minu = u;
    }
    if (u > maxu) {
      maxu = u;
    }
    if (v < minv) {
      minv = v;
    }
    if (v > maxv) {
      maxv = v;
    }
  }
  return vpRect(vpImagePoint(minv, minu), vpImagePoint(maxv, maxu));
}

END_VISP_NAMESPACE
