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
  \file vpRBFeatureTrackerInput.h
  \brief Processable frame for render-based tracking. This includes input images as well as renders
*/
#ifndef VP_RB_FEATURE_TRACKER_INPUT_H
#define VP_RB_FEATURE_TRACKER_INPUT_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpRect.h>

#include <visp3/rbt/vpRBSilhouettePoint.h>

BEGIN_VISP_NAMESPACE


/**
 * \brief Render data storage
 *
 * \ingroup group_rbt_core
*/
struct VISP_EXPORT vpRBRenderData
{
  vpImage<vpRGBf> normals; //! Image containing the per-pixel normal vector (RGB, in object space)
  vpImage<float> depth;
  vpImage<vpRGBa> color;
  vpImage<float> silhouetteCanny; //! Image containing the orientation of the gradients
  vpImage<unsigned char> isSilhouette; //! Binary image indicating whether a given pixel is part of the silhouette
  double zNear, zFar; // clipping values
  double objectDiameter; // Object diameter
  //! Center of the 3D bounding box of the object. Expressed in the object frame.
  vpTranslationVector objectCenter;
  vpRect boundingBox;
  vpHomogeneousMatrix cMo; //! Pose of the object in the camera frame for which the renders were generated.

  vpRBRenderData() : zNear(0.0), zFar(0.0), boundingBox() { }

  vpRBRenderData(const vpRBRenderData &other)
  {
    *this = other;
  }

  vpRBRenderData(vpRBRenderData &&other)
  {
    *this = std::move(other);
  }

  vpRBRenderData &operator=(const vpRBRenderData &o)
  {
    normals = o.normals;
    depth = o.depth;
    color = o.color;
    silhouetteCanny = o.silhouetteCanny;
    isSilhouette = o.isSilhouette;
    objectDiameter = o.objectDiameter;
    objectCenter = o.objectCenter;
    zNear = o.zNear;
    zFar = o.zFar;
    boundingBox = o.boundingBox;
    cMo = o.cMo;
    return *this;
  }

  vpRBRenderData &operator=(vpRBRenderData &&o)
  {
    normals = std::move(o.normals);
    depth = std::move(o.depth);
    color = std::move(o.color);
    silhouetteCanny = std::move(o.silhouetteCanny);
    isSilhouette = std::move(o.isSilhouette);
    objectDiameter = std::move(o.objectDiameter);
    objectCenter = std::move(o.objectCenter);
    zNear = std::move(o.zNear);
    zFar = std::move(o.zFar);
    boundingBox = std::move(o.boundingBox);
    cMo = std::move(o.cMo);
    return *this;
  }
};

/**
 * \brief All the data related to a single tracking frame.
 * This contains both the input data (from a real camera/outside source) and renders from Panda.
 *
 * \ingroup group_rbt_core
 */
class VISP_EXPORT vpRBFeatureTrackerInput
{
public:
  vpImage<unsigned char> I; //! Image luminance
  vpImage<vpRGBa> IRGB; //! RGB image, 0 sized if RGB is not available
  vpImage<float> depth; //! depth image, 0 sized if depth is not available
  vpImage<float> mask;
  std::vector<vpRBSilhouettePoint> silhouettePoints;
  vpCameraParameters cam; //! camera parameters
  vpRBRenderData renders;

  bool hasDepth() const { return depth.getSize() > 0; }
  bool hasMask() const { return mask.getSize() > 0; }
  bool hasColorImage() const { return IRGB.getSize() > 0; }

  vpRBFeatureTrackerInput() = default;

  vpRBFeatureTrackerInput &operator=(const vpRBFeatureTrackerInput &o)
  {
    I = o.I;
    IRGB = o.IRGB;
    depth = o.depth;
    mask = o.mask;
    silhouettePoints = o.silhouettePoints;
    cam = o.cam;
    renders = o.renders;
    return *this;
  }

  vpRBFeatureTrackerInput(const vpRBFeatureTrackerInput &other)
  {
    *this = other;
  }

  vpRBFeatureTrackerInput &operator=(vpRBFeatureTrackerInput &&o)
  {
    I = std::move(o.I);
    IRGB = std::move(o.IRGB);
    depth = std::move(o.depth);
    mask = std::move(o.mask);
    silhouettePoints = std::move(o.silhouettePoints);
    cam = std::move(o.cam);
    renders = std::move(o.renders);
    return *this;
  }

  vpRBFeatureTrackerInput(vpRBFeatureTrackerInput &&other)
  {
    *this = std::move(other);
  }
};

END_VISP_NAMESPACE

#endif
