#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2025 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Display helpers for ViSP
#
#############################################################################
import numpy as np
from enum import Enum
import torch

from visp.core import ColVector, Matrix, ArrayInt2D
from visp.core import CameraParameters, HomogeneousMatrix, PixelMeterConversion, Robust
from visp.core import ImageGray, ImageRGBa
from visp.core import Color
from visp.core import Display

from visp.rbt import RBFeatureTracker, RBFeatureTrackerInput, RBFeatureTrackerFactory
from visp.rbt import TemporalWeighting

from visp.python.rbt import TrackedDescriptorMap
from visp.python.rbt.xfeat import XFeatTrackingBackend

class RBXFeatFeatureTracker(RBFeatureTracker):
  """A trackable feature implementation for the RBT that relies on XFeat for keypoint extraction.
  XFeat is used to extract keypoints (2D) along with their descriptors in every frame.
  At the end of every tracking iteration, Novel keypoints are added in a map (see visp.python.rbt.TrackedDescriptorMap)
  Using the 2D keypoint location, the newly computed 3D camera pose and the render information

  This feature relies on the XFeat tracker backend,
  which should be customized to set the number of keypoints to retrieve each frame along with the minimum similarity score for the matching step.

  The error that is computed is either the reprojection error (2D, in normalized image plane coordinates, similar to an IBVS)
  or in 3D (in metric space) if the option is chosen and an RGB-D camera is used.

  An example JSON configuration of the tracker is:
  {
    "type": "xfeat",
    "weight": 1,
    "use_3d": false,
    "display": true,
    "numPoints": 4096,
    "reprojectionThreshold": 5,
    "minDistNewPoint": 0.0,
    "maxDepthErrorVisible": 0.02,
    "maxDepthErrorCandidate": 0.02
  }

  See the TrackedDescriptorMap class for some of the settings description.

  """

  class DisplayType(Enum):
    SIMPLE = 'simple'
    SIMPLE_MODEL_AND_PROJ = 'simpleModelAndProj'
    ERROR = 'error'
    WEIGHT_AND_ERROR = 'weightAndError'


  def __init__(self, backend: XFeatTrackingBackend):
    RBFeatureTracker.__init__(self)

    self.backend = backend
    self.object_map = TrackedDescriptorMap(num_points=2048,
                                reprojection_threshold=5.0,
                                min_dist_new_point=1e-4,
                                max_depth_error_visible=1e-2)
    self.idx_curr_obj_matched, self.idx_object_map_matched = None, None
    self.iter = 0
    self.use_3d = False
    self.last_cMo = None
    self.robust = Robust()
    self.current_representation = None
    self.display_type = RBXFeatFeatureTracker.DisplayType.SIMPLE

  def requiresRGB(self) -> bool:
    return True
  def requiresDepth(self) -> bool:
    return self.use_3d
  def requiresSilhouetteCandidates(self) -> bool:
    return True

  def load_settings(self, dict_rep):
    import json
    self.setTrackerWeight(TemporalWeighting.parseTemporalWeightingRawJson(json.dumps(dict_rep['weight'])))
    self.use_3d = dict_rep.get('use3d', self.use_3d)
    self.setFeaturesShouldBeDisplayed(dict_rep.get('display', False))
    self.object_map.parse_settings(dict_rep)

  def onTrackingIterStart(self, frame: RBFeatureTrackerInput, cMo: HomogeneousMatrix):
    self.cov.resize(6, 6)
    self.LTL.resize(6, 6)
    self.LTR.resize(6)
    self.numFeatures = 0


  def extractFeatures(self, frame: RBFeatureTrackerInput, previousFrame: RBFeatureTrackerInput, cMo: HomogeneousMatrix):
    self.frame = frame
    self.backend.process_frame(frame, self.iter)

  def trackFeatures(self, frame: RBFeatureTrackerInput, previousFrame: RBFeatureTrackerInput, cMo: HomogeneousMatrix):


    self.current_representation = self.backend.get_current_object_data()
    self.idx_curr_obj_matched, self.idx_object_map_matched = None, None

    with torch.no_grad():
      if self.object_map.has_points() and self.current_representation is not None:
        h, w = frame.I.getRows(), frame.I.getCols()
        depth_map = frame.renders.depth
        import time
        visible_indices = np.ascontiguousarray(self.object_map.point_map.getVisiblePoints(h, w, frame.cam, frame.renders.cMo, depth_map))
        if len(visible_indices) > 0:
          visible_object_descriptors = self.object_map.descriptors[visible_indices]
          self.idx_curr_obj_matched, self.idx_object_map_matched = self.backend.match(self.current_representation.descriptors, visible_object_descriptors)
          self.idx_curr_obj_matched = self.idx_curr_obj_matched.cpu().numpy()
          self.idx_object_map_matched = visible_indices[self.idx_object_map_matched.cpu().numpy()]

    self.numFeatures = 0
    if self.idx_curr_obj_matched is not None:

      # If we're using 3D points, retain only those with a valid depth
      if self.use_3d:
        valid_indices = []
        self.Zs = []
        for i, kp in enumerate(self.current_representation.keypoints[self.idx_curr_obj_matched]):
          Z = frame.depth[int(kp[1]), int(kp[0])]
          if Z > 0.0:
            valid_indices.append(i)
            self.Zs.append(Z)
        self.Zs = np.ascontiguousarray(self.Zs)
        self.idx_curr_obj_matched = self.idx_curr_obj_matched[valid_indices]
        self.idx_object_map_matched = self.idx_object_map_matched[valid_indices]

      self.numFeatures = len(self.idx_curr_obj_matched) * (2 if not self.use_3d else 3)


  def initVVS(self, frame: RBFeatureTrackerInput, _previousFrame: RBFeatureTrackerInput, _cMo: HomogeneousMatrix):
    self.L.resize(self.numFeatures, 6, False, False)
    self.error.resize(self.numFeatures, False)
    self.weighted_error.resize(self.numFeatures, False)
    self.weights.resize(self.numFeatures, False)
    self.covWeightDiag.resize(self.numFeatures, False)

    if self.idx_object_map_matched is not None:
      self.visp_indices_matched = ArrayInt2D.view(self.idx_object_map_matched[..., None].astype(np.int32))
      current_px_matched = self.current_representation.keypoints[self.idx_curr_obj_matched]
      xoc, yoc = PixelMeterConversion.convertPoints(frame.cam, current_px_matched[:, 0], current_px_matched[:, 1])
      if not self.use_3d:
        self.current_xy_obj = Matrix.view(np.ascontiguousarray(np.concatenate((xoc[..., None], yoc[..., None]), axis=-1), dtype=np.float64))
      else:
        xyz = np.concatenate([arr[:, None] for arr in (xoc * self.Zs, yoc * self.Zs, self.Zs)], axis=-1, dtype=np.float64)
        self.observations = Matrix.view(xyz)
    else:
      self.visp_indices_matched = ArrayInt2D()

    self.error_per_point = ColVector(self.numFeatures // (2 if not self.use_3d else 3))
    self.weight_per_point = ColVector(self.error_per_point.getCols())

  def computeVVSIter(self, frame: RBFeatureTrackerInput, cMo: HomogeneousMatrix, iteration: int):
    if self.numFeatures < 8 * (3 if self.use_3d else 2):
      self.numFeatures = 0
      return
    # Error and Jacobian computation
    if not self.use_3d:
      self.object_map.point_map.computeReprojectionErrorAndJacobian(self.visp_indices_matched, cMo, self.current_xy_obj, self.L, self.error)
    else:
      self.object_map.point_map.compute3DErrorAndJacobian(self.visp_indices_matched, cMo, self.observations, self.L, self.error)

    # Compute Min MAD for M-estimator
    threshold = 0.0
    if not self.use_3d:
      # Compute min threshold as relative to the bounding box (object size in the iamge)
      bb = frame.renders.boundingBox
      rect_diagonal = np.sqrt(bb.getHeight() ** 2 + bb.getWidth() ** 2)
      cam = frame.cam
      threshold = (rect_diagonal / np.sqrt(cam.get_px() ** 2 + cam.get_py() ** 2)) * 0.001
    else:
      diff = frame.renders.zFar - frame.renders.zNear
      threshold = diff * 0.001

    # M-estimator weighting performed relative to the euclidean distance between points, not the individual component error
    self.robust.setMinMedianAbsoluteDeviation(threshold)
    error_per_point = np.linalg.norm(self.error.numpy().reshape((-1, 2 if not self.use_3d else 3)), axis=-1)
    self.error_per_point[:] = error_per_point

    self.robust.MEstimator(Robust.TUKEY, self.error_per_point, self.weight_per_point)

    # Compute optimizer quantities
    self.weights[:] = np.repeat(self.weight_per_point, 2 if not self.use_3d else 3)
    self.updateOptimizerTerms(cMo)

  def onTrackingIterEnd(self, cMo: HomogeneousMatrix):

    # Update displayed points before removing them from map
    if self.idx_curr_obj_matched is not None and self.numFeatures > 0 and self.idx_object_map_matched is not None:
      cX, xs, uvs = Matrix(), Matrix(), Matrix()
      map_indices = ArrayInt2D.view(np.ascontiguousarray(self.idx_object_map_matched[:, None]).astype(np.int32))
      self.object_map.point_map.project(self.frame.cam, map_indices, cMo, cX, xs, uvs)
      self.pixel_pos_visible = uvs.numpy().copy()

    # Update map
    if self.current_representation is not None:
      self.removed_indices = self.object_map.update(self.frame, cMo,
                                                    self.frame.renders.depth,
                                                    self.frame.depth,
                                                    self.idx_curr_obj_matched,
                                                    self.idx_object_map_matched,
                                                    self.current_representation.keypoints,
                                                    self.current_representation.descriptors)
    self.frame = None
    self.iter += 1

  def reset(self):
    self.object_map.reset()

  def display(self, cam: CameraParameters, I: ImageGray, IRGB: ImageRGBa, I_depth: ImageGray):
    if self.idx_curr_obj_matched is None or self.numFeatures == 0:
      return
    ps = self.current_representation.keypoints[self.idx_curr_obj_matched]
    uvs = self.pixel_pos_visible
    error = np.linalg.norm(uvs - ps, axis=1)

    max_error_display = np.maximum(np.max(error), self.object_map.point_map.getOutlierReprojectionErrorThreshold())
    ps = np.rint(ps).astype(np.int32)
    uvs = np.rint(uvs).astype(np.int32)
    if self.display_type == RBXFeatFeatureTracker.DisplayType.SIMPLE:
      Display.displayCrosses(IRGB, ps[:, 1], ps[:, 0], 8, Color.green, 1)
    elif self.display_type == RBXFeatFeatureTracker.DisplayType.SIMPLE_MODEL_AND_PROJ:
      Display.displayCrosses(IRGB, ps[:, 1], ps[:, 0], 8, Color.green, 1)
      Display.displayCrosses(IRGB, uvs[:, 1], np.rint(uvs[:, 0]).astype(np.int32), 8, Color.red, 1)

    elif self.display_type == RBXFeatFeatureTracker.DisplayType.WEIGHT_AND_ERROR:
      weights = (np.sum(self.weights.numpy().reshape((len(error), 2)), axis=-1) / 2.0)
      c = Color()
      for p in range(len(error)):
        c.setColor(
          np.rint((error[p] / max_error_display) * 255).astype(np.uint8),
          np.rint(weights[p] * 255.0).astype(np.uint8),
          0, 255
        )
        Display.displayCircleStatic(IRGB, ps[p, 1], ps[p, 0], 2, c, True, 1)
    else:
      raise RuntimeError('Display time not implemented')

# def create_instance():
#   return XFeatFeatureTracker(None)

# TrackerPyBase.register_type('xfeat', create_instance)
