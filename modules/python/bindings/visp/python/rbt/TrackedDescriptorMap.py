#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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

from typing import Any, Dict, List, Optional
import numpy as np
import torch
import time

from visp.core import CameraParameters, HomogeneousMatrix
from visp.rbt import RBFeatureTrackerInput, PointMap
from visp.core import Matrix
from visp.core import ArrayInt2D, ImageFloat

class TrackedDescriptorMap():
  """
  A map that associates 3D points in a common frame to a visual descriptive representation (i.e., an XFeat or Sift descriptor).
  The map has a maximum number of points that are stored and least recently added points are replaced first when maximum capacity is reached.
  """
  def __init__(self, num_points: int, reprojection_threshold: float, min_dist_new_point: float, max_depth_error_visible: float):
    """_summary_

    Args:
        num_points (int): Maximum number of points that can be stored in the map.
        reprojection_threshold (float): Reprojection threshold (in pixels) that is used to filter outliers.
        When map points are matched with points in an image,
        if their reprojection is too far from the actually observed point, then they are marked as outliers and removed.
        min_dist_new_point (float): Minimum distance (in meters) that a point should have to all the map points to be considered as a candidate for addition to the map
        max_depth_error_visible (float): Maximum depth (in meters) error that is tolerated for a point to be considered as visible. Used to filter out self occlusion.
    """
    self.descriptors: Optional[torch.tensor] = None
    self.data_for_update = None
    self.indices_removal = None

    self.reprojection_threshold: float = reprojection_threshold
    self.num_points: int = num_points
    self.min_dist_new_point: float = min_dist_new_point
    self.max_depth_error_visible: float = max_depth_error_visible
    self.max_depth_error_candidates: float = 0.0
    self.point_map = PointMap(self.num_points, self.min_dist_new_point, self.max_depth_error_visible, self.max_depth_error_candidates, self.reprojection_threshold)

  def reset(self):
    self.descriptors = None
    self.point_map.clear()

  def parse_settings(self, d: Dict[str, Any]):
    """Update map settings from a dictionary

    Args:
        d (_type_): Dictionary containing the map parameters
    """
    self.reprojection_threshold = d['reprojectionThreshold']
    self.num_points = d['numPoints']
    self.min_dist_new_point = d['minDistNewPoint']
    self.max_depth_error_visible = d['maxDepthErrorVisible']
    self.max_depth_error_candidates = d['maxDepthErrorCandidate']

    self.point_map.setOutlierReprojectionErrorThreshold(self.reprojection_threshold)
    self.point_map.setNumMaxPoints(self.num_points)
    self.point_map.setMinDistanceAddNewPoints(self.min_dist_new_point)
    self.point_map.setMaxDepthErrorVisibilityCriterion(self.max_depth_error_visible)
    self.point_map.setMaxDepthErrorCandidate(self.max_depth_error_candidates)

  def has_points(self) -> bool:
    """Returns whether this map stores any points.

    Returns:
        bool: _description_
    """
    return self.point_map.getPoints().getRows() > 0

  def update(self, frame: RBFeatureTrackerInput, cMo: HomogeneousMatrix, render_depth: ImageFloat, depth_map: ImageFloat, idx_curr_matched: np.ndarray, idx_matched_map: np.ndarray, current_kps: np.ndarray, current_descriptors: torch.Tensor) -> List[int]:
    """ Update the map: Remove outliers and add new points

    Args:
        frame (RBFeatureTrackerInput): Input frame
        cMo (HomogeneousMatrix): Current pose corresponding to the acquired frame
        render_depth (ImageFloat): Depth in render
        depth_map (ImageFloat): Depth observed by the camera (can be zero sized image, in which case it isn't used)
        idx_curr_matched (np.ndarray): Indices of the points points that were matched for the set of current keypoints (in the camera frame).
        idx_matched_map (np.ndarray): Indices of the points that were matched for the set of points from this map. After this method has been called it may no longer be valid.
        current_kps (np.ndarray): N x 2 array that contains the pixel locations of all the keypoints detected in the image. Will be indexed with idx_curr_matched
        current_descriptors (torch.Tensor): N x D tensor containing the descriptors associated to the keypoints observed in the image.

    Returns:
        List[int]: The indices of the points that were removed
    """
    if current_descriptors is None or current_kps is None:
      return

    if idx_curr_matched is not None:
      assert len(idx_curr_matched.shape) == 1
      assert len(idx_matched_map.shape) == 1
      assert len(idx_curr_matched) == len(idx_matched_map)

    indices_outliers = []
    if self.has_points() and idx_curr_matched is not None and idx_matched_map is not None:
      # For matched points, find outliers and mark them for removal
      cam = frame.cam
      if len(idx_matched_map) > 0:
        idx_map = ArrayInt2D.view(idx_matched_map[:, None].astype(np.int32))
        cX, x, uvs = Matrix(), Matrix(), Matrix()
        self.point_map.project(cam, idx_map, cMo, cX, x, uvs)
        indices_outliers = self.point_map.getOutliers(idx_map, uvs, Matrix.view(np.ascontiguousarray(current_kps[idx_curr_matched])))
      self.descriptors[idx_matched_map] = current_descriptors[idx_curr_matched]
      # self.descriptors[idx_matched_map] = self.descriptors[idx_matched_map] * 0.9 + current_descriptors[idx_curr_matched] * 0.1

      self.mark_points_to_remove(indices_outliers)

    num_points = len(current_kps)

    # Get all non matched keypoints
    if idx_curr_matched is None:
      indices_points_not_matched_object = np.arange(num_points, dtype=np.int32)
    else:
      indices_points_not_matched_object = np.ascontiguousarray(np.setdiff1d(np.arange(num_points, dtype=np.int32), idx_curr_matched, assume_unique=True))
    points_to_add_px = np.ascontiguousarray(current_kps[indices_points_not_matched_object], dtype=np.float64)
    if len(indices_points_not_matched_object) == 0:
      indices_array = ArrayInt2D()
    else:
      indices_array = ArrayInt2D.view(indices_points_not_matched_object[..., None])

    oX_new = Matrix()
    cam = frame.cam
    if render_depth is None:
      render_depth = ImageFloat()
    before_candidates = time.time()
    indices_not_matched_to_add = self.point_map.selectValidNewCandidates(cam, cMo, indices_array, Matrix.view(points_to_add_px), render_depth, depth_map, oX_new)

    self.set_points_to_add(oX_new, current_descriptors[indices_not_matched_to_add])
    indices_to_remove = self.finalize_update()

    return indices_to_remove


  def mark_points_to_remove(self, indices):
    self.indices_removal = indices


  def set_points_to_add(self, X: Matrix, descriptors: torch.tensor):
    assert X is not None and descriptors is not None
    assert X.getRows() == descriptors.size(0)
    assert X.getCols() == 3
    self.data_for_update = (X, None, descriptors)

  def finalize_update(self) -> List[int]:
    """Actually remove outliers and add new points to the map.
    This step needs to be done in one go so as to not invalidate any indexing that was done prior to this.

    Returns:
      List[int]: The indices of the points that were removed
    """
    removed_indices = self.indices_removal or []
    if self.indices_removal is None or len(self.indices_removal) == 0:
      outlier_array = ArrayInt2D()
    else:
      outlier_array = ArrayInt2D.view(np.ascontiguousarray(self.indices_removal)[:, None].astype(np.int32))
    removed_indices, num_added_points = self.point_map.updatePoints(outlier_array, self.data_for_update[0])

    if len(removed_indices) > 0:
      if len(removed_indices) == len(self.descriptors):
        self.descriptors = None
      else:
        kept_indices = np.setdiff1d(np.arange(len(self.descriptors)), np.ascontiguousarray(removed_indices), assume_unique=True)
        kept_indices = torch.tensor(kept_indices, device=self.descriptors.device)
        self.descriptors = self.descriptors.index_select(0, kept_indices)
    if self.data_for_update is not None:
      (_, _, descriptors) = self.data_for_update
      if self.descriptors is None:
        self.descriptors = descriptors[:num_added_points]
      else:
        self.descriptors = torch.concatenate((self.descriptors, descriptors[:num_added_points]), dim=0)

    self.data_for_update = None
    self.indices_removal = None

    return removed_indices

  def get_px_proj(self, indices: np.ndarray, cam: CameraParameters, cMo: HomogeneousMatrix) -> Matrix:
    """Helper function to compute the pixel coordinates of a set of map points.

    Args:
        indices (np.ndarray): The indices of the points for which to compute the projection
        cam (CameraParameters): Camera intrinsics
        cMo (HomogeneousMatrix): Camera to map reference pose

    Returns:
        Matrix: a indices.shape[0] x 2 Matrix containing the pixel coordinates of the 3D points in the camera frame
    """
    indices = ArrayInt2D() if len(indices) == 0 else ArrayInt2D(indices[..., None].astype(np.int32))
    cX, xs, uvs = Matrix(), Matrix(), Matrix()
    self.point_map.project(cam, indices, cMo, cX, xs, uvs)
    return uvs
