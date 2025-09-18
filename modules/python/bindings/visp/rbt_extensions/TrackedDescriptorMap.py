
import numpy as np
import torch
import time

from visp.core import CameraParameters, HomogeneousMatrix
from visp.rbt import RBFeatureTrackerInput, PointMap
from visp.core import Matrix
from visp.core import ArrayInt2D, ImageFloat

class TrackedDescriptorMap():
  def __init__(self, num_points: int, reprojection_threshold: float, min_dist_new_point: float, max_depth_error_visible: float):

    self.descriptors = None
    self.normals = None
    self.additional_data = None
    self.indices_removal = None

    self.reprojection_threshold = reprojection_threshold
    self.num_points = num_points
    self.min_dist_new_point = min_dist_new_point
    self.max_depth_error_visible = max_depth_error_visible
    self.max_depth_error_candidates = 0.0
    self.point_map = PointMap(self.num_points, self.min_dist_new_point, self.max_depth_error_visible, self.max_depth_error_candidates, self.reprojection_threshold)

  def reset(self):
    self.descriptors = None
    self.point_map.clear()

  def parse_settings(self, d):
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
    return self.point_map.getPoints().getRows() > 0

  def update(self, frame: RBFeatureTrackerInput, cMo: HomogeneousMatrix, render_depth: ImageFloat, depth_map: ImageFloat, idx_curr_matched: np.ndarray, idx_matched_map: np.ndarray, current_kps: np.ndarray, current_descriptors: torch.Tensor):
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
    self.additional_data = (X, None, descriptors)

  def finalize_update(self):
    removed_indices = self.indices_removal or []
    if self.indices_removal is None or len(self.indices_removal) == 0:
      outlier_array = ArrayInt2D()
    else:
      outlier_array = ArrayInt2D.view(np.ascontiguousarray(self.indices_removal)[:, None].astype(np.int32))
    removed_indices, num_added_points = self.point_map.updatePoints(outlier_array, self.additional_data[0])

    if len(removed_indices) > 0:
      if len(removed_indices) == len(self.descriptors):
        self.descriptors = None
      else:
        kept_indices = np.setdiff1d(np.arange(len(self.descriptors)), np.ascontiguousarray(removed_indices), assume_unique=True)
        kept_indices = torch.tensor(kept_indices, device=self.descriptors.device)
        self.descriptors = self.descriptors.index_select(0, kept_indices)
    if self.additional_data is not None:
      (_, _, descriptors) = self.additional_data
      if self.descriptors is None:
        self.descriptors = descriptors[:num_added_points]
      else:
        self.descriptors = torch.concatenate((self.descriptors, descriptors[:num_added_points]), dim=0)

    self.additional_data = None
    self.indices_removal = None

    return removed_indices

  def get_px_proj(self, indices, cam: CameraParameters, cMo: HomogeneousMatrix):
    indices = ArrayInt2D() if len(indices) == 0 else ArrayInt2D(indices[..., None].astype(np.int32))
    cX, xs, uvs = Matrix(), Matrix(), Matrix()
    self.point_map.project(cam, indices, cMo, cX, xs, uvs)
    return uvs
