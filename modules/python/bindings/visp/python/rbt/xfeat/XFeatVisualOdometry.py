import numpy as np
from enum import Enum
import torch

from visp.core import Matrix
from visp.core import CameraParameters, HomogeneousMatrix, PixelMeterConversion, ArrayInt2D
from visp.core import Color, Display
from visp.core import ImageGray, ImageRGBa
from visp.rbt import RBFeatureTrackerInput
from visp.rbt import RBVisualOdometry, RBVisualOdometryUtils, LevenbergMarquardtParameters

from visp.python.rbt import TrackedDescriptorMap
from visp.python.rbt.xfeat import XFeatTrackingBackend


class XFeatVisualOdometry(RBVisualOdometry):

  class DisplayType(Enum):
    SIMPLE = 'simple'
    SIMPLE_MODEL_AND_PROJ = 'simpleModelAndProj'
    ERROR = 'error'
    WEIGHT_AND_ERROR = 'weightAndError'

  def __init__(self, backend: XFeatTrackingBackend):
    RBVisualOdometry.__init__(self)
    self.backend = backend
    self.environment_map = TrackedDescriptorMap(num_points=1024,
                                      reprojection_threshold=10.0,
                                      min_dist_new_point=0,
                                      max_depth_error_visible=2e-2)

    self.display_type = XFeatVisualOdometry.DisplayType.SIMPLE
    self.use_3d = False
    self.current_representation = None
    # XFeat params
    self.idx_curr_env_matched, self.idx_environment_map = None, None
    self.iter = 0

    # Odometry parameters
    self.cTw = HomogeneousMatrix()
    self.cnTc = None

    # Initial optimization pass: Minimize reprojection error, wrt to cTw
    self.optim_params = LevenbergMarquardtParameters()
    self.optim_params.gain = 0.5
    self.optim_params.maxNumIters = 10
    self.optim_params.muInit = 0.1
    self.optim_params.muIterFactor = 0.5
    self.optim_params.minImprovementFactor = 1e-3

    self.verbose = backend.verbose

  def load_settings(self, d):
    self.environment_map.parse_settings(d)
    self.use_3d = d.get('use3d', self.use_3d)
    self.optim_params.gain = d['gain']
    self.optim_params.maxNumIters = d['maxNumIters']
    self.optim_params.muInit = d['muInit']
    self.optim_params.muIterFactor = d['muIterFactor']
    self.optim_params.minImprovementFactor = d['minImprovementFactor']


  def compute(self, frame: RBFeatureTrackerInput, previousFrame: RBFeatureTrackerInput) -> None:

    if self.current_representation is not None:
      removed_indices = self.environment_map.update(frame, self.cTw, None, frame.depth, self.idx_curr_env_matched, self.idx_environment_map,
                                                     self.current_representation.keypoints, self.current_representation.descriptors)


    self.backend.process_frame(frame, self.iter)
    self.current_representation = self.backend.get_current_environment_data()


    self.idx_curr_env_matched, self.idx_environment_map = None, None
    with torch.no_grad():
      if self.environment_map.has_points() and self.current_representation is not None:
        h, w = frame.I.getRows(), frame.I.getCols()

        visible_indices = np.ascontiguousarray(self.environment_map.point_map.getVisiblePoints(h, w, frame.cam, self.cTw, frame.depth))
        # visible_indices = self.environment_map.get_visible_points(self.cTw, frame)
        if len(visible_indices) > 0:
          visible_env_descriptors = self.environment_map.descriptors[visible_indices]
          self.idx_curr_env_matched, self.idx_environment_map = self.backend.match(self.current_representation, visible_env_descriptors)
          self.idx_curr_env_matched = self.idx_curr_env_matched.cpu().numpy()
          self.idx_environment_map = visible_indices[self.idx_environment_map.cpu().numpy()]


    self.numFeatures = 0
    self.iter += 1

    if self.idx_curr_env_matched is not None:

      if self.use_3d:
        valid_indices = []
        self.Zs = []
        for i, kp in enumerate(self.current_representation.keypoints[self.idx_curr_env_matched]):
          Z = frame.depth[int(kp[1]), int(kp[0])]
          if Z > 0.0:
            valid_indices.append(i)
            self.Zs.append(Z)
        self.Zs = np.ascontiguousarray(self.Zs)
        self.idx_curr_env_matched = self.idx_curr_env_matched[valid_indices]
        self.idx_environment_map = self.idx_environment_map[valid_indices]

      self.numFeatures = len(self.idx_curr_env_matched) * (2 if not self.use_3d else 3)


    self.optimize(frame)


  def optimize(self, frame: RBFeatureTrackerInput):
    self.previous_cTw = HomogeneousMatrix(self.cTw) # Calling constructor is important, we want to keep the value, not the reference as current cTw will be modified!

    if self.numFeatures == 0:
      self.cnTc = HomogeneousMatrix()
      return

    if self.idx_curr_env_matched is None or len(self.idx_curr_env_matched) == 0:
      self.cnTc = HomogeneousMatrix()
      return

    current_px_matched_env = self.current_representation.keypoints[self.idx_curr_env_matched]
    xoe, yoe = PixelMeterConversion.convertPoints(frame.cam, current_px_matched_env[:, 0], current_px_matched_env[:, 1])
    mapX = Matrix()
    indices = ArrayInt2D.view(np.ascontiguousarray(self.idx_environment_map[:, None].astype(np.int32)))
    self.environment_map.point_map.getPoints(indices, mapX)
    c = HomogeneousMatrix(self.cTw)
    if self.use_3d:
      current_xyz_env = Matrix.view(np.stack((xoe * self.Zs, yoe * self.Zs, self.Zs), axis=-1))
      RBVisualOdometryUtils.levenbergMarquardtKeypoints3D(mapX, current_xyz_env, self.optim_params, c)
    else:
      current_xy_env = Matrix.view(np.stack((xoe, yoe), axis=-1))
      RBVisualOdometryUtils.levenbergMarquardtKeypoints2D(mapX, current_xy_env, self.optim_params, c)

    self.cTw = c
    self.cnTc = self.cTw * self.previous_cTw.inverse()


  def reset(self):
    self.environment_map.reset()
    self.current_representation = None
    self.previous_cTw = HomogeneousMatrix()
    self.cTw = HomogeneousMatrix()
    self.cnTc = HomogeneousMatrix()
    self.idx_curr_env_matched = None
    self.idx_environment_map = None

  def getCameraMotion(self) -> HomogeneousMatrix:
    return self.cnTc

  def getCameraPose(self) -> HomogeneousMatrix:
    return self.cTw

  def display(self, cam: CameraParameters, I: ImageGray, IRGB: ImageRGBa, I_depth: ImageGray):
    if self.idx_curr_env_matched is None or self.idx_environment_map is None:
      return

    ps = self.current_representation.keypoints[self.idx_curr_env_matched]
    cX, xs, uvs = Matrix(), Matrix(), Matrix()
    self.environment_map.point_map.project(cam, ArrayInt2D.view(np.ascontiguousarray(self.idx_environment_map[:, None]).astype(np.int32)), self.cTw, cX, xs, uvs)
    uvs = uvs.numpy().copy()
    error = np.linalg.norm(uvs - ps, axis=1)
    threshold = self.environment_map.point_map.getOutlierReprojectionErrorThreshold()
    if len(error) == 0:
      return
    max_error_display = np.maximum(np.max(error), self.environment_map.point_map.getOutlierReprojectionErrorThreshold())
    ps = np.rint(ps).astype(np.int32)
    uvs = np.rint(uvs).astype(np.int32)

    if self.display_type == XFeatVisualOdometry.DisplayType.SIMPLE:
      Display.displayCrosses(IRGB, ps[:, 1], ps[:, 0], 8, Color.green, 1)

    elif self.display_type == XFeatVisualOdometry.DisplayType.SIMPLE_MODEL_AND_PROJ:
      Display.displayCrosses(IRGB, ps[:, 1], ps[:, 0], 8, Color.blue, 1)
      Display.displayCrosses(IRGB, uvs[:, 1], np.rint(uvs[:, 0]).astype(np.int32), 8, Color.red, 1)

    elif self.display_type == XFeatVisualOdometry.DisplayType.WEIGHT_AND_ERROR:
      c = Color()
      for p in range(len(error)):
        e = np.minimum(error[p], threshold)  / threshold
        c.setColor(
          np.rint((e) * 255).astype(np.uint8),
          0,
          np.rint((1.0 - e) * 255).astype(np.uint8),
          255
        )

        Display.displayCircleStatic(IRGB, ps[p, 1], ps[p, 0], 2, c, True, 1)
    else:
      raise RuntimeError('Display type not implemented')
