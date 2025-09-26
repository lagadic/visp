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
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple
import numpy as np
import time

import torch

from visp.core import ImageRGBa, CameraParameters, Point, ImageRGBf, PoseVector
from visp.core import PixelMeterConversion, HomogeneousMatrix, MeterPixelConversion
from visp.rbt import RBFeatureTrackerInput
from visp.vision import Pose
from visp.rbt import RBVisualOdometryUtils, LevenbergMarquardtParameters

from visp.python.vision.xfeat import XFeatBackend
from visp.python.display_utils import get_display

class XFeatInitializer():
  def __init__(self, backend: XFeatBackend, save_folder: Path):
    self.xfeat_backend = backend
    self.points_3d = None
    self.descriptors = None
    self.save_folder = save_folder
    self.path_pts_3d = save_folder / 'xfeat_init_points3d.npy'
    self.path_descriptors = save_folder / 'xfeat_init_descriptors.npy'
    if self.path_pts_3d.exists() and self.path_descriptors.exists():
      print('Loading keypoints database...')
      self.points_3d = np.load(self.path_pts_3d)
      self.descriptors = torch.tensor(np.load(self.path_descriptors)).cuda()

    self.pose_estimator = Pose()
    maxNbTrials = 500
    threshold = 0.0001
    numberOfInlierToReachAConsensus = 4
    self.pose_estimator.setRansacFilterFlag(Pose.RANSAC_FILTER_FLAGS.PREFILTER_DEGENERATE_POINTS)
    self.pose_estimator.setRansacMaxTrials(maxNbTrials)
    self.pose_estimator.setRansacNbInliersToReachConsensus(numberOfInlierToReachAConsensus)
    self.pose_estimator.setRansacThreshold(threshold)


  def record(self, frame: RBFeatureTrackerInput, cTo: HomogeneousMatrix):
    kps, descriptors = self.xfeat_backend.get_keypoints(frame.IRGB)
    us, vs = kps[:, 0], kps[:, 1]
    us, vs = np.rint(us).astype(np.int32), np.rint(vs).astype(np.int32)
    is_object = frame.renders.depth.numpy()[vs, us] > 0.0
    us, vs, descriptors = us[is_object], vs[is_object], descriptors[is_object]
    Z = frame.renders.depth.numpy()[vs, us]
    xs, ys = PixelMeterConversion.convertPoints(frame.cam, us, vs)
    points_3d_cam = np.empty((len(xs), 4))
    points_3d_cam[:, 0] = xs * Z
    points_3d_cam[:, 1] = ys * Z
    points_3d_cam[:, 2] = Z
    points_3d_cam[:, 3] = 1.0

    points_3d_object = cTo.inverse().numpy() @ points_3d_cam.T
    points_3d_object = points_3d_object.T

    if self.points_3d is None:
      self.points_3d = points_3d_object
      self.descriptors = descriptors
    else:
      self.points_3d = np.concatenate((self.points_3d, points_3d_object), axis=0)
      self.descriptors = torch.concat((self.descriptors, descriptors), dim=0)

  def estimate_pose(self, image: ImageRGBa, cam: CameraParameters) -> HomogeneousMatrix:
    if self.points_3d is None:
      raise RuntimeError('No 3D points were learned before trying to compute pose')
    t1 = time.time()
    kps, descriptors = self.xfeat_backend.get_keypoints(image)
    t2 = time.time()
    print(len(self.descriptors))
    idx_curr_matches, idx_stored_matches = self.xfeat_backend.match(descriptors, self.descriptors, min_cos=0.0)
    idx_curr_matches = idx_curr_matches.cpu().numpy()
    idx_stored_matches = idx_stored_matches.cpu().numpy()
    print(f'Matching took: {(time.time()  - t2) * 1000}')
    pos_2d = kps[idx_curr_matches]
    print(pos_2d)
    xs, ys = PixelMeterConversion.convertPoints(cam, pos_2d[:, 0], pos_2d[:, 1])

    pos_3d = self.points_3d[idx_stored_matches]
    self.pose_estimator.clearPoint()
    for x, y, X in zip(xs, ys, pos_3d):
      p = Point()
      p.set_x(x)
      p.set_y(y)
      p.set_oX(X[0])
      p.set_oY(X[1])
      p.set_oZ(X[2])
      p.set_oW(X[3])

      self.pose_estimator.addPoint(p)
    t3 = time.time()
    cTo = HomogeneousMatrix()
    pose_ok = self.pose_estimator.computePose(Pose.RANSAC, cTo)
    t4 = time.time()
    print('extraction took: ', (t2 - t1) * 1000)
    print('Problem formulation took: ', (t3 - t2) * 1000)
    print('Resolution took: ', (t4 - t3) * 1000)
    # pose_estimator.computePoseDementhonLagrangeVVS(cTo)
    # pose_estimator.computePose(Pose.DEMENTHON_LAGRANGE_VIRTUAL_VS, cTo)
    return cTo, pose_ok

  def save(self):
    print('Saving keypoints database...')
    self.save_folder.mkdir(exist_ok=True)
    np.save(self.path_pts_3d, self.points_3d)
    np.save(self.path_descriptors, self.descriptors.cpu().numpy())



class XFeatInitializerViewPointSpecific():

  class ViewPointMatcher():
    def __init__(self, min_cossim):
      self.pose = None
      self.view_id = None
      self.points_3d = None
      self.descriptors = None
      self.representation = {}

      self.min_cossim = min_cossim

      self.optim_params = LevenbergMarquardtParameters()
      self.optim_params.gain = 1.0
      self.optim_params.maxNumIters = 1000
      self.optim_params.muInit = 0.1
      self.optim_params.muIterFactor = 0.99
      self.optim_params.minImprovementFactor = 0.00001

      self.pose_estimator = Pose()
      self.pose_estimator.setVvsIterMax(500)
      self.pose_estimator.setLambda(0.5)
      self.pose_estimator.setRansacMaxTrials(200)
      self.pose_estimator.setUseParallelRansac(True)
      self.pose_estimator.setNbParallelRansacThreads(4)

      self.pose_estimator.setRansacNbInliersToReachConsensus(4)
      self.pose_estimator.setRansacThreshold(0.1)

    def save(self, path: Path):
      if self.view_id is None:
        raise RuntimeError("Id is none and trying to save viewpoint data")

      save_folder = path / str(self.view_id)
      save_folder.mkdir(exist_ok=True)
      np.save(save_folder / 'points.npy', self.points_3d)
      print(f'Saving pose {self.pose}')
      np.save(save_folder / 'pose.npy', self.pose.numpy())
      for k in self.representation.keys():
        v = self.representation[k]
        if isinstance(v, torch.Tensor):
          v = v.cpu().numpy()
        np.save(save_folder / f'{k}.npy', v)

    def load(self, path: Path, view_id: int) -> bool:
      self.view_id = view_id
      load_folder = path / str(self.view_id)
      if not load_folder.exists():
        return False

      self.points_3d = np.load(load_folder / 'points.npy')
      self.pose = HomogeneousMatrix(np.load(load_folder / 'pose.npy'))
      # self.descriptors = torch.tensor(np.load(load_folder / 'descriptors.npy')).cuda()
      self.representation = {}
      for k in ['keypoints', 'descriptors', 'scales']:
        self.representation[k] = torch.tensor(np.load(load_folder / f'{k}.npy')).cuda()
      self.representation = XFeatInitializerViewPointSpecific.ViewPointMatcher.reformat_repr_if_needed(self.representation)
      return True


    @staticmethod
    def reformat_repr_if_needed(r):
      new_repr = {}
      for k, shape in [('keypoints', 3), ('descriptors', 3), ('scales', 2)]:
        v = r[k]
        new_v = None
        if not isinstance(v, torch.Tensor):
          new_v = torch.tensor(v).cuda()
        else:
          new_v = v
        if len(v.shape) != shape:
          new_v = new_v.reshape(-1, *new_v.size())
        new_repr[k] = new_v
      return new_repr

    def match_and_score(self, cam, backend: XFeatBackend, current_repr):
      kps_curr, kps_stored, self.idx_curr_matches, self.idx_stored_matches = backend.match_dense(current_repr, self.representation, min_cossim=self.min_cossim)
      # self.idx_curr_matches = self.idx_curr_matches.cpu().numpy()
      # self.idx_stored_matches = self.idx_stored_matches.cpu().numpy()
      self.matched_p3d = self.points_3d[self.idx_stored_matches].copy()
      self.matched_obs = kps_curr
      self.matched_obs_x, self.matched_obs_y = PixelMeterConversion.convertPoints(cam, self.matched_obs[:, 0], self.matched_obs[:, 1])

      return len(self.idx_stored_matches) / len(self.points_3d)


    def optimize(self, depth, threshold):
      cMo = HomogeneousMatrix(self.pose)
      self.pose_estimator.clearPoint()
      oMc = self.pose.inverse()


      Zs = depth.numpy()[(self.matched_obs[:, 1].astype(np.int32)), self.matched_obs[:, 0].astype(np.int32)][..., None]

      oX = (oMc.numpy() @ self.matched_p3d.T).T
      oX_3 = np.ascontiguousarray(oX[..., :3])
      observations = np.concatenate((self.matched_obs_x[..., None] * Zs, self.matched_obs_y[..., None] * Zs, Zs), axis=-1)
      from visp.core import Matrix
      observations = Matrix.view(observations)

      RBVisualOdometryUtils.levenbergMarquardtKeypoints3D(Matrix.view(oX_3), observations, self.optim_params, cMo)

      cX = (cMo.numpy() @ oX.T).T
      from visp.core import Robust, ColVector
      robust = Robust()
      distances = np.linalg.norm(observations.numpy() - cX[..., :3], axis=-1)

      weights = ColVector(len(distances))
      robust.MEstimator(Robust.TUKEY, ColVector.view(distances), weights)
      res = distances * weights.numpy()

      self.error = np.mean(res)



      # for i in range(len(self.matched_p3d)):
      #   p = Point()

      #   p.setWorldCoordinates(oX[i, 0], oX[i, 1], oX[i, 2])
      #   p.set_x(self.matched_obs_x[i])
      #   p.set_y(self.matched_obs_y[i])
      #   self.pose_estimator.addPoint(p)
      # # ok = True
      # self.pose_estimator.setRansacNbInliersToReachConsensus(len(self.matched_p3d) // 2)
      # ok = self.pose_estimator.computePose(Pose.RANSAC, cMo)
      # self.ransac_inliers_indices = self.pose_estimator.getRansacInlierIndex()

      # ok = self.pose_estimator.computePose(Pose.RANSAC, cMo)
      # if ok:
      #   self.pose_estimator.poseVirtualVSrobust(cMo)

      return cMo, self.error <= threshold

    def display_record(self, image: ImageRGBa, cTo, cam: CameraParameters):
      from visp.core import Display, MeterPixelConversion, Color
      IG = ImageRGBa(image)
      d = get_display()
      d.init(IG)

      Display.display(IG)

      cTr = cTo * self.pose.inverse()
      points_cam = (cTr.numpy() @ self.points_3d.T).T
      xs,ys = points_cam[:, 0] / points_cam[:, 2], points_cam[:, 1] / points_cam[:, 2]
      us,vs = MeterPixelConversion.convertPoints(cam, xs, ys)
      for i in range(len(us)):
        Display.displayPoint(IG, int(np.rint(vs[i])), int(np.rint(us[i])), Color.green, 2)


      Display.displayFrame(IG, self.pose, cam, 0.05, Color.yellow)
      Display.displayFrame(IG, cTo, cam, 0.05)

      Display.flush(IG)
      Display.getKeyboardEvent(IG, True)

    def display_result(self, image, cMo, cam):


      from visp.core import Display, MeterPixelConversion, Color
      oMc = self.pose.inverse()
      oX = (oMc.numpy() @ self.matched_p3d.T).T
      for i in range(len(self.matched_p3d)):
        p = Point()

        p.setWorldCoordinates(oX[i, 0], oX[i, 1], oX[i, 2])
        p.set_x(self.matched_obs_x[i])
        p.set_y(self.matched_obs_y[i])
        self.pose_estimator.addPoint(p)

      IG = ImageRGBa(image)
      d = get_display()
      d.init(IG)

      Display.display(IG)
      points: List[Point] = self.pose_estimator.getPoints()


      for i,p in enumerate(points):
        # if i not in self.ransac_inliers_indices:
        #   continue
        p.changeFrame(self.pose)
        p.project()
        u1, v1 = MeterPixelConversion.convertPoint(cam, p.get_x(), p.get_y())
        Display.displayPoint(IG, int(np.rint(v1)), int(np.rint(u1)), Color.blue, 2)
        Display.displayPoint(IG, int(np.rint(self.matched_obs[i, 1])), int(np.rint(self.matched_obs[i, 0])), Color.green, 2)

        p.changeFrame(cMo)
        p.project()
        u2, v2 = MeterPixelConversion.convertPoint(cam, p.get_x(), p.get_y())
        Display.displayPoint(IG, int(np.rint(v2)), int(np.rint(u2)), Color.red, 2)
        Display.displayLine(IG, int(np.rint(v2)), int(np.rint(u2)), int(np.rint(self.matched_obs[i, 1])), int(np.rint(self.matched_obs[i, 0])), Color.red, 1, segment=True)



      Display.displayFrame(IG, self.pose, cam, 0.05, Color.yellow)
      Display.displayFrame(IG, cMo, cam, 0.05)

      Display.flush(IG)
      Display.getKeyboardEvent(IG, True)


    def compute_residual(self, cMo, cam):
      return self.pose_estimator.computeResidual(cMo) / len(self.matched_obs)


  def __init__(self, backend: XFeatBackend, save_folder: Path):
    self.xfeat_backend = backend
    self.save_folder = save_folder
    self.views: List[XFeatInitializerViewPointSpecific.ViewPointMatcher] = []

    self.min_match_ratio = 0.5
    self.point_distance_threshold = 0.01
    self.min_cossim = 0.25
    if self.save_folder.exists():
      print('Loading viewpoints matching db...')
      view_id = 0
      while True:
        v = XFeatInitializerViewPointSpecific.ViewPointMatcher(self.min_cossim)
        loaded = v.load(self.save_folder, view_id)
        if not loaded:
          break
        self.views.append(v)
        view_id += 1
      # drift_detector = tracker.getDriftDetector()
      # if drift_detector is not None and isinstance(drift_detector, RBProbabilistic3DDriftDetector):
      #   drift_repr_json = self.save_folder / 'drift_detector.json'
      #   if drift_repr_json.exists():
      #     drift_detector.loadRepresentation(str(drift_repr_json))
      #   else:
      #     print('Could not find surface representation for drift detection')

      print(f'Loading database finished: found {view_id} viewpoints!')



  def record(self, frame: RBFeatureTrackerInput, cTo: HomogeneousMatrix):
    kps, descriptors, scales = self.xfeat_backend.get_repr_dense(frame.IRGB, 8192)

    # kps, descriptors = self.xfeat_backend.get_keypoints(frame.IRGB)
    us, vs = kps[:, 0], kps[:, 1] # Keypoints in current image with cTo intrinsics
    Z = frame.depth.numpy()[np.rint(vs).astype(np.int32), np.rint(us).astype(np.int32)]
    xs,ys = PixelMeterConversion.convertPoints(frame.cam, us, vs)


    depth_ok = Z > 0
    xs, ys, Z, kps, descriptors, scales = [array[depth_ok] for array in [xs, ys, Z, kps, descriptors, scales]]


    points_3d_cam = np.empty((len(xs), 4))
    points_3d_cam[:, 0] = xs * Z
    points_3d_cam[:, 1] = ys * Z
    points_3d_cam[:, 2] = Z
    points_3d_cam[:, 3] = 1.0
    points_3d_render = ((frame.renders.cMo * cTo.inverse()).numpy() @ points_3d_cam.T).T


    xsr, ysr = points_3d_render[:, 0] / points_3d_render[:, 2], points_3d_render[:, 1] / points_3d_render[:, 2]
    usr, vsr = MeterPixelConversion.convertPoints(frame.cam, xsr, ysr)

    in_image = np.logical_and(np.logical_and(usr >= 0, vsr >= 0), np.logical_and(usr < frame.renders.depth.getWidth(), vsr < frame.renders.depth.getHeight()))
    usr, vsr, points_3d_render, kps, descriptors, scales = [array[in_image] for array in [usr, vsr, points_3d_render, kps, descriptors, scales]]

    usri, vsri = (v.astype(np.int32) for v in (usr, vsr))

    Zr = frame.renders.depth.numpy()[vsri, usri]
    is_object = np.logical_and(Zr > 0, np.abs(Zr - points_3d_render[:, 2]) < 1e-2)
    points_3d_render[:, 2] = Zr

    points_3d_render, kps, descriptors, scales = [array[is_object] for array in [points_3d_render, kps, descriptors, scales]]

    assert len(points_3d_render) == len(descriptors) == len(kps) == len(scales)
    print(f'Recording new view with {len(descriptors)} keypoints!')
    viewpoint = XFeatInitializerViewPointSpecific.ViewPointMatcher(self.min_cossim)
    viewpoint.view_id = len(self.views)
    viewpoint.pose = HomogeneousMatrix(frame.renders.cMo)
    print(f'Recorded view {viewpoint.view_id} with pose {PoseVector(viewpoint.pose).t()}')
    viewpoint.points_3d = points_3d_render
    viewpoint.descriptors = descriptors
    viewpoint.representation = {
      'keypoints': kps,
      'descriptors': descriptors,
      'scales': scales
    }

    # viewpoint.display_record(frame.IRGB, cTo, frame.cam)

    self.views.append(viewpoint)

  def estimate_pose(self, image: ImageRGBa, depth, cam: CameraParameters) -> HomogeneousMatrix:

    # kps, descriptors = self.xfeat_backend.get_keypoints(image)
    kps, descriptors, scales = self.xfeat_backend.get_repr_dense(image, top_k=8192)
    current_representation = {
      'keypoints': kps,
      'descriptors': descriptors,
      'scales': scales
    }

    current_representation = XFeatInitializerViewPointSpecific.ViewPointMatcher.reformat_repr_if_needed(current_representation)


    scores = []
    for view in self.views:
      scores.append(view.match_and_score(cam, self.xfeat_backend, current_representation))
    views_enough_matches = list(filter(lambda x: x[1] > self.min_match_ratio, zip(self.views, scores)))

    if len(views_enough_matches) == 0:
      return HomogeneousMatrix(), False


    optim_data: List[Tuple[XFeatInitializerViewPointSpecific.ViewPointMatcher, float]] = []

    for view, score in views_enough_matches:
      optim_data.append((view, 1.0))


    poses = []
    errors = []
    final_views = []
    for view, weight in optim_data:
      cMo, pose_ok = view.optimize(depth, self.point_distance_threshold)
      if not pose_ok:
        continue

      final_views.append(view)
      poses.append(cMo)
      errors.append(view.error)

    if len(poses) == 0:
      return HomogeneousMatrix(), False


    print(f'Matching: {len(views_enough_matches)}, Geometric: {len(poses)}', end='\r')

    pose_ok = len(errors) >= 1
    best_view_index = np.argmin(errors)
    cMo = poses[best_view_index]


    return cMo, pose_ok

  def save(self):
    self.save_folder.mkdir(exist_ok=True)
    for v in self.views:
      v.save(self.save_folder)
