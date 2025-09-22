
import numpy as np

import torch

from visp.core import ImageTools, Rect
from visp.core import ImageRGBa, Matrix
from visp.rbt import RBFeatureTrackerInput
from visp.rbt import RBVisualOdometryUtils
from visp.python.vision.xfeat import XFeatBackend, XFeatRepresentation


class XFeatTrackingBackend(XFeatBackend):
  def __init__(self, num_points: int, min_cos: float, use_dense=False, scale_factor = 1.0, only_on_bb = False):
    XFeatBackend.__init__(self, num_points, min_cos, use_dense, scale_factor)

    self.last_iter = -1

    self.min_obj_mask_value = 0.5
    self.min_obj_distance = 1.0
    self.min_env_distance = 5.0
    self.only_on_bb = only_on_bb


    self.current_representation_object = None
    self.current_representation_environment = None

    self.previous_representation_object = None
    self.previous_representation_environment = None

  def load_settings(self, d: dict):
    XFeatBackend.load_settings(self, d)
    self.only_on_bb = d.get('onlyOnBB', False)
    self.min_env_distance = d['minSilhouetteDistanceEnvPoint']
    self.min_obj_distance = d['minSilhouetteDistanceObjectPoint']
    self.min_obj_mask_value = d['minObjMaskValue']

  def process_frame(self, frame: RBFeatureTrackerInput, iteration: int):
    if self.last_iter > 0 and self.last_iter == iteration:
      return # Frame was already processed

    with torch.no_grad():
      input_rgb = frame.IRGB
      bb = Rect(frame.renders.boundingBox)
      if self.only_on_bb:
        crop = ImageRGBa()
        bb.setWidth(np.maximum(32, bb.getWidth()))
        bb.setHeight(np.maximum(32, bb.getHeight()))
        bb.setLeft(np.maximum(0, bb.getLeft()))
        bb.setTop(np.maximum(0, bb.getTop()))

        ImageTools.crop(frame.IRGB, bb, crop)

        input_rgb = crop
      representation = self.detect(input_rgb)
      if self.only_on_bb:

        l = np.maximum(0, np.ceil(bb.getLeft()))
        t = np.maximum(0, np.ceil(bb.getTop()))
        representation.keypoints[:, 0] += l
        representation.keypoints[:, 1] += t

    self.last_iter = iteration
    self.previous_representation_object = self.current_representation_object
    self.previous_representation_environment = self.current_representation_environment
    obj_indices, env_indices = RBVisualOdometryUtils.computeIndicesObjectAndEnvironment(Matrix(representation.keypoints), frame,
                                                                                        self.min_obj_mask_value,
                                                                                        self.min_obj_distance, self.min_env_distance)
    self.current_representation_object, self.current_representation_environment = representation.split(obj_indices, env_indices)

  def get_current_object_data(self) -> XFeatRepresentation:
    return self.current_representation_object
  def get_previous_object_data(self) -> XFeatRepresentation:
    return self.previous_representation_object
  def get_current_environment_data(self) -> XFeatRepresentation:
    return self.current_representation_environment
