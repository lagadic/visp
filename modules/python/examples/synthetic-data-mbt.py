#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# ViSP Python bindings example
#
#############################################################################

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
import numpy as np
import time
import faulthandler
faulthandler.enable()

from visp.core import XmlParserCamera, CameraParameters, ColVector, HomogeneousMatrix, Display, ImageConvert
from visp.core import ImageGray, ImageUInt16
from visp.io import ImageIo
from visp.mbt import MbGenericTracker, MbTracker
from visp.display_utils import get_display
from visp.core import Color
from visp.core import PixelMeterConversion

try:
  import cv2
except:
  print('Could not import opencv python! make sure that it is installed as it is required')
  import sys
  sys.exit(1)

import matplotlib.pyplot as plt


class MBTModelData:
  def __init__(self, data_root: Path):
    model_path = data_root / 'model' / 'teabox'
    assert model_path.exists()
    self.config_color = model_path / 'teabox_color.xml'
    self.config_depth = model_path / 'teabox_depth.xml'
    self.cad_file = model_path / 'teabox.cao'
    self.init_file = model_path / 'teabox.init'


class MBTConfig:
  def __init__(self, data_root: Path):
    data_path = data_root / 'data' / 'teabox'
    assert data_path.exists()
    self.color_camera_name = 'Camera_L'
    self.depth_camera_name = 'Camera_R'

    self.color_intrinsics_file = data_path / f'{self.color_camera_name}.xml'
    self.depth_intrinsics_file = data_path / f'{self.depth_camera_name}.xml'

    self.color_images_dir = data_path / 'color'
    self.depth_images_dir = data_path / 'depth'
    self.ground_truth_dir = data_path / 'ground-truth'


    self.depth_intrinsics_file = data_path / f'{self.depth_camera_name}.xml'

    self.extrinsic_file = str(data_path / 'depth_M_color.txt')
    # self.ground_truth = str(data_root / 'data' / 'depth_M_color.txt')

@dataclass
class FrameData:
  I: ImageGray
  I_depth: Optional[ImageUInt16]
  point_cloud: Optional[np.ndarray]
  cMo_ground_truth: HomogeneousMatrix

def read_data(exp_config: MBTConfig, cam_depth: CameraParameters | None, I: ImageGray):
  color_format = '{:04d}_L.jpg'
  depth_format = 'Image{:04d}_R.exr'
  use_depth = cam_depth is not None
  iteration = 1
  while True:
    start_parse_time = time.time()
    color_filepath = exp_config.color_images_dir / color_format.format(iteration)
    if not color_filepath.exists():
      print(f'Could not find image {color_filepath}, is the sequence finished?')
      return
    ImageIo.read(I, str(color_filepath), ImageIo.IO_DEFAULT_BACKEND)


    I_depth_raw = None
    point_cloud = None
    if use_depth:
      t = time.time()
      depth_filepath = exp_config.depth_images_dir / depth_format.format(iteration)
      if not depth_filepath.exists():
        print(f'Could not find image {depth_filepath}')
        return
      I_depth_np = cv2.imread(str(depth_filepath), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
      I_depth_np = I_depth_np[..., 0]
      print(f'\tDepth load took {(time.time() - t) * 1000}ms')
      I_depth_raw = ImageUInt16(I_depth_np * 32767.5)
      if I_depth_np.size == 0:
        print('Could not successfully read the depth image')
        return
      t = time.time()
      # point_cloud = np.empty((*I_depth_np.shape, 3), dtype=np.float64)
      Z = I_depth_np.copy()
      Z[Z > 2] = 0.0 # Clamping values that are too high

      vs, us = np.meshgrid(range(I_depth_np.shape[0]), range(I_depth_np.shape[1]), indexing='ij')
      xs, ys = PixelMeterConversion.convertPoints(cam_depth, us, vs)
      point_cloud = np.stack((xs * Z, ys * Z, Z), axis=-1)

      print(f'\tPoint_cloud took {(time.time() - t) * 1000}ms')


    cMo_ground_truth = HomogeneousMatrix()
    ground_truth_file = exp_config.ground_truth_dir / (exp_config.color_camera_name + '_{:04d}.txt'.format(iteration))
    cMo_ground_truth.load(str(ground_truth_file))
    iteration += 1
    end_parse_time = time.time()
    print(f'Data parsing took: {(end_parse_time - start_parse_time) * 1000}ms')
    yield FrameData(I, I_depth_raw, point_cloud, cMo_ground_truth)

def parse_camera_file(exp_config: MBTConfig, is_color: bool) -> CameraParameters:
  cam = CameraParameters()
  xml_parser = XmlParserCamera()
  if is_color:
    camera_name, file_path = exp_config.color_camera_name, exp_config.color_intrinsics_file
  else:
    camera_name, file_path = exp_config.depth_camera_name, exp_config.depth_intrinsics_file
  parse_res = xml_parser.parse(cam, str(file_path), camera_name,
                               CameraParameters.perspectiveProjWithoutDistortion, 0, 0, True)
  assert parse_res == XmlParserCamera.SEQUENCE_OK # Check result
  return cam

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data-root', type=str, required=True,
                      help='Path to the folder containing all the data for the synthetic MBT example')
  parser.add_argument('--disable-klt', action='store_true', help='Disable KLT features for tracking.')
  parser.add_argument('--disable-depth', action='store_true', help='Do not use depth to perform tracking.')
  parser.add_argument('--step-by-step', action='store_true', help='Perform tracking frame by frame. Go to the next frame by clicking.')
  parser.add_argument('--init-ground-truth', action='store_true')

  args = parser.parse_args()
  data_root = Path(args.data_root)

  mbt_model = MBTModelData(data_root)
  exp_config = MBTConfig(data_root)

  assert data_root.exists() and data_root.is_dir()

  rgb_tracker: int = MbGenericTracker.EDGE_TRACKER | (MbGenericTracker.KLT_TRACKER if not args.disable_klt else 0)
  tracker_types: List[int] = [rgb_tracker]
  if not args.disable_depth:
    depth_tracker = MbGenericTracker.DEPTH_DENSE_TRACKER
    tracker_types.append(depth_tracker)

  tracker = MbGenericTracker(tracker_types)

  if args.disable_depth:
    tracker.loadConfigFile(str(mbt_model.config_color))
  else:
    tracker.loadConfigFile(str(mbt_model.config_color), str(mbt_model.config_depth))
  tracker.loadModel(str(mbt_model.cad_file))

  # Camera intrinsics
  cam_color = parse_camera_file(exp_config, True)
  cam_depth = parse_camera_file(exp_config, False) if not args.disable_depth else None

  tracker.setCameraParameters(*((cam_color,) if args.disable_depth else (cam_color, cam_depth)))
  tracker.setDisplayFeatures(True)

  print('Color intrinsics:', cam_color)
  print('Depth intrinsics:', cam_depth)
  I = ImageGray()
  data_generator = read_data(exp_config, cam_depth, I)
  frame_data = next(data_generator) # Get first frame for init

  depth_M_color = HomogeneousMatrix()
  if not args.disable_depth:
    depth_M_color.load(exp_config.extrinsic_file)
    tracker.setCameraTransformationMatrix('Camera2', depth_M_color)

  # Initialize displays
  dI = get_display()
  dI.init(I, 0, 0, 'Color image')

  I_depth = None if args.disable_depth else ImageGray()
  dDepth = get_display()
  if not args.disable_depth:
    ImageConvert.createDepthHistogram(frame_data.I_depth, I_depth)
    dDepth.init(I_depth,  I.getWidth(), 0, 'Depth')

  if args.init_ground_truth:
    tracker.initFromPose(I, frame_data.cMo_ground_truth)
  else:
    tracker.initClick(I, str(mbt_model.init_file))

  start_time =  time.time()
  for frame_data in data_generator:
    if frame_data.I_depth is not None:
      ImageConvert.createDepthHistogram(frame_data.I_depth, I_depth)

    Display.display(I)
    if not args.disable_depth:
      Display.display(I_depth)

    if args.disable_depth:
      tracker.track(I=I)
    else:
      pc = frame_data.point_cloud
      image_dict = {
        'Camera1': I
      }
      t = time.time()
      tracker.track(image_dict, {'Camera2': pc})
      print(f'Tracking took {(time.time() - t) * 1000}ms')
    cMo = HomogeneousMatrix()
    tracker.getPose(cMo)

    Display.displayFrame(I, cMo, cam_color, 0.05, Color.none, 2)
    tracker.display(I, cMo, cam_color, Color.red, 2)
    Display.flush(I)
    if not args.disable_depth:
      Display.flush(I_depth)
    if args.step_by_step:
      Display.getKeyboardEvent(I, blocking=True)
  end_time = time.time()
  print(f'total time = {end_time - start_time}s')
