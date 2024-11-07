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
from typing import List, Optional, Tuple
import numpy as np
import time
import faulthandler
faulthandler.enable()


from visp.core import CameraParameters, HomogeneousMatrix
from visp.core import Color, Display, ImageConvert
from visp.core import ImageGray, ImageUInt16, ImageRGBa
from visp.io import ImageIo
from visp.mbt import MbGenericTracker
from visp.display_utils import get_display
import pyrealsense2 as rs


try:
  import cv2
except:
  print('Could not import opencv-python! make sure that it is installed as it is required')
  import sys
  sys.exit(1)

import matplotlib.pyplot as plt


class MBTModelData:
  def __init__(self, data_root: Path, object_name: str):
    model_path = data_root / 'model' / object_name
    assert model_path.exists()
    self.config_color = model_path / f'{object_name}.xml'
    self.config_depth = model_path / f'{object_name}_depth.xml'
    self.cad_file = model_path / f'{object_name}.cao'
    self.init_file = model_path / f'{object_name}.init'


class MBTConfig:
  def __init__(self, data_root: Path):
    data_path = data_root / 'data'
    assert data_path.exists()
    self.extrinsic_file = str(data_path / 'depth_M_color.txt')


@dataclass
class FrameData:
  I: ImageGray
  I_depth: Optional[ImageUInt16]
  point_cloud: Optional[np.ndarray]




def read_data(cam_depth: CameraParameters | None, I: ImageGray, pipe: rs.pipeline):
  use_depth = cam_depth is not None
  iteration = 1
  point_cloud_computer = rs.pointcloud()
  while True:
    frames = pipe.wait_for_frames()
    I_np = np.asanyarray(frames.get_color_frame().as_frame().get_data())
    I_np = np.concatenate((I_np, np.ones_like(I_np[..., 0:1], dtype=np.uint8)), axis=-1)
    I_rgba = ImageRGBa(I_np)
    ImageConvert.convert(I_rgba, I, 0)
    I_depth_raw = None
    point_cloud = None
    if use_depth:
      I_depth_raw = np.asanyarray(frames.get_depth_frame().as_frame().get_data())
      point_cloud = np.asanyarray(point_cloud_computer.calculate(frames.get_depth_frame()).get_vertices()).view((np.float32, 3))
    iteration += 1
    yield FrameData(I, ImageUInt16(I_depth_raw), point_cloud)



def cam_from_rs_profile(profile) -> Tuple[CameraParameters, int, int]:
  intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
  return CameraParameters(intr.fx, intr.fy, intr.ppx, intr.ppy), intr.height, intr.width

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data-root', type=str, required=True,
                      help='Path to the folder containing all the data for the MBT example.')
  parser.add_argument('--object-name', type=str, required=True,
                      help='Name of the object to track.')
  parser.add_argument('--disable-klt', action='store_true', help='Disable KLT features for tracking.')
  parser.add_argument('--disable-depth', action='store_true', help='Do not use depth to perform tracking.')
  parser.add_argument('--step-by-step', action='store_true', help='Perform tracking frame by frame. Go to the next frame by clicking.')


  args = parser.parse_args()
  data_root = Path(args.data_root)


  # Initialize realsense2
  pipe = rs.pipeline()
  config = rs.config()
  config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
  config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 60)

  cfg = pipe.start(config)

  assert data_root.exists() and data_root.is_dir()

  mbt_model = MBTModelData(data_root, args.object_name)
  exp_config = MBTConfig(data_root)

  cam_color, color_height, color_width = cam_from_rs_profile(cfg.get_stream(rs.stream.color))
  cam_depth, depth_height, depth_width = cam_from_rs_profile(cfg.get_stream(rs.stream.depth))


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

  tracker.setCameraParameters(*((cam_color,) if args.disable_depth else (cam_color, cam_depth)))
  tracker.setDisplayFeatures(True)

  print('Color intrinsics:', cam_color)
  print('Depth intrinsics:', cam_depth)
  I = ImageGray()
  data_generator = read_data(cam_depth, I, pipe)
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

  for frame in data_generator:
    Display.display(I)
    Display.displayText(I, 0, 0, 'Click to initialize tracking', Color.red)
    Display.flush(I)
    event = Display.getClick(I, blocking=False)
    if event:
      break

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
      tracker.track(image_dict, {'Camera2': pc.reshape(depth_height, depth_width, 3)})
    cMo = HomogeneousMatrix()
    tracker.getPose(cMo)

    Display.displayFrame(I, cMo, cam_color, 0.05, Color.none, 2)
    tracker.display(I, cMo, cam_color, Color.red, 2)
    Display.flush(I)
    if not args.disable_depth:
      Display.flush(I_depth)

    if args.step_by_step:
      Display.getKeyboardEvent(I, blocking=True)
    else:
      event = Display.getClick(I, blocking=False)
      if event:
        break
  end_time = time.time()
  print(f'total time = {end_time - start_time}s')
