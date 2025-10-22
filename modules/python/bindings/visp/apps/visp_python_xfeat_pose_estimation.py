import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Generator
import numpy as np

import pyrealsense2 as rs


from visp.core import CameraParameters, ImageGray, ImageRGBa, ImageConvert, ImageFloat
from visp.core import Display, Color, HomogeneousMatrix, MouseButton
from visp.ar import Panda3DGeometryRenderer, Panda3DRendererSet, Panda3DRenderParameters
from visp.rbt import Panda3DDepthCannyFilter
from visp.rbt import RBTracker
from visp.python.rbt import PythonRBExtensions
from visp.python.rbt.xfeat import XFeatViewPointPoseEstimator, XFeatBackend
from visp.python.display_utils import get_display


@dataclass
class Frame():
  I: ImageGray
  IRGB: ImageRGBa
  I_depth: ImageFloat

class FrameSource():
  def intrinsics(self) -> CameraParameters:
    raise NotImplementedError()
  def frames(self):
    raise NotImplementedError()

class RealSenseSource(FrameSource):
  def __init__(self, args):
    import pyrealsense2 as rs
    self.pipe = rs.pipeline()
    self.config = rs.config()
    self.h, self.w = args.height, args.width
    self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.rgb8, args.fps)
    self.config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, args.fps)


    self.cfg = self.pipe.start(self.config)
    self.profile = self.cfg.get_stream(rs.stream.color)
    intr = self.profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
    self.cam = CameraParameters(intr.fx, intr.fy, intr.ppx, intr.ppy)
    self.depth_scale = self.cfg.get_device().first_depth_sensor().get_depth_scale()
    print(f'Opened Realsense camera with intrinsics:\n{self.cam}')

  def intrinsics(self) -> CameraParameters:
    return self.cam

  def frames(self):

    def generator():
      align_to = rs.align(rs.stream.color)
      img = ImageGray(self.h, self.w)
      rgb = ImageRGBa(self.h, self.w)
      depth = ImageFloat(self.h, self.w)
      while True:
        frame = self.pipe.wait_for_frames()
        frame = align_to.process(frame)
        I_np = np.asanyarray(frame.get_color_frame().as_frame().get_data())
        I_depth_raw = np.asanyarray(frame.get_depth_frame().as_frame().get_data())
        I_depth_float = I_depth_raw.astype(np.float32) * self.depth_scale
        ImageConvert.RGBToGrey(I_np, img)
        ImageConvert.RGBToRGBa(I_np, rgb)
        np.copyto(src=I_depth_float, dst=depth.numpy())

        yield Frame(img, rgb, depth)

    return generator()


class DisplayFrames:
  def __init__(self, h, w):
    self.h = h
    self.w = w
    pad = w // 5
    self.I_disp = ImageGray(h, w)
    self.rgb_disp = ImageRGBa(h, w)
    self.depth_disp = ImageGray(h, w)

    self.dI, self.dRGB, self.dDepth = [get_display() for _ in range(3)]
    self.dI.init(self.I_disp, 0, 0, 'Gray image')
    self.dRGB.init(self.rgb_disp,  w + pad, 0, 'Color image')
    self.dDepth.init(self.depth_disp, 0, h + pad, 'Depth image')

  def display(self, I: ImageGray, RGB: ImageRGBa, depth: ImageFloat):
    assert I.getSize() == self.I_disp.getSize()
    assert RGB.getSize() == self.rgb_disp.getSize()
    assert depth.getSize() == self.depth_disp.getSize()

    np.copyto(src=I.numpy(), dst=self.I_disp.numpy())
    np.copyto(src=RGB.numpy(), dst=self.rgb_disp.numpy())
    depth_uint = (np.minimum(depth.numpy() / 1.0, 1.0) * 255.0).astype(np.uint8)
    np.copyto(src=depth_uint, dst=self.depth_disp.numpy())
    for f in [self.I_disp, self.rgb_disp, self.depth_disp]:
      Display.display(f)

  def flush(self):
    for f in [self.I_disp, self.rgb_disp, self.depth_disp]:
      Display.flush(f)


def main():
  parser = argparse.ArgumentParser(description='Pose estimation with XFeat')

  parser.add_argument('--database', type=str, required=True, help='Path where to save the keypoints and their descriptors. If it already exists, then it will be used to load keypoints.')
  parser.add_argument('--object', type=str, required=True, help='Path to the .obj file of the object for which to estimate the pose')
  parser.add_argument('--rbt-config', type=str, required=True, help='Path to the Render Based Tracker configuration. It is used to continuously estimate the pose in the learning step')
  parser.add_argument('--init-file', type=str, help='An initialization file, required to init tracker when XFeat points have not yet been learned.')

  parser.add_argument('--width', type=int, default=640, help='Width of the input images')
  parser.add_argument('--height', type=int, default=480, help='Height of the input images')
  parser.add_argument('--fps', type=int, default=60, help='Camera framerate')

  args = parser.parse_args()

  db_path = Path(args.database).absolute()
  is_learning = not db_path.exists()

  object_path = Path(args.object).absolute()
  rbt_path = Path(args.rbt_config).absolute()

  assert object_path.exists()
  assert rbt_path.exists()

  pose_estimator = XFeatViewPointPoseEstimator(db_path, False, False, 2 ** 12, 0.5, 0.25, 10)
  pose_estimator.optim_params.gain = 0.5
  pose_estimator.optim_params.minImprovementFactor = 0.001


  frame_source = RealSenseSource(args)
  exts = PythonRBExtensions()

  if is_learning:
    print('Database was not found, going into learning mode!')

    print('Setting up tracker..')
    tracker = RBTracker()

    tracker.loadConfigurationFile(str(rbt_path))

    exts.parse_python_extensions(tracker, rbt_path)
    tracker.setModelPath(str(object_path))
    tracker.setCameraParameters(frame_source.intrinsics(), frame_source.h, frame_source.w)
    tracker.startTracking()

    learn_loop(args, tracker, frame_source, pose_estimator)

  else:
    estimation_loop(object_path, frame_source, pose_estimator)

def tracker_click_init_loop(frames, tracker: RBTracker, init_path: Path, display_frames: DisplayFrames):
  for frame in frames:
    display_frames.display(frame.I, frame.IRGB, frame.I_depth)
    Display.displayText(display_frames.I_disp, 10, 10, 'Click to initialize by click', Color.red)
    display_frames.flush()

    clicked = Display.getClick(display_frames.I_disp, blocking=False)

    if clicked:
      tracker.initClick(display_frames.I_disp, str(init_path), True)
      break

def learn_loop(args, tracker: RBTracker, frame_source: RealSenseSource, pose_estimator: XFeatViewPointPoseEstimator):
  frames = frame_source.frames()
  display_frames = DisplayFrames(frame_source.h, frame_source.w)
  print('Initializing tracker before recording keypoints...')
  # Initialize by click
  if args.init_file is None:
    raise RuntimeError('Init file is required to initialize the Render Base tracker before learning keypoints.')
  init_path = Path(args.init_file).absolute()

  tracker_click_init_loop(frames, tracker, init_path, display_frames)

  cMo = HomogeneousMatrix()
  tracker.getPose(cMo)
  print('Tracker was initialized with pose:\n', cMo)
  line_height = 20
  for frame in frames:
    tracking_result = tracker.track(frame.I, frame.IRGB, frame.I_depth)

    tracker.getPose(cMo)

    display_frames.display(frame.I, frame.IRGB, frame.I_depth)
    tracker.display(display_frames.I_disp, display_frames.rgb_disp, display_frames.depth_disp)

    Display.displayText(display_frames.I_disp, 10, 10, 'Left click to record a viewpoint', Color.red)
    Display.displayText(display_frames.I_disp, 10 + line_height, 10, 'Right click to stop', Color.red)
    Display.displayText(display_frames.I_disp, 10 + line_height * 2, 10, 'Press R to reinitialize', Color.red)

    button = MouseButton.MouseButtonType.none
    clicked = Display.getClick(display_frames.I_disp, button, blocking=False)
    key_pressed, key = Display.getKeyboardEventWithKey(display_frames.I_disp, blocking=False)


    if clicked:
      if button == MouseButton.button1:
        pose_estimator.record(tracker.getMostRecentFrame(), cMo)
        print('Recorded new viewpoint')

      elif button == MouseButton.button3:
        pose_estimator.save()
        print('Finished recording, exiting...')
        break
    display_frames.flush()

    if key_pressed:
      if key.lower() == 'r':
        tracker_click_init_loop(frames, tracker, init_path, display_frames)




def estimation_loop(object_path: Path, frame_source: RealSenseSource, pose_estimator: XFeatViewPointPoseEstimator):
  frames = frame_source.frames()
  display_frames = DisplayFrames(frame_source.h, frame_source.w)
  line_height = 20
  last_estimated_cMo = HomogeneousMatrix()
  h, w = frame_source.h, frame_source.w
  canny_or, canny_valid = ImageFloat(h, w), ImageGray(h, w)
  point_indices = None

  geometry_renderer = Panda3DGeometryRenderer(Panda3DGeometryRenderer.RenderType.OBJECT_NORMALS, False)
  canny_renderer = Panda3DDepthCannyFilter('canny', geometry_renderer, True, 0.01)
  renderer = Panda3DRendererSet()

  renderer.addSubRenderer(geometry_renderer)
  renderer.addSubRenderer(canny_renderer)

  render_params = Panda3DRenderParameters(frame_source.intrinsics(), frame_source.h, frame_source.w, 0.001, 0.25)
  renderer.setRenderParameters(render_params)
  renderer.initFramework()
  renderer.addObjectToScene('object', str(object_path))
  for frame in frames:
    display_frames.display(frame.I, frame.IRGB, frame.I_depth)

    if point_indices is not None:
      for vs, us in zip(*point_indices):
        Display.displayPoint(display_frames.I_disp, vs, us, Color.green, thickness=2)
    Display.displayText(display_frames.I_disp, 10, 10, 'Left click to perform pose estimation', Color.red)
    Display.displayText(display_frames.I_disp, 10 + line_height, 10, 'Right click to stop', Color.red)
    button = MouseButton.MouseButtonType.none
    clicked = Display.getClick(display_frames.I_disp, button, blocking=False)

    if clicked:
      if button == MouseButton.button1:
        pose_ok, cMo = pose_estimator.estimate_pose(frame.IRGB, frame.I_depth, frame_source.intrinsics())
        if pose_ok:
          print('Pose estimation was detected as successful!')

          last_estimated_cMo = cMo
          renderer.setCameraPose(cMo.inverse())
          nearV, farV = geometry_renderer.computeNearAndFarPlanesFromNode('object', True)
          params = Panda3DRenderParameters(frame_source.intrinsics(), frame_source.h, frame_source.w, nearV, farV)
          renderer.setRenderParameters(params)
          renderer.renderFrame()
          canny_renderer.getRender(canny_or, canny_valid)
          point_indices = np.nonzero(canny_valid.numpy())

      elif button == MouseButton.button3:
        print('Finished pose estimation test, exiting...')
        break

    for frame_display in [display_frames.I_disp, display_frames.rgb_disp, display_frames.depth_disp]:
      Display.displayFrame(frame_display, last_estimated_cMo, frame_source.intrinsics(), 0.05)

    display_frames.flush()

    from visp.ar import Panda3DFrameworkManager
    Panda3DFrameworkManager.getInstance().exit()





if __name__ == '__main__':
  main()
