import pyrealsense2 as rs
import numpy as np
from visp.core import CameraParameters
from visp.core import ImageRGBa, ImageUInt16, ImageGray
from visp.core import ImageConvert, Display
from visp.gui import DisplayX

def cam_from_rs_profile(profile) -> CameraParameters:
  '''Get camera intrinsics from the realsense framework'''
  # Downcast to video_stream_profile and fetch intrinsics
  intr = profile.as_video_stream_profile().get_intrinsics()
  return CameraParameters(intr.fx, intr.fy, intr.ppx, intr.ppy)

if __name__ == '__main__':

  # Initialize realsense2
  pipe = rs.pipeline()
  config = rs.config()
  fps = 60
  h, w = 480, 640
  config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
  config.enable_stream(rs.stream.color, w, h, rs.format.rgba8, fps)

  cfg = pipe.start(config)

  I_gray = ImageGray(h, w)
  display_gray = DisplayX()
  display_gray.init(I_gray, 0, 0, 'Color')
  I_depth_hist = ImageGray(h, w)
  display_depth = DisplayX()
  display_depth.init(I_depth_hist, 640, 0, 'Color')


  # Retrieve intrinsics
  cam_color = cam_from_rs_profile(cfg.get_stream(rs.stream.color))
  cam_depth = cam_from_rs_profile(cfg.get_stream(rs.stream.depth))

  point_cloud_computer = rs.pointcloud()
  while True:
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    # NumPy Representations of realsense frames
    I_color_np = np.asanyarray(color_frame.as_frame().get_data())
    I_depth_np = np.asanyarray(depth_frame.as_frame().get_data())
    # ViSP representations
    I_color = ImageRGBa(I_color_np) # This works because format is rs.format.rgba8, otherwise concat or conversion needed
    I_depth = ImageUInt16(I_depth_np)
    # Transform depth frame as point cloud and view it as an N x 3 numpy array
    point_cloud = np.asanyarray(point_cloud_computer.calculate(depth_frame).get_vertices()).view((np.float32, 3))

    ImageConvert.convert(I_color, I_gray)
    ImageConvert.createDepthHistogram(I_depth, I_depth_hist)

    Display.display(I_gray)
    Display.display(I_depth_hist)
    Display.flush(I_gray)
    Display.flush(I_depth_hist)
