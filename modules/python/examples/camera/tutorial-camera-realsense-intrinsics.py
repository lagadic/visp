import pyrealsense2 as rs

from visp.core import CameraParameters


# Set the camera image dimensions
WIDTH = 640
HEIGHT = 480
FPS = 60

def get_camera_parameters(profile, stream_type):
  # Get the calibration data for the selected video stream.
  intrinsics = (profile.get_stream(stream_type).as_video_stream_profile().get_intrinsics())
  return CameraParameters(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

# Configure the camera streams
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

# Start streaming and get the active camera profile
pipeline = rs.pipeline()
profile = pipeline.start(config)

try:
  # Read the color camera intrinsics
  color_camera = get_camera_parameters(profile, rs.stream.color)
  print("Color camera intrinsics:")
  print(f"  ViSP parameters: {color_camera}")

  # Read the depth camera intrinsics
  depth_camera = get_camera_parameters(profile, rs.stream.depth)
  print("Depth camera intrinsics:")
  print(f"  ViSP parameters: {depth_camera}")

finally:
  # Stop streaming when the program exits
  pipeline.stop()