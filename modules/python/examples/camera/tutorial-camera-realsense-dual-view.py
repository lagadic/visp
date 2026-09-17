import numpy as np
import pyrealsense2 as rs

from visp.core import ImageRGBa, Display
from visp.python.display_utils import get_display


# Set the camera image dimensions
WIDTH = 640
HEIGHT = 480
FPS = 60

# Configure the camera stream
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

# Start streaming frames from the camera
pipeline = rs.pipeline()
pipeline.start(config)

try:
  # Create and initialize display images for both color and depth views
  image_color = ImageRGBa(HEIGHT, WIDTH)
  display_color = get_display()
  display_color.init(image_color, 0, 0, "Color view")

  image_depth = ImageRGBa(HEIGHT, WIDTH)
  display_depth = get_display()
  display_depth.init(image_depth, WIDTH, 0, "Depth view")

  # Access the display images data as a NumPy array
  image_color_array = np.asarray(image_color)
  image_depth_array = np.asarray(image_depth)

  # Get a colorizer to convert raw depth values into a color image
  colorizer = rs.colorizer()

  while True:
    # Capture the latest camera frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Colorize the depth image for visualization
    depth_color_frame = colorizer.colorize(depth_frame)

    # Convert the frames to NumPy arrays
    color_array = np.asanyarray(color_frame.get_data())
    depth_array = np.asanyarray(depth_color_frame.get_data())

    # Copy the camera images into the display images
    image_color_array[..., :3] = color_array
    image_depth_array[..., :3] = depth_array

    # Render the updated images
    Display.display(image_color)
    Display.flush(image_color)
    Display.display(image_depth)
    Display.flush(image_depth)

    # Stop when the user clicks inside one of the displays
    if Display.getClick(image_color, False) or Display.getClick(image_depth, False):
      break

finally:
  # Stop streaming when the program exits
  pipeline.stop()
