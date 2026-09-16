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
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

# Start streaming frames from the camera
pipeline = rs.pipeline()
pipeline.start(config)

try:
  # Create and initialize the display image
  image = ImageRGBa(HEIGHT, WIDTH)
  display = get_display()
  display.init(image)

  # Access the display image data as a NumPy array
  image_array = np.asarray(image)

  # Get a colorizer to convert raw depth values into a color image
  colorizer = rs.colorizer()

  while True:
    # Capture the latest camera frame
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    # Colorize the depth image for visualization
    depth_color_frame = colorizer.colorize(depth_frame)

    # Convert the depth frame to a NumPy array
    depth_image = np.asanyarray(depth_color_frame.get_data())

    # Copy the camera image into the display image
    image_array[..., :3] = depth_image

    # Render the updated image
    Display.display(image)
    Display.flush(image)

    # Stop when the user clicks inside the display
    if Display.getClick(image, False):
      break

finally:
  # Stop streaming when the program exits
  pipeline.stop()
