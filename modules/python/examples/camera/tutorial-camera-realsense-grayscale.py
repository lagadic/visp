import numpy as np
import pyrealsense2 as rs

from visp.core import ImageGray, ImageRGBa, ImageConvert, Display
from visp.python.display_utils import get_display


# Set the camera image dimensions
WIDTH = 640
HEIGHT = 480
FPS = 60

# Configure the camera stream
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)

# Start streaming frames from the camera
pipeline = rs.pipeline()
pipeline.start(config)

try:
  # Create and initialize the display image
  image = ImageGray(HEIGHT, WIDTH)
  display = get_display()
  display.init(image, 0, 0, "Grayscale view")

  # Create a RGBa image to store each frame
  image_rgba = ImageRGBa(HEIGHT, WIDTH)

  # Access the RGBa image data as a NumPy array
  image_array = np.asarray(image_rgba)

  while True:
    # Capture the latest camera frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Convert the color frame to a NumPy array
    color_image = np.asanyarray(color_frame.get_data())

    # Copy the camera image into the display image
    image_array[..., :3] = color_image

    # Convert it to grayscale format
    ImageConvert.convert(image_rgba, image)

    # Render the updated image
    Display.display(image)
    Display.flush(image)

    # Stop when the user clicks inside the display
    if Display.getClick(image, False):
      break

finally:
  # Stop streaming when the program exits
  pipeline.stop()
