import numpy as np
import pyrealsense2 as rs

from visp.detection import DetectorAprilTag
from visp.core import ImageGray, ImageRGBa, ImageConvert, Color, CameraParameters
from visp.core import Display
from visp.python.display_utils import get_display


# Set the camera image dimensions
WIDTH = 640
HEIGHT = 480
FPS = 60

# Set the tag parameters
TAG_FAMILY = DetectorAprilTag.TAG_36h11
TAG_SIZE = 0.053 # in meters

try:
  # Configure the camera stream
  config = rs.config()
  config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)

  # Start streaming frames from the camera
  pipeline = rs.pipeline()
  profile = pipeline.start(config)

  # Create and initialize the display image
  image = ImageRGBa(HEIGHT, WIDTH)
  display = get_display()
  display.init(image, 0, 0, "Camera view")

  # Access the display image data as a NumPy array
  image_array = np.asarray(image)

  # Create a grayscale image for detection calcul
  image_gray = ImageGray(HEIGHT, WIDTH)

  # Retrieve the camera intrinsics parameters
  intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
  camera = CameraParameters(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

  # Initialize the AprilTag detection
  detector = DetectorAprilTag(TAG_FAMILY)

  while True:
    # Capture the latest camera frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    if not color_frame:
      continue

    # Convert the color frame to a NumPy array
    color_image = np.asanyarray(color_frame.get_data())

    # Copy the camera image into the display image
    image_array[..., :3] = color_image

    # Convert it to grayscale format
    ImageConvert.convert(image, image_gray)

    # Detect the tags on the image
    _, tag_poses = detector.detect(image_gray, TAG_SIZE, camera)
    tag_corners = detector.getTagsCorners()

    # Render the updated image
    Display.display(image)

    # Display the tags frames and poses
    detector.displayTags(image, tag_corners, Color.none, 3)
    detector.displayFrames(image, tag_poses, camera, TAG_SIZE / 2.0, Color.none, 3)

    Display.flush(image)

    # Stop when the user clicks inside the display
    if Display.getClick(image, False):
      break

finally:
  # Stop streaming when the program exits
  pipeline.stop()
