import argparse
import math
import sys
import numpy as np
import pyrealsense2 as rs

from visp.core import ImageRGBa, ImageGray, ImageConvert, Color
from visp.core import PoseVector, HomogeneousMatrix, ColVector, Point
from visp.core import CameraParameters
from visp.core import measureTimeMs
from visp.core import MouseButton

from visp.detection import DetectorAprilTag
from visp.robot import RobotFranka, Robot
from visp.vs import Servo, ServoDisplay, AdaptiveGain
from visp.visual_features import FeaturePoint3D

from visp.core import Display
from visp.python.display_utils import get_display

# -----------------------------------------------------------------------------
# Parser
# -----------------------------------------------------------------------------

parser = argparse.ArgumentParser(description='Python example of eye-in-hand visual servoing using a Franka robot and a Realsense Camera')
parser.add_argument("--ip", type=str, default="192.168.30.10", dest="robot_ip", help="Franka robot IP address. Default: %(default)s")
parser.add_argument("--tag-size", type=float, default=0.120, dest="tag_size", help="AprilTag size in meters. Default: %(default)s")
parser.add_argument("--emc", type=str, default="", dest="emc_file", metavar="FILE", help="File containing the homogeneous transformation matrix between the robot and camera frame.")
parser.add_argument("--adaptive-gain", action="store_true", dest="adaptive_gain", help="Enable adaptive gain.")
parser.add_argument("--convergence-threshold", type=float, default=0.00005, dest="convergence_threshold", help="Convergence threshold of the servoing before stopping. Default: %(default)s")
parser.add_argument("--no-convergence-threshold", action="store_true", dest="no_convergence_threshold", help="Disable the convergence threshold used to stop visual servoing.")
parser.add_argument("--distance-to-tag", type=float, default=0.4, dest="distance_to_tag", help="Desired distance to the AprilTag in meters. Default: %(default)s")

args = parser.parse_args()

# -----------------------------------------------------------------------------
# Program parameters
# -----------------------------------------------------------------------------

# Set the camera configuration
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_FPS = 60

# Set the camera extrinsics parameters (the camera pose relative to the robot)
extrinsics = PoseVector(0, 0, 0, 0, 0, 0)
if args.emc_file:
  with open(args.emc_file, "r") as file:
    i = 0

    for line in file:
      line = line.strip()

      if not line or line.startswith("#"):
        continue

      extrinsics[i] = float(line)
      i += 1

      if i >= 6:
        break

# Set the Franka robot parameters
FRANKA_IP = args.robot_ip
if args.adaptive_gain:
  LAMBDA = AdaptiveGain(1.5, 0.4, 30.0)
else:
  LAMBDA = 0.5

# Set the AprilTags parameters
TAG_FAMILY = DetectorAprilTag.TAG_36h11
TAG_SIZE = args.tag_size

# Set the visual servoing parameters
DISTANCE_TO_TAG = args.distance_to_tag # in meters
if args.no_convergence_threshold:
  CONVERGENCE_THRESHOLD = 0.0
else:
  CONVERGENCE_THRESHOLD = args.convergence_threshold



try:
  # -----------------------------------------------------------------------------
  # Camera initialization
  # -----------------------------------------------------------------------------

  # Configure the camera stream
  config = rs.config()
  config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.rgb8, CAMERA_FPS)

  # Start streaming frames from the camera
  pipeline = rs.pipeline()
  profile = pipeline.start(config)

  # Create and initialize the display image
  image = ImageRGBa(IMAGE_HEIGHT, IMAGE_WIDTH)
  display = get_display()
  display.init(image, 0, 0, "Camera view")

  # Access the display image data as a NumPy array
  image_array = np.asarray(image)

  # Create a grayscale image for detection calcul
  image_gray = ImageGray(IMAGE_HEIGHT, IMAGE_WIDTH)

  # Retrieve the camera intrinsics parameters
  intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
  camera = CameraParameters(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

  # Initialize the AprilTag detection
  detector = DetectorAprilTag(TAG_FAMILY)

  # -----------------------------------------------------------------------------
  # Franka initialization
  # -----------------------------------------------------------------------------

  # Get the camera extrinsics parameters
  extrinsics_matrix = HomogeneousMatrix(extrinsics)

  # Connect and initialize the Franka robot
  robot = RobotFranka()
  robot.connect(FRANKA_IP, RobotFranka.RealtimeConfig.kEnforce)
  robot.set_eMc(extrinsics_matrix)
  robot.setRobotState(Robot.STATE_VELOCITY_CONTROL)

  # Create the current and desired position
  centroid = FeaturePoint3D()
  centroid_desired = FeaturePoint3D()
  centroid_desired.buildFrom(0, 0, DISTANCE_TO_TAG)

  # Create the visual servoing task
  task = Servo()
  task.addFeature(centroid, centroid_desired)
  task.setServo(Servo.EYEINHAND_CAMERA)
  task.setInteractionMatrixType(Servo.CURRENT)

  # Set the lambda of the control law
  task.setLambda(LAMBDA)

  trajectory = []
  error = float('inf')
  can_move = False
  start_time = measureTimeMs()

  while True:
    # -----------------------------------------------------------------------------
    # Image acquisition
    # -----------------------------------------------------------------------------

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

    # -----------------------------------------------------------------------------
    # Visual servoing
    # -----------------------------------------------------------------------------

    # Detect the tags on the image
    _, tag_poses = detector.detect(image_gray, TAG_SIZE, camera)
    tag_corners = detector.getTagsCorners()

    # Check if only one tag is detected
    if detector.getNbObjects() == 1:
      # Get the tag position
      tag_position = tag_poses[0]

      # Get the tag projection on the camera frame
      tag_projection = ColVector()
      centroid_3d = Point(0, 0, 0)
      centroid_3d.changeFrame(tag_position, tag_projection)
      centroid.buildFrom(tag_projection[0], tag_projection[1], tag_projection[2])

      # Compute the robot movements
      velocity = task.computeControlLaw()

      # Compute the error
      error = task.getError().sumSquare()

      # Stop the robot if the threshold is passed
      if error < CONVERGENCE_THRESHOLD:
        can_move = False
        velocity = ColVector(6, 0)

    else :
      velocity = ColVector(6, 0)

    # Move the robot
    if can_move:
      robot.setVelocity(Robot.CAMERA_FRAME, velocity)
    else:
      robot.setVelocity(Robot.CAMERA_FRAME, ColVector(6, 0))

    # -----------------------------------------------------------------------------
    # Display camera stream
    # -----------------------------------------------------------------------------

    # Render the updated image
    Display.display(image)

    # Display the tag frame
    detector.displayTags(image, tag_corners, Color.none, 3)

    # Display the servoing
    ServoDisplay.display(task, camera, image, Color.green, Color.red)

    # Display the trajectory
    if can_move and detector.getNbObjects() == 1:
      trajectory.append(detector.getCog(0))
      
    for i in range(len(trajectory)-1):
      Display.displayLine(image, trajectory[i], trajectory[i+1], Color.blue, 2)

    # Print status information on the display
    if can_move:
      text_color = Color.green
      Display.displayText(image, 20, 20, "Left click to stop movement. Right click to stop.", text_color)
    else:
      text_color = Color.red
      Display.displayText(image, 20, 20, "Left click to resume movement. Right click to stop.", text_color)

    # Print the elapsed time
    Display.displayText(image, 40, 20, "Elapsed time: " + str(round((measureTimeMs() - start_time))/1000) + " s", text_color)

    # Print the robot state and velocity
    Display.displayText(image, 300, 20, "Moving: " + str(can_move), text_color)
    Display.displayText(image, 320, 20, "Error: " + str(round(error, 5)), text_color)
    Display.displayText(image, 340, 20, "Velocity:", text_color)
    for i in range(velocity.size()):
      Display.displayText(image, 360 + 20*i, 20, str(round(velocity[i], 3)), text_color)

    Display.flush(image)

    # -----------------------------------------------------------------------------
    # End of loop
    # -----------------------------------------------------------------------------

    # Check for the user click
    button = MouseButton.MouseButtonType(0)
    Display.getClick(image, button, False)

    # Switch the robot movement mode or quit the loop depending of the button clicked
    if button == MouseButton.MouseButtonType.button3:
      break
    elif button == MouseButton.MouseButtonType.button1:
      if can_move:
        can_move = False
      else:
        can_move = True
        trajectory = []
        frame_number = 0

except Exception as error:
 print(f"Exception raised: {error}")
 raise
"""
finally:
  # Stop the robot
  try:
    robot.setRobotState(Robot.STATE_STOP)
    print("Robot successfully stopped")
  except Exception as error:
    print(f"Exception raised: {error}")
    raise

  # Stop the camera
  try:
    pipeline.stop()
  except Exception as error:
    print(f"Exception raised: {error}")
    raise
"""
