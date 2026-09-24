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

##########
# Parser
##########

parser = argparse.ArgumentParser(description='Python wrapper example over vpRobotFranka ViSP class')
parser.add_argument('--ip', type=str, default="192.168.30.10", dest='robot_ip', help='Robot IP address: --robot_ip [ip]')

args, unknown_args = parser.parse_known_args()
if unknown_args:
  print(f"The following arguments are not recognized and will not be used: {unknown_args}")
  print('Exiting...')
  sys.exit(1)

# -----------------------------------------------------------------------------
# Program parameters
# -----------------------------------------------------------------------------

# Set the camera configuration
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_FPS = 60

# Set the camera extrinsics parameters (the camera pose relative to the robot)
CAMERA_TX = 0.05963842756 # in meters
CAMERA_TY = -0.04413103437
CAMERA_TZ = 0.04261230688
CAMERA_TUX = math.radians(-0.721) # in radiants
CAMERA_TUY = math.radians(0.168)
CAMERA_TUZ = math.radians(44.123)

# Set the Franka robot parameters
FRANKA_IP = "192.168.100.3"
LAMBDA = 0.5
LAMBDA = AdaptiveGain(1.5, 0.4, 30.0)

# Set the AprilTags parameters
TAG_FAMILY = DetectorAprilTag.TAG_36h11
TAG_SIZE = 0.053 # in meters

# Set the visual servoing parameters
DISTANCE_TO_TAG = 0.2 # in meters
#CONVERGENCE_THRESHOLD = 0.00005
CONVERGENCE_THRESHOLD = 0.00001

try:
  ##########
  # Camera
  ##########

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

  ##########
  # Franka
  ##########

  # Get the camera extrinsics parameters
  extrinsics = PoseVector(CAMERA_TX, CAMERA_TY, CAMERA_TZ, CAMERA_TUX, CAMERA_TUY, CAMERA_TUZ)
  extrinsics_matrix = HomogeneousMatrix(extrinsics)

  robot = RobotFranka()

  # Create the current and desired position
  centroid = FeaturePoint3D()
  centroid_desired = FeaturePoint3D()
  centroid_desired.buildFrom(0, 0, DISTANCE_TO_TAG)

  # Connect and initialize the Franka robot
  robot.connect(FRANKA_IP, RobotFranka.RealtimeConfig.kEnforce)
  robot.set_eMc(extrinsics_matrix)
  robot.setRobotState(Robot.STATE_VELOCITY_CONTROL)

  # Create the visual servoing task
  task = Servo()
  task.addFeature(centroid, centroid_desired)
  task.setServo(Servo.EYEINHAND_CAMERA)
  task.setInteractionMatrixType(Servo.CURRENT)

  # Set the lambda of the control law
  task.setLambda(LAMBDA)

  trajectory = []
  frame_number = 0
  error = float('inf')
  can_move = False

  # Start the time measurement
  start_time = measureTimeMs()

  while True:
    ##########
    # Image acquisition
    ##########

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

    ##########
    # Visual servoing
    ##########

    # Detect the tags on the image
    _, tag_poses = detector.detect(image_gray, TAG_SIZE, camera)
    tag_corners = detector.getTagsCorners()

    # Check if only one tag is detected
    if can_move and detector.getNbObjects() == 1:
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

      # Save the position center of the tag
      if frame_number % 10 == 0:
        trajectory.append(detector.getCog(0))
      frame_number += 1

    else :
      velocity = ColVector(6, 0)

    # Move the robot
    robot.setVelocity(Robot.CAMERA_FRAME, velocity)

    ##########
    # Display
    ##########

    # Render the updated image
    Display.display(image)

    # Display the tag frame
    detector.displayTags(image, tag_corners, Color.none, 3)

    # Display the servoing
    ServoDisplay.display(task, camera, image, Color.green, Color.red)

    # Display the trajectory
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

    ##########
    # End of loop
    ##########

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

