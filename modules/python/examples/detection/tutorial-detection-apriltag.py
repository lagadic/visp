import sys
import time

from visp.core import ImageGray, ImageRGBa, Color, CameraParameters
from visp.io import ImageIo

from visp.detection import DetectorAprilTag

from visp.core import Display
from visp.python.display_utils import get_display

# 
#INPUT_PATH = "apriltag2.png"
INPUT_PATH = "visp-images/AprilTag/AprilTag.png"
TAG_SIZE = 0.053

# Initialize the camera
camera = CameraParameters()
camera.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779)

# Read the image
image = ImageGray()
try:
  ImageIo.read(image, INPUT_PATH)
except Exception as exception:
  print(f"Could not read image: {exception}")
  sys.exit(1)

# Detect tags on the image
detector = DetectorAprilTag()
detected, tag_poses = detector.detect(image, TAG_SIZE, camera)
print(f"detected {detector.getNbObjects()} tags")

if not detected:
  sys.exit()

# Get tags informations
print("Tag IDs:", detector.getTagsId())
print("Decision margins:", detector.getTagsDecisionMargin())
print("Hamming distances:", detector.getTagsHammingDistance())

corners = detector.getTagsCorners()

# Display the image with tags boxes
display = get_display()
display.init(image)
Display.display(image)
detector.displayTags(image, corners, Color.none, 3)
detector.displayFrames(image, tag_poses, camera, TAG_SIZE / 2.0, Color.none, 3)
Display.flush(image)
Display.getClick(image)