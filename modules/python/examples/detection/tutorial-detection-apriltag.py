import sys
from pathlib import Path

from visp.core import ImageGray, Color, CameraParameters
from visp.io import ImageIo
from visp.detection import DetectorAprilTag
from visp.core import Display
from visp.python.display_utils import get_display

# Configuration
IMAGE_PATH = str(Path(__file__).parent) + "/AprilTag.png"
TAG_SIZE = 0.053 # in meters

# Camera intrinsics parameters
CAMERA_PX = 615.1674805 # horizontal focal length
CAMERA_PY = 615.1675415 # vertical focal length
CAMERA_U0 = 312.1889954 # principal point x
CAMERA_V0 = 243.4373779 # principal point y

# Create the camera model
camera = CameraParameters()
camera.initPersProjWithoutDistortion(CAMERA_PX, CAMERA_PY, CAMERA_U0, CAMERA_V0)

# Read the image
image = ImageGray()

try:
  ImageIo.read(image, IMAGE_PATH)
except Exception as exception:
  print(f"Could not read image: {exception}")
  sys.exit(1)

# Detect the tags on the image
detector = DetectorAprilTag()
detected, tag_poses = detector.detect(image, TAG_SIZE, camera)

number_of_tags = detector.getNbObjects()
print(f"Detected {number_of_tags} AprilTag(s)")

if not detected:
  print("No AprilTags found.")
  sys.exit(0)

# Get and print the tag information
tags_ids = detector.getTagsId()
print("Tag IDs:", tags_ids)
print("Decision margins:", detector.getTagsDecisionMargin())
print("Hamming distances:", detector.getTagsHammingDistance())

# Get and print the tag cog and corners
tag_corners = detector.getTagsCorners()
for i in range(number_of_tags):
  print(f"Tag {i} with ID {tags_ids[i]} has cog: {detector.getCog(i)} and has 4 corners: {tag_corners[i]}")

# Display the image with tags frames and axes
display = get_display()
display.init(image, 0, 0, "Image with AprilTags frames and axes")
Display.display(image)
detector.displayTags(image, tag_corners, Color.none, 3)

# Draw the tags borders and coordinate frames
detector.displayFrames(image, tag_poses, camera, TAG_SIZE / 2.0, Color.none, 3)

Display.flush(image)
Display.getClick(image)
