import sys
from pathlib import Path

from visp.core import ImageGray, ImageRGBa, ImageConvert
from visp.core import Color, ImageHSVucFS
from visp.core import Display
from visp.core import ImageTools
from visp.python.display_utils import get_display

from visp.io import ImageIo

# Function displaying an image
def display(I, title, block, winx, winy):
  # Display the image
  Display.setTitle(I, title)
  Display.display(I)
  Display.setWindowPosition(I, winx=winx, winy = winy)
  Display.flush(I)

  # Wait for user input
  if block:
    Display.displayText(I=I, i=20,j=20, color=Color.red, s="A click to quit...")
    Display.getClick(I, blocking=block)

# Read the image
inputPath = str(Path(__file__).parent) + "/ballons.jpg"
I = ImageRGBa()
try:
  ImageIo.read(I, inputPath)
  print(f"Successfully loaded image: {inputPath}")
except Exception as e:
  print(e)
  sys.exit()

# Definition of the HSV boundaries
h_threshold = 14
s_threshold = 255
v_threshold = 209
offset = 30
h_low = max(0, h_threshold - offset)
h_high = min(h_threshold + offset, 255)
s_low = max(0, s_threshold - offset)
s_high = min(s_threshold + offset, 255)
v_low = max(0, v_threshold - offset)
v_high = min(v_threshold + offset, 255)
hsv_range = [h_low, h_high, s_low, s_high, v_low, v_high]
# Converting the image
Ihsv = ImageHSVucFS()
ImageConvert.convert(I, Ihsv)
# Computing the mask
maskHSV = ImageGray()
ImageTools.inRange(Ihsv, hsv_range, maskHSV)
#Performing the segmentation
I_segmented = ImageRGBa()
ImageTools.inMask(I, mask=maskHSV, I_mask=I_segmented)
# Display the loaded image
d_rgba = get_display()
d_rgba.init(I)
display(I, "Loaded image", block=False, winx=0, winy=0)
# Display the HSV mask
d_mask = get_display()
d_mask.init(maskHSV)
display(maskHSV, "Mask", block=False, winx=I.getCols() + 20, winy=0)
# Display the segmented image
d_seg = get_display()
d_seg.init(I_segmented)
display(I_segmented, "Result of the segmentation", block=True, winx=0, winy=I.getRows() + 20)
