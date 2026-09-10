import sys
from pathlib import Path
import matplotlib.pyplot as plt

from visp.core import ImageRGBa
from visp.io import ImageIo
from visp.core import Display
from visp.python.display_utils import get_display

from visp.core import ImageDraw, Rect, ImagePoint, Color

# Read the image
inputPath = str(Path(__file__).parent) + "/monkey.jpeg"
I = ImageRGBa()
I2 = ImageRGBa()
try:
  ImageIo.read(I, inputPath)
  ImageIo.read(I2, inputPath)
except Exception as e:
  print(e)
  sys.exit()

# Insert a rectangle on the image
rect = Rect(ImagePoint(I2.getHeight()/4, I2.getWidth()/4), 128, 128)
ImageDraw.drawRectangle(I2, rect, Color.black, True)

# Display the two images
fig, axes = plt.subplots(1, 2)

# Original image
axes[0].set_title("Original image")
axes[0].axis('off')
axes[0].imshow(I, interpolation='nearest')

# Altered image
axes[1].set_title("Altered image")
axes[1].axis('off')
axes[1].imshow(I2, interpolation='nearest')

plt.show()
