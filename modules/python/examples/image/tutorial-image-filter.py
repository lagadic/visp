import sys
from pathlib import Path
import matplotlib.pyplot as plt

from visp.core import ImageGray, ImageDouble, ImageConvert
from visp.io import ImageIo

from visp.core import ImageFilter

# Read the image as a grayscale image
inputPath = str(Path(__file__).parent) + "/monkey.jpeg"
I = ImageGray()
I2 = ImageGray()
try:
  ImageIo.read(I, inputPath)
  ImageIo.read(I2, inputPath)
except Exception as e:
  print(e)
  sys.exit()

# Apply a gaussian blur filter to the image
IBlur = ImageDouble()
ImageFilter.gaussianBlur(I2, IBlur, 7, 0)

# Convert the filtered image to a grayscale image
IOut = ImageGray()
ImageConvert.convert(IBlur, IOut)

# Display the two images
fig, axes = plt.subplots(1, 2)

# Original image
axes[0].set_title("Original image")
axes[0].axis('off')
axes[0].imshow(I, interpolation='nearest', cmap="gray", vmin=0, vmax=255)

# Filtered image
axes[1].set_title("Filtered image")
axes[1].axis('off')
axes[1].imshow(IOut, interpolation='nearest', cmap="gray", vmin=0, vmax=255)

plt.show()
