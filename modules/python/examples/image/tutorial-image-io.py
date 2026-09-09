import sys

from visp.core import ImageGray, ImageRGBa, ImageConvert
from visp.core import Display
from visp.python.display_utils import get_display

from visp.io import ImageIo

# Function displaying an image
def display(I, title):
  # Display the image
  d = get_display()
  d.init(I)
  Display.setTitle(I, title)
  Display.display(I)
  Display.flush(I)

  # Wait for user input
  print("A click to quit...")
  Display.getClick(I)

# Read the image
inputPath = sys.path[0] + "/monkey.jpeg"
I = ImageRGBa()
try:
  ImageIo.read(I, inputPath)
  print(f"Successfully loaded image: {inputPath}")
except Exception as e:
  print(e)
  sys.exit()

# Display the loaded image
display(I, "Loaded image")

# Convert the image into a grayscale image
Igray = ImageGray()
ImageConvert.convert(I, Igray)

# Write the image
outputPath = sys.path[0] + "/grayscale_monkey.jpeg"
try:
  ImageIo.write(Igray, outputPath)
  print(f"Image successfully written to '{outputPath}'")
except Exception as e:
  print(e)

# Display the written image
display(Igray, "Written image")
