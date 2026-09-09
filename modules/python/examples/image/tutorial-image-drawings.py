import sys
from pathlib import Path

from visp.core import ImageRGBa
from visp.io import ImageIo
from visp.core import Display
from visp.python.display_utils import get_display

from visp.core import ImageDraw, ImageCircle, ImagePoint, Color

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
inputPath = str(Path(__file__).parent) + "/monkey.jpeg"
I = ImageRGBa()
try:
  ImageIo.read(I, inputPath)
  print(f"Successfully loaded image: {inputPath}")
except Exception as e:
  print(e)
  sys.exit()

# Display the original image
display(I, "Original image")

# Draw a circle on the image
ImageDraw.drawCircle(I, ImageCircle(ImagePoint(I.getHeight()/2, I.getWidth()/2), 50), Color.black, 50)

# Display the altered image
display(I, "Altered image")
