import sys

from visp.core import ImageRGBa
from visp.io import ImageIo

from visp.core import Display
from visp.python.display_utils import get_display

# Image path
path = sys.path[0] + "/monkey.jpeg"

# Read the image
I = ImageRGBa()
try:
  ImageIo.read(I, path)
except:
  print(f"Cannot read image {path}")
  sys.exit()

# Create the display
d = get_display()
d.init(I)
Display.setTitle(I, "monkey.jpeg")

# Display the image
Display.display(I)
Display.flush(I)

# Wait for user input
print("A click to quit...")
Display.getClick(I)

