import sys

# ViSP Python bindings
from visp.core import ImageGray, ImageRGBa
from visp.io import ImageIo
from visp.core import Display
from visp.python.display_utils import get_display

path = sys.path[0] + "/monkey.jpeg"

# Read the image 
I = ImageRGBa()
try:
  ImageIo.read(I, path)
except:
  print(f"Cannot read image {path}")
  sys.exit()

# Display the image
d = get_display()
d.init(I)
Display.setTitle(I, "monkey.jpeg")
Display.display(I)
Display.flush(I)

# Wait for user input
print("A click to quit...")
Display.getClick(I)