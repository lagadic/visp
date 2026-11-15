import sys
from pathlib import Path

from visp.core import ImageRGBa, Rect, ImagePoint, Color
from visp.io import ImageIo
from visp.core import Display
from visp.python.display_utils import get_display

# Read the image
path = str(Path(__file__).parent) + "/monkey.jpeg"
I = ImageRGBa()
try:
  ImageIo.read(I, path)
except:
  print(f"Cannot read image {path}")
  sys.exit()

# Display the image
d = get_display()
d.init(I)
Display.setTitle(I, "Default scale")

# Draw the image on the display
Display.display(I)

# Draw a rectangle on the display
rect = Rect(ImagePoint(I.getHeight()/4, I.getWidth()/4), 128, 128)
Display.displayRectangle(I, rect, Color.black, True)

Display.flush(I)

# Wait for user input
print("A click to quit...")
d.getClick(I)
