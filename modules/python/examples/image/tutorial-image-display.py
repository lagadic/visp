import sys

# [imports]
# ViSP Python bindings
from visp.core import ImageRGBa
from visp.io import ImageIo

from visp.core import Display
from visp.python.display_utils import get_display
# [end-imports]

# [read-image]
# Image path
path = sys.path[0] + "/monkey.jpeg"

# Read the image
I = ImageRGBa()
try:
  ImageIo.read(I, path)
except:
  print(f"Cannot read image {path}")
  sys.exit()
# [end-read-image]

# [initialize-display]
# Display the image
d = get_display()
d.init(I)
Display.setTitle(I, "monkey.jpeg")
# [end-initialize-display]

# [display-image]
Display.display(I)
Display.flush(I)
# [end-display-image]

# [wait-for-click]
# Wait for user input
print("A click to quit...")
Display.getClick(I)
# [end-wait-for-click]
