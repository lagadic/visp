import sys

# [imports]
import matplotlib.pyplot as plt
# [end-imports]

# ViSP Python bindings
from visp.core import ImageRGBa
from visp.io import ImageIo

# Image path
path = sys.path[0] + "/monkey.jpeg"

# Read the image 
I = ImageRGBa()
try:
  ImageIo.read(I, path)
except:
  print(f"Cannot read image {path}")
  sys.exit()

# [display-image]
# Display using Matplotlib
plt.imshow(I)
plt.axis('off')
plt.title("monkey.jpeg")
plt.show()
# [end-display-image]