# ViSP Python bindings
from visp.core import ImageGray, ImageCircle, ImagePoint, Color
from visp.io import ImageIo
from visp.core import Display
from visp.python.display_utils import get_display

# Create a gray image
I = ImageGray(2160, 3840, 0)
d = get_display()
d.init(I)
Display.setTitle(I, "Default scale")

# Draw a red circle on the image
Display.display(I)
Display.displayCircleStatic(I, ImageCircle(ImagePoint(I.getHeight()/2, I.getWidth()/2), 200), Color.red, True)
Display.flush(I)

# Wait for user input
print("A click to quit...")
d.getClick(I)
