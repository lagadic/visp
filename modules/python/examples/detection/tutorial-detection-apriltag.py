from visp.core import ImageRGBa, ImageGray
from visp.io import ImageIo

from visp.core import Display
from visp.python.display_utils import get_display

from visp.detection import DetectorAprilTag

# Read the image
#inputPath = str(Path(__file__).parent) + "apriltag2.png"
inputPath = "apriltag2.png"
image = ImageGray()
try:
  ImageIo.read(image, inputPath)
except Exception as e:
  print(e)
  sys.exit()

display = get_display()
display.init(image)
Display.display(image)
Display.flush(image)
Display.getClick(image)

detector = DetectorAprilTag()
detector.detect(image)
print(f"detected {detector.getNbObjects()} tags")

corners = detector.getTagsCorners()
detector.displayTags(image, corners)