import numpy as np
import matplotlib.pyplot as plt

from visp.core import ImageGray
from visp.core import ImageDraw, ImagePoint, Rect, Font

# Create a Matplotlib figure
fig, axes = plt.subplots(2, 4)
axes = axes.ravel()

# Create subplots for the figure
def create_subplot(i, title):
  axes[i].set_title(title)
  axes[i].axis('off')
  axes[i].imshow(I[i], interpolation='nearest', cmap="gray", vmin=0, vmax=255)

# Create 8 black images
I = [ImageGray(200, 200, 0) for _ in range(8)]

# Draw a point
ip = ImagePoint(100, 100)
ImageDraw.drawPoint(I[0], ip, 255, 3)
create_subplot(0, "Point")

# Draw a line
ip1 = ImagePoint(50, 50)
ip2 = ImagePoint(150, 150)
ImageDraw.drawLine(I[1], ip1, ip2, 255, 3)
create_subplot(1, "Line")

# Draw a circle
ip = ImagePoint(100, 100)
ImageDraw.drawCircle(I[2], ip, 80, 255, 3)
create_subplot(2, "Circle")

# Draw a rectangle
ip = ImagePoint(50, 20)
w = 160
h = 100
ImageDraw.drawRectangle(I[3], Rect(ip, w, h), 255, 3)
create_subplot(3, "Rectangle")

# Draw a polygon
points = [
  ImagePoint(100, 20),
  ImagePoint(176, 75),
  ImagePoint(147, 165),
  ImagePoint(53, 165),
  ImagePoint(24, 75),
]
ImageDraw.drawPolygon(I[4], points, 255, 3)
create_subplot(4, "Polygon")

# Draw a cross
ip = ImagePoint(100, 100)
ImageDraw.drawCross(I[5], ip, 50, 255, 3)
create_subplot(5, "Cross")

# Draw an arrow
start = ImagePoint(100, 50)
end = ImagePoint(100, 150)
ImageDraw.drawArrow(I[6], start, end, 255, 25, 25, 3)
create_subplot(6, "Arrow")

# Insert text
ip = ImagePoint(86, 50)
color = 255 # white
background = 0 # black
font = Font(28, Font.FontFamily.GENERIC_MONOSPACE)
font.drawText(I[7], "Test...", ip, color, background)
create_subplot(7, "Text")

# Display results
plt.show()
