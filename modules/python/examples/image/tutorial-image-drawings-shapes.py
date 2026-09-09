import numpy as np
import matplotlib.pyplot as plt

# ViSP Python bindings
from visp.core import ImageGray
from visp.core import ImageDraw, ImagePoint, Rect, Font

# Create a Matplotlib figure
fig, axes = plt.subplots(2, 3)
axes = axes.ravel()

# Create subplots for the figure
def create_subplot(i, title):
	axes[i].set_title(title)
	axes[i].axis('off')
	axes[i].imshow(I[i], interpolation='nearest', cmap="gray", vmin=0, vmax=255)

# Create 6 black images
I = [ImageGray(100, 100, 0) for _ in range(6)]

# Draw a point
ip = ImagePoint(50, 50)
ImageDraw.drawPoint(I[0], ip, 255, 3)
create_subplot(0, "Point")

# Draw a line
ip1 = ImagePoint(25, 25)
ip2 = ImagePoint(75, 75)
ImageDraw.drawLine(I[1], ip1, ip2, 255, 3)
create_subplot(1, "Line")

# Draw a circle
ip = ImagePoint(50, 50)
ImageDraw.drawCircle(I[2], ip, 40, 255, 3)
create_subplot(2, "Circle")

# Draw a rectangle
ip = ImagePoint(25, 10)
w = 80
h = 50
ImageDraw.drawRectangle(I[3], Rect(ip, w, h), 255, 3)
create_subplot(3, "Rectangle")

# Draw a cross
ip = ImagePoint(50, 50)
ImageDraw.drawCross(I[4], ip, 25, 255, 3)
create_subplot(4, "Cross")

# Insert text
ip = ImagePoint(43, 25)
color = 255 # white
background = 0 # black
font = Font(14, Font.FontFamily.GENERIC_MONOSPACE)
font.drawText(I[5], "Test...", ip, color, background)
create_subplot(5, "Text")

# Display results
plt.show()