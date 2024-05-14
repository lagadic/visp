

from typing import Optional, Tuple
from visp.core import ColVector, Point, Color, PixelMeterConversion, Display
from visp.core import CameraParameters, HomogeneousMatrix , ExponentialMap, PoseVector

from visp.core import ImageRGBa
from visp.robot import ImageSimulator
from visp.visual_features import BasicFeature, FeaturePoint
from visp.vs import Servo
from visp.gui import DisplayOpenCV

try:
  from ultralytics import YOLO
except ImportError:
  print('This example requires yoloV8: pip install ultralytics')

import time
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

plt.rcParams['text.usetex'] = True

def get_simulator(scene_image: ImageRGBa) -> ImageSimulator:
  simulator = ImageSimulator()
  l = 1.5
  L = 1
  scene_3d = [
    [-l, -L, 0.0],
    [l, -L, 0.0],
    [l, L, 0.0],
    [-l, L, 0.0],
  ]
  simulator.init(scene_image, list(map(lambda X: Point(X), scene_3d)))
  simulator.setCleanPreviousImage(True, color=Color.black)
  simulator.setInterpolationType(ImageSimulator.InterpolationType.BILINEAR_INTERPOLATION)
  return simulator

class VSPlot(object):
  def __init__(self):
    self.v = []
    self.error = []
    self.r = []
    self.I = []

  def on_iter(self, Idisp: ImageRGBa, v: ColVector, error: ColVector, cTw: HomogeneousMatrix) -> None:
    self.I.append(Idisp)
    self.v.append(v.numpy()[3:5].flatten())
    self.error.append(error.numpy().flatten())
    self.r.append(PoseVector(cTw).numpy()[3:5].flatten())

  def generate_anim(self):
    self.error = np.asarray(self.error)[1:]
    self.v = np.asarray(self.v)[1:]
    self.r = np.asarray(self.r)[1:]


    fig, axs = plt.subplots(2, 2, figsize=(15, 15 * (self.I[0].getHeight() / self.I[0].getWidth())))
    axs = [axs[0][0], axs[0][1], axs[1][0],axs[1][1]]
    titles = ['I', 'Feature error', 'Velocity', 'Pose']
    legends = [
      None,
      [r"$x$", r"$y$"],
      [r"$\mathbf{\upsilon}_x$", r"$\mathbf{\upsilon}_y$"],
      [r"$\theta\mathbf{u}_x$", r"$\theta\mathbf{u}_y$"],
    ]
    data = [None, self.error, self.v, self.r]
    artists = []
    for i in range(len(axs)):
      axs[i].set_title(titles[i])
      if data[i] is not None:
        axs[i].set_xlabel('Iteration')
        axs[i].grid()
        axs[i].set_xlim(0, len(self.v))
        min_val, max_val =  np.min(data[i]), np.max(data[i])
        margin = (max_val - min_val) * 0.05
        axs[i].set_ylim(min_val - margin, max_val + margin)
        artists.append(axs[i].plot(data[i]))
        axs[i].legend(legends[i])
      else:
        artists.append(axs[i].imshow(self.I[0]))
        axs[i].set_axis_off()
    plt.tight_layout()
    def animate(i):
      xs = range(i)
      artists[0].set_data(self.I[i])
      for j in range(2):
        artists[1][j].set_data(xs, self.error[:i, j])
        artists[2][j].set_data(xs, self.v[:i, j])
        artists[3][j].set_data(xs, self.r[:i, j])
      return artists

    anim = animation.FuncAnimation(fig, animate, frames=len(self.v), interval=30, blit=False, repeat=False)
    writervideo = animation.FFMpegWriter(fps=30)
    anim.save('exp.mp4', writer=writervideo)
    plt.savefig('exp.pdf')
    plt.close()

if __name__ == '__main__':
  h, w = 480, 640
  Z = 5.0
  cam = CameraParameters(px=600, py=600, u0=w / 2.0, v0=h / 2.0)
  detection_model = YOLO('yolov8n.pt')
  # Initialize simulator
  scene_image = np.asarray(Image.open('/mnt/d/Downloads/car-img.jpg'))
  scene_image = np.concatenate((scene_image, np.ones_like(scene_image[..., 0:1]) * 255), axis=-1)
  scene_image = ImageRGBa(scene_image)
  simulator = get_simulator(scene_image)

  plotter = VSPlot()

  cTw = HomogeneousMatrix(-1.0, 0.5, Z, 0.0, 0.0, 0.0)
  I = ImageRGBa(h, w)
  Idisp = ImageRGBa(h, w)

  simulator.setCameraPosition(cTw)
  simulator.getImage(I, cam)

  s = FeaturePoint()
  s.buildFrom(0.0, 0.0, Z)
  # Define centering task
  xd, yd = PixelMeterConversion.convertPoint(cam, w / 2.0, h / 2.0)
  sd = FeaturePoint()
  sd.buildFrom(xd, yd, Z)

  task = Servo()
  task.addFeature(s, sd)
  task.setLambda(0.5)
  task.setCameraDoF(ColVector([0, 0, 0, 1, 1, 0]))
  task.setServo(Servo.ServoType.EYEINHAND_CAMERA)
  task.setInteractionMatrixType(Servo.ServoIteractionMatrixType.DESIRED)
  prev_v = ColVector(6, 0.0)
  target_class = 2 # Car

  d = DisplayOpenCV()
  d.init(I)
  error_norm = 1e10
  # Servoing loop
  while error_norm > 5e-6:
    start = time.time()
    # Data acquisition
    simulator.getImage(I, cam)
    # Build current features
    results = detection_model(np.array(I.numpy()[..., 2::-1]))
    boxes = map(lambda result: result.boxes, results)
    boxes = filter(lambda box: box.cls is not None and len(box.cls) > 0 and box.cls[0] == target_class, boxes)
    boxes = sorted(boxes, key=lambda box: box.conf[0])
    bbs = list(map(lambda box: box.xywh[0].cpu().numpy(), boxes))
    if len(bbs) > 0:
      bb = bbs[-1] # Take highest confidence
      u, v = bb[0], bb[1]
      x, y = PixelMeterConversion.convertPoint(cam, u, v)
      s.buildFrom(x, y, Z)
      v = task.computeControlLaw()
      prev_v = v
    else:
      v = prev_v
    error: ColVector = task.getError()
    error_norm = error.sumSquare()

    # Display and logging
    Display.display(I)
    sd.display(cam, I, Color.green)
    s.display(cam, I, Color.red)
    Display.getImage(I, Idisp)
    Display.flush(I)
    plotter.on_iter(Idisp, v, error, cTw)

    # Move robot/update simulator
    cTcn = ExponentialMap.direct(v, time.time() - start)
    cTw = cTcn.inverse() * cTw
    simulator.setCameraPosition(cTw)

  plotter.generate_anim()
