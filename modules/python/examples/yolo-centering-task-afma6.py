

import visp
from visp.core import ColVector, Color, PixelMeterConversion, Display, Matrix
from visp.core import CameraParameters, HomogeneousMatrix , PoseVector, ImagePoint
from visp.core import ImageRGBa, ImageUInt16
from visp.core import UnscentedKalman, UKSigmaDrawerMerwe

from visp.visual_features import BasicFeature, FeaturePoint
from visp.vs import Servo
from visp.gui import ColorBlindFriendlyPalette
from visp.display_utils import get_display
from visp.robot import RobotAfma6, Robot

from typing import List, Optional, Tuple
try:
  from ultralytics import YOLO
except ImportError:
  print('This example requires YoloV8: pip install ultralytics')

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import argparse
import pyrealsense2 as rs
from functools import partial
plt.rcParams['text.usetex'] = False



def fx(x: ColVector, dt: float) -> ColVector:
  """
  @brief Process function that projects in time the internal state of the UKF.

  @param x The internal state of the UKF.
  @param dt The sampling time: how far in the future are we projecting x.

  @return ColVector The updated internal state, projected in time, also known as the prior.
  """
  return ColVector([
  	x[0] + dt * x[1] + dt ** 2 * x[2],
  	x[1] + dt * x[2],
    x[2],
  	x[3] + dt * x[4] + dt ** 2 * x[5],
  	x[4] + dt * x[5],
    x[5]
  ])


def hx(x: ColVector) -> ColVector:
  """
  @brief Measurement function that expresses the internal state of the UKF in the measurement space.

  @param x The internal state of the UKF.

  @return ColVector The internal state, expressed in the measurement space.
  """
  return ColVector([
  	x[0],
	  x[3]
  ])

def add_state_vectors(a, b) -> ColVector:
  """
  @brief Method that permits to add two state vectors.

  @param a The first state vector to which another state vector must be added.
  @param b The other state vector that must be added to a.

  @return ColVector The sum a + b.
  """
  return a + b

def residual_state_vectors(a, b) -> ColVector:
  """
  @brief Method that permits to substract a state vector to another.

  @param a The first state vector to which another state vector must be substracted.
  @param b The other state vector that must be substracted to a.

  @return ColVector The substraction a - b.
  """
  return a - b

def generate_Q_matrix(dt: float, sigma: float) -> Matrix:
  """
  @brief Method that generates the process covariance matrix for a process for which the
  state vector can be written as (x, dx/dt)^T

  @param dt The sampling period.

  @return Matrix The corresponding process covariance matrix.
  """
  return Matrix(np.asarray([
    [dt**4/4, dt**3/3, dt**2/2, 0, 0, 0],
    [dt**3/3, dt**2/2, dt, 0, 0, 0],
    [dt**2/2, dt, 1, 0, 0, 0],
    [0, 0, 0, dt**4/4, dt**3/3, dt**2/2],
    [0, 0, 0, dt**3/3, dt**2/2, dt],
    [0, 0, 0, dt**2/2, dt, 1],
  ]) * sigma)



def read_data(I_rgba: ImageRGBa, I_depth: ImageUInt16, pipe: rs.pipeline, align: rs.align) -> None:
  frames = pipe.wait_for_frames()
  aligned_frames = align.process(frames)
  I_np = np.asanyarray(aligned_frames.get_color_frame().as_frame().get_data())
  I_np = np.concatenate((I_np, np.ones_like(I_np[..., 0:1], dtype=np.uint8) * 255), axis=-1)
  rgba_numpy_view = I_rgba.numpy() # writable numpy view of rgba image
  np.copyto(rgba_numpy_view, I_np)
  depth_numpy_view = I_depth.numpy()
  depth_np = np.asanyarray(aligned_frames.get_depth_frame().as_frame().get_data())
  np.copyto(depth_numpy_view, depth_np)

def cam_from_rs_profile(profile) -> Tuple[CameraParameters, int, int]:
  intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
  return CameraParameters(intr.fx, intr.fy, intr.ppx, intr.ppy), intr.height, intr.width

class VSPlot(object):
  def __init__(self):
    self.v = []
    self.error = []
    self.r = []
    self.I = []

  def on_iter(self, Idisp: ImageRGBa, v: ColVector, error: ColVector, cTw: HomogeneousMatrix) -> None:
    self.I.append(Idisp.numpy().copy())
    self.v.append(v.numpy()[3:5].flatten())
    self.error.append(error.numpy().flatten())
    self.r.append(PoseVector(cTw).numpy()[3:5].flatten())

  def generate_anim(self):
    self.error = np.asarray(self.error)[1:]
    self.v = np.asarray(self.v)[1:]
    self.r = np.asarray(self.r)[1:]


    fig, axs = plt.subplots(2, 2, figsize=(15, 15 * (self.I[0].shape[0] / self.I[0].shape[1])))
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
      print(f'Processing frame {i+1}/{len(self.I)}')
      xs = range(i)
      artists[0].set_data(self.I[i])
      for j in range(2):
        artists[1][j].set_data(xs, self.error[:i, j])
        artists[2][j].set_data(xs, self.v[:i, j])
        artists[3][j].set_data(xs, self.r[:i, j])
      return artists

    anim = animation.FuncAnimation(fig, animate, frames=len(self.v), blit=False, repeat=False)
    writervideo = animation.FFMpegWriter(fps=30)
    anim.save('exp.mp4', writer=writervideo)
    plt.savefig('exp.pdf')
    plt.close()

if __name__ == '__main__':
  parser = argparse.ArgumentParser('Centering task using a YOLO network')
  parser.add_argument('--class-id', type=int, help='COCO class id of the object to track (e.g, 2 for a car)')
  args = parser.parse_args()

  detection_model = YOLO('yolov8n.pt')

  plotter = VSPlot()

  # Initialization
  print('Initializing Realsense camera...')
  # Realsense2
  pipe = rs.pipeline()
  config = rs.config()
  fps = 30
  config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
  config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)
  cfg = pipe.start(config)

  # Initialize data
  cam, h, w = cam_from_rs_profile(cfg.get_stream(rs.stream.color))
  depth_scale = cfg.get_device().first_depth_sensor().get_depth_scale()
  print(f'Depth scale is {depth_scale}')

  I = ImageRGBa(h, w)
  I_depth = ImageUInt16(h, w)
  Idisp = ImageRGBa(h, w)

  # Align depth stream with color stream
  align = rs.align(rs.stream.color)
  get_images = partial(read_data, pipe=pipe, align=align)

  print('Initializing Afma6...')
  robot = RobotAfma6()
  robot.setPositioningVelocity(5.0)
  print(robot.getPosition(Robot.ControlFrameType.REFERENCE_FRAME))

  print('Moving Afma6 to starting pose...')
  r = PoseVector(0.06706274856, 0.3844766362, -0.04551332622 , 0.3111005431, 0.3031078532, 0.01708581392)
  cTw = HomogeneousMatrix(r)

  robot.setPosition(Robot.ControlFrameType.REFERENCE_FRAME, r)


  print('Warming up camera...')
  for _ in range(100):
    get_images(I, I_depth)

  # Define kalman filter

  drawer = UKSigmaDrawerMerwe(6, alpha=0.0001, beta=2, kappa=-3, resFunc=residual_state_vectors, addFunc=add_state_vectors)
  pixel_noise = 1
  noise_x, noise_y = [pixel_noise / f for f in [cam.get_px(), cam.get_py()]]
  noise_vel = 1e-8
  noise_accel = 1e-12
  measure_covariance = Matrix([
    [noise_x ** 2, 0.0],
    [0.0, noise_y ** 2]
  ])
  process_covariance = Matrix([
    [noise_x ** 2, 0, 0, 0, 0, 0],
    [0, noise_vel, 0, 0, 0, 0],
    [0, 0, noise_accel, 0, 0, 0],
    [0, 0, 0, noise_y ** 2, 0,0],
    [0, 0, 0, 0, noise_vel,0],
    [0, 0, 0, 0, 0, noise_accel],
  ])
  kalman = UnscentedKalman(generate_Q_matrix(1 / fps, sigma=1e-8), measure_covariance, drawer, fx, hx)


  # Define centering task
  xd, yd = PixelMeterConversion.convertPoint(cam, w / 2.0, h / 2.0)
  get_images(I, I_depth)
  Zd = I_depth[h // 2, w // 2] * depth_scale
  print(f'Desired depth is {Zd}')
  sd = FeaturePoint()
  sd.build(xd, yd, Zd)

  s = FeaturePoint()
  s.build(0.0, 0.0, Zd)

  task = Servo()
  task.addFeature(s, sd)
  task.setLambda(0.4)
  task.setCameraDoF(ColVector([0, 0, 0, 1, 1, 0]))
  task.setServo(Servo.ServoType.EYEINHAND_CAMERA)
  task.setInteractionMatrixType(Servo.ServoIteractionMatrixType.CURRENT)
  target_class = args.class_id # Car

  v = ColVector(6, 0.0)

  d = get_display()
  d.init(I)
  Display.display(I)
  Display.flush(I)
  _ = detection_model(np.array(I.numpy()[..., 2::-1]))
  error_norm = 1e10
  last_detection_time = -1.0
  first_iter = True
  # Servoing loop
  while error_norm > 5e-7:
    start = time.time()
    # Data acquisition
    get_images(I, I_depth)

    def has_class_box(box):
      return box.cls is not None and len(box.cls) > 0 and box.cls[0]

    # Build current features
    results = detection_model(np.array(I.numpy()[..., 2::-1]))[0] # Run detection
    boxes = results.boxes
    max_conf = 0.0
    idx = -1
    bb = None
    for i in range(len(boxes.conf)):
      if boxes.cls[i] == target_class and boxes.conf[i] > max_conf:
        idx = i
        max_conf = boxes.conf[i]
        bb = boxes.xywh[i].cpu().numpy()

    if bb is not None:
      u, v = bb[0], bb[1]
      x, y = PixelMeterConversion.convertPoint(cam, u, v)
      if first_iter:
        initial_state = ColVector([x, 0, 0, y, 0, 0])
        kalman.init(initial_state, process_covariance)
        first_iter = False
      kalman.filter(ColVector([x, y]), (1 / fps))
      kalman_state = kalman.getXest()
      last_detection_time = time.time()
      s.build(kalman_state[0], kalman_state[3], Zd)
      v = task.computeControlLaw()
    else:
      if last_detection_time < 0.0:
        raise RuntimeError('No detection at first iteration')
      kalman.predict(time.time() - last_detection_time)
      kalman_pred = kalman.getXpred()
      s.build(kalman_pred[0], kalman_pred[3], Zd)
      task.computeControlLaw()

    error: ColVector = task.getError()
    error_norm = error.sumSquare()

    # Display and logging
    Display.display(I)
    current_color = ColorBlindFriendlyPalette(ColorBlindFriendlyPalette.SkyBlue).to_vpColor()
    if bb is not None:
      Display.displayRectangle(I, i=int(bb[1] - bb[3] / 2), j=int(bb[0] - bb[2] / 2), width=bb[2], height=bb[3],
                               color=current_color, fill=False, thickness=2)
    sd.display(cam, I, ColorBlindFriendlyPalette(ColorBlindFriendlyPalette.Yellow).to_vpColor(), thickness=3)
    s.display(cam, I, current_color, thickness=3)
    Display.flush(I)
    Display.getImage(I, Idisp)
    robot.getPosition(Robot.ControlFrameType.REFERENCE_FRAME, r)
    cTw.build(r)
    plotter.on_iter(Idisp, v, error, cTw)

    # Move robot/update simulator
    robot.setRobotState(Robot.RobotStateType.STATE_VELOCITY_CONTROL)
    robot.setVelocity(Robot.ControlFrameType.CAMERA_FRAME, v)
    print(f'Iter took {time.time() - start}s')
    #simulator.setCameraPosition(cTw)

  robot.stopMotion()

  plotter.generate_anim()
