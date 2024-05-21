#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP Python example to simulate an image-based visual servo on four 2D points
#
#############################################################################

import numpy as np
import argparse
import sys

# For plots
import matplotlib.pyplot as plt
import os

# Use latex in plot legends and labels
plt.rc('text', usetex=True)
plt.rc('text.latex', preamble=r'\usepackage{amsmath}')

# ViSp Python bindings
from visp.core import ExponentialMap
from visp.core import HomogeneousMatrix
from visp.core import Math
from visp.core import Point
from visp.core import RotationMatrix
from visp.core import ThetaUVector
from visp.core import TranslationVector

from visp.visual_features import FeatureBuilder
from visp.visual_features import FeaturePoint

from visp.vs import Servo

class PlotIbvs:
  def __init__(self, e, norm_e, v, x, xd, c_T_w, plot_log_scale):
    self.vector_e  = e
    self.vector_ne = norm_e
    self.vector_v  = v
    self.vector_x  = x
    self.vector_xd = xd
    self.vector_w_t_c = c_T_w.inverse().getTranslationVector()
    self.plot_log_scale = plot_log_scale

  def stack(self, e, norm_e, v, x, xd, c_T_w):
    self.vector_e  = np.vstack((self.vector_e, e))
    self.vector_ne = np.vstack((self.vector_ne, norm_e))
    self.vector_v  = np.vstack((self.vector_v, v))
    self.vector_x  = np.vstack((self.vector_x, x))
    self.vector_w_t_c = np.vstack((self.vector_w_t_c, c_T_w.inverse().getTranslationVector()))

  def display(self, fig_filename):
    plt.figure(figsize=(10,10))

    plot_e  = plt.subplot(2, 2, 1)
    plot_v  = plt.subplot(2, 2, 2)
    plot_ne = plt.subplot(2, 2, 3)
    plot_x  = plt.subplot(2, 2, 4)

    plot_e.set_title('error')
    plot_v.set_title('camera velocity')
    plot_x.set_title('point trajectory in the image plane')

    if self.plot_log_scale:
      plot_ne.set_title('log(norm error)')
      plot_ne.plot(np.log(self.vector_ne))
    else:
      plot_ne.set_title('norm error')
      plot_ne.plot(self.vector_ne)

    plot_ne.grid(True)
    plot_e.grid(True)
    plot_v.grid(True)
    plot_x.grid(True)

    plot_e.plot(self.vector_e)
    plot_e.legend(['$x_1$','$y_1$','$x_2$','$y_2$','$x_3$','$y_3$','$x_4$','$y_4$',])

    for i in range(self.vector_v.shape[1]): # Should be 6
      plot_v.plot(self.vector_v[:,i])
    plot_v.legend(['$v_x$','$v_y$','$v_z$','$\omega_x$','$\omega_y$','$\omega_z$'])

    for i in range(self.vector_x.shape[1]):
      pts = np.asarray([[p.get_x(), p.get_y()] for p in self.vector_x[:, i]])
      plot_x.plot(pts[:, 0], pts[:, 1])
    plot_x.legend(['$x_1$','$x_2$','$x_3$','$x_4$'])

    for i in range(self.vector_x.shape[1] ):
      plot_x.plot(self.vector_xd[i].get_x(),  self.vector_xd[i].get_y(),'o')

    # Create output folder it it doesn't exist
    output_folder = os.path.dirname(fig_filename)
    if not os.path.exists(output_folder):
      os.makedirs(output_folder)
      print("Create output folder: ", output_folder)

    print(f"Figure is saved in {fig_filename}")
    plt.savefig(fig_filename)

    # Plot 3D camera trajectory
    plot_traj = plt.figure().add_subplot(projection='3d')
    plot_traj.scatter(self.vector_w_t_c[0][0], self.vector_w_t_c[0][1], self.vector_w_t_c[0][2], marker='x', c='r', label='Initial position')
    # Hack to ensure that the scale is at minimum between -0.5 and 0.5 along X and Y axis
    min_s = np.min(self.vector_w_t_c, axis=0)
    max_s = np.max(self.vector_w_t_c, axis=0)
    for i in range(len(min_s)):
      if (max_s[i] - min_s[i]) < 1.:
        max_s[i] += 0.5
        min_s[i] -= 0.5
    plot_traj.axis(xmin=min_s[0], xmax=max_s[0])
    plot_traj.axis(ymin=min_s[1], ymax=max_s[1])

    plot_traj.plot(self.vector_w_t_c[:, 0], self.vector_w_t_c[:, 1], zs=self.vector_w_t_c[:, 2], label='Camera trajectory')
    plot_traj.set_title('Camera trajectory w_t_c in world space')
    plot_traj.legend()
    filename = os.path.splitext(fig_filename)[0] + "-traj-w_t_c.png"
    print(f"Figure is saved in {filename}")
    plt.savefig(filename)

    plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='The script corresponding to TP 4, IBVS on 4 points.')
  parser.add_argument('--initial-position', type=int, default=2, dest='initial_position', help='Initial position selection (value 1, 2, or 3)')
  parser.add_argument('--interaction', type=str, default="current", dest='interaction_matrix_type', help='Interaction matrix type (value \"current\" or \"desired\")')
  parser.add_argument('--plot-log-scale', action='store_true', help='Plot norm of the error using a logarithmic scale')
  parser.add_argument('--no-plot', action='store_true', help='Disable plots')

  args, unknown_args = parser.parse_known_args()
  if unknown_args:
    print("The following args are not recognized and will not be used: %s" % unknown_args)
    sys.exit()

  print(f"Use initial position {args.initial_position}")

  # Position of the reference in the camera frame
  c_T_w = HomogeneousMatrix()

  if args.initial_position == 1:
    # - CASE 1
    thetau = ThetaUVector(0.0, 0.0, 0.0)
    c_R_w = RotationMatrix(thetau)
    c_t_w = TranslationVector(0.0, 0.0, 1.3)
    c_T_w.insert(c_R_w)
    c_T_w.insert(c_t_w)
  elif args.initial_position == 2:
    # - CASE 2
    thetau = ThetaUVector(Math.rad(10), Math.rad(20), Math.rad(30))
    c_R_w = RotationMatrix(thetau)
    c_t_w = TranslationVector(-0.2, -0.1, 1.3)
    c_T_w.insert(c_R_w)
    c_T_w.insert(c_t_w)
  elif args.initial_position == 3:
    # - CASE 3 : 90 rotation along Z axis
    thetau = ThetaUVector(0.0, 0.0, Math.rad(90))
    c_R_w = RotationMatrix(thetau)
    c_t_w = TranslationVector(0.0, 0.0, 1.0)
    c_T_w.insert(c_R_w)
    c_T_w.insert(c_t_w)
  else:
    raise ValueError(f"Wrong initial position value. Values are 1, 2 or 3")

  # Position of the desired camera in the world reference frame
  cd_T_w = HomogeneousMatrix()
  thetau = ThetaUVector(0, 0, 0)
  cd_R_w = RotationMatrix(thetau)
  cd_t_w = TranslationVector(0.0, 0.0, 1.0)
  cd_T_w.insert(cd_R_w)
  cd_T_w.insert(cd_t_w)

  # 3D point in the reference frame in homogeneous coordinates
  wX = []
  wX.append(Point(-0.1,  0.1, 0.0))
  wX.append(Point( 0.1,  0.1, 0.0))
  wX.append(Point( 0.1, -0.1, 0.0))
  wX.append(Point(-0.1, -0.1, 0.0))

  # Creation of the current and desired features vectors respectively in x and xd
  x = [FeaturePoint(), FeaturePoint(), FeaturePoint(),FeaturePoint()]   # Initialize current visual feature
  xd = [FeaturePoint(), FeaturePoint(), FeaturePoint(), FeaturePoint()] # Initialize desired visual feature

  # Create the visual servo task
  task = Servo()
  task.setServo(Servo.EYEINHAND_CAMERA)
  task.setLambda(0.1) # Set the constant gain

  # For each point
  for i in range(len(wX)):
    # Create current visual feature
    wX[i].track(c_T_w)
    FeatureBuilder.create(x[i], wX[i])

    print(f"Visual features at initial position for point {i}:")
    print(f"  wX[{i}]: {wX[i].get_oX()} {wX[i].get_oY()} {wX[i].get_oZ()}")
    print(f"  cX[{i}]: {wX[i].get_X()} {wX[i].get_Y()} {wX[i].get_Z()}")
    print(f"  x[{i}]: {x[i].get_x()} {x[i].get_y()} {x[i].get_Z()}")

    # Create desired visual feature
    wX[i].track(cd_T_w)
    FeatureBuilder.create(xd[i], wX[i])

    print(f"Visual features at desired position for point {i}:")
    print(f"cdX[{i}]: {wX[i].get_X()} {wX[i].get_Y()} {wX[i].get_Z()}")
    print(f"xd[{i}]: {xd[i].get_x()} {xd[i].get_y()} {xd[i].get_Z()}")

    # Add current and desired features to the visual servo task
    task.addFeature(x[i], xd[i])

  iter = 0

  # Control loop
  while (iter == 0 or norm_e > 0.0001):
    print(f"---- Visual servoing iteration {iter} ----")
    # Considered vars:
    #   e: error vector
    #   norm_e: norm of the error vector
    #   v: velocity to apply to the camera
    #   x: current visual feature vector
    #   xd: desired visual feature vector
    #   c_T_w: current position of the camera in the world frame
    if args.interaction_matrix_type == "current":
      # Set interaction matrix type
      task.setInteractionMatrixType(Servo.CURRENT, Servo.PSEUDO_INVERSE)
      # Update visual features in x for each point
      for i in range(len(wX)):
        # Update current visual feature
        wX[i].track(c_T_w);
        FeatureBuilder.create(x[i], wX[i])
    elif args.interaction_matrix_type == "desired":
      # Set interaction matrix type
      task.setInteractionMatrixType(Servo.DESIRED, Servo.PSEUDO_INVERSE)
      # Update visual features in xd
      for i in range(len(wX)):
        # Update current visual feature
        wX[i].track(c_T_w);
        FeatureBuilder.create(x[i], wX[i])
        # Update desired visual feature
        wX[i].track(cd_T_w);
        FeatureBuilder.create(xd[i], wX[i])
    else:
      raise ValueError(f"Wrong interaction matrix type. Values are \"current\" or \"desired\"")

    # Compute the control law
    v = task.computeControlLaw()
    e = task.getError()
    norm_e = e.frobeniusNorm()
    Lx = task.getInteractionMatrix()

    xplot = []

    if not args.no_plot:
      for f in x:
        p = FeaturePoint()
        p.build(f.get_x(), f.get_y(), f.get_Z())
        xplot.append(p)

      if iter == 0:
        plot = PlotIbvs(e, norm_e, v, xplot, xd, c_T_w, args.plot_log_scale)
      else:
        plot.stack(e, norm_e, v, xplot, xd, c_T_w)

    # Compute camera displacement after applying the velocity for delta_t seconds.
    c_T_c_delta_t = ExponentialMap.direct(v, 0.040)

    # Compute the new position of the camera
    c_T_w = c_T_c_delta_t.inverse() * c_T_w

    print(f"e: \n{e}")
    print(f"norm e: \n{norm_e}")
    print(f"Lx: \n{Lx}")
    print(f"v: \n{v}")
    print(f"c_T_w: \n{c_T_w}")

    # Increment iteration counter
    iter += 1

  print(f"\nConvergence achieved in {iter} iterations")

  if not args.no_plot:
    # Display the servo behavior
    plot.display("results/fig-ibvs-four-points-initial-position-" + str(args.initial_position) + ".png")

    print("Kill the figure to quit...")
