#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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

""" @example ukf-linear-example.py
 Example of a simple linear use-case of the Unscented Kalman Filter (UKF). Using a UKF
 in this case is not necessary, it is done for learning purpous only.

 The system we are interested in is a system moving on a 2D-plane.

 The state vector of the UKF is:
  \f[
  \begin{array}{lcl}
        \textbf{x}[0] &=& x \\
        \textbf{x}[1] &=& \dot{x} \\
        \textbf{x}[1] &=& y \\
        \textbf{x}[2] &=& \dot{y}
  \end{array}
  \f]

 The measurement \f$ \textbf{z} \f$ corresponds to the position along the x-axis
 and y-axis. The measurement vector can be written as:
 \f[
  \begin{array}{lcl}
      \textbf{z}[0] &=& x \\
      \textbf{z}[1] &=& y
 \end{array}
  \f]

 Some noise is added to the measurement vector to simulate a sensor which is
 not perfect.
"""

from visp.core import ColVector, Matrix, UnscentedKalman, UKSigmaDrawerMerwe

# For the Graphical User Interface
try:
  from visp.gui import Plot
  has_gui = True
except:
  has_gui = False
import numpy as np

def fx(x: ColVector, dt: float) -> ColVector:
  """
  Process function that projects in time the internal state of the UKF.

  :param x: The internal state of the UKF.
  :param dt: The sampling time: how far in the future are we projecting x.

  :return ColVector: The updated internal state, projected in time, also known as the prior.
  """
  return ColVector([
  	x[0] + dt * x[1],
  	x[1],
  	x[2] + dt * x[3],
  	x[3]
  ])


def hx(x: ColVector) -> ColVector:
  """
  Measurement function that expresses the internal state of the UKF in the measurement space.

  :param x: The internal state of the UKF.

  :return ColVector: The internal state, expressed in the measurement space.
  """
  return ColVector([
  	x[0],
	x[2]
  ])

def add_state_vectors(a, b) -> ColVector:
  """
  Method that permits to add two state vectors.

  :param a: The first state vector to which another state vector must be added.
  :param b: The other state vector that must be added to a.

  :return ColVector: The sum a + b.
  """
  return a + b

def residual_state_vectors(a, b) -> ColVector:
  """
  Method that permits to substract a state vector to another.

  :param a: The first state vector to which another state vector must be substracted.
  :param b: The other state vector that must be substracted to a.

  :return ColVector: The substraction a - b.
  """
  return a - b

def generate_Q_matrix(dt: float) -> Matrix:
  """
  Method that generates the process covariance matrix for a process for which the
  state vector can be written as (x, dx/dt)^T

  :param dt: The sampling period.

  :return Matrix: The corresponding process covariance matrix.
  """
  return Matrix(
  	[[dt**3/3, dt**2/2, 0, 0],
  	[dt**2/2, dt, 0, 0],
  	[0, 0, dt**3/3, dt**2/2],
  	[0, 0, dt**2/2, dt]])

if __name__ == '__main__':
  dt = 0.01 # The sampling period
  gt_dx = 0.01 # The displacement along the x-axis between two timesteps
  gt_dy = 0.005 # The displacement along the y-axis between two timesteps
  gt_dX = ColVector([gt_dx, gt_dy]) # The displacement vector between two timesteps
  gt_vx = gt_dx / dt # The velocity along the x-axis
  gt_vy = gt_dy / dt # The velocity along the y-axis
  proc_var = 0.000004 # The variance of the process function
  sigma_x_meas = 0.05 # The standard deviation of the measurement noise for the x-axis measurement
  sigma_y_meas = 0.05 # The standard deviation of the measurement noise for the y-axis measurement

  # The object that draws the sigma points used by the UKF
  drawer = UKSigmaDrawerMerwe(n=4, alpha=0.3, beta=2, kappa=-1, resFunc=residual_state_vectors, addFunc=add_state_vectors)

  P0 = Matrix(np.eye(4) * 1.) # The initial estimate of the state covariance matrix
  R = Matrix(np.eye(2) * 0.01) # The measurement covariance matrix
  Q = generate_Q_matrix(dt) * proc_var # The process covariance matrix
  ukf = UnscentedKalman(Q, R, drawer, fx, hx) # The Unscented Kalman Filter instance

  # Initializing the state vector and state covariance matrix estimates
  ukf.init(ColVector([0., 0.75 * gt_vx, 0., 0.75 * gt_vy]), P0)

  # Initializing the Graphical User Interface if the needed libraries are available
  if has_gui:
    num_plots = 4
    plot = Plot(num_plots)
    plot_titles = [
      'Position along X-axis', 'Velocity along X-axis',
      'Position along Y-axis', 'Velocity along Y-axis'
    ]
    plot_y_units = [
      'Position (m)', 'Velocity (m/s)',
      'Position (m)', 'Velocity (m/s)'
    ]
    plot_legends = ['GT', 'Measure', 'Filtered']

    # Initializing the subplots
    for plot_index in range(num_plots):
      plot.initGraph(plot_index, len(plot_legends))
      plot.setTitle(plot_index, plot_titles[plot_index])
      plot.setUnitY(plot_index, plot_y_units[plot_index])
      plot.setUnitX(plot_index, 'Time (s)')
      for legend_index in range(len(plot_legends)):
        plot.setLegend(plot_index, legend_index, plot_legends[legend_index])

  gt_X = ColVector(2, 0.) # Ground truth position
  z_prec = ColVector(2, 0.) # Previous measurement vector
  np.random.seed(4224)

  for i in range(100):
    # Creating noisy measurements
    x_meas = gt_X[0] + np.random.normal(0.0, sigma_x_meas)
    y_meas = gt_X[1] + np.random.normal(0.0, sigma_y_meas)
    z = ColVector([x_meas, y_meas])

    # Filtering using the UKF
    ukf.filter(z, dt)

    # Getting the filtered state vector
    Xest = ukf.getXest()

    # Computing the noisy velocity from the measueremnts
    vX_meas = (x_meas - z_prec[0]) / dt
    vY_meas = (y_meas - z_prec[1]) / dt

    # Update the GUI if available
    if has_gui:
      plot.plot(0, 0, i, gt_X[0])
      plot.plot(0, 1, i, x_meas)
      plot.plot(0, 2, i, Xest[0])

      plot.plot(1, 0, i, gt_vx)
      plot.plot(1, 1, i, vX_meas)
      plot.plot(1, 2, i, Xest[1])

      plot.plot(2, 0, i, gt_X[1])
      plot.plot(2, 1, i, y_meas)
      plot.plot(2, 2, i, Xest[2])

      plot.plot(3, 0, i, gt_vy)
      plot.plot(3, 1, i, vY_meas)
      plot.plot(3, 2, i, Xest[3])

    # Updating ground-truth position
    gt_X += gt_dX

    # Updating last measurement for future computation of the noisy velocity
    z_prec = z

  print('Finished')
  input('Press enter to quit')
