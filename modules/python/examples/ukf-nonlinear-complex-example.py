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
# For unp.sing ViSP with software that can not be combined with the GNU
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

""" @example ukf-nonlinear-complex-example.py
 Example of a complex non-linear use-case of the Unscented Kalman Filter (UKF).
  The system we are interested in is a 4-wheel robot, moving at a low velocity.
  As such, it can be modeled unp.sing a bicycle model.

  The state vector of the UKF is:
  \f[
  \begin{array}{lcl}
        \textbf{x}[0] &=& x \\
        \textbf{x}[1] &=& y \\
        \textbf{x}[2] &=& \theta
   \end{array}
   \f]
   where \f$ \theta \f$ is the heading of the robot.

   The measurement \f$ \textbf{z} \f$ corresponds to the distance and relative orientation of the
   robot with different landmarks. Be \f$ p_x^i \f$ and \f$ p_y^i \f$ the position of the \f$ i^{th} \f$ landmark
   along the x and y axis, the measurement vector can be written as:
   \f[
   \begin{array}{lcl}
        \textbf{z}[2i] &=& \sqrt{(p_x^i - x)^2 + (p_y^i - y)^2} \\
        \textbf{z}[2i+1] &=& \tan^{-1}{\frac{p_y^i - y}{p_x^i - x}} - \theta
   \end{array}
   \f]

  Some noise is added to the measurement vector to simulate measurements which are
  not perfect.

   The mean of several angles must be computed in the Unscented Transform. The definition we chose to use
   is the following:

   \f$ mean(\boldsymbol{\theta}) = atan2 (\frac{\sum_{i=1}^n sin{\theta_i}}{n}, \frac{\sum_{i=1}^n cos{\theta_i}}{n})  \f$

   As the Unscented Transform uses a weighted mean, the actual implementation of the weighted mean
   of several angles is the following:

   \f$ mean_{weighted}(\boldsymbol{\theta}) = atan2 (\sum_{i=1}^n w_m^i sin{\theta_i}, \sum_{i=1}^n w_m^i cos{\theta_i})  \f$

   where \f$ w_m^i \f$ is the weight associated to the \f$ i^{th} \f$ measurements for the weighted mean.

   Additionnally, the addition and substraction of angles must be carefully done, as the result
   must stay in the interval \f$[- \pi ; \pi ]\f$ or \f$[0 ; 2 \pi ]\f$ . We decided to use
   the interval \f$[- \pi ; \pi ]\f$ .
"""

from visp.core import ColVector, Matrix, UnscentedKalman, UKSigmaDrawerMerwe, Math
import numpy as np
from typing import List
from math import sqrt

# For the Graphical User Interface
try:
  from visp.gui import Plot
  has_gui = True
except:
  has_gui = False
import numpy as np

def normalize_angle(angle: float) -> float:
  angle_0_to_2pi = Math.modulo(angle, 2. * np.pi)
  angle_MinPi_Pi = angle_0_to_2pi
  if angle_MinPi_Pi > np.pi:
    # Substract 2 PI to be in interval [-Pi; Pi]
    angle_MinPi_Pi = angle_MinPi_Pi - 2. * np.pi
  return angle_MinPi_Pi

def measurement_mean(measurements: List[ColVector], wm: List[float]) -> ColVector:
  """
  Compute the weighted mean of measurement vectors.

  :param measurements: The measurement vectors, such as v[0] = dist_0 ; v[1] = bearing_0;
    v[2] = dist_1 ; v[3] = bearing_1 ...
  :param wm: The associated weights.

  :return ColVector: The weighted mean.
  """
  nb_points = len(measurements)
  size_measurement = measurements[0].size()
  nb_landmarks = size_measurement // 2
  mean = np.zeros(size_measurement)
  sum_cos = np.zeros(nb_landmarks)
  sum_sin = np.zeros(nb_landmarks)

  for i in range(nb_points):
    for j in range(nb_landmarks):
      mean[2*j] += wm[i] * measurements[i][2*j]
      sum_cos[j] += np.cos(measurements[i][(2*j)+1]) * wm[i]
      sum_sin[j] += np.sin(measurements[i][(2*j)+1]) * wm[i]

  orientations = np.arctan2(sum_sin, sum_cos)
  mean[1::2] = orientations
  return ColVector(mean)

def measurement_residual(meas: ColVector, to_substract: ColVector) -> ColVector:
  """
  Compute the substraction between two vectors expressed in the measurement space,
  such as v[0] = dist_0 ; v[1] = bearing_0; v[2] = dist_1 ; v[3] = bearing_1 ...

  :param meas: Measurement to which we must substract something.
  :param toSubstract: The something we must substract.

  :return ColVector: \b meas - \b toSubstract .
  """
  res = meas.numpy() - to_substract.numpy()
  res[1::2] = [normalize_angle(angle) for angle in res[1::2]]
  return ColVector(res)

def state_add_vectors(a: ColVector, b: ColVector) -> ColVector:
  """
  Compute the addition between two vectors expressed in the state space,
  such as v[0] = x ; v[1] = y; v[2] = heading .

  :param a: The first state vector to which another state vector must be added.
  :param b: The other state vector that must be added to a.

  :return ColVector: The sum a + b.
  """
  add = a + b
  return ColVector([add[0], add[1], normalize_angle(add[2])] )


def state_mean_vectors(states: List[ColVector], wm: List[float]) -> ColVector:
  """
  Compute the weighted mean of state vectors.

  :param states: The state vectors.
  :param wm: The associated weights.
  :return ColVector: The weighted mean.
 """
  mean = np.zeros(3)
  wm_np = np.array(wm)
  weighted_x = wm_np * np.array([state[0] for state in states])
  weighted_y = wm_np * np.array([state[1] for state in states])
  mean[0] = np.array(weighted_x).sum()
  mean[1] = np.array(weighted_y).sum()
  sumCos = (wm_np * np.array([np.cos(state[2]) for state in states])).sum()
  sumSin = (wm_np * np.array([np.sin(state[2]) for state in states])).sum()
  mean[2] = np.arctan2(sumSin, sumCos)
  return ColVector(mean)

def state_residual_vectors(a, b) -> ColVector:
  """
  Compute the substraction between two vectors expressed in the state space,
  such as v[0] = x ; v[1] = y; v[2] = heading .

  :param a: The first state vector to which another state vector must be substracted.
  :param b: The other state vector that must be substracted to a.

  :return ColVector: The substraction a - b.
  """
  res = a - b
  return ColVector([res[0], res[1], normalize_angle(res[2])])

def fx(x: ColVector, dt: float) -> ColVector:
  """
  As the state model {x, y, \f$ \theta \f$} does not contain any velocity
  information, it does not evolve without commands.

  :param x: The internal state of the UKF.
  :param dt: The sampling time: how far in the future are we projecting x.

  :return ColVector: The updated internal state, projected in time, also known as the prior.
  """
  return x

def generate_turn_commands(v: float, angleStart: float, angleStop: float, nbSteps: int) -> List[ColVector]:
  """
  Compute the commands realising a turn at constant linear velocity.

  :param v: Constant linear velocity.
  :param angleStart: Starting angle (in degrees).
  :param angleStop: Stop angle (in degrees).
  :param nbSteps: Number of steps to perform the turn.
  :return List[ColVector]: The corresponding list of commands.
  """
  cmds = []
  dTheta = Math.rad(angleStop - angleStart) / float(nbSteps - 1)
  for i in range(nbSteps):
    theta = Math.rad(angleStart) + dTheta * float(i)
    cmd = ColVector([v, theta])
    cmds.append(cmd)
  return cmds

def generate_commands() -> List[ColVector]:
  """
  Generate the list of commands for the simulation.

  :return List[ColVector]: The list of commands to use in the simulation
  """
  cmds = []
  # Starting by a straight line acceleration
  nbSteps = 30
  dv = (1.1 - 0.001) / float(nbSteps - 1)
  for i in range(nbSteps):
    cmd = ColVector([0.001 + float(i) * dv, 0.])
    cmds.append(cmd)

  # Left turn
  lastLinearVelocity = cmds[len(cmds) -1][0]
  leftTurnCmds = generate_turn_commands(lastLinearVelocity, 0, 2, 15)
  cmds.extend(leftTurnCmds)
  for i in range(100):
    cmds.append(cmds[len(cmds) -1])

  # Right turn
  lastLinearVelocity = cmds[len(cmds) -1][0]
  rightTurnCmds = generate_turn_commands(lastLinearVelocity, 2, -2, 15);
  cmds.extend(rightTurnCmds)
  for i in range(200):
    cmds.append(cmds[len(cmds) -1])

  # Left  turn again
  lastLinearVelocity = cmds[len(cmds) -1][0]
  leftTurnCmds = generate_turn_commands(lastLinearVelocity, -2, 0, 15)
  cmds.extend(leftTurnCmds)
  for i in range(150):
    cmds.append(cmds[len(cmds) -1])

  lastLinearVelocity = cmds[len(cmds) -1][0]
  leftTurnCmds = generate_turn_commands(lastLinearVelocity, 0, 1, 25)
  cmds.extend(leftTurnCmds)
  for i in range(150):
    cmds.append(cmds[len(cmds) -1])

  return cmds

class vpBicycleModel:
  """
  Class that approximates a 4-wheel robot unp.sing a bicycle model.
  """
  def __init__(self, w: float):
    """
    Construct a new vpBicycleModel object.

    :param w:The length of the wheelbase.
    """
    self._w = w # The length of the wheelbase.

  def compute_motion(self, u: ColVector, x: ColVector, dt: float) -> ColVector:
    """
    Models the effect of the command on the state model.

    :param u: The commands. u[0] = velocity ; u[1] = steeringAngle .
    :param x: The state model. x[0] = x ; x[1] = y ; x[2] = heading
    :param dt: The period.
    :return ColVector: The state model after applying the command.
    """
    heading = x[2]
    vel = u[0]
    steeringAngle = u[1]
    distance = vel * dt

    if (abs(steeringAngle) > 0.001):
      # The robot is turning
      beta = (distance / self._w) * np.tan(steeringAngle)
      radius = self._w / np.tan(steeringAngle)
      sinh = np.sin(heading)
      sinhb = np.sin(heading + beta)
      cosh = np.cos(heading)
      coshb = np.cos(heading + beta)
      motion = ColVector([
        -radius * sinh + radius * sinhb,
        radius * cosh - radius * coshb,
        beta
      ])
      return motion
    else:
      # The robot is moving in straight line
      motion = ColVector([
        distance * np.cos(heading),
        distance * np.sin(heading),
        0.
      ])
      return motion

  def move(self, u: ColVector, x: ColVector, dt: float) -> ColVector:
    """
    Models the effect of the command on the state model.

    :param u: The commands. u[0] = velocity ; u[1] = steeringAngle .
    :param x: The state model. x[0] = x ; x[1] = y ; x[2] = heading
    :param dt: The period.
    :return ColVector: The state model after applying the command.
    """
    motion = self.compute_motion(u, x, dt)
    newX = x + motion
    normalizedAngle = normalize_angle(newX[2])
    return ColVector([newX[0], newX[1], normalizedAngle])

class LandmarkMeasurements:
  """
  Class that permits to convert the position + heading of the 4-wheel
  robot into measurements from a landmark.
  """
  def __init__(self, x: float, y: float,  range_std: float, rel_angle_std: float):
    """
    Construct a new LandmarkMeasurements object.

    :param x: The position along the x-axis of the landmark.
    :param y: The position along the y-axis of the landmark.
    :param range_std: The standard deviation of the range measurements.
    :param rel_angle_std: The standard deviation of the relative angle measurements.
    """
    self._x = x # The position along the x-axis of the landmark
    self._y = y # The position along the y-axis of the landmark
    self._range_std = range_std # The standard deviation of the range measurement
    self._rel_angle_std = rel_angle_std # The standard deviation of the relative angle measurement
    np.random.seed(4224)

  def state_to_measurement(self, chi: ColVector) -> ColVector:
    """
    Convert the prior of the UKF into the measurement space.

    :param chi: The prior.
    :return ColVector: The prior expressed in the measurement space.
    """
    dx = self._x - chi[0]
    dy = self._y - chi[1]
    meas = ColVector([sqrt(dx * dx + dy * dy), normalize_angle(np.arctan2(dy, dx) - chi[2])])
    return meas

  def measure_gt(self, pos: ColVector) -> ColVector:
    """
    Perfect measurement of the range and relative orientation of the robot
    located at pos.

    :param pos: The actual position of the robot (pos[0]: x, pos[1]: y, pos[2] = heading.
    :return ColVector: [0] the range [1] the relative orientation of the robot.
    """
    dx = self._x - pos[0]
    dy = self._y - pos[1]
    range = sqrt(dx * dx + dy * dy)
    orientation = normalize_angle(np.arctan2(dy, dx) - pos[2])
    measurements = ColVector([range, orientation])
    return measurements

  def measure_with_noise(self, pos: ColVector) -> ColVector:
    """
    Noisy measurement of the range and relative orientation that
    correspond to pos.

    :param pos: The actual position of the robot (pos[0]: x ; pos[1] = y ; pos[2] = heading).
    :return ColVector: [0] the range [1] the relative orientation.
    """
    measurementsGT = self.measure_gt(pos)
    measurementsNoisy = measurementsGT
    range = measurementsNoisy[0] + np.random.normal(0., self._range_std)
    relAngle = normalize_angle(measurementsNoisy[1] + np.random.normal(0., self._rel_angle_std))
    return ColVector([range, relAngle])


class LandmarksGrid:
  """
  Class that represent a grid of landmarks that measure the distance and
  relative orientation of the 4-wheel robot.
  """
  def __init__(self, landmarks: List[LandmarkMeasurements]):
    """
    Construct a new LandmarksGrid object.

    :param landmarks: The list of landmarks forming the grid.
    """
    self._landmarks = landmarks # The list of landmarks forming the grid.

  def state_to_measurement(self, chi: ColVector) -> ColVector:
    """
    Convert the prior of the UKF into the measurement space.

    :param chi: The prior.
    :return ColVector: The prior expressed in the measurement space.
    """
    nbLandmarks = len(self._landmarks)
    measurements = np.zeros(2*nbLandmarks)
    for i in range (nbLandmarks):
      landmarkMeas = self._landmarks[i].state_to_measurement(chi)
      measurements[2*i] = landmarkMeas[0]
      measurements[(2*i) + 1] = landmarkMeas[1]
    return ColVector(measurements)

  def measure_gt(self, pos: ColVector) -> ColVector:
    """
    Perfect measurement from each landmark of the range and relative orientation of the robot
    located at pos.

    :param pos: The actual position of the robot (pos[0]: x, pos[1]: y, pos[2] = heading.
    :return ColVector: n x ([0] the range [1] the relative orientation of the robot), where
    n is the number of landmarks.
    """
    nbLandmarks = len(self._landmarks)
    measurements = np.zeros(2*nbLandmarks)
    for i in range(nbLandmarks):
      landmarkMeas = self._landmarks[i].measure_gt(pos)
      measurements[2*i] = landmarkMeas[0]
      measurements[(2*i) + 1] = landmarkMeas[1]
    return ColVector(measurements)

  def measure_with_noise(self, pos: ColVector) -> ColVector:
    """
    Noisy measurement from each landmark of the range and relative orientation that
    correspond to pos.

    :param pos: The actual position of the robot (pos[0]: x ; pos[1] = y ; pos[2] = heading).
    :return ColVector: n x ([0] the range [1] the relative orientation of the robot), where
    n is the number of landmarks.
    """
    nbLandmarks = len(self._landmarks)
    measurements = np.zeros(2*nbLandmarks)
    for i in range(nbLandmarks):
      landmarkMeas = self._landmarks[i].measure_with_noise(pos)
      measurements[2*i] = landmarkMeas[0]
      measurements[(2*i) + 1] = landmarkMeas[1]
    return ColVector(measurements)

if __name__ == '__main__':
  dt = 0.1 # Period of 0.1s
  step = 1. # Number of update of the robot position between two UKF filtering
  sigma_range = 0.3 # Standard deviation of the range measurement: 0.3m
  sigma_bearing = Math.rad(0.5) # Standard deviation of the bearing angle: 0.5deg
  wheelbase = 0.5 # Wheelbase of 0.5m
  process_variance = 0.0001
  positions = [ (5, 10) , (10, 5), (15, 15), (20, 5), (0, 30), (50, 30), (40, 10)] # Positions of the landmarks constituing the grid
  landmarks = [LandmarkMeasurements(x, y, sigma_range, sigma_bearing) for x,y in positions] # Vector of landmarks constituing the grid
  nbLandmarks = len(landmarks) # Number of landmarks constituing the grid
  cmds = generate_commands()
  nb_cmds = len(cmds)

  # Creation of the simulated grid of landmarks and robot
  grid = LandmarksGrid(landmarks)
  robot = vpBicycleModel(wheelbase)

  # The object that draws the sigma points used by the UKF
  drawer = UKSigmaDrawerMerwe(n=3, alpha=0.1, beta=2, kappa=0, resFunc=state_residual_vectors, addFunc=state_add_vectors)

  # The matrices require for the construction of the Unscented filter
  P0 = Matrix([[0.1, 0., 0.],
               [0., 0.1, 0.],
               [0., 0., 0.05]]) # The initial estimate of the state covariance matrix
  R1landmark = Matrix([[sigma_range * sigma_range, 0], [0, sigma_bearing*sigma_bearing]]) # The measurement covariance matrix for 1 landmark
  R = Matrix(2*nbLandmarks, 2 * nbLandmarks) # The measurement covariance matrix for the grid of landmarks
  for i in range(nbLandmarks):
    R.insert(R1landmark, 2*i, 2*i)

  Q = Matrix() # The process covariance matrix
  Q.eye(3)
  Q = Q * process_variance
  X0 = ColVector([2., 6., 0.3]) # robot_x, robot_y, robot_orientation(rad)

  # Creation of the Unscented Kalman filter
  ukf = UnscentedKalman(Q, R, drawer, fx, grid.state_to_measurement) # The Unscented Kalman Filter instance

  # Initializing the state vector and state covariance matrix estimates
  ukf.init(X0, P0)
  ukf.setCommandStateFunction(robot.compute_motion)
  ukf.setMeasurementMeanFunction(measurement_mean)
  ukf.setMeasurementResidualFunction(measurement_residual)
  ukf.setStateAddFunction(state_add_vectors)
  ukf.setStateMeanFunction(state_mean_vectors)
  ukf.setStateResidualFunction(state_residual_vectors)

  # Initializing the Graphical User Interface if the needed libraries are available
  if has_gui:
    num_plots = 1
    plot = Plot(num_plots)
    plot.initGraph(0, 2)
    plot.setTitle(0, "Position of the robot")
    plot.setUnitX(0, "Position along x(m)")
    plot.setUnitY(0, "Position along y (m)")
    plot.setLegend(0, 0, "GT")
    plot.setLegend(0, 1, "Filtered")

  robot_pos = X0
  for i in range(nb_cmds):
    robot_pos = robot.move(cmds[i], robot_pos, dt / step)

    if (i % int(step) == 0):
      # Perform the measurement
      z = grid.measure_with_noise(robot_pos)

      # Use the UKF to filter the measurement
      ukf.filter(z, dt, cmds[i])

      if has_gui:
        # Plot the filtered state
        Xest = ukf.getXest()
        plot.plot(0, 1, Xest[0], Xest[1])

    # Update the GUI if available
    if has_gui:
      # Plot the ground truth
      plot.plot(0, 0, robot_pos[0], robot_pos[1])

  print('Finished')
  input('Press enter to quit')
