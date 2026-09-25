.. _tutorial-franka-ibvs:

==================================================
Visual servoing with a Franka robot
==================================================

Introduction
==================================================

Goal
--------------------------------------------------

In this tutorial you will learn how to:

- Connect to a Franka robot with :py:class:`~visp.robot.RobotFranka` and :py:class:`~visp.robot.Robot`.
- Compute a visual servoing movement with :py:class:`~visp.core.Servo`.

Prerequisites
--------------------------------------------------

You should first read the following tutorials:

- :ref:`tutorial-camera-realsense`
- :ref:`tutorial-detection-apriltag`


To run or edit the code presented, you will need the ``numpy`` and ``pyrealsense2`` Python packages,
and to download the Franka module for ViSP.

You will also need the following hardware:

- A Franka robot.
- A RealSense camera mounted on it.
- An AprilTag (see :ref:`here <tutorial-image-display>` how to generate one)

Visual servoing
==================================================

The following :ref:`example <code-franka-ibvs>` detects an AprilTag with the RealSense camera
and move the Franka robot to place its gravity center at the center of the streamed image, at a specific distance:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :linenos:

Running the example
--------------------------------------------------

.. attention::

  Make sure that the Franka robot is placed in a safe environment, with sufficient space for all commanded motions.

You can run the example with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/franka/tutorial-franka-ibvs.py

Usage
--------------------------------------------------

A window opens and displays the stream of the camera.

At startup, the robot movement is disabled, as shown on the ``Idle state`` picture.
You can click the left mouse button to start or stop the visual servoing,
and click the right mouse button to stop the programm.

When activated, if one AprilTag is detected, the robot moves to match the center of gravity of the tag with the center of the screen, at a set distance.
You can see the two centers represented by respectively the green and red crosses on the ``Starting`` image.
The trajectory of the tag center is shown by the blue trail on the ``Moving state`` image.
After reaching the desired position within a specified margin, the robot stops, as on the ``Stopping`` image.

.. list-table::

  * - **Idle state**
  
      .. image:: images/result-franka-ibvs-idle.png

    - **Starting**
      
      .. image:: images/result-franka-ibvs-start.png

  * - **Moving State**
  
      .. image:: images/result-franka-ibvs-moving.png

    - **Stopping**
    
      .. image:: images/result-franka-ibvs-stop.png


The visual servoing error and the robot velocities are shown at all time.

Explanation
==================================================

The program performs four main tasks:

1. It configures the RealSense camera and Franka robot.
2. It detects the AprilTag and estimates its pose.
3. It computes the next movement and moves the robot accordingly.
4. It displays the camera image and relevant information.

We will only describe here how the visual servoing task and Franka robot initialization and movement are effectued.
The RealSense camera configuration and its stream acquisition,
as well as the Apriltag detection, are the same as in their dedicated tutorials.

Import the required modules
--------------------------------------------------

We will need ``pyrealsense2``, ``numpy`` and :py:class:`~visp.core.ImageRGBa` to acquire and store the camera stream.
:py:class:`~visp.core.ImageConvert` converts the color image into an
:py:class:`~visp.core.ImageGray` for use with :py:class:`~visp.detection.DetectorAprilTag`.

We will use :py:class:`~visp.core.PoseVector`, :py:class:`~visp.core.HomogeneousMatrix`,
:py:class:`~visp.core.ColVector`, and :py:class:`~visp.core.Point` as data structures.
The Franka robot is manipulated with :py:class:`~visp.robot.Robot` and :py:class:`~visp.robot.RobotFranka`,
and the visual-servoing task is implemented with :py:class:`~visp.vs.Servo`.
The result is displayed with :py:class:`~visp.core.Display`,
:py:func:`~visp.python.display_utils.get_display`, and :py:class:`~visp.vs.ServoDisplay`.

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: import argparse
  :end-at: from visp.python.display_utils import get_display

Franka configuration
--------------------------------------------------

First, we define the homogeneous transformation between the robot :math:`e` and the camera frame :math:`c`:

.. math::

  {}^e\mathbf{M}_c =
  \begin{bmatrix}
  {}^e\mathbf{R}_c & {}^e\mathbf{t}_c \\
  \mathbf{0}^T & 1
  \end{bmatrix}

The transformation is created from the extrinsic camera parameters:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Get the camera extrinsics parameters
  :end-at: extrinsics_matrix = HomogeneousMatrix(extrinsics)

It is then passed to the robot with :meth:`~visp.core.RobotFranka.set_eMc`, along with its IP adress.
This allows the robot interface to situate the camera in space, and move accordingly towards the desired position:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Connect and initialize the Franka robot
  :end-at: robot.setRobotState(Robot.STATE_VELOCITY_CONTROL)

Visual servoing initialization
--------------------------------------------------

We use a position-based 3D visual feature to control the robot.
The current feature is represented by ``centroid`` and the desired feature by ``centroid_desired``:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Create the current and desired position
  :end-at: centroid_desired.buildFrom(0, 0, DISTANCE_TO_TAG)

We then create a :py:class:`~visp.vs.Servo` object to indicate the objective of the servoing.
It is done by giving the two :py:class:`~visp.visual_features.FeaturePoint3D` to the :py:meth:`~visp.vs.Servo.addFeature`:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Create the visual servoing task
  :end-at: task.setLambda(LAMBDA)

The ``LAMBDA`` parameter is the gain of the movement. It can be fixe or adaptative.
An adaptive gain starts with a larger value and decreases as the robot approaches the desired position.
This can improve convergence speed while reducing motion near the target.

Visual servoing loop
--------------------------------------------------

The control law computes a camera velocity from the visual-feature error:

.. math::

   \mathbf{v}_c = -\lambda \, \widehat{\mathbf{L}}_s^{+}
   \left(\mathbf{s} - \mathbf{s}^{*}\right)

where :math:`\lambda` is the control gain, :math:`\widehat{\mathbf{L}}_s` is the estimated interaction matrix,
and :math:`\mathbf{s}` and :math:`\mathbf{s}^{*}` are respectively the current and desired visual features.

After detecting the AprilTag, the detector returns its pose within the camera frame.
The origin of the tag frame is its center, so transforming it gives the 3D position of the tag center:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Get the tag position
  :end-at: centroid.buildFrom(tag_projection[0], tag_projection[1], tag_projection[2])

We update the current visual feature with the resulting coordinates.
The call to :py:meth:`~visp.vs.Servo.computeControlLaw` then calculates the next camera velocity:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Compute the robot movements
  :end-at: velocity = task.computeControlLaw()

The returned vector contains six components:

.. math::

   \mathbf{v}_c =
   \begin{bmatrix}
   v_x & v_y & v_z & \omega_x & \omega_y & \omega_z
   \end{bmatrix}^{T}

The first three components are translational velocities in meters per second,
and the last three components are angular velocities in radians per second.

We obtain the error with :py:meth:`~visp.vs.Servo.getError()`.
It is compared with the user-defined convergence threshold:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Compute the error
  :end-at: velocity = ColVector(6, 0)

When the error is below the threshold, the movement is disabled.
If no tag, or more than one tag, is detected, the example also sends a null velocity.
This prevents the robot from moving with no intended position.

Finally, we send the velocity to the robot, allowing it to move:

.. literalinclude:: /examples/franka/tutorial-franka-ibvs.py
  :language: python
  :start-at: # Move the robot
  :end-at: robot.setVelocity(Robot.CAMERA_FRAME, velocity)
