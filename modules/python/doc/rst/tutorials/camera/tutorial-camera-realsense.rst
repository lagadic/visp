.. _tutorial-camera-realsense:

==================================================
Display a RealSense camera stream
==================================================

Introduction
==================================================

Goal
--------------------------------------------------

In this tutorial you will learn how to:

- Configure a RealSense camera.
- Use the :py:class:`~visp.core.Display` class to display a video stream.

Prerequisites
--------------------------------------------------

You should first read the following tutorials:

- :ref:`tutorial-image-getting-started`
- :ref:`tutorial-image-display`

You also need the ``numpy`` and ``pyrealsense2`` Python packages.

- Install them with `pip`:
.. code-block:: bash

  pip install numpy pyrealsense2

- or with `conda`:
.. code-block:: bash

  conda install numpy pyrealsense2

Display a camera stream
==================================================

The following :ref:`example <code-camera-realsense>` displays the color stream
from a RealSense camera in a ViSP display window:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :linenos:

Running the example
--------------------------------------------------

You can run the example with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/camera/tutorial-camera-realsense.py

Result
--------------------------------------------------

A window opens and displays the live color stream from the camera.
Click on the window to stop the example.

Explanation
==================================================

The program performs three main tasks:

1. It configures and starts the camera.
2. It creates a ViSP image and display window.
3. It continuously copies camera frames into the displayed image.

Import the required modules
--------------------------------------------------

We will need ``pyrealsense2`` to communicate with the RealSense camera,
``numpy`` to manipulate image data, and ViSP :py:class:`~visp.core.ImageRGBa`
and :py:class:`~visp.core.Display` classes to store and display the camera image:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: import numpy as np
  :end-at: from visp.python.display_utils import get_display

Configure the camera
--------------------------------------------------

First, we configure the camera to enable the color stream,
with a 640 × 480 pixels resolution, using RGB format, and at a frame rate of 60 frames per second:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Set the camera image dimensions
  :end-at: config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)

The RealSense pipeline manages frame acquisition. Start it with the
configuration created above:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Start streaming frames from the camera
  :end-at: pipeline.start(config)

Initialize the display
--------------------------------------------------

Before entering the acquisition loop, we create a :py:class:`~visp.core.ImageRGBa`
image with the same dimensions as the camera frames.
We then obtain a display object and associate it with the image:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Create and initialize the display image
  :end-at: display.init(image, 0, 0, "Camera view")

The display window will show the contents of ``image``, while the image itself is
updated during every iteration of the acquisition loop.

To update the image efficiently, we create a NumPy view of its underlying
storage:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Access the display image data as a NumPy array
  :end-at: image_array = np.asarray(image)

This does not create a separate copy of the image. Instead, ``image_array``
refers to the data already stored in ``image``. Changes made through the
NumPy array are therefore visible in the display image.

Acquire and displaying frames
--------------------------------------------------

The program now enters an infinite loop. On each iteration, it waits for the
next set of frames from the camera:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Capture the latest camera frame
  :end-at: color_frame = frames.get_color_frame()

We then select the color frame from the returned frame set.
The frame data is converted into a NumPy array, and copied
into the first three channels of ``image``:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Convert the color frame to a NumPy array
  :end-at: image_array[..., :3] = color_image

The expression ``image_array[..., :3]`` selects the red, green, and blue
channels provided by the camera while leaving the fourth alpha channel unchanged.

The updated image is then rendered:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Render the updated image
  :end-at: Display.flush(image)

Stop the program
--------------------------------------------------

The loop ends when the user clicks inside the display window:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Stop when the user clicks inside the display
  :end-at: break

After the loop exits, we stop the RealSense pipeline:

.. literalinclude:: /examples/camera/tutorial-camera-realsense.py
  :language: python
  :start-at: # Stop streaming when the program exits
  :end-at: pipeline.stop()

Other options
==================================================

Display a grayscale stream
--------------------------------------------------

.. _tutorial-camera-realsense-display-a-grayscale-stream:

RealSense cameras provide color and depth streams, but not a native grayscale stream.
To display a grayscale image, acquire the color stream and convert each frame before displaying it.

You can try it with this
:ref:`example <code-camera-realsense-grayscale>`.

Run it with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/camera/tutorial-camera-realsense-grayscale.py

We use a :py:class:`~visp.core.ImageGray` instead of a
:py:class:`~visp.core.ImageRGBa` to display the stream:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-grayscale.py
  :language: python
  :start-at: # Create and initialize the display image
  :end-at: display.init(image, 0, 0, "Grayscale view")

The :py:class:`~visp.core.ImageRGBa` is still needed to store the frame data:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-grayscale.py
  :language: python
  :start-at: # Create a RGBa image to store each frame
  :end-at: image_rgba = ImageRGBa(HEIGHT, WIDTH)

Then, we just have to convert the :py:class:`~visp.core.ImageRGBa` obtained
using the :py:meth:`~visp.core.ImageConvert.convert` method:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-grayscale.py
  :language: python
  :start-at: # Convert it to grayscale format
  :end-at: ImageConvert.convert(image_rgba, image)

Display depth
--------------------------------------------------

A RealSense camera can also provide depth data.

You can display a colorized view of depth as shown in this
:ref:`example <code-camera-realsense-depth>`.

You can run it with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/camera/tutorial-camera-realsense-depth.py

We first enable the depth stream instead of the color stream:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-depth.py
  :language: python
  :start-at: config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
  :end-at: config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

The raw depth values are not directly suitable for display because they
represent distances rather than display intensities.
We then create a ``colorizer`` object to convert the depth values into a color image:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-depth.py
  :language: python
  :start-at: # Get a colorizer to convert raw depth values into a color image
  :end-at: colorizer = rs.colorizer()

It will let us colorize the frame in the acquisition loop before rendering:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-depth.py
  :language: python
  :start-at: # Colorize the depth image for visualization
  :end-at: depth_color_frame = colorizer.colorize(depth_frame)

Display both color and depth views
--------------------------------------------------

This :ref:`example <code-camera-realsense-depth>` let you display both color and depth views.

You can run it with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/camera/tutorial-camera-realsense-dual-view.py

Get the camera intrinsics parameters
--------------------------------------------------

Many ViSP image operations require to know the camera's intrinsic parameters.
These parameters must be stored in a
:py:class:`~visp.core.CameraParameters` object.

This :ref:`example <code-camera-realsense-intrinsics>` shows how to
retrieve the parameters for the RealSense color and depth streams.

Run it with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/camera/tutorial-camera-realsense-intrinsics.py

We first import the :py:class:`~visp.core.CameraParameters` class:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-intrinsics.py
  :language: python
  :start-at: from visp.core import CameraParameters
  :end-at: from visp.core import CameraParameters

We then get the profile of the camera while starting the RealSense pipeline:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-intrinsics.py
  :language: python
  :start-at: # Start streaming and get the active camera profile
  :end-at: profile = pipeline.start(config)

The profile contains the calibration data for each enabled video stream.
We define a helper function that retrieves the stream's intrinsic parameters and
convert then to a :py:class:`~visp.core.CameraParameters` object:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-intrinsics.py
  :language: python
  :start-at: def get_camera_parameters(profile, stream_type):
  :end-at: return CameraParameters(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

We call this function separately for the color and depth streams, as they come from different cameras:

.. literalinclude:: /examples/camera/tutorial-camera-realsense-intrinsics.py
  :language: python
  :start-at: # Read the color camera intrinsics
  :end-at: print(f"  ViSP parameters: {depth_camera}")

You should obtain an output similar to this (with your own values):

.. code-block:: text

  Color camera intrinsics:
    ViSP parameters: Camera parameters for perspective projection without distortion:
    px = 387.232   py = 386.652
    u0 = 325.677   v0 = 236.252

  Depth camera intrinsics:
    ViSP parameters: Camera parameters for perspective projection without distortion:
    px = 390.406   py = 390.406
    u0 = 326.319   v0 = 239.871

Here:

- ``px`` and ``py`` are the focal lengths in pixels along the horizontal and
  vertical image axes.
- ``u0`` and ``v0`` are the coordinates of the principal point in pixels.
  This point is usually near the center of the image.
