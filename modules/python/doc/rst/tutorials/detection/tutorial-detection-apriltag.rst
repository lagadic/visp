.. _tutorial-detection-apriltag:

==================================================
Detect an AprilTag in an image
==================================================

Introduction
==================================================

Goal
--------------------------------------------------

In this tutorial you will learn how to:

- Detect AprilTags in an image.
- Estimate the pose of each detected tag.
- Display tag borders and coordinate frames.
- Use the :py:class:`~visp.detection.DetectorAprilTag` class to detect and analyze AprilTags in an image.

Prerequisites
--------------------------------------------------

You should first read the following tutorials:

- :ref:`tutorial-image-getting-started`
- :ref:`tutorial-image-display`
- :ref:`tutorial-image-io`

You should also consult the part on how to get a camera intrinsics parameters
in the :ref:`tutorial-camera-realsense` tutorial.

Detect an AprilTag
==================================================

The following :ref:`example <code-detection-apriltag>` loads an image,
detects AprilTags in it, estimates their poses, and displays the results:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :linenos:

Running the example
--------------------------------------------------

You can run the example with:

.. code-block:: bash

  python3 $VISP_WS/visp/modules/python/examples/detection/tutorial-detection-apriltag.py

Result
--------------------------------------------------

The example displays the input image in grayscale followed by 
the addition of the detected tag borders and coordinate frames:

.. list-table::

  * - .. image:: images/result-detection-apriltag-input.png

    - .. image:: images/result-detection-apriltag-output.png

It also prints the number of detected tags, their IDs, decision margins,
and Hamming distances:

.. code-block:: text

  Detected 12 AprilTag(s)
  Tag IDs: [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
  Decision margins: [78.44377899169922, 86.25739288330078, 101.34090423583984, 74.03752899169922, 101.83423614501953, 108.31098175048828, 104.10408020019531, 97.47992706298828, 97.76911163330078, 118.44223022460938, 116.25943756103516, 81.77754211425781]
  Hamming distances: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

Explanation
==================================================

The program performs four main tasks:

1. It creates a camera model and loads the image.
2. It detects all AprilTags in the image.
3. It retrieves information about the detected tags.
4. It displays the tag borders and coordinate frames.

Import the required modules
--------------------------------------------------

We will need :py:class:`~visp.core.CameraParameters` to describe the camera model,
:py:class:`~visp.detection.DetectorAprilTag` to detect the tags, and :py:class:`~visp.core.ImageRGBa`
and :py:class:`~visp.core.Display` classes to store and display the image we will manipulate.

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: import sys
  :end-at: from visp.python.display_utils import get_display

Configure the camera and read the image
--------------------------------------------------

We first define the path of the image we will manipulate and the size of the AprilTags.
The latter is necessary to correctly estimate their positions:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Configuration
  :end-at: TAG_SIZE = 0.053 # in meters

To estimate the tag poses accurately, we also need to specify the camera intrinsic parameters.
They consist of the horizontal and vertical focal lengths and the coordinates of the principal point:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Camera intrinsics parameters
  :end-at: CAMERA_V0 = 243.4373779 # principal point y

These values must correspond to the camera used to acquire the image.
Using incorrect parameters can result in inaccurate pose estimates.

We then store the intrisics parameters in a
:py:class:`~visp.core.CameraParameters` object:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Create the camera model
  :end-at: camera.initPersProjWithoutDistortion(CAMERA_PX, CAMERA_PY, CAMERA_U0, CAMERA_V0)

We finally load the image in a :py:class:`~visp.core.ImageGray` object instead of :py:class:`~visp.core.ImageRGBa`,
because AprilTag detection operates on grayscale images:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Read the image
  :end-at: sys.exit(1)

Then we display it:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Display the image
  :end-at: Display.getClick(image)

Detect and inspect the AprilTags
--------------------------------------------------

We create a :py:class:`~visp.detection.DetectorAprilTag` object
and call its :py:meth:`~visp.detection.DetectorAprilTag.detect`
method to detect all AprilTags in the image:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Detect the tags on the image
  :end-at: detected, tag_poses = detector.detect(image, TAG_SIZE, camera)

It returns us a boolean ``detected`` which indicate whether a tag was detected,
and the estimated pose of each detected tag ``tag_poses``.

The boolean can be used to exit the program if no AprilTags are found:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: if not detected:
  :end-at:   sys.exit(0)

Get the tag information
--------------------------------------------------

We can obtain the number of detected tags with
:py:meth:`~visp.detection.DetectorAprilTag.getNbObjects`:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: number_of_tags = detector.getNbObjects()
  :end-at: print(f"Detected {number_of_tags} AprilTag(s)")

The detector also provides additional information about the detections.
We retrieve the identifier of each detected tag,
the decision margin associated with each detection,
the Hamming distances, and the image coordinates of the detected tag corners.

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Print the tag information
  :end-at: tag_corners = detector.getTagsCorners()

These entries follow the order of the
detected tags IDs.

Display the tag frames
--------------------------------------------------

We initialize a :py:class:`~visp.core.Display` using the input image:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Display the image with tags frames and axes
  :end-at: Display.display(image)

The display uses the same :py:class:`~visp.core.ImageGray` object that was
passed to the detector. This makes it possible to draw the detection results
directly on the image.

We draw the detected tag corners using
:py:meth:`~visp.detection.DetectorAprilTag.displayTags`:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: detector.displayTags(image, tag_corners, Color.none, 3)
  :end-at: detector.displayTags(image, tag_corners, Color.none, 3)

The ``tag_corners`` argument contains the image coordinates returned by
:py:meth:`~visp.detection.DetectorAprilTag.getTagsCorners`.
The final argument specifies the drawing thickness.

We can visualize the estimated tag poses with
:py:meth:`~visp.detection.DetectorAprilTag.displayFrames`:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Draw the tags borders and coordinate frames
  :end-at: detector.displayFrames(image, tag_poses, camera, TAG_SIZE / 2.0, Color.none, 3)

This method uses the estimated poses, the camera parameters, and the tag size
to display the coordinate frames. The frames represent the estimated position and
orientation of the tags relative to the camera.

.. note::

  The frames are drawn in the display window, and thus do not modify the image.

Finally, we render the image with the tag poses and frames,
and wait for a mouse click:

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: Display.flush(image)
  :end-at: Display.getClick(image)

Other options
==================================================

Create an AprilTag marker
--------------------------------------------------

You can quickly generate an AprilTag, using this `website <https://chev.me/arucogen/>`_.

Alternatively, you can download pre-generated tag families on the
`apriltag-imgs GitHub repository <https://github.com/AprilRobotics/apriltag-imgs>`_.