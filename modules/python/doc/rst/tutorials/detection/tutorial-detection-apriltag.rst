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

The example displays the input image in grayscale and in overlay the detected tag borders and coordinate frames:

.. list-table::

  * - .. image:: images/result-detection-apriltag-input.png

    - .. image:: images/result-detection-apriltag-output.png

It also prints the number of detected tags, their IDs, decision margins, Hamming distances and for each tag
its center of gravity and the coordinates of the 4 corners:

.. code-block:: text

  Detected 12 AprilTag(s)
  Tag IDs: [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
  Decision margins: [78.44377899169922, 86.25739288330078, 101.3409194946289, 74.03752899169922, 101.83423614501953, 108.31098175048828, 104.10406494140625, 97.47992706298828, 97.76911163330078, 118.44223022460938, 116.25943756103516, 81.77754211425781]
  Hamming distances: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  Tag 0 with ID 8 has cog: 54.2943, 286.518 and has 4 corners: [51.5299, 245.279, 74.2249, 283.907, 56.063, 327.523, 35.3594, 289.362]
  Tag 1 with ID 9 has cog: 91.2506, 334.585 and has 4 corners: [74.745, 302.952, 106.629, 302.952, 107.699, 368.496, 75.9296, 363.94]
  Tag 2 with ID 10 has cog: 118.824, 406.399 and has 4 corners: [116.01, 359.003, 145.225, 410.403, 119.874, 453.762, 94.1869, 402.429]
  Tag 3 with ID 11 has cog: 82.39, 233.993 and has 4 corners: [96.7007, 269.058, 66.1396, 262.882, 67.399, 201.307, 99.3205, 202.725]
  Tag 4 with ID 12 has cog: 123.084, 274.781 and has 4 corners: [119.996, 226.395, 150.021, 271.028, 124.646, 323.348, 97.6717, 278.352]
  Tag 5 with ID 13 has cog: 147.621, 347.246 and has 4 corners: [144.569, 296.831, 177.27, 347.959, 149.087, 397.517, 119.556, 346.676]
  Tag 6 with ID 14 has cog: 104.749, 159.094 and has 4 corners: [101.686, 111.292, 130.168, 148.714, 106.84, 207.385, 80.3039, 168.984]
  Tag 7 with ID 15 has cog: 143.151, 199.523 and has 4 corners: [139.507, 148.153, 172.79, 190.745, 145.026, 251.276, 115.284, 207.918]
  Tag 8 with ID 16 has cog: 189, 257.17 and has 4 corners: [157.104, 243.891, 197.746, 201.936, 223.189, 272.666, 177.962, 310.188]
  Tag 9 with ID 17 has cog: 137.71, 73.5261 and has 4 corners: [133.703, 21.6882, 167.46, 56.248, 139.955, 126.626, 109.722, 89.5427]
  Tag 10 with ID 18 has cog: 183.489, 119.469 and has 4 corners: [207.731, 68.1279, 208.076, 154.342, 160.17, 166.989, 157.977, 88.4176]
  Tag 11 with ID 19 has cog: 237.174, 179.625 and has 4 corners: [232.618, 117.444, 280.43, 167.733, 238.449, 242.11, 197.199, 191.212]

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
the decision margin associated with each detection and the Hamming distances.

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Get and print the tag information
  :end-at: print("Hamming distances:", detector.getTagsHammingDistance())

We can also retrieve the image coordinates of the detected tag center of gravity (cog) and the 4 corners.
These entries follow the order of the detected tags IDs.

.. literalinclude:: /examples/detection/tutorial-detection-apriltag.py
  :language: python
  :start-at: # Get and print the tag cog and corners
  :end-at: print(f"Tag {i} with ID {tags_ids[i]} has cog: {detector.getCog(i)} and has 4 corners: {tag_corners[i]}")

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
