.. _tutorial-color-segmentation:

========================================================
Introduction to color segmentation using HSV color scale
========================================================

Introduction
===========================

Goal
---------------------------

In this tutorial you will learn how to:

- Use the :py:class:`~visp.core.HSVucFullScale` class and :py:class:`~visp.core.ImageConvert` to convert an :py:class:`~visp.core.ImageRGBa` into a :py:class:`~visp.core.ImageHSVucFS`
- Use the :py:class:`~visp.core.ImageTools` class to perform color segmentation.

Prerequisites
---------------------------

You should first read the :ref:`Read and write an image file <tutorial-image-io>` tutorial.

Use the HSV class and convert from and to RGBa
==============================================

Code
---------------------------

The following :ref:`example <code-segmentation-color>` reads the ``ballons.jpg`` file, converts it into an HSV image encoded
using unsigned char, and performs color segmentation using predefined thresholds:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
	:language: python
	:linenos:

You can run the example with:

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/segmentation/tutorial-color-segmentation.py

Result
---------------------------

The example displays the input image followed by the mask and resulting segmented images:

.. list-table::

  * - .. image:: images/original.png

    - .. image:: images/mask.png

    - .. image:: images/segmented.png

Explanation
---------------------------

We first import the classes required to create, convert and display images.
We also import :py:class:`~visp.io.ImageTools`,
which we use to perform the segmentation:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :end-at: from visp.io import ImageIo

We define a helper function to display the images that we manipulate:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Function displaying an image
  :end-at:   Display.getClick(I, blocking=block)

.. note::

  For a more detailed explanation of how displays work in ViSP, see the :ref:`tutorial-image-display`
  tutorial.

We then read the image ``ballons.jpg`` using the :py:meth:`~visp.io.ImageIo.read` method.
We use an exception handler so that we can stop the program in case the reading fails because the remaining operations
require a valid input image:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Read the image
  :end-at:   sys.exit()

We define the HSV boundaries for the segmentation. We chose to use unsigned-char-encoded boundaries:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Definition of the HSV boundaries
  :end-at: hsv_range = [h_low, h_high, s_low, s_high, v_low, v_high]

We convert the input image into an HSV image. Here ``FS`` means "Full scale", because the Hue channel is encoded using
the 256 values an unsigned char can use. There is also the class :py:class:`~visp.core.ImageHSVucLS` if you want to use
the same limited range than OpenCV for the Hue channel. If we had used HSV boundaries expressed as floating number
between 0 and 1, we would have used an :py:class:`~visp.core.ImageHSVd` instead:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Converting the image
  :end-at: ImageConvert.convert(I, Ihsv)

We then compute the boolean mask indicating if a pixel respects the HSV boundaries with :py:meth:`~visp.core.ImageTools.inRange`.

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Computing the mask
  :end-at:   ImageTools.inRange(Ihsv, hsv_range, maskHSV)

We then compute the segmented image from the input image and the boolean mask with :py:meth:`~visp.core.ImageTools.inMask`.

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Computing the mask
  :end-at:   ImageTools.inMask(I, mask=maskHSV, I_mask=I_segmented)

Finally, we display the images:

.. literalinclude:: /examples/segmentation/tutorial-color-segmentation.py
  :language: python
  :start-at: # Display the loaded image

Next Tutorial
===========================

For the moment, no other tutorial on segmentation has been written in Python.
