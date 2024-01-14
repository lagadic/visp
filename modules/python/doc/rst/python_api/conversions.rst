Using ViSP with other libraries
===============================================

ViSP provides multiple types to manipulate mathematical objects, such as:

* Vectors

  * :py:class:`visp.core.ColVector`
  * :py:class:`visp.core.RowVector`
  * :py:class:`visp.core.ThetaUVector`

* Matrices

  * :py:class:`visp.core.Matrix`
  * :py:class:`visp.core.RotationMatrix`
  * :py:class:`visp.core.HomogeneousMatrix`


While these representations should allow you to work with all the ViSP functions,
they are foreign to all the other Python libraries.

For most scientific computing libraries, the standard data representation is based on `NumPy <https://numpy.org/>`_.
Since most libraries will accept and manipulate these arrays, ViSP provides conversion functions.


NumPy ↔ ViSP
-----------------------------------------



Mapping between NumPy arrays and ViSP types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here, we give the list of all supported NumPy-convertible types

.. list-table:: Core math types
   :header-rows: 1

   * - Python type
     - NumPy Shape and dtype
   * - :py:class:`visp.core.Matrix`
     - (N, M), np.float64
   * - :py:class:`visp.core.ColVector`
     - (M,), np.float64
   * - :py:class:`visp.core.RowVector`
     - (N,), np.float64
   * - :py:class:`visp.core.RotationMatrix`
     - (3, 3), np.float64
   * - :py:class:`visp.core.HomogeneousMatrix`
     - (4, 4), np.float64
   * - :py:class:`visp.core.ThetaUVector`
     - (3,), np.float64

.. list-table:: Core image types
   :header-rows: 1

   * - C++ type
     - Python type
     - NumPy Shape and dtype
   * - :code:`vpImage<unsigned char>`
     - :py:class:`visp.core.ImageGray`
     - (H, W), np.uint8
   * - :code:`vpImage<uint16_t>`
     - :py:class:`visp.core.ImageUInt16`
     - (H, W), np.uint16
   * - :code:`vpImage<vpRGBa>`
     - :py:class:`visp.core.ImageRGBa`
     - (H, W, 4), np.uint8
   * - :code:`vpImage<vpRGBf>`
     - :py:class:`visp.core.ImageRGBf`
     - (H, W, 3), np.float32



Acquiring a view of a ViSP object
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is possible to view a ViSP object as a NumPy array. When using a view, changes made to one representation is reflected in the other.

See `the NumPy documentation <https://numpy.org/doc/stable/user/basics.copies.html>`_ for more information.

To reinterpret a supported ViSP object as a Numpy array, use either:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  list_representation = [i for i in range(3)]
  vec = ColVector(list_representation) # Initialize a 3 vector from a list
  np_vec = vec.numpy() # A 1D numpy array of size 3

  print(np.all(np_vec == list_representation))


  vec *= 2.0
  print(np.all(np_vec == list_representation))

.. testoutput::

  True
  False


or

.. testcode::

  from visp.core import ColVector
  import numpy as np

  list_representation = [i for i in range(3)]
  vec = ColVector(list_representation) # Initialize a 3 vector from a list
  np_vec = np.array(vec, copy=False) # A 1D numpy array of size 3

  print(np.all(np_vec == list_representation))
  # Modifying the ViSP vector modifies the NumPy view
  vec *= 2.0
  print(np.all(np_vec == list_representation))

  # Modifying the NumPy array modifies the ViSP object
  np_vec[:2] = 0.0
  print(vec[0] == 0.0 and vec[1] == 0.0)

.. testoutput::

  True
  False
  True


Note that with these methods, some ViSP objects cannot be modified.
That is the case for :py:class:`visp.core.HomogeneousMatrix` and :py:class:`visp.core.RotationMatrix`, where an undesired modification
may lead to an invalid representation (Such as a rotation matrix not conserving its properties)

Thus, this code will not work:

.. testcode::

  from visp.core import RotationMatrix, HomogeneousMatrix
  import numpy as np

  R = RotationMatrix()
  R.numpy()[0, 1] = 1.0

  T = HomogeneousMatrix()
  T.numpy()[0, 1] = 1.0

.. testoutput::
  :options: +IGNORE_EXCEPTION_DETAIL

  Traceback (most recent call last):
   File "<stdin>", line 1, in <module>
  ValueError: assignment destination is read-only
  Traceback (most recent call last):
   File "<stdin>", line 1, in <module>
  ValueError: assignment destination is read-only


Copying to a NumPy array
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To obtain a copy of the ViSP representation you can simply use:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  vec = ColVector(3, 0)
  np_vec = vec.numpy().copy() # or np.array(vec, copy=True)
  np_vec[0] = 1

  print(np_vec[0] == vec[0])

.. testoutput::

  False


Keep in mind that it may be preferable to use a copy of the data, especially if you are using both numpy and ViSP representations for different tasks at the same time

For instance, the following code will lead to an undesired behaviour:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  def compute_velocity(velocity: ColVector) -> None:
    # Dummy function to illustrate in place ops
    velocity *= 2.0 # This code modifies the content of velocity

  velocity = ColVector(6, 1.0)
  iteration = 0
  # Store the velocities in a list
  log_data = []

  # Servoing loop
  while iteration < 10:
    compute_velocity(velocity)
    log_data.append(velocity.numpy())
    iteration += 1

  # Do some logging...
  print(log_data[0])
  print(log_data[-1])

.. testoutput::

  [1024. 1024. 1024. 1024. 1024. 1024.]
  [1024. 1024. 1024. 1024. 1024. 1024.]


Although we're multiplying the velocity by 2 at each iteration,
we can see that we have the same values for the first and last iterations.


.. warning::

  In essence, this is because while we store 10 different NumPy arrays, they all share the same underlying storage.
  This storage is, at each iteration, modified by the :python:`compute_velocity` function.

.. note::

  To remedy this, you can either:

  * Make a copy of the NumPy array at every iteration before storing it in the list :python:`log_data.append(velocity.numpy().copy())`
  * Change the :python:`compute_velocity` to return a new :py:class:`visp.core.ColVector`


Building a ViSP object from a NumPy array
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the above section, we have shown how to convert a ViSP representation to a NumPy array.

To perform the inverse operation, a custom constructor is defined for each class that allows the Numpy → ViSP conversion.

This constructor performs a **copy** of the NumPy data into the newly created ViSP object.

For instance, to build a new matrix

.. testcode::

  from visp.core import Matrix
  import numpy as np

  random_mat = np.random.rand(5, 10) # 10 x 10 random matrix

  mat = Matrix(random_mat)
  print(mat.getRows(), mat.getCols())
  print(np.all(random_mat == mat))

  # We built a matrix by copying the numpy array: modifying one does not impact the other
  random_mat[:, 0] = 0
  print(np.all(random_mat == mat))

.. testoutput::

  5 10
  True
  False


.. warning::

  A way to build a ViSP object as a view of a NumPy array is still lacking

Numpy-like indexing of ViSP arrays
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ViSP data types now support numpy-like indexing, and methods like slicing and iterating on values.

To read values, rows and columns of a Matrix, you can use:

.. testcode::

  from visp.core import Matrix

  m = Matrix(2, 3, 1.0)
  print(m[0, 0])
  print(m[0]) # First row
  print(m[:, 0]) # First column


.. testoutput::

  1.0
  [1. 1. 1.]
  [1. 1.]


Using RealSense cameras with ViSP
---------------------------------------

In the C++ version of ViSP, a class is provided to work with Intel cameras such as the D435.
This class, that acts as a thin wrapper around the Realsense libraries, cannot be used in Python.

Instead we recommend to use the Python wrapper provided by Intel, :code:`pyrealsense2`.

You can install it with:

  python -m pip install pyrealsense2

This library allows us to acquire frames from the camera.
It is possible to convert them to a numpy representation,
and they can thus be used with ViSP.

The code below demonstrates how to use the Realsense package with ViSP:

.. code-block:: python

  import pyrealsense2 as rs
  import numpy as np
  from visp.core import CameraParameters
  from visp.core import ImageRGBa, ImageUInt16, ImageGray
  from visp.core import ImageConvert, Display
  from visp.gui import DisplayX

  def cam_from_rs_profile(profile) -> CameraParameters:
    '''Get camera intrinsics from the realsense framework'''
    # Downcast to video_stream_profile and fetch intrinsics
    intr = profile.as_video_stream_profile().get_intrinsics()
    return CameraParameters(intr.fx, intr.fy, intr.ppx, intr.ppy)

  if __name__ == '__main__':

    # Initialize realsense2
    pipe = rs.pipeline()
    config = rs.config()
    fps = 60
    h, w = 480, 640
    config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, w, h, rs.format.rgba8, fps)

    cfg = pipe.start(config)

    I_gray = ImageGray(h, w)
    display_gray = DisplayX()
    display_gray.init(I_gray, 0, 0, 'Color')
    I_depth_hist = ImageGray(h, w)
    display_depth = DisplayX()
    display_depth.init(I_depth_hist, 640, 0, 'Color')


    # Retrieve intrinsics
    cam_color = cam_from_rs_profile(cfg.get_stream(rs.stream.color))
    cam_depth = cam_from_rs_profile(cfg.get_stream(rs.stream.depth))

    point_cloud_computer = rs.pointcloud()
    while True:
      frames = pipe.wait_for_frames()
      color_frame = frames.get_color_frame()
      depth_frame = frames.get_depth_frame()
      # NumPy Representations of realsense frames
      I_color_np = np.asanyarray(color_frame.as_frame().get_data())
      I_depth_np = np.asanyarray(depth_frame.as_frame().get_data())
      # ViSP representations
      I_color = ImageRGBa(I_color_np) # This works because format is rs.format.rgba8, otherwise concat or conversion needed
      I_depth = ImageUInt16(I_depth_np)
      # Transform depth frame as point cloud and view it as an N x 3 numpy array
      point_cloud = np.asanyarray(point_cloud_computer.calculate(depth_frame).get_vertices()).view((np.float32, 3))

      ImageConvert.convert(I_color, I_gray)
      ImageConvert.createDepthHistogram(I_depth, I_depth_hist)

      Display.display(I_gray)
      Display.display(I_depth_hist)
      Display.flush(I_gray)
      Display.flush(I_depth_hist)
