import bpy
from mathutils import Matrix

# https://blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model/38189#38189
def get_calibration_matrix_K_from_blender(cam_data):
  """! Get the camera matrix from Blender
  @param[in] cam_data Camera data.
  @return Camera matrix K that contains intrinsic parameters.
  """
  f = cam_data.lens
  scene = bpy.context.scene
  resolution_x_in_px = scene.render.resolution_x
  resolution_y_in_px = scene.render.resolution_y
  scale = scene.render.resolution_percentage / 100
  sensor_width = cam_data.sensor_width
  sensor_height = cam_data.sensor_height
  pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
  if (cam_data.sensor_fit == 'VERTICAL'):
    # the sensor height is fixed (sensor fit is horizontal),
    # the sensor width is effectively changed with the pixel aspect ratio
    s_u = resolution_x_in_px * scale / sensor_width / pixel_aspect_ratio
    s_v = resolution_y_in_px * scale / sensor_height
  else: # 'HORIZONTAL' and 'AUTO'
    # the sensor width is fixed (sensor fit is horizontal),
    # the sensor height is effectively changed with the pixel aspect ratio
    pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
    s_u = resolution_x_in_px * scale / sensor_width
    s_v = resolution_y_in_px * scale * pixel_aspect_ratio / sensor_height

  # Parameters of intrinsic calibration matrix K
  p_x = f * s_u
  p_y = f * s_v
  u_0 = resolution_x_in_px*scale / 2
  v_0 = resolution_y_in_px*scale / 2
  skew = 0 # only use rectangular pixels

  K = Matrix(
    ((p_x, skew, u_0),
    (   0,  p_y, v_0),
    (   0,    0,   1 )))
  return K

if __name__ == "__main__":
  """! Python script that dials with Blender to retrieve camera intrinsic parameters.
  """
  # Modify your camera name below
  # If you follow carefully the tutorial, you can set "Camera_L" for the color camera and "Camera_R" for the depth camera
  camera_name = "Camera_R"
  K = get_calibration_matrix_K_from_blender(bpy.data.objects[camera_name].data)
  print(f"Intrinsics for {camera_name} are K = \n{K}")
