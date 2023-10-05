import bpy
from mathutils import Matrix

# https://blender.stackexchange.com/questions/15102/what-is-blenders-camera-projection-matrix-model/38189#38189
def get_calibration_matrix_K_from_blender(camd):
  """! Get the camera matrix from Blender
  @param[in] camd Camera data.
  @return Camera matrix K that contains intrinsic parameters.
  """
  f_in_mm = camd.lens
  scene = bpy.context.scene
  resolution_x_in_px = scene.render.resolution_x
  resolution_y_in_px = scene.render.resolution_y
  scale = scene.render.resolution_percentage / 100
  sensor_width_in_mm = camd.sensor_width
  sensor_height_in_mm = camd.sensor_height
  pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
  if (camd.sensor_fit == 'VERTICAL'):
    # the sensor height is fixed (sensor fit is horizontal),
    # the sensor width is effectively changed with the pixel aspect ratio
    s_u = resolution_x_in_px * scale / sensor_width_in_mm / pixel_aspect_ratio
    s_v = resolution_y_in_px * scale / sensor_height_in_mm
  else: # 'HORIZONTAL' and 'AUTO'
    # the sensor width is fixed (sensor fit is horizontal),
    # the sensor height is effectively changed with the pixel aspect ratio
    pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
    s_u = resolution_x_in_px * scale / sensor_width_in_mm
    s_v = resolution_y_in_px * scale * pixel_aspect_ratio / sensor_height_in_mm

  # Parameters of intrinsic calibration matrix K
  alpha_u = f_in_mm * s_u
  alpha_v = f_in_mm * s_v
  u_0 = resolution_x_in_px*scale / 2
  v_0 = resolution_y_in_px*scale / 2
  skew = 0 # only use rectangular pixels

  K = Matrix(
    ((alpha_u, skew,    u_0),
    (    0  ,  alpha_v, v_0),
    (    0  ,    0,      1 )))
  return K

if __name__ == "__main__":
  """! Python script that dials with Blender to retrieve camera intrinsic parameters.
  """
  # Modify your camera name below 
  # If you follow carefully the totorial, you can set "Camera" for the color camera and "Camera.001" for the depth camera
  camera_name = "Camera.001"
  K = get_calibration_matrix_K_from_blender(bpy.data.objects[camera_name].data)
  print(f"Intrinsics for {camera_name} are K = \n{K}")
