import bpy
import os
from mathutils import Matrix
import sys

def save_depth_M_color_file(prefix_data, scene, camera_name_color, camera_name_depth):
  """! Save depth_M_color.txt file.
  @param[in] prefix_data String corresponding to the folder where depth_M_color.txt file has to be saved.
  @param[in] scene Scene handler.
  @param[in] camera_name_color String corresponding to the name of the color camera to consider.
  @param[in] camera_name_depth String corresponding to the name of the depth camera to consider.
  """

  w_M_c = get_object_pose_in_world(camera_name_color)
  w_M_d = get_object_pose_in_world(camera_name_depth)
  d_M_c = w_M_d.inverted() @ w_M_c
  #print("d_M_c: \n", d_M_c)

  filename = prefix_data + "depth_M_color.txt"
  save_pose(filename, d_M_c)
  print(f"Saved: {filename}")

def get_object_pose_in_world(object_name):
  """! Return the object pose in the world frame w_M_o.
  @param[in] object_name  String corresponding to the name of the object to consider.
  @return The 4-by-4 homogeneous matrix corresponding to w_M_object.
  """
  w_M_object = bpy.data.objects[object_name].matrix_world
  w_M_object_normalized = w_M_object.copy()
  w_R_object_normalized = w_M_object_normalized.to_3x3().normalized()
  for i in range(3):
    for j in range(3):
      w_M_object_normalized[i][j] = w_R_object_normalized[i][j]
  return w_M_object_normalized

def get_camera_pose(camera_name, object_name):
  """! Return the object pose in the camera frame as an homogenous matrices.
  @param[in] camera_name  String corresponding to the name of the camera to consider.
  @param[in] object_name  String corresponding to the name of the object to consider.
  @return The 4-by-4 homogeneous matrix corresponding to c_M_o.
  """

  # OpenGL to Computer vision camera frame convention
  M = Matrix().to_4x4()
  M[1][1] = -1
  M[2][2] = -1

  #print("M: \n", M)

  w_M_c = get_object_pose_in_world(camera_name)
  w_M_o = get_object_pose_in_world(object_name)

  #print("w_M_c: \n", w_M_c)
  #print("w_M_o: \n", w_M_o)

  c_M_o = M @ w_M_c.inverted() @ w_M_o

  #print("c_M_o:\n", c_M_o)
  return c_M_o

def save_pose(filename, pose):
  """! Save pose in a .txt file.
  @param[in] filename String corresponding to the file name that will contain the saved pose.
  @param[in] pose Object pose to save as a 4-by-4 homogeneous matrix.
  """

  print(f"Save: {filename}")
  with open(filename, 'w') as f:
    f.write(str(pose[0][0]) + " ")
    f.write(str(pose[0][1]) + " ")
    f.write(str(pose[0][2]) + " ")
    f.write(str(pose[0][3]) + " ")
    f.write("\n")

    f.write(str(pose[1][0]) + " ")
    f.write(str(pose[1][1]) + " ")
    f.write(str(pose[1][2]) + " ")
    f.write(str(pose[1][3]) + " ")
    f.write("\n")

    f.write(str(pose[2][0]) + " ")
    f.write(str(pose[2][1]) + " ")
    f.write(str(pose[2][2]) + " ")
    f.write(str(pose[2][3]) + " ")
    f.write("\n")

    f.write(str(pose[3][0]) + " ")
    f.write(str(pose[3][1]) + " ")
    f.write(str(pose[3][2]) + " ")
    f.write(str(pose[3][3]) + " ")
    f.write("\n")

  return

def save_color_camera_pose(prefix, scene, camera_name, object_name):
  """! Save homogeneous transformation from camera to object frames (cMo).
  @param[in] prefix String corresponding to the folder name that will contain the saved poses.
  @param[in] scene Scene handler.
  @param[in] camera_name String corresponding to the name of the camera to consider.
  @param[in] object_name String corresponding to the name of the object to track.
  """
  frame_number = scene.frame_current
  print("\n\nCurrent Frame", scene.frame_current)
  camera_pose = get_camera_pose(camera_name, object_name)

  filename = prefix + camera_name_color + "_%04d" % frame_number + ".txt"
  save_pose(filename, camera_pose)

def get_intrinsics(camera_name):
  """! Save camera intrinsics in xml.
  @param[in] camera_name String corresponding to the name of the camera to consider.
  """
  cam_data = bpy.data.objects[camera_name].data
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
  w = resolution_x_in_px
  h = resolution_y_in_px
  p_x = f * s_u
  p_y = f * s_v
  u_0 = resolution_x_in_px*scale / 2
  v_0 = resolution_y_in_px*scale / 2

  return w, h, p_x, p_y, u_0, v_0

def save_intrinsics(filename, camera_name, w, h, p_x, p_y, u_0, v_0):
  """! Save camera intrinsics in xml.
  @param[in] filename Name of the file that will contain the intrinsics in xml format.
  @param[in] camera_name Camera name.
  @param[in] w, h  Image size.
  @param[in] p_x, p_y  Ratio between the focal length and the size of the pixel.
  @param[in] u_0, v_0  Coordinates of the principal point.
  """
  print(f"Save: {filename}")
  with open(filename, 'w') as f:
    f.write("<?xml version=\"1.0\"?>\n")
    f.write("<root>\n")
    f.write("  <camera>\n")
    f.write(f"    <name>{camera_name}</name>\n")
    f.write(f"    <image_width>{w}</image_width>\n")
    f.write(f"    <image_height>{h}</image_height>\n")
    f.write("    <model>\n")
    f.write("      <type>perspectiveProjWithoutDistortion</type>\n")
    f.write(f"      <px>{p_x}</px>\n")
    f.write(f"      <py>{p_y}</py>\n")
    f.write(f"      <u0>{u_0}</u0>\n")
    f.write(f"      <v0>{v_0}</v0>\n")
    f.write("    </model>\n")
    f.write("  </camera>\n")
    f.write("</root>\n")
    f.write("\n")

if __name__ == '__main__':
  """! Python script that save camera poses, color and depth images, but also depth to color transform.
  """
  camera_name_color = "Camera_L"
  camera_name_depth = "Camera_R"
  object_name = "teabox"
  prefix_data = "/tmp/teabox/"
  prefix_pose = prefix_data  + "ground-truth/"
  prefix_color_images = prefix_data  + "color/"
  prefix_depth_images = prefix_data  + "depth/"
  images_suffix = "JPEG"

  if not os.path.exists(prefix_data):
    print(f"Create {prefix_data}")
    os.makedirs(prefix_data)

  if not os.path.exists(prefix_pose):
    print(f"Create {prefix_pose}")
    os.makedirs(prefix_pose)

  scene = bpy.context.scene

  save_depth_M_color_file(prefix_data, scene, camera_name_color, camera_name_depth)

  color_w, color_h, color_p_x, color_p_y, color_u_0, color_v_0 = get_intrinsics(camera_name_color)
  filename_color_intrinsics = prefix_data + camera_name_color + ".xml"
  save_intrinsics(filename_color_intrinsics, camera_name_color, color_w, color_h, color_p_x, color_p_y, color_u_0, color_v_0)

  depth_w, depth_h, depth_p_x, depth_p_y, depth_u_0, depth_v_0 = get_intrinsics(camera_name_depth)
  filename_depth_intrinsics = prefix_data + camera_name_depth + ".xml"
  save_intrinsics(filename_depth_intrinsics, camera_name_depth, depth_w, depth_h, depth_p_x, depth_p_y, depth_u_0, depth_v_0)

  for step in range(scene.frame_start, scene.frame_end):
    # Set render frame
    scene.frame_set(step)

    # Set filename and render
    if not os.path.exists(prefix_color_images):
      os.makedirs(prefix_color_images)
    scene.render.filepath = (prefix_color_images + '%04d') % step
    scene.render.image_settings.file_format = images_suffix
    bpy.ops.render.render( write_still=True )

    save_color_camera_pose(prefix_pose, scene, camera_name_color, object_name)

    # Remove useless rendered images to keep only
    # - color images from Camera_L
    # - depth images from Camera_R
    filename_color_R = (prefix_color_images + '%04d' + "_R" + scene.render.file_extension) % step
    if not os.path.exists(filename_color_R):
      raise Exception(f"Color image {filename_color_R} doesn't exists. Should not occur.")

    print(f"Remove file: {filename_color_R}")
    os.remove(filename_color_R)

    filename_depth_L = (prefix_depth_images + 'Image%04d' + "_L" + ".exr") % step
    if not os.path.exists(filename_depth_L):
      raise Exception(f"Depth image {filename_depth_L} doesn't exists. Check if you set the extension to OpenEXR in the compositor.")

    print(f"Remove file: {filename_depth_L}")
    os.remove(filename_depth_L)
