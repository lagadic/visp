import bpy
import os
from mathutils import Matrix
import sys

def save_depth_M_color_file(prefix_data, scene, camera_name_rgb, camera_name_depth):
  """! Save depth_M_color.txt file.
  @param[in] prefix_data String corresponding to the folder where depth_M_color.txt file has to be saved.
  @param[in] scene Scene handler.
  @param[in] camera_name_rgb String corresponding to the name of the color camera to consider.
  @param[in] camera_name_depth String corresponding to the name of the depth camera to consider.
  """

  w_M_c = get_object_pose_in_world(camera_name_rgb)
  w_M_d = get_object_pose_in_world(camera_name_depth)
  d_M_c = w_M_d.inverted() @ w_M_c
  print("d_M_c: \n", d_M_c)

  filename = prefix_data + "/depth_M_color.txt"
  save_pose(filename, d_M_c)
  print("Saved: ", filename)

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

  print("M: \n", M)

  w_M_c = get_object_pose_in_world(camera_name)
  w_M_o = get_object_pose_in_world(object_name)

  print("w_M_c: \n", w_M_c)
  print("w_M_o: \n", w_M_o)

  c_M_o = M @ w_M_c.inverted() @ w_M_o

  print("c_M_o:\n", c_M_o)
  return c_M_o

def save_pose(filename, pose):
  """! Save pose in a .txt file.
  @param[in] filename String corresponding to the file name that will contain the saved pose.
  @param[in] pose Object pose to save as a 4-by-4 homogeneous matrix.
  """
  if not os.path.exists(prefix_pose):
    os.makedirs(prefix_pose)

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


def my_handler(prefix_pose, scene, camera_name, object_name):
  """! Handler that does the job for a given rendered frame.
  @param[in] prefix_pose String corresponding to the folder name that will contain the saved poses.
  @param[in] scene Scene handler.
  @param[in] camera_name String corresponding to the name of the camera to consider.
  @param[in] object_name String corresponding to the name of the object to track.
  """
  frame_number = scene.frame_current
  print("\n\nCurrent Frame", scene.frame_current)
  camera_pose = get_camera_pose(camera_name, object_name)

  filename = prefix_pose + camera_name + "_%03d" % frame_number + ".txt"
  save_pose(filename, camera_pose)

if __name__ == '__main__':
  """! Python script that save camera poses and RGB and depth images.
  """
  camera_name_rgb = "Camera_L"
  camera_name_depth = "Camera_R"
  object_name = "teabox"
  prefix_data = "/tmp/teabox/"
  prefix_pose = prefix_data  + "camera_poses/"
  prefix_rgb_image = prefix_data  + "images/"

  if not os.path.exists(prefix_data):
    print("fCreate {prefix_data}")
    os.makedirs(prefix_data)

  scene = bpy.context.scene

  save_depth_M_color_file(prefix_data, scene, camera_name_rgb, camera_name_depth)

  for step in range(scene.frame_start, scene.frame_end):
    # Set render frame
    scene.frame_set(step)

    # Set filename and render
    if not os.path.exists(prefix_rgb_image):
      os.makedirs(prefix_rgb_image)
    scene.render.filepath = (prefix_rgb_image + '%04d.png') % step
    bpy.ops.render.render( write_still=True )

    my_handler(prefix_pose, scene, camera_name_rgb, object_name)
