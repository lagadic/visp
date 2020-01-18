import bpy
import os
from mathutils import *

prefix_pose = "/tmp/camera_poses/"
prefix_image = "/tmp/images/"

def get_camera_pose(cameraName, objectName, scene, frameNumber):
  if not os.path.exists(prefix_pose):
    os.makedirs(prefix_pose)

  # OpenGL to Computer vision camera frame convention
  M = Matrix().to_4x4()
  M[1][1] = -1
  M[2][2] = -1

  cam = bpy.data.objects[cameraName]
  object_pose = bpy.data.objects[objectName].matrix_world

  #Normalize orientation with respect to the scale
  object_pose_normalized = object_pose.copy()
  object_orientation_normalized = object_pose_normalized.to_3x3().normalized()
  for i in range(3):
    for j in range(3):
        object_pose_normalized[i][j] = object_orientation_normalized[i][j]

  camera_pose = M*cam.matrix_world.inverted()*object_pose_normalized
  print("camera_pose:\n", camera_pose)
  
  filename = prefix_pose + cameraName + "_%03d" % frameNumber + ".txt"
  with open(filename, 'w') as f:
    f.write(str(camera_pose[0][0]) + " ")
    f.write(str(camera_pose[0][1]) + " ")
    f.write(str(camera_pose[0][2]) + " ")
    f.write(str(camera_pose[0][3]) + " ")
    f.write("\n")

    f.write(str(camera_pose[1][0]) + " ")
    f.write(str(camera_pose[1][1]) + " ")
    f.write(str(camera_pose[1][2]) + " ")
    f.write(str(camera_pose[1][3]) + " ")
    f.write("\n")

    f.write(str(camera_pose[2][0]) + " ")
    f.write(str(camera_pose[2][1]) + " ")
    f.write(str(camera_pose[2][2]) + " ")
    f.write(str(camera_pose[2][3]) + " ")
    f.write("\n")

    f.write(str(camera_pose[3][0]) + " ")
    f.write(str(camera_pose[3][1]) + " ")
    f.write(str(camera_pose[3][2]) + " ")
    f.write(str(camera_pose[3][3]) + " ")
    f.write("\n")

  return


def my_handler(scene):
  frameNumber = scene.frame_current
  print("\n\nFrame Change", scene.frame_current)
  get_camera_pose("Camera", "tea_box_02", scene, frameNumber)

step_count = 250
scene = bpy.context.scene
for step in range(1, step_count):
  # Set render frame
  scene.frame_set(step)

  # Set filename and render
  if not os.path.exists(prefix_image):
    os.makedirs(prefix_image)
  scene.render.filepath = (prefix_image + '%04d.png') % step
  bpy.ops.render.render( write_still=True )

  my_handler(scene)
