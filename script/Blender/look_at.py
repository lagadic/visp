#          Apache License
#    Version 2.0, January 2004
# http://www.apache.org/licenses/

from __future__ import print_function
import numpy as np
import bpy
from mathutils import *

# https://blender.stackexchange.com/questions/5210/pointing-the-camera-in-a-particular-direction-programmatically
def look_at(obj_camera, point):
    loc_camera = obj_camera.matrix_world.to_translation()

    direction = point - loc_camera
    # point the cameras '-Z' and use its 'Y' as up
    rot_quat = direction.to_track_quat('-Z', 'Y')

    # assume we're using euler rotation
    obj_camera.rotation_euler = rot_quat.to_euler()

def print_pose(pose):
    for i in range(4):
        for j in range(4):
            print(f"{str(pose[i][j])} ", end='')
        print()

def print_camera_pose(cameraName, objectName):
    # OpenGL to Computer vision camera frame convention
    M = Matrix().to_4x4()
    M[1][1] = -1
    M[2][2] = -1

    cam = bpy.data.objects[cameraName]
    object_pose = bpy.data.objects[objectName].matrix_world

    # Normalize orientation with respect to the scale
    object_pose_normalized = object_pose.copy()
    object_orientation_normalized = object_pose_normalized.to_3x3().normalized()
    for i in range(3):
        for j in range(3):
            object_pose_normalized[i][j] = object_orientation_normalized[i][j]

    camera_pose = M @ cam.matrix_world.inverted() @ object_pose_normalized
    print(f"cMo:")
    print_pose(camera_pose)
    print(f"oMc:")
    print_pose(camera_pose.inverted())

def save_camera_frame(scene):
    scene.render.filepath = '/tmp/%04d.png' % scene.frame_current
    bpy.ops.render.render(write_still=True)
    scene.frame_current += 1

def main():
    # Set camera pose to an arbitrary pose, but not looking toward the cube
    obj_camera = bpy.data.objects["Camera"]
    obj_camera.rotation_euler = [np.radians(10), np.radians(10), np.radians(10)]
    obj_camera.location = [8.867762565612793, -1.1965436935424805, 2.1211400032043457]

    scene = bpy.context.scene
    save_camera_frame(scene)

    # Print camera pose and its inverse
    cameraName = "Camera"
    cubeName = "Cube"
    print_camera_pose(cameraName, cubeName)

    # Look-at the cube center
    cube_pose = bpy.data.objects[cubeName]
    look_at(obj_camera, cube_pose.location)

    save_camera_frame(scene)
    print_camera_pose(cameraName, cubeName)

if __name__ == '__main__':
    main()
