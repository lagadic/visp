#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2024 ViSP contributor
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import bpy
import os
import numpy as np

# https://www.rojtberg.net/1601/from-blender-to-opencv-camera-and-back/
def get_calibration_matrix_K_from_blender(camera_name):
    # get the relevant data
    cam = bpy.data.objects[camera_name].data
    scene = bpy.context.scene
    # assume image is not scaled
    assert scene.render.resolution_percentage == 100
    # assume angles describe the horizontal field of view
    assert cam.sensor_fit != 'VERTICAL'

    f_in_mm = cam.lens
    sensor_width_in_mm = cam.sensor_width

    w = scene.render.resolution_x
    h = scene.render.resolution_y

    pixel_aspect = scene.render.pixel_aspect_y / scene.render.pixel_aspect_x

    f_x = f_in_mm / sensor_width_in_mm * w
    f_y = f_x * pixel_aspect

    # yes, shift_x is inverted. WTF blender?
    c_x = w * (0.5 - cam.shift_x)
    # and shift_y is still a percentage of width..
    c_y = h * 0.5 + w * cam.shift_y

    K = np.array([
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0,   0,   1]
    ])

    return K

def inv_transfo(w_T_o):
    R = w_T_o[:3,:3]
    t = w_T_o[:3,3]

    o_T_w = np.eye(4)
    o_T_w[:3,:3] = R.T
    o_T_w[:3,3] = -R.T @ t

    return o_T_w

def get_camera_pose(cameraName, objectName):
    M = np.eye(4)
    M[1][1] = -1
    M[2][2] = -1

    cam = bpy.data.objects[cameraName]
    object_pose = bpy.data.objects[objectName].matrix_world

    # Normalize orientation with respect to the scale
    object_pose_normalized_blender = object_pose.copy()
    object_orientation_normalized_blender = object_pose_normalized_blender.to_3x3().normalized()
    for i in range(3):
        for j in range(3):
            object_pose_normalized_blender[i][j] = object_orientation_normalized_blender[i][j]

    w_T_o = np.array(object_pose_normalized_blender)
    print(f"object_pose_normalized:\n{object_pose_normalized_blender}")

    w_T_c = np.array(cam.matrix_world)
    print(f"w_T_c:\n{w_T_c}")

    c_T_w = np.array(cam.matrix_world.inverted())
    print(f"c_T_w:\n{c_T_w}")

    c_T_w2 = inv_transfo(w_T_c)
    print(f"c_T_w2:\n{c_T_w2}")

    c_T_o = M @ c_T_w @ w_T_o
    print(f"c_T_o:\n{c_T_o}")

    return c_T_o

if __name__ == "__main__":
    K = get_calibration_matrix_K_from_blender("Camera")
    print(f"Camera Matrix:\n", K)

    output_dir = "/tmp"
    output_image_name = "blender_render.png"

    output_image_filepath = os.path.join(output_dir, output_image_name)
    bpy.context.scene.render.filepath = output_image_filepath
    bpy.ops.render.render(write_still = True)

    camera_name = "Camera"
    object_name = "Suzanne"
    c_T_o = get_camera_pose(camera_name, object_name)

    output_pose_name = "c_T_o.txt"
    output_pose_filepath = os.path.join(output_dir, output_pose_name)
    np.savetxt(output_pose_filepath, c_T_o)