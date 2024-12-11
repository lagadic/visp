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
import cv2 as cv
import numpy as np
from mathutils import geometry, Vector

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
    # Camera frame in OpenGL:
    # X-axis to the right
    # Y-axis up
    # Z-axis backward
    cv_M_gl = np.eye(4)
    cv_M_gl[1][1] = -1
    cv_M_gl[2][2] = -1

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

    w_T_c = np.array(cam.matrix_world) @ cv_M_gl
    print(f"w_T_c:\n{w_T_c}")

    c_T_w = cv_M_gl @ np.array(cam.matrix_world.inverted())
    print(f"c_T_w:\n{c_T_w}")

    c_T_o = c_T_w @ w_T_o
    print(f"c_T_o:\n{c_T_o}")

    return c_T_o

def get_object_vertices(objectName):
    # https://blender.stackexchange.com/questions/3637/get-indices-of-vertices-of-triangulated-faces-in-python/3657#3657
    obj = bpy.data.objects[objectName]
    mesh = obj.data

    model_faces = []
    model_normals = []
    for face in mesh.polygons:
        face_vert = []
        for i in range(len(face.vertices)):
            vert = mesh.vertices[face.vertices[i]]
            vert_xyz = vert.co.xyz
            face_vert.append(vert_xyz)
            model_normals.append(vert.normal)

        model_faces.append(face_vert)

    return [model_faces, model_normals]

def vec_augment(pt_3d):
    return np.array([pt_3d[0], pt_3d[1], pt_3d[2], 1])

def convert_pt(pt_3d, w_T_o):
    o_pt = vec_augment(pt_3d)
    w_pt = w_T_o @ o_pt

    return w_pt[:,3]

def compute_ray(K, im_pt):
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]

    x = (im_pt[0] - cx) / fx
    y = (im_pt[1] - cy) / fy
    return np.array([x, y, 1])

def is_face_visible(ray_, vertex0_, normal_):
    # back-face culling
    # https://en.wikipedia.org/wiki/Back-face_culling
    ray = np.array(ray_)
    vertex0 = np.array(vertex0_)
    normal = np.array(normal_)
    eps = 1e-9
    return np.dot((vertex0 - ray), normal) > eps

def ray_vertex_intersection(ray, vertex, c_T_o):
    c_vert0 = c_T_o @ vec_augment(vertex[0])
    c_vert1 = c_T_o @ vec_augment(vertex[1])
    c_vert2 = c_T_o @ vec_augment(vertex[2])

    # https://docs.blender.org/api/4.3/mathutils.geometry.html#mathutils.geometry.intersect_ray_tri
    clip = True
    intersect = geometry.intersect_ray_tri(c_vert0, c_vert1, c_vert2, ray, [0,0,0], clip)
    return intersect

def getNED(lon_, lat_, r, in_radian=False):
    """
    Get the homogeneous transformation matrix corresponding to the local tangent plane transformation at the specified
    longitude/latitude and radius coordinates, using the NED and ECEF conventions and a perfect sphere.
    See also:
        - https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
        - https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates

    Parameters
    ----------
    lon_: float
        The longitude coordinate.
    lat_: float
        The latitude coordinate.
    r: float
        The sphere radius.
    in_radian: boolean
        If true coordinates are in radian, otherwise in degree.

    Returns:
    -------
    numpy matrix
        The homogeneous matrix allowing converting a 3D point expressed in the NED frame to the ECEF frame.
    """
    if not in_radian:
        # lambda
        lon = np.radians(lon_)
        # phi
        lat = np.radians(lat_)
    else:
        lon = lon_
        lat = lat_

    Tdata = [ [-np.sin(lat)*np.cos(lon), -np.sin(lon), -np.cos(lat)*np.cos(lon), r*np.cos(lon)*np.cos(lat)], \
              [-np.sin(lat)*np.sin(lon),  np.cos(lon), -np.cos(lat)*np.sin(lon), r*np.sin(lon)*np.cos(lat)], \
              [ np.cos(lat),              0,           -np.sin(lat),             r*np.sin(lat)], \
              [ 0,                        0,            0,                       1] \
            ]
    T = np.matrix(Tdata)

    return T

def getENU(lon_, lat_, r, in_radian=False):
    """
    Get the homogeneous transformation matrix corresponding to the local tangent plane transformation at the specified
    longitude/latitude and radius coordinates, using the ENU and ECEF conventions and a perfect sphere.
    See also:
        - https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
        - https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates

    Parameters
    ----------
    lon_: float
        The longitude coordinate.
    lat_: float
        The latitude coordinate.
    r: float
        The sphere radius.
    in_radian: boolean
        If true coordinates are in radian, otherwise in degree.

    Returns:
    -------
    numpy matrix
        The homogeneous matrix allowing converting a 3D point expressed in the ENU frame to the ECEF frame.
    """
    if not in_radian:
        # lambda
        lon = np.radians(lon_)
        # phi
        lat = np.radians(lat_)
    else:
        lon = lon_
        lat = lat_

    Tdata = [ [-np.sin(lon), -np.sin(lat)*np.cos(lon), np.cos(lat)*np.cos(lon), r*np.cos(lon)*np.cos(lat)], \
              [ np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)*np.sin(lon), r*np.sin(lon)*np.cos(lat)], \
              [ 0,            np.cos(lat),             np.sin(lat),             r*np.sin(lat)], \
              [ 0,            0,                       0,                       1] \
            ]
    T = np.matrix(Tdata)

    return T

def regular_on_sphere_points(num, full_sphere=False):
    """
    Generate equidistributed points on the surface of a sphere.
    From:
      - "How to generate equidistributed points on the surface of a sphere", Markus Deserno
      - https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf
      - https://gist.github.com/dinob0t/9597525

    Parameters
    ----------
    num: int
        The desired number of points on the surface of a sphere.

    Returns:
    -------
    list
        The list of equidistributed points on the surface of a sphere in the lon-lat coordinates.

    Note:
    -------
        This method does not return exactly the specified number of points.
    """
    r = 1
    points = []
    # Break out if zero points
    if num == 0:
        return points

    a = 4.0 * np.pi*(r**2.0 / num)
    d = np.sqrt(a)
    m_theta = int(round(np.pi / d))
    d_theta = np.pi / m_theta
    d_phi = a / d_theta
    pi_2 = np.pi/2

    if full_sphere:
        m_upper_bound = m_theta
    else:
        m_upper_bound = int(m_theta/2)

    for m in range(m_upper_bound):
        theta = np.pi * (m + 0.5) / m_theta
        m_phi = int(round(2.0 * np.pi * np.sin(theta) / d_phi))

        for n in range(m_phi):
            phi = 2.0 * np.pi * n / m_phi
            lon = phi
            lat = pi_2-theta
            points.append([lon,lat])

    return points

def look_at(ecef_T_cv, point):
    # https://blender.stackexchange.com/questions/5210/pointing-the-camera-in-a-particular-direction-programmatically/5220#5220
    # https://docs.blender.org/api/current/mathutils.html#mathutils.Vector
    direction = Vector((point[0], point[1], point[2])) - Vector((ecef_T_cv[0,3], ecef_T_cv[1,3], ecef_T_cv[2,3]))
    # point the cameras '-Z' and use its 'Y' as up
    rot_quat = direction.to_track_quat('-Z', 'Y')

    ecef_T_cv_look_at = np.eye(4)
    # https://docs.blender.org/api/current/mathutils.html#mathutils.Quaternion.to_matrix
    ecef_T_cv_look_at[:3,:3] = rot_quat.to_matrix()
    ecef_T_cv_look_at[:3,3] = ecef_T_cv[:3,3].ravel()

    return ecef_T_cv_look_at

def set_camera_pose(obj_camera, pose):
    # gl2cv = Matrix().to_4x4()
    # gl2cv[1][1] = -1
    # gl2cv[2][2] = -1
    # obj_camera.matrix_world = (gl2cv * Matrix(pose)).inverted()

    # cv_T_gl = np.eye(4)
    # cv_T_gl[1,1] = -1
    # cv_T_gl[1,2] = -1

    # w_T_cv = pose
    # w_T_gl = w_T_cv @ cv_T_gl
    # # obj_camera.matrix_world = w_T_gl.T
    # obj_camera.matrix_world = w_T_gl

    # Column-major?
    obj_camera.matrix_world = pose.T

if __name__ == "__main__":
    output_dir = "/tmp"
    output_file_pattern_string = "blender_render_%04d.png"

    K = get_calibration_matrix_K_from_blender("Camera")
    print(f"Camera Matrix:\n", K)

    data_dict = dict()
    data_dict["K"] = K

    debug_print = False

    npoints = 60
    full_sphere = False
    regular_surf_points = regular_on_sphere_points(npoints, full_sphere)
    print(f"regular_surf_points={len(regular_surf_points)}\n{regular_surf_points}")

    # Transformation from CV frame to NED frame
    ned_T_cv = np.eye(4)
    ned_T_cv[0,0] = 0
    ned_T_cv[0,1] = -1
    ned_T_cv[1,0] = 1
    ned_T_cv[1,1] = 0
    print(f"ned_T_cv:\n{ned_T_cv}")

    radius = 5.0
    camera_name = "Camera"
    object_name = "Suzanne"
    camera = bpy.data.objects[camera_name]
    for keyframe, point in enumerate(regular_surf_points):
        print()
        print(f"{keyframe+1}/{len(regular_surf_points)}")
        lon = point[0]
        lat = point[1]

        in_radian = True
        ecef_T_ned = getNED(lon, lat, radius, in_radian)
        ecef_T_cv = ecef_T_ned @ ned_T_cv
        print(f"ecef_T_cv:\n{ecef_T_cv}")

        ecef_T_cv_look_at = look_at(ecef_T_cv, np.zeros((3,1)))
        print(f"ecef_T_cv_look_at:\n{ecef_T_cv_look_at}")
        set_camera_pose(camera, ecef_T_cv_look_at)

        output_filepath = os.path.join(output_dir, (output_file_pattern_string % keyframe))
        bpy.context.scene.render.filepath = output_filepath
        bpy.ops.render.render(write_still = True)

        img = cv.imread(output_filepath)
        print(f"img: {img.shape}")

        sift_detector = cv.SIFT.create()
        keypoints, descriptors = sift_detector.detectAndCompute(img, None)
        if keypoints is not None and descriptors is not None:
            print(f"keypoints: {len(keypoints)} ; type={type(keypoints)}")
            print(f"descriptors: {len(descriptors)} ; type={type(descriptors)} ; shape={descriptors.shape}")
        if keypoints is None:
            keypoints = []
        if descriptors is None:
            descriptors = []

        c_T_o = get_camera_pose(camera_name, object_name)

        obj_faces_vert, obj_faces_normal = get_object_vertices(object_name)
        print(f"obj_faces_vert={len(obj_faces_vert)}")

        if debug_print:
            for face_id, face in enumerate(obj_faces_vert):
                print(f"\nFace {face_id}")
                print(f"    normal: {obj_faces_normal[face_id]}")
                for vertex_id, vertex in enumerate(face):
                    print(f"    vertex {vertex_id}: {face}")

        pointcloud = []
        image_pts = []
        object_pts = []
        descriptors_pcl = []
        img_results = np.copy(img)
        for idx, kpt in enumerate(keypoints):
            kpt_pt_normalized = compute_ray(K, kpt.pt)
            # print(f"kpt: {kpt_pt_normalized}")

            kpt_pt = np.array(kpt.pt, dtype=np.int32)
            cv.drawMarker(img_results, kpt_pt, (0,0,255),cv.MARKER_CROSS, 6, 1)

            for face_id, face in enumerate(obj_faces_vert):
                is_visible = is_face_visible(kpt_pt_normalized, face[0], obj_faces_normal[face_id])
                if is_visible:
                    intersect_pt = ray_vertex_intersection(kpt_pt_normalized, face, c_T_o)
                    if intersect_pt is not None:
                        if debug_print:
                            print(f"Face {face_id}, is visible={is_visible}, intersect_pt={intersect_pt}")

                        pointcloud.append([intersect_pt[0], intersect_pt[1], intersect_pt[2]])
                        image_pts.append(kpt_pt)
                        c_object_pt = vec_augment([intersect_pt[0], intersect_pt[1], intersect_pt[2]])
                        o_object_pt = inv_transfo(c_T_o) @ c_object_pt
                        object_pts.append(o_object_pt[:3])
                        descriptors_pcl.append(descriptors[idx])

        cv.imwrite("/tmp/blender_render_{:04d}_results.png".format(keyframe), img_results)

        data_dict["image_{:04d}".format(keyframe)] = img
        data_dict["pointcloud_{:04d}".format(keyframe)] = pointcloud
        data_dict["image_pts_{:04d}".format(keyframe)] = image_pts
        data_dict["object_pts_{:04d}".format(keyframe)] = object_pts
        data_dict["descriptors_pcl_{:04d}".format(keyframe)] = descriptors_pcl
        data_dict["descriptors_{:04d}".format(keyframe)] = descriptors
        data_dict["c_T_o_{:04d}".format(keyframe)] = c_T_o

    data_dict["nb_data"] = len(regular_surf_points)
    np.savez("/tmp/blender_render_keypoints_sampling.npz", **data_dict)
