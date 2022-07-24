#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2022- ViSP contributor
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

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from numpy import linspace
import numpy as np
import argparse
import os

debug_print = False

def visp_thetau_to_rotation(thetau):
    theta = np.linalg.norm(thetau)
    si = np.sin(theta)
    co = np.cos(theta)
    sinc = visp_sinc(si, theta)
    mcosc = visp_mcosc(co, theta)

    R = np.empty((3,3))
    R[0,0] = co + mcosc * thetau[0]**2
    R[0,1] = -sinc * thetau[2] + mcosc * thetau[0] * thetau[1]
    R[0,2] = sinc * thetau[1] + mcosc * thetau[0] * thetau[2]
    R[1,0] = sinc * thetau[2] + mcosc * thetau[1] * thetau[0]
    R[1,1] = co + mcosc * thetau[1]**2
    R[1,2] = -sinc * thetau[0] + mcosc * thetau[1] * thetau[2]
    R[2,0] = -sinc * thetau[1] + mcosc * thetau[2] * thetau[0]
    R[2,1] = sinc * thetau[0] + mcosc * thetau[2] * thetau[1]
    R[2,2] = co + mcosc * thetau[2]**2
    return R

def visp_sinc(sinx, x):
    ang_min_sinc = 1e-8
    if np.abs(x) < ang_min_sinc:
        return 1.0
    else:
        return sinx/x

def visp_mcosc(cosx, x):
    ang_min_mc = 2.5e-4
    if np.abs(x) < ang_min_mc:
        return 0.5
    else:
        return (1-cosx) / x**2

def visp_rotation_to_thetau(R):
    s = (R[1][0]-R[0][1])*(R[1][0]-R[0][1]) \
        + (R[2][0]-R[0][2])*(R[2][0]-R[0][2]) \
        + (R[2][1]-R[1][2])*(R[2][1]-R[1][2])
    s = np.sqrt(s) / 2
    c = (R[0][0]+R[1][1]+R[2][2]-1.0) / 2
    theta = np.arctan2(s, c)

    minimum = 0.0001
    thetau = np.zeros((1,3))
    if 1+c > minimum:
        sinc = visp_sinc(s,theta)

        thetau[0,0] = (R[2][1]-R[1][2])/(2*sinc)
        thetau[0,1] = (R[0][2]-R[2][0])/(2*sinc)
        thetau[0,2] = (R[1][0]-R[0][1])/(2*sinc)
    else:
        if (R[0][0]-c) < np.finfo(float).eps:
            thetau[0,0] = 0
        else:
            thetau[0,0] = theta*(np.sqrt((R[0][0]-c)/(1-c)))
        if R[2][1]-R[1][2] < 0:
            thetau[0,0] = -thetau[0,0];

        if (R[1][1]-c) < np.finfo(float).eps:
            thetau[0,1] = 0
        else:
            thetau[0,1] = theta*(np.sqrt((R[1][1]-c)/(1-c)))

        if (R[0][2]-R[2][0]) < 0:
            thetau[0,1] = -thetau[0,1]

        if (R[2][2]-c) < np.finfo(float).eps:
            thetau[0,2] = 0
        else:
            thetau[0,2] = theta*(np.sqrt((R[2][2]-c)/(1-c)))

        if ((R[1][0]-R[0][1]) < 0):
            thetau[0,2] = -thetau[0,2]
    return thetau

def load_camera_poses(filename, use_thetau=False):
    if use_thetau:
        camera_poses_raw = np.loadtxt(filename)
        camera_poses = np.zeros((4*camera_poses_raw.shape[0], 4))
        for i in range(camera_poses_raw.shape[0]):
            camera_poses[i*4:i*4+3, 0:3] = visp_thetau_to_rotation(camera_poses_raw[i, 3:])
            camera_poses[i*4:i*4+3, 3] = camera_poses_raw[i,0:3].T
            camera_poses[i*4+3, 3] = 1
        return camera_poses
    else:
        return np.loadtxt(filename)

def inverse_homogeneoux_matrix(M):
    R = M[0:3, 0:3]
    T = M[0:3, 3]
    M_inv = np.identity(4)
    M_inv[0:3, 0:3] = R.T
    M_inv[0:3, 3] = -(R.T).dot(T)
    return M_inv

def draw_camera(ax, cam_pose, cam_width, cam_height, cam_focal, scale, color='r'):
    width_scale = scale * cam_width
    height_scale = scale * cam_height
    f_scale = scale * cam_focal

    # Draw camera image plane
    xs_img_plane = [-width_scale, width_scale, width_scale, -width_scale, -width_scale]
    ys_img_plane = [height_scale, height_scale, -height_scale, -height_scale, height_scale]
    zs_img_plane = [f_scale, f_scale, f_scale, f_scale, f_scale]

    for i in range(len(xs_img_plane)):
        vec = np.ones((4,1))
        vec[0] = xs_img_plane[i]
        vec[1] = ys_img_plane[i]
        vec[2] = zs_img_plane[i]

        res = cam_pose.dot(vec)
        xs_img_plane[i] = res[0,0]
        ys_img_plane[i] = res[1,0]
        zs_img_plane[i] = res[2,0]

    ax.plot3D(xs_img_plane, ys_img_plane, zs_img_plane, color=color)

    xs_center1 = [0, -width_scale]
    ys_center1 = [0, height_scale]
    zs_center1 = [0, f_scale]

    for i in range(len(xs_center1)):
        vec = np.ones((4,1))
        vec[0] = xs_center1[i]
        vec[1] = ys_center1[i]
        vec[2] = zs_center1[i]

        res = cam_pose @ vec
        xs_center1[i] = res[0,0]
        ys_center1[i] = res[1,0]
        zs_center1[i] = res[2,0]

    ax.plot3D(xs_center1, ys_center1, zs_center1, color=color)

    xs_center2 = [0, width_scale]
    ys_center2 = [0, height_scale]
    zs_center2 = [0, f_scale]

    for i in range(len(xs_center2)):
        vec = np.ones((4,1))
        vec[0] = xs_center2[i]
        vec[1] = ys_center2[i]
        vec[2] = zs_center2[i]

        res = cam_pose.dot(vec)
        xs_center2[i] = res[0,0]
        ys_center2[i] = res[1,0]
        zs_center2[i] = res[2,0]

    ax.plot3D(xs_center2, ys_center2, zs_center2, color=color)

    xs_center3 = [0, width_scale]
    ys_center3 = [0, -height_scale]
    zs_center3 = [0, f_scale]

    for i in range(len(xs_center3)):
        vec = np.ones((4,1))
        vec[0] = xs_center3[i]
        vec[1] = ys_center3[i]
        vec[2] = zs_center3[i]

        res = cam_pose.dot(vec)
        xs_center3[i] = res[0,0]
        ys_center3[i] = res[1,0]
        zs_center3[i] = res[2,0]

    ax.plot3D(xs_center3, ys_center3, zs_center3, color=color)

    xs_center4 = [0, -width_scale]
    ys_center4 = [0, -height_scale]
    zs_center4 = [0, f_scale]

    for i in range(len(zs_center4)):
        vec = np.ones((4,1))
        vec[0] = xs_center4[i]
        vec[1] = ys_center4[i]
        vec[2] = zs_center4[i]

        res = cam_pose.dot(vec)
        xs_center4[i] = res[0,0]
        ys_center4[i] = res[1,0]
        zs_center4[i] = res[2,0]

    ax.plot3D(xs_center4, ys_center4, zs_center4, color=color)

    # Draw triangle above the camera image plane
    X_triangle = np.ones((4,3))
    X_triangle[0:3,0] = [-width_scale, -height_scale, f_scale]
    X_triangle[0:3,1] = [0, -2*height_scale, f_scale]
    X_triangle[0:3,2] = [width_scale, -height_scale, f_scale]

    for i in range(X_triangle.shape[1]):
        vec = np.ones((4,1))
        vec[0] = X_triangle[0,i]
        vec[1] = X_triangle[1,i]
        vec[2] = X_triangle[2,i]

        res = cam_pose.dot(vec)
        X_triangle[0,i] = res[0,0]
        X_triangle[1,i] = res[1,0]
        X_triangle[2,i] = res[2,0]

    ax.plot3D(X_triangle[0,:], X_triangle[1,:], X_triangle[2,:], color=color)

def draw_camera_path(ax, camera_poses, colors):
    for i in range(0, camera_poses.shape[0]-4, 4):
        camera_pose1 = camera_poses[i:i+4,:]
        camera_pose2 = camera_poses[i+4:i+8,:]
        ax.plot([camera_pose1[0,3], camera_pose2[0,3]],
                [camera_pose1[1,3], camera_pose2[1,3]],
                [camera_pose1[2,3], camera_pose2[2,3]], color=colors[i//4])

def draw_sphere(ax, tx, ty, tz, radius, color='b'):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = radius * np.cos(u)*np.sin(v) + tx
    y = radius * np.sin(u)*np.sin(v) + ty
    z = radius * np.cos(v) + tz
    ax.plot_wireframe(x, y, z, color=color)

def draw_model(ax, model, color='b'):
    # Lines
    if debug_print:
        print(f"model.lines_vec: {len(model.lines_vec)}")
    for line in model.lines_vec:
        ax.plot3D(xs=[line.start_pt[0], line.end_pt[0]], ys=[line.start_pt[1], line.end_pt[1]], zs=[line.start_pt[2], line.end_pt[2]], color=color)

    # Faces
    if debug_print:
        print(f"model.faces_vec: {len(model.faces_vec)}")
    for face in model.faces_vec:
        for idx, pt0 in enumerate(face.pts_vec):
            if idx == len(face.pts_vec)-1:
                pt1 = face.pts_vec[0]
            else:
                pt1 = face.pts_vec[idx+1]
            ax.plot3D(xs=[pt0[0], pt1[0]], ys=[pt0[1], pt1[1]], zs=[pt0[2], pt1[2]], color=color)

    # Cylinders
    if debug_print:
        print(f"model.cylinders_vec: {len(model.cylinders_vec)}")
    for cylinder in model.cylinders_vec:
        ax.plot_surface(cylinder.Xo, cylinder.Yo, cylinder.Zo, alpha=0.5)

    # Spheres
    if debug_print:
        print(f"model.spheres_vec: {len(model.spheres_vec)}")
    for sphere in model.spheres_vec:
        draw_sphere(ax, sphere.center[0], sphere.center[1], sphere.center[2], sphere.radius, color=color)

# https://stackoverflow.com/a/11156353/6055233
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

    # https://github.com/matplotlib/matplotlib/issues/21688#issuecomment-974912574
    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))

        return np.min(zs)

def draw_model_frame(ax, wRo, size=0.05):
    x_axis = wRo @ np.array([size, 0, 0])
    x_arrow = Arrow3D([0, x_axis[0]], [0, x_axis[1]], [0, x_axis[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
    ax.add_artist(x_arrow)

    y_axis = wRo @ np.array([0, size, 0])
    y_arrow = Arrow3D([0, y_axis[0]], [0, y_axis[1]], [0, y_axis[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
    ax.add_artist(y_arrow)

    z_axis = wRo @ np.array([0, 0, size])
    z_arrow = Arrow3D([0, z_axis[0]], [0, z_axis[1]], [0, z_axis[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
    ax.add_artist(z_arrow)

# https://stackoverflow.com/a/63625222
# Functions from @Mateen Ulhaq and @karlo
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

def plot_poses(camera_poses):
    fig, axarr = plt.subplots(3,2, sharex=True)

    axarr[0,0].plot(camera_poses[0::4,3])
    axarr[1,0].plot(camera_poses[1::4,3])
    axarr[2,0].plot(camera_poses[2::4,3])
    axarr[0,0].grid(True)
    axarr[1,0].grid(True)
    axarr[2,0].grid(True)

    thetau_vec = np.zeros((camera_poses.shape[0]//4,3))
    for i in range(0,camera_poses.shape[0],4):
        thetau_vec[i//4] = visp_rotation_to_thetau(camera_poses[i:i+3,:3])

    axarr[0,1].plot(np.degrees(thetau_vec[:,0]))
    axarr[1,1].plot(np.degrees(thetau_vec[:,1]))
    axarr[2,1].plot(np.degrees(thetau_vec[:,2]))
    axarr[0,1].grid(True)
    axarr[1,1].grid(True)
    axarr[2,1].grid(True)

    axarr[2,0].set_xlabel("Frame #")
    axarr[0,0].set_ylabel(r"$ t_x $ (m)")
    axarr[1,0].set_ylabel(r"$ t_y $ (m)")
    axarr[2,0].set_ylabel(r"$ t_z $ (m)")

    axarr[2,1].set_xlabel("Frame #")
    axarr[0,1].set_ylabel(r"$ \theta u_x $ (deg)")
    axarr[1,1].set_ylabel(r"$ \theta u_y $ (deg)")
    axarr[2,1].set_ylabel(r"$ \theta u_z $ (deg)")

    fig.suptitle('Camera poses')

def get_colormap(number_of_lines, colormap):
    start = 0
    stop = 1
    cm_subsection = linspace(start, stop, number_of_lines)
    colors = [ plt.get_cmap(colormap)(x) for x in cm_subsection ]
    return colors

def remove_comment(str):
    idx = str.find("#")
    if idx != -1:
        return str[0:idx]
    else:
        return str

def remove_primitive_name(str):
    idx = str.find("name=")
    if idx != -1:
        return str[0:idx]
    else:
        return str

def transform(X, M):
    X_transf = [
        M[0,0]*X[0] + M[0,1]*X[1] + M[0,2]*X[2] + M[0,3],
        M[1,0]*X[0] + M[1,1]*X[1] + M[1,2]*X[2] + M[1,3],
        M[2,0]*X[0] + M[2,1]*X[1] + M[2,2]*X[2] + M[2,3]
    ]
    return X_transf

class Line:
    def __init__(self, start_pt, end_pt):
        self.start_pt = start_pt
        self.end_pt = end_pt

    def __repr__(self):
        return f"start: {self.start_pt} / end: {self.end_pt}\n"

    def __str__(self):
        return f"start: {self.start_pt} / end: {self.end_pt}\n"

    # TODO: use object points and transformed object points members?
    def transform(self, M):
        self.start_pt = transform(self.start_pt, M)
        self.end_pt = transform(self.end_pt, M)

class Face:
    def __init__(self, pts_vec):
        self.pts_vec = pts_vec

    def __repr__(self):
        return f"pts_vec={len(self.pts_vec)}:\n{self.pts_vec}\n"

    def __str__(self):
        return f"pts_vec={len(self.pts_vec)}:\n{self.pts_vec}\n"

    # TODO: use object points and transformed object points members?
    def transform(self, M):
        for i in range(len(self.pts_vec)):
            self.pts_vec[i] = transform(self.pts_vec[i], M)

class Cylinder:
    def __init__(self, point_1, point_2, radius):
        self.point_1 = np.asarray(point_1)
        self.point_2 = np.asarray(point_2)
        self.radius = radius
        center_x = (point_2[0] - point_1[0]) / 2
        center_y = (point_2[1] - point_1[1]) / 2
        height = np.linalg.norm(self.point_1 - self.point_2)
        self.Xc, self.Yc, self.Zc = data_for_cylinder_along_z(center_x, center_y, radius, height)
        self.Xo = self.Xc
        self.Yo = self.Yc
        self.Zo = self.Zc

    def __repr__(self):
        return f"point_1={self.point_1} ; point_2={self.point_2} ; radius={self.radius}\n"

    def __str__(self):
        return f"point_1={self.point_1} ; point_2={self.point_2} ; radius={self.radius}\n"

    def transform(self, M):
        for i in range(self.Xc.shape[0]):
            for j in range(self.Xc.shape[1]):
                X_transf = transform(np.array([self.Xc[i,j], self.Yc[i,j], self.Zc[i,j]]), M)
                self.Xo[i,j] = X_transf[0]
                self.Yo[i,j] = X_transf[1]
                self.Zo[i,j] = X_transf[2]

class Sphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def __repr__(self):
        return f"center: {self.center} ; radius: {self.radius}\n"

    def __str__(self):
        return f"center: {self.center} ; radius: {self.radius}\n"

    # TODO: use object points and transformed object points members?
    def transform(self, M):
        self.center = transform(self.center, M)

class Model:
    def __init__(self, lines_vec, faces_vec, cylinders_vec, spheres_vec):
        self.lines_vec = lines_vec
        self.faces_vec = faces_vec
        self.cylinders_vec = cylinders_vec
        self.spheres_vec = spheres_vec

    def __repr__(self):
        return f"Lines:\n{self.lines_vec}\nFaces:{self.faces_vec}\nCylinders:{self.cylinders_vec}\nSpheres:{self.spheres_vec}\n"

    def __str__(self):
        return f"Lines:\n{self.lines_vec}\nFaces:{self.faces_vec}\nCylinders:{self.cylinders_vec}\nSpheres:{self.spheres_vec}\n"

    def extend(self, other):
        self.lines_vec.extend(other.lines_vec)
        self.faces_vec.extend(other.faces_vec)
        self.cylinders_vec.extend(other.cylinders_vec)
        self.spheres_vec.extend(other.spheres_vec)

    def transform(self, M):
        for i in range(len(self.lines_vec)):
            self.lines_vec[i].transform(M)

        for i in range(len(self.faces_vec)):
            self.faces_vec[i].transform(M)

        for i in range(len(self.cylinders_vec)):
            self.cylinders_vec[i].transform(M)

        for i in range(len(self.spheres_vec)):
            self.spheres_vec[i].transform(M)

# https://stackoverflow.com/a/49311446/6055233
def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y

    return x_grid,y_grid,z_grid

def parse_cao_model(filename):
    with open(filename) as file:
        state = 0
        pts_vec = []
        lines_vec = []
        line_faces_vec = []
        point_faces_vec = []
        cylinders_vec = []
        circles_vec = []
        max_iter = 10000
        for _ in range(max_iter):
            raw_line = file.readline()
            if "" == raw_line:
                break
            line = remove_comment(raw_line.rstrip('\n').strip())

            if line:
                if state == 0:
                    if any(version_number in line for version_number in ["V0", "V1"]):
                        state = 1
                    else:
                        raise ValueError("CAO model should have at the beginning the version number (either V0 or V1).")
                elif state == 1:
                    nb_pts = int(line)
                    print(f"nb_pts: {nb_pts}")

                    # Parse object points coordinates
                    i = 0
                    while i < nb_pts:
                        raw_line = file.readline()
                        if "" == raw_line:
                            break
                        line = remove_comment(raw_line.rstrip('\n').strip())
                        if line:
                            i += 1
                            pts_vec.append([float(number) for number in line.split()])
                    state = 2
                elif state == 2:
                    nb_lines = int(line)
                    print(f"nb_lines: {nb_lines}")
                    if nb_lines == 0:
                        state = 3

                    # Parse line points indices
                    i = 0
                    while i < nb_lines:
                        raw_line = file.readline()
                        if "" == raw_line:
                            break
                        line = remove_comment(raw_line.rstrip('\n').strip())
                        line = remove_primitive_name(line)
                        if line:
                            i += 1
                            line_pt_idx = [int(number) for number in line.split()]
                            assert len(line_pt_idx) == 2, "len(line_pt_idx) != 2"
                            lines_vec.append(Line(pts_vec[line_pt_idx[0]], pts_vec[line_pt_idx[1]]))
                    state = 3
                elif state == 3:
                    nb_line_faces = int(line)
                    print(f"nb_line_faces: {nb_line_faces}")
                    if nb_line_faces == 0:
                        state = 4

                    # Parse line face number + indices
                    i = 0
                    while i < nb_line_faces:
                        raw_line = file.readline()
                        if "" == raw_line:
                            break
                        line = remove_comment(raw_line.rstrip('\n').strip())
                        line = remove_primitive_name(line)
                        if line:
                            i += 1
                            line_faces_idx = [int(number) for number in line.split()]
                            dbg_msg = f"len(line_faces_idx) == line_faces_idx[0]+1, len(line_faces_idx)={len(line_faces_idx)}, line_faces_idx[0]={line_faces_idx[0]}"
                            assert len(line_faces_idx) == line_faces_idx[0]+1, dbg_msg
                            line_faces_idx = line_faces_idx[1:]
                            face_pts = []
                            face_pts.extend([[lines_vec[idx].start_pt, lines_vec[idx].end_pt] for idx in line_faces_idx])
                            line_faces_vec.append(Face(face_pts))
                    state = 4
                elif state == 4:
                    nb_point_faces = int(line)
                    print(f"nb_point_faces: {nb_point_faces}")
                    if nb_point_faces == 0:
                        state = 5

                    # Parse point face number + indices
                    i = 0
                    while i < nb_point_faces:
                        raw_line = file.readline()
                        if "" == raw_line:
                            break
                        line = remove_comment(raw_line.rstrip('\n').strip())
                        line = remove_primitive_name(line)
                        if line:
                            i += 1
                            point_faces_idx = [int(number) for number in line.split()]
                            dbg_msg = f"len(point_faces_idx) == point_faces_idx[0]+1, len(point_faces_idx)={len(point_faces_idx)}, point_faces_idx[0]={point_faces_idx[0]}"
                            assert len(point_faces_idx) == point_faces_idx[0]+1, dbg_msg
                            point_faces_idx = point_faces_idx[1:]
                            face_pts = []
                            face_pts.extend([pts_vec[idx] for idx in point_faces_idx])
                            point_faces_vec.append(Face(face_pts))
                    state = 5
                elif state == 5:
                    nb_cylinders = int(line)
                    print(f"nb_cylinders: {nb_cylinders}")
                    if nb_cylinders == 0:
                        state = 6

                    # Parse cylinders
                    i = 0
                    while i < nb_cylinders:
                        raw_line = file.readline()
                        if "" == raw_line:
                            break
                        line = remove_comment(raw_line.rstrip('\n').strip())
                        line = remove_primitive_name(line)
                        if line:
                            i += 1
                            data = [number for number in line.split()]
                            radius = float(data[2])
                            point_1 = pts_vec[int(data[0])]
                            point_2 = pts_vec[int(data[1])]
                            cylinders_vec.append(Cylinder(point_1, point_2, radius))
                    state = 6
                elif state == 6:
                    nb_circles = int(line)
                    print(f"nb_circles: {nb_circles}")
                    if nb_circles == 0:
                        state = 7

                    # Parse circles
                    i = 0
                    while i < nb_circles:
                        raw_line = file.readline()
                        if "" == raw_line:
                            break
                        line = remove_comment(raw_line.rstrip('\n').strip())
                        line = remove_primitive_name(line)
                        if line:
                            i += 1
                            data = [number for number in line.split()]
                            radius = float(data[0])
                            center = pts_vec[int(data[1])]
                            circles_vec.append(Sphere(center, radius))
                    state = 7

        if debug_print:
            print(f"pts_vec:\n{pts_vec}")
            print(f"lines_vec:\n{lines_vec}")
            print(f"line_faces_vec:\n{line_faces_vec}")
            print(f"point_faces_vec:\n{point_faces_vec}")
            print(f"cylinders_vec:\n{cylinders_vec}")
            print(f"circles_vec:\n{circles_vec}")

        faces_vec = line_faces_vec
        faces_vec.extend(point_faces_vec)
        return Model(lines_vec, faces_vec, cylinders_vec, circles_vec)

def center_viewport(ax, x_lim, y_lim, z_lim, print_lim=False):
    # https://stackoverflow.com/a/31364297/6055233
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    if np.isnan(x_lim[0]) or np.isnan(x_lim[1]):
        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    else:
        ax.set_xlim3d([x_lim[0], x_lim[1]])

    if np.isnan(y_lim[0]) or np.isnan(y_lim[1]):
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    else:
        ax.set_ylim3d([y_lim[0], y_lim[1]])

    if np.isnan(z_lim[0]) or np.isnan(z_lim[1]):
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    else:
        ax.set_zlim3d([z_lim[0], z_lim[1]])

    if print_lim:
        print(f"X-axis limit: {ax.get_xlim3d()}")
        print(f"Y-axis limit: {ax.get_ylim3d()}")
        print(f"Z-axis limit: {ax.get_zlim3d()}")

def main():
    parser = argparse.ArgumentParser(description='Plot camera trajectory from poses and CAO model files.')
    parser.add_argument('-p', type=str, nargs=1, required=True, help='Path to poses file.')
    parser.add_argument('--theta-u', action='store_true', default=False,
                        help='If true, camera poses are expressed using [tx ty tz tux tuy tuz] formalism, otherwise in homogeneous form.')
    parser.add_argument('-m', type=str, nargs=1, required=True, help='Path to CAO model file.')
    parser.add_argument('--colormap', default='gist_rainbow', type=str, help='Colormap to use for the camera path.')
    parser.add_argument('--save', action='store_true', help='If true, save the figures on disk.')
    parser.add_argument('--save-dir',  default='images', type=str, help='If --save flag is set, the folder where to save the plot images.')
    parser.add_argument('--save-pattern',  default='image_{:06d}.png', type=str, help='Image filename pattern when saving.')
    parser.add_argument('--save-dpi',  default=300, type=int, help='Image dpi when saving.')
    parser.add_argument('--step', type=int, help='Step number between each camera poses when drawing.')
    parser.add_argument('--axes-label-size', type=int, default=20, help='Axes label size.')
    parser.add_argument('--xtick-label-size', type=int, default=14, help='X-tick label size.')
    parser.add_argument('--ytick-label-size', type=int, default=14, help='Y-tick label size.')
    parser.add_argument('--axes-title-size', type=int, default=28, help='Axes title size.')
    parser.add_argument('--figure-title-size', type=int, default=30, help='Figure title size.')
    parser.add_argument('--cam-width', type=float, default=640/2000., help='Camera width size when drawing the camera poses.')
    parser.add_argument('--cam-height', type=float, default=480/2000., help='Camera height size when drawing the camera poses.')
    parser.add_argument('--cam-focal', type=float, default=600/1000., help='Camera focal length when drawing the camera poses.')
    parser.add_argument('--cam-scale', type=float, default=0.2, help='This is used to scale the camera when drawing the camera poses.')
    parser.add_argument('--wRo', type=float, nargs=3, default=[0, 0, 0],
                        help='Rotation from object model frame to Matplotlib frame in theta.u format.\n'
                        'You can use this site for rotation conversion: https://www.andre-gaschler.com/rotationconverter/')
    parser.add_argument('--frame-size', type=float, default=0.05, help='Coordinates frame size for display.')
    parser.add_argument('--plot-cam-poses', action='store_true', help='If true, plot camera poses on a graph.')
    parser.add_argument('--azim', type=float, default=-60, help='3D plot initial view azimuth.')
    parser.add_argument('--elev', type=float, default=30, help='3D plot initial view elevation.')
    parser.add_argument('--num-cam', type=int, default=10, help='Number of camera poses to draw.')
    parser.add_argument('--x-lim', type=float, nargs=2, default=[np.nan, np.nan], help='Manually set the x-limit for the viewport.')
    parser.add_argument('--y-lim', type=float, nargs=2, default=[np.nan, np.nan], help='Manually set the y-limit for the viewport.')
    parser.add_argument('--z-lim', type=float, nargs=2, default=[np.nan, np.nan], help='Manually set the z-limit for the viewport.')
    args = parser.parse_args()

    axes_label_size = args.axes_label_size
    xtick_label_size = args.xtick_label_size
    ytick_label_size = args.ytick_label_size
    axes_title_size = args.axes_title_size
    fig_title_size = args.figure_title_size
    print(f"Figure axes label size: {axes_label_size}")
    print(f"Figure x-tick label size: {xtick_label_size}")
    print(f"Figure y-tick label size: {ytick_label_size}")
    print(f"Figure axes title size: {axes_title_size}")
    print(f"Figure title size: {fig_title_size}")

    params = {
        'axes.labelsize': axes_label_size,
        'xtick.labelsize': xtick_label_size,
        'ytick.labelsize': ytick_label_size,
        'axes.titlesize': axes_title_size,
        'figure.titlesize': fig_title_size
    }
    plt.rcParams.update(params)

    cam_width = args.cam_width
    cam_height = args.cam_height
    cam_focal = args.cam_focal
    cam_scale = args.cam_scale
    print(f"Camera width when drawing the camera poses: {cam_width}")
    print(f"Camera height when drawing the camera poses: {cam_height}")
    print(f"Camera focal length when drawing the camera poses: {cam_focal}")
    print(f"Camera scale when drawing the camera poses: {cam_scale}")

    # Load camera poses
    camera_pose_filename = args.p[0]
    use_thetau = args.theta_u
    print(f"Load camera poses from: {camera_pose_filename} ; Use theta-u? {use_thetau}")
    camera_poses = load_camera_poses(camera_pose_filename, use_thetau)
    print("poses: ", camera_poses.shape)

    colormap = args.colormap
    print(f"Colormap: {colormap}")
    camera_colors = get_colormap(camera_poses.shape[0]//4, colormap)

    # Load model
    model_filename = args.m[0]
    print(f"Load object CAO model from: {model_filename}")
    model = parse_cao_model(model_filename)
    w_M_o = np.eye(4)
    w_M_o[:3,:3] = visp_thetau_to_rotation(np.array(args.wRo))
    print(f"w_M_o:\n{w_M_o}")
    model.transform(w_M_o)

    inverse_camera_poses = np.zeros(camera_poses.shape)
    for i in range(0, camera_poses.shape[0], 4):
        inverse_camera_poses[i:i+4,:] = w_M_o @ inverse_homogeneoux_matrix(camera_poses[i:i+4,:])

    frame_size = args.frame_size
    num_cam = args.num_cam

    x_lim = args.x_lim
    y_lim = args.y_lim
    z_lim = args.z_lim

    pose_step = inverse_camera_poses.shape[0] // (4*num_cam)
    print(f"Draw approximatively {num_cam} cameras, or approximatively every {pose_step} poses") # TODO:

    if args.save:
        output_folder = args.save_dir
        print(f"Save the plot images to {output_folder} folder")
        os.makedirs(output_folder, exist_ok=True)

        for cpt in range(0, inverse_camera_poses.shape[0]-4, 4):
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_aspect("auto")

            inverse_camera_pose1 = inverse_camera_poses[cpt:cpt+4,:]
            inverse_camera_pose2 = inverse_camera_poses[cpt+4:cpt+8,:]
            ax.plot([inverse_camera_pose1[0,3], inverse_camera_pose2[0,3]],
                    [inverse_camera_pose1[1,3], inverse_camera_pose2[1,3]],
                    [inverse_camera_pose1[2,3], inverse_camera_pose2[2,3]], color=camera_colors[cpt//4])

            for i in range(0, cpt, 4*pose_step):
                camera_pose = inverse_camera_poses[i:i+4,:]
                draw_camera(ax, camera_pose, cam_width, cam_height, cam_focal, cam_scale, camera_colors[i//4])

            draw_camera_path(ax, inverse_camera_poses[:cpt+8,:], camera_colors)
            draw_model(ax, model)
            # Draw current camera pose
            draw_camera(ax, inverse_camera_poses[cpt:cpt+4,:], cam_width, cam_height, cam_focal, cam_scale, camera_colors[cpt//4])
            draw_model_frame(ax, w_M_o[:3,:3], frame_size)

            center_viewport(ax, x_lim, y_lim, z_lim)

            ax.set_xlabel('X (m)', labelpad=10)
            ax.set_ylabel('Y (m)', labelpad=10)
            ax.set_zlabel('Z (m)', labelpad=10)

            ax.view_init(args.elev, args.azim)

            # set_axes_equal(ax)
            plt.title('Camera poses', pad=20)
            fig.set_size_inches(fig.get_size_inches()*2)
            plt.draw()

            plt.savefig(os.path.join(output_folder, args.save_pattern.format(cpt//4)), dpi=args.save_dpi)
            plt.clf()
            plt.close()
    else:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_aspect("auto")

        for i in range(0, inverse_camera_poses.shape[0], 4*pose_step):
            camera_pose = inverse_camera_poses[i:i+4,:]
            draw_camera(ax, camera_pose, cam_width, cam_height, cam_focal, cam_scale)

        # Draw the last camera pose
        for i in range(inverse_camera_poses.shape[0]-4, inverse_camera_poses.shape[0], 4):
            camera_pose = inverse_camera_poses[i:i+4,:]
            draw_camera(ax, camera_pose, cam_width, cam_height, cam_focal, cam_scale)

        draw_camera_path(ax, inverse_camera_poses, camera_colors)
        draw_model(ax, model)
        draw_model_frame(ax, w_M_o[:3,:3], frame_size)

        center_viewport(ax, x_lim, y_lim, z_lim, print_lim=True)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        def on_click(event):
            azim, elev = ax.azim, ax.elev
            print(f"azim: {azim} ; elev: {elev}")

        cid = fig.canvas.mpl_connect('button_release_event', on_click)
        ax.view_init(args.elev, args.azim)

        if args.plot_cam_poses:
            plot_poses(camera_poses)

        ax.set_title('Camera poses')
        plt.show()

if __name__ == "__main__":
    main()
