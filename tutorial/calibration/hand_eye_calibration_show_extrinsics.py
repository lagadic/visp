#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2020 by Inria. All rights reserved.
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   * Neither the names of the copyright holders nor the names of the contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# This software is provided by the copyright holders and contributors "as is" and
# any express or implied warranties, including, but not limited to, the implied
# warranties of merchantability and fitness for a particular purpose are disclaimed.
# In no event shall copyright holders or contributors be liable for any direct,
# indirect, incidental, special, exemplary, or consequential damages
# (including, but not limited to, procurement of substitute goods or services;
# loss of use, data, or profits; or business interruption) however caused
# and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of
# the use of this software, even if advised of the possibility of such damage.

# Python 2/3 compatibility
from __future__ import print_function
from __future__ import division

from scipy.spatial.transform import Rotation
import numpy as np
from numpy import linspace
# pip install pyyaml for Python2
# pip3 install pyyaml for Python3
import yaml

def inverse_homogeneoux_matrix(M):
    R = M[0:3, 0:3]
    T = M[0:3, 3]
    M_inv = np.identity(4)
    M_inv[0:3, 0:3] = R.T
    M_inv[0:3, 3] = -(R.T).dot(T)

    return M_inv

def pose_to_homogeneous_matrix(pose):
    R = Rotation.from_rotvec(pose[3:]).as_dcm()
    M = np.identity(4)
    M[0:3,0:3] = R
    M[0:3,3] = pose[:3]
    return M

def transform_to_matplotlib_frame(cMo, X, patternCentric=False):
    M = np.identity(4)
    if patternCentric:
        M[1,1] = -1
        M[2,2] = -1
    else:
        M[1,1] = 0
        M[1,2] = 1
        M[2,1] = -1
        M[2,2] = 0

    if patternCentric:
        return M.dot(inverse_homogeneoux_matrix(cMo).dot(X))
    else:
        return M.dot(cMo.dot(X))

def create_frame(size):
    X_frame = np.ones((4,4))
    X_frame[0:3,0] = [0, 0, 0]
    X_frame[0:3,1] = [size, 0, 0]
    X_frame[0:3,2] = [0, size, 0]
    X_frame[0:3,3] = [0, 0, size]

    return X_frame

def create_camera_model(width, height, focal_px, scale_focal, draw_frame_axis=False):
    f_scale = scale_focal / focal_px

    # draw image plane
    X_img_plane = np.ones((4,5))
    X_img_plane[0:3,0] = [-width, height, f_scale]
    X_img_plane[0:3,1] = [width, height, f_scale]
    X_img_plane[0:3,2] = [width, -height, f_scale]
    X_img_plane[0:3,3] = [-width, -height, f_scale]
    X_img_plane[0:3,4] = [-width, height, f_scale]

    # draw triangle above the image plane
    X_triangle = np.ones((4,3))
    X_triangle[0:3,0] = [-width, -height, f_scale]
    X_triangle[0:3,1] = [0, -2*height, f_scale]
    X_triangle[0:3,2] = [width, -height, f_scale]

    # draw camera
    X_center1 = np.ones((4,2))
    X_center1[0:3,0] = [0, 0, 0]
    X_center1[0:3,1] = [-width, height, f_scale]

    X_center2 = np.ones((4,2))
    X_center2[0:3,0] = [0, 0, 0]
    X_center2[0:3,1] = [width, height, f_scale]

    X_center3 = np.ones((4,2))
    X_center3[0:3,0] = [0, 0, 0]
    X_center3[0:3,1] = [width, -height, f_scale]

    X_center4 = np.ones((4,2))
    X_center4[0:3,0] = [0, 0, 0]
    X_center4[0:3,1] = [-width, -height, f_scale]

    # draw camera frame axis
    X_frame1 = np.ones((4,2))
    X_frame1[0:3,0] = [0, 0, 0]
    X_frame1[0:3,1] = [f_scale/2, 0, 0]

    X_frame2 = np.ones((4,2))
    X_frame2[0:3,0] = [0, 0, 0]
    X_frame2[0:3,1] = [0, f_scale/2, 0]

    X_frame3 = np.ones((4,2))
    X_frame3[0:3,0] = [0, 0, 0]
    X_frame3[0:3,1] = [0, 0, f_scale/2]

    if draw_frame_axis:
        return [X_img_plane, X_triangle, X_center1, X_center2, X_center3, X_center4, X_frame1, X_frame2, X_frame3]
    else:
        return [X_img_plane, X_triangle, X_center1, X_center2, X_center3, X_center4]

def create_board_model(extrinsics, board_width, board_height, square_size, draw_frame_axis=False):
    width = board_width*square_size
    height = board_height*square_size

    # draw calibration board
    X_board = np.ones((4,5))
    X_board[0:3,0] = [0,0,0]
    X_board[0:3,1] = [width,0,0]
    X_board[0:3,2] = [width,height,0]
    X_board[0:3,3] = [0,height,0]
    X_board[0:3,4] = [0,0,0]

    # draw board frame axis
    X_frame1 = np.ones((4,2))
    X_frame1[0:3,0] = [0, 0, 0]
    X_frame1[0:3,1] = [height/2, 0, 0]

    X_frame2 = np.ones((4,2))
    X_frame2[0:3,0] = [0, 0, 0]
    X_frame2[0:3,1] = [0, height/2, 0]

    X_frame3 = np.ones((4,2))
    X_frame3[0:3,0] = [0, 0, 0]
    X_frame3[0:3,1] = [0, 0, height/2]

    if draw_frame_axis:
        return [X_board, X_frame1, X_frame2, X_frame3]
    else:
        return [X_board]

def draw(ax, cam_width, cam_height, focal_px, scale_focal,
         extrinsics, board_width, board_height, square_size,
         eMc, frame_size):
    from matplotlib import cm

    min_values = np.zeros((3,1))
    min_values = np.inf
    max_values = np.zeros((3,1))
    max_values = -np.inf

    X_moving = create_camera_model(cam_width, cam_height, focal_px, scale_focal)
    X_static = create_board_model(extrinsics, board_width, board_height, square_size, True)
    X_frame = create_frame(frame_size)

    cm_subsection = linspace(0.0, 1.0, extrinsics.shape[0])
    colors = [ cm.jet(x) for x in cm_subsection ]

    patternCentric = True

    for i in range(len(X_static)):
        X = np.zeros(X_static[i].shape)
        for j in range(X_static[i].shape[1]):
            X[:,j] = transform_to_matplotlib_frame(np.eye(4), X_static[i][:,j], patternCentric)
        ax.plot3D(X[0,:], X[1,:], X[2,:], color='r')
        min_values = np.minimum(min_values, X[0:3,:].min(1))
        max_values = np.maximum(max_values, X[0:3,:].max(1))

    for idx in range(extrinsics.shape[0]):
        cMo = pose_to_homogeneous_matrix(extrinsics[idx,:])
        for i in range(len(X_moving)):
            X = np.zeros(X_moving[i].shape)
            for j in range(X_moving[i].shape[1]):
                X[0:4,j] = transform_to_matplotlib_frame(cMo, X_moving[i][0:4,j], patternCentric)
            ax.plot3D(X[0,:], X[1,:], X[2,:], color=colors[idx])
            min_values = np.minimum(min_values, X[0:3,:].min(1))
            max_values = np.maximum(max_values, X[0:3,:].max(1))

        eMo = eMc.dot(cMo)
        oX = np.zeros((4,2), dtype=np.float64)
        oY = np.zeros((4,2), dtype=np.float64)
        oZ = np.zeros((4,2), dtype=np.float64)
        ec = np.zeros((4,2), dtype=np.float64)

        oX[:,0] = transform_to_matplotlib_frame(eMo, X_frame[:,0], patternCentric)
        oX[:,1] = transform_to_matplotlib_frame(eMo, X_frame[:,1], patternCentric)
        ax.plot3D(oX[0,:], oX[1,:], oX[2,:], color='r')

        oY[:,0] = transform_to_matplotlib_frame(eMo, X_frame[:,0], patternCentric)
        oY[:,1] = transform_to_matplotlib_frame(eMo, X_frame[:,2], patternCentric)
        ax.plot3D(oY[0,:], oY[1,:], oY[2,:], color='g')

        oZ[:,0] = transform_to_matplotlib_frame(eMo, X_frame[:,0], patternCentric)
        oZ[:,1] = transform_to_matplotlib_frame(eMo, X_frame[:,3], patternCentric)
        ax.plot3D(oZ[0,:], oZ[1,:], oZ[2,:], color='b')

        ec[:,0] = transform_to_matplotlib_frame(eMo, X_frame[:,0], patternCentric)
        ec[:,1] = transform_to_matplotlib_frame(cMo, X_moving[2][0:4,0], patternCentric)
        ax.plot3D(ec[0,:], ec[1,:], ec[2,:], color=colors[idx])

    return min_values, max_values

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Plot hand-eye calibration extrinsics.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--ndata', type=int, required=True,
                        help='Number of camera poses.')
    parser.add_argument('--eMc_yaml', type=str, required=True,
                        help='Path to the estimated eMc yaml file.')
    parser.add_argument('--start_index', type=int, default=1,
                        help='Start index.')
    parser.add_argument('--cPo_file_pattern', type=str, default='pose_cPo_%d.yaml',
                        help='cPo filename pattern for camera poses.')
    parser.add_argument('--cam_width', type=float, default=0.064/2,
                        help='Width/2 of the displayed camera.')
    parser.add_argument('--cam_height', type=float, default=0.048/2,
                        help='Height/2 of the displayed camera.')
    parser.add_argument('--focal_px', type=float, default=600,
                        help='Camera focal length to draw the camera visualization.')
    parser.add_argument('--scale_focal', type=float, default=40,
                        help='Value to scale the focal length.')
    parser.add_argument('--board_width', type=int, default=9,
                        help='Calibration board width.')
    parser.add_argument('--board_height', type=int, default=6,
                        help='Calibration board height.')
    parser.add_argument('--square_size', type=float, default=0.025,
                        help='Calibration board square size.')
    parser.add_argument('--frame_size', type=float, default=0.025,
                        help='End-effector frame size for visualization.')
    args = parser.parse_args()

    def load_yaml_pose(filename):
        with open(filename) as file:
            pose_dict = yaml.load(file, Loader=yaml.FullLoader)
            return np.array(pose_dict['data']).flatten()

    camera_poses = np.empty((args.ndata, 6), dtype=np.float64)
    for idx in range(args.start_index, args.start_index + args.ndata):
        camera_pose_filename = args.cPo_file_pattern % (idx)
        camera_poses[idx-args.start_index,:] = load_yaml_pose(camera_pose_filename)

    print('camera_poses:\n', camera_poses)
    eMc = load_yaml_pose(args.eMc_yaml)
    print('eMc:', eMc.transpose())

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=unused-variable

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect("equal")

    cam_width = args.cam_width
    cam_height = args.cam_height
    scale_focal = args.scale_focal
    min_values, max_values = draw(ax, cam_width, cam_height, args.focal_px,
                                  scale_focal, camera_poses, args.board_width,
                                  args.board_height, args.square_size,
                                  pose_to_homogeneous_matrix(eMc), args.frame_size)

    X_min = min_values[0]
    X_max = max_values[0]
    Y_min = min_values[1]
    Y_max = max_values[1]
    Z_min = min_values[2]
    Z_max = max_values[2]
    max_range = np.array([X_max-X_min, Y_max-Y_min, Z_max-Z_min]).max() / 2.0

    mid_x = (X_max+X_min) * 0.5
    mid_y = (Y_max+Y_min) * 0.5
    mid_z = (Z_max+Z_min) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel('x')
    ax.set_ylabel('-y')
    ax.set_zlabel('-z')
    ax.set_title('Hand-Eye Calibration Visualization')

    plt.show()

if __name__ == '__main__':
    main()
