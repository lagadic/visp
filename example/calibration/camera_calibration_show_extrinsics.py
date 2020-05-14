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

from scipy.spatial.transform import Rotation
import numpy as np
from numpy import linspace
from enum import Enum
import xml.etree.ElementTree as ET
import re

class PatternType(Enum):
    CHESSBOARD = 'Chessboard'
    CIRCLES_GRID = 'Circles grid'

class PoseVector:
    def __init__(self, str):
        nums = [float(n) for n in str.split()]
        self.pose = np.array(nums)

    def __str__(self):
        return "Pose: {}".format(self.pose.transpose())

class CameraInfo:
    def __init__(self, name, image_width, image_height, px, py, u0, v0, \
                 pattern_type, board_width, board_height, square_size, cMo_vec):
        self.name = name
        self.image_width = image_width
        self.image_height = image_height
        self.intrinsics = np.array([[px, 0, u0], [0, py, v0], [0, 0, 1]], np.float64)
        self.pattern_type = pattern_type
        self.board_width = board_width
        self.board_height = board_height
        self.square_size = square_size
        self.cMo_vec = cMo_vec

    def __str__(self):
        str = "CameraInfo:\n\t camera name:{}\n\timage: {:d}x{:d}\n\tcamera intrinsics:\n{}\n\t{}\n\tboard_size: {:d}x{:d}\n\tsquare_size: {:g}\n\tposes:\n".format( \
                self.name, self.image_width, self.image_height, self.intrinsics, \
                self.pattern_type, self.board_width, self.board_height, self.square_size)
        for cMo in self.cMo_vec:
            str += "\t\t" + cMo.__str__() + "\n"
        return str

def inverse_homogeneoux_matrix(M):
    R = M[0:3, 0:3]
    T = M[0:3, 3]
    M_inv = np.identity(4)
    M_inv[0:3, 0:3] = R.T
    M_inv[0:3, 3] = -(R.T).dot(T)

    return M_inv

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

def create_camera_model(camera_matrix, width, height, scale_focal, draw_frame_axis=False):
    fx = camera_matrix[0,0]
    fy = camera_matrix[1,1]
    focal = 2 / (fx + fy)
    f_scale = scale_focal * focal

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

def draw_camera_boards(ax, camera_matrix, cam_width, cam_height, scale_focal,
                       extrinsics, board_width, board_height, square_size,
                       patternCentric):
    from matplotlib import cm

    min_values = np.zeros((3,1))
    min_values = np.inf
    max_values = np.zeros((3,1))
    max_values = -np.inf

    if patternCentric:
        X_moving = create_camera_model(camera_matrix, cam_width, cam_height, scale_focal)
        X_static = create_board_model(extrinsics, board_width, board_height, square_size, True)
    else:
        X_static = create_camera_model(camera_matrix, cam_width, cam_height, scale_focal, True)
        X_moving = create_board_model(extrinsics, board_width, board_height, square_size, True)

    cm_subsection = linspace(0.0, 1.0, extrinsics.shape[0])
    colors = [ cm.jet(x) for x in cm_subsection ]

    for i in range(len(X_static)):
        X = np.zeros(X_static[i].shape)
        for j in range(X_static[i].shape[1]):
            X[:,j] = transform_to_matplotlib_frame(np.eye(4), X_static[i][:,j], patternCentric)
        ax.plot3D(X[0,:], X[1,:], X[2,:], color='r')
        min_values = np.minimum(min_values, X[0:3,:].min(1))
        max_values = np.maximum(max_values, X[0:3,:].max(1))

    for idx in range(extrinsics.shape[0]):
        R = Rotation.from_rotvec(extrinsics[idx,0:3]).as_dcm()
        cMo = np.eye(4,4)
        cMo[0:3,0:3] = R
        cMo[0:3,3] = extrinsics[idx,3:6]
        for i in range(len(X_moving)):
            X = np.zeros(X_moving[i].shape)
            for j in range(X_moving[i].shape[1]):
                X[0:4,j] = transform_to_matplotlib_frame(cMo, X_moving[i][0:4,j], patternCentric)
            ax.plot3D(X[0,:], X[1,:], X[2,:], color=colors[idx])
            min_values = np.minimum(min_values, X[0:3,:].min(1))
            max_values = np.maximum(max_values, X[0:3,:].max(1))

    return min_values, max_values

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Plot camera calibration extrinsics.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--calibration', type=str, default='camera.xml',
                        help='XML camera calibration file.')
    parser.add_argument('--cam_width', type=float, default=0.064/2,
                        help='Width/2 of the displayed camera.')
    parser.add_argument('--cam_height', type=float, default=0.048/2,
                        help='Height/2 of the displayed camera.')
    parser.add_argument('--scale_focal', type=float, default=40,
                        help='Value to scale the focal length.')
    parser.add_argument('--cameraCentric', action='store_true',
                        help='If argument is present, the camera is static and the calibration boards are moving.')
    args = parser.parse_args()

    tree = ET.parse(args.calibration)
    root = tree.getroot()

    def loadCameraInfo(root):
        camera_info_vec = []
        for camera in root.iter('camera'):
            pattern_type = PatternType.CHESSBOARD
            if camera.find('additional_information').find('calibration_pattern_type').text == 'Circles grid':
                pattern_type = PatternType.CIRCLES_GRID

            board_size = re.search(r"(\d+)x(\d+)", camera.find('additional_information').find('board_size').text)

            cMo_vec = []
            for cMo in camera.find('additional_information').find('camera_poses').iter('cMo'):
                cMo_vec.append(PoseVector(cMo.text))

            camera_info_vec.append(CameraInfo( \
                camera.find('name').text, int(camera.find('image_width').text), int(camera.find('image_height').text), \
                float(camera.find('model').find('px').text), float(camera.find('model').find('py').text), \
                float(camera.find('model').find('u0').text), float(camera.find('model').find('v0').text), \
                pattern_type, int(board_size.group(1)), int(board_size.group(2)), \
                float(camera.find('additional_information').find('square_size').text), \
                cMo_vec \
            ))

        return camera_info_vec

    camera_info_vec = loadCameraInfo(root)

    for camera_info in camera_info_vec:
        print('\n', camera_info)

        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=unused-variable

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_aspect("equal")

        cMo_vec = camera_info.cMo_vec
        extrinsics = np.empty((len(cMo_vec), 6), np.float64)
        for idx, cMo in enumerate(cMo_vec):
            extrinsics[idx, :3] = cMo.pose[3:]
            extrinsics[idx, 3:] = cMo.pose[:3]

        cam_width = args.cam_width
        cam_height = args.cam_height
        scale_focal = args.scale_focal
        pattern_centric = not args.cameraCentric
        min_values, max_values = draw_camera_boards(ax, camera_info.intrinsics, cam_width, cam_height,
                                                    scale_focal, extrinsics, camera_info.board_width,
                                                    camera_info.board_height, camera_info.square_size,
                                                    pattern_centric)

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

        if pattern_centric:
            ax.set_xlabel('x')
            ax.set_ylabel('-y')
            ax.set_zlabel('-z')
            ax.set_title('Camera Poses Visualization')
        else:
            ax.set_xlabel('x')
            ax.set_ylabel('z')
            ax.set_zlabel('-y')
            ax.set_title('Calibration Board Poses Visualization')

        plt.show()

if __name__ == '__main__':
    main()
