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
import argparse
import cv2 as cv
import numpy as np
import math
import random

def vec_augment(pt_3d):
    return np.array([pt_3d[0], pt_3d[1], pt_3d[2], 1])

def project(pt_3d, K, c_T_o):
    pt_4d = vec_augment(pt_3d)
    perspective_proj = np.zeros((3,4))
    perspective_proj[0:3,0:3] = np.eye(3)

    pt_2d = K @ perspective_proj @ c_T_o @ pt_4d
    pt_2d[0] /= pt_2d[2]
    pt_2d[1] /= pt_2d[2]

    return pt_2d[:2]

def draw_keypoints(img, pointcloud, K, c_T_o):
    for idx in range(pointcloud.shape[0]):
        pt_3d = pointcloud[idx,]
        # print(f"pt_3d={pt_3d}")
        # pt_2d = project(pt_3d, K, np.eye(4))
        pt_2d = project(pt_3d, K, c_T_o)
        # print(f"pt_2d={pt_2d}")
        pos = pt_2d.astype(np.int32)
        # print(f"pos={pos}")
        cv.drawMarker(img, pos, (0,0,255), cv.MARKER_CROSS, 6, 1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Keypoints matching.')
    parser.add_argument("--input", type=str, default="/tmp/blender_render.png", help="Input image")
    parser.add_argument("--npz", type=str, default="/tmp/blender_render_keypoints_sampling.npz", help="NPZ filepath")
    parser.add_argument("--gt", type=str, default="/tmp/c_T_o.txt", help="Ground truth pose")
    parser.add_argument("--display-ransac", action="store_true", help="Display matching after RANSAC")
    parser.add_argument("--random-colors", action="store_true", help="Random color")
    parser.add_argument("--flann", action="store_true", help="Use FLANN-based matching, other BF with cross-check")
    parser.add_argument('--ransac-iter', type=int, default=1000, help='RANSAC iterations count.')
    parser.add_argument('--ransac-reproj-error', type=float, default=6.0, help='RANSAC reprojection error.')
    args = parser.parse_args()

    # print(cv.getBuildInformation())

    input_img = args.input
    print(f"Input image filepath: {input_img}")
    img = cv.imread(input_img)
    print(f"img: {img.shape}")

    input_npz = args.npz
    print(f"Input NPZ: {input_npz}")

    npz_data = np.load(input_npz)
    nb_data = npz_data["nb_data"]
    print(f"nb_data: {nb_data}")

    input_gt = args.gt
    c_T_o_gt = np.loadtxt(input_gt)
    print(f"c_T_o_gt:\n{c_T_o_gt}")

    display_after_ransac = args.display_ransac
    print(f"Display keypoints matching after RANSAC? {display_after_ransac}")

    random_colors = args.random_colors
    print(f"random_colors={random_colors}")

    flann_matching = args.flann
    print(f"Use FLANN-based matching? {flann_matching}")

    ransac_iter = args.ransac_iter
    print(f"RANSAC iterations: {ransac_iter}")
    ransac_reproj_error = args.ransac_reproj_error
    print(f"RANSAC reprojection error: {ransac_reproj_error}")

    cam_K = npz_data["K"]
    print(f"cam_K:\n{cam_K}")

    nb_data = npz_data["nb_data"]
    train_descriptors = npz_data["descriptors_pcl_{:04d}".format(0)]
    train_object_pts = npz_data["object_pts_{:04d}".format(0)]
    train_image_pts = npz_data["image_pts_{:04d}".format(0)]

    keypoints_img_dict = dict()
    for idx in range(train_image_pts.shape[0]):
        keypoints_img_dict[idx] = 0

    idx_start = train_image_pts.shape[0]
    for idx1 in range(1, nb_data):
        train_descriptors = np.concatenate((train_descriptors, npz_data["descriptors_pcl_{:04d}".format(idx1)]))
        train_object_pts = np.concatenate((train_object_pts, npz_data["object_pts_{:04d}".format(idx1)]))
        train_image_pts_cur = npz_data["image_pts_{:04d}".format(idx1)]
        train_image_pts = np.concatenate((train_image_pts, train_image_pts_cur))

        for _ in range(train_image_pts_cur.shape[0]):
            keypoints_img_dict[idx_start] = idx1
            idx_start += 1

    print(f"train_descriptors: {train_descriptors.shape}")
    print(f"train_object_pts: {train_object_pts.shape}")
    print(f"train_image_pts: {train_image_pts.shape}")
    print(f"keypoints_img_dict: {len(keypoints_img_dict)}")


    sift_detector = cv.SIFT.create()
    keypoints, descriptors = sift_detector.detectAndCompute(img, None)
    print(f"Current keypoints={len(keypoints)}")
    print(f"Current descriptors={descriptors.shape}")

    crossCheck = True
    if flann_matching:
      # https://docs.opencv.org/4.x/dc/de2/classcv_1_1FlannBasedMatcher.html
      matcher = cv.FlannBasedMatcher.create()
    else:
      # https://docs.opencv.org/4.x/d3/da1/classcv_1_1BFMatcher.html
      matcher = cv.BFMatcher.create(crossCheck=crossCheck)

    # https://docs.opencv.org/4.x/db/d39/classcv_1_1DescriptorMatcher.html#a695c0aceafc907c024c24a0b5cdff758
    matches = matcher.match(descriptors, train_descriptors)
    print(f"matches: {len(matches)}")

    pts_3d = []
    pts_2d = []
    pts_2d_train = []
    img_idx = []
    for match in matches:
        pts_3d.append(train_object_pts[match.trainIdx])
        pts_2d.append(keypoints[match.queryIdx].pt)
        pts_2d_train.append(train_image_pts[match.trainIdx])
        img_idx.append(keypoints_img_dict[match.trainIdx])

    pts_3d_np = np.array(pts_3d, dtype=np.float64)
    pts_2d_np = np.array(pts_2d, dtype=np.float64)
    print(f"pts_3d_np={pts_3d_np.shape}")
    print(f"pts_2d_np={pts_2d_np.shape}")

    rvec = None
    tvec = None
    # https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e
    retval, rvec, tvec, inliers = cv.solvePnPRansac(pts_3d_np, pts_2d_np, cam_K, None, iterationsCount=ransac_iter, reprojectionError=ransac_reproj_error)
    print(f"retval={retval}")
    if retval == 0:
        print(f"Running solvePnPRansac() returns 0")
        exit()
    rot_3x3, _ = cv.Rodrigues(rvec)
    print(f"retval={retval} ; inliers={len(inliers)}")
    c_T_o_est = np.eye(4)
    c_T_o_est[:3,:3] = rot_3x3
    c_T_o_est[:3,3] = tvec.ravel()

    rvec_gt, _ = cv.Rodrigues(c_T_o_gt[:3,:3])
    tvec_gt = c_T_o_gt[:3,3]

    for pt_2d in pts_2d:
        cv.drawMarker(img, np.array(pt_2d).astype(np.int32), (0,0,255), cv.MARKER_CROSS, 6, 1)

    frame_length = 0.1
    cv.drawFrameAxes(img, cam_K, None, rvec, tvec, frame_length, 2)
    # cv.drawFrameAxes(img, cam_K, None, rvec_gt, tvec_gt, frame_length, 1)

    model = []
    model.append([ [0.367188, -0.53125, -0.890625], [0.351562, -0.570312, -0.695312] ])
    model.append([ [0.351562, -0.570312, -0.695312], [0.3125, -0.570312, -0.4375] ])
    model.append([ [0.3125, -0.570312, -0.4375], [0.257812, -0.554688, -0.3125] ])
    model.append([ [0.257812, -0.554688, -0.3125], [0.21875, -0.429688, -0.28125] ])
    model.append([ [0.21875, -0.429688, -0.28125], [0.40625, -0.148438, -0.171875] ])
    model.append([ [0.40625, -0.148438, -0.171875], [0.59375, 0.164062, -0.125] ])
    model.append([ [0.59375, 0.164062, -0.125], [0.789062, 0.328125, -0.125] ])
    model.append([ [0.789062, 0.328125, -0.125], [1.03906, 0.492188, -0.085938] ])
    model.append([ [1.03906, 0.492188, -0.085938], [1.3125, 0.53125, 0.054688] ])
    model.append([ [1.3125, 0.53125, 0.054688], [1.36719, 0.5, 0.296875] ])
    model.append([ [1.36719, 0.5, 0.296875], [1.25, 0.546875, 0.46875] ])
    model.append([ [1.25, 0.546875, 0.46875], [0.859375, 0.382812, 0.382812] ])
    model.append([ [0.859375, 0.382812, 0.382812], [0.796875, 0.359375, 0.539062] ])
    model.append([ [0.796875, 0.359375, 0.539062], [0.640625, 0.195312, 0.75] ])
    model.append([ [0.640625, 0.195312, 0.75], [0.453125, 0.070312, 0.929688] ])
    model.append([ [0.453125, 0.070312, 0.929688], [0, 0.078125, 0.984375] ])
    model.append([ [0, 0.078125, 0.984375], [-0.453125, 0.070312, 0.929688] ])
    model.append([ [-0.453125, 0.070312, 0.929688], [-0.640625, 0.195312, 0.75] ])
    model.append([ [-0.640625, 0.195312, 0.75], [-0.796875, 0.359375, 0.539062] ])
    model.append([ [-0.796875, 0.359375, 0.539062], [-1.02344, 0.3125, 0.476562] ])
    model.append([ [-1.02344, 0.3125, 0.476562], [-1.23438, 0.421875, 0.507812] ])
    model.append([ [-1.23438, 0.421875, 0.507812], [-1.35156, 0.421875, 0.320312] ])
    model.append([ [-1.35156, 0.421875, 0.320312], [-1.28125, 0.429688, 0.054688] ])
    model.append([ [-1.28125, 0.429688, 0.054688], [-1.03906, 0.328125, -0.101562] ])
    model.append([ [-1.03906, 0.328125, -0.101562], [-0.773438, 0.125, -0.140625] ])
    model.append([ [-0.773438, 0.125, -0.140625], [-0.59375, 0.164062, -0.125] ])
    model.append([ [-0.59375, 0.164062, -0.125], [-0.40625, -0.148438, -0.171875] ])
    model.append([ [-0.40625, -0.148438, -0.171875], [-0.234375, -0.40625, -0.351562] ])
    model.append([ [-0.234375, -0.40625, -0.351562], [-0.25, -0.390625, -0.5] ])
    model.append([ [-0.25, -0.390625, -0.5], [-0.289062, -0.382812, -0.710938] ])
    model.append([ [-0.289062, -0.382812, -0.710938], [-0.328125, -0.398438, -0.914062] ])
    model.append([ [-0.328125, -0.398438, -0.914062], [-0.164062, -0.4375, -0.945312] ])
    model.append([ [-0.164062, -0.4375, -0.945312], [0, -0.460938, -0.976562] ])

    # Left eye
    model.append([ [-0.375, -0.742188, 0.0625], [-0.226562, -0.78125, 0.109375] ])
    model.append([ [-0.226562, -0.78125, 0.109375], [-0.1875, -0.773438, 0.15625] ])
    model.append([ [-0.1875, -0.773438, 0.15625], [-0.171875, -0.78125, 0.21875] ])
    model.append([ [-0.171875, -0.78125, 0.21875], [-0.179688, -0.78125, 0.296875] ])
    model.append([ [-0.179688, -0.78125, 0.296875], [-0.210938, -0.78125, 0.375] ])
    model.append([ [-0.210938, -0.78125, 0.375], [-0.335938, -0.75, 0.40625] ])
    model.append([ [-0.335938, -0.75, 0.40625], [-0.414062, -0.75, 0.390625] ])
    model.append([ [-0.414062, -0.75, 0.390625], [-0.53125, -0.679688, 0.335938] ])
    model.append([ [-0.53125, -0.679688, 0.335938], [-0.554688, -0.671875, 0.28125] ])
    model.append([ [-0.554688, -0.671875, 0.28125], [-0.476562, -0.71875, 0.101562] ])
    model.append([ [-0.476562, -0.71875, 0.101562], [-0.375, -0.742188, 0.0625] ])

    # Right eye
    model.append([ [0.375, -0.742188, 0.0625], [0.476562, -0.71875, 0.101562] ])
    model.append([ [0.476562, -0.71875, 0.101562], [0.578125, -0.679688, 0.195312] ])
    model.append([ [0.578125, -0.679688, 0.195312], [0.585938, -0.6875, 0.289062] ])
    model.append([ [0.585938, -0.6875, 0.289062], [0.5625, -0.695312, 0.351562] ])
    model.append([ [0.5625, -0.695312, 0.351562], [0.421875, -0.773438, 0.398438] ])
    model.append([ [0.421875, -0.773438, 0.398438], [0.335938, -0.75, 0.40625] ])
    model.append([ [0.335938, -0.75, 0.40625], [0.28125, -0.765625, 0.398438] ])
    model.append([ [0.28125, -0.765625, 0.398438], [0.210938, -0.78125, 0.375] ])
    model.append([ [0.210938, -0.78125, 0.375], [0.179688, -0.78125, 0.296875] ])
    model.append([ [0.179688, -0.78125, 0.296875], [0.171875, -0.78125, 0.21875] ])
    model.append([ [0.171875, -0.78125, 0.21875], [0.1875, -0.773438, 0.15625] ])
    model.append([ [0.1875, -0.773438, 0.15625], [0.226562, -0.78125, 0.109375] ])
    model.append([ [0.226562, -0.78125, 0.109375], [0.375, -0.742188, 0.0625] ])

    for line_model in model:
        start_line = vec_augment(line_model[0])
        end_line = vec_augment(line_model[1])

        start_line_px = project(start_line, cam_K, c_T_o_est).astype(np.int32)
        end_line_px = project(end_line, cam_K, c_T_o_est).astype(np.int32)

        cv.line(img, start_line_px, end_line_px, (0,0,255), 2)

    cv.imwrite("/tmp/img_result.png", img)


    # Mosaic
    nb_data_all = nb_data + 1
    center_idx = nb_data_all // 2
    mosaic_root = math.ceil(math.sqrt(nb_data_all))
    print(f"center_idx={center_idx} ; mosaic_root={mosaic_root}")

    img0 = npz_data["image_{:04d}".format(0)]
    mosaic_img = np.zeros((mosaic_root*img0.shape[0], mosaic_root*img0.shape[1], 3), dtype=np.uint8)
    print(f"mosaic_img={mosaic_img.shape}")
    idx_shift = 0
    for idx1 in range(mosaic_root):
        for idx2 in range(mosaic_root):
            idx = idx1*mosaic_root + idx2 + idx_shift

            if idx == center_idx:
                mosaic_img[idx1*img.shape[0]:(idx1+1)*img.shape[0], idx2*img.shape[1]:(idx2+1)*img.shape[1],] = img
                idx_shift = 1
                idx = idx1*mosaic_root + idx2 + idx_shift

                img_load = npz_data["image_{:04d}".format(idx)]
                idx1_ = idx // mosaic_root
                idx2_ = idx % mosaic_root
                mosaic_img[idx1_*img.shape[0]:(idx1_+1)*img.shape[0], idx2_*img.shape[1]:(idx2_+1)*img.shape[1],] = img_load

            elif idx < nb_data_all:
                idx1_ = idx // mosaic_root
                idx2_ = idx % mosaic_root
                img_load = npz_data["image_{:04d}".format(idx-idx_shift)]
                mosaic_img[idx1_*img.shape[0]:(idx1_+1)*img.shape[0], idx2_*img.shape[1]:(idx2_+1)*img.shape[1],] = img_load

    if display_after_ransac:
        for idx_inlier in range(0, len(inliers), 1):
            idx = inliers[idx_inlier,0]
            idx_img = img_idx[idx]
            if idx_img >= center_idx:
                idx_img += 1

            shift_w = idx_img % mosaic_root
            shift_h = idx_img // mosaic_root

            pt_2d_1 = np.array(pts_2d_train[idx]).astype(np.int32)
            pt_2d_1[0] += img0.shape[1]*shift_w
            pt_2d_1[1] += img0.shape[0]*shift_h
            cv.drawMarker(mosaic_img, pt_2d_1.astype(np.int32), (255,0,0), cv.MARKER_CROSS, 6, 1)

            pt_2d_2 = np.array(pts_2d[idx]).astype(np.int32)
            pt_2d_2[0] += img0.shape[1] * (center_idx % mosaic_root)
            pt_2d_2[1] += img0.shape[0] * (center_idx // mosaic_root)

            color = (0,255,0)
            if random_colors:
                color = (random.randrange(256), random.randrange(256), random.randrange(256))
            cv.line(mosaic_img, pt_2d_1, pt_2d_2, color, 1)
    else:
        for idx in range(0, len(pts_2d_train), 1):
            idx_img = img_idx[idx]
            if idx_img >= center_idx:
                idx_img += 1

            shift_w = idx_img % mosaic_root
            shift_h = idx_img // mosaic_root

            pt_2d_1 = np.array(pts_2d_train[idx]).astype(np.int32)
            pt_2d_1[0] += img0.shape[1]*shift_w
            pt_2d_1[1] += img0.shape[0]*shift_h
            cv.drawMarker(mosaic_img, pt_2d_1.astype(np.int32), (255,0,0), cv.MARKER_CROSS, 6, 1)

            pt_2d_2 = np.array(pts_2d[idx]).astype(np.int32)
            pt_2d_2[0] += img0.shape[1] * (center_idx % mosaic_root)
            pt_2d_2[1] += img0.shape[0] * (center_idx // mosaic_root)
            cv.line(mosaic_img, pt_2d_1, pt_2d_2, (0,255,0), 1)

    cv.imwrite("/tmp/img_mosaic.png", mosaic_img)

    print()
    print(f"rvec={rvec.T}")
    print(f"rvec_gt={rvec_gt.T}")

    print()
    print(f"rot_3x3=\n{rot_3x3}")
    print(f"rot_3x3_gt=\n{c_T_o_gt[:3,:3]}")

    print()
    print(f"tvec={tvec.T}")
    print(f"tvec_gt={tvec_gt}")

    print()
    print(f"c_T_o_est:\n{c_T_o_est}")
    print(f"c_T_o_gt:\n{c_T_o_gt}")
