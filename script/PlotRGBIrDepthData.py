from __future__ import print_function

import argparse
import glob
from pathlib import Path
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from PIL import Image

print(f"matplotlib version: {matplotlib.__version__}")

abort = False
def close_figure(event):
    plt.close(event.canvas.figure)
    if event.key == 'escape':
        global abort
        abort = True

def load_data(iter, input_color_filenames, input_infrared_filenames, input_depth_filenames, input_pcl_filenames):
    color_filename = input_color_filenames[iter]
    ir_filename = input_infrared_filenames[iter]
    depth_filename = input_depth_filenames[iter]
    pcl_filename = input_pcl_filenames[iter]

    if not Path(color_filename).is_file():
        color = np.zeros((480,640,3), dtype=np.uint8)
    else:
        color = Image.open(color_filename)

    if not Path(ir_filename).is_file():
        infrared = np.zeros((480,640,1), dtype=np.uint8)
    else:
        infrared = Image.open(ir_filename)

    if not Path(depth_filename).is_file():
        depth = np.zeros((480,640,1), dtype=np.uint8)
    else:
        with np.load(depth_filename) as data:
            height = data['height'][0]
            width = data['width'][0]
            depth_data_raw = data['data'].reshape((height, width))

            max_uint16 = 65536
            hist, bin_edges = np.histogram(depth_data_raw, bins=max_uint16, range=(0, max_uint16-1))
            # https://stackoverflow.com/a/30460089
            dx = bin_edges[1] - bin_edges[0]
            cumsum = np.cumsum(hist)*dx
            depth = np.zeros(depth_data_raw.shape, dtype=np.uint8)
            for i in range(depth_data_raw.shape[0]):
                for j in range(depth_data_raw.shape[1]):
                    try:
                        depth_value = depth_data_raw[i,j]
                        if depth_value > 0:
                            depth[i,j] = np.uint8(cumsum[depth_value] * 255.0 / cumsum[-1])
                    except:
                        raise

    if not Path(pcl_filename).is_file():
        pointcloud = np.zeros((480,640,3), dtype=np.float)
    else:
        with np.load(pcl_filename) as data:
            height = data['height'][0]
            width = data['width'][0]
            channel = data['channel'][0]
            pointcloud = data['data']
            assert(len(pointcloud.shape) == 3)
            assert(pointcloud.shape[0] == height)
            assert(pointcloud.shape[1] == width)
            assert(pointcloud.shape[2] == channel)

    return color, infrared, depth, pointcloud

def main():
    parser = argparse.ArgumentParser(description='Plot color/IR/depth (in npz fileformat) data stored with saveRealSenseData.cpp file.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--input', type=str, default="",
                        help='Input data folder.')
    parser.add_argument('--color-pattern', type=str, default="color_image_*.png",
                        help='Filename pattern for the color images.')
    parser.add_argument('--infrared-pattern', type=str, default="infrared_image_*.png",
                        help='Filename pattern for the infrared images.')
    parser.add_argument('--depth-pattern', type=str, default="depth_image_*.npz",
                        help='Filename pattern for the depth images.')
    parser.add_argument('--pcl-pattern', type=str, default="point_cloud_*.npz",
                        help='Filename pattern for the pointcloud data.')
    parser.add_argument('--pcl-subsample', type=int, default=10,
                        help='Pointcloud subsampling.')
    args = parser.parse_args()

    input_folder = Path(args.input)
    color_pattern = Path(input_folder) / args.color_pattern
    infrared_pattern = Path(input_folder) / args.infrared_pattern
    depth_pattern = Path(input_folder) / args.depth_pattern
    pcl_pattern = Path(input_folder) / args.pcl_pattern
    pcl_subsample = args.pcl_subsample
    print(f"Input folder: {input_folder}")
    print(f"Color filename pattern: {color_pattern}")
    print(f"Infrared filename pattern: {infrared_pattern}")
    print(f"Depth filename pattern: {depth_pattern}")
    print(f"Pointcloud filename pattern: {pcl_pattern}")
    print(f"Pointcloud subsampling: {pcl_subsample}")

    if not input_folder.is_dir():
        print(f"Invalid folder: {input_folder}")
        return

    input_color_filenames = sorted(glob.glob(str(color_pattern)))
    input_infrared_filenames = sorted(glob.glob(str(infrared_pattern)))
    input_depth_filenames = sorted(glob.glob(str(depth_pattern)))
    input_pcl_filenames = sorted(glob.glob(str(pcl_pattern)))

    max_data = max(len(input_color_filenames), len(input_infrared_filenames), len(input_depth_filenames), len(input_pcl_filenames))
    if max_data == 0:
        raise ValueError('Input data are empty.')

    max_value = 1e9
    nb_color = max_value if len(input_color_filenames) == 0 else len(input_color_filenames)
    nb_infrared = max_value if len(input_infrared_filenames) == 0 else len(input_infrared_filenames)
    nb_depth = max_value if len(input_depth_filenames) == 0 else len(input_depth_filenames)
    nb_pcl = max_value if len(input_pcl_filenames) == 0 else len(input_pcl_filenames)
    max_data = min(nb_color, nb_infrared, nb_depth, nb_pcl)

    # Truncate data if needed
    print(f"Nb color: {len(input_color_filenames)} ; IR: {len(input_infrared_filenames)} ;\
          depth: {len(input_depth_filenames)} ; PCL: {len(input_pcl_filenames)} ; Truncate data to: {max_data}")
    input_color_filenames = [""] * max_data if len(input_color_filenames) == 0 else input_color_filenames[:max_data]
    input_infrared_filenames = [""] * max_data if len(input_infrared_filenames) == 0 else input_infrared_filenames[:max_data]
    input_depth_filenames = [""] * max_data if len(input_depth_filenames) == 0 else input_depth_filenames[:max_data]
    input_pcl_filenames = [""] * max_data if len(input_pcl_filenames) == 0 else input_pcl_filenames[:max_data]

    color_vec = []
    ir_vec = []
    depth_vec = []
    pcl_vec = []

    color_data, ir_data, depth_data, pcl_data = load_data(0, input_color_filenames, input_infrared_filenames, input_depth_filenames, input_pcl_filenames)
    color_vec.append(color_data)
    ir_vec.append(ir_data)
    depth_vec.append(depth_data)
    pcl_vec.append(pcl_data)

    # https://matplotlib.org/stable/gallery/mplot3d/mixed_subplots.html
    # https://matplotlib.org/stable/gallery/mplot3d/subplot3d.html
    fig = plt.figure()
    title = fig.suptitle('RGB + IR + Depth + PCL ({}/{})'.format(1, max_data), fontsize=30)
    ax00 = fig.add_subplot(2, 2, 1)
    ax01 = fig.add_subplot(2, 2, 2)
    ax10 = fig.add_subplot(2, 2, 3)
    ax11 = fig.add_subplot(2, 2, 4, projection='3d')
    im0 = ax00.imshow(color_vec[0])
    im1 = ax01.imshow(ir_vec[0], cmap='gray')
    im2 = ax10.imshow(depth_vec[0])
    im3 = ax11.scatter(pcl_vec[0][::pcl_subsample,::pcl_subsample,0],
                       pcl_vec[0][::pcl_subsample,::pcl_subsample,1],
                       pcl_vec[0][::pcl_subsample,::pcl_subsample,2])

    def init():
        im0.set_array(color_vec[0])
        im1.set_array(ir_vec[0])
        im2.set_array(depth_vec[0])
        subsample = pcl_vec[0][::pcl_subsample,::pcl_subsample,:2]
        # https://stackoverflow.com/a/9416663
        im3.set_offsets(subsample.reshape(-1,2))
        im3.set_array(pcl_vec[0][::pcl_subsample,::pcl_subsample,2].flatten())

    # https://stackoverflow.com/a/57259405
    def update_func(frame):
        if frame >= len(color_vec):
            color_data, ir_data, depth_data, pcl_data = load_data(frame, input_color_filenames,
                                                                  input_infrared_filenames, input_depth_filenames,
                                                                  input_pcl_filenames)
            color_vec.append(color_data)
            ir_vec.append(ir_data)
            depth_vec.append(depth_data)
            pcl_vec.append(pcl_data)

        im0.set_array(color_vec[frame])
        im1.set_array(ir_vec[frame])
        im2.set_array(depth_vec[frame])
        # Did not found a way to simultaneously update scatter data and the axis limits
        # So here we clear and replot the data
        subsample_xy = pcl_vec[frame][::pcl_subsample,::pcl_subsample,:2].reshape(-1,2)
        subsample_z = pcl_vec[frame][::pcl_subsample,::pcl_subsample,2].flatten()
        ax11.clear()
        im3 = ax11.scatter(subsample_xy[:,0],
                           subsample_xy[:,1],
                           subsample_z)
        ax11.set_xlabel('X')
        ax11.set_ylabel('Y')
        ax11.set_zlabel('Z')
        # # This does not work
        # ax11.clear()
        # # https://stackoverflow.com/a/27741495
        # corners = (min(subsample_xy[:,0]), min(subsample_xy[:,1]), min(subsample_z)), \
        #           (max(subsample_xy[:,0]), max(subsample_xy[:,1]), max(subsample_z))
        # print(f"corners={corners}")
        # ax11.update_datalim(corners)
        # ax11.margins(0.05, 0.05, 0.05)
        # ax11.autoscale_view()
        # im3.set_offsets(subsample_xy)
        # im3.set_array(subsample_z)
        # ax11.add_collection(im3)

        title.set_text('RGB + IR + Depth + PCL ({}/{})'.format(frame+1, max_data))

    # https://matplotlib.org/stable/api/animation_api.html
    anim = animation.FuncAnimation(
                                    fig,
                                    update_func,
                                    frames = np.arange(len(input_color_filenames)),
                                    interval = 33, # in ms
                                    init_func = init
                                    )

    plt.gcf().canvas.mpl_connect('key_press_event', close_figure)
    plt.show(block=False)
    plt.pause(0.033)

    if abort:
        return

if __name__ == '__main__':
    main()
