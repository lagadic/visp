import blenderproc as bproc
'''
Script to generate a synthetic dataset, stored in hdf5 format.
This dataset can contain:
 - RGB images
 - Object poses
 - Segmentation maps
 - Bounding boxes
 - Depth map
 - Normal map

To run, blenderproc and other things should be installed (virtual environment recommended):
$ conda activate dataset_generation
$ pip install blenderproc numpy
$ blenderproc quickstart # verify that it works, install minor dependencies


'''
import numpy as np
import argparse
import json

class Config:
  def __init__(self, config_path):
    self.config_path = config_path
    self.parser = argparse.ArgumentParser(prefix_chars='--')
    self.parser.add_argument('config', type=str, required=False, help='Path to the JSON config file')
    self.parser.add_argument('height', type=int, required=False, help='Height of the generated images')
    self.parser.add_argument('width', type=int, required=False, help='Width of the generated images')
    self.parser.add_argument('px', type=float, required=False, help='Camera parameter: ratio between pixel width and focal length')
    self.parser.add_argument('py', type=float, required=False, help='Camera parameter: ratio between pixel height and focal length')
    self.parser.add_argument('u0', type=float, required=False, help='Camera parameter: x-axis location of the principal point, in pixels')
    self.parser.add_argument('v0', type=float, required=False, help='Camera parameter: y-axis location of the principal point, in pixels')
    self.parser.add_argument('camera-randomization-range', type=float, required=False, help='Amount of randomization applied to a camera\'s intrinsics parameters, as a percentage of the base values')
    
    
    

    
    
    # with open()


if __name__ == '__main__':

  bproc.init()

  # Create a simple object:
  obj = bproc.object.create_primitive("MONKEY")

  # Create a point light next to it
  light = bproc.types.Light()
  light.set_location([2, -2, 0])
  light.set_energy(300)

  # Set the camera to be in front of the object
  cam_pose = bproc.math.build_transformation_mat([0, -5, 0], [np.pi / 2, 0, 0])
  bproc.camera.add_camera_pose(cam_pose)

  # Render the scene
  data = bproc.renderer.render()

  # Write the rendering into an hdf5 file
  bproc.writer.write_hdf5("output/", data)

