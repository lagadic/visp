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

'''
TODO:
 - Random objects selection: max number of objects, location, rotation, sample_on_surface tuto
 - Random cc0
 - Random distractors
 - Random light
'''
import numpy as np
import argparse
import json
from operator import itemgetter
from typing import Dict


class Config:
  def __init__(self, config_path):
    self.config_path = config_path
    with open(self.config_path, 'r') as json_config_file:
      self.json_config = json.load(json_config_file)
    print(self.json_config)

    self.objects = self.get_objects()
    self.cc_textures = bproc.loader.load_ccmaterials(self.json_config['cc_textures_path'])

  def get_objects(self) -> Dict:
    models_path = self.json_config['models_path']


  def setup_renderer(self) -> None:
    depth, normals, segmentation = itemgetter('depth', 'normals', 'segmentation')(self.json_config['dataset'])

    if depth:
      bproc.renderer.enable_depth_output(activate_antialiasing=False)
    if normals:
      bproc.renderer.enable_normals_output()
    if segmentation:
      bproc.renderer.enable_segmentation_output()

  def set_camera_intrinsics(self) -> None:
    '''
    Set camera intrinsics from config.
    Randomized depending on randomize_params_percent. This does not impact image resolution
    '''
    px, py, u0, v0, h, w , r = itemgetter('px', 'py', 'u0', 'v0', 'h', 'w', 'randomize_params_percent')(self.json_config['camera'])
    r = r / 100.0
    randomize = lambda x: x + np.random.uniform(-x * r, x * r)
    K = [
      [randomize(px), 0, randomize(u0)],
      [0, randomize(py), randomize(v0)],
      [0, 0, 1],
    ]
    bproc.camera.set_intrinsics_from_K_matrix(K, w, h)


def create_room(size, textures):
  ground = bproc.object.create_primitive('PLANE')
  ground.set_location([0, 0, 0])
  ground.set_scale([size / 2, size / 2, 1])

  objects = [ground]
  wall_data = [
    {'loc': [size / 2, 0, size / 2], 'rot': [0, np.pi / 2, 0]},
    {'loc': [-size / 2, 0, size / 2], 'rot': [0, np.pi / 2, 0]},
    {'loc': [0, size / 2, size / 2], 'rot': [np.pi / 2, 0, 0]},
    {'loc': [0, -size / 2, size / 2], 'rot': [np.pi / 2, 0, 0]},
  ]
  for w_data in wall_data:
    wall = bproc.object.create_primitive('PLANE')
    wall.set_location(w_data['loc'])
    wall.set_rotation_euler(w_data['rot'])
    wall.set_scale([size / 2, size / 2, 1])
    objects.append(wall)

  for obj in objects:
    random_texture = np.random.choice(textures)
    obj.replace_materials(random_texture)


  return objects







from blenderproc.python.renderer import RendererUtility

if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('--config', required=True, type=str, help='Path to the JSON configuration file for the dataset generation script')
  args = parser.parse_args()



  RendererUtility.set_render_devices(use_only_cpu=True)
  bproc.clean_up(clean_up_camera=True)
  config = Config(args.config)
  # bproc.init() # Works if you have a GPU
  config.set_camera_intrinsics()
  # Create a simple object:
  obj = bproc.object.create_primitive("MONKEY")

  # Create a point light next to it
  light = bproc.types.Light()
  light.set_location([2, -2, 0])
  light.set_energy(1000)

  # Set the camera to be in front of the object
  cam_pose = bproc.math.build_transformation_mat([0, -5, 1], [np.pi / 2, 0, 0])
  bproc.camera.add_camera_pose(cam_pose)
  create_room(10.0, config.cc_textures)

  # Render the scene

  config.setup_renderer()

  data = bproc.renderer.render()

  # Write the rendering into an hdf5 file
  bproc.writer.write_hdf5("output/", data)

