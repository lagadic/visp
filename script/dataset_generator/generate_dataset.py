
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

import os
import numpy as np
import argparse
import json
from pathlib import Path
import subprocess


if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('--config', required=True, type=str, help='Path to the JSON configuration file for the dataset generation script')
  args = parser.parse_args()
  config_path = Path(args.config)
  assert config_path.exists(), f'Config file {config_path} does not exist'
  with open(config_path, 'r') as json_config_file:
    json_config = json.load(json_config_file)
  
  num_scenes = json_config['dataset']['num_scenes']
  scenes_per_run = json_config['dataset']['scenes_per_run']
  
  print(__file__)
  scene_generator = (Path(__file__).parent / 'generate_scene.py').absolute()
  import time
  for i in range(num_scenes):
    args = [
      'blenderproc', 'run', str(scene_generator),
      '--config', str(config_path.absolute()),
      '--scene-index', str(i * scenes_per_run),
      '--scene-count', str(scenes_per_run)
    ]
    t = time.time()
    result = subprocess.run(args)
    print(f'Generating scene {i} took {time.time() - t}s')
    if result.returncode != 0:
      raise RuntimeError(f'Generation of scene {i} failed!')



