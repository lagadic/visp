'''
Script to export an hdf5 stored dataset to the format expected by a YoloV7.
This requires bounding boxes to be generated along with the RGB images.
This is done through the json config file.

'''

from pathlib import Path
import argparse
from typing import List
import h5py
from PIL import Image
import numpy as np
import json
import progressbar
import yaml

def make_name(hdf5_path: Path):
  hdf5_name = hdf5_path.name[:-len('.hdf5')]
  return hdf5_path.parent.name + '_' + hdf5_name

def export_split(hdf5_paths: List[Path], images_export_path: Path, labels_export_path: Path):
  bar = progressbar.ProgressBar()
  detection_data = {}
  for hdf5_path in bar(hdf5_paths):
    with h5py.File(str(hdf5_path)) as hdf5:
      rgb = np.array(hdf5['colors'])
      img = Image.fromarray(rgb)
      base_name = make_name(hdf5_path)
      img.save(images_export_path / (base_name + '.png'))
      h, w = rgb.shape[:2]
      text = np.array(hdf5['object_data']).tobytes()
      with open(labels_export_path / (base_name + '.txt'), 'w') as label_file:
        if len(text) > 0:
          object_data = json.loads(text)
          for object in object_data:
            if 'bounding_box' in object:
              bb = object['bounding_box']
              # Yolo format: [x_center, y_center, width, height]. all coordinates normalized by image dimensions
              x_center, y_center = bb[0] + bb[2] / 2, bb[1] + bb[3] / 2
              x, width = x_center / w, bb[2] / w
              y, height = y_center / h, bb[3] / h
              bb = [x, y, width, height]
              cls = object['class']
              fmt = f'{cls - 1} {x} {y} {width} {height}\n'
              label_file.write(fmt)
              if cls not in detection_data:
                detection_data[cls] = 0
              detection_data[cls] += 1

  print('Detection summary:')
  for cls in detection_data.keys():
    print(f'\tClass: {cls}\t Detection count: {detection_data[cls]}')

if __name__ == '__main__':
  parser = argparse.ArgumentParser('Convert an HDF5 dataset generated with Blenderproc to a YoloV7-ready format')
  parser.add_argument('--input', type=str, required=True, help='''Path to the HDF5 dataset. It is the root folder of the dataset,
                       containing one subfolder for each generated scene''')
  parser.add_argument('--output', type=str, required=True, help='''Where to export the dataset to YoloV7 format''')
  parser.add_argument('--train-split', required=True, type=float, help='''Portion of the dataset to take for training. Between 0 and 1''')

  args = parser.parse_args()

  input_path = Path(args.input)
  assert input_path.exists(), f'Path to the HDF5 dataset must be valid and exist, but got {input_path}'

  output_path = Path(args.output)
  assert not output_path.exists() or len(list(output_path.iterdir())), f'The path to the exported folder should not exist, or should be empty'
  output_path.mkdir(exist_ok=True)

  train_split = args.train_split
  assert train_split > 0 and train_split < 1, 'Train split should be in range (0, 1)'


  num_scenes = 0
  hdf5_paths = []
  print('Collecting hdf5 paths...')
  for scene_dir in input_path.iterdir():
    if not scene_dir.is_dir():
      continue
    for file in scene_dir.iterdir():
      if file.name.endswith('.hdf5'):
        hdf5_paths.append(file.absolute())
    num_scenes += 1
  print(f'Found {num_scenes} scenes, for a total of {len(hdf5_paths)} images!')
  train_count = int(len(hdf5_paths) * train_split)
  val_count = len(hdf5_paths) - train_count
  print(f'Splitting randomly: taking {train_count} images for test, {val_count} for validation')
  train_hdf5, val_hdf5 = [], []
  train_indices = set(np.random.choice(len(hdf5_paths), size=train_count, replace=False))
  for i in range(len(hdf5_paths)):
    train_hdf5.append(hdf5_paths[i]) if i in train_indices else val_hdf5.append(hdf5_paths[i])
  print(len(train_hdf5), len(val_hdf5))

  images_path = output_path / 'images'
  labels_path = output_path / 'labels'
  images_train_path = images_path / 'train'
  images_val_path = images_path / 'val'
  labels_train_path = labels_path / 'train'
  labels_val_path = labels_path / 'val'

  for path in [images_path, labels_path, images_train_path, images_val_path, labels_train_path, labels_val_path]:
    path.mkdir()

  classes = []
  with open(input_path / 'classes.txt') as cls_file:
    for line in cls_file.readlines():
      classes.append(line.strip())
  with open(output_path / 'dataset.yaml', 'w') as yaml_file:
    data = {
      'train': str(images_train_path.absolute()),
      'val': str(images_val_path.absolute()),
      'nc': len(classes),
      'names': classes
    }
    yaml.dump(data, yaml_file)

  print('Generating train split...')
  export_split(train_hdf5, images_train_path, labels_train_path)
  print('Generating validation split...')
  export_split(val_hdf5, images_val_path, labels_val_path)

