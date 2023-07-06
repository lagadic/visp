import h5py
from pathlib import Path
import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import argparse

def show(file: h5py.File):
    fig = plt.figure()
    rgb = np.array(file['colors'])
    text = np.array(f["object_data"]).tostring()
    object_data = json.loads(text)
    for object in object_data:
      bb = object['bounding_box']
      rect = patches.Rectangle((bb[0], bb[1]), bb[2], bb[3], linewidth=1, edgecolor='r', facecolor='none')
      plt.gca().add_patch(rect)
    plt.imshow(rgb)
    plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--path', type=str, help='Path to a folder containing hdf5 files')
  args = parser.parse_args()
  folder = Path(args.path)
  assert folder.exists()
  for file in folder.iterdir():
    if file.name.endswith('.hdf5'):
      with h5py.File(str(file)) as f:
        print(f'Visualizing file {f}')
        show(f)
