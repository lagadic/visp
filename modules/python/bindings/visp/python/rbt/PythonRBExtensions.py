
from pathlib import Path
import json

from visp.rbt import RBTracker

from visp.python.rbt.xfeat import RBXFeatFeatureTracker, XFeatVisualOdometry, XFeatTrackingBackend


class PythonRBExtensions():
  '''
  This class is used to keep references to Python created types:
  If we do not do so, Python loses track of Python types that override C++ types
  and "slices" them back to pure cpp objects when calling eg. tracker.getOdometryMethod()
  '''
  def __init__(self):
    self.xfeat_backend = XFeatTrackingBackend(4096, 0.8)
    self.extensions = []

  def parse_python_extensions(self, tracker: RBTracker, json_path: Path):

    with open(json_path, 'r') as json_file:
      json_dict = json.load(json_file)

    if "python_ext" not in json_dict:
      print('No python extension found')
      return

    python_extensions = json_dict['python_ext']
    # XFeat backend is a special case for a Python extension:
    # Visual odometry and RBXfeatTracker.
    # Since both  depend on the same feature extraction,
    # we use the same backend  to run extraction only once.
    if 'xfeat' in python_extensions:
      self.xfeat_backend.load_settings(python_extensions['xfeat'])

    if 'odometry' in python_extensions:
      vo = XFeatVisualOdometry(self.xfeat_backend)
      vo.load_settings(python_extensions['odometry'])
      tracker.setOdometryMethod(vo)
      self.extensions.append(vo)

    if 'features' in python_extensions:
      features_json = python_extensions['features']
      assert isinstance(features_json, list)
      for fj in features_json:
        if fj['type'] == 'xfeat':
          xfeat = RBXFeatFeatureTracker(self.xfeat_backend)
          xfeat.load_settings(fj)
          self.extensions.append(xfeat)
          tracker.addTracker(xfeat)
