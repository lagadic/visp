import argparse
from pathlib import Path
from typing import List

from visp.core import XmlParserCamera, CameraParameters
from visp.mbt import MbGenericTracker

class MBTModelData:
  def __init__(self, data_root: Path):
    model_path = data_root / 'model' / 'teabox'
    assert model_path.exists()
    self.config_color = model_path / 'teabox_color.xml'
    self.config_depth = model_path / 'teabox_depth.xml'
    self.cad_file = model_path / 'teabox.cao'
    self.init_file = model_path / 'teabox.init'


class MBTConfig:

  def __init__(self, data_root: Path):
    data_path = data_root / 'data'
    assert data_path.exists()
    self.color_camera_name = 'Camera_L'
    self.depth_camera_name = 'Camera_R'

    self.color_intrinsics_file = data_path / f'{self.color_camera_name}.xml'
    self.depth_intrinsics_file = data_path / f'{self.depth_camera_name}.xml'

    self.extrinsic_file = str(data_root / 'data' / 'depth_M_color.txt')
    # self.ground_truth = str(data_root / 'data' / 'depth_M_color.txt')



def parse_camera_file(file: Path, camera_name: str, proj_model: CameraParameters.CameraParametersProjType) -> CameraParameters:
  assert file.exists()
  cam = CameraParameters()
  xml_parser = XmlParserCamera()
  parse_res = xml_parser.parse(cam, str(file.absolute()), camera_name, proj_model)
  assert parse_res == XmlParserCamera.SEQUENCE_OK # Check result
  return cam

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data-root', type=str, required=True,
                      help='Path to the folder containing all the data for the synthetic MBT example')
  parser.add_argument('--display-ground-truth', action='store_true')
  parser.add_argument('--disable-klt', action='store_true')
  parser.add_argument('--disable-depth', action='store_true')



  args = parser.parse_args()
  data_root = Path(args.data_root)

  mbt_model = MBTModelData(data_root)
  exp_config = MBTConfig(data_root)

  assert data_root.exists() and data_root.is_dir()

  rgb_tracker: int = MbGenericTracker.EDGE_TRACKER | (MbGenericTracker.KLT_TRACKER if not args.disable_klt else 0)
  tracker_types: List[int] = [rgb_tracker]
  if not args.disable_depth:
    depth_tracker = MbGenericTracker.DEPTH_DENSE_TRACKER
    tracker_types.append(depth_tracker)

  tracker = MbGenericTracker(tracker_types)

  if args.disable_depth:
    tracker.loadConfigFile(str(mbt_model.config_color))
  else:
    tracker.loadConfigFile(str(mbt_model.config_color), str(mbt_model.config_depth))
  tracker.loadModel(str(mbt_model.cad_file))
