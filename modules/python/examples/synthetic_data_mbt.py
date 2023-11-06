import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from visp.core import XmlParserCamera, CameraParameters, ColVector, HomogeneousMatrix, Display, ImageConvert
from visp.core import ImageGray, ImageUInt16
from visp.io import ImageIo
from visp.mbt import MbGenericTracker, MbTracker
from visp.gui import DisplayOpenCV

try:
  import cv2
except:
  print('Could not import opencv python! make sure that it is installed as it is required')
  import sys
  sys.exit(1)

import matplotlib.pyplot as plt

# bool read_data(unsigned int cpt, const std::string &video_color_images, const std::string &video_depth_images,
#                bool disable_depth, const std::string &video_ground_truth,
#                vpImage<unsigned char> &I, vpImage<uint16_t> &I_depth_raw,
#                unsigned int &depth_width, unsigned int &depth_height,
#                std::vector<vpColVector> &pointcloud, const vpCameraParameters &cam_depth,
#                vpHomogeneousMatrix &cMo_ground_truth)
# {
#   char buffer[FILENAME_MAX];
#   // Read color
#   snprintf(buffer, FILENAME_MAX, video_color_images.c_str(), cpt);
#   std::string filename_color = buffer;

#   if (!vpIoTools::checkFilename(filename_color)) {
#     std::cerr << "Cannot read: " << filename_color << std::endl;
#     return false;
#   }
#   vpImageIo::read(I, filename_color);

#   if (!disable_depth) {
#     // Read depth
#     snprintf(buffer, FILENAME_MAX, video_depth_images.c_str(), cpt);
#     std::string filename_depth = buffer;

#     if (!vpIoTools::checkFilename(filename_depth)) {
#       std::cerr << "Cannot read: " << filename_depth << std::endl;
#       return false;
#     }
#     cv::Mat depth_raw = cv::imread(filename_depth, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
#     if (depth_raw.empty()) {
#       std::cerr << "Cannot read: " << filename_depth << std::endl;
#       return false;
#     }

#     depth_width = static_cast<unsigned int>(depth_raw.cols);
#     depth_height = static_cast<unsigned int>(depth_raw.rows);
#     I_depth_raw.resize(depth_height, depth_width);
#     pointcloud.resize(depth_width * depth_height);

#     for (int i = 0; i < depth_raw.rows; i++) {
#       for (int j = 0; j < depth_raw.cols; j++) {
#         I_depth_raw[i][j] = static_cast<uint16_t>(32767.5f * depth_raw.at<cv::Vec3f>(i, j)[0]);
#         double x = 0.0, y = 0.0;
#         // Manually limit the field of view of the depth camera
#         double Z = depth_raw.at<cv::Vec3f>(i, j)[0] > 2.0f ? 0.0 : static_cast<double>(depth_raw.at<cv::Vec3f>(i, j)[0]);
#         vpPixelMeterConversion::convertPoint(cam_depth, j, i, x, y);
#         size_t idx = static_cast<size_t>(i * depth_raw.cols + j);
#         pointcloud[idx].resize(3);
#         pointcloud[idx][0] = x * Z;
#         pointcloud[idx][1] = y * Z;
#         pointcloud[idx][2] = Z;
#       }
#     }
#   }

#   // Read ground truth
#   snprintf(buffer, FILENAME_MAX, video_ground_truth.c_str(), cpt);
#   std::string filename_pose = buffer;

#   cMo_ground_truth.load(filename_pose);

#   return true;







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
    data_path = data_root / 'data' / 'teabox'
    assert data_path.exists()
    self.color_camera_name = 'Camera_L'
    self.depth_camera_name = 'Camera_R'

    self.color_intrinsics_file = data_path / f'{self.color_camera_name}.xml'
    self.depth_intrinsics_file = data_path / f'{self.depth_camera_name}.xml'

    self.color_images_dir = data_path / 'color'
    self.depth_images_dir = data_path / 'depth'
    self.ground_truth_dir = data_path / 'ground-truth'


    self.depth_intrinsics_file = data_path / f'{self.depth_camera_name}.xml'

    self.extrinsic_file = str(data_path / 'depth_M_color.txt')
    # self.ground_truth = str(data_root / 'data' / 'depth_M_color.txt')

@dataclass
class FrameData:
  I: ImageGray
  I_depth: Optional[ImageUInt16]
  point_cloud: Optional[List[ColVector]]
  cMo_ground_truth: HomogeneousMatrix



def read_data(exp_config: MBTConfig, use_depth: bool, I: ImageGray):
  color_format = '{:04d}_L.jpg'
  depth_format = 'Image{:04d}_R.exr'

  iteration = 1
  while True:
    color_filepath = exp_config.color_images_dir / color_format.format(iteration)
    if not color_filepath.exists():
      print(f'Could not find image {color_filepath}, is the sequence finished?')
      return
    ImageIo.read(I, str(color_filepath), ImageIo.IO_DEFAULT_BACKEND)


    I_depth_raw = None
    if use_depth:
      depth_filepath = exp_config.depth_images_dir / depth_format.format(iteration)
      if not depth_filepath.exists():
        print(f'Could not find image {depth_filepath}, is the sequence finished?')
        return
      I_depth_np = cv2.imread(str(depth_filepath), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
      I_depth_np = I_depth_np[..., 0]
      I_depth_raw = ImageUInt16(I_depth_np * 32767.5)
      if I_depth_np.size == 0:
        print('Could not successfully read the depth image')

    cMo_ground_truth = HomogeneousMatrix()
    ground_truth_file = exp_config.ground_truth_dir / (exp_config.color_camera_name + '_{:04d}.txt'.format(iteration))
    cMo_ground_truth.load(str(ground_truth_file))
    iteration += 1
    yield FrameData(I, I_depth_raw, None, cMo_ground_truth)



def parse_camera_file(exp_config: MBTConfig, is_color: bool) -> CameraParameters:
  cam = CameraParameters()
  xml_parser = XmlParserCamera()
  if is_color:
    camera_name, file_path = exp_config.color_camera_name, exp_config.color_intrinsics_file
  else:
    camera_name, file_path = exp_config.depth_camera_name, exp_config.depth_intrinsics_file
  parse_res = xml_parser.parse(cam, str(file_path), camera_name,
                               CameraParameters.perspectiveProjWithoutDistortion, 0, 0, True)
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

  # Camera intrinsics
  cam_color = parse_camera_file(exp_config, True)
  cam_depth = parse_camera_file(exp_config, False) if not args.disable_depth else None

  tracker.setCameraParameters(*((cam_color,) if args.disable_depth else (cam_color, cam_depth)))
  tracker.setDisplayFeatures(True)

  print('Color intrinsics:', cam_color)
  print('Depth intrinsics:', cam_depth)
  I = ImageGray()
  data_generator = read_data(exp_config, not args.disable_depth, I)

  frame_data = next(data_generator) # Get first frame for init

  depth_M_color = HomogeneousMatrix()
  if not args.disable_depth:
    depth_M_color.load(exp_config.extrinsic_file)
    tracker.setCameraTransformationMatrix('Camera2', depth_M_color)


  # tracker.initClick(I, str(mbt_model.init_file), True, HomogeneousMatrix()) TODO: does not work
  tracker.initFromPose(I, frame_data.cMo_ground_truth)

  # Initialize displays
  dI = DisplayOpenCV()
  dI.init(I, 0, 0, 'Color image')
  I_depth = None if args.disable_depth else ImageGray()
  dDepth = DisplayOpenCV()
  if not args.disable_depth:
    ImageConvert.createDepthHistogram(frame_data.I_depth, I_depth)
    dDepth.init(I_depth, 0, 640, 'Depth')

  for frame_data in data_generator:
    if frame_data.I_depth is not None:
      ImageConvert.createDepthHistogram(frame_data.I_depth, I_depth)
    Display.display(I)
    if not args.disable_depth:
      Display.display(I_depth)
    tracker.track(I)
    cMo = HomogeneousMatrix()
    tracker.getPose(cMo)
    print(cMo)
    Display.flush(I)
    if not args.disable_depth:
      Display.flush(I_depth)





    pass
