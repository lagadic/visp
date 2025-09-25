
import argparse
import sys
from pathlib import Path

from visp.core import CameraParameters, HomogeneousMatrix, ImageGray, ImageConvert, Color, Point, ImagePoint
from visp.detection import DetectorAprilTag
from visp.core import Display

from visp.display_utils import get_display

import numpy as np
from typing import Generator, List

##### Detector building

# Table for argument name to DetectorAprilTag family conversion
family_mapping = {
  '16h5': DetectorAprilTag.AprilTagFamily.TAG_16h5,
  '25h7': DetectorAprilTag.AprilTagFamily.TAG_25h7,
  '25h9': DetectorAprilTag.AprilTagFamily.TAG_25h9,
  '36h10': DetectorAprilTag.AprilTagFamily.TAG_36h10,
  '36h11': DetectorAprilTag.AprilTagFamily.TAG_36h11,
}

method_mapping = {
  'homography_vvs': DetectorAprilTag.PoseEstimationMethod.HOMOGRAPHY_VIRTUAL_VS,
  'dementhon_vvs': DetectorAprilTag.PoseEstimationMethod.DEMENTHON_VIRTUAL_VS,
  'lagrange_vvs': DetectorAprilTag.PoseEstimationMethod.LAGRANGE_VIRTUAL_VS
}


def build_detector_from_args(args) -> DetectorAprilTag:
  detector = DetectorAprilTag()
  detector.setAprilTagDecisionMarginThreshold(args.tag_decision_margin)
  detector.setAprilTagHammingDistanceThreshold(args.tag_hamming_distance)
  detector.setAprilTagQuadDecimate(args.tag_decimate)
  detector.setAprilTagFamily(family_mapping[args.tag_family])
  detector.setAprilTagPoseEstimationMethod(method_mapping[args.tag_pose_method])
  return detector


#### Data acquisition

class FrameSource():
  def intrinsics(self) -> CameraParameters:
    raise NotImplementedError()
  def frames(self) -> Generator[ImageGray]:
    raise NotImplementedError()

class RealSenseSource(FrameSource):
  def __init__(self, args):
    import pyrealsense2 as rs
    self.pipe = rs.pipeline()
    self.config = rs.config()
    self.h, self.w = args.height, args.width
    self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.rgb8, args.fps)

    self.cfg = self.pipe.start(self.config)
    self.profile = self.cfg.get_stream(rs.stream.color)
    intr = self.profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
    self.cam = CameraParameters(intr.fx, intr.fy, intr.ppx, intr.ppy)
    print(f'Opened Realsense camera with intrinsics:\n{self.cam}')

  def intrinsics(self) -> CameraParameters:
    return self.cam

  def frames(self) -> Generator[ImageGray]:

    def generator():
      while True:
        frame = self.pipe.wait_for_frames()
        img = ImageGray(self.h, self.w)
        I_np = np.asanyarray(frame.get_color_frame().as_frame().get_data())
        ImageConvert.RGBToGrey(I_np, img)
        yield img

    return generator()

class FileSource(FrameSource):
  def __init__(self, args):
    self.source_path = args.source
    from visp.io import VideoReader
    self.reader = VideoReader()
    self.reader.setFileName(self.source_path)

    self.cam = CameraParameters()
    if any(v == 0 for v in [args.px, args.py, args.u0, args.v0]):
      raise RuntimeError('Camera intrinsics need to be specified through the command line arguments')

    if args.kud != 0 or args.kdu != 0:
      self.cam.initPersProjWithDistortion(args.px, args.py, args.u0, args.v0, args.kud, args.kdu)
    else:
      self.cam.initPersProjWithoutDistortion(args.px, args.py, args.u0, args.v0)

  def intrinsics(self):
    return self.cam

  def frames(self):
    def generator():
      I = ImageGray()
      self.reader.open(I)
      while not self.reader.end():
        self.reader.acquire(I)
        yield I

    return generator()



def build_source_from_args(args) -> FrameSource:
  if args.source == 'rs2':
    return RealSenseSource(args)
  else:
    return FileSource(args)

class DetectionsLogger():
  def __init__(self, log_path: Path):
    self.log_path = log_path
    if self.log_path.exists():
      raise FileExistsError(f'File {str(self.log_path.absolute())} already exists')
    self.data = []
  def log_frame(self, detector: DetectorAprilTag, tag_size: float, cam: CameraParameters):
    ids = detector.getTagsId()
    tag_size_dict = {tag_id: tag_size for tag_id in ids}
    points_3d: List[List[Point]] = detector.getTagsPoints3D(ids, tag_size_dict)
    points_2d: List[List[ImagePoint]] = detector.getTagsCorners()
    frame_log = {}
    for i, tag_id in enumerate(ids):
      id_log = {
        'points3d': [p.get_oP().numpy().tolist() for p in points_3d[i]],
        'points2d': [(p.get_i(), p.get_j()) for p in points_2d[i]],
        'cMo': detector.getPose(i, tag_size, cam)[1].numpy().tolist()
      }

      frame_log[tag_id] = id_log
    self.data.append(frame_log)

  def finalize_logging(self):
    print('Writing detections to log file: ', str(self.log_path))
    with open(self.log_path, 'w') as log_file:
      import json
      json.dump(self.data, log_file, indent=4)

def main():
  parser = argparse.ArgumentParser(description='AprilTag detection program from a Realsense camera')
  tag_parser = parser.add_argument_group('AprilTag options')
  tag_parser.add_argument('--tag-family', choices=family_mapping.keys(), default='36h11', help='The family of apriltags to detect')
  tag_parser.add_argument('--tag-size', type=float, default=0.053, help='The size of the tags to detect, in meters')
  tag_parser.add_argument('--tag-pose-method', choices=method_mapping.keys(), default='homography_vvs', help='The method used to estimated the 6D pose of the tags')
  tag_parser.add_argument('--tag-decimate', type=float, required=False, default=1.0)
  tag_parser.add_argument('--tag-decision-margin', type=float, required=False, default=2.0)
  tag_parser.add_argument('--tag-hamming-distance', type=int, required=False, default=50)

  source_parser = parser.add_argument_group('Image acquisition parsing')
  source_help = '''Image source. if rs2, we open the realsense stream (if available).
  Otherwise we consider that it is a stored sequence. It should be specified as /path/to/img-%%d.jpg.
  See the visp.io.VideoReader documentation for more information'''
  source_parser.add_argument('--source', help=source_help, default='rs2')
  source_parser.add_argument('--width', help='Image width', default=640)
  source_parser.add_argument('--height', help='Image height', default=480)
  source_parser.add_argument('--fps', help='Framerate', default=30)

  camera_parser = parser.add_argument_group('Camera intrinsics')
  camera_parser.add_argument('--px', default=0.0, type=float)
  camera_parser.add_argument('--py', default=0.0, type=float)
  camera_parser.add_argument('--u0', default=0.0, type=float)
  camera_parser.add_argument('--v0', default=0.0, type=float)
  camera_parser.add_argument('--kud', default=0.0, type=float)
  camera_parser.add_argument('--kdu', default=0.0, type=float)


  parser.add_argument('--log-path', default=None, required=False, help='The path to the JSON file where to log the detections.')

  args, unknown_args = parser.parse_known_args()


  if len(unknown_args) > 0:
    print(f'There were unknown arguments: {unknown_args}', file=sys.stderr)
    raise RuntimeError()

  detector = build_detector_from_args(args)
  tag_size = args.tag_size

  frame_source = build_source_from_args(args)
  frames = frame_source.frames()

  logger = None
  if args.log_path is not None:
    log_path = Path(args.log_path)
    logger = DetectionsLogger(log_path)

  I_disp = ImageGray()

  cam = frame_source.intrinsics()

  first_frame = True
  for frame in frames:
    if first_frame:
      I_disp.resize(frame.getHeight(), frame.getWidth())
      gray_display = get_display()
      gray_display.init(I_disp, 0, 0, 'Image')
      first_frame = False

    # Copy into I_disp to display tags
    np.copyto(dst=I_disp.numpy(), src=frame.numpy())

    has_tag, tag_poses = detector.detect(frame, tag_size, cam)
    if logger is not None:
      logger.log_frame(detector, tag_size, cam)

    Display.display(I_disp)
    Display.displayText(I_disp, 20, 20, 'Click to stop detection', Color.red)
    detector.displayFrames(I_disp, tag_poses, cam, 0.05, Color.none)
    detector.displayTags(I_disp, detector.getTagsCorners())
    Display.flush(I_disp)

    clicked = Display.getClick(I_disp, blocking=False)
    if clicked:
      break

  if logger is not None:
    logger.finalize_logging()


if __name__ == '__main__':
  main()
