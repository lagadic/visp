
import argparse
import sys

from visp.core import CameraParameters, HomogeneousMatrix, ImageGray, ImageConvert
from visp.detection import DetectorAprilTag


import pyrealsense2 as rs
import numpy as np
from typing import Generator

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
  return args


#### Data acquisition

class FrameSource():
  def intrinsics(self) -> CameraParameters:
    raise NotImplementedError()
  def frames(self) -> Generator[ImageGray]:
    raise NotImplementedError()

class RealSenseSource(FrameSource):
  def __init__(self, args):
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

def build_source_from_args(args) -> FrameSource:
  if args.source == 'rs2':
    return RealSenseSource(args)
  elif args.source == 'sequence':
    raise NotImplementedError('Sequence parsing not yet implemented')

def main():
  parser = argparse.ArgumentParser(description='AprilTag detection program from a Realsense camera')
  tag_parser = parser.add_argument_group('AprilTag options')
  tag_parser.add_argument('--tag-family', choices=family_mapping.keys(), default='36h11', help='The family of apriltags to detect')
  tag_parser.add_argument('--tag-size', type=float, default=0.0955, help='The size of the tags to detect, in meters')
  tag_parser.add_argument('--tag-pose-method', choices=method_mapping.keys(), default='homography_vvs', help='The method used to estimated the 6D pose of the tags')
  tag_parser.add_argument('--tag-decimate', type=float, required=False, default=1.0)
  tag_parser.add_argument('--tag-decision-margin', type=float, required=False, default=2.0)
  tag_parser.add_argument('--tag-hamming-distance', type=int, required=False, default=50)

  source_parser = parser.add_argument_group('Image acquisition parsing')
  source_parser.add_argument('--source', help='Image source', default='rs2')
  source_parser.add_argument('--width', help='Image width', default=640)
  source_parser.add_argument('--height', help='Image height', default=480)
  source_parser.add_argument('--fps', help='Framerate', default=30)



  args, unknown_args = parser.parse_known_args()


  if len(unknown_args) > 0:
    print(f'There were unknown arguments: {unknown_args}', file=sys.stderr)
    raise RuntimeError()

  detector = build_detector_from_args(args)
  tag_size = args.tag_size

  frame_source = build_source_from_args(args)
  frames = frame_source.frames()
  for frame in frames:
    print(frame)


if __name__ == '__main__':
  main()
