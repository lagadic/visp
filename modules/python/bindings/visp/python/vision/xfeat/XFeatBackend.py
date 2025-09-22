from typing import List, Optional, Tuple
import numpy as np
import time
import sys
import os
from pathlib import Path

if not 'XFEAT_PATH' in os.environ:
  raise EnvironmentError('you should set the value of the environment variable XFEAT_PATH to the folder containing the xfeat sources')

xfeat_path = Path(os.environ['XFEAT_PATH']).absolute()
if not xfeat_path.exists():
  print('XFeat could not be found at location', str(xfeat_path))
  sys.exit(1)

sys.path.append(xfeat_path)
from modules.xfeat import XFeat

import torch
import torch.nn.functional as F


from visp.core import ImageRGBa, Matrix


class XFeatRepresentation():
  def __init__(self, kps, descriptors):
    assert isinstance(kps, np.ndarray) and len(kps.shape) == 2 and kps.shape[1] == 2
    assert len(kps) == len(descriptors)
    self.keypoints = kps
    self.descriptors = descriptors

  def split(self, i1, i2) -> Tuple['XFeatRepresentation', 'XFeatRepresentation']:
    r1, r2 = None, None
    if len(i1) > 0:
      kps = self.keypoints[i1]
      descriptors = self.descriptors[i1]
      r1 = XFeatRepresentation(kps, descriptors)
    if len(i2) > 0:
      kps = self.keypoints[i2]
      descriptors = self.descriptors[i2]
      r2 = XFeatRepresentation(kps, descriptors)

    return r1, r2

  def merged_with(self, r: 'XFeatRepresentation') -> 'XFeatRepresentation':
    keypoints = np.concatenate((self.keypoints, r.keypoints), axis=0)
    descriptors = torch.concatenate((self.descriptors, r.descriptors), dim=0)
    return XFeatRepresentation(keypoints, descriptors)


class XFeatStarRepresentation(XFeatRepresentation):
  def __init__(self, kps, descriptors, scales):
    super().__init__(kps, descriptors)
    self.repr = {
      'keypoints': kps,
      'descriptors': descriptors,
      'scale': scales
    }

  def split(self, i1, i2):
    repr1 = {k: v[i1] for k, v in self.repr.items()}
    repr2 = {k: v[i2] for k, v in self.repr.items()}
    return (XFeatStarRepresentation(*(r[k] for k in ['keypoints', 'descriptors', 'scale'])) for r in (repr1, repr2))

  def add(self, r: 'XFeatStarRepresentation'):
    raise RuntimeError('Not implemented')


class XFeatBackend():
  def __init__(self, num_points: int, min_cos: float, use_dense=False, scale_factor = 1.0):
    self.xfeat = XFeat().eval()

    self.k = num_points
    self.min_cos = min_cos
    self.use_dense = use_dense
    self.scale_factor = scale_factor


  def load_settings(self, d: dict):
    self.k = d['numPoints']
    self.min_cos = d['minCos']
    self.scale_factor = d.get('scaleFactor', 1.0)
    self.use_dense = d.get('useDense', self.use_dense)

  def refine_dense(self, feats1, feats2, matches, fine_conf=0.0):
    idx0, idx1 = matches[0]

    #Compute fine offsets
    offsets = self.xfeat.net.fine_matcher(torch.cat([feats1[idx0], feats2[idx1]],dim=-1))
    conf = F.softmax(offsets*3, dim=-1).max(dim=-1)[0]

    mask_good = conf > fine_conf

    return idx0[mask_good], idx1[mask_good]

  def match_dense(self, r1: XFeatStarRepresentation, descriptors: XFeatStarRepresentation, min_cossim=None):
    if min_cossim is None:
      min_cossim = self.min_cos
    with torch.no_grad():

      idxs_list = self.xfeat.batch_match(r1.descriptors[None], descriptors[None], min_cossim)
      indices_1, indices_2 = self.refine_dense(r1.descriptors, descriptors, matches=idxs_list)
      return indices_1, indices_2

  def match(self, r1: XFeatRepresentation, descriptors: torch.Tensor, min_cos=None): # Return indices of matched points
    if r1 is None:
      return [], []
    if min_cos is None:
      min_cos = self.min_cos
    if not self.use_dense:
      return self.xfeat.match(r1.descriptors, descriptors, min_cossim=min_cos)
    else:
      assert isinstance(r1, XFeatStarRepresentation)
      return self.match_dense(r1, descriptors, min_cos)

  def computeDense(self, image: ImageRGBa, top_k = None):
    with torch.no_grad():
      if top_k is None:
        top_k = self.k
      rgb = self.xfeat.parse_input(image.numpy())
      if self.scale_factor != 1:
        t1 = time.time()
        rgb = torch.nn.functional.interpolate(rgb, scale_factor=self.scale_factor, mode='nearest', antialias=True, align_corners=True)
        print('Interp took: ', (time.time() - t1) * 1000)

      t1 = time.time()
      representation = self.xfeat.detectAndComputeDense(rgb, top_k = top_k, multiscale=True)


      kps = representation['keypoints'][0]
      if self.scale_factor != 1:
        kps /= self.scale_factor
      return kps.cpu().numpy(), representation['descriptors'][0], representation['scales'][0].cpu().numpy()

  def detect(self, IRGB) -> XFeatRepresentation:
    if self.use_dense:
      return XFeatStarRepresentation(*self.computeDense(IRGB, self.k))
    else:
      rgb = torch.from_numpy(IRGB.numpy()[None, ..., :3]).to(self.xfeat.dev).permute((0, 3, 1, 2)) / 255.0
      # rgb = torch.from_numpy(IRGB.numpy()).to('cuda')
      # rgb = self.xfeat.parse_input(IRGB.numpy())

      if self.scale_factor != 1:
        rgb = torch.nn.functional.interpolate(rgb, scale_factor=self.scale_factor, mode='nearest')
      t1 = time.time()
      rep = self.xfeat.detectAndCompute(rgb, top_k = self.k)[0]

      kps = rep['keypoints'].cpu().numpy()
      if self.scale_factor != 1:
        kps /= self.scale_factor
      #print('full took: ', (time.time() - detect_start) * 1000)
      return XFeatRepresentation(kps, rep['descriptors'])
