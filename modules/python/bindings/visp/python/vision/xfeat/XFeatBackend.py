#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Display helpers for ViSP
#
#############################################################################
from typing import List, Optional, Tuple
import numpy as np
import time
import sys
import os
from pathlib import Path
import torch
import torch.nn.functional as F


from visp.core import ImageRGBa

class XFeatRepresentation():
  def __init__(self, kps, descriptors):
    assert isinstance(kps, np.ndarray) and len(kps.shape) == 2 and kps.shape[1] == 2
    assert len(kps) == len(descriptors)
    self.keypoints: np.ndarray = kps
    self.descriptors: torch.Tensor = descriptors

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
    self.scales = scales

  def split(self, i1, i2):
    repr1 = {k: v[i1] for k, v in self.repr.items()}
    repr2 = {k: v[i2] for k, v in self.repr.items()}
    return (XFeatStarRepresentation(*(r[k] for k in ['keypoints', 'descriptors', 'scale'])) for r in (repr1, repr2))

  def add(self, r: 'XFeatStarRepresentation'):
    raise RuntimeError('Not implemented')


class XFeatBackend():
  def __init__(self, num_points: int, min_cos: float, use_dense=False, scale_factor = 1.0):
    try:
      self.xfeat = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained = True, top_k = 4096).eval()
    except:
      print('Could not load XFeat from torchhub', file=sys.stderr)
      xfeat_env_var_name = 'XFEAT_PATH'
      print(f'Looking at {xfeat_env_var_name} environment variable to load XFeat!', file=sys.stderr)

      if not xfeat_env_var_name in os.environ:
        raise EnvironmentError('you should set the value of the environment variable XFEAT_PATH to the folder containing the xfeat sources')

      xfeat_path = Path(os.environ[xfeat_env_var_name]).absolute()
      if not xfeat_path.exists():
        raise EnvironmentError(f'XFeat folder {str(xfeat_path)} does not exist')

      sys.path.append(str(xfeat_path))
      from modules.xfeat import XFeat
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

  def match_dense(self, r1: torch.Tensor, descriptors: torch.Tensor, min_cossim=None):
    if min_cossim is None:
      min_cossim = self.min_cos
    with torch.no_grad():
      idxs_list = self.xfeat.batch_match(r1[None], descriptors[None], min_cossim)
      indices_1, indices_2 = self.refine_dense(r1, descriptors, matches=idxs_list)
      return indices_1, indices_2

  def match(self, r1: torch.Tensor, descriptors: torch.Tensor, min_cos=None): # Return indices of matched points
    if r1 is None:
      return [], []
    if min_cos is None:
      min_cos = self.min_cos

    if len(r1.size()) == 3:
      assert r1.size(0) == 1
      r1 = r1[0]
    if len(descriptors.size()) == 3:
      assert descriptors.size(0) == 1
      descriptors = descriptors[0]

    if not self.use_dense:
      return self.xfeat.match(r1, descriptors, min_cossim=min_cos)
    else:
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
