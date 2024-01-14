def test_pixel_meter_convert_points():
  from visp.core import PixelMeterConversion, CameraParameters
  import numpy as np

  h, w = 240, 320
  cam = CameraParameters(px=600, py=600, u0=320, v0=240)

  vs, us = np.meshgrid(range(h), range(w), indexing='ij') # vs and us are 2D arrays

  xs, ys = PixelMeterConversion.convertPoints(cam, us, vs)
  # xs and ys have the same shape as us and vs
  assert xs.shape == (h, w) and ys.shape == (h, w)

  # Converting a numpy array to normalized coords has the same effect as calling on a single image point
  for v in range(h):
    for u in range(w):
      x, y = PixelMeterConversion.convertPoint(cam, u, v)

      assert x == xs[v, u] and y == ys[v, u]

def test_meter_pixel_convert_points():
  from visp.core import MeterPixelConversion, CameraParameters
  import numpy as np

  h, w = 240, 320
  cam = CameraParameters(px=600, py=600, u0=320, v0=240)

  # We use xs and ys as pixel coordinates here, but it's not really true (it's just more convenient)
  ys, xs = np.meshgrid(range(h), range(w), indexing='ij') # vs and us are 2D arrays

  us, vs = MeterPixelConversion.convertPoints(cam, xs, ys)
  # xs and ys have the same shape as us and vs
  assert us.shape == (h, w) and vs.shape == (h, w)

  # Converting a numpy array to normalized coords has the same effect as calling on a single image point
  for y in range(h):
    for x in range(w):
      u, v = MeterPixelConversion.convertPoint(cam, x, y)
      assert u == us[y, x] and v == vs[y, x]
