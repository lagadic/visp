from visp.core import ImageConvert
import numpy as np

def test_hsv_to_rgb_rgba():
  h, w = 50, 50
  cases = [
    {
      'bytes': 3,
      'input_dtype': np.uint8,
      'fn': ImageConvert.HSVToRGB
    },
    {
      'bytes': 3,
      'input_dtype': np.float64,
      'fn': ImageConvert.HSVToRGB
    },
    {
      'bytes': 4,
      'input_dtype': np.uint8,
      'fn': ImageConvert.HSVToRGBa
    },
    {
      'bytes': 4,
      'input_dtype': np.float64,
      'fn': ImageConvert.HSVToRGBa
    }
  ]
  for case in cases:
    hsv = np.zeros((3, h, w), dtype=case['input_dtype'])
    rgb = np.ones((h, w, case['bytes']), dtype=np.uint8)
    rgb_old = rgb.copy()
    case['fn'](hsv, rgb)
    assert not np.allclose(rgb, rgb_old)


def test_bgr_to_grey():
  h, w = 50, 50
  I = np.empty((h, w, 3), dtype=np.uint8)
  from visp.core import ImageGray
  out = ImageGray(h, w)
  ImageConvert.RGBToGrey(I, out)
  out2 = np.empty((h,w), dtype=np.uint8)
  ImageConvert.RGBToGrey(I, out2)
  assert np.all(np.equal(out, out2))


def test_bgr_to_rgba():
  h, w = 50, 50
  I = np.empty((h, w, 3), dtype=np.uint8)
  from visp.core import ImageRGBa
  out = ImageRGBa(h, w)
  ImageConvert.BGRToRGBa(I, out)
  out2 = np.empty((h, w, 4), dtype=np.uint8)
  ImageConvert.BGRToRGBa(I, out2)
  assert np.all(np.equal(out, out2))


def test_rgb_rgba_to_hsv():
  h, w = 50, 50
  cases = [
    {
      'bytes': 3,
      'input_dtype': np.uint8,
      'fn': ImageConvert.RGBToHSV
    },
    {
      'bytes': 3,
      'input_dtype': np.float64,
      'fn': ImageConvert.RGBToHSV
    },
    {
      'bytes': 4,
      'input_dtype': np.uint8,
      'fn': ImageConvert.RGBaToHSV
    },
    {
      'bytes': 4,
      'input_dtype': np.float64,
      'fn': ImageConvert.RGBaToHSV
    }
  ]
  for case in cases:
    hsv = np.zeros((3, h, w), dtype=case['input_dtype'])
    rgb = np.ones((h, w, case['bytes']), dtype=np.uint8)
    hsv_old = hsv.copy()
    case['fn'](rgb, hsv)
    assert not np.allclose(hsv, hsv_old)



def test_demosaic():
  h, w = 32, 32
  fns = [
    ImageConvert.demosaicRGGBToRGBaMalvar,
    ImageConvert.demosaicGRBGToRGBaMalvar,
    ImageConvert.demosaicGBRGToRGBaMalvar,
    ImageConvert.demosaicBGGRToRGBaMalvar,
    ImageConvert.demosaicRGGBToRGBaBilinear,
    ImageConvert.demosaicGRBGToRGBaBilinear,
    ImageConvert.demosaicGBRGToRGBaBilinear,
    ImageConvert.demosaicBGGRToRGBaBilinear,
  ]
  for fn in fns:
    for dtype in [np.uint8, np.uint16]:
      bayer_data = np.ones((h, w), dtype=dtype) * 128
      rgba = np.zeros((h, w, 4), dtype=dtype)
      old_rgba = rgba.copy()
      fn(bayer_data, rgba)
      assert not np.allclose(rgba, old_rgba), f'Error when testing {fn}, with dtype {dtype}'
