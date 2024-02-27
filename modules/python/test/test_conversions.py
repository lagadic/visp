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
