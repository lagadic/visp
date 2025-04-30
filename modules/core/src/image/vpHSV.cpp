#include <visp3/core/vpHSV.h>

BEGIN_VISP_NAMESPACE

template<>
vpHSV<double> &vpHSV<double>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  set(hsv, 0., 1.);
  return *this;
}

template<>
vpHSV<unsigned char, true> &vpHSV<unsigned char, true>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  hsv[0] *= 255.;
  hsv[1] *= 255.;
  hsv[2] *= 255.;

  set(hsv, static_cast<unsigned char>(0), static_cast<unsigned char>(255.));
  return *this;
}

template<>
vpHSV<unsigned char, false> &vpHSV<unsigned char, false>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  hsv[0] *= static_cast<double>(maxHueUsingLimitedRange);
  hsv[1] *= 255.;
  hsv[2] *= 255.;

  set(hsv, static_cast<unsigned char>(0), static_cast<unsigned char>(255.));
  return *this;
}

END_VISP_NAMESPACE
