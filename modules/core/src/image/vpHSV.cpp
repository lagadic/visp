#include <visp3/core/vpHSV.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
BEGIN_VISP_NAMESPACE

template<>
vpHSV<double> &vpHSV<double>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  set(hsv);
  return *this;
}

template<>
vpHSV<unsigned char, true> &vpHSV<unsigned char, true>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  hsv[0] *= 255.;
  hsv[1] *= 255.;
  hsv[2] *= 255.;

  set(hsv);
  return *this;
}

template<>
vpHSV<unsigned char, false> &vpHSV<unsigned char, false>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  hsv[0] *= static_cast<double>(maxHueUsingLimitedRange);
  hsv[1] *= 255.;
  hsv[2] *= 255.;

  set(hsv);
  return *this;
}

END_VISP_NAMESPACE
#else
void dummy_vpHSV() { }
#endif
