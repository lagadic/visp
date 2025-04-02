#include <visp3/core/vpHSV.h>

BEGIN_VISP_NAMESPACE

template<>
const float vpHSV<double>::maxGradValue = vpHSV<double>::mahalanobisDistance<float>(vpHSV<double>(0., 0., 0.), vpHSV<double>(1., 1., 1.));

template<>
const float vpHSV<unsigned char, true>::maxGradValue = vpHSV<unsigned char, true>::mahalanobisDistance<float>(vpHSV<unsigned char, true>(0U, 0U, 0U), vpHSV<unsigned char, true>(255U, 255U, 255U));

template<>
const float vpHSV<unsigned char, false>::maxGradValue = vpHSV<unsigned char, false>::mahalanobisDistance<float>(vpHSV<unsigned char, false>(0U, 0U, 0U), vpHSV<unsigned char, false>(180U, 255U, 255U));

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
