#include <iostream>
#include <limits>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpHSV.h>

#include "hsvUtils.h"

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/**
 * \brief Check if the computed HSV value corresponds to the ground-truth.
 *
 * \tparam Type The type of the vpHSV channels.
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; vpHSV<unsigned char, false>::maxHueUsingLimitedRange].
 * \param[in] hsv_computed Computed vpRGBa value.
 * \param[in] rgb_truth vpRGBa value  that was used to compute rgb_computed.
 * \param[in] hsv_truth The HSVground-truth value.
 * \return true If hsv_computed and hsv_truth are equal.
 * \return false Otherwise
 */
template<typename Type, bool useFullScale >
bool test_hsv(const vpHSV<Type, useFullScale> &hsv_computed, const vpRGBa &rgb_truth,
              const vpHSV<Type, useFullScale> &hsv_truth)
{
  // Compare HSV values
  if ((!vpMath::equal(hsv_computed.H, hsv_truth.H)) ||
      (!vpMath::equal(hsv_computed.S, hsv_truth.S)) ||
      (!vpMath::equal(hsv_computed.V, hsv_truth.V))) {

    std::cout << static_cast<int>(rgb_truth.R) << ","
      << static_cast<int>(rgb_truth.G) << ","
      << static_cast<int>(rgb_truth.B) << "): Expected hsv value: ("
      << static_cast<int>(hsv_truth.H) << ","
      << static_cast<int>(hsv_truth.S) << ","
      << static_cast<int>(hsv_truth.V) << ") converted value: ("
      << static_cast<int>(hsv_computed.H) << ","
      << static_cast<int>(hsv_computed.S) << ","
      << static_cast<int>(hsv_computed.V) << ")" << std::endl;
    return false;
  }

  return true;
}

int main()
{
  bool isSuccess = true;

  std::vector< std::vector<double> > rgb_truth;
  rgb_truth.push_back({ 0, 0, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 255, 255, 255, vpRGBa::alpha_default });
  rgb_truth.push_back({ 255, 0, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 0, 255, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 0, 0, 255, vpRGBa::alpha_default });
  rgb_truth.push_back({ 255, 255, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 0, 255, 255, vpRGBa::alpha_default });
  rgb_truth.push_back({ 255, 0, 255, vpRGBa::alpha_default });
  rgb_truth.push_back({ 128, 128, 128, vpRGBa::alpha_default });
  rgb_truth.push_back({ 128, 128, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 128, 0, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 0, 128, 0, vpRGBa::alpha_default });
  rgb_truth.push_back({ 0, 128, 128, vpRGBa::alpha_default });
  rgb_truth.push_back({ 0, 0, 128, vpRGBa::alpha_default });
  rgb_truth.push_back({ 128, 0, 128, vpRGBa::alpha_default });

  double h_max;
  bool h_full;
  size_t size = rgb_truth.size();

  for (size_t test = 0; test < 2; ++test) {
    if (test == 0) {
      h_max = 255;
      h_full = true;
    }
    else {
      h_max = vpHSV<unsigned char, false>::maxHueUsingLimitedRange;
      h_full = false;
    }

    // See https://www.rapidtables.com/convert/color/hsv-to-rgb.html
    std::vector< std::vector<double> > hsv_truth;
    hsv_truth.push_back({ 0., 0., 0. });
    hsv_truth.push_back({ 0., 0., 255. });
    hsv_truth.push_back({ 0., 255., 255. });
    hsv_truth.push_back({ h_max * 120. / 360., 255., 255. });
    hsv_truth.push_back({ h_max * 240. / 360., 255., 255. });
    hsv_truth.push_back({ h_max * 60. / 360., 255., 255. });
    hsv_truth.push_back({ h_max * 180. / 360., 255., 255. });
    hsv_truth.push_back({ h_max * 300. / 360., 255., 255. });
    hsv_truth.push_back({ 0., 0., 128. });
    hsv_truth.push_back({ h_max * 60. / 360., 255., 128. });
    hsv_truth.push_back({ 0., 255., 128. });
    hsv_truth.push_back({ h_max * 120. / 360., 255., 128. });
    hsv_truth.push_back({ h_max * 180. / 360., 255., 128. });
    hsv_truth.push_back({ h_max * 240. / 360., 255., 128. });
    hsv_truth.push_back({ h_max * 300. / 360., 255., 128. });

    // SECTION("RGB -> HSV (unsigned char) -> RGB")
    std::cout << std::endl << "----- Test rgba -> hsv (unsigned char) conversion with h full scale: " << (h_full ? "yes" : "no") << " -----" << std::endl;
    for (unsigned int id = 0; id < size; ++id) {
      vpRGBa rgba(rgb_truth[id]);
      if (h_full) {
        vpHSV<unsigned char, true> hsv(static_cast<unsigned char>(0), static_cast<unsigned char>(0), static_cast<unsigned char>(0));
        hsv.buildFrom(rgba);
        isSuccess = isSuccess && test_hsv(hsv, rgba, vpHSV<unsigned char, true>(hsv_truth[id]));
      }
      else {
        vpHSV<unsigned char, false> hsv(static_cast<unsigned char>(0), static_cast<unsigned char>(0), static_cast<unsigned char>(0));
        hsv.buildFrom(rgba);
        isSuccess = isSuccess && test_hsv(hsv, rgba, vpHSV<unsigned char, false>(hsv_truth[id]));
      }

    }

    if (h_full) {
      // SECTION("RGB -> HSV (double) -> RGB")
      std::cout << std::endl << "----- Test rgb -> hsv (double) conversion -----" << std::endl;
      for (unsigned int id = 0; id < size; ++id) {
        vpRGBa rgba(rgb_truth[id]);
        vpHSV<double> hsv(rgba);
        for (unsigned char c = 0; c < vpHSV<double>::nbChannels; ++c) {
          hsv_truth[id][c] = hsv_truth[id][c] / 255.;
        }
        isSuccess = isSuccess && test_hsv(hsv, rgba, vpHSV<double>(hsv_truth[id]));
      }
    }
  }

  std::cout << std::endl << "----- Testing vpImageConvert::convert(vpImage<vpRGBa>, vpImage<vpHSV>) conversions -----" << std::endl;
  vpImage<vpRGBa> Irgb(480, 640, vpRGBa(128, 128, 128));
  vpImage<vpHSV<unsigned char, false>> Ihsvucf_truth(480, 640, vpHSV<unsigned char, false>(0, 0, 128)), Ihsvucf;
  vpImage<vpHSV<unsigned char, true>> Ihsvuct_truth(480, 640, vpHSV<unsigned char, true>(0, 0, 128)), Ihsvuct;
  vpImage<vpHSV<double>> Ihsvd_truth(480, 640, vpHSV<double>(0., 0., 0.5)), Ihsvd;
  vpImageConvert::convert(Irgb, Ihsvucf);
  vpImageConvert::convert(Irgb, Ihsvuct);
  vpImageConvert::convert(Irgb, Ihsvd);

  bool localSuccess = vpHSVTests::areAlmostEqual(Ihsvucf, Ihsvucf_truth);
  if (!localSuccess) {
    std::cerr << "vpImageConvert(rgba, hsv<uchar, false>) failed!" << std::endl;
  }
  isSuccess = isSuccess && localSuccess;

  localSuccess = vpHSVTests::areAlmostEqual(Ihsvuct, Ihsvuct_truth);
  if (!localSuccess) {
    std::cerr << "vpImageConvert(rgba, hsv<uchar, true>) failed!" << std::endl;
  }
  isSuccess = isSuccess && localSuccess;

  localSuccess = vpHSVTests::areAlmostEqual(Ihsvd, Ihsvd_truth);
  if (!localSuccess) {
    std::cerr << "vpImageConvert(rgba, hsv<double>) failed!" << std::endl;
  }
  isSuccess = isSuccess && localSuccess;

  if (isSuccess) {
    std::cout << "All tests were successful !" << std::endl;
    return EXIT_SUCCESS;
  }
  std::cerr << "ERROR: Something went wrong !" << std::endl;
  return EXIT_FAILURE;
}
#else
int main()
{
  std::cout << "vpHSV class is not available, please use CXX 11 standard" << std::endl;
  return EXIT_SUCCESS;
}
#endif
