#include <iostream>
#include <limits>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpHSV.h>

#include "hsvUtils.h"

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/**
 * \brief Check if the computed RGBa value corresponds to the ground-truth.
 *
 * \tparam Type The type of the vpHSV channels.
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; vpHSV<unsigned char, false>::maxHueUsingLimitedRange].
 * \param[in] rgb_computed Computed vpRGBa value.
 * \param[in] rgb_truth Ground-truth vpRGBa value.
 * \param[in] hsv_truth The HSV value that was used to compute rgb_computed.
 * \return true If rgb_computed and rgb_truth are equal.
 * \return false Otherwise
 */
template<typename Type, bool useFullScale >
bool test_rgb(const vpRGBa &rgb_computed, const vpRGBa &rgb_truth,
              const vpHSV<Type, useFullScale> &hsv_truth)
{
  // Compare RGB values
  if ((!vpMath::equal(rgb_computed.R, rgb_truth.R)) ||
      (!vpMath::equal(rgb_computed.G, rgb_truth.G)) ||
      (!vpMath::equal(rgb_computed.B, rgb_truth.B))) {

    std::cout << static_cast<int>(hsv_truth.H) << ","
      << static_cast<int>(hsv_truth.S) << ","
      << static_cast<int>(hsv_truth.V) << "): Expected RGB value: ("
      << static_cast<int>(rgb_truth.R) << ","
      << static_cast<int>(rgb_truth.G) << ","
      << static_cast<int>(rgb_truth.B) << ") converted value: ("
      << static_cast<int>(rgb_computed.R) << ","
      << static_cast<int>(rgb_computed.G) << ","
      << static_cast<int>(rgb_computed.B) << ")" << std::endl;
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

    // SECTION("HSV (unsigned char) -> RGB")
    std::cout << std::endl << "----- Test hsv (unsigned char) -> rgba conversion with h full scale: " << (h_full ? "yes" : "no") << " -----" << std::endl;
    for (unsigned int id = 0; id < size; ++id) {
      vpRGBa rgba_truth(rgb_truth[id]);
      if (h_full) {
        if (vpMath::round(hsv_truth[id][0] * 10.) % 10 == 0) { // To avoid the problem that 360 / 255 is not an integer
          std::cout << "Running the test for H = " << hsv_truth[id][0] << " ..."  << std::endl;
          vpHSV<unsigned char, true> hsv(static_cast<unsigned char>(hsv_truth[id][0]), static_cast<unsigned char>(hsv_truth[id][1]), static_cast<unsigned char>(hsv_truth[id][2]));
          vpRGBa rgba(hsv);
          isSuccess = isSuccess && test_rgb(rgba, rgba_truth, hsv);
        }
      }
      else {
        if (vpMath::round(hsv_truth[id][0] * 10.) % 10 == 0) { // To avoid the problem that 360 / 255 is not an integer
          std::cout << "Running the test for H = " << hsv_truth[id][0] << " ..."  << std::endl;
          vpHSV<unsigned char, false> hsv(static_cast<unsigned char>(hsv_truth[id][0]), static_cast<unsigned char>(hsv_truth[id][1]), static_cast<unsigned char>(hsv_truth[id][2]));
          vpRGBa rgba(hsv);
          isSuccess = isSuccess && test_rgb(rgba, rgba_truth, hsv);
        }
      }

    }

    if (h_full) {
      // SECTION("HSV (double) -> RGB")
      std::cout << std::endl << "----- Test hsv (double) -> rgba conversion -----" << std::endl;
      for (unsigned int id = 0; id < size; ++id) {
        vpRGBa rgba_truth(rgb_truth[id]);
        std::vector<double> hsv_vec = hsv_truth[id];
        std::cout << "Running the test for HSV = " << hsv_truth[id][0] << " ; " << hsv_truth[id][1] << "; " << hsv_truth[id][2] << " ..."  << std::endl;
        for (unsigned char c = 0; c < vpHSV<double>::nbChannels; ++c) {
          hsv_vec[c] = hsv_vec[c] / 255.;
        }
        vpHSV<double> hsv(hsv_vec);
        vpRGBa rgba(hsv);
        isSuccess = isSuccess && test_rgb(rgba, rgba_truth, hsv);
      }
    }
  }

  std::cout << std::endl << "----- Testing vpImageConvert::convert(vpImage<vpHSV>, vpImage<vpRGBa>) conversions -----" << std::endl;
  vpImage<vpRGBa> Irgb_truth(480, 640, vpRGBa(128, 128, 128)), Irgb;
  vpImage<vpHSV<unsigned char, false>> Ihsvucf_truth(480, 640, vpHSV<unsigned char, false>(0, 0, 128));
  vpImage<vpHSV<unsigned char, true>> Ihsvuct_truth(480, 640, vpHSV<unsigned char, true>(0, 0, 128));
  vpImage<vpHSV<double>> Ihsvd_truth(480, 640, vpHSV<double>(0., 0., 0.5));

  vpImageConvert::convert(Ihsvucf_truth, Irgb);
  bool localSuccess = vpHSVTests::areAlmostEqual(Irgb, Irgb_truth);
  if (!localSuccess) {
    std::cerr << "vpImageConvert(hsv<uchar, false>, rgba) failed!" << std::endl;
  }
  isSuccess = isSuccess && localSuccess;

  vpImageConvert::convert(Ihsvuct_truth, Irgb);
  localSuccess = vpHSVTests::areAlmostEqual(Irgb, Irgb_truth);
  if (!localSuccess) {
    std::cerr << "vpImageConvert(hsv<uchar, true>, rgba) failed!" << std::endl;
  }
  isSuccess = isSuccess && localSuccess;

  vpImageConvert::convert(Ihsvd_truth, Irgb);
  localSuccess = vpHSVTests::areAlmostEqual(Irgb, Irgb_truth);
  if (!localSuccess) {
    std::cerr << "vpImageConvert(hsv<double>, rgba) failed!" << std::endl;
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
