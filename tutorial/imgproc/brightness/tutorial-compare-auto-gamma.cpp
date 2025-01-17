//! \example tutorial-compare-auto-gamma.cpp

#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpFont.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>

// VISP_HAVE_SIMDLIB is required for INTERPOLATION_AREA
#if defined(VISP_HAVE_MODULE_IMGPROC) && defined(VISP_HAVE_SIMDLIB) && \
  ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L)))
#include <visp3/imgproc/vpImgproc.h>
#include <memory>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
template <class T>
void computeMeanMaxStdev(const vpImage<T> &I, float &mean, float &max, float &stdev)
{
  max = std::numeric_limits<float>::epsilon();
  mean = 0.;
  stdev = 0.;
  unsigned int nbRows = I.getRows();
  unsigned int nbCols = I.getCols();
  float scale = 1.f / (static_cast<float>(nbRows) * static_cast<float>(nbCols));
  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      mean += I[r][c];
      max = std::max<float>(max, static_cast<float>(I[r][c]));
    }
  }
  mean *= scale;
  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      stdev += (I[r][c] - mean) * (I[r][c] - mean);
    }
  }
  stdev *= scale;
  stdev = std::sqrt(stdev);
}

void computeCanny(const vpImage<unsigned char> &I, vpCannyEdgeDetection &cannyDetector, int gaussianKernelSize,
                  float gaussianStdev, int apertureSize, vpImageFilter::vpCannyFilteringAndGradientType filteringType,
                  vpImage<unsigned char> &dIxy_uchar, vpImage<unsigned char> &I_canny_visp)
{
  vpImage<float> dIx, dIy, dIxy(I.getHeight(), I.getWidth());
  vpImageFilter::computePartialDerivatives(I, dIx, dIy, true, true, true, gaussianKernelSize, gaussianStdev,
      apertureSize, filteringType);

  for (unsigned int i = 0; i < dIx.getHeight(); i++) {
    for (unsigned int j = 0; j < dIx.getWidth(); j++) {
      dIxy[i][j] = std::sqrt(dIx[i][j]*dIx[i][j] + dIy[i][j]*dIy[i][j]);
    }
  }

  float mean, max, stdev;
  computeMeanMaxStdev(dIxy, mean, max, stdev);
  vpImageConvert::convert(dIx, dIxy_uchar);

  // Set the gradients of the vpCannyEdgeDetection
  cannyDetector.setGradients(dIx, dIy);

  I_canny_visp = cannyDetector.detect(I);
}

double computeImageEntropy(const vpImage<unsigned char> &I)
{
  // https://github.com/dengyueyun666/Image-Contrast-Enhancement/blob/cd2b1eb5bf6396e2fc3b94cd27f73933d5467147/src/Ying_2017_CAIP.cpp#L186-L207
  std::vector<int> hist(256, 0);
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      int bin = I[i][j];
      hist[bin]++;
    }
  }

  double N = I.getSize();
  double cost = 0;
  for (size_t i = 0; i < hist.size(); i++) {
    if (hist[i] == 0) {
      continue;
    }
    double p = hist[i] / N;
    cost += -p * std::log2(p);
  }

  return cost;
}
} // namespace

int main(int argc, const char **argv)
{
  std::string input = "Sample_low_brightness.png";
  std::string output = "Results";
  int gaussianKernelSize = 3;
  float gaussianStdev = 1.0f;
  int apertureSize = 3;
  bool half = false;
  vpImageFilter::vpCannyFilteringAndGradientType filteringType = vpImageFilter::CANNY_GBLUR_SOBEL_FILTERING;
  VISP_NAMESPACE_NAME::vpGammaColorHandling gamma_colorspace = VISP_NAMESPACE_NAME::GAMMA_HSV;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      ++i;
      input = std::string(argv[i]);
    }
    else if (std::string(argv[i]) == "--half") {
      half = true;
    }
    else if (std::string(argv[i]) == "--gaussian-kernel-size" && i + 1 < argc) {
      ++i;
      gaussianKernelSize = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--gaussian-std" && i + 1 < argc) {
      ++i;
      gaussianStdev = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--aperture-size" && i + 1 < argc) {
      ++i;
      apertureSize = std::atoi(argv[i]);
    }
    else if (std::string(argv[i]) == "--canny-filtering-type" && i + 1 < argc) {
      ++i;
      int type = std::atoi(argv[i]);
      if (type == 1) {
        filteringType = vpImageFilter::CANNY_GBLUR_SCHARR_FILTERING;
      }
    }
    else if (std::string(argv[i]) == "--gamma-rgb") {
      gamma_colorspace = VISP_NAMESPACE_NAME::GAMMA_RGB;
    }
    else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      ++i;
      output = std::string(argv[i]);
    }
    else {
      std::cout << "Usage: " << argv[0]
        << " [--input <input path or image sequence pattern>]"
        " [--half (use half image resolution)]"
        " [--gaussian-kernel-size <e.g. 3, 5, 7>]"
        " [--gaussian-std <e.g. 1>]"
        " [--aperture-size <e.g. 3>]"
        " [--canny-filtering-type <0=CANNY_GBLUR_SOBEL_FILTERING, 1=CANNY_GBLUR_SCHARR_FILTERING>]"
        " [--gamma-rgb (RGB colorspace, else HSV]"
        " [--output <folder path> (to save results)]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Input: " << input << std::endl;
  std::cout << "Process on half image resolution? " << half << std::endl;
  std::cout << "Gaussian kernel size: " << gaussianKernelSize << std::endl;
  std::cout << "Gaussian standard deviation: " << gaussianStdev << std::endl;
  std::cout << "Aperture size: " << apertureSize << std::endl;
  std::cout << "Canny filtering type: " << filteringType << std::endl;
  std::cout << "RGB colorspace? " << (gamma_colorspace == VISP_NAMESPACE_NAME::GAMMA_RGB) << std::endl;
  std::cout << "Output result folder: " << output << std::endl;

  // Canny parameters
  float lowerThresh = -1.;
  float upperThresh = -1.;
  float lowerThreshRatio = 0.6f;
  float upperThreshRatio = 0.8f;
  vpCannyEdgeDetection cannyDetector(gaussianKernelSize, gaussianStdev, apertureSize,
                                    lowerThresh, upperThresh, lowerThreshRatio, upperThreshRatio,
                                    filteringType);

  bool single_image = vpIoTools::checkFilename(input);
  vpVideoReader reader;
  vpImage<vpRGBa> I_color_ori, I_color;
  if (single_image) {
    vpImageIo::read(I_color_ori, input);
  }
  else {
    reader.setFileName(input);
    reader.open(I_color_ori);
  }
  if (half) {
    vpImageTools::resize(I_color_ori, I_color, I_color_ori.getWidth()/2, I_color_ori.getHeight()/2,
      vpImageTools::INTERPOLATION_AREA);
  }
  else {
    I_color = I_color_ori;
  }

  vpIoTools::makeDirectory(output);

  const int nb_methods = VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT - 1; // all except GAMMA_MANUAL
  std::vector<std::vector<double>> computation_times(nb_methods);
  int nb_images = 0;

  vpImage<vpRGBa> I_color_gamma_correction, I_res_stack;
  vpImage<unsigned char> I_gray, I_gray_gamma_correction, dIxy_uchar, I_canny_visp;
  vpImage<vpRGBa> dIxy_uchar_color, I_canny_visp_color;
  vpFont font(32);
  bool read_single_image = false;
  while (!read_single_image && (single_image || !reader.end())) {
    if (!single_image) {
      reader.acquire(I_color_ori);
    }
    if (half) {
      vpImageTools::resize(I_color_ori, I_color, I_color_ori.getWidth()/2, I_color_ori.getHeight()/2,
        vpImageTools::INTERPOLATION_AREA);
    }
    else {
      I_color = I_color_ori;
    }
    nb_images++;

    const int nb_methods = VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT - 1; // all except GAMMA_MANUAL
    I_res_stack.init(nb_methods*I_color.getHeight(), 4*I_color.getWidth());
    dIxy_uchar.init(I_color.getHeight(), I_color.getWidth());
    I_canny_visp.init(I_color.getHeight(), I_color.getWidth());

    // Output results
    int offset_text_start_y = 25;
    int text_h = 40;
    int offset_idx = 0;
    double offset_text1 = 0.01;
    double offset_text2 = 0.26;
    double start_time = 0, end_time = 0;
    char buffer[FILENAME_MAX];

    vpImageConvert::convert(I_color, I_gray);
    const double img_ori_entropy = computeImageEntropy(I_gray);

    for (int gamma_idx = 1; gamma_idx < VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT; ++gamma_idx, offset_idx++) {
      VISP_NAMESPACE_NAME::vpGammaMethod gamma_method = static_cast<VISP_NAMESPACE_NAME::vpGammaMethod>(gamma_idx);
      if (gamma_method == VISP_NAMESPACE_NAME::GAMMA_MANUAL) {
        continue;
      }

      const double gamma = -1;
      start_time = vpTime::measureTimeMs();
      VISP_NAMESPACE_NAME::gammaCorrection(I_color, I_color_gamma_correction, static_cast<float>(gamma),
        gamma_colorspace, gamma_method);
      end_time = vpTime::measureTimeMs();
      std::cout << "Computation time (" << VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method)
        << "): " << (end_time-start_time) << " ms" << std::endl;
      computation_times[offset_idx].push_back(end_time-start_time);

      vpImageConvert::convert(I_color_gamma_correction, I_gray_gamma_correction);
      const double img_corrected_entropy = computeImageEntropy(I_gray_gamma_correction);
      computeCanny(I_gray_gamma_correction, cannyDetector, gaussianKernelSize, gaussianStdev, apertureSize,
        filteringType, dIxy_uchar, I_canny_visp);
      vpImageConvert::convert(dIxy_uchar, dIxy_uchar_color);
      vpImageConvert::convert(I_canny_visp, I_canny_visp_color);
      I_res_stack.insert(I_color, vpImagePoint(offset_idx*I_color.getHeight(), 0));
      I_res_stack.insert(I_color_gamma_correction, vpImagePoint(offset_idx*I_color.getHeight(), I_color.getWidth()));
      I_res_stack.insert(I_canny_visp_color, vpImagePoint(offset_idx*I_color.getHeight(), 2*I_color.getWidth()));
      I_res_stack.insert(dIxy_uchar_color, vpImagePoint(offset_idx*I_color.getHeight(), 3*I_color.getWidth()));
      // Entropy original
      snprintf(buffer, FILENAME_MAX, "Entropy: %.4f", img_ori_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y, offset_text1*I_res_stack.getWidth()), vpColor::red);
      // Computation time
      std::ostringstream oss;
      oss <<  VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method) << " (%.2f ms)";
      snprintf(buffer, FILENAME_MAX, oss.str().c_str(), (end_time-start_time));
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y,
                                                      offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Canny
      snprintf(buffer, FILENAME_MAX, "Canny mean: (%.2f)", I_canny_visp.getMeanValue());
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+text_h,
                                                      offset_text2*I_res_stack.getWidth()), vpColor::red);
      // Entropy
      snprintf(buffer, FILENAME_MAX, "Entropy: %.4f", img_corrected_entropy);
      font.drawText(I_res_stack, buffer, vpImagePoint(offset_idx*I_color.getHeight() + offset_text_start_y+2*text_h, offset_text2*I_res_stack.getWidth()), vpColor::red);
    }

    if (!output.empty()) {
      std::stringstream output_filename;
      const std::string extension = ".jpeg";
      if (single_image) {
        output_filename << vpIoTools::createFilePath(output, vpIoTools::getNameWE(input)) << extension;
      }
      else {
        output_filename << vpIoTools::createFilePath(output, vpIoTools::getNameWE(reader.getFrameName())) << extension;
      }
      std::cout << "Write result to: " << output_filename.str() << std::endl;
      vpImageIo::write(I_res_stack, output_filename.str());
    }

    if (single_image) {
      read_single_image = true;
    }
  }

  std::cout << "\nStats:" << std::endl;
  std::cout << "Nb images: " << nb_images << std::endl;

  for (int gamma_idx = 1; gamma_idx < VISP_NAMESPACE_NAME::GAMMA_METHOD_COUNT; ++gamma_idx) {
    VISP_NAMESPACE_NAME::vpGammaMethod gamma_method = static_cast<VISP_NAMESPACE_NAME::vpGammaMethod>(gamma_idx);
    if (gamma_method == VISP_NAMESPACE_NAME::GAMMA_MANUAL) {
      continue;
    }
    std::cout << VISP_NAMESPACE_NAME::vpGammaMethodToString(gamma_method) << ": mean="
      << vpMath::getMean(computation_times[gamma_idx-1]) << " ms ; median="
      << vpMath::getMedian(computation_times[gamma_idx-1]) << " ms" << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "C++11 is required." << std::endl;
  return EXIT_SUCCESS;
}
#endif
