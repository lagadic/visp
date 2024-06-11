//! \example tutorial-hsv-range-tuner.cpp

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(HAVE_OPENCV_HIGHGUI) && defined(VISP_HAVE_X11)
#include <vector>

#include <opencv2/highgui.hpp>

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>

std::vector<int> hsv_values_trackbar(6);
const cv::String window_detection_name = "Object Detection";

void set_trackbar_H_min(int val)
{
  cv::setTrackbarPos("Low H", window_detection_name, val);
}
void set_trackbar_H_max(int val)
{
  cv::setTrackbarPos("High H", window_detection_name, val);
}
void set_trackbar_S_min(int val)
{
  cv::setTrackbarPos("Low S", window_detection_name, val);
}
void set_trackbar_S_max(int val)
{
  cv::setTrackbarPos("High S", window_detection_name, val);
}
void set_trackbar_V_min(int val)
{
  cv::setTrackbarPos("Low V", window_detection_name, val);
}
void set_trackbar_V_max(int val)
{
  cv::setTrackbarPos("High V", window_detection_name, val);
}
static void on_low_H_thresh_trackbar(int, void *)
{
  hsv_values_trackbar[0] = std::min(hsv_values_trackbar[1]-1, hsv_values_trackbar[0]);
  set_trackbar_H_min(hsv_values_trackbar[0]);
}
static void on_high_H_thresh_trackbar(int, void *)
{
  hsv_values_trackbar[1] = std::max(hsv_values_trackbar[1], hsv_values_trackbar[0]+1);
  set_trackbar_H_max(hsv_values_trackbar[1]);
}
static void on_low_S_thresh_trackbar(int, void *)
{
  hsv_values_trackbar[2] = std::min(hsv_values_trackbar[3]-1, hsv_values_trackbar[2]);
  set_trackbar_S_min(hsv_values_trackbar[2]);
}
static void on_high_S_thresh_trackbar(int, void *)
{
  hsv_values_trackbar[3] = std::max(hsv_values_trackbar[3], hsv_values_trackbar[2]+1);
  set_trackbar_S_max(hsv_values_trackbar[3]);
}
static void on_low_V_thresh_trackbar(int, void *)
{
  hsv_values_trackbar[4] = std::min(hsv_values_trackbar[5]-1, hsv_values_trackbar[4]);
  set_trackbar_V_min(hsv_values_trackbar[4]);
}
static void on_high_V_thresh_trackbar(int, void *)
{
  hsv_values_trackbar[5] = std::max(hsv_values_trackbar[5], hsv_values_trackbar[4]+1);
  set_trackbar_V_max(hsv_values_trackbar[5]);
}

int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  bool opt_save_img = false;
  std::string opt_hsv_filename = "calib/hsv-thresholds.yml";
  std::string opt_img_filename;
  bool show_helper = false;
  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--hsv-thresholds") && ((i+1) < argc)) {
      opt_hsv_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--image") {
      if ((i+1) < argc) {
        opt_img_filename = std::string(argv[++i]);
      }
      else {
        show_helper = true;
        std::cout << "ERROR \nMissing input image name after parameter " << std::string(argv[i]) << std::endl;
      }
    }
    else if (std::string(argv[i]) == "--save-img") {
      opt_save_img = true;
    }
    if (show_helper || std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--image <input image>]"
        << " [--hsv-thresholds <output filename.yml>]"
        << " [--save-img]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --image <input image>" << std::endl
        << "    Name of the input image filename." << std::endl
        << "    When this option is not set, we use librealsense to stream images from a Realsense camera. " << std::endl
        << "    Example: --image ballons.jpg" << std::endl
        << std::endl
        << "  --hsv-thresholds <output filename.yml>" << std::endl
        << "    Name of the output filename with yaml extension that will contain HSV low/high thresholds." << std::endl
        << "    Default: " << opt_hsv_filename << std::endl
        << std::endl
        << "  --save-img" << std::endl
        << "    Enable RGB, HSV and segmented image saving" << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  bool use_realsense = false;
#if defined(VISP_HAVE_REALSENSE2)
  use_realsense = true;
#endif
  if (use_realsense) {
    if (!opt_img_filename.empty()) {
      use_realsense = false;
    }
  }
  else if (opt_img_filename.empty()) {
    std::cout << "Error: you should use --image <input image> option to specify an input image..." << std::endl;
    return EXIT_FAILURE;
  }

  int max_value_H = 255;
  int max_value = 255;

  hsv_values_trackbar[0] = 0;           // Low H
  hsv_values_trackbar[1] = max_value_H; // High H
  hsv_values_trackbar[2] = 0;           // Low S
  hsv_values_trackbar[3] = max_value;   // High S
  hsv_values_trackbar[4] = 0;           // Low V
  hsv_values_trackbar[5] = max_value;   // High V

  vpImage<vpRGBa> I;
  int width, height;

#if defined(VISP_HAVE_REALSENSE2)
  vpRealSense2 rs;
#endif

  if (use_realsense) {
#if defined(VISP_HAVE_REALSENSE2)
    width = 848; height = 480;
    int fps = 60;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED, 1);
    config.disable_stream(RS2_STREAM_INFRARED, 2);
    rs.open(config);
    rs.acquire(I);
#endif
  }
  else {
    try {
      vpImageIo::read(I, opt_img_filename);
    }
    catch (const vpException &e) {
      std::cout << e.getStringMessage() << std::endl;
      return EXIT_FAILURE;
    }
    width = I.getWidth();
    height = I.getHeight();
  }

  cv::namedWindow(window_detection_name);

  vpArray2D<int> hsv_values(hsv_values_trackbar), hsv_values_prev;
  if (vpArray2D<int>::loadYAML(opt_hsv_filename, hsv_values)) {
    std::cout << "Load hsv values from " << opt_hsv_filename << " previous tuning " << std::endl;
    std::cout << hsv_values.t() << std::endl;
    hsv_values_prev = hsv_values;
    for (size_t i = 0; i < hsv_values.size(); ++i) {
      hsv_values_trackbar[i] = hsv_values.data[i];
    }
  }

  // Trackbars to set thresholds for HSV values
  cv::createTrackbar("Low H", window_detection_name, &hsv_values_trackbar[0], max_value_H, on_low_H_thresh_trackbar);
  cv::createTrackbar("High H", window_detection_name, &hsv_values_trackbar[1], max_value_H, on_high_H_thresh_trackbar);
  cv::createTrackbar("Low S", window_detection_name, &hsv_values_trackbar[2], max_value, on_low_S_thresh_trackbar);
  cv::createTrackbar("High S", window_detection_name, &hsv_values_trackbar[3], max_value, on_high_S_thresh_trackbar);
  cv::createTrackbar("Low V", window_detection_name, &hsv_values_trackbar[4], max_value, on_low_V_thresh_trackbar);
  cv::createTrackbar("High V", window_detection_name, &hsv_values_trackbar[5], max_value, on_high_V_thresh_trackbar);

  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);
  vpImage<unsigned char> mask(height, width);
  vpImage<vpRGBa> I_segmented(height, width);

  vpDisplayX d_I(I, 0, 0, "Current frame");
  vpDisplayX d_I_segmented(I_segmented, I.getWidth()+75, 0, "Segmented frame");
  bool quit = false;

  while (!quit) {
    if (use_realsense) {
#if defined(VISP_HAVE_REALSENSE2)
      rs.acquire(I);
#endif
    }
    else {
      vpImageIo::read(I, opt_img_filename);
    }
    vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap),
                              reinterpret_cast<unsigned char *>(H.bitmap),
                              reinterpret_cast<unsigned char *>(S.bitmap),
                              reinterpret_cast<unsigned char *>(V.bitmap), I.getSize());

    vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                                                reinterpret_cast<unsigned char *>(S.bitmap),
                                                reinterpret_cast<unsigned char *>(V.bitmap),
                                                hsv_values_trackbar,
                                                reinterpret_cast<unsigned char *>(mask.bitmap),
                                                mask.getSize());

    vpImageTools::inMask(I, mask, I_segmented);

    vpDisplay::display(I);
    vpDisplay::display(I_segmented);
    vpDisplay::displayText(I, 20, 20, "Left click to learn HSV value...", vpColor::red);
    vpDisplay::displayText(I, 40, 20, "Middle click to get HSV value...", vpColor::red);
    vpDisplay::displayText(I, 60, 20, "Right click to quit...", vpColor::red);
    vpImagePoint ip;
    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, ip, button, false)) {
      if (button == vpMouseButton::button3) {
        quit = true;
      }
      else if (button == vpMouseButton::button2) {
        unsigned int i = ip.get_i();
        unsigned int j = ip.get_j();
        int h = static_cast<int>(H[i][j]);
        int s = static_cast<int>(S[i][j]);
        int v = static_cast<int>(V[i][j]);
        std::cout << "RGB[" << i << "][" << j << "]: " << static_cast<int>(I[i][j].R) << " " << static_cast<int>(I[i][j].G)
          << " " << static_cast<int>(I[i][j].B) << " -> HSV: " << h << " " << s << " " << v << std::endl;
      }
      else if (button == vpMouseButton::button1) {
        unsigned int i = ip.get_i();
        unsigned int j = ip.get_j();
        int h = static_cast<int>(H[i][j]);
        int s = static_cast<int>(S[i][j]);
        int v = static_cast<int>(V[i][j]);
        int offset = 30;
        hsv_values_trackbar[0] = std::max(0, h - offset);
        hsv_values_trackbar[1] = std::min(max_value_H, h + offset);
        hsv_values_trackbar[2] = std::max(0, s - offset);
        hsv_values_trackbar[3] = std::min(max_value, s + offset);
        hsv_values_trackbar[4] = std::max(0, v - offset);
        hsv_values_trackbar[5] = std::min(max_value, v + offset);
        std::cout << "HSV learned: " << h << " " << s << " " << v << std::endl;
        set_trackbar_H_min(hsv_values_trackbar[0]);
        set_trackbar_H_max(hsv_values_trackbar[1]);
        set_trackbar_S_min(hsv_values_trackbar[2]);
        set_trackbar_S_max(hsv_values_trackbar[3]);
        set_trackbar_V_min(hsv_values_trackbar[4]);
        set_trackbar_V_max(hsv_values_trackbar[5]);
      }
    }
    if (quit) {
      std::string parent = vpIoTools::getParent(opt_hsv_filename);
      if (vpIoTools::checkDirectory(parent) == false) {
        std::cout << "Create directory: " << parent << std::endl;
        vpIoTools::makeDirectory(parent);
      }

      vpArray2D<int> hsv_values_new(hsv_values_trackbar);
      if (hsv_values_new != hsv_values_prev) {
        if (vpIoTools::checkFilename(opt_hsv_filename)) {
          std::string hsv_filename_backup(opt_hsv_filename + std::string(".previous"));
          std::cout << "Create a backup of the previous calibration in " << hsv_filename_backup << std::endl;
          vpIoTools::copy(opt_hsv_filename, hsv_filename_backup);
        }

        std::cout << "Save new calibration in: " << opt_hsv_filename << std::endl;
        std::string header = std::string("# File created ") + vpTime::getDateTime();
        vpArray2D<int>::saveYAML(opt_hsv_filename, hsv_values_new, header.c_str());
      }
      if (opt_save_img) {
        std::string path_img(parent + "/images-visp");
        if (vpIoTools::checkDirectory(path_img) == false) {
          std::cout << "Create directory: " << path_img << std::endl;
          vpIoTools::makeDirectory(path_img);
        }

        std::cout << "Save images in path_img folder..." << std::endl;
        vpImage<vpRGBa> I_HSV;
        vpImageConvert::merge(&H, &S, &V, nullptr, I_HSV);
        vpImageIo::write(I, path_img + "/I.png");
        vpImageIo::write(I_HSV, path_img + "/I-HSV.png");
        vpImageIo::write(I_segmented, path_img + "/I-HSV-segmented.png");
      }
      break;
    }
    vpDisplay::flush(I);
    vpDisplay::flush(I_segmented);
    cv::waitKey(10); // To display trackbar
  }
  return EXIT_SUCCESS;
}

#else
int main()
{
#if !defined(HAVE_OPENCV_HIGHGUI)
  std::cout << "This tutorial needs OpenCV highgui module as 3rd party." << std::endl;
#endif
#if !defined(VISP_HAVE_X11)
  std::cout << "This tutorial needs X11 3rd party enabled." << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and rebuild ViSP." << std::endl;
  return EXIT_SUCCESS;
}
#endif
