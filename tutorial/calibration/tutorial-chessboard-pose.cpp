//! \example tutorial-chessboard-pose.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020300

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/vision/vpPose.h>

namespace {
void calcChessboardCorners(int width, int height, double squareSize, std::vector<vpPoint> &corners) {
  corners.resize(0);

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      vpPoint pt;
      pt.set_oX(j*squareSize);
      pt.set_oY(i*squareSize);
      pt.set_oZ(0.0);
      corners.push_back(pt);
    }
  }
}
} //namespace

int main(int argc, const char ** argv) {
  int chessboard_width = 9, chessboard_height = 6;
  double chessboard_square_size = 0.03;
  std::string input_filename = "";
  std::string intrinsic_file = "camera.xml";
  std::string camera_name = "Camera";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-w" && i+1 < argc) {
      chessboard_width = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "-h" && i+1 < argc) {
      chessboard_height = atoi(argv[i+1]);
    } else if (std::string(argv[i]) == "--square_size" && i+1 < argc) {
      chessboard_square_size = atof(argv[i+1]);
    } else if (std::string(argv[i]) == "--input" && i+1 < argc) {
      input_filename = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i+1 < argc) {
      intrinsic_file = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--camera_name" && i+1 < argc) {
      camera_name = std::string(argv[i+1]);
    }
    else if (std::string(argv[i]) == "--help") {
      std::cout << argv[0] << " [-w <chessboard width>] [-w <chessboard height>] [--square_size <square size in meter>] [--input <input images path>] [--intrinsic <Camera intrinsic parameters xml file>] [--camera_name <Camera name in the xml intrinsic file>]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Parameters:" << std::endl;
  std::cout << "chessboard_width=" << chessboard_width << std::endl;
  std::cout << "chessboard_height=" << chessboard_height << std::endl;
  std::cout << "chessboard_square_size=" << chessboard_square_size << std::endl;
  std::cout << "input_filename=" << input_filename << std::endl;
  std::cout << "intrinsic_file=" << intrinsic_file << std::endl;
  std::cout << "camera_name=" << camera_name << std::endl;

  vpVideoReader reader;
  if (input_filename.empty()) {
    std::cerr << "input_filename.empty()" << std::endl;
    return EXIT_FAILURE;
  }
  reader.setFileName(input_filename);

  vpImage<vpRGBa> I;
  reader.open(I);

#ifdef VISP_HAVE_X11
  vpDisplayX d(I);
#elif defined VISP_HAVE_GDI
  vpDisplayGDI d(I);
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV d(I);
#endif

  std::vector<vpPoint> corners_pts;
  calcChessboardCorners(chessboard_width, chessboard_height, chessboard_square_size, corners_pts);

  vpCameraParameters cam;
#ifdef VISP_HAVE_PUGIXML
  vpXmlParserCamera parser;
  if (!intrinsic_file.empty() && !camera_name.empty()) {
    parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithDistortion);
  }
#endif
  std::cout << "cam:\n" << cam << std::endl;

  bool quit = false;
  while (!quit && !reader.end()) {
    reader.acquire(I);

    cv::Mat matImg;
    vpImageConvert::convert(I, matImg);

    vpDisplay::displayText(I, 20, 20, "Right click to quit.", vpColor::red);

    cv::Size chessboardSize(chessboard_width, chessboard_height);
    std::vector<cv::Point2f> corners2D;
    bool found = cv::findChessboardCorners(matImg, chessboardSize, corners2D,
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
                                   cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
#else
                                   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
#endif

    vpHomogeneousMatrix cMo;
    if (found) {
      cv::Mat matImg_gray;
      cv::cvtColor(matImg, matImg_gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(matImg_gray, corners2D, cv::Size(11,11),
                    cv::Size(-1,-1),
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
                    cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
#else
                    cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
#endif

      for (size_t i = 0; i < corners_pts.size(); i++) {
        vpImagePoint imPt(corners2D[i].y, corners2D[i].x);
        double x = 0.0, y = 0.0;
        vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
        corners_pts[i].set_x(x);
        corners_pts[i].set_y(y);
      }

      vpPose pose;
      pose.addPoints(corners_pts);
      vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
      double r_dementhon = std::numeric_limits<double>::max(), r_lagrange = std::numeric_limits<double>::max();
      bool pose_dementhon = pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
      if (pose_dementhon)
        r_dementhon = pose.computeResidual(cMo_dementhon);

      bool pose_lagrange = pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
      if (pose_lagrange)
        r_lagrange = pose.computeResidual(cMo_lagrange);

      cMo = (r_dementhon < r_lagrange) ? cMo_dementhon : cMo_lagrange;
      if (!pose.computePose(vpPose::VIRTUAL_VS, cMo)) {
        std::cerr << "Problem when computing final pose using VVS" << std::endl;
        return EXIT_FAILURE;
      }

      cv::drawChessboardCorners(matImg, chessboardSize, corners2D, found);
      vpImageConvert::convert(matImg, I);
    }

    vpDisplay::display(I);

    vpDisplay::displayText(I, 20, 20, "Left click for the next image, right click to quit.", vpColor::red);
    if (found)
      vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);

    vpDisplay::flush(I);

    if (found) {
      vpPoseVector pose_vec(cMo);
      std::stringstream ss;
      ss << "pose_cPo_" << reader.getFrameIndex() << ".yaml";
      std::cout << "Save " << ss.str() << std::endl;
      pose_vec.saveYAML(ss.str(), pose_vec);
    }

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(I, button, true)) {
      switch (button) {
        case vpMouseButton::button3:
          quit = true;
          break;

        default:
          break;
      }
    }
  }

  return EXIT_SUCCESS;
}
#else
int main() {
  std::cerr << "OpenCV 2.3.0 or higher is requested to run the calibration." << std::endl;
  return EXIT_SUCCESS;
}
#endif

