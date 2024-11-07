/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Camera calibration with chessboard or circle calibration grid.
 *
*****************************************************************************/
#include <iostream>

#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_HIGHGUI) && \
  defined(HAVE_OPENCV_IMGPROC) && defined(VISP_HAVE_PUGIXML)

#include <map>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp3/vision/vpCalibration.h>

#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>

#include "calibration-helper.hpp"

using namespace calib_helper;

void usage(const char *argv[], int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0] << " <configuration file>.cfg [--init-from-xml <camera-init.xml>]"
    << " [--camera-name <name>] [--aspect-ratio <ratio>] [--output <file.xml>] [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  <configuration file>.cfg  Configuration file. See example in" << std::endl
    << "    \"default-chessboard.cfg\" or in \"default-circles.cfg\"." << std::endl
    << "    Default: \"default.cfg\"." << std::endl
    << std::endl
    << "  --init-from-xml <camera-init.xml>  XML file that contains camera parameters" << std::endl
    << "    used to initialize the calibration process." << std::endl
    << std::endl
    << "  --camera-name <name>  Camera name in the XML file set using \"--init-from-xml\" option." << std::endl
    << "    Default: \"Camera\"." << std::endl
    << std::endl
    << "  --aspect-ratio <ratio>  Pixel aspect ratio. " << std::endl
    << "    To estimate px = py, use \"--aspect-ratio 1\" option. Set to -1" << std::endl
    << "    to unset any constraint for px and py parameters. " << std::endl
    << "    Default: -1." << std::endl
    << std::endl
    << "  --output <file.xml>  XML file containing estimated camera parameters." << std::endl
    << "    Default: \"camera.xml\"." << std::endl
    << std::endl
    << "  --help, -h  Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char *argv[])
{
#if defined(ENABLE_VISP_NAMESPACE)
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    if (argc == 1) {
      usage(argv, 0);
      return EXIT_SUCCESS;
    }
    std::string opt_output_file_name = "camera.xml";
    Settings s;
    const std::string opt_inputSettingsFile = argc > 1 ? argv[1] : "default.cfg";
    std::string opt_init_camera_xml_file;
    std::string opt_camera_name = "Camera";
    double opt_aspect_ratio = -1; // Not used

    for (int i = 2; i < argc; i++) {
      if (std::string(argv[i]) == "--init-from-xml" && i + 1 < argc) {
        opt_init_camera_xml_file = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
        opt_camera_name = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
        opt_output_file_name = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--aspect-ratio" && i + 1 < argc) {
        opt_aspect_ratio = std::atof(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    std::cout << "Settings from config file: " << argv[1] << std::endl;
    if (!s.read(opt_inputSettingsFile)) {
      std::cout << "Could not open the configuration file: \"" << opt_inputSettingsFile << "\"" << std::endl;
      usage(argv, 0);
      return EXIT_FAILURE;
    }

    if (!s.goodInput) {
      std::cout << "Invalid input detected. Application stopping. " << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "\nSettings from command line options: " << std::endl;
    if (!opt_init_camera_xml_file.empty()) {
      std::cout << "Init parameters: " << opt_init_camera_xml_file << std::endl;
    }
    std::cout << "Ouput xml file : " << opt_output_file_name << std::endl;
    std::cout << "Camera name    : " << opt_camera_name << std::endl;

    // Check if output file name exists
    if (vpIoTools::checkFilename(opt_output_file_name)) {
      std::cout << "\nOutput file name " << opt_output_file_name << " already exists." << std::endl;
      std::cout << "Remove this file or change output file name using [--output <file.xml>] command line option."
        << std::endl;
      return EXIT_SUCCESS;
    }

    // Start the calibration code
    vpImage<unsigned char> I;
    vpVideoReader reader;
    reader.setFileName(s.input);
    try {
      reader.open(I);
    }
    catch (const vpException &e) {
      std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
      std::cout << "Check if input images name \"" << s.input << "\" set in " << opt_inputSettingsFile
        << " config file is correct..." << std::endl;
      return EXIT_FAILURE;
    }

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
#endif

    vpCameraParameters cam_init;
    bool init_from_xml = false;
    if (!opt_init_camera_xml_file.empty()) {
      if (!vpIoTools::checkFilename(opt_init_camera_xml_file)) {
        std::cout << "Input camera file \"" << opt_init_camera_xml_file << "\" doesn't exist!" << std::endl;
        std::cout << "Modify [--init-from-xml <camera-init.xml>] option value" << std::endl;
        return EXIT_FAILURE;
      }
      init_from_xml = true;
    }
    if (init_from_xml) {
      std::cout << "Initialize camera parameters from xml file: " << opt_init_camera_xml_file << std::endl;
      vpXmlParserCamera parser;
      if (parser.parse(cam_init, opt_init_camera_xml_file, opt_camera_name,
                       vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Unable to find camera with name \"" << opt_camera_name
          << "\" in file: " << opt_init_camera_xml_file << std::endl;
        std::cout << "Modify [--camera-name <name>] option value" << std::endl;
        return EXIT_FAILURE;
      }
    }
    else {
      std::cout << "Initialize camera parameters with default values " << std::endl;
      // Initialize camera parameters
      double px = cam_init.get_px();
      double py = cam_init.get_py();
      // Set (u0,v0) in the middle of the image
      double u0 = I.getWidth() / 2;
      double v0 = I.getHeight() / 2;
      cam_init.initPersProjWithoutDistortion(px, py, u0, v0);
    }

    std::cout << "Camera parameters used for initialization:\n" << cam_init << std::endl;

    std::vector<vpPoint> model;
    std::vector<vpCalibration> calibrator;

    for (int i = 0; i < s.boardSize.height; i++) {
      for (int j = 0; j < s.boardSize.width; j++) {
        model.push_back(vpPoint(j * s.squareSize, i * s.squareSize, 0));
      }
    }

    std::vector<CalibInfo> calib_info;
    std::multimap<double, vpCameraParameters, std::less<double> > map_cam_sorted; // Sorted by residual

    map_cam_sorted.insert(std::make_pair(1000, cam_init));

    do {
      reader.acquire(I);
      long frame_index = reader.getFrameIndex();
      char filename[FILENAME_MAX];
      snprintf(filename, FILENAME_MAX, s.input.c_str(), frame_index);
      std::string frame_name = vpIoTools::getName(filename);
      vpDisplay::display(I);
      vpDisplay::flush(I);

      cv::Mat cvI;
      std::vector<cv::Point2f> pointBuf;
      vpImageConvert::convert(I, cvI);

      std::cout << "Process frame: " << frame_name << std::flush;
      bool found = extractCalibrationPoints(s, cvI, pointBuf);

      std::cout << ", grid detection status: " << found;
      if (!found)
        std::cout << ", image rejected" << std::endl;
      else
        std::cout << ", image used as input data" << std::endl;

      if (found) { // If image processing done with success
        vpDisplay::setTitle(I, frame_name);

        std::vector<vpImagePoint> data;
        for (unsigned int i = 0; i < pointBuf.size(); i++) {
          vpImagePoint ip(pointBuf[i].y, pointBuf[i].x);
          data.push_back(ip);
          vpDisplay::displayCross(I, ip, 10 * vpDisplay::getDownScalingFactor(I), vpColor::red);
        }

        // Calibration on a single mono image
        std::vector<vpPoint> calib_points;
        vpCalibration calib;
        calib.setLambda(0.5);
        calib.setAspectRatio(opt_aspect_ratio);
        for (unsigned int i = 0; i < model.size(); i++) {
          calib.addPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ(), data[i]);
          calib_points.push_back(vpPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ()));
          calib_points.back().set_x(data[i].get_u());
          calib_points.back().set_y(data[i].get_v());
        }

        vpHomogeneousMatrix cMo;
        bool calib_status = false;
        std::multimap<double, vpCameraParameters>::const_iterator it_cam;
        for (it_cam = map_cam_sorted.begin(); it_cam != map_cam_sorted.end(); ++it_cam) {
          vpCameraParameters cam = it_cam->second;
          if (calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS, cMo, cam, false) == EXIT_SUCCESS) {
            calibrator.push_back(calib);
            // Add calibration info
            calib_info.push_back(CalibInfo(I, calib_points, data, frame_name));
            calib_status = true;
            double residual = calib.getResidual();
            map_cam_sorted.insert(std::make_pair(residual, cam));
            break;
          }
        }
        if (!calib_status) {
          std::cout << "frame: " << frame_name << ", unable to calibrate from single image, image rejected"
            << std::endl;
          found = false;
        }
      }

      if (found)
        vpDisplay::displayText(I, 15 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "Image processing succeed", vpColor::green);
      else
        vpDisplay::displayText(I, 15 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "Image processing failed", vpColor::green);

      if (s.tempo > 10.f) {
        vpDisplay::displayText(I, 35 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "A click to process the next image", vpColor::green);
        vpDisplay::flush(I);
        vpDisplay::getClick(I);
      }
      else {
        vpDisplay::flush(I);
        vpTime::wait(s.tempo * 1000);
      }
    } while (!reader.end());

    // Now we consider the multi image calibration
    // Calibrate by a non linear method based on virtual visual servoing
    if (calibrator.empty()) {
      std::cerr << "Unable to calibrate. Image processing failed !" << std::endl;
      return EXIT_FAILURE;
    }

    // Display calibration pattern occupancy
    drawCalibrationOccupancy(I, calib_info, s.boardSize.width);

    cv::Mat1b img(I.getHeight(), I.getWidth());
    vpImageConvert::convert(I, img);
    cv::Mat3b imgColor(I.getHeight(), I.getWidth());
    cv::applyColorMap(img, imgColor, cv::COLORMAP_JET);

    // Draw calibration board corners
    for (size_t idx1 = 0; idx1 < calib_info.size(); idx1++) {
      const CalibInfo &calib = calib_info[idx1];

      for (size_t idx2 = 0; idx2 < calib.m_imPts.size(); idx2++) {
        const vpImagePoint &imPt = calib.m_imPts[idx2];
        cv::rectangle(imgColor,
                      cv::Rect(static_cast<int>(imPt.get_u() - 2), static_cast<int>(imPt.get_v() - 2),
                               4 * vpDisplay::getDownScalingFactor(I), 4 * vpDisplay::getDownScalingFactor(I)),
                      cv::Scalar(0, 0, 0), -1);
      }
    }

    vpImage<vpRGBa> I_color;
    vpImageConvert::convert(imgColor, I_color);
    d.close(I);
    d.init(I_color, 0, 0, "Calibration pattern occupancy");

    vpDisplay::display(I_color);
    vpDisplay::displayText(I_color, 15 * vpDisplay::getDownScalingFactor(I_color),
                           15 * vpDisplay::getDownScalingFactor(I_color), "Calibration pattern occupancy in the image",
                           vpColor::red);

    if (s.tempo > 10.f) {
      vpDisplay::displayText(I_color, I_color.getHeight() - 20 * vpDisplay::getDownScalingFactor(I_color),
                            15 * vpDisplay::getDownScalingFactor(I_color), "Click to continue...", vpColor::red);
      vpDisplay::flush(I_color);
      vpDisplay::getClick(I_color);
    }
    else {
      vpDisplay::flush(I_color);
      vpTime::wait(s.tempo * 1000);
    }

    d.close(I_color);
    d.init(I);

    std::stringstream ss_additional_info;
    ss_additional_info << "<date>" << vpTime::getDateTime() << "</date>";
    ss_additional_info << "<nb_calibration_images>" << calibrator.size() << "</nb_calibration_images>";
    ss_additional_info << "<calibration_pattern_type>";

    switch (s.calibrationPattern) {
    case Settings::CHESSBOARD:
      ss_additional_info << "Chessboard";
      break;

    case Settings::CIRCLES_GRID:
      ss_additional_info << "Circles grid";
      break;

    case Settings::UNDEFINED:
    default:
      ss_additional_info << "Undefined";
      break;
    }
    ss_additional_info << "</calibration_pattern_type>";
    ss_additional_info << "<board_size>" << s.boardSize.width << "x" << s.boardSize.height << "</board_size>";
    ss_additional_info << "<square_size>" << s.squareSize << "</square_size>";

    double error;
    // Initialize with camera parameter that has the lowest residual
    vpCameraParameters cam = map_cam_sorted.begin()->second;
    std::cout << "\nCalibration without distortion in progress on " << calibrator.size() << " images..." << std::endl;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS, calibrator, cam, error, false) ==
        EXIT_SUCCESS) {
      std::cout << cam << std::endl;
      vpDisplay::setTitle(I, "Without distortion results");

      for (size_t i = 0; i < calibrator.size(); i++) {
        double reproj_error = sqrt(calibrator[i].getResidual() / calibrator[i].get_npt());

        const CalibInfo &calib = calib_info[i];
        std::cout << "Image " << calib.m_frame_name << " reprojection error: " << reproj_error << std::endl;
        I = calib.m_img;
        vpDisplay::display(I);

        std::ostringstream ss;
        ss << "Reprojection error: " << reproj_error;
        vpDisplay::displayText(I, 15 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               calib.m_frame_name, vpColor::red);
        vpDisplay::displayText(I, 30 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               ss.str(), vpColor::red);
        vpDisplay::displayText(I, 45 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "Extracted points", vpColor::red);
        vpDisplay::displayText(I, 60 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "Projected points", vpColor::green);

        for (size_t idx = 0; idx < calib.m_points.size(); idx++) {
          vpDisplay::displayCross(I, calib.m_imPts[idx], 12 * vpDisplay::getDownScalingFactor(I), vpColor::red);

          vpPoint pt = calib.m_points[idx];
          pt.project(calibrator[i].cMo);
          vpImagePoint imPt;
          vpMeterPixelConversion::convertPoint(cam, pt.get_x(), pt.get_y(), imPt);
          vpDisplay::displayCross(I, imPt, 12 * vpDisplay::getDownScalingFactor(I), vpColor::green);
        }

        if (s.tempo > 10.f) {
          vpDisplay::displayText(I, I.getHeight() - 20 * vpDisplay::getDownScalingFactor(I),
                               15 * vpDisplay::getDownScalingFactor(I), "Click to continue...", vpColor::red);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
        else {
          vpDisplay::flush(I);
          vpTime::wait(s.tempo * 1000);
        }
      }

      std::cout << "\nGlobal reprojection error: " << error << std::endl;
      ss_additional_info << "<global_reprojection_error><without_distortion>" << error << "</without_distortion>";

      vpXmlParserCamera xml;

      if (xml.save(cam, opt_output_file_name.c_str(), opt_camera_name, I.getWidth(), I.getHeight()) ==
          vpXmlParserCamera::SEQUENCE_OK)
        std::cout << "Camera parameters without distortion successfully saved in \"" << opt_output_file_name << "\""
        << std::endl;
      else {
        std::cout << "Failed to save the camera parameters without distortion in \"" << opt_output_file_name << "\""
          << std::endl;
        std::cout << "A file with the same name exists. Remove it to be able "
          "to save the parameters..."
          << std::endl;
      }
    }
    else {
      std::cout << "Calibration without distortion failed." << std::endl;
      return EXIT_FAILURE;
    }
    vpCameraParameters cam_without_dist = cam;
    std::vector<vpCalibration> calibrator_without_dist = calibrator;

    std::cout << "\n\nCalibration with distortion in progress on " << calibrator.size() << " images..." << std::endl;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS_DIST, calibrator, cam, error, false) ==
        EXIT_SUCCESS) {
      std::cout << cam << std::endl;
      vpDisplay::setTitle(I, "With distortion results");

      for (size_t i = 0; i < calibrator.size(); i++) {
        double reproj_error = sqrt(calibrator[i].getResidual_dist() / calibrator[i].get_npt());

        const CalibInfo &calib = calib_info[i];
        std::cout << "Image " << calib.m_frame_name << " reprojection error: " << reproj_error << std::endl;
        I = calib.m_img;
        vpDisplay::display(I);

        std::ostringstream ss;
        ss << "Reprojection error: " << reproj_error;
        vpDisplay::displayText(I, 15 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               calib.m_frame_name, vpColor::red);
        vpDisplay::displayText(I, 30 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               ss.str(), vpColor::red);
        vpDisplay::displayText(I, 45 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "Extracted points", vpColor::red);
        vpDisplay::displayText(I, 60 * vpDisplay::getDownScalingFactor(I), 15 * vpDisplay::getDownScalingFactor(I),
                               "Projected points", vpColor::green);

        for (size_t idx = 0; idx < calib.m_points.size(); idx++) {
          vpDisplay::displayCross(I, calib.m_imPts[idx], 12 * vpDisplay::getDownScalingFactor(I), vpColor::red);

          vpPoint pt = calib.m_points[idx];
          pt.project(calibrator[i].cMo_dist);
          vpImagePoint imPt;
          vpMeterPixelConversion::convertPoint(cam, pt.get_x(), pt.get_y(), imPt);
          vpDisplay::displayCross(I, imPt, 12 * vpDisplay::getDownScalingFactor(I), vpColor::green);
        }

        if (s.tempo > 10.f) {
          vpDisplay::displayText(I, I.getHeight() - 20 * vpDisplay::getDownScalingFactor(I),
                               15 * vpDisplay::getDownScalingFactor(I), "Click to continue...", vpColor::red);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
        else {
          vpDisplay::flush(I);
          vpTime::wait(s.tempo * 1000);
        }
      }

      std::cout << "\nGlobal reprojection error: " << error << std::endl;
      ss_additional_info << "<with_distortion>" << error << "</with_distortion></global_reprojection_error>";

      vpImage<unsigned char> I_undist;
      vpImage<unsigned char> I_dist_undist(I.getHeight(), 2 * I.getWidth());
      d.close(I);
      d.init(I_dist_undist, 0, 0, "Straight lines have to be straight - distorted image / undistorted image");

      for (size_t idx = 0; idx < calib_info.size(); idx++) {
        std::cout << "\nThis tool computes the line fitting error (mean distance error) on image points extracted from "
          "the raw distorted image."
          << std::endl;

        I = calib_info[idx].m_img;
        vpImageTools::undistort(I, cam, I_undist);

        I_dist_undist.insert(I, vpImagePoint(0, 0));
        I_dist_undist.insert(I_undist, vpImagePoint(0, I.getWidth()));
        vpDisplay::display(I_dist_undist);

        vpDisplay::displayText(I_dist_undist, 15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                               15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                               calib_info[idx].m_frame_name + std::string(" distorted"), vpColor::red);
        vpDisplay::displayText(I_dist_undist, 30 * vpDisplay::getDownScalingFactor(I_dist_undist),
                               15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                               "Draw lines from first / last points.", vpColor::red);
        std::vector<vpImagePoint> grid_points = calib_info[idx].m_imPts;
        for (int i = 0; i < s.boardSize.height; i++) {
          std::vector<vpImagePoint> current_line(grid_points.begin() + i * s.boardSize.width,
                                                 grid_points.begin() + (i + 1) * s.boardSize.width);

          std::vector<vpImagePoint> current_line_undist = undistort(cam, current_line);
          double a = 0, b = 0, c = 0;
          double line_fitting_error = vpMath::lineFitting(current_line, a, b, c);
          double line_fitting_error_undist = vpMath::lineFitting(current_line_undist, a, b, c);
          std::cout << calib_info[idx].m_frame_name << " line " << i + 1
            << " fitting error on distorted points: " << line_fitting_error
            << " ; on undistorted points: " << line_fitting_error_undist << std::endl;

          vpImagePoint ip1 = current_line.front();
          vpImagePoint ip2 = current_line.back();
          vpDisplay::displayLine(I_dist_undist, ip1, ip2, vpColor::red);
        }

        std::cout << "\nThis tool computes the line fitting error (mean distance error) on image points extracted from "
          "the undistorted image"
          << " (vpImageTools::undistort())." << std::endl;
        cv::Mat cvI;
        std::vector<cv::Point2f> pointBuf;
        vpImageConvert::convert(I_undist, cvI);

        bool found = extractCalibrationPoints(s, cvI, pointBuf);
        if (found) {
          std::vector<vpImagePoint> grid_points;
          for (unsigned int i = 0; i < pointBuf.size(); i++) {
            vpImagePoint ip(pointBuf[i].y, pointBuf[i].x);
            grid_points.push_back(ip);
          }

          vpDisplay::displayText(I_dist_undist, 15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                                 I.getWidth() + 15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                                 calib_info[idx].m_frame_name + std::string(" undistorted"), vpColor::red);
          for (int i = 0; i < s.boardSize.height; i++) {
            std::vector<vpImagePoint> current_line(grid_points.begin() + i * s.boardSize.width,
                                                   grid_points.begin() + (i + 1) * s.boardSize.width);

            double a = 0, b = 0, c = 0;
            double line_fitting_error = vpMath::lineFitting(current_line, a, b, c);
            std::cout << calib_info[idx].m_frame_name << " undistorted image, line " << i + 1
              << " fitting error: " << line_fitting_error << std::endl;

            vpImagePoint ip1 = current_line.front() + vpImagePoint(0, I.getWidth());
            vpImagePoint ip2 = current_line.back() + vpImagePoint(0, I.getWidth());
            vpDisplay::displayLine(I_dist_undist, ip1, ip2, vpColor::red);
          }
        }
        else {
          std::string msg("Unable to detect grid on undistorted image");
          std::cout << msg << std::endl;
          std::cout << "Check that the grid is not too close to the image edges" << std::endl;
          vpDisplay::displayText(I_dist_undist, 15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                                 15 * vpDisplay::getDownScalingFactor(I_dist_undist),
                                 calib_info[idx].m_frame_name + std::string(" undistorted"), vpColor::red);
          vpDisplay::displayText(I_dist_undist, 30 * vpDisplay::getDownScalingFactor(I_dist_undist),
                                 15 * vpDisplay::getDownScalingFactor(I_dist_undist), msg, vpColor::red);
        }

        if (s.tempo > 10.f) {
          vpDisplay::displayText(
              I_dist_undist, I_dist_undist.getHeight() - 20 * vpDisplay::getDownScalingFactor(I_dist_undist),
              15 * vpDisplay::getDownScalingFactor(I_dist_undist), "Click to continue...", vpColor::red);
          vpDisplay::flush(I_dist_undist);
          vpDisplay::getClick(I_dist_undist);
        }
        else {
          vpDisplay::flush(I_dist_undist);
          vpTime::wait(s.tempo * 1000);
        }
      }

      std::cout << std::endl;
      vpXmlParserCamera xml;

      // Camera poses
      ss_additional_info << "<camera_poses>";
      for (size_t i = 0; i < calibrator.size(); i++) {
        vpPoseVector pose(calibrator[i].cMo);
        ss_additional_info << "<cMo>" << pose.t() << "</cMo>";
      }
      for (size_t i = 0; i < calibrator.size(); i++) {
        vpPoseVector pose(calibrator[i].cMo_dist);
        ss_additional_info << "<cMo_dist>" << pose.t() << "</cMo_dist>";
      }
      ss_additional_info << "</camera_poses>";

      if (xml.save(cam, opt_output_file_name.c_str(), opt_camera_name, I.getWidth(), I.getHeight(),
                   ss_additional_info.str()) == vpXmlParserCamera::SEQUENCE_OK)
        std::cout << "Camera parameters without distortion successfully saved in \"" << opt_output_file_name << "\""
        << std::endl;
      else {
        std::cout << "Failed to save the camera parameters without distortion in \"" << opt_output_file_name << "\""
          << std::endl;
        std::cout << "A file with the same name exists. Remove it to be able "
          "to save the parameters..."
          << std::endl;
      }
      std::cout << std::endl;
      std::cout << "Estimated pose using vpPoseVector format: [tx ty tz tux tuy tuz] with translation in meter and "
        "rotation in rad"
        << std::endl;
      for (unsigned int i = 0; i < calibrator.size(); i++)
        std::cout << "Estimated pose on input data extracted from " << calib_info[i].m_frame_name << ": "
        << vpPoseVector(calibrator[i].cMo_dist).t() << std::endl;
    }
    else {
      std::cout << "Calibration with distortion failed." << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "\nCamera calibration succeeded. Results are savec in " << "\"" << opt_output_file_name << "\"" << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
#if !((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_CALIB3D) && defined(HAVE_OPENCV_HIGHGUI) &&  defined(HAVE_OPENCV_IMGPROC))
  std::cout << "OpenCV calib3d, highgui and imgproc modules are requested to run the calibration." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install OpenCV, configure again ViSP using cmake and build again this example" << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "pugixml built-in 3rdparty is requested to run the calibration." << std::endl;
#endif
  return EXIT_SUCCESS;
  }
#endif
