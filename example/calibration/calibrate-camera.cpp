/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#include <iostream>

#include <visp3/core/vpConfig.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020300

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp3/vision/vpCalibration.h>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>

#include "calibration-helper.hpp"

using namespace calib_helper;

int main(int argc, const char **argv)
{
  try {
    std::string opt_output_file_name = "camera.xml";
    Settings s;
    const std::string opt_inputSettingsFile = argc > 1 ? argv[1] : "default.cfg";
    std::string opt_init_camera_xml_file;
    std::string opt_camera_name = "Camera";

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--init-from-xml")
        opt_init_camera_xml_file = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--camera-name")
        opt_camera_name = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--output")
        opt_output_file_name = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [<configuration file>.cfg] [--init-from-xml <camera-init.xml>]"
                  << " [--camera-name <name>] [--output <file.xml>] [--help] [-h] \n" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    if (!s.read(opt_inputSettingsFile)) {
      std::cout << "Could not open the configuration file: \"" << opt_inputSettingsFile << "\"" << std::endl;
      std::cout << std::endl << "Usage: " << argv[0] << " <configuration file>.cfg" << std::endl;
      return EXIT_FAILURE;
    }

    if (!s.goodInput) {
      std::cout << "Invalid input detected. Application stopping. " << std::endl;
      return EXIT_FAILURE;
    }

    // Start the calibration code
    vpImage<unsigned char> I;
    vpVideoReader reader;
    reader.setFileName(s.input);
    try {
      reader.open(I);
    }
    catch(const vpException &e) {
      std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
      std::cout << "Check if input images name \"" << s.input << "\" set in " << opt_inputSettingsFile << " config file is correct..." << std::endl;
      return EXIT_FAILURE;
    }

#ifdef VISP_HAVE_X11
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#elif defined VISP_HAVE_GTK
    vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
#endif

    vpCameraParameters cam;
    bool init_from_xml = false;
    if (! opt_init_camera_xml_file.empty()) {
#ifdef VISP_HAVE_PUGIXML
      if (! vpIoTools::checkFilename(opt_init_camera_xml_file)) {
        std::cout << "Input camera file \"" << opt_init_camera_xml_file << "\" doesn't exist!" << std::endl;
        std::cout << "Modify [--init-from-xml <camera-init.xml>] option value" << std::endl;
        return EXIT_FAILURE;
      }
      init_from_xml = true;
#else
      std::cout << "Cannot initialize camera parameters from xml file: " << opt_init_camera_xml_file << std::endl;
#endif
    }
    if (init_from_xml) {
#ifdef VISP_HAVE_PUGIXML
      std::cout << "Initialize camera parameters from xml file: " << opt_init_camera_xml_file << std::endl;
      vpXmlParserCamera parser;
      if (parser.parse(cam, opt_init_camera_xml_file, opt_camera_name, vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Unable to find camera with name \"" << opt_camera_name << "\" in file: " << opt_init_camera_xml_file << std::endl;
        std::cout << "Modify [--camera-name <name>] option value" << std::endl;
        return EXIT_FAILURE;
      }
#endif
    } else {
      std::cout << "Initialize camera parameters with default values " << std::endl;
      // Initialize camera parameters
      double px = cam.get_px();
      double py = cam.get_py();
      // Set (u0,v0) in the middle of the image
      double u0 = I.getWidth() / 2;
      double v0 = I.getHeight() / 2;
      cam.initPersProjWithoutDistortion(px, py, u0, v0);
    }

    std::cout << "Camera parameters used for initialization:\n" << cam << std::endl;

    std::vector<vpPoint> model;
    std::vector<vpCalibration> calibrator;

    for (int i = 0; i < s.boardSize.height; i++) {
      for (int j = 0; j < s.boardSize.width; j++) {
        model.push_back(vpPoint(j * s.squareSize, i * s.squareSize, 0));
      }
    }

    std::vector<CalibInfo> calib_info;
    do {
      reader.acquire(I);

      long frame_index = reader.getFrameIndex();
      vpDisplay::display(I);

      cv::Mat cvI;
      std::vector<cv::Point2f> pointBuf;
      vpImageConvert::convert(I, cvI);

      bool found = extractCalibrationPoints(s, cvI, pointBuf);

      std::cout << "frame: " << frame_index << ", status: " << found;
      if (!found)
        std::cout << ", image rejected" << std::endl;
      else
        std::cout << ", image used as input data" << std::endl;

      if (found) // If done with success,
      {
        std::stringstream ss;
        ss << "image " << frame_index;
        vpDisplay::setTitle(I, ss.str());

        std::vector<vpImagePoint> data;
        for (unsigned int i = 0; i < pointBuf.size(); i++) {
          vpImagePoint ip(pointBuf[i].y, pointBuf[i].x);
          data.push_back(ip);
          vpDisplay::displayCross(I, ip, 10*vpDisplay::getDownScalingFactor(I), vpColor::red);
        }

        // Calibration on a single mono image
        std::vector<vpPoint> calib_points;
        vpCalibration calib;
        calib.setLambda(0.5);
        for (unsigned int i = 0; i < model.size(); i++) {
          calib.addPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ(), data[i]);
          calib_points.push_back(vpPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ()));
          calib_points.back().set_x(data[i].get_u());
          calib_points.back().set_y(data[i].get_v());
        }

        // Add calibration info
        calib_info.push_back(CalibInfo(I, calib_points, data));

        vpHomogeneousMatrix cMo;
        if (calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS, cMo, cam, false) == EXIT_SUCCESS) {
          //std::cout << "camera parameters for frame " << frame_index << ": " << cam << std::endl;
          calibrator.push_back(calib);
        }
      }

      if (found)
        vpDisplay::displayText(I, 15*vpDisplay::getDownScalingFactor(I), 15*vpDisplay::getDownScalingFactor(I),
                               "Image processing succeed", vpColor::green);
      else
        vpDisplay::displayText(I, 15*vpDisplay::getDownScalingFactor(I), 15*vpDisplay::getDownScalingFactor(I),
                               "Image processing fails", vpColor::green);

      if (s.tempo > 10.f) {
        vpDisplay::displayText(I, 35*vpDisplay::getDownScalingFactor(I), 15*vpDisplay::getDownScalingFactor(I),
                               "A click to process the next image", vpColor::green);
        vpDisplay::flush(I);
        vpDisplay::getClick(I);
      } else {
        vpDisplay::flush(I);
        vpTime::wait(s.tempo * 1000);
      }
    } while (!reader.end());

    // Now we consider the multi image calibration
    // Calibrate by a non linear method based on virtual visual servoing
    if (calibrator.empty()) {
      std::cerr << "Unable to calibrate. Image processing failed !" << std::endl;
      return 0;
    }

    // Display calibration pattern occupancy
    drawCalibrationOccupancy(I, calib_info, s.boardSize.width);
    vpDisplay::setTitle(I, "Calibration pattern occupancy");
    vpDisplay::display(I);
    vpDisplay::displayText(I, 15, 15, "Calibration pattern occupancy in the image", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

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

    std::cout << "\nCalibration without distortion in progress on " << calibrator.size() << " images..." << std::endl;
    double error;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS, calibrator, cam, error, false) == EXIT_SUCCESS) {
      std::cout << cam << std::endl;
      vpDisplay::setTitle(I, "Reprojection error");

      for (size_t i = 0; i < calibrator.size(); i++) {
        double reproj_error = sqrt(calibrator[i].getResidual()/calibrator[i].get_npt());
        std::cout << i << ") reprojection error: " << reproj_error << std::endl;

        const CalibInfo& calib = calib_info[i];
        I = calib.m_img;
        vpDisplay::display(I);

        std::ostringstream ss;
        ss << "reprojection error: " << reproj_error;
        vpDisplay::displayText(I, 15, 15, ss.str(), vpColor::red);
        vpDisplay::displayText(I, 30, 15, "extracted points", vpColor::red);
        vpDisplay::displayText(I, 45, 15, "projected points", vpColor::green);

        for (size_t idx = 0; idx < calib.m_points.size(); idx++) {
          vpDisplay::displayCross(I, calib.m_imPts[idx], 12, vpColor::red);

          vpPoint pt = calib.m_points[idx];
          pt.project(calibrator[i].cMo);
          vpImagePoint imPt;
          vpMeterPixelConversion::convertPoint(cam, pt.get_x(), pt.get_y(), imPt);
          vpDisplay::displayCross(I, imPt, 12, vpColor::green);
        }

        vpDisplay::flush(I);
        vpDisplay::getClick(I);
      }

      std::cout << "\nGlobal reprojection error: " << error << std::endl;
      ss_additional_info << "<global_reprojection_error><without_distortion>" << error << "</without_distortion>";

#ifdef VISP_HAVE_PUGIXML
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
#endif
    } else {
      std::cout << "Calibration without distortion failed." << std::endl;
      return EXIT_FAILURE;
    }
    vpCameraParameters cam_without_dist = cam;
    std::vector<vpCalibration> calibrator_without_dist = calibrator;

    std::cout << "\n\nCalibration with distortion in progress on " << calibrator.size() << " images..." << std::endl;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS_DIST, calibrator, cam, error, false) ==
        EXIT_SUCCESS) {
      std::cout << cam << std::endl;

      for (size_t i = 0; i < calibrator.size(); i++) {
        std::cout << i << ") reprojection error: " << sqrt(calibrator[i].getResidual_dist()/calibrator[i].get_npt()) << std::endl;
      }

      std::cout << "\nGlobal reprojection error: " << error << std::endl;
      ss_additional_info << "<with_distortion>" << error << "</with_distortion></global_reprojection_error>";

      vpImage<unsigned char> I_undist = I;
#ifdef VISP_HAVE_X11
    vpDisplayX d_undist(I_undist, I.getWidth(), 0, "Undistorted image", vpDisplay::SCALE_AUTO);
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d_undist(I_undist, I.getWidth(), 0, "Undistorted image", vpDisplay::SCALE_AUTO);
#elif defined VISP_HAVE_GTK
    vpDisplayGTK d_undist(I_undist, I.getWidth(), 0, "Undistorted image", vpDisplay::SCALE_AUTO);
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d_undist(I_undist, I.getWidth(), 0, "Undistorted image", vpDisplay::SCALE_AUTO);
#endif

      vpDisplay::setTitle(I, "Line fitting on distorted image");
      vpDisplay::setTitle(I_undist, "Line fitting on undistorted image");
      std::cout << "\nThis tool computes the line fitting error (RMSE) on image points extracted from the raw distorted image"
                << " and on image points after undistortion (vpPixelMeterConversion::convertPoint())." << std::endl;
      for (size_t idx = 0; idx < calib_info.size(); idx++) {
        I = calib_info[idx].m_img;
        vpImageTools::undistort(I, cam, I_undist);

        vpDisplay::display(I);
        vpDisplay::display(I_undist);

        vpDisplay::displayText(I, 15, 15, "Draw lines from first / last points.", vpColor::red);
        std::vector<vpImagePoint> grid_points = calib_info[idx].m_imPts;
        for (int i = 0; i < s.boardSize.height; i++) {
          std::vector<vpImagePoint> current_line(grid_points.begin() + i*s.boardSize.width,
                                                 grid_points.begin() + (i+1)*s.boardSize.width);

          std::vector<vpImagePoint> current_line_undist = undistort(cam, current_line);
          double a = 0, b = 0, c = 0;
          double line_fitting_error = lineFitting(current_line, a, b, c);
          double line_fitting_error_undist = lineFitting(current_line_undist, a, b, c);
          std::cout << "Line fitting error on distorted points: " << line_fitting_error
                    << " ; on undistorted points: " << line_fitting_error_undist << std::endl;

          vpImagePoint ip1 = current_line.front();
          vpImagePoint ip2 = current_line.back();
          vpDisplay::displayLine(I, ip1, ip2, vpColor::red);
        }

        std::cout << "\nThis tool computes the line fitting error (RMSE) on image points extracted from the undistorted image"
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
            vpDisplay::displayCross(I_undist, ip, 10, vpColor::red);
          }

          std::cout << std::endl;
          vpDisplay::displayText(I_undist, 15, 15, "Draw fitting lines from extracted points (cross).", vpColor::red);
          for (int i = 0; i < s.boardSize.height; i++) {
            std::vector<vpImagePoint> current_line(grid_points.begin() + i*s.boardSize.width,
                                                   grid_points.begin() + (i+1)*s.boardSize.width);

            double a = 0, b = 0, c = 0;
            double line_fitting_error = lineFitting(current_line, a, b, c);
            std::cout << "Undistorted image, line fitting error: " << line_fitting_error << std::endl;

            vpImagePoint ip1;
            ip1.set_u(current_line.front().get_u());
            ip1.set_v( -(c + a*ip1.get_u()) / b );

            vpImagePoint ip2;
            ip2.set_u(current_line.back().get_u());
            ip2.set_v( -(c + a*ip2.get_u()) / b );

            vpDisplay::displayLine(I_undist, ip1, ip2, vpColor::red);
          }
        }

        vpDisplay::flush(I);
        vpDisplay::flush(I_undist);
        vpDisplay::getClick(I);
      }

#ifdef VISP_HAVE_PUGIXML
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

      if (xml.save(cam, opt_output_file_name.c_str(), opt_camera_name, I.getWidth(), I.getHeight(), ss_additional_info.str()) ==
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
#endif
      std::cout << std::endl;
      for (unsigned int i = 0; i < calibrator.size(); i++)
        std::cout << "Estimated pose on input data " << i << ": " << vpPoseVector(calibrator[i].cMo_dist).t()
                  << std::endl;

    } else {
      std::cout << "Calibration with distortion failed." << std::endl;
      return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "OpenCV 2.3.0 or higher is requested to run the calibration." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install OpenCV, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
