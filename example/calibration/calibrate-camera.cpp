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
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class Settings
{
public:
  Settings()
    : boardSize(), calibrationPattern(UNDEFINED), squareSize(0.), input(), tempo(0.), goodInput(false), patternToUse()
  {
    boardSize = cv::Size(0, 0);
    calibrationPattern = UNDEFINED;
    squareSize = 0.025f;
    goodInput = false;
    tempo = 1.f;
  }
  enum Pattern { UNDEFINED, CHESSBOARD, CIRCLES_GRID };

  bool read(const std::string &filename) // Read the parameters
  {
    // reading configuration file
    if (!vpIoTools::loadConfigFile(filename))
      return false;
    vpIoTools::readConfigVar("BoardSize_Width:", boardSize.width);
    vpIoTools::readConfigVar("BoardSize_Height:", boardSize.height);
    vpIoTools::readConfigVar("Square_Size:", squareSize);
    vpIoTools::readConfigVar("Calibrate_Pattern:", patternToUse);
    vpIoTools::readConfigVar("Input:", input);
    vpIoTools::readConfigVar("Tempo:", tempo);

    std::cout << "grid width : " << boardSize.width << std::endl;
    std::cout << "grid height: " << boardSize.height << std::endl;
    std::cout << "square size: " << squareSize << std::endl;
    std::cout << "pattern    : " << patternToUse << std::endl;
    std::cout << "input seq  : " << input << std::endl;
    std::cout << "tempo      : " << tempo << std::endl;
    interprate();
    return true;
  }
  void interprate()
  {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
      std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
      goodInput = false;
    }
    if (squareSize <= 10e-6) {
      std::cerr << "Invalid square size " << squareSize << std::endl;
      goodInput = false;
    }

    if (input.empty()) // Check for valid input
      goodInput = false;

    calibrationPattern = UNDEFINED;
    if (patternToUse.compare("CHESSBOARD") == 0)
      calibrationPattern = CHESSBOARD;
    else if (patternToUse.compare("CIRCLES_GRID") == 0)
      calibrationPattern = CIRCLES_GRID;
    if (calibrationPattern == UNDEFINED) {
      std::cerr << " Inexistent camera calibration mode: " << patternToUse << std::endl;
      goodInput = false;
    }
  }

public:
  cv::Size boardSize;         // The size of the board -> Number of items by width and
                              // height
  Pattern calibrationPattern; // One of the Chessboard, circles, or asymmetric
                              // circle pattern
  float squareSize;           // The size of a square in your defined unit (point,
                              // millimeter,etc).
  std::string input;          // The input image sequence
  float tempo;                // Tempo in seconds between two images. If > 10 wait a click to
                              // continue
  bool goodInput;

private:
  std::string patternToUse;
};
#endif

int main(int argc, const char **argv)
{
  try {
    std::string outputFileName = "camera.xml";
    Settings s;
    const std::string inputSettingsFile = argc > 1 ? argv[1] : "default.cfg";

    if (!s.read(inputSettingsFile)) {
      std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
      std::cout << std::endl << "Usage: " << argv[0] << " <configuration file>.cfg" << std::endl;
      return -1;
    }

    if (!s.goodInput) {
      std::cout << "Invalid input detected. Application stopping. " << std::endl;
      return -1;
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
      std::cout << "Check if input images name \"" << s.input << "\" set in " << inputSettingsFile << " config file is correct..." << std::endl;
      return EXIT_FAILURE;
    }

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined VISP_HAVE_GDI
    vpDisplayGDI d(I);
#elif defined VISP_HAVE_GTK
    vpDisplayGTK d(I);
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV d(I);
#endif

    vpCameraParameters cam;

    // Initialize camera parameters
    double px = cam.get_px();
    double py = cam.get_px();
    // Set (u0,v0) in the middle of the image
    double u0 = I.getWidth() / 2;
    double v0 = I.getHeight() / 2;
    cam.initPersProjWithoutDistortion(px, py, u0, v0);

    std::vector<vpPoint> model;
    std::vector<vpCalibration> calibrator;

    for (int i = 0; i < s.boardSize.height; i++) {
      for (int j = 0; j < s.boardSize.width; j++) {
        model.push_back(vpPoint(j * s.squareSize, i * s.squareSize, 0));
      }
    }

    while (!reader.end()) {
      reader.acquire(I);

      long frame_index = reader.getFrameIndex();
      vpDisplay::display(I);

      cv::Mat cvI;
      std::vector<cv::Point2f> pointBuf;
      vpImageConvert::convert(I, cvI);

      bool found = false;
      switch (s.calibrationPattern) // Find feature points on the input format
      {
      case Settings::CHESSBOARD:
        // std::cout << "Use chessboard " << std::endl;
        found = findChessboardCorners(cvI, s.boardSize, pointBuf,
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                          cv::CALIB_CB_NORMALIZE_IMAGE);
#else
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |
                                          CV_CALIB_CB_NORMALIZE_IMAGE);
#endif
        break;
      case Settings::CIRCLES_GRID:
        // std::cout << "Use circle grid " << std::endl;
        found = findCirclesGrid(cvI, s.boardSize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID);
        break;
      case Settings::UNDEFINED:
      default:
        std::cout << "Unkown calibration grid " << std::endl;
        break;
      }

      std::cout << "frame: " << frame_index << ", status: " << found;
      if (!found)
        std::cout << ", image rejected" << std::endl;
      else
        std::cout << ", image used as input data" << std::endl;

      if (found) // If done with success,
      {
        std::vector<vpImagePoint> data;

        if (s.calibrationPattern == Settings::CHESSBOARD) {
          // improve the found corners' coordinate accuracy for chessboard
          cornerSubPix(cvI, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
#else
                       cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
#endif
        }
        std::stringstream ss;
        ss << "image " << frame_index;
        vpDisplay::setTitle(I, ss.str());
        for (unsigned int i = 0; i < pointBuf.size(); i++) {
          vpImagePoint ip(pointBuf[i].y, pointBuf[i].x);
          data.push_back(ip);
          vpDisplay::displayCross(I, ip, 10, vpColor::red);
        }

        // Calibration on a single mono image
        vpCalibration calib;
        calib.setLambda(0.5);
        calib.clearPoint();
        for (unsigned int i = 0; i < model.size(); i++) {
          calib.addPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ(), data[i]);
        }
        vpHomogeneousMatrix cMo;

        if (calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS, cMo, cam, false) == EXIT_SUCCESS) {
          //std::cout << "camera parameters for frame " << frame_index << ": " << cam << std::endl;
          calibrator.push_back(calib);
        }
      }

      if (found)
        vpDisplay::displayText(I, 15, 15, "Image processing succeed", vpColor::green);
      else
        vpDisplay::displayText(I, 15, 15, "Image processing fails", vpColor::green);

      if (s.tempo > 10.f) {
        vpDisplay::displayText(I, 35, 15, "A click to process the next image", vpColor::green);
        vpDisplay::flush(I);
        vpDisplay::getClick(I);
      } else {
        vpDisplay::flush(I);
        vpTime::wait(s.tempo * 1000);
      }
    }

    // Now we consider the multi image calibration
    // Calibrate by a non linear method based on virtual visual servoing
    if (calibrator.empty()) {
      std::cerr << "Unable to calibrate. Image processing failed !" << std::endl;
      return 0;
    }

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
      std::cout << "Global reprojection error: " << error << std::endl;
      ss_additional_info << "<global_reprojection_error><without_distortion>" << error << "</without_distortion>";

#ifdef VISP_HAVE_XML2
      vpXmlParserCamera xml;

      if (xml.save(cam, outputFileName.c_str(), "Camera", I.getWidth(), I.getHeight()) ==
          vpXmlParserCamera::SEQUENCE_OK)
        std::cout << "Camera parameters without distortion successfully saved in \"" << outputFileName << "\""
                  << std::endl;
      else {
        std::cout << "Failed to save the camera parameters without distortion in \"" << outputFileName << "\""
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

    std::cout << "\nCalibration with distortion in progress on " << calibrator.size() << " images..." << std::endl;
    if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS_DIST, calibrator, cam, error, false) ==
        EXIT_SUCCESS) {
      std::cout << cam << std::endl;
      std::cout << "Global reprojection error: " << error << std::endl;
      ss_additional_info << "<with_distortion>" << error << "</with_distortion></global_reprojection_error>";

#ifdef VISP_HAVE_XML2
      vpXmlParserCamera xml;

      if (xml.save(cam, outputFileName.c_str(), "Camera", I.getWidth(), I.getHeight(), ss_additional_info.str()) ==
          vpXmlParserCamera::SEQUENCE_OK)
        std::cout << "Camera parameters without distortion successfully saved in \"" << outputFileName << "\""
                  << std::endl;
      else {
        std::cout << "Failed to save the camera parameters without distortion in \"" << outputFileName << "\""
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
