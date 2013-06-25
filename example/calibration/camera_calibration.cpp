/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Camera calibration with chessboard or circle calibration grid.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visp/vpCalibration.h>

#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpIoTools.h>
#include <visp/vpPoint.h>
#include <visp/vpVideoReader.h>
#include <visp/vpXmlParserCamera.h>

class Settings
{
public:
  Settings() : goodInput(false) {}
  enum Pattern { UNDEFINED, CHESSBOARD, CIRCLES_GRID};

  bool read(const std::string &filename)    // Read the parameters
  {
    // reading configuration file
    if (! vpIoTools::loadConfigFile(filename) )
      return false;
    vpIoTools::readConfigVar("BoardSize_Width:", boardSize.width);
    vpIoTools::readConfigVar("BoardSize_Height:", boardSize.height);
    vpIoTools::readConfigVar("Square_Size:", squareSize);
    vpIoTools::readConfigVar("Calibrate_Pattern:", patternToUse);
    vpIoTools::readConfigVar("Input:", input);

    std::cout << "grid width : " << boardSize.width << std::endl;
    std::cout << "grid height: " << boardSize.height << std::endl;
    std::cout << "square size: " << squareSize << std::endl;
    std::cout << "pattern    : " << patternToUse << std::endl;
    std::cout << "input seq  : " << input << std::endl;
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

    if (input.empty())      // Check for valid input
      goodInput = false;

    calibrationPattern = UNDEFINED;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (calibrationPattern == UNDEFINED) {
      std::cerr << " Inexistent camera calibration mode: " << patternToUse << std::endl;
      goodInput = false;
    }
  }

public:
  cv::Size boardSize;        // The size of the board -> Number of items by width and height
  Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
  float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
  std::string input;         // The input image sequence
  bool goodInput;

private:
  std::string patternToUse;
};


#if VISP_HAVE_OPENCV_VERSION >= 0x020300
int main(int argc, const char ** argv)
{
  std::string outputFileName = "camera.xml";

  Settings s;
  const std::string inputSettingsFile = argc > 1 ? argv[1] : "default.cfg";
  if (! s.read(inputSettingsFile) ) {
    std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
    std::cout << std::endl << "Usage: " << argv[0] << " <configuration file>.cfg" << std::endl;
    return -1;
  }

  if (! s.goodInput)
  {
    std::cout << "Invalid input detected. Application stopping. " << std::endl;
    return -1;
  }

  // Start the calibration code
  vpImage<unsigned char> I;
  vpVideoReader reader;
  reader.setFileName(s.input);
  reader.open(I);

#ifdef VISP_HAVE_X11
  vpDisplayX d(I);
#elif defined VISP_HAVE_GDI
  vpDisplayGDI d(I);
#elif defined VISP_HAVE_GTK
  vpDisplayGTK d(I);
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV d(I);
#endif

  std::vector<vpPoint> model;
  std::vector<vpCalibration> calibrator;

  for (int i=0; i< s.boardSize.height; i++) {
    for (int j=0; j< s.boardSize.width; j++) {
      vpPoint P;
      P.setWorldCoordinates(j*s.squareSize, i*s.squareSize, 0);
      model.push_back(P);
    }
  }

  long frame_index;
  while(! reader.end()) {
    frame_index = reader.getFrameIndex();
    reader.acquire(I);
    vpDisplay::display(I);

    cv::Mat cvI;
    std::vector<cv::Point2f> pointBuf;
    vpImageConvert::convert(I, cvI);

    bool found;
    switch( s.calibrationPattern ) // Find feature points on the input format
    {
    case Settings::CHESSBOARD:
      found = findChessboardCorners( cvI, s.boardSize, pointBuf,
                                     CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
      break;
    case Settings::CIRCLES_GRID:
      found = findCirclesGrid( cvI, s.boardSize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID  );
      break;
    default:
      break;
    }

    std::cout << "frame: " << frame_index << " status: " << found << std::endl;

    if ( found)                // If done with success,
    {
      std::vector<vpImagePoint> data;

      if (s.calibrationPattern == Settings::CHESSBOARD) {
        // improve the found corners' coordinate accuracy for chessboard
        cornerSubPix( cvI, pointBuf, cv::Size(11,11),
                      cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
      }
      char title[20]; sprintf(title, "image %ld", frame_index);
      vpDisplay::setTitle(I, title);
      for (unsigned int i=0; i < pointBuf.size(); i++) {
        vpImagePoint ip(pointBuf[i].y, pointBuf[i].x);
        data.push_back(ip);
        vpDisplay::displayCross(I, ip, 10, vpColor::red);

      }

      // Calibration on a single mono image
      vpCalibration calib;
      calib.setLambda(0.5);
      calib.clearPoint();
      for (unsigned int i=0; i<model.size(); i++) {
        calib.addPoint(model[i].get_oX(), model[i].get_oY(), model[i].get_oZ(), data[i]);
      }
      vpHomogeneousMatrix cMo;
      vpCameraParameters cam;

      // Set (u0,v0) in the middle of the image
      double px = cam.get_px();
      double py = cam.get_px();
      double u0 = I.getWidth()/2;
      double v0 = I.getHeight()/2;
      cam.initPersProjWithoutDistortion(px, py, u0, v0);

      if (calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS, cMo, cam, false) == 0) {
        //std::cout << "camera parameters: " << cam << std::endl;
        calibrator.push_back(calib);
      }
    }

    vpDisplay::flush(I);
    //vpDisplay::getClick(I);
  }

  // Now we consider the multi image calibration
  // Calibrate by a non linear method based on virtual visual servoing
  if (calibrator.empty()) {
    std::cerr << "Unable to calibrate. Image processing failed !" << std::endl;
    return 0;
  }

  std::cout << "\nCalibration without distorsion in progress on " << calibrator.size() << " images..." << std::endl;
  vpCameraParameters cam;
  double error;
  if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS, calibrator, cam, error, false) == 0) {
    std::cout << cam << std::endl;
    std::cout << "Global reprojection error: " << error << std::endl;
#ifdef VISP_HAVE_XML2
    vpXmlParserCamera xml;

    if(xml.save(cam, outputFileName.c_str(), "Camera", I.getWidth(), I.getHeight()) == vpXmlParserCamera::SEQUENCE_OK)
      std::cout << "Camera parameters without distortion successfully saved in \"" << outputFileName << "\"" << std::endl;
    else {
      std::cout << "Failed to save the camera parameters without distortion in \"" << outputFileName << "\"" << std::endl;
      std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
    }
#endif
  }
  else
    std::cout << "Calibration without distortion failed." << std::endl;

  std::cout << "\nCalibration with distorsion in progress on " << calibrator.size() << " images..." << std::endl;
  if (vpCalibration::computeCalibrationMulti(vpCalibration::CALIB_VIRTUAL_VS_DIST, calibrator, cam, error, false) == 0) {
    std::cout << cam << std::endl;
    std::cout << "Global reprojection error: " << error << std::endl;
#ifdef VISP_HAVE_XML2
    vpXmlParserCamera xml;

    if(xml.save(cam, outputFileName.c_str(), "Camera", I.getWidth(), I.getHeight()) == vpXmlParserCamera::SEQUENCE_OK)
      std::cout << "Camera parameters without distortion successfully saved in \"" << outputFileName << "\"" << std::endl;
    else {
      std::cout << "Failed to save the camera parameters without distortion in \"" << outputFileName << "\"" << std::endl;
      std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
    }
#endif
  }
  else
    std::cout << "Calibration with distortion failed." << std::endl;
}
#else
int main()
{
  std::cout << "OpenCV 2.3.0 or higher is requested to run the calibration." << std::endl;
}
#endif
