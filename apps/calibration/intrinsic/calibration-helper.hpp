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
 * Helper functions for camera calibration tool.
 *
*****************************************************************************/
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)
#include <opencv2/core/core.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpPolygon.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

namespace calib_helper
{
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
    if (!VISP_NAMESPACE_ADDRESSING vpIoTools::loadConfigFile(filename))
      return false;
    VISP_NAMESPACE_ADDRESSING vpIoTools::readConfigVar("BoardSize_Width:", boardSize.width);
    VISP_NAMESPACE_ADDRESSING vpIoTools::readConfigVar("BoardSize_Height:", boardSize.height);
    VISP_NAMESPACE_ADDRESSING vpIoTools::readConfigVar("Square_Size:", squareSize);
    VISP_NAMESPACE_ADDRESSING vpIoTools::readConfigVar("Calibrate_Pattern:", patternToUse);
    VISP_NAMESPACE_ADDRESSING vpIoTools::readConfigVar("Input:", input);
    VISP_NAMESPACE_ADDRESSING vpIoTools::readConfigVar("Tempo:", tempo);

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

struct CalibInfo
{
  CalibInfo(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &img, const std::vector<VISP_NAMESPACE_ADDRESSING vpPoint> &points,
            const std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> &imPts, const std::string &frame_name)
    : m_img(img), m_points(points), m_imPts(imPts), m_frame_name(frame_name)
  { }

  VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> m_img;
  std::vector<VISP_NAMESPACE_ADDRESSING vpPoint> m_points;
  std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> m_imPts;
  std::string m_frame_name;
};

void drawCalibrationOccupancy(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const std::vector<CalibInfo> &calib_info,
                              unsigned int patternW)
{
  I = 0u;
  unsigned char pixel_value = static_cast<unsigned char>(255.0 / calib_info.size());
  for (size_t idx = 0; idx < calib_info.size(); idx++) {
    const CalibInfo &calib = calib_info[idx];

    std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> corners;
    corners.push_back(calib.m_imPts.front());
    corners.push_back(*(calib.m_imPts.begin() + patternW - 1));
    corners.push_back(calib.m_imPts.back());
    corners.push_back(*(calib.m_imPts.end() - patternW));
    VISP_NAMESPACE_ADDRESSING vpPolygon poly(corners);

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        if (poly.isInside(VISP_NAMESPACE_ADDRESSING vpImagePoint(i, j))) {
          I[i][j] += pixel_value;
        }
      }
    }
  }
}

std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> undistort(const VISP_NAMESPACE_ADDRESSING vpCameraParameters &cam_dist, const std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> &imPts)
{
  std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> imPts_undist;

  VISP_NAMESPACE_ADDRESSING vpCameraParameters cam(cam_dist.get_px(), cam_dist.get_py(), cam_dist.get_u0(), cam_dist.get_v0());
  for (size_t i = 0; i < imPts.size(); i++) {
    double x = 0, y = 0;
    VISP_NAMESPACE_ADDRESSING vpPixelMeterConversion::convertPoint(cam_dist, imPts[i], x, y);

    VISP_NAMESPACE_ADDRESSING vpImagePoint imPt;
    VISP_NAMESPACE_ADDRESSING vpMeterPixelConversion::convertPoint(cam, x, y, imPt);
    imPts_undist.push_back(imPt);
  }

  return imPts_undist;
}

bool extractCalibrationPoints(const Settings &s, const cv::Mat &cvI, std::vector<cv::Point2f> &pointBuf)
{
  bool found = false;
  switch (s.calibrationPattern) // Find feature points on the input format
  {
  case Settings::CHESSBOARD:
    found =
      findChessboardCorners(cvI, s.boardSize, pointBuf,
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
#else
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
#endif
    break;
  case Settings::CIRCLES_GRID:
    found = findCirclesGrid(cvI, s.boardSize, pointBuf, cv::CALIB_CB_SYMMETRIC_GRID);
    break;
  case Settings::UNDEFINED:
  default:
    break;
  }

  if (found) // If done with success,
  {
    std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> data;

    if (s.calibrationPattern == Settings::CHESSBOARD) {
      // improve the found corners' coordinate accuracy for chessboard
      cornerSubPix(cvI, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
#else
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
#endif
    }
  }

  return found;
}

} // namespace calib_helper

#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif // VISP_HAVE_OPENCV
