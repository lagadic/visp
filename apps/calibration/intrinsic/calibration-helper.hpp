/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 */
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV)
#include <opencv2/core/core.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpColormap.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#if defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME;
#endif

namespace calib_helper
{

// Adapted from:
// https://stackoverflow.com/questions/58881746/c-how-to-cout-and-write-at-the-same-time/58881939#58881939
class Tee
{
private:
  std::ostream &os;
  std::ofstream &file;

public:
  Tee(std::ostream &os_, std::ofstream &file_) : os(os_), file(file_) { }

  template <typename T>
  Tee &operator<<(const T &thing)
  {
    os << thing;
    if (file.is_open()) {
      file << thing;
    }
    return *this;
  }
};

class Settings
{
public:
  Settings(Tee &tee_)
    : boardSize(), calibrationPattern(UNDEFINED), squareSize(0.), input(), tempo(0.), goodInput(false), patternToUse(),
    tee(tee_)
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

    tee << "grid width : " << boardSize.width << "\n";
    tee << "grid height: " << boardSize.height << "\n";
    tee << "square size: " << squareSize << "\n";
    tee << "pattern    : " << patternToUse << "\n";
    tee << "input seq  : " << input << "\n";
    tee << "tempo      : " << tempo << "\n";
    interprate();
    return true;
  }

  void interprate()
  {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
      tee << "Invalid Board size: " << boardSize.width << " " << boardSize.height << "\n";
      goodInput = false;
    }
    if (squareSize <= 10e-6) {
      tee << "Invalid square size " << squareSize << "\n";
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
      tee << " Inexistent camera calibration mode: " << patternToUse << "\n";
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
  Tee &tee;
};

struct CalibInfo
{
  CalibInfo(const vpImage<unsigned char> &img, const std::vector<vpPoint> &points,
            const std::vector<vpImagePoint> &imPts, const std::string &frame_name)
    : m_img(img), m_points(points), m_imPts(imPts), m_frame_name(frame_name)
  { }

  vpImage<unsigned char> m_img;
  std::vector<vpPoint> m_points;
  std::vector<vpImagePoint> m_imPts;
  std::string m_frame_name;
};

void drawCalibrationOccupancy(vpImage<unsigned char> &I, const std::vector<CalibInfo> &calib_info, unsigned int patternW)
{
  I = 0u;
  unsigned char pixel_value = static_cast<unsigned char>(255.0 / calib_info.size());
  for (size_t idx = 0; idx < calib_info.size(); idx++) {
    const CalibInfo &calib = calib_info[idx];

    std::vector<vpImagePoint> corners;
    corners.push_back(calib.m_imPts.front());
    corners.push_back(*(calib.m_imPts.begin() + patternW - 1));
    corners.push_back(calib.m_imPts.back());
    corners.push_back(*(calib.m_imPts.end() - patternW));
    vpPolygon poly(corners);

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        if (poly.isInside(vpImagePoint(i, j))) {
          I[i][j] += pixel_value;
        }
      }
    }
  }
}

std::vector<vpImagePoint> undistort(const vpCameraParameters &cam_dist, const std::vector<vpImagePoint> &imPts)
{
  std::vector<vpImagePoint> imPts_undist;

  vpCameraParameters cam(cam_dist.get_px(), cam_dist.get_py(), cam_dist.get_u0(), cam_dist.get_v0());
  for (size_t i = 0; i < imPts.size(); i++) {
    double x = 0, y = 0;
    vpPixelMeterConversion::convertPoint(cam_dist, imPts[i], x, y);

    vpImagePoint imPt;
    vpMeterPixelConversion::convertPoint(cam, x, y, imPt);
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
  }

  return found;
}

void computeDistortionDisplacementMap(const vpCameraParameters &cam_dist, vpImage<vpRGBa> &I_color)
{
  vpImage<double> displacement_map(I_color.getHeight(), I_color.getWidth());

  double min_displ = 1e12, max_displ = 0;
  for (unsigned int i = 0; i < I_color.getHeight(); i++) {
    for (unsigned int j = 0; j < I_color.getWidth(); j++) {
      double x = 0, y = 0, id = 0, jd = 0;
      vpPixelMeterConversion::convertPointWithoutDistortion(cam_dist, j, i, x, y);
      vpMeterPixelConversion::convertPoint(cam_dist, x, y, jd, id);

      double erri = id - i;
      double errj = jd - j;
      double displ = std::sqrt(erri*erri + errj*errj);
      displacement_map[i][j] = displ;
      min_displ = displ < min_displ ? displ : min_displ;
      max_displ = displ > max_displ ? displ : max_displ;
    }
  }

  double a = 255 / (max_displ - min_displ);
  double b = (-255 * min_displ) / (max_displ - min_displ);

  vpImage<unsigned char> I_gray(I_color.getHeight(), I_color.getWidth());
  for (unsigned int i = 0; i < displacement_map.getHeight(); i++) {
    for (unsigned int j = 0; j < displacement_map.getWidth(); j++) {
      I_gray[i][j] = static_cast<unsigned char>(vpMath::clamp(a * displacement_map[i][j] + b, 0.0, 255.0));
    }
  }

  vpColormap colormap(vpColormap::COLORMAP_TURBO);
  colormap.convert(I_gray, I_color);
}

void createMosaic(const std::vector<vpImage<vpRGBa>> &list_imgs, std::vector<vpImage<vpRGBa>> &list_mosaics,
  unsigned int nb_rows = 4, unsigned int nb_cols = 6)
{
  const unsigned int nb_totals = nb_rows*nb_cols;
  if (list_imgs.empty()) {
    return;
  }

  const unsigned int img_h = list_imgs[0].getHeight();
  const unsigned int img_w = list_imgs[0].getWidth();
  vpImage<vpRGBa> mosaic(img_h*nb_rows, img_w*nb_cols);
  for (size_t i = 0; i < list_imgs.size(); i += nb_totals) {
    mosaic = vpRGBa(0, 0, 0);

    for (size_t j = 0; j < nb_totals; j++) {
      const size_t idx = i + j;
      const unsigned int pos_mod = idx % nb_totals;
      if (idx >= list_imgs.size()) {
        break;
      }

      const unsigned int pos_row = pos_mod / nb_cols;
      vpImagePoint top_left(pos_row*img_h, (pos_mod - pos_row*nb_cols)*img_w);
      mosaic.insert(list_imgs[idx], top_left);
    }

    list_mosaics.push_back(mosaic);
  }
}

double getProjectionErrorUV(const std::vector<vpCalibration> &calibrator, const std::vector<CalibInfo> &calib_info,
    std::vector<std::vector<vpImagePoint>> &err_imPt_imgs, std::vector<vpImagePoint> &err_imPt, bool with_dist)
{
  assert(!calib_info.empty());
  double max_scale_uv = -1e6;
  err_imPt_imgs.reserve(calib_info.size());
  err_imPt.reserve(calib_info.size()*calib_info[0].m_points.size());

  for (size_t i = 0; i < calibrator.size(); i++) {
    const vpCalibration &calib = calibrator[i];
    const CalibInfo &calib_info_cur = calib_info[i];
    std::vector<vpImagePoint> err_imPt_per_img;
    err_imPt_per_img.reserve(calib_info_cur.m_points.size());

    for (size_t j = 0; j < calib_info_cur.m_points.size(); j++) {
      vpPoint pt_3d = calib_info_cur.m_points[j];
      vpImagePoint pt_proj;
      if (with_dist) {
        pt_3d.project(calib.cMo_dist);
        vpMeterPixelConversion::convertPoint(calib.cam_dist, pt_3d.get_x(), pt_3d.get_y(), pt_proj);
      }
      else {
        pt_3d.project(calib.cMo);
        vpMeterPixelConversion::convertPoint(calib.cam, pt_3d.get_x(), pt_3d.get_y(), pt_proj);
      }
      err_imPt_per_img.push_back(calib_info_cur.m_imPts[j] - pt_proj);
      err_imPt.push_back(calib_info_cur.m_imPts[j] - pt_proj);

      double err_u = std::fabs(err_imPt_per_img.back().get_u());
      double err_v = std::fabs(err_imPt_per_img.back().get_v());
      max_scale_uv = err_u > max_scale_uv ? err_u : max_scale_uv;
      max_scale_uv = err_v > max_scale_uv ? err_v : max_scale_uv;
    }
    err_imPt_imgs.push_back(err_imPt_per_img);
  }

  return max_scale_uv;
}

void displayProjectionErrorUV(const vpImage<unsigned char> &I_err_imPt, const std::vector<vpImagePoint> &err_imPt,
  unsigned int disp_size, double max_scale_uv, bool with_dist, const vpColor &color, unsigned int offset_text,
  const vpColor &color_text, bool cv_calib, unsigned int offset_text_incr = 15)
{
  const unsigned int margin = 25;
  const unsigned int margin_2 = margin/2;
  const unsigned int graph_size = disp_size - 2*margin;
  const unsigned int tick_size = 10;
  const unsigned int graph_offset = cv_calib ? 0 : (with_dist ? disp_size : 0);

  // axis arrows
  // left
  vpDisplay::displayArrow(I_err_imPt, vpImagePoint(disp_size/2, graph_offset+disp_size/2),
    vpImagePoint(disp_size/2, graph_offset+margin_2), vpColor::white, 2);
  // right
  vpDisplay::displayArrow(I_err_imPt, vpImagePoint(disp_size/2, graph_offset+disp_size/2),
    vpImagePoint(disp_size/2, graph_offset+disp_size-margin_2), vpColor::white, 2);
  // up
  vpDisplay::displayArrow(I_err_imPt, vpImagePoint(disp_size/2, graph_offset+disp_size/2),
    vpImagePoint(margin_2, graph_offset+disp_size/2), vpColor::white, 2);
  // down
  vpDisplay::displayArrow(I_err_imPt, vpImagePoint(disp_size/2, graph_offset+disp_size/2),
    vpImagePoint(disp_size-margin_2, graph_offset+disp_size/2), vpColor::white, 2);

  // outermost tick
  // left
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(disp_size/2-tick_size, graph_offset+margin),
    vpImagePoint(disp_size/2+tick_size, graph_offset+margin), vpColor::white);
  // right
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(disp_size/2-tick_size, graph_offset+disp_size-margin),
    vpImagePoint(disp_size/2+tick_size, graph_offset+disp_size-margin), vpColor::white);
  // up
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(margin, graph_offset+disp_size/2-tick_size),
    vpImagePoint(margin, graph_offset+disp_size/2+tick_size), vpColor::white);
  // down
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(disp_size-margin, graph_offset+disp_size/2-tick_size),
    vpImagePoint(disp_size-margin, graph_offset+disp_size/2+tick_size), vpColor::white);
  // label
  std::ostringstream oss_max;
  oss_max << std::fixed << std::setprecision(2) << max_scale_uv << " px";
  std::string max_val = oss_max.str();
  vpDisplay::displayText(I_err_imPt, vpImagePoint(disp_size/2 + 30, graph_offset+disp_size-50), max_val, vpColor::white);

  // half tick
  const unsigned int half_tick_pos = graph_size/4;
  // left
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(disp_size/2-tick_size, graph_offset+margin+half_tick_pos),
    vpImagePoint(disp_size/2+tick_size, graph_offset+margin+half_tick_pos), vpColor::white);
  // right
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(disp_size/2-tick_size, graph_offset+disp_size/2+half_tick_pos),
    vpImagePoint(disp_size/2+tick_size, graph_offset+disp_size/2+half_tick_pos), vpColor::white);
  // up
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(margin+half_tick_pos, graph_offset+disp_size/2-tick_size),
    vpImagePoint(margin+half_tick_pos, graph_offset+disp_size/2+tick_size), vpColor::white);
  // down
  vpDisplay::displayLine(I_err_imPt, vpImagePoint(disp_size/2+half_tick_pos, graph_offset+disp_size/2-tick_size),
    vpImagePoint(disp_size/2+half_tick_pos, graph_offset+disp_size/2+tick_size), vpColor::white);

  std::vector<double> u_vec, v_vec;
  u_vec.reserve(err_imPt.size());
  v_vec.reserve(err_imPt.size());

  double disp_scale_imPt = graph_size / (2*max_scale_uv);
  for (size_t i = 0; i < err_imPt.size(); i++) {
    vpImagePoint display_imPt(I_err_imPt.getHeight()/2.0 + disp_scale_imPt*err_imPt[i].get_i(),
      graph_offset + I_err_imPt.getWidth()/(cv_calib ? 2.0 : 4.0) + disp_scale_imPt*err_imPt[i].get_j());
    vpDisplay::displayCross(I_err_imPt, display_imPt, 8, color);

    u_vec.push_back(err_imPt[i].get_j());
    v_vec.push_back(err_imPt[i].get_v());
  }

  double reproj_error = 0;
  for (size_t i = 0; i < err_imPt.size(); i++) {
    reproj_error += err_imPt[i].get_i()*err_imPt[i].get_i() + err_imPt[i].get_j()*err_imPt[i].get_j();
  }
  reproj_error = std::sqrt(reproj_error / err_imPt.size());

  double u_err_mean = vpMath::getMean(u_vec), u_err_med = vpMath::getMedian(u_vec), u_err_std = vpMath::getStdev(u_vec);
  double v_err_mean = vpMath::getMean(v_vec), v_err_med = vpMath::getMedian(v_vec), v_err_std = vpMath::getStdev(v_vec);

  std::ostringstream oss;
  if (!cv_calib) {
    oss << std::fixed << std::setprecision(3) << "Reprojection error"
      << (with_dist ? " (with dist): " : " (without dist): ") << reproj_error << " (" << err_imPt.size() << " pts)";
    vpDisplay::displayText(I_err_imPt, offset_text, graph_offset+20, oss.str(), color_text);
  }
  else {
    oss << std::fixed << std::setprecision(3) << "Reprojection error: " << reproj_error
      << " (" << err_imPt.size() << " pts)";
    vpDisplay::displayText(I_err_imPt, offset_text, graph_offset+20, oss.str(), color_text);
  }

  oss.str("");
  oss << std::fixed << std::setprecision(3) << "u error: " << u_err_mean << " (mean) " << u_err_med
    << " (median) " << u_err_std << " (std)";
  vpDisplay::displayText(I_err_imPt, offset_text+offset_text_incr, graph_offset+20, oss.str(), color_text);

  oss.str("");
  oss << std::fixed << std::setprecision(3) << "v error: " << v_err_mean << " (mean) " << v_err_med
    << " (median) " << v_err_std << " (std)";
  vpDisplay::displayText(I_err_imPt, offset_text+2*offset_text_incr, graph_offset+20, oss.str(), color_text);
}

//
// OpenCV calibration from opencv/samples/cpp/calibration.cpp
//
enum CvPattern { CV_CHESSBOARD, CV_CIRCLES_GRID, CV_ASYMMETRIC_CIRCLES_GRID, CV_CHARUCOBOARD };

double computeReprojectionErrors(
        const std::vector<std::vector<cv::Point3f> > &objectPoints,
        const std::vector<std::vector<cv::Point2f> > &imagePoints,
        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, std::vector<std::vector<cv::Point2f>> &imagePoints_proj,
        std::vector<float> &perViewErrors)
{
  imagePoints_proj.resize(objectPoints.size());
  std::vector<cv::Point2f> imagePoints2;
  int totalPoints = 0;
  double totalErr = 0, err = 0;
  perViewErrors.resize(objectPoints.size());

  for (int i = 0; i < (int)objectPoints.size(); i++) {
    projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
                  cameraMatrix, distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
    imagePoints_proj[i] = imagePoints2;
  }

  return std::sqrt(totalErr/totalPoints);
}

void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f> &corners,
    CvPattern patternType = CV_CHESSBOARD)
{
  corners.resize(0);

  switch (patternType) {
  case CV_CHESSBOARD:
  case CV_CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; i++)
      for (int j = 0; j < boardSize.width; j++)
        corners.push_back(cv::Point3f(float(j*squareSize),
                                      float(i*squareSize), 0));
    break;

  case CV_ASYMMETRIC_CIRCLES_GRID:
    for (int i = 0; i < boardSize.height; i++)
      for (int j = 0; j < boardSize.width; j++)
        corners.push_back(cv::Point3f(float((2*j + i % 2)*squareSize),
                                      float(i*squareSize), 0));
    break;

  case CV_CHARUCOBOARD:
    for (int i = 0; i < boardSize.height-1; i++)
      for (int j = 0; j < boardSize.width-1; j++)
        corners.push_back(cv::Point3f(float(j*squareSize),
                                      float(i*squareSize), 0));
    break;
  default:
    throw vpException(vpException::badValue, "Unknown pattern type: " + std::to_string(patternType));
  }
}

void saveCameraParams(const std::string &filename,
                      cv::Size imageSize, cv::Size boardSize,
                      float squareSize, float aspectRatio, int flags,
                      const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                      const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                      const std::vector<float> &reprojErrs,
                      const std::vector<std::vector<cv::Point2f> > &imagePoints,
                      const std::vector<cv::Point3f> &newObjPoints,
                      double totalAvgErr)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  time_t tt;
  time(&tt);
  struct tm *t2 = localtime(&tt);
  char buf[1024];
  strftime(buf, sizeof(buf)-1, "%c", t2);

  fs << "calibration_time" << buf;

  if (!rvecs.empty() || !reprojErrs.empty())
    fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  fs << "board_width" << boardSize.width;
  fs << "board_height" << boardSize.height;
  fs << "square_size" << squareSize;

  if (flags & cv::CALIB_FIX_ASPECT_RATIO)
    fs << "aspectRatio" << aspectRatio;

  if (flags != 0) {
    snprintf(buf, sizeof(buf), "flags: %s%s%s%s",
        flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
        flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
        flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
        flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    //cvWriteComment( *fs, buf, 0 );
  }

  fs << "flags" << flags;

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;
  if (!reprojErrs.empty())
    fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

  if (!rvecs.empty() && !tvecs.empty()) {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
    for (int i = 0; i < (int)rvecs.size(); i++) {
      cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0, 3));
      cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3, 6));

      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //*.t() is MatExpr (not Mat) so we can use assignment operator
      r = rvecs[i].t();
      t = tvecs[i].t();
    }
    //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
    fs << "extrinsic_parameters" << bigmat;
  }

  if (!imagePoints.empty()) {
    cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for (int i = 0; i < (int)imagePoints.size(); i++) {
      cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      cv::Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points" << imagePtMat;
  }

  if (!newObjPoints.empty()) {
    fs << "grid_points" << newObjPoints;
  }
}

bool runCalibration(const std::vector<std::vector<cv::Point2f>> &imagePoints,
                    cv::Size imageSize, cv::Size boardSize, CvPattern patternType,
                    float squareSize, float aspectRatio,
                    float grid_width, bool release_object,
                    int flags, cv::Mat &cameraMatrix, cv::Mat &distCoeffs,
                    std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs,
                    std::vector<float> &reprojErrs,
                    std::vector<cv::Point3f> &newObjPoints,
                    std::vector<std::vector<cv::Point2f>> &imagePoints_proj,
                    double &totalAvgErr, Tee &tee)
{
  if (flags & cv::CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = aspectRatio;

  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  std::vector<std::vector<cv::Point3f> > objectPoints(1);
  calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
  // Board imperfectness correction introduced in PR #12772
  // The correction does not make sense for asymmetric and assymetric circles grids
  if (patternType == CV_CHESSBOARD) {
    int offset = boardSize.width - 1;
    objectPoints[0][offset].x = objectPoints[0][0].x + grid_width;
  }
  else if (patternType == CV_CHARUCOBOARD) {
    int offset = boardSize.width - 2;
    objectPoints[0][offset].x = objectPoints[0][0].x + grid_width;
  }

  newObjPoints = objectPoints[0];

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  double rms;
  int iFixedPoint = -1;
  if (release_object)
    iFixedPoint = boardSize.width - 1;
  rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                          cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                          flags | cv::CALIB_USE_LU);
  // printf("RMS error reported by calibrateCamera: %g\n", rms);
  tee << "RMS error reported by OpenCV calibrateCameraRO(): " << rms << "\n";

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  if (release_object) {
    tee << "New board corners: " << "\n";
    tee << newObjPoints[0] << "\n";
    tee << newObjPoints[boardSize.width - 1] << "\n";
    tee << newObjPoints[boardSize.width * (boardSize.height - 1)] << "\n";
    tee << newObjPoints.back() << "\n";
  }

  objectPoints.clear();
  objectPoints.resize(imagePoints.size(), newObjPoints);
  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
              rvecs, tvecs, cameraMatrix, distCoeffs, imagePoints_proj, reprojErrs);

  return ok;
}

bool runAndSave(const std::string &outputFilename,
                const std::vector<std::vector<cv::Point2f> > &imagePoints,
                std::vector<std::vector<cv::Point2f>> &imagePoints_proj,
                cv::Size imageSize, cv::Size boardSize, CvPattern patternType, float squareSize,
                float grid_width, bool release_object,
                float aspectRatio, int flags, cv::Mat &cameraMatrix,
                cv::Mat &distCoeffs, std::vector<float> &reprojErrs,
                bool writeExtrinsics, bool writePoints, bool writeGrid,
                std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs,
                Tee &tee)
{
  double totalAvgErr = 0;
  std::vector<cv::Point3f> newObjPoints;

  bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                 aspectRatio, grid_width, release_object, flags, cameraMatrix, distCoeffs,
                 rvecs, tvecs, reprojErrs, newObjPoints, imagePoints_proj, totalAvgErr, tee);
  // printf("%s. avg reprojection error = %.7f\n",
  //        ok ? "Calibration succeeded" : "Calibration failed",
  //        totalAvgErr);
  tee << (ok ? "OpenCV Calibration succeeded. " : "OpenCV Calibration failed. ")
    << "Avg reprojection error = " << totalAvgErr << "\n";

  if (ok)
    saveCameraParams(outputFilename, imageSize,
                     boardSize, squareSize, aspectRatio,
                     flags, cameraMatrix, distCoeffs,
                     writeExtrinsics ? rvecs : std::vector<cv::Mat>(),
                     writeExtrinsics ? tvecs : std::vector<cv::Mat>(),
                     writeExtrinsics ? reprojErrs : std::vector<float>(),
                     writePoints ? imagePoints : std::vector<std::vector<cv::Point2f> >(),
                     writeGrid ? newObjPoints : std::vector<cv::Point3f>(),
                     totalAvgErr);
  return ok;
}

std::vector<vpImagePoint> cv_undistort(const cv::Mat &cv_cam, const cv::Mat &cv_dist,
  const std::vector<vpImagePoint> &imPts)
{
  std::vector<vpImagePoint> imPts_undist;
  std::vector<cv::Point2d> cv_imPts, cv_imPts_undist;
  imPts_undist.reserve(imPts.size());
  cv_imPts.reserve(imPts.size());
  cv_imPts_undist.reserve(imPts.size());

  for (size_t i = 0; i < imPts.size(); i++) {
    cv_imPts.push_back(cv::Point2d(imPts[i].get_u(), imPts[i].get_v()));
  }
  cv::undistortPoints(cv_imPts, cv_imPts_undist, cv_cam, cv_dist);

  for (size_t i = 0; i < cv_imPts_undist.size(); i++) {
    imPts_undist.push_back(vpImagePoint(cv_imPts_undist[i].y, cv_imPts_undist[i].x));
  }

  return imPts_undist;
}

void undistortDistort(const cv::Mat &cam, const cv::Mat &dist, double j, double i, double &jd, double &id)
{
  double inv_px = 1 / cam.at<double>(0, 0);
  double inv_py = 1 / cam.at<double>(1, 1);

  double x = 0, y = 0;
  x = (j - cam.at<double>(0, 2)) * inv_px;
  y = (i - cam.at<double>(1, 2)) * inv_py;

  vpMeterPixelConversion::convertPoint(cam, dist, x, y, jd, id);
}

void computeDistortionDisplacementMap(const cv::Mat &cam, const cv::Mat &dist, vpImage<vpRGBa> &I_color)
{
  assert(cam.type() == CV_64F);
  assert(dist.type() == CV_64F);
  vpImage<double> displacement_map(I_color.getHeight(), I_color.getWidth());

  double min_displ = 1e12, max_displ = 0;
  for (unsigned int i = 0; i < I_color.getHeight(); i++) {
    for (unsigned int j = 0; j < I_color.getWidth(); j++) {
      double jd = 0, id = 0;
      undistortDistort(cam, dist, j, i, jd, id);

      double erri = id - i;
      double errj = jd - j;
      double displ = std::sqrt(erri*erri + errj*errj);
      displacement_map[i][j] = displ;
      min_displ = displ < min_displ ? displ : min_displ;
      max_displ = displ > max_displ ? displ : max_displ;
    }
  }

  double a = 255 / (max_displ - min_displ);
  double b = (-255 * min_displ) / (max_displ - min_displ);

  vpImage<unsigned char> I_gray(I_color.getHeight(), I_color.getWidth());
  for (unsigned int i = 0; i < displacement_map.getHeight(); i++) {
    for (unsigned int j = 0; j < displacement_map.getWidth(); j++) {
      I_gray[i][j] = static_cast<unsigned char>(vpMath::clamp(a * displacement_map[i][j] + b, 0.0, 255.0));
    }
  }

  vpColormap colormap(vpColormap::COLORMAP_TURBO);
  colormap.convert(I_gray, I_color);
}

double getProjectionErrorUV(const cv::Mat &cam, const cv::Mat &dist, const std::vector<CalibInfo> &calib_info,
  std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs, std::vector<std::vector<vpImagePoint>> &err_imPt_imgs,
  std::vector<vpImagePoint> &err_imPt, bool with_dist)
{
  assert(!calib_info.empty());
  double max_scale_uv = -1e6;
  err_imPt_imgs.reserve(calib_info.size());
  err_imPt.reserve(calib_info.size()*calib_info[0].m_points.size());

  std::vector<cv::Point3d> obj_pts;
  obj_pts.reserve(calib_info[0].m_points.size());
  for (size_t j = 0; j < calib_info[0].m_points.size(); j++) {
    cv::Point3d obj_pt(
      calib_info[0].m_points[j].get_oX(),
      calib_info[0].m_points[j].get_oY(),
      calib_info[0].m_points[j].get_oZ()
    );
    obj_pts.push_back(obj_pt);
  }

  for (size_t i = 0; i < calib_info.size(); i++) {
    const CalibInfo &calib_info_cur = calib_info[i];
    std::vector<vpImagePoint> err_imPt_per_img;
    err_imPt_per_img.reserve(calib_info_cur.m_points.size());

    std::vector<cv::Point2d> img_pts;
    img_pts.reserve(calib_info_cur.m_points.size());

    cv::projectPoints(obj_pts, rvecs[i], tvecs[i], cam, with_dist ? dist : cv::noArray(), img_pts);

    for (size_t j = 0; j < calib_info_cur.m_points.size(); j++) {
      vpImagePoint pt_proj(img_pts[j].y, img_pts[j].x);

      err_imPt_per_img.push_back(calib_info_cur.m_imPts[j] - pt_proj);
      err_imPt.push_back(calib_info_cur.m_imPts[j] - pt_proj);

      double err_u = std::fabs(err_imPt_per_img.back().get_u());
      double err_v = std::fabs(err_imPt_per_img.back().get_v());
      max_scale_uv = err_u > max_scale_uv ? err_u : max_scale_uv;
      max_scale_uv = err_v > max_scale_uv ? err_v : max_scale_uv;
    }

    err_imPt_imgs.push_back(err_imPt_per_img);
  }

  return max_scale_uv;
}

} // namespace calib_helper

#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif // VISP_HAVE_OPENCV
