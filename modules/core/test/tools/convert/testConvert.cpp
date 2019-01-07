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
 * Test functions in vpIoTools.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!

  \example testConvert.cpp

  \brief Test functions in Convert.

*/

#include <iostream> // std::cout
#include <limits>   // std::numeric_limits
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpConvert.h>

bool areSame(double a, double b) { return fabs(a - b) < std::numeric_limits<double>::epsilon(); }

void testConvertFromImagePointToPoint2f()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImagePoint imPt1(12.5f, .85f);
  vpImagePoint imPt2(-44.26f, 125.11f);
  vpImagePoint imPt3(0.0f, -1.756e-10f);

  cv::Point2f pt1, pt2, pt3;
  vpConvert::convertToOpenCV(imPt1, pt1);
  vpConvert::convertToOpenCV(imPt2, pt2);
  vpConvert::convertToOpenCV(imPt3, pt3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(imPt1.get_u(), pt1.x) && areSame(imPt1.get_v(), pt1.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt2.get_u(), pt2.x) && areSame(imPt2.get_v(), pt2.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt3.get_u(), pt3.x) && areSame(imPt3.get_v(), pt3.y))
    nbOk++;
  else
    nbNOk++;

  std::vector<vpImagePoint> listOfImPts(3);
  listOfImPts[0] = imPt1;
  listOfImPts[1] = imPt2;
  listOfImPts[2] = imPt3;

  std::vector<cv::Point2f> listOfPts;
  vpConvert::convertToOpenCV(listOfImPts, listOfPts);

  if (listOfImPts.size() == listOfPts.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfImPts[i].get_u(), listOfPts[i].x) && areSame(listOfImPts[i].get_v(), listOfPts[i].y))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromImagePointToPoint2f=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

void testConvertFromPoint2fToImagePoint()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImagePoint imPt1, imPt2, imPt3;

  cv::Point2f pt1(12.5f, .85f), pt2(-44.26f, 125.11f), pt3(0.0f, -1.756e-10f);
  vpConvert::convertFromOpenCV(pt1, imPt1);
  vpConvert::convertFromOpenCV(pt2, imPt2);
  vpConvert::convertFromOpenCV(pt3, imPt3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(imPt1.get_u(), pt1.x) && areSame(imPt1.get_v(), pt1.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt2.get_u(), pt2.x) && areSame(imPt2.get_v(), pt2.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt3.get_u(), pt3.x) && areSame(imPt3.get_v(), pt3.y))
    nbOk++;
  else
    nbNOk++;

  std::vector<vpImagePoint> listOfImPts;

  std::vector<cv::Point2f> listOfPts(3);
  listOfPts[0] = pt1;
  listOfPts[1] = pt2;
  listOfPts[2] = pt3;

  vpConvert::convertFromOpenCV(listOfPts, listOfImPts);

  if (listOfImPts.size() == listOfPts.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfImPts[i].get_u(), listOfPts[i].x) && areSame(listOfImPts[i].get_v(), listOfPts[i].y))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromPoint2fToImagePoint=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

void testConvertFromImagePointToPoint2d()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImagePoint imPt1(12.5, .85);
  vpImagePoint imPt2(-44.26, 125.11);
  vpImagePoint imPt3(0, -1.756e-10);

  cv::Point2d pt1, pt2, pt3;
  vpConvert::convertToOpenCV(imPt1, pt1);
  vpConvert::convertToOpenCV(imPt2, pt2);
  vpConvert::convertToOpenCV(imPt3, pt3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(imPt1.get_u(), pt1.x) && areSame(imPt1.get_v(), pt1.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt2.get_u(), pt2.x) && areSame(imPt2.get_v(), pt2.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt3.get_u(), pt3.x) && areSame(imPt3.get_v(), pt3.y))
    nbOk++;
  else
    nbNOk++;

  std::vector<vpImagePoint> listOfImPts(3);
  listOfImPts[0] = imPt1;
  listOfImPts[1] = imPt2;
  listOfImPts[2] = imPt3;

  std::vector<cv::Point2d> listOfPts;
  vpConvert::convertToOpenCV(listOfImPts, listOfPts);

  if (listOfImPts.size() == listOfPts.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfImPts[i].get_u(), listOfPts[i].x) && areSame(listOfImPts[i].get_v(), listOfPts[i].y))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromImagePointToPoint2d=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

void testConvertFromPoint2dToImagePoint()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImagePoint imPt1, imPt2, imPt3;

  cv::Point2d pt1(12.5, .85), pt2(-44.26, 125.11), pt3(0, -1.756e-10);
  vpConvert::convertFromOpenCV(pt1, imPt1);
  vpConvert::convertFromOpenCV(pt2, imPt2);
  vpConvert::convertFromOpenCV(pt3, imPt3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(imPt1.get_u(), pt1.x) && areSame(imPt1.get_v(), pt1.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt2.get_u(), pt2.x) && areSame(imPt2.get_v(), pt2.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt3.get_u(), pt3.x) && areSame(imPt3.get_v(), pt3.y))
    nbOk++;
  else
    nbNOk++;

  std::vector<vpImagePoint> listOfImPts;

  std::vector<cv::Point2d> listOfPts(3);
  listOfPts[0] = pt1;
  listOfPts[1] = pt2;
  listOfPts[2] = pt3;

  vpConvert::convertFromOpenCV(listOfPts, listOfImPts);

  if (listOfImPts.size() == listOfPts.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfImPts[i].get_u(), listOfPts[i].x) && areSame(listOfImPts[i].get_v(), listOfPts[i].y))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromPoint2dToImagePoint=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

void testConvertFromKeyPointToImagePoint()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  cv::KeyPoint kp1(12.5f, .85f, 0), kp2(-44.26f, 125.11f, 0), kp3(0.0f, -1.756e-10f, 0);
  vpImagePoint imPt1, imPt2, imPt3;

  vpConvert::convertFromOpenCV(kp1, imPt1);
  vpConvert::convertFromOpenCV(kp2, imPt2);
  vpConvert::convertFromOpenCV(kp3, imPt3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(imPt1.get_u(), kp1.pt.x) && areSame(imPt1.get_v(), kp1.pt.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt2.get_u(), kp2.pt.x) && areSame(imPt2.get_v(), kp2.pt.y))
    nbOk++;
  else
    nbNOk++;
  if (areSame(imPt3.get_u(), kp3.pt.x) && areSame(imPt3.get_v(), kp3.pt.y))
    nbOk++;
  else
    nbNOk++;

  std::vector<cv::KeyPoint> listOfKeyPoints(3);
  listOfKeyPoints[0] = kp1;
  listOfKeyPoints[1] = kp2;
  listOfKeyPoints[2] = kp3;

  std::vector<vpImagePoint> listOfImPts;
  vpConvert::convertFromOpenCV(listOfKeyPoints, listOfImPts);

  if (listOfImPts.size() == listOfKeyPoints.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfImPts[i].get_u(), listOfKeyPoints[i].pt.x) &&
          areSame(listOfImPts[i].get_v(), listOfKeyPoints[i].pt.y))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromKeyPointToImagePoint=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

void testConvertFromPoint3fToPoint()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  cv::Point3f pt1(12.5f, .85f, 110.0f), pt2(-44.26f, 125.11f, -98e2f), pt3(0.0f, -1.756e-10f, 0.00015f);
  vpPoint point1, point2, point3;

  vpConvert::convertFromOpenCV(pt1, point1);
  vpConvert::convertFromOpenCV(pt2, point2);
  vpConvert::convertFromOpenCV(pt3, point3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(pt1.x, point1.get_oX()) && areSame(pt1.y, point1.get_oY()) && areSame(pt1.z, point1.get_oZ()))
    nbOk++;
  else
    nbNOk++;
  if (areSame(pt2.x, point2.get_oX()) && areSame(pt2.y, point2.get_oY()) && areSame(pt2.z, point2.get_oZ()))
    nbOk++;
  else
    nbNOk++;
  if (areSame(pt3.x, point3.get_oX()) && areSame(pt3.y, point3.get_oY()) && areSame(pt3.z, point3.get_oZ()))
    nbOk++;
  else
    nbNOk++;

  std::vector<cv::Point3f> listOfPoints3f(3);
  listOfPoints3f[0] = pt1;
  listOfPoints3f[1] = pt2;
  listOfPoints3f[2] = pt3;

  std::vector<vpPoint> listOfPts;
  vpConvert::convertFromOpenCV(listOfPoints3f, listOfPts);

  if (listOfPoints3f.size() == listOfPts.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfPts[i].get_oX(), listOfPoints3f[i].x) && areSame(listOfPts[i].get_oY(), listOfPoints3f[i].y) &&
          areSame(listOfPts[i].get_oZ(), listOfPoints3f[i].z))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromPoint3fToPoint=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

void testConvertFromPointToPoint3f()
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  cv::Point3f pt1, pt2, pt3;
  vpPoint point1, point2, point3;
  point1.set_oX(12.5f);
  point1.set_oY(.85f);
  point1.set_oZ(110.0f);

  point2.set_oX(-44.26f);
  point2.set_oY(125.11f);
  point2.set_oZ(-98e2f);

  point3.set_oX(0.0f);
  point3.set_oY(-1.756e-10f);
  point3.set_oZ(0.00015f);

  vpConvert::convertToOpenCV(point1, pt1);
  vpConvert::convertToOpenCV(point2, pt2);
  vpConvert::convertToOpenCV(point3, pt3);

  int nbOk = 0, nbNOk = 0;
  if (areSame(pt1.x, point1.get_oX()) && areSame(pt1.y, point1.get_oY()) && areSame(pt1.z, point1.get_oZ()))
    nbOk++;
  else
    nbNOk++;
  if (areSame(pt2.x, point2.get_oX()) && areSame(pt2.y, point2.get_oY()) && areSame(pt2.z, point2.get_oZ()))
    nbOk++;
  else
    nbNOk++;
  if (areSame(pt3.x, point3.get_oX()) && areSame(pt3.y, point3.get_oY()) && areSame(pt3.z, point3.get_oZ()))
    nbOk++;
  else
    nbNOk++;

  std::vector<cv::Point3f> listOfPoints3f;
  std::vector<vpPoint> listOfPts(3);
  listOfPts[0] = point1;
  listOfPts[1] = point2;
  listOfPts[2] = point3;

  vpConvert::convertToOpenCV(listOfPts, listOfPoints3f);

  if (listOfPoints3f.size() == listOfPts.size()) {
    for (size_t i = 0; i < 3; i++) {
      if (areSame(listOfPts[i].get_oX(), listOfPoints3f[i].x) && areSame(listOfPts[i].get_oY(), listOfPoints3f[i].y) &&
          areSame(listOfPts[i].get_oZ(), listOfPoints3f[i].z))
        nbOk++;
      else
        nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromPointToPoint3f=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

int main()
{
  testConvertFromImagePointToPoint2f();
  testConvertFromPoint2fToImagePoint();
  testConvertFromImagePointToPoint2d();
  testConvertFromPoint2dToImagePoint();

  testConvertFromKeyPointToImagePoint();
  testConvertFromPoint3fToPoint();
  testConvertFromPointToPoint3f();
  return 0;
}
