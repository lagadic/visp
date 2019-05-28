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
 * Performs various tests on the vpPixelMeterConversion and
 * vpPixelMeterConversion class.
 *
 * Authors:
 * Anthony saunier
 *
 *****************************************************************************/

/*!
  \file testCameraParametersConversion.cpp

  Performs various tests on the vpPixelMeterConversion and
  vpPixelMeterConversion class.
*/

#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>

int main()
{
  try {
    {
      std::cout << "* Test operator=()" << std::endl;
      vpCameraParameters cam1, cam2;
      cam1.initPersProjWithDistortion(700.0, 700.0, 320.0, 240.0, 0.1, 0.1);
      cam2.initPersProjWithoutDistortion(700.0, 700.0, 320.0, 240.0);
      if (cam1 == cam2) {
        std::cerr << "Issue with vpCameraParameters comparison operator." << std::endl;
        return EXIT_FAILURE;
      }

      cam2 = cam1;
      if (cam1 != cam2) {
        std::cerr << "Issue with vpCameraParameters comparison operator." << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "* Test computeFov()" << std::endl;
      cam2.computeFov(640u, 480u);
      if (cam1 == cam2) {
        std::cerr << "Issue with vpCameraParameters comparison operator." << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam;
    double px, py, u0, v0;
    px = 1657.429131;
    py = 1658.818598;
    u0 = 322.2437833;
    v0 = 230.8012737;
    vpCameraParameters camDist;
    double px_dist, py_dist, u0_dist, v0_dist, kud_dist, kdu_dist;
    px_dist = 1624.824731;
    py_dist = 1625.263641;
    u0_dist = 324.0923411;
    v0_dist = 245.2421388;
    kud_dist = -0.1741532338;
    kdu_dist = 0.1771165148;

    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    camDist.initPersProjWithDistortion(px_dist, py_dist, u0_dist, v0_dist, kud_dist, kdu_dist);

    double u1 = 320;
    double v1 = 240;
    double x1 = 0, y1 = 0;
    double u2 = 0, v2 = 0;
    std::cout << "* Test point conversion without distorsion" << std::endl;
    vpPixelMeterConversion::convertPoint(cam, u1, v1, x1, y1);
    vpMeterPixelConversion::convertPoint(cam, x1, y1, u2, v2);
    if (!vpMath::equal(u1, u2) || !vpMath::equal(v1, v2)) {
      std::cerr << "Error in point conversion without distortion:\n"
                << "u1 = " << u1 << ", u2 = " << u2 << std::endl
                << "v1 = " << v1 << ", v2 = " << v2 << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "* Test point conversion with distorsion" << std::endl;
    vpPixelMeterConversion::convertPoint(camDist, u1, v1, x1, y1);
    vpMeterPixelConversion::convertPoint(camDist, x1, y1, u2, v2);
    if (!vpMath::equal(u1, u2) || !vpMath::equal(v1, v2)) {
      std::cerr << "Error in point conversion without distortion:\n"
                << "u1 = " << u1 << ", u2 = " << u2 << std::endl
                << "v1 = " << v1 << ", v2 = " << v2 << std::endl;
      return EXIT_FAILURE;
    }

#if VISP_HAVE_OPENCV_VERSION >= 0x020300
    {
      std::cout << "* Compare ViSP and OpenCV point pixel meter conversion without distorsion" << std::endl;
      cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                        0, py, v0,
                                                        0,  0,  1);
      cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64FC1);
      double x2, y2, u2, v2;

      vpPixelMeterConversion::convertPoint(cam, u1, v1, x1, y1);
      vpPixelMeterConversion::convertPoint(cameraMatrix, distCoeffs, u1, v1, x2, y2);
      if ( !vpMath::equal(x1, x2, 1e-6) || !vpMath::equal(y1, y2, 1e-6)) {
        std::cerr << "Error in point pixel meter conversion: visp result (" << x1 << ", " << y1 << ") "
                  << "differ from OpenCV result (" << x2 << ", " << y2 << ")" << std::endl;
        return EXIT_FAILURE;
      }

      vpImagePoint ip(v1, u1);
      vpPixelMeterConversion::convertPoint(cam, ip, x1, y1);
      vpPixelMeterConversion::convertPoint(cameraMatrix, distCoeffs, ip, x2, y2);
      if ( !vpMath::equal(x1, x2, 1e-6) || !vpMath::equal(y1, y2, 1e-6)) {
        std::cerr << "Error in point pixel meter conversion: visp result (" << x1 << ", " << y1 << ") "
                  << "differ from OpenCV result (" << x2 << ", " << y2 << ")" << std::endl;
        return EXIT_FAILURE;
      }


      std::cout << "* Compare ViSP and OpenCV point meter pixel conversion without distorsion" << std::endl;
      vpMeterPixelConversion::convertPoint(cam, x1, y1, u1, v1);
      vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, x1, y1, u2, v2);
      if ( !vpMath::equal(u1, u2, 1e-6) || !vpMath::equal(v1, v2, 1e-6)) {
        std::cerr << "Error in point meter pixel conversion: visp result (" << u1 << ", " << v1 << ") "
                  << "differ from OpenCV result (" << u2 << ", " << v2 << ")" << std::endl;
        return EXIT_FAILURE;
      }

      vpImagePoint iP1, iP2;
      vpMeterPixelConversion::convertPoint(cam, x1, y1, iP1);
      vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, x1, y1, iP2);
      if ( vpImagePoint::distance(iP1, iP2) > 1e-6) {
        std::cerr << "Error in point meter pixel conversion: visp result (" << u1 << ", " << v1 << ") "
                  << "differ from OpenCV result (" << u2 << ", " << v2 << ")" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::cout << "* Compare ViSP and OpenCV point pixel meter conversion with distorsion" << std::endl;
      cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px_dist,       0, u0_dist,
                                                             0, py_dist, v0_dist,
                                                             0,       0,       1);
      cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64FC1);
      distCoeffs.at<double>(0,0) = kdu_dist;
      double x2, y2;

      vpPixelMeterConversion::convertPoint(camDist, u1, v1, x1, y1);
      vpPixelMeterConversion::convertPoint(cameraMatrix, distCoeffs, u1, v1, x2, y2);
      if ( !vpMath::equal(x1, x2, 1e-6) || !vpMath::equal(y1, y2, 1e-6)) {
        std::cerr << "Error in point conversion: visp result (" << x1 << ", " << y1 << ") "
                  << "differ from OpenCV result (" << x2 << ", " << y2 << ")" << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "* Compare ViSP and OpenCV point meter pixel conversion with distorsion" << std::endl;
      distCoeffs.at<double>(0,0) = kud_dist;
      vpMeterPixelConversion::convertPoint(camDist, x1, y1, u1, v1);
      vpMeterPixelConversion::convertPoint(cameraMatrix, distCoeffs, x1, y1, u2, v2);
      if ( !vpMath::equal(u1, u2, 1e-6) || !vpMath::equal(v1, v2, 1e-6)) {
        std::cerr << "Error in point meter pixel conversion: visp result (" << u1 << ", " << v1 << ") "
                  << "differ from OpenCV result (" << u2 << ", " << v2 << ")" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::cout << "* Compare ViSP and OpenCV line pixel meter conversion without distorsion" << std::endl;
      cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                        0, py, v0,
                                                        0,  0,  1);
      double rho_p = 100, theta_p = vpMath::rad(45);
      double rho_m1, theta_m1, rho_m2, theta_m2;

      vpPixelMeterConversion::convertLine(cam, rho_p, theta_p, rho_m1, theta_m1);
      vpPixelMeterConversion::convertLine(cameraMatrix, rho_p, theta_p, rho_m2, theta_m2);
      if ( !vpMath::equal(rho_m1, rho_m2, 1e-6) || !vpMath::equal(theta_m1, theta_m2, 1e-6)) {
        std::cerr << "Error in line pixel meter conversion: visp result (" << rho_m1 << ", " << theta_m1 << ") "
                  << "differ from OpenCV result (" << rho_m2 << ", " << theta_m1 << ")" << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "* Compare ViSP and OpenCV line meter pixel conversion without distorsion" << std::endl;
      double rho_p1, theta_p1, rho_p2, theta_p2;
      vpMeterPixelConversion::convertLine(cam, rho_m1, theta_m1, rho_p1, theta_p1);
      vpMeterPixelConversion::convertLine(cameraMatrix, rho_m1, theta_m1, rho_p2, theta_p2);
      if ( !vpMath::equal(rho_p1, rho_p2, 1e-6) || !vpMath::equal(theta_p1, theta_p2, 1e-6)) {
        std::cerr << "Error in line meter pixel conversion: visp result (" << rho_p1 << ", " << theta_p1 << ") "
                  << "differ from OpenCV result (" << rho_p2 << ", " << theta_p1 << ")" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::cout << "* Compare ViSP and OpenCV moments pixel meter conversion without distorsion" << std::endl;
      cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                        0, py, v0,
                                                        0,  0,  1);
      unsigned int order = 3;
      double m00 = 2442, m10 = 414992, m01 = 470311, m11 = 7.99558e+07, m02 = 9.09603e+07, m20 = 7.11158e+07;

      vpMatrix mp(order, order);
      vpMatrix m1(order, order), m2(order, order);

      mp[0][0] = m00;
      mp[1][0] = m10;
      mp[0][1] = m01;
      mp[2][0] = m20;
      mp[1][1] = m11;
      mp[0][2] = m02;

      vpPixelMeterConversion::convertMoment(cam, order, mp, m1);
      vpPixelMeterConversion::convertMoment(cameraMatrix, order, mp, m2);
      for (unsigned int i = 0; i < m1.getRows(); i ++) {
        for (unsigned int j = 0; j < m1.getCols(); j ++) {
          if ( !vpMath::equal(m1[i][j], m1[i][j], 1e-6) ) {
            std::cerr << "Error in moments pixel meter conversion: visp result for ["<< i << "]["<< j << "] (" << m1[i][j] << ") "
                      << "differ from OpenCV result (" << m2[i][j] << ")" << std::endl;
            return EXIT_FAILURE;
          }
        }
      }
    }

    {
      std::cout << "* Compare ViSP and OpenCV ellipse from circle meter pixel conversion without distorsion" << std::endl;
      cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << px,  0, u0,
                                                        0, py, v0,
                                                        0,  0,  1);
      vpCircle circle;
      circle.setWorldCoordinates(0, 0, 1, 0, 0, 0, 0.1); // plane:(Z=0),X0=0,Y0=0,Z=0,R=0.1
      vpHomogeneousMatrix cMo(0.1, 0.2, 0.5, vpMath::rad(10), vpMath::rad(5), vpMath::rad(45));
      circle.changeFrame(cMo);
      circle.projection();
      vpImagePoint center_p1, center_p2;
      double mu20_p1, mu11_p1, mu02_p1, mu20_p2, mu11_p2, mu02_p2;

      vpMeterPixelConversion::convertEllipse(cam, circle, center_p1, mu20_p1, mu11_p1, mu02_p1);
      vpMeterPixelConversion::convertEllipse(cameraMatrix, circle, center_p2, mu20_p2, mu11_p2, mu02_p2);

      if ( !vpMath::equal(mu20_p1, mu20_p2, 1e-6) || !vpMath::equal(mu11_p1, mu11_p2, 1e-6) || !vpMath::equal(mu02_p1, mu02_p2, 1e-6)) {
        std::cerr << "Error in ellipse from circle meter pixel conversion: visp result (" << mu20_p1 << ", " << mu11_p1 << ", " << mu02_p1 << ") "
                  << "differ from OpenCV result (" << mu20_p2 << ", " << mu11_p2 << ", " << mu02_p2 << ")" << std::endl;
        return EXIT_FAILURE;
      }
      if ( vpImagePoint::distance(center_p1, center_p2) > 1e-6) {
        std::cerr << "Error in ellipse from circle meter pixel conversion: visp result (" << center_p1 << ") "
                  << "differ from OpenCV result (" << center_p2 << ")" << std::endl;
        return EXIT_FAILURE;
      }

      std::cout << "* Compare ViSP and OpenCV ellipse from sphere meter pixel conversion without distorsion" << std::endl;
      vpSphere sphere;
      sphere.setWorldCoordinates(0, 0, 0, 0.1); // X0=0,Y0=0,Z0=0,R=0.1
      circle.changeFrame(cMo);
      circle.projection();
      vpMeterPixelConversion::convertEllipse(cam, sphere, center_p1, mu20_p1, mu11_p1, mu02_p1);
      vpMeterPixelConversion::convertEllipse(cameraMatrix, sphere, center_p2, mu20_p2, mu11_p2, mu02_p2);

      if ( !vpMath::equal(mu20_p1, mu20_p2, 1e-6) || !vpMath::equal(mu11_p1, mu11_p2, 1e-6) || !vpMath::equal(mu02_p1, mu02_p2, 1e-6)) {
        std::cerr << "Error in ellipse from sphere meter pixel conversion: visp result (" << mu20_p1 << ", " << mu11_p1 << ", " << mu02_p1 << ") "
                  << "differ from OpenCV result (" << mu20_p2 << ", " << mu11_p2 << ", " << mu02_p2 << ")" << std::endl;
        return EXIT_FAILURE;
      }
      if ( vpImagePoint::distance(center_p1, center_p2) > 1e-6) {
        std::cerr << "Error in ellipse from sphere meter pixel conversion: visp result (" << center_p1 << ") "
                  << "differ from OpenCV result (" << center_p2 << ")" << std::endl;
        return EXIT_FAILURE;
      }
    }
#endif

    std::cout << "Test succesful" << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
