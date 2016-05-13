/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test RealSense RGB-D sensor.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRealSense.cpp
  This example shows how to retrieve data from a RealSense RGB-D sensor.

*/

#include <iostream>

#include <visp3/sensor/vpRealSense.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>

//#undef VISP_HAVE_PCL

#ifdef VISP_HAVE_PCL
#  include <pcl/visualization/cloud_viewer.h>
#  include <pcl/visualization/pcl_visualizer.h>
#endif

int main()
{
#ifdef VISP_HAVE_REALSENSE
  try {
    vpRealSense rs;
    //rs.setDeviceBySerialNumber("541142003219");
    rs.open();

    vpImage<vpRGBa> color(rs.getIntrinsics()[RS_STREAM_COLOR].height, rs.getIntrinsics()[RS_STREAM_COLOR].width);
    vpImage<u_int16_t> infrared;
    vpImage<unsigned char> infrared_display(rs.getIntrinsics()[RS_STREAM_INFRARED].height, rs.getIntrinsics()[RS_STREAM_INFRARED].width);;
    vpImage<u_int16_t> depth;
    vpImage<vpRGBa> depth_display(rs.getIntrinsics()[RS_STREAM_DEPTH].height, rs.getIntrinsics()[RS_STREAM_DEPTH].width);

#ifdef VISP_HAVE_PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
#else
    std::vector<vpPoint3dTextured> pointcloud;
#endif

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(color, 10, 10, "Color image");
    vpDisplayX di(infrared_display, color.getWidth()+80, 10, "Infrared image");
    vpDisplayX dd(depth_display, 10, color.getHeight()+80, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(color, 10, 10, "Color image");
    vpDisplayGDI di(infrared_display, color.getWidth()+80, 10, "Infrared image");
    vpDisplayGDI dd(depth_display, 10, color.getHeight()+80, "Depth image");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    while(1) {
      rs.acquire(color, infrared, depth, pointcloud);

#ifdef VISP_HAVE_PCL
      static bool update = false;
      if (! update) {
        viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
        update = true;
      }
      else {
        viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
      }

      viewer->spinOnce (100);
#endif

      vpImageConvert::convert(infrared, infrared_display);
      vpImageConvert::createDepthHistogram(depth, depth_display);

      vpDisplay::display(color);
      vpDisplay::display(infrared_display);
      vpDisplay::display(depth_display);

#ifdef REMOVE

#ifdef VISP_HAVE_PCL
      //viewer.showCloud(pointcloud);
//      std::cout << "------- Cloud size: " << pointcloud->width << " " << pointcloud->height << std::endl;
//      for (unsigned int i=0; i< pointcloud->width*pointcloud->height; i++) {
//        std::cout << "[" << pointcloud->points[i].x << " " << pointcloud->points[i].y << " "
//                  << pointcloud->points[i].z << "] ";
//      }
//      std::cout << std::endl;
#else
      std::cout << "Cloud size: " << pointcloud.size() << std::endl;
      if (1){
        // Solve AX = b
        vpMatrix A(pointcloud.size(), 3);
        vpColVector X(3);
        vpColVector b(pointcloud.size(), -1);
        for(unsigned int i=0; i<pointcloud.size(); i++) {
          A[i][0] = pointcloud[i].get_X();
          A[i][1] = pointcloud[i].get_Y();
          A[i][2] = pointcloud[i].get_Z();
        }
        X = A.pseudoInverse() * b;
        vpColVector n = X.normalize();
        std::cout << "Normal to plane: " << n << std::endl;

        // Compute pose of the plane
        vpColVector x(3);
        x[0] = 1;
        vpColVector ry = vpColVector::crossProd(n, x);
        vpColVector rx = vpColVector::crossProd(ry, n);
        vpRotationMatrix R;
        for (unsigned int i=0; i<3; i++) {
          R[i][0] = rx[i];
          R[i][1] = ry[i];
          R[i][2] = n[i];
        }

        // Compute centroid
        vpColVector centroid(3);
        for(unsigned int i=0; i<pointcloud.size(); i++) {
          centroid[0] += pointcloud[i].get_X();
          centroid[1] += pointcloud[i].get_Y();
          centroid[2] += pointcloud[i].get_Z();
        }
        centroid /= pointcloud.size();

        vpTranslationVector t(centroid);
        vpHomogeneousMatrix cMp(t, R);

        std::cout << "cMp:\n" << cMp << std::endl;

        std::vector <rs::intrinsics> intrinsics = rs.getIntrinsics();
        double u0 = intrinsics[(int)rs::stream::depth].ppx;
        double v0 = intrinsics[(int)rs::stream::depth].ppy;
        double px = intrinsics[(int)rs::stream::depth].fx;
        double py = intrinsics[(int)rs::stream::depth].fy;
        std::cout << "Intrinsics: "
                  << "u0 = " << u0 << " "
                  << "v0 = " << v0 << " "
                  << "px = " << px << " "
                  << "py = " << py << std::endl;
        vpCameraParameters cam(px, py, u0, v0);


        vpDisplay::displayFrame(depth_display, cMp, cam, 0.2);
      }
#endif
#endif // #ifdef REMOVE

      vpDisplay::displayText(color, 15, 15, "Click to quit", vpColor::red);
      if (vpDisplay::getClick(color, false) || vpDisplay::getClick(infrared_display, false) || vpDisplay::getClick(depth_display, false))
        break;
      vpDisplay::flush(color);
      vpDisplay::flush(infrared_display);
      vpDisplay::flush(depth_display);
    }

    std::cout << "RealSense sensor characteristics: \n" << rs << std::endl;

    rs.close();
  }
  catch(const vpException &e) {
    std::cerr << "RealSense error " << e.getStringMessage() << std::endl;
  }
  catch(const rs::error & e)  {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
  }
  catch(const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

#else
  std::cout << "Install RealSense SDK to make this test working" << std::endl;
#endif
  return 0;
}

