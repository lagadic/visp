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
 * libStructure interface.
 *
 * Authors:
 * Joudy Nader
 *
*****************************************************************************/

#ifndef _vpOccipitalStructure_h_
#define _vpOccipitalStructure_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_THREADS)
#include <condition_variable>
#include <mutex>

#include <ST/CaptureSession.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
#include <pcl/common/common_headers.h>
#endif

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

BEGIN_VISP_NAMESPACE
 /*!
  \class vpOccipitalStructure

  \ingroup group_sensor_rgbd

  This class provides a wrapper over the Occipital Structure SDK
  library https://structure.io/developers. It allows to capture
  data from the Occipital Structure Core camera.

  \note Supported devices for Occipital Structure SDK 0.9:
  - Occipital Structure Core.

  The usage of vpOccipitalStructure class is enabled when libStructure 3rd party is
  successfully installed. The following tutorials explain how to proceed:
  - \ref tutorial-install-ubuntu
  - \ref tutorial-install-win10-msvc16
  - \ref tutorial-install-osx-homebrew

  Moreover, if Point Cloud Library (PCL) 3rd party is installed, we also
  propose interfaces to retrieve point cloud as pcl::PointCloud<pcl::PointXYZ>
  or pcl::PointCloud<pcl::PointXYZRGB> data structures.

  \warning Notice that the usage of this class requires compiler and library
  support for the ISO C++ 2011 standard. This support is enabled by default
  in ViSP when supported by the compiler. Hereafter we give an example of a
  CMakeLists.txt file that allows to build `sample-structure-core.cpp` that
  uses vpOccipitalStructure class.

  \code
  cmake_minimum_required(VERSION 3.5)

  project(sample)

  find_package(VISP REQUIRED)
  include_directories(${VISP_INCLUDE_DIRS})

  add_executable(sample-structure-core sample-structure-core.cpp)
  target_link_libraries(sample-structure-core ${VISP_LIBRARIES})
  \endcode

  To acquire images from the Structure Core color camera and convert them into grey
  level images, a good starting is to use the following code that corresponds to
  the content of ``sample-structure-core.cpp`:

  \code
  #include <visp3/gui/vpDisplayGDI.h>
  #include <visp3/gui/vpDisplayX.h>
  #include <visp3/sensor/vpOccipitalStructure.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpOccipitalStructure sc;
    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.visibleEnabled = true;

    sc.open(settings);

    vpImage<unsigned char> I(sc.getHeight(vpOccipitalStructure::visible), sc.getWidth(vpOccipitalStructure::visible));
  #ifdef VISP_HAVE_X11
    vpDisplayX d(I);
  #elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
  #endif

    while (true) {
      sc.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
    return 0;
  }
  \endcode

  If you want to acquire color images, in the previous sample replace:
  \code
  vpImage<unsigned char> I(sc.getHeight(vpOccipitalStructure::visible), sc.getWidth(vpOccipitalStructure::visible));
  \endcode
  by
  \code
  vpImage<vpRGBa> I(sc.getHeight(vpOccipitalStructure::visible), sc.getWidth(vpOccipitalStructure::visible));
  \endcode

  If you are interested in the point cloud and if ViSP is build with PCL
  support, you can start from the following example where we use PCL library to
  visualize the point cloud

  \code
  #include <pcl/visualization/cloud_viewer.h>
  #include <pcl/visualization/pcl_visualizer.h>
  #include <visp3/sensor/vpOccipitalStructure.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpOccipitalStructure sc;
    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.visibleEnabled = true;
    settings.applyExpensiveCorrection = true; // Apply a correction and clean filter to the depth before streaming.

    sc.open(settings);
    // Calling these 2 functions to set internal variables.
    sc.getCameraParameters(vpOccipitalStructure::visible);
    sc.getCameraParameters(vpOccipitalStructure::depth);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    sc.acquire(nullptr, nullptr, nullptr, pointcloud);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -0.5, 0, -1, 0);

    while (true) {
      sc.acquire(nullptr, nullptr, nullptr, pointcloud);

      static bool update = false;
      if (!update) {
        viewer->addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        update = true;
      } else {
        viewer->updatePointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
      }

      viewer->spinOnce(30);
    }
    return 0;
  }
  \endcode

  References to `ST::CaptureSession` and `ST::CaptureSessionSettings` can be retrieved
  with (`sc.open() must be called before`):
  \code
  ST::CaptureSession &getCaptureSession();
  ST::CaptureSessionSettings &getCaptureSessionSettings();
  \endcode

 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct SessionDelegate : ST::CaptureSessionDelegate
{
  std::mutex m_sampleLock;
  std::condition_variable cv_sampleLock;

  ST::ColorFrame m_visibleFrame;
  ST::DepthFrame m_depthFrame;
  ST::InfraredFrame m_infraredFrame;
  ST::AccelerometerEvent m_accelerometerEvent;
  ST::GyroscopeEvent m_gyroscopeEvent;
  ST::StructureCoreCameraType m_cameraType;
  ST::CaptureSessionUSBVersion m_USBVersion;
  std::string m_serialNumber;

  ~SessionDelegate() { }

  void captureSessionEventDidOccur(ST::CaptureSession *session, ST::CaptureSessionEventId event) VP_OVERRIDE
  {
    switch (event) {
    case ST::CaptureSessionEventId::Booting:
      break;
    case ST::CaptureSessionEventId::Connected:
      printf("Starting streams...\n");
      session->startStreaming();
      // The following wait function will let the capture session load correctly.
      vpTime::wait(1000);

      // Getting details about capture session.
      // (USB Version, Serial Number of the camera connected, Camera Monochorme/Color)
      m_USBVersion = session->USBVersion();
      m_serialNumber = session->sensorInfo().serialNumber;
      m_cameraType = session->getCameraType();
      break;
    case ST::CaptureSessionEventId::Disconnected:
      break;
    case ST::CaptureSessionEventId::Error:
      throw vpException(vpException::fatalError, "Capture session error");
      break;
    default:
      printf("Capture session event unhandled\n");
    }
  }

  void captureSessionDidOutputSample(ST::CaptureSession *, const ST::CaptureSessionSample &sample) VP_OVERRIDE
  {
    // acquire sampleLock mutex.
    std::lock_guard<std::mutex> u(m_sampleLock);

    // Perform the modification needed on the shared variables.
    if (sample.visibleFrame.isValid())
      m_visibleFrame = sample.visibleFrame;

    if (sample.depthFrame.isValid())
      m_depthFrame = sample.depthFrame;

    if (sample.infraredFrame.isValid())
      m_infraredFrame = sample.infraredFrame;

    if (sample.type == ST::CaptureSessionSample::Type::AccelerometerEvent)
      m_accelerometerEvent = sample.accelerometerEvent;

    if (sample.type == ST::CaptureSessionSample::Type::GyroscopeEvent)
      m_gyroscopeEvent = sample.gyroscopeEvent;

    // If any thread is waiting on `cv_sampleLock`, the following instruction will unblock it.
    // In our case, `open()` and `acquire()` will be blocked on `cv_sampleLock`.
    cv_sampleLock.notify_one();
  }
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

class VISP_EXPORT vpOccipitalStructure
{
public:
  typedef enum
  {
    visible,  //!< Visible stream
    depth,    //!< Depth stream
    infrared, //!< Infrared stream
    imu       //!< IMU stream
  } vpOccipitalStructureStream;

  vpOccipitalStructure();
  ~vpOccipitalStructure();

  void acquire(vpImage<unsigned char> &gray, bool undistorted = false, double *ts = nullptr);
  void acquire(vpImage<vpRGBa> &rgb, bool undistorted = false, double *ts = nullptr);

  void acquire(vpImage<vpRGBa> *rgb, vpImage<vpRGBa> *depth, vpColVector *acceleration_data = nullptr,
               vpColVector *gyroscope_data = nullptr, bool undistorted = false, double *ts = nullptr);
  void acquire(vpImage<unsigned char> *gray, vpImage<vpRGBa> *depth, vpColVector *acceleration_data = nullptr,
               vpColVector *gyroscope_data = nullptr, bool undistorted = false, double *ts = nullptr);

  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud = nullptr, unsigned char *const data_infrared = nullptr,
               vpColVector *acceleration_data = nullptr, vpColVector *gyroscope_data = nullptr, bool undistorted = true,
               double *ts = nullptr);

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud,
               unsigned char *const data_infrared = nullptr, vpColVector *acceleration_data = nullptr,
               vpColVector *gyroscope_data = nullptr, bool undistorted = true, double *ts = nullptr);
  void acquire(unsigned char *const data_image, unsigned char *const data_depth,
               std::vector<vpColVector> *const data_pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
               unsigned char *const data_infrared = nullptr, vpColVector *acceleration_data = nullptr,
               vpColVector *gyroscope_data = nullptr, bool undistorted = true, double *ts = nullptr);
#endif

  void getIMUVelocity(vpColVector *imu_vel, double *ts);
  void getIMUAcceleration(vpColVector *imu_acc, double *ts);
  void getIMUData(vpColVector *imu_vel, vpColVector *imu_acc, double *ts = nullptr);

  bool open(const ST::CaptureSessionSettings &settings);
  void close();

  /*!
    Get camera type: Color or Monochrome.
   */
  ST::StructureCoreCameraType getCameraType() const { return m_delegate.m_cameraType; }

  ST::CaptureSessionUSBVersion getUSBVersion() const { return m_delegate.m_USBVersion; }
  std::string getSerialNumber() const { return m_delegate.m_serialNumber; }
  ST::CaptureSession &getCaptureSession() { return m_captureSession; }
  ST::CaptureSessionSettings &getCaptureSessionSettings() { return m_captureSessionSettings; }

  unsigned int getWidth(vpOccipitalStructureStream stream_type);
  unsigned int getHeight(vpOccipitalStructureStream stream_type);

  // Returns depth in millimeters at (x,y) if it exists, NAN otherwise.
  float getDepth(int x, int y);

  vpPoint unprojectPoint(int row, int col);

  vpHomogeneousMatrix getTransform(const vpOccipitalStructureStream from, const vpOccipitalStructureStream to);

  ST::Intrinsics getIntrinsics(const vpOccipitalStructureStream stream_type) const;

  vpCameraParameters getCameraParameters(
      const vpOccipitalStructureStream stream_type,
      vpCameraParameters::vpCameraParametersProjType type = vpCameraParameters::perspectiveProjWithoutDistortion);

  void saveDepthImageAsPointCloudMesh(std::string &filename);

protected:
  bool m_init;
  float m_invalidDepthValue;
  float m_maxZ;

  ST::CaptureSession m_captureSession;
  ST::CaptureSessionSettings m_captureSessionSettings;
  SessionDelegate m_delegate;
  vpCameraParameters m_visible_camera_parameters, m_depth_camera_parameters;

  void getPointcloud(std::vector<vpColVector> &pointcloud);
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON)
  void getPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
  void getColoredPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
#endif
};
END_VISP_NAMESPACE
#endif
#endif
