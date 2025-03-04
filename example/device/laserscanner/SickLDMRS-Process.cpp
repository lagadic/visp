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
 * Sick LD-MRS laser driver.
 *
*****************************************************************************/

/*!
  \example SickLDMRS-Process.cpp

  \brief Example that shows how to acquire Sick LD-MRS laser measurements.

  This example shows by multithreading how to:
  - acquire Sick LD-MRS laser measurements
  - how to process the data in order to display the laser scans and how
    to save the scan data in a file
  - and how to acquire and display images from a firewire camera if connected.

  \warning For the moment, this example is only working on UNIX
  platforms since the Sick LD-MRS driver was not ported to Windows.

  Concerning the laser, layer 1 is displayed in red, layer 2 in green,
  l ayer 3 in blue and layer 4 in yellow.

  Thanks to the -layer command line option, this example allows to
  select the layers to proceed.

*/
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpSickLDMRS.h>
#ifdef VISP_HAVE_MODULE_GUI
#include <visp3/gui/vpDisplayFactory.h>
#endif
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

#if (!defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))) && \
    defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_THREADS)

#include <thread>
#include <mutex>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static int save = 0;
static int layerToDisplay = 0xF; // 0xF = 1111 => all the layers are selected
static vpLaserScan shm_laserscan[4];
double time_offset = 0;
std::mutex shm_mutex;
std::string output_path;

void laser_display_and_save_loop()
{
  vpImage<unsigned char> map(700, 300);
  map = 0;
  unsigned int width = map.getWidth();
  unsigned int height = map.getHeight();
  vpImagePoint O; // Beam origin
  O.set_i(height);
  O.set_j(width / 2.);
  vpScanPoint p;
  vpColor color[4]; // one color per layer
  char filename[FILENAME_MAX];
  std::ofstream fdscan;
  vpLaserScan laserscan[4];

  for (int layer = 0; layer < 4; layer++) {
    switch (layer) {
    case 0:
      color[layer] = vpColor::red;
      break;
    case 1:
      color[layer] = vpColor::green;
      break;
    case 2:
      color[layer] = vpColor::blue;
      break;
    case 3:
      color[layer] = vpColor::yellow;
      break;
    }
  }

#ifdef VISP_HAVE_MODULE_GUI
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay();
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay();
#endif
  display->init(map, 10, 10, "Laser scan");
#endif

  unsigned int iter = 0;
  for (;;) {

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK))
    vpDisplay::display(map);
#endif

    shm_mutex.lock();
    for (int layer = 0; layer < 4; layer++) {
      laserscan[layer] = shm_laserscan[layer];
    }
    shm_mutex.unlock();

    // Parse the four layers
    for (int layer = 0; layer < 4; layer++) {
      if (!((0x1 << layer) & layerToDisplay)) {
        std::cout << "Layer " << layer + 1 << " is not displayed" << std::endl;
        continue;
      }

      std::vector<vpScanPoint> pointsLayer = laserscan[layer].getScanPoints();

      if (save) {
        // Set the scan data filename to store the measures
        snprintf(filename, FILENAME_MAX, "%s/scan%04u-layer%d.txt", output_path.c_str(), iter, layer + 1);
        fdscan.open(filename);

        // Write the file header
        fdscan << "# Scan layer [1 to 4] : " << layer + 1 << std::endl
          << "# Start timestamp (s) : " << laserscan[layer].getStartTimestamp() - time_offset << std::endl
          << "# End timestamp (s)   : " << laserscan[layer].getEndTimestamp() - time_offset << std::endl
          << "# Data : \"radial distance (m)\" \"horizontal angle "
          "(rad)\" \"vertical angle (rad)\" \"X (m)\" \"Y (m)\" \"Z "
          "(m)\""
          << std::endl;
      }

      vpImagePoint E;        // Beam echo
      double resolution = 5; // 100 pixels = 1 meter - increase this value to
                             // see better near info
      for (unsigned int i = 0; i < pointsLayer.size(); i++) {
        p = pointsLayer[i];
        E.set_i(height - resolution * p.getRadialDist() * cos(p.getHAngle()));
        E.set_j(width / 2. - resolution * p.getRadialDist() * sin(p.getHAngle()));
// std::cout << "E: " << E << std::endl;
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK))
        vpDisplay::displayLine(map, O, E, color[layer]);
#endif
        if (save) {
          // Save the measures in the file
          fdscan << p << std::endl;
        }
      }
      if (save) {
        fdscan.close();
      }
    }
#if defined(VISP_HAVE_DISPLAY)
    vpDisplay::flush(map);
#endif
    iter++;
    // std::cout << "display time: " << vpTime::measureTimeMs() - t1 <<
    // std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_MODULE_GUI)
  if (display != nullptr) {
    delete display;
  }
#endif
}

void laser_acq_loop()
{
  std::string ip = "131.254.12.119";

  vpSickLDMRS laser;
  laser.setIpAddress(ip);
  laser.setup();
  vpLaserScan laserscan[4];

  for (;;) {
    double t1 = vpTime::measureTimeMs();
    if (laser.measure(laserscan) == false)
      continue;

    shm_mutex.lock();
    for (int layer = 0; layer < 4; layer++) {
      shm_laserscan[layer] = laserscan[layer];
    }
    shm_mutex.unlock();

    std::cout << "laser acq time: " << vpTime::measureTimeMs() - t1 << std::endl;
  }
}

void camera_acq_and_display_loop()
{
#ifdef VISP_HAVE_DC1394
  try {
    // Initialize the firewire framegrabber
    vp1394TwoGrabber g; // Create a grabber based on libdc1394-2.x third party lib

    // If no camera found return
    if (g.getNumCameras() == 0) {
      return;
    }

    //     g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    //     g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);

    vpImage<unsigned char> I; // Create a gray level image container
    vpImage<unsigned char> Q; // Create a quarter size gray level image container
    g.acquire(I);             // Acquire an image
    I.quarterSizeImage(Q);

#ifdef VISP_HAVE_MODULE_GUI
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay();
#else
    vpDisplay *display = vpDisplayFactory::allocateDisplay();
#endif
    display->init(Q, 320, 10, "Camera");
#endif

    // Create a file with image time stamps
    std::ofstream fdimage_ts;
    if (save) {
      std::string filename = output_path + "/image_timestamp.txt";
      fdimage_ts.open(filename.c_str());
      fdimage_ts << "# [image name] [time stamp in second]" << std::endl;
    }
    unsigned iter = 0;
    char filename[FILENAME_MAX];
    uint64_t timestamp;
    uint32_t id;
    for (;;) {
      dc1394video_frame_t *frame = g.dequeue(I, timestamp, id); // Acquire an image
      I.quarterSizeImage(Q);
      double image_timestamp = timestamp / 1000000. - time_offset;
      std::cout << "camera timestamp: " << image_timestamp << " s " << std::endl;
      if (save) {
        // Set the image filename
        snprintf(filename, FILENAME_MAX, "%s/image%04u.png", output_path.c_str(), iter);
        vpImageIo::write(Q, filename);
        fdimage_ts << filename << " " << image_timestamp << std::endl;
      }
#if defined(VISP_HAVE_DISPLAY)
      vpDisplay::display(Q);
      vpDisplay::flush(Q);
#endif
      g.enqueue(frame);

      iter++;
    }
    if (save) {
      fdimage_ts.close();
    }
  }
  catch (...) {
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_MODULE_GUI)
  if (display != nullptr) {
    delete display;
  }
#endif
#endif
}

int main(int argc, const char **argv)
{
  try {
    output_path = "data";
    // Test if the output path directory exist. If no try to create it
    if (vpIoTools::checkDirectory(output_path) == false) {
      try {
        // Create a directory with name "username"
        vpIoTools::makeDirectory(output_path);
      }
      catch (...) {
        std::cout << "Cannot create " << output_path << " directory" << std::endl;
        return EXIT_FAILURE;
      }
    }

    // Parse the command line to set the variables
    vpParseArgv::vpArgvInfo argTable[] = {
        {"-layer", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&layerToDisplay,
         "The layer to display:\n"
         "\t\t. 0x1 for layer 1.\n"
         "\t\t. 0x2 for layer 2.\n"
         "\t\t. 0x4 for layer 3.\n"
         "\t\t. 0x8 for layer 4.\n"
         "\t\tTo display all the layers you should set 0xF value."},
        {"-save", vpParseArgv::ARGV_INT, (char *)nullptr, (char *)&save, "Turn to 1 in order to save data."},
        {"-h", vpParseArgv::ARGV_HELP, (char *)nullptr, (char *)nullptr,
         "Display one or more measured layers form a Sick LD-MRS laser "
         "scanner."},
        {(char *)nullptr, vpParseArgv::ARGV_END, (char *)nullptr, (char *)nullptr, (char *)nullptr} };

    // Read the command line options
    if (vpParseArgv::parse(&argc, argv, argTable,
                           vpParseArgv::ARGV_NO_LEFTOVERS | vpParseArgv::ARGV_NO_ABBREV |
                           vpParseArgv::ARGV_NO_DEFAULTS)) {
      return (EXIT_FAILURE);
    }
    time_offset = vpTime::measureTimeSecond();

    std::thread thread_camera_acq(&camera_acq_and_display_loop);
    std::thread thread_laser_acq(&laser_acq_loop);
    std::thread thread_laser_display(&laser_display_and_save_loop);
    thread_camera_acq.join();
    thread_laser_acq.join();
    thread_laser_display.join();

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#elif !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK))
int main()
{
  std::cout << "You do not have X11, or GTK functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GTK, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#else // #ifdef UNIX and display

int main()
{
  std::cout << "This example is only working on unix-like platforms \n"
    << "since the Sick LD-MRS driver was not ported to Windows." << std::endl;
  return EXIT_SUCCESS;
}

#endif // #ifdef UNIX
