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
 * Acquire images using 1394 device with cfox (MAC OSX) and display it
 * using GTK or GTK.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example grabV4l2MultiCpp11Thread.cpp

  \brief Example of using V4l2 backend to capture multiple camera streams
         with C++11 threads.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CPP11_COMPATIBILITY) && defined(VISP_HAVE_V4L2) &&                                               \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK))

#include <condition_variable>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoWriter.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#define GETOPTARGS "d:oh"

namespace
{

void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
SYNOPSIS:\n\
  %s [-d <device count>] [-o] [-h]\n\
\n\
DESCRIPTION:\n\
  Capture multiple camera streams and save the stream without slowing down the acquisition.\n\
    \n\
OPTIONS:                                               \n\
  -d <device count>                                 \n\
     Open the specified number of camera streams.\n\
    \n\
  -o                                 \n\
     Save each stream in a dedicated folder.\n\
    \n\
  -h \n\
     Print the help.\n\n", name);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

bool getOptions(int argc, char **argv, unsigned int &deviceCount, bool &saveVideo)
{
  const char *optarg;
  const char **argv1 = (const char **)argv;
  int c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'd':
      deviceCount = (unsigned int)atoi(optarg);
      break;
    case 'o':
      saveVideo = true;
      break;
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

// Code adapted from the original author Dan Mašek to be compatible with ViSP
// image
class FrameQueue
{

public:
  struct cancelled {
  };

  FrameQueue()
    : m_cancelled(false), m_cond(), m_queueColor(), m_maxQueueSize(std::numeric_limits<size_t>::max()), m_mutex()
  {
  }

  void cancel()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cancelled = true;
    m_cond.notify_all();
  }

  // Push the image to save in the queue (FIFO)
  void push(const vpImage<vpRGBa> &image)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_queueColor.push(image);

    // Pop extra images in the queue
    while (m_queueColor.size() > m_maxQueueSize) {
      m_queueColor.pop();
    }

    m_cond.notify_one();
  }

  // Pop the image to save from the queue (FIFO)
  vpImage<vpRGBa> pop()
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    while (m_queueColor.empty()) {
      if (m_cancelled) {
        throw cancelled();
      }

      m_cond.wait(lock);

      if (m_cancelled) {
        throw cancelled();
      }
    }

    vpImage<vpRGBa> image(m_queueColor.front());
    m_queueColor.pop();

    return image;
  }

  void setMaxQueueSize(const size_t max_queue_size) { m_maxQueueSize = max_queue_size; }

private:
  bool m_cancelled;
  std::condition_variable m_cond;
  std::queue<vpImage<vpRGBa> > m_queueColor;
  size_t m_maxQueueSize;
  std::mutex m_mutex;
};

class StorageWorker
{

public:
  StorageWorker(FrameQueue &queue, const std::string &filename, const unsigned int width, const unsigned int height)
    : m_queue(queue), m_filename(filename), m_width(width), m_height(height)
  {
  }

  // Thread main loop
  void run()
  {
    vpImage<vpRGBa> O_color(m_height, m_width);

    vpVideoWriter writer;
    if (!m_filename.empty()) {
      writer.setFileName(m_filename);
      writer.open(O_color);
    }

    try {
      for (;;) {
        vpImage<vpRGBa> image(m_queue.pop());

        if (!m_filename.empty()) {
          writer.saveFrame(image);
        }
      }
    } catch (FrameQueue::cancelled &) {
    }
  }

private:
  FrameQueue &m_queue;
  std::string m_filename;
  unsigned int m_width;
  unsigned int m_height;
};

class ShareImage
{

private:
  bool m_cancelled;
  std::condition_variable m_cond;
  std::mutex m_mutex;
  unsigned char *m_pImgData;
  unsigned int m_totalSize;

public:
  struct cancelled {
  };

  ShareImage() : m_cancelled(false), m_cond(), m_mutex(), m_pImgData(NULL), m_totalSize(0) {}

  virtual ~ShareImage()
  {
    if (m_pImgData != NULL) {
      delete[] m_pImgData;
    }
  }

  void cancel()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cancelled = true;
    m_cond.notify_all();
  }

  // Get the image to display
  void getImage(unsigned char *const imageData, const unsigned int totalSize)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    if (m_cancelled) {
      throw cancelled();
    }

    m_cond.wait(lock);

    if (m_cancelled) {
      throw cancelled();
    }

    // Copy to imageData
    if (totalSize <= m_totalSize) {
      memcpy(imageData, m_pImgData, totalSize * sizeof(unsigned char));
    } else {
      std::cerr << "totalSize <= m_totalSize !" << std::endl;
    }
  }

  bool isCancelled()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cancelled;
  }

  // Set the image to display
  void setImage(const unsigned char *const imageData, const unsigned int totalSize)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_pImgData == NULL || m_totalSize != totalSize) {
      m_totalSize = totalSize;

      if (m_pImgData != NULL) {
        delete[] m_pImgData;
      }

      m_pImgData = new unsigned char[m_totalSize];
    }

    // Copy from imageData
    memcpy(m_pImgData, imageData, m_totalSize * sizeof(unsigned char));

    m_cond.notify_one();
  }
};

void capture(vpV4l2Grabber *const pGrabber, ShareImage &share_image)
{
  vpImage<vpRGBa> local_img;

  // Open the camera stream
  pGrabber->open(local_img);

  while (true) {
    if (share_image.isCancelled()) {
      break;
    }

    pGrabber->acquire(local_img);

    // Update share_image
    share_image.setImage((unsigned char *)local_img.bitmap, local_img.getSize() * 4);
  }
}

void display(const unsigned int width, const unsigned int height, const int win_x, const int win_y,
             const unsigned int deviceId, ShareImage &share_image, FrameQueue &queue, const bool save)
{
  vpImage<vpRGBa> local_img(height, width);

#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#endif

  // Init Display
  std::stringstream ss;
  ss << "Camera stream " << deviceId;
  display.init(local_img, win_x, win_y, ss.str());

  try {
    vpMouseButton::vpMouseButtonType button;

    vpImage<unsigned char> I_red(height, width), I_green(height, width), I_blue(height, width), I_alpha(height, width);
    ;
    vpImage<unsigned char> I_red_gaussian(height, width), I_green_gaussian(height, width),
        I_blue_gaussian(height, width);
    vpImage<double> I_red_gaussian_double, I_green_gaussian_double, I_blue_gaussian_double;

    bool exit = false, gaussian_blur = false;
    while (!exit) {
      double t = vpTime::measureTimeMs();

      // Get image
      share_image.getImage((unsigned char *)local_img.bitmap, local_img.getSize() * 4);

      // Apply gaussian blur to simulate a computation on the image
      if (gaussian_blur) {
        // Split channels
        vpImageConvert::split(local_img, &I_red, &I_green, &I_blue, &I_alpha);
        vpImageConvert::convert(I_red, I_red_gaussian_double);
        vpImageConvert::convert(I_green, I_green_gaussian_double);
        vpImageConvert::convert(I_blue, I_blue_gaussian_double);

        vpImageFilter::gaussianBlur(I_red_gaussian_double, I_red_gaussian_double, 21);
        vpImageFilter::gaussianBlur(I_green_gaussian_double, I_green_gaussian_double, 21);
        vpImageFilter::gaussianBlur(I_blue_gaussian_double, I_blue_gaussian_double, 21);

        vpImageConvert::convert(I_red_gaussian_double, I_red_gaussian);
        vpImageConvert::convert(I_green_gaussian_double, I_green_gaussian);
        vpImageConvert::convert(I_blue_gaussian_double, I_blue_gaussian);

        vpImageConvert::merge(&I_red_gaussian, &I_green_gaussian, &I_blue_gaussian, NULL, local_img);
      }

      t = vpTime::measureTimeMs() - t;
      std::stringstream ss;
      ss << "Time: " << t << " ms";

      vpDisplay::display(local_img);

      vpDisplay::displayText(local_img, 20, 20, ss.str(), vpColor::red);
      vpDisplay::displayText(local_img, 40, 20, "Left click to quit, right click for Gaussian blur.", vpColor::red);

      vpDisplay::flush(local_img);

      if (save) {
        queue.push(local_img);
      }

      if (vpDisplay::getClick(local_img, button, false)) {
        switch (button) {
        case vpMouseButton::button3:
          gaussian_blur = !gaussian_blur;
          break;

        default:
          exit = true;
          break;
        }
      }
    }
  } catch (ShareImage::cancelled &) {
    std::cout << "Cancelled!" << std::endl;
  }

  share_image.cancel();
}

} // Namespace

int main(int argc, char *argv[])
{
  unsigned int deviceCount = 1;
  unsigned int cameraScale = 1; // 640x480
  bool saveVideo = false;

  // Read the command line options
  if (!getOptions(argc, argv, deviceCount, saveVideo)) {
    return (-1);
  }

  std::vector<vpV4l2Grabber *> grabbers;

  const unsigned int offsetX = 100, offsetY = 100;
  for (unsigned int devicedId = 0; devicedId < deviceCount; devicedId++) {
    try {
      vpV4l2Grabber *pGrabber = new vpV4l2Grabber;
      std::stringstream ss;
      ss << "/dev/video" << devicedId;
      pGrabber->setDevice(ss.str());
      pGrabber->setScale(cameraScale);

      grabbers.push_back(pGrabber);
    } catch (const vpException &e) {
      std::cerr << "Exception: " << e.what() << std::endl;
    }
  }

  std::cout << "Grabbers: " << grabbers.size() << std::endl;

  std::vector<ShareImage> share_images(grabbers.size());
  std::vector<std::thread> capture_threads;
  std::vector<std::thread> display_threads;

  // Synchronized queues for each camera stream
  std::vector<FrameQueue> save_queues(grabbers.size());
  std::vector<StorageWorker> storages;
  std::vector<std::thread> storage_threads;

  std::string parent_directory = vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  for (size_t deviceId = 0; deviceId < grabbers.size(); deviceId++) {
    // Start the capture thread for the current camera stream
    capture_threads.emplace_back(capture, grabbers[deviceId], std::ref(share_images[deviceId]));
    int win_x = deviceId * offsetX, win_y = deviceId * offsetY;

    // Start the display thread for the current camera stream
    display_threads.emplace_back(display, grabbers[deviceId]->getWidth(), grabbers[deviceId]->getHeight(), win_x, win_y,
                                 deviceId, std::ref(share_images[deviceId]), std::ref(save_queues[deviceId]),
                                 saveVideo);

    if (saveVideo) {
      std::stringstream ss;
      ss << parent_directory << "/Camera_Stream" << deviceId;
      std::cout << "Create directory: " << ss.str() << std::endl;
      vpIoTools::makeDirectory(ss.str());
      ss << "/%06d.png";
      std::string filename = ss.str();

      storages.emplace_back(std::ref(save_queues[deviceId]), std::cref(filename), grabbers[deviceId]->getWidth(),
                            grabbers[deviceId]->getHeight());
    }
  }

  if (saveVideo) {
    for (auto &s : storages) {
      // Start the storage thread for the current camera stream
      storage_threads.emplace_back(&StorageWorker::run, &s);
    }
  }

  // Join all the worker threads, waiting for them to finish
  for (auto &ct : capture_threads) {
    ct.join();
  }

  for (auto &dt : display_threads) {
    dt.join();
  }

  // Clean first the grabbers to avoid camera problems when cancelling the
  // storage threads in the terminal
  for (auto &g : grabbers) {
    delete g;
  }

  if (saveVideo) {
    std::cout << "\nWaiting for finishing thread to write images..." << std::endl;
  }

  // We're done reading, cancel all the queues
  for (auto &qu : save_queues) {
    qu.cancel();
  }

  // Join all the worker threads, waiting for them to finish
  for (auto &st : storage_threads) {
    st.join();
  }

  return EXIT_SUCCESS;
}
#else
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK))
int main()
{
  std::cout << "You do not have X11, or GTK functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GTK, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#elif !defined(VISP_HAVE_V4L2)
int main()
{
  std::cout << "You do not have Video 4 Linux 2 functionality enabled" << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install libv4l2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You do not build ViSP with C++11 compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CPP11=ON, and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
#endif

