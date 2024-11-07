/*
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
 * Image queue for storage helper.
 */

#ifndef vpImageQueue_h
#define vpImageQueue_h

#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_THREADS)

#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*!
  \class vpImageQueue

  \ingroup group_io_image

  Create a queue containing images and optional additional strings that could be useful to save additional information
  like the timestamp.

  This call is to use with vpImageStorageWorker.

*/
template <class Type> class vpImageQueue
{
public:
  struct vpCancelled_t
  { };

  /*!
   * Queue (FIFO) constructor. By default the max queue size is set to 1024*8.
   *
   * \param[in] seqname : Generic sequence name like `"folder/I%04d.png"`. If this name contains a parent folder, it
   * will be created.
   * \param[in] record_mode : 0 to record a sequence of images, 1 to record single images.
   */
  vpImageQueue(const std::string &seqname, int record_mode)
    : m_cancelled(false), m_cond(), m_queue_image(), m_queue_data(), m_maxQueueSize(1024 * 8), m_mutex(),
    m_seqname(seqname), m_recording_mode(record_mode), m_start_recording(false), m_directory_to_create(false),
    m_recording_trigger(false)
  {
    m_directory = vpIoTools::getParent(seqname);
    if (!m_directory.empty()) {
      if (!vpIoTools::checkDirectory(m_directory)) {
        m_directory_to_create = true;
      }
    }
    m_text_record_mode =
      std::string("Record mode: ") + (m_recording_mode ? std::string("single") : std::string("continuous"));
  }

  /*!
   * Emit cancel signal.
   */
  void cancel()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout << "Wait to finish saving images..." << std::endl;
    m_cancelled = true;
    m_cond.notify_all();
  }

  /*!
   * Return record mode; 0 when recording a sequence of images, 1 when recording recording single imagess.
   */
  int getRecordingMode() const { return m_recording_mode; }

  /*!
   * Return recording trigger indicating if recording is started.
   */
  bool getRecordingTrigger() const { return m_recording_trigger; }

  /*!
   * Return generic name of the sequence of images.
   */
  std::string getSeqName() const { return m_seqname; }

  /*!
   * Pop the image to save from the queue (FIFO).
   *
   * \param[out] I : Image to record.
   * \param[out] data : Data to record.
   *
   */
  void pop(vpImage<Type> &I, std::string &data)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    while (m_queue_image.empty()) {
      if (m_cancelled) {
        throw vpCancelled_t();
      }

      m_cond.wait(lock);

      if (m_cancelled) {
        throw vpCancelled_t();
      }
    }

    I = m_queue_image.front();

    m_queue_image.pop();

    if (!m_queue_data.empty()) {
      data = m_queue_data.front();
      m_queue_data.pop();
    }
  }

  /*!
   * Push data to save in the queue (FIFO).
   *
   * \param[in] I : Image to record.
   * \param[in] data : Data to record.
   */
  void push(const vpImage<Type> &I, std::string *data)
  {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_queue_image.push(I);

    if (data != nullptr) {
      m_queue_data.push(*data);
    }

    // Pop extra data in the queue
    while (m_queue_image.size() > m_maxQueueSize) {
      m_queue_image.pop();
    }

    if (data != nullptr) {
      while (m_queue_data.size() > m_maxQueueSize) {
        m_queue_data.pop();
      }
    }

    m_cond.notify_one();
  }

  /*!
   * Record helper that display information in the windows associated to the image, pop current image and additional
   * data in the queue.
   * \param[in] I : Image to record.
   * \param[in] data : Data to record. Set to nullptr when no additional data have to be considered.
   * \param[in] trigger_recording : External trigger to start data saving.
   * \param[in] disable_left_click : Disable left click usage to trigger data saving.
   * \return true when the used asked to quit using a right click in the display window.
   */
  bool record(const vpImage<Type> &I, std::string *data = nullptr, bool trigger_recording = false,
              bool disable_left_click = false)
  {
    if (I.display) {
      if (!m_seqname.empty()) {
        if (!disable_left_click) {
          if (!m_recording_mode) { // continuous
            if (m_start_recording) {
              vpDisplay::displayText(I, 20 * vpDisplay::getDownScalingFactor(I),
                                     10 * vpDisplay::getDownScalingFactor(I), "Left  click: stop recording",
                                     vpColor::red);
            }
            else {
              vpDisplay::displayText(I, 20 * vpDisplay::getDownScalingFactor(I),
                                     10 * vpDisplay::getDownScalingFactor(I), "Left  click: start recording",
                                     vpColor::red);
            }
          }
          else {
            vpDisplay::displayText(I, 20 * vpDisplay::getDownScalingFactor(I), 10 * vpDisplay::getDownScalingFactor(I),
                                   "Left  click: record image", vpColor::red);
          }
        }
        vpDisplay::displayText(I, 40 * vpDisplay::getDownScalingFactor(I), 10 * vpDisplay::getDownScalingFactor(I),
                               "Right click: quit", vpColor::red);
      }
      else {
        vpDisplay::displayText(I, 20 * vpDisplay::getDownScalingFactor(I), 10 * vpDisplay::getDownScalingFactor(I),
                               "Click to quit", vpColor::red);
      }

      if (!m_seqname.empty()) {
        vpDisplay::displayText(I, 60 * vpDisplay::getDownScalingFactor(I), 10 * vpDisplay::getDownScalingFactor(I),
                               m_text_record_mode, vpColor::red);
      }
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (!m_seqname.empty()) {                                        // Recording requested
          if (button == vpMouseButton::button1 && !disable_left_click) { // enable/disable recording
            m_start_recording = !m_start_recording;
          }
          else if (button == vpMouseButton::button3) { // quit
            return true;
          }
        }
        else { // any button to quit
          return true;
        }
      }
    }
    else if (!m_seqname.empty()) {
      m_start_recording = true;
    }

    if (trigger_recording) {
      m_start_recording = true;
    }

    m_recording_trigger = m_start_recording;

    if (m_start_recording) {

      if (m_directory_to_create) {
        std::cout << "Create directory \"" << m_directory << "\"" << std::endl;
        vpIoTools::makeDirectory(m_directory);
        m_directory_to_create = false;
      }

      push(I, data);

      if (m_recording_mode == 1) { // single shot mode
        m_start_recording = false;
      }
    }
    return false;
  }

  /*!
   * Set queue size.
   * \param[in] max_queue_size : Queue size.
   */
  void setMaxQueueSize(const size_t max_queue_size) { m_maxQueueSize = max_queue_size; }

private:
  bool m_cancelled;
  std::condition_variable m_cond;
  std::queue<vpImage<Type> > m_queue_image;
  std::queue<std::string> m_queue_data;
  size_t m_maxQueueSize;
  std::mutex m_mutex;
  std::string m_seqname;
  std::string m_directory;
  int m_recording_mode;
  bool m_start_recording;
  std::string m_text_record_mode;
  bool m_directory_to_create;
  bool m_recording_trigger;
};

#endif // DOXYGEN_SHOULD_SKIP_THIS
END_VISP_NAMESPACE
#endif
#endif
