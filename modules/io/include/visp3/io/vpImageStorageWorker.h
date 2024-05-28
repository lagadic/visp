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
 * Image storage helper.
 */

#ifndef vpImageStorageWorker_h
#define vpImageStorageWorker_h

#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_THREADS)

#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpImageQueue.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpImageStorageWorker

  \ingroup group_io_image

  Save data contained in an vpImageQueue.

*/
template <class Type> class vpImageStorageWorker
{
public:
  /*!
   * Constructor.
   * \param[in] queue : A reference to a queue.
   */
  vpImageStorageWorker(vpImageQueue<Type> &queue)
    : m_queue(queue), m_dataname(""), m_cpt(1), m_ofs_data(), m_data_file_created(false)
  {
    m_seqname = queue.getSeqName();
    m_record_mode = queue.getRecordingMode();
  }

  /*!
   * Thread main loop that save the images and additional data.
   */
  void run()
  {
    try {
      vpImage<Type> I;
      std::string data;
      char filename[FILENAME_MAX];

      for (;;) {
        m_queue.pop(I, data);

        // Save image
        snprintf(filename, FILENAME_MAX, m_seqname.c_str(), m_cpt);

        if (m_record_mode > 0) { // Single image
          std::cout << "Save image: " << filename << std::endl;
        }
        else if (m_cpt == 1) {
          std::cout << "Started sequence saving: " << m_seqname << std::endl;
        }
        vpImageIo::write(I, filename);

        if (!data.empty()) {
          if (!m_data_file_created) {
            std::string parent = vpIoTools::getParent(m_seqname);
            if (!parent.empty()) {
              m_dataname = vpIoTools::getParent(m_seqname) + "/";
            }
            m_dataname += vpIoTools::getNameWE(m_seqname);
            m_dataname += ".txt";

            std::cout << "Create data file: " << m_dataname << std::endl;
            m_ofs_data.open(m_dataname);

            m_data_file_created = true;
          }
          m_ofs_data << vpIoTools::getName(filename) << " " << data << std::endl;
        }

        m_cpt++;
      }
    }
    catch (const vpImageQueue<vpRGBa>::vpCancelled_t &) {
      std::cout << "Receive cancel during color image saving." << std::endl;
      if (m_data_file_created) {
        std::cout << "Close data file: " << m_dataname << std::endl;
        m_ofs_data.close();
      }
    }
    catch (const vpImageQueue<unsigned char>::vpCancelled_t &) {
      std::cout << "Receive cancel during gray image saving." << std::endl;
      if (m_data_file_created) {
        std::cout << "Close data file: " << m_dataname << std::endl;
        m_ofs_data.close();
      }
    }
  }

private:
  vpImageQueue<Type> &m_queue;
  std::string m_seqname;
  std::string m_dataname;
  int m_record_mode;
  unsigned int m_cpt;
  std::ofstream m_ofs_data;
  bool m_data_file_created;
};
END_VISP_NAMESPACE
#endif
#endif
