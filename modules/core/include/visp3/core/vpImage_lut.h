/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Image handling.
 */

#ifndef VP_IMAGE_LUT_H
#define VP_IMAGE_LUT_H

// Warning: this file shouldn't be included by the user. Internal usage only to reduce length of vpImage.h

#if defined(VISP_HAVE_THREADS)
namespace
{
struct vpImageLut_Param_t
{
  unsigned int m_start_index;
  unsigned int m_end_index;

  unsigned char m_lut[256];
  unsigned char *m_bitmap;

  vpImageLut_Param_t() : m_start_index(0), m_end_index(0), m_lut(), m_bitmap(nullptr) { }

  vpImageLut_Param_t(unsigned int start_index, unsigned int end_index, unsigned char *bitmap)
    : m_start_index(start_index), m_end_index(end_index), m_lut(), m_bitmap(bitmap)
  { }
};

void performLutThread(vpImageLut_Param_t *imageLut_param)
{
  unsigned int start_index = imageLut_param->m_start_index;
  unsigned int end_index = imageLut_param->m_end_index;

  unsigned char *bitmap = imageLut_param->m_bitmap;

  unsigned char *ptrStart = bitmap + start_index;
  unsigned char *ptrEnd = bitmap + end_index;
  unsigned char *ptrCurrent = ptrStart;

  if (end_index - start_index >= 8) {
    // Unroll loop version
    for (; ptrCurrent <= ptrEnd - 8;) {
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
      ++ptrCurrent;
    }
  }

  for (; ptrCurrent != ptrEnd; ++ptrCurrent) {
    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent];
  }
}

struct vpImageLutRGBa_Param_t
{
  unsigned int m_start_index;
  unsigned int m_end_index;

  VISP_NAMESPACE_ADDRESSING vpRGBa m_lut[256];
  unsigned char *m_bitmap;

  vpImageLutRGBa_Param_t() : m_start_index(0), m_end_index(0), m_lut(), m_bitmap(nullptr) { }

  vpImageLutRGBa_Param_t(unsigned int start_index, unsigned int end_index, unsigned char *bitmap)
    : m_start_index(start_index), m_end_index(end_index), m_lut(), m_bitmap(bitmap)
  { }
};

void performLutRGBaThread(vpImageLutRGBa_Param_t *imageLut_param)
{
  unsigned int start_index = imageLut_param->m_start_index;
  unsigned int end_index = imageLut_param->m_end_index;

  unsigned char *bitmap = imageLut_param->m_bitmap;

  unsigned char *ptrStart = bitmap + start_index * 4;
  unsigned char *ptrEnd = bitmap + end_index * 4;
  unsigned char *ptrCurrent = ptrStart;

  if (end_index - start_index >= 4 * 2) {
    // Unroll loop version
    for (; ptrCurrent <= ptrEnd - 4 * 2;) {
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].R;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].G;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].B;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].A;
      ptrCurrent++;

      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].R;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].G;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].B;
      ptrCurrent++;
      *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].A;
      ptrCurrent++;
    }
  }

  while (ptrCurrent != ptrEnd) {
    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].R;
    ptrCurrent++;

    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].G;
    ptrCurrent++;

    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].B;
    ptrCurrent++;

    *ptrCurrent = imageLut_param->m_lut[*ptrCurrent].A;
    ptrCurrent++;
  }
}
} // namespace
#endif

/*!
  \warning This generic method is not implemented. You should rather use the
  instantiated methods for unsigned char and vpRGBa images.

  \sa vpImage<unsigned char>::performLut(const unsigned char (&lut)[256], unsigned int nbThreads)
  \sa vpImage<vpRGBa>::performLut(const vpRGBa (&lut)[256], unsigned int nbThreads)

*/
template <class Type> void vpImage<Type>::performLut(const Type(&)[256], unsigned int)
{
  std::cerr << "Not implemented !" << std::endl;
}

/*!
  \relates vpImage

  Modify the intensities of a grayscale image using the look-up table passed
  in parameter.

  \param lut : Look-up table (unsigned char array of size=256) which maps each
  intensity to his new value.
  \param nbThreads : Number of threads to use for the computation.
*/
template <> inline void vpImage<unsigned char>::performLut(const unsigned char(&lut)[256], unsigned int nbThreads)
{
  unsigned int size = getWidth() * getHeight();
  unsigned char *ptrStart = static_cast<unsigned char *>(bitmap);
  unsigned char *ptrEnd = ptrStart + size;
  unsigned char *ptrCurrent = ptrStart;

  bool use_single_thread = ((nbThreads == 0) || (nbThreads == 1));
#if !defined(VISP_HAVE_THREADS)
  use_single_thread = true;
#endif

  if ((!use_single_thread) && (getSize() <= nbThreads)) {
    use_single_thread = true;
  }

  if (use_single_thread) {
    // Single thread

    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = lut[*ptrCurrent];
      ++ptrCurrent;
    }
  }
  else {
#if defined(VISP_HAVE_THREADS)
    // Multi-threads
    std::vector<std::thread *> threadpool;
    std::vector<vpImageLut_Param_t *> imageLutParams;

    unsigned int image_size = getSize();
    unsigned int step = image_size / nbThreads;
    unsigned int last_step = image_size - step * (nbThreads - 1);

    for (unsigned int index = 0; index < nbThreads; ++index) {
      unsigned int start_index = index * step;
      unsigned int end_index = (index + 1) * step;

      if (index == nbThreads - 1) {
        end_index = start_index + last_step;
      }

      vpImageLut_Param_t *imageLut_param = new vpImageLut_Param_t(start_index, end_index, bitmap);
      memcpy(imageLut_param->m_lut, lut, 256 * sizeof(unsigned char));

      imageLutParams.push_back(imageLut_param);

      // Start the threads
      std::thread *imageLut_thread = new std::thread(&performLutThread, imageLut_param);
      threadpool.push_back(imageLut_thread);
    }

    for (size_t cpt = 0; cpt < threadpool.size(); ++cpt) {
      // Wait until thread ends up
      threadpool[cpt]->join();
    }

    // Delete
    for (size_t cpt = 0; cpt < threadpool.size(); ++cpt) {
      delete threadpool[cpt];
    }

    for (size_t cpt = 0; cpt < imageLutParams.size(); ++cpt) {
      delete imageLutParams[cpt];
    }
#endif
  }
}

/*!
  \relates vpImage

  Modify the intensities of a color image using the look-up table passed in
  parameter.

  \param lut : Look-up table (vpRGBa array of size=256) which maps each
  intensity to his new value.
  \param nbThreads : Number of threads to use for the computation.
*/
template <> inline void vpImage<vpRGBa>::performLut(const vpRGBa(&lut)[256], unsigned int nbThreads)
{
  unsigned int size = getWidth() * getHeight();
  unsigned char *ptrStart = reinterpret_cast<unsigned char *>(bitmap);
  unsigned char *ptrEnd = ptrStart + (size * 4);
  unsigned char *ptrCurrent = ptrStart;

  bool use_single_thread = ((nbThreads == 0) || (nbThreads == 1));
#if !defined(VISP_HAVE_THREADS)
  use_single_thread = true;
#endif

  if ((!use_single_thread) && (getSize() <= nbThreads)) {
    use_single_thread = true;
  }

  if (use_single_thread) {
    // Single thread
    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = lut[*ptrCurrent].R;
      ++ptrCurrent;

      *ptrCurrent = lut[*ptrCurrent].G;
      ++ptrCurrent;

      *ptrCurrent = lut[*ptrCurrent].B;
      ++ptrCurrent;

      *ptrCurrent = lut[*ptrCurrent].A;
      ++ptrCurrent;
    }
  }
  else {
#if defined(VISP_HAVE_THREADS)
    // Multi-threads
    std::vector<std::thread *> threadpool;
    std::vector<vpImageLutRGBa_Param_t *> imageLutParams;

    unsigned int image_size = getSize();
    unsigned int step = image_size / nbThreads;
    unsigned int last_step = image_size - step * (nbThreads - 1);

    for (unsigned int index = 0; index < nbThreads; ++index) {
      unsigned int start_index = index * step;
      unsigned int end_index = (index + 1) * step;

      if (index == nbThreads - 1) {
        end_index = start_index + last_step;
      }

      vpImageLutRGBa_Param_t *imageLut_param = new vpImageLutRGBa_Param_t(start_index, end_index, (unsigned char *)bitmap);
      memcpy(static_cast<void *>(imageLut_param->m_lut), lut, 256 * sizeof(vpRGBa));

      imageLutParams.push_back(imageLut_param);

      // Start the threads
      std::thread *imageLut_thread = new std::thread(&performLutRGBaThread, imageLut_param);
      threadpool.push_back(imageLut_thread);
    }

    for (size_t cpt = 0; cpt < threadpool.size(); ++cpt) {
      // Wait until thread ends up
      threadpool[cpt]->join();
    }

    // Delete
    for (size_t cpt = 0; cpt < threadpool.size(); ++cpt) {
      delete threadpool[cpt];
    }

    for (size_t cpt = 0; cpt < imageLutParams.size(); ++cpt) {
      delete imageLutParams[cpt];
    }
#endif
  }
}

#endif
