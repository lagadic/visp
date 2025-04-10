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
 * Gray level histogram manipulation.
 */

/*!
  \file vpHistogram.h
  \brief Declaration of the vpHistogram class.
  Class vpHistogram defines gray level image histograms

*/

#ifndef VP_HISTOGRAM_H
#define VP_HISTOGRAM_H

#include <sstream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpHistogramPeak.h>
#include <visp3/core/vpHistogramValey.h>
#include <visp3/core/vpImage.h>

#include <list>
#if defined(VISP_HAVE_THREADS)
#include <thread>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpHistogram
  \ingroup group_core_histogram
  \brief Class to compute a gray level image histogram.

  Here are two examples showing how to use this class to determine the
  threshold which can be used to segment two objects.

  The code below:

  \code
  vpImage<unsigned char> I;
  ...
  unsigned char dist = 60;
  vpHistogramValey valey;
  vpHistogram h(I);
  h.smooth();                   // Filter the histogram values
  vpHistogramPeak peakl, peakr; // Two highest peaks in the histogram
                                // - peakl: Peak on the left
                                // - peakr: Peak on the right

  h.getPeaks(dist, peakl, peakr, valey);
  unsigned char threshold;      // Position of the valey between the two peaks
  threshold = valey.getLevel();
  \endcode

  has the same behaviour than this one:

  \code
  vpImage<unsigned char> I;
  ...
  unsigned char dist = 60;
  vpHistogram h(I);
  h.smooth();                   // Filter the histogram values
  vpHistogramPeak peak1, peak2; // Two highest peaks in the histogram
                                // - peak1: Highest peak
                                // - peakr: Second highest peak

  // Get the two highest peaks
  h.getPeaks(dist, peak1, peak2);

  // Get the valey between the two highest peaks
  vpHistogramValey valey;
  h.getValey(peak1, peak2, valey);

  unsigned char threshold; // Position of the valey between the two peaks
  threshold = valey.getLevel();
  \endcode

*/
class VISP_EXPORT vpHistogram
{
public:
  vpHistogram(const unsigned int &size = constr_val_256);
  vpHistogram(const vpHistogram &h);
  VP_EXPLICIT vpHistogram(const vpImage<unsigned char> &I);
  VP_EXPLICIT vpHistogram(const vpImage<unsigned char> &I, const vpImage<bool> *p_mask);
  virtual ~vpHistogram();

  vpHistogram &operator=(const vpHistogram &h);

  /*!

    Return the number of pixels having the gray \e level.

    \param level : Gray level in the histogram.

    \return Number of pixels having the gray level.

    \code
    vpImage<unsigned char> I; // A gray level image

    vpHistogram h;
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    for (int i=0; i < h.getSize(); ++i)
      printf("%d: %d\n", i, h[i]);
    \endcode

  */
  inline unsigned operator[](const unsigned char level) const
  {
    if (level < m_size) {
      return m_histogram[level];
    }

    std::stringstream ss;
    ss << "Level is > to size (" << m_size << ") !";
    throw vpException(vpException::dimensionError, ss.str().c_str());
  }
  /*!

    Return the number of pixels having the gray \e level.

    \param level : Gray level in the histogram.

    \return Number of pixels having the gray level.

    \code
    vpImage<unsigned char> I; // A gray level image

    vpHistogram h;
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    for (int i=0; i < h.getSize(); ++i)
      printf("%d: %d\n", i, h(i));
    \endcode

  */
  inline unsigned operator()(const unsigned char level) const
  {
    if (level < m_size) {
      return m_histogram[level];
    }

    std::stringstream ss;
    ss << "Level is > to size (" << m_size << ") !";
    throw vpException(vpException::dimensionError, ss.str().c_str());
  }
  /*!

    Return the number of pixels having the gray \e level.

    \param level : Gray level in the histogram.

    \return Number of pixels having the gray level.

    \code
    vpImage<unsigned char> I; // A gray level image

    vpHistogram h;
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    for (int i=0; i < h.getSize(); ++i)
      printf("%d: %d\n", i, h.get(i));
    \endcode

  */
  inline unsigned get(const unsigned char level) const
  {
    if (level < m_size) {
      return m_histogram[level];
    }

    std::stringstream ss;
    ss << "Level is > to size (" << m_size << ") !";
    throw vpException(vpException::dimensionError, ss.str().c_str());
  }

  /*!

    Set the number of pixels having the gray \e level.

    \param level : Gray level in the histogram. Level is in [0:255]

    \param value : Number of pixels having the gray level.

    \code
    vpHistogram h;

    // Set histogram values
    for (int i=0; i < h.getSize(); ++i)
      h.set(i, i*2); // for each level i, set a value of 2*i
    \endcode

  */
  inline void set(const unsigned char level, unsigned int value)
  {
    if (level < m_size) {
      m_histogram[level] = value;
    }
    else {
      std::stringstream ss;
      ss << "Level is > to size (" << m_size << ") !";
      throw vpException(vpException::dimensionError, ss.str().c_str());
    }
  }

  /**
   * \brief Set a mask to ignore pixels for which the mask is false.
   *
   * \warning The mask must be reset manually by the user (either for another mask
   * or set to \b nullptr ) before computing the histogram of another image.
   *
   * @param p_mask If different of \b nullptr , a mask of booleans where \b true
   * indicates that a pixel must be considered and \b false that the pixel should
   * be ignored.
   */
  inline void setMask(const vpImage<bool> *p_mask)
  {
    mp_mask = p_mask;
  }

  void calculate(const vpImage<unsigned char> &I, unsigned int nbins = 256, unsigned int nbThreads = 1);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  template <typename ArithmeticType>
  typename std::enable_if<std::is_floating_point<ArithmeticType>::value, void>::type calculate(const vpImage<ArithmeticType> &I, const ArithmeticType &minVal, const ArithmeticType &maxVal, ArithmeticType &widthBin, unsigned int nbins = 256, unsigned int nbThreads = 1)
  {
    widthBin = (maxVal - minVal)/static_cast<ArithmeticType>(nbins);
    if (m_size < nbins) {
      init(nbins);
    }

    memset(m_histogram, 0, m_size * sizeof(unsigned int));

    bool use_single_thread;
#if !defined(VISP_HAVE_THREADS)
    use_single_thread = true;
#else
    use_single_thread = (nbThreads == 0 || nbThreads == 1);
#endif

    if ((!use_single_thread) && (I.getSize() <= nbThreads)) {
      use_single_thread = true;
    }

    if (use_single_thread) {
      // Single thread
      const bool alwaysTrue = true;
      const bool *ptrMaskCurrent = &alwaysTrue;
      if (mp_mask) {
        ptrMaskCurrent = static_cast<const bool *>(mp_mask->bitmap);
      }

      unsigned int size_ = I.getWidth() * I.getHeight();
      unsigned int idCurrent = 0;

      m_total = 0;
      while (idCurrent < size_) {
        if (*ptrMaskCurrent) {
          unsigned int id = std::floor((I.bitmap[idCurrent] - minVal)/widthBin);
          ++m_histogram[id];
          ++m_total;
        }
        ++idCurrent;
        if (mp_mask) {
          ++ptrMaskCurrent;
        }
      }
    }
    else {
#if defined(VISP_HAVE_THREADS)
      // Multi-threads
      std::vector<std::thread *> threadpool;
      std::vector<vpHistogramFloatingPoints_Param_t<ArithmeticType> *> histogramParams;

      unsigned int image_size = I.getSize();
      unsigned int step = image_size / nbThreads;
      unsigned int last_step = image_size - step * (nbThreads - 1);

      for (unsigned int index = 0; index < nbThreads; ++index) {
        unsigned int start_index = index * step;
        unsigned int end_index = (index + 1) * step;

        if (index == nbThreads - 1) {
          end_index = start_index + last_step;
        }

        vpHistogramFloatingPoints_Param_t<ArithmeticType> *histogram_param = new vpHistogramFloatingPoints_Param_t<ArithmeticType>(start_index, end_index, minVal, widthBin, &I, mp_mask);
        histogram_param->m_histogram = new unsigned int[m_size];
        histogram_param->m_mask = mp_mask;
        memset(histogram_param->m_histogram, 0, m_size * sizeof(unsigned int));

        histogramParams.push_back(histogram_param);

        // Start the threads
        std::thread *histogram_thread = new std::thread(&computeHistogramFloatingPointThread<ArithmeticType>, histogram_param);
        threadpool.push_back(histogram_thread);
      }

      for (size_t cpt = 0; cpt < threadpool.size(); ++cpt) {
        // Wait until thread ends up
        threadpool[cpt]->join();
      }

      m_total = 0;
      for (unsigned int cpt1 = 0; cpt1 < m_size; ++cpt1) {
        unsigned int sum = 0;

        for (size_t cpt2 = 0; cpt2 < histogramParams.size(); ++cpt2) {
          sum += histogramParams[cpt2]->m_histogram[cpt1];
        }

        m_histogram[cpt1] = sum;
        m_total += sum;
      }

      // Delete
      for (size_t cpt = 0; cpt < threadpool.size(); ++cpt) {
        delete threadpool[cpt];
      }

      for (size_t cpt = 0; cpt < histogramParams.size(); ++cpt) {
        delete histogramParams[cpt];
      }
#endif
    }
  }
#endif

  void equalize(const vpImage<unsigned char> &I, vpImage<unsigned char> &Iout);

  void display(const vpImage<unsigned char> &I, const vpColor &color = vpColor::white, unsigned int thickness = 2,
               unsigned int maxValue_ = 0);

  void smooth(unsigned int fsize = 3);
  unsigned getPeaks(std::list<vpHistogramPeak> &peaks);
  unsigned getPeaks(unsigned char dist, vpHistogramPeak &peak1, vpHistogramPeak &peak2);
  bool getPeaks(unsigned char dist, vpHistogramPeak &peakl, vpHistogramPeak &peakr, vpHistogramValey &valey);
  unsigned getValey(std::list<vpHistogramValey> &valey);
  bool getValey(const vpHistogramPeak &peak1, const vpHistogramPeak &peak2, vpHistogramValey &valey);
  unsigned getValey(unsigned char dist, const vpHistogramPeak &peak, vpHistogramValey &valeyl,
                    vpHistogramValey &valeyr);
  unsigned sort(std::list<vpHistogramPeak> &peaks);

  bool write(const std::string &filename);
  bool write(const char *filename);

  /*!
    Get the histogram size.

    \return The size of the histogram, or the image maximum gray
    levels numbers.

    \sa getValues()
  */
  inline unsigned getSize() const { return m_size; }

  /*!

    Get the histogram values.

    \return A pointer to the array of histogram values. The size of
    this array is given by getSize().

    \code
    vpImage<unsigned char> I; // A gray level image

    vpHistogram h;
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    unsigned char *values = h.getValues();
    for (int i=0; i < h.getSize(); ++i)
      printf("%d: %d\n", i, values[i]);
    \endcode

    \sa getSize()
  */
  inline unsigned *getValues() { return m_histogram; }

  /**
   * \brief Get the total number of pixels in the input image.
   *
   * \return unsigned int Cumulated number of pixels in the input image.
   */
  inline unsigned int getTotal() { return m_total; }

private:
#if defined(VISP_HAVE_THREADS) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  template <typename ArithmeticType>
  struct vpHistogramFloatingPoints_Param_t
  {
    unsigned int m_start_index;
    unsigned int m_end_index;

    unsigned int *m_histogram;
    const ArithmeticType m_minVal;
    const ArithmeticType m_step;
    const vpImage<ArithmeticType> *m_I;
    const vpImage<bool> *m_mask;

    vpHistogramFloatingPoints_Param_t() : m_start_index(0), m_end_index(0), m_histogram(nullptr), m_minVal(0.), m_step(1.), m_I(nullptr), m_mask(nullptr) { }

    vpHistogramFloatingPoints_Param_t(unsigned int start_index, unsigned int end_index, const ArithmeticType &minVal, const ArithmeticType &step, const vpImage<ArithmeticType> *const I, const vpImage<bool> *const mask)
      : m_start_index(start_index), m_end_index(end_index), m_histogram(nullptr), m_minVal(minVal), m_step(step), m_I(I), m_mask(mask)
    { }

    ~vpHistogramFloatingPoints_Param_t()
    {
      if (m_histogram != nullptr) {
        delete[] m_histogram;
      }
    }
  };

  template <typename ArithmeticType>
  static typename std::enable_if<std::is_floating_point<ArithmeticType>::value, void>::type computeHistogramFloatingPointThread(vpHistogramFloatingPoints_Param_t<ArithmeticType> *histogram_param)
  {
    unsigned int start_index = histogram_param->m_start_index;
    unsigned int end_index = histogram_param->m_end_index;
    unsigned int stopUnroll = end_index - 8;
    unsigned int current_index = start_index;

    const vpImage<ArithmeticType> *I = histogram_param->m_I;

    // Compute the index of a floating point value according to the min and step
    const ArithmeticType &minVal = histogram_param->m_minVal;
    const ArithmeticType &step = histogram_param->m_step;

    auto computeIndex = [&minVal, &step](const ArithmeticType &val) {
      return static_cast<unsigned int> (std::floor((val - minVal)/step));
      };

    const bool alwaysTrue = true;
    const bool *ptrMaskCurrent = &alwaysTrue;
    if (histogram_param->m_mask) {
      ptrMaskCurrent = (const bool *)histogram_param->m_mask->bitmap + start_index;
    }

    if (end_index >= 8 + start_index) {
      // Unroll loop version
      while (current_index <= stopUnroll) {
        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }

        if (*ptrMaskCurrent) {
          unsigned int id = computeIndex(I->bitmap[current_index]);
          histogram_param->m_histogram[id]++;
        }
        ++current_index;
        if (histogram_param->m_mask != nullptr) {
          ++ptrMaskCurrent;
        }
      }
    }

    while (current_index < end_index) {
      if (*ptrMaskCurrent) {
        unsigned int id = computeIndex(I->bitmap[current_index]);
        histogram_param->m_histogram[id]++;
      }
      if (histogram_param->m_mask != nullptr) {
        ++ptrMaskCurrent;
      }
      ++current_index;
    }
  }
#endif // DOXYGEN_SHOULD_SKIP_THIS
#endif

  void init(unsigned size = 256);

  unsigned int *m_histogram; /*!< The storage for the histogram.*/
  unsigned m_size; /*!< Histogram size (max allowed 256).*/
  const vpImage<bool> *mp_mask; /*!< Mask that permits to consider only the pixels for which the mask is true.*/
  unsigned int m_total; /*!< Cumulated number of pixels in the input image. */
  static const unsigned int constr_val_256;
};
END_VISP_NAMESPACE
#endif
