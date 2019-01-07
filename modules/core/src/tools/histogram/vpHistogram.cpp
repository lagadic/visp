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
 * Gray level histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpHistogram.cpp
  \brief Definition of the vpHistogram class member functions.

  Class vpHistogram allows to calculate and manipulate gray level
  image histograms.

*/

#include <stdlib.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImageConvert.h>

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
#include <visp3/core/vpThread.h>

namespace
{
struct Histogram_Param_t {
  unsigned int m_start_index;
  unsigned int m_end_index;

  unsigned int m_lut[256];
  unsigned int *m_histogram;
  const vpImage<unsigned char> *m_I;

  Histogram_Param_t() : m_start_index(0), m_end_index(0), m_lut(), m_histogram(NULL), m_I(NULL) {}

  Histogram_Param_t(const unsigned int start_index, const unsigned int end_index, const vpImage<unsigned char> *const I)
    : m_start_index(start_index), m_end_index(end_index), m_lut(), m_histogram(NULL), m_I(I)
  {
  }

  ~Histogram_Param_t()
  {
    if (m_histogram != NULL) {
      delete[] m_histogram;
    }
  }
};

vpThread::Return computeHistogramThread(vpThread::Args args)
{
  Histogram_Param_t *histogram_param = static_cast<Histogram_Param_t *>(args);
  unsigned int start_index = histogram_param->m_start_index;
  unsigned int end_index = histogram_param->m_end_index;

  const vpImage<unsigned char> *I = histogram_param->m_I;

  unsigned char *ptrStart = (unsigned char *)(I->bitmap) + start_index;
  unsigned char *ptrEnd = (unsigned char *)(I->bitmap) + end_index;
  unsigned char *ptrCurrent = ptrStart;

  //    while(ptrCurrent != ptrEnd) {
  //      histogram_param->m_histogram[ histogram_param->m_lut[ *ptrCurrent ]
  //      ] ++;
  //      ++ptrCurrent;
  //    }

  if (end_index - start_index >= 8) {
    // Unroll loop version
    for (; ptrCurrent <= ptrEnd - 8;) {
      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;

      histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
      ++ptrCurrent;
    }
  }

  for (; ptrCurrent != ptrEnd; ++ptrCurrent) {
    histogram_param->m_histogram[histogram_param->m_lut[*ptrCurrent]]++;
  }

  return 0;
}
}
#endif

bool compare_vpHistogramPeak(vpHistogramPeak first, vpHistogramPeak second);

// comparison,
bool compare_vpHistogramPeak(vpHistogramPeak first, vpHistogramPeak second)
{
  if (first.getValue() > second.getValue())
    return true;
  else
    return false;
}

/*!
  Defaut constructor for a gray level histogram.
*/
vpHistogram::vpHistogram() : histogram(NULL), size(256) { init(); }

/*!
  Copy constructor of a gray level histogram.
*/
vpHistogram::vpHistogram(const vpHistogram &h) : histogram(NULL), size(256)
{
  init(h.size);
  memcpy(histogram, h.histogram, size * sizeof(unsigned));
}

/*!
  Calculates the histrogram from a gray level image.

  \param I : Gray level image.

  \sa calculate()
*/
vpHistogram::vpHistogram(const vpImage<unsigned char> &I) : histogram(NULL), size(256)
{
  init();

  calculate(I);
}

/*!
  Destructor.
*/
vpHistogram::~vpHistogram()
{
  if (histogram != NULL) {
    //    vpTRACE("free: %p", &histogram);
    delete[] histogram;
    histogram = NULL;
    size = 0;
  }
}

/*!

  Copy operator.
  \param h : Histogram to copy.

  \code
  vpImage<unsigned char> I;

  vpHistogram h(I);
  vpHistogram hcopied = h;

  \endcode
*/
vpHistogram &vpHistogram::operator=(const vpHistogram &h)
{
  init(h.size);
  memcpy(histogram, h.histogram, size * sizeof(unsigned));

  return *this;
}

/*!
  Initialise the histogram structure.
  - Allocates the array
  - Initialise all the values to zero.
*/
void vpHistogram::init(unsigned size_)
{
  if (histogram != NULL) {
    delete[] histogram;
    histogram = NULL;
  }

  this->size = size_;
  histogram = new unsigned[size];

  memset(histogram, 0, size * sizeof(unsigned));

  //  vpTRACE("alloc: %p", &histogram);
}

/*!

  Calculate the histogram from a gray level image.

  \param I : Gray level image.
  \param nbins : Number of bins to compute the histogram.
  \param nbThreads : Number of threads to use for the computation.
*/
void vpHistogram::calculate(const vpImage<unsigned char> &I, const unsigned int nbins, const unsigned int nbThreads)
{
  if (size != nbins) {
    if (histogram != NULL) {
      delete[] histogram;
      histogram = NULL;
    }

    size = nbins > 256 ? 256 : (nbins > 0 ? nbins : 256);
    if (nbins > 256 || nbins == 0) {
      std::cerr << "nbins=" << nbins << " , nbins should be between ]0 ; 256] ; use by default nbins=256" << std::endl;
    }
    histogram = new unsigned int[size];
  }

  memset(histogram, 0, size * sizeof(unsigned int));

  bool use_single_thread;
#if !defined(VISP_HAVE_PTHREAD) && !defined(_WIN32)
  use_single_thread = true;
#else
  use_single_thread = (nbThreads == 0 || nbThreads == 1);
#endif

  if (!use_single_thread && I.getSize() <= nbThreads) {
    use_single_thread = true;
  }

  unsigned int lut[256];
  for (unsigned int i = 0; i < 256; i++) {
    lut[i] = (unsigned int)(i * size / 256.0);
  }

  if (use_single_thread) {
    // Single thread

    unsigned int size_ = I.getWidth() * I.getHeight();
    unsigned char *ptrStart = (unsigned char *)I.bitmap;
    unsigned char *ptrEnd = ptrStart + size_;
    unsigned char *ptrCurrent = ptrStart;

    while (ptrCurrent != ptrEnd) {
      histogram[lut[*ptrCurrent]]++;
      ++ptrCurrent;
    }
  } else {
#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))
    // Multi-threads

    std::vector<vpThread *> threadpool;
    std::vector<Histogram_Param_t *> histogramParams;

    unsigned int image_size = I.getSize();
    unsigned int step = image_size / nbThreads;
    unsigned int last_step = image_size - step * (nbThreads - 1);

    for (unsigned int index = 0; index < nbThreads; index++) {
      unsigned int start_index = index * step;
      unsigned int end_index = (index + 1) * step;

      if (index == nbThreads - 1) {
        end_index = start_index + last_step;
      }

      Histogram_Param_t *histogram_param = new Histogram_Param_t(start_index, end_index, &I);
      histogram_param->m_histogram = new unsigned int[size];
      memset(histogram_param->m_histogram, 0, size * sizeof(unsigned int));
      memcpy(histogram_param->m_lut, lut, 256 * sizeof(unsigned int));

      histogramParams.push_back(histogram_param);

      // Start the threads
      vpThread *histogram_thread = new vpThread((vpThread::Fn)computeHistogramThread, (vpThread::Args)histogram_param);
      threadpool.push_back(histogram_thread);
    }

    for (size_t cpt = 0; cpt < threadpool.size(); cpt++) {
      // Wait until thread ends up
      threadpool[cpt]->join();
    }

    for (unsigned int cpt1 = 0; cpt1 < size; cpt1++) {
      unsigned int sum = 0;

      for (size_t cpt2 = 0; cpt2 < histogramParams.size(); cpt2++) {
        sum += histogramParams[cpt2]->m_histogram[cpt1];
      }

      histogram[cpt1] = sum;
    }

    // Delete
    for (size_t cpt = 0; cpt < threadpool.size(); cpt++) {
      delete threadpool[cpt];
    }

    for (size_t cpt = 0; cpt < histogramParams.size(); cpt++) {
      delete histogramParams[cpt];
    }
#endif
  }
}

/*!
  Display the histogram distribution in an image, the minimal image size is
  36x36 px.

  \param I : Image with the histogram distribution displayed.
  \param color : Color used for the display.
  \param thickness : Thickness of the line.
  \param maxValue_ : Maximum value in the histogram, if 0 it will be computed
  from the current histogram. Useful to plot a 3 channels histogram for a RGB
  image for example to keep a coherent vertical scale between the channels.
*/
void vpHistogram::display(const vpImage<unsigned char> &I, const vpColor &color, const unsigned int thickness,
                          const unsigned int maxValue_)
{
  unsigned int width = I.getWidth(), height = I.getHeight();
  // Minimal width and height are 36 px
  if (width < 36 || height < 36) {
    std::cerr << "Image I must have at least width >= 36 && height >= 36 !" << std::endl;
    return;
  }

  unsigned int maxValue = maxValue_;
  if (maxValue == 0) {
    for (unsigned int i = 0; i < size; i++) {
      if (histogram[i] > maxValue) {
        maxValue = histogram[i];
      }
    }
  }

  if (maxValue == 0) {
    throw(vpException(vpException::divideByZeroError, "Cannot display histogram; max value=0"));
  }
  // Keep 12 free pixels at the top
  unsigned int max_height = height - 12;
  double ratio_height = max_height / (double)maxValue;
  double ratio_width = (width - 1) / (double)(size - 1.0);

  for (unsigned int i = 1; i < size; i++) {
    unsigned int value1 = histogram[i - 1];
    unsigned int value2 = histogram[i];

    vpImagePoint startPt((height - 1) - (value1 * ratio_height), (i - 1) * ratio_width);
    vpImagePoint endPt((height - 1) - (value2 * ratio_height), (i * ratio_width));
    vpDisplay::displayLine(I, startPt, endPt, color, thickness);
  }
}

/*!

  Smoothes the histogram and returns it.

  A simple average scheme is used where each value \f$h(i)\f$ in the
  histogram is replaced by the average of itself and the neighbours.

  \f[h(i) = \sum_{j=i-\frac{fsize}{2}}^{i+\frac{fsize}{2}} h(j) \f]

  \param fsize : Filter size. Corresponds to the number of values
  around each point used to compute the mean value.

  \return Smoothed histogram.

  \exception vpImageException::notInitializedError : Histogram array
  not initialsed. Means that the histogram was not calculated before.

  \sa calculate()

*/
void vpHistogram::smooth(const unsigned int fsize)
{
  if (histogram == NULL) {
    vpERROR_TRACE("Histogram array not initialised\n");
    throw(vpImageException(vpImageException::notInitializedError, "Histogram array not initialised"));
  }

  vpHistogram h;
  h = *this;

  int hsize = (int)fsize / 2; // half filter size

  for (unsigned i = 0; i < size; i++) {
    unsigned int sum = 0;
    unsigned int nb = 0;
    for (int j = -hsize; j <= hsize; j++) {
      // exploitation of the overflow to detect negative value...
      if (/*(i + j) >= 0 &&*/ (i + (unsigned int)j) < size) {
        sum += h.histogram[i + (unsigned int)j];
        nb++;
      }
    }
    histogram[i] = sum / nb;
  }
}

/*!

  Build a list of all histogram peaks. This peak list is gray level
  sorted from 0 to 255. That mean that the first peak has a gray level
  less than the second one, etc.

  To sort this list from highest peak value to the lowest one, you can
  use sort().

  \param peaks :  List of peaks.
  \return The number of peaks in the histogram.

  \sa sort()
*/
unsigned vpHistogram::getPeaks(std::list<vpHistogramPeak> &peaks)
{
  if (histogram == NULL) {
    vpERROR_TRACE("Histogram array not initialised\n");
    throw(vpImageException(vpImageException::notInitializedError, "Histogram array not initialised"));
  }

  int prev_slope;    // Previous histogram inclination
  vpHistogramPeak p; // An histogram peak
  unsigned nbpeaks;  // Number of peaks in the histogram (ie local maxima)

  peaks.clear();

  // Parse the histogram to get the local maxima
  unsigned cpt = 0;
  unsigned sum_level = 0;
  nbpeaks = 0;
  prev_slope = 1;

  for (unsigned i = 0; i < size - 1; i++) {
    int next_slope = (int)histogram[i + 1] - (int)histogram[i]; // Next histogram inclination

    //     if ((prev_slope < 0) && (next_slope > 0) ) {
    //       sum_level += i;
    //       cpt ++;
    //       continue;
    //     }

    if ((prev_slope > 0) && (next_slope == 0)) {
      sum_level += i + 1;
      cpt++;
      continue;
    }

    // Peak detection
    if ((prev_slope > 0) && (next_slope < 0)) {
      sum_level += i;
      cpt++;

      unsigned int level = sum_level / cpt;
      p.set((unsigned char)level, histogram[level]);
      //      vpTRACE("add %d %d", p.getLevel(), p.getValue());
      peaks.push_back(p);

      nbpeaks++;
    }

    prev_slope = next_slope;
    sum_level = 0;
    cpt = 0;
  }
  if (prev_slope > 0) {
    p.set((unsigned char)size - 1u, histogram[size - 1]);
    //      vpTRACE("add %d %d", p.getLevel(), p.getValue());
    peaks.push_back(p);
    nbpeaks++;
  }

  return nbpeaks;
}

/*!

  Find the two highest peaks in the histogram. Usually, the first
  one correspond to the image background, the second one to
  the object.

  \param dist : Minimal distance between two significative histogram peaks.

  \param peak1 : Highest peak in the histogram.
  \param peak2 : Second highest peak in the histogram.

  \return The number of highest peaks:
  - 2: If the histogram is bimodal
  - 1: If no second highest peak was found.
  - 0: if no peaks were found.

*/
unsigned vpHistogram::getPeaks(unsigned char dist, vpHistogramPeak &peak1, vpHistogramPeak &peak2)
{
  std::list<vpHistogramPeak> peaks;
  unsigned nbpeaks; // Number of peaks in the histogram (ie local maxima)

  nbpeaks = getPeaks(peaks);
  sort(peaks);

  if (nbpeaks == 0) {
    peak1.set(0, 0);
    peak2.set(0, 0);
    return 0;
  }

  if (nbpeaks == 1) {
    peak1 = peaks.front();
    peak2.set(0, 0);
    return 1;
  }

  // Parse the peaks list to get the peak with a distance greater
  // than dist to the highest
  peak1 = peaks.front();
  for (std::list<vpHistogramPeak>::const_iterator it = peaks.begin(); it != peaks.end(); ++it) {
    vpHistogramPeak p = *it;
    if (abs(p.getLevel() - peak1.getLevel()) > dist) {
      // The second peak is found
      peak2 = p;
      return 2;
    }
  }

  // No second peak found
  peak2.set(0, 0);
  return 1;
}

/*!

  Determine the two highest peaks in the histogram and compute a
  threshold to separate the two objects. Here we dont know which is the
  highest peak. It could be \e peakl or \e peakr.

  \param dist  : Distance between two significative histogram maxima
  \param peakl : Position of the left histogram peak.
  \param peakr : Position of the right histogram peak.

  \param valey : Valey between the two peaks \e peakl and \e peakr.

  \return true if the histogram is bimodal, false otherwise.
*/

bool vpHistogram::getPeaks(unsigned char dist, vpHistogramPeak &peakl, vpHistogramPeak &peakr, vpHistogramValey &valey)
{
  unsigned char *peak;         // Local maxima values
  int prev_slope;              // Previous histogram inclination
  unsigned index_highest_peak; // Index in peak[] array of the highest peak
  unsigned index_second_peak;  // Index in peak[] array of the second peak

  unsigned int prof;
  unsigned int maxprof;    // Nb pixels difference between 2 maxi peaks
  unsigned int nbmini;     // Minimum numbers
  unsigned int sumindmini; // Sum
  unsigned int mini;       // current minimum
  unsigned int nbpeaks;    // Number of peaks in the histogram (ie local maxima)

  // Init the valey
  valey.set(0, 0);

  // Allocation for the
  peak = new unsigned char[size];

  // Parse the histogram to get the local maxima
  nbpeaks = 0;
  prev_slope = 1;
  for (unsigned i = 0; i < size - 1; i++) {
    int next_slope = (int)histogram[i + 1] - (int)histogram[i]; // Next histogram inclination
    if (next_slope == 0)
      continue;
    // Peak detection
    if ((prev_slope > 0) && (next_slope < 0))
      peak[nbpeaks++] = (unsigned char)i;

    prev_slope = next_slope;
  }
  if (prev_slope > 0)
    peak[nbpeaks++] = (unsigned char)(size - 1);

  //   vpTRACE("nb peaks: %d", nbpeaks);
  //   for (unsigned i=0; i < nbpeaks; i ++)
  //     vpTRACE("peak %d: pos %d value: %d", i, peak[i], histogram[ peak[i]
  //     ]);

  // Get the global maximum
  index_highest_peak = 0;
  for (unsigned int i = 0; i < nbpeaks; i++) {
    if (histogram[peak[i]] > histogram[peak[index_highest_peak]]) {
      index_highest_peak = i;
    }
  }

  //   vpTRACE("highest peak index: %d pos: %d value: %d",
  // 	  index_highest_peak, peak[index_highest_peak],
  // 	  histogram[ peak[index_highest_peak] ]);

  maxprof = 0;
  index_second_peak = index_highest_peak;

  // Search second local maximum on the left of the global maximum
  for (unsigned i = 0; i < index_highest_peak; i++) {
    if (peak[index_highest_peak] - peak[i] > dist) {
      prof = 0;
      for (int j = peak[i]; j <= peak[index_highest_peak] - dist; j++)
        if ((histogram[peak[i]] - histogram[j]) > prof)
          prof = histogram[peak[i]] - histogram[j];

      if (prof > maxprof) {
        maxprof = prof;
        index_second_peak = i;
      }
    }
  }

  // Search second local maximum on the right of the global maximum
  for (unsigned i = index_highest_peak + 1; i < nbpeaks; i++) {
    if (peak[i] - peak[index_highest_peak] > dist) {
      prof = 0;
      for (int j = peak[index_highest_peak] + dist; j <= peak[i]; j++)
        if ((histogram[peak[i]] - histogram[j]) > prof)
          prof = histogram[peak[i]] - histogram[j];

      if (prof > maxprof) {
        maxprof = prof;
        index_second_peak = i;
      }
    }
  }
  //   vpTRACE("second peak index: %d pos: %d value: %d",
  // 	  index_second_peak, peak[index_second_peak],
  // 	  histogram[ peak[index_second_peak] ]);

  // Determine position of the first and second highest peaks
  if (peak[index_highest_peak] < peak[index_second_peak]) {
    peakr.set(peak[index_second_peak], histogram[peak[index_second_peak]]);
    peakl.set(peak[index_highest_peak], histogram[peak[index_highest_peak]]);
  } else {
    peakl.set(peak[index_second_peak], histogram[peak[index_second_peak]]);
    peakr.set(peak[index_highest_peak], histogram[peak[index_highest_peak]]);
  }

  if (peakl == peakr) {

    delete[] peak;

    return (false); // Not a bimodal histogram
  }

  // Search the valey
  mini = peakl.getValue();
  sumindmini = 0;
  nbmini = 0;
  for (unsigned i = peakl.getLevel(); i <= peakr.getLevel(); i++) {
    if (histogram[i] < mini) {
      mini = histogram[i];
      nbmini = 1;
      sumindmini = i;
      continue;
    }
    if (histogram[i] == mini) {
      sumindmini += i;
      nbmini++;
    }
  }
  // vpTRACE("nbmini %d", nbmini);
  // vpTRACE("sumindmini %d", sumindmini);
  // vpTRACE("mini: indmini: %d", sumindmini/nbmini);

  if (nbmini == 0) {
    // no valey found
    valey.set(0, 0);

    delete[] peak;

    return false;
  } else {
    mini = sumindmini / nbmini; // mean
    valey.set((unsigned char)mini, histogram[mini]);

    delete[] peak;

    return (true);
  }
}

/*!

  Build a list of all histogram valey. This valey list is gray level
  sorted from 0 to 255. That mean that the first valey has a gray level
  less than the second one, etc.

  \param valey :  List of valey.
  \return The number of valey in the histogram.

  \sa sort()
*/
unsigned vpHistogram::getValey(std::list<vpHistogramValey> &valey)
{
  if (histogram == NULL) {
    vpERROR_TRACE("Histogram array not initialised\n");
    throw(vpImageException(vpImageException::notInitializedError, "Histogram array not initialised"));
  }

  int prev_slope;     // Previous histogram inclination
  vpHistogramValey p; // An histogram valey
  unsigned nbvaley;   // Number of valey in the histogram (ie local minima)

  valey.clear();

  // Parse the histogram to get the local minima
  unsigned cpt = 0;
  unsigned sum_level = 0;
  nbvaley = 0;
  prev_slope = -1;

  for (unsigned i = 0; i < size - 1; i++) {
    int next_slope = (int)histogram[i + 1] - (int)histogram[i]; // Next histogram inclination

    if ((prev_slope < 0) && (next_slope == 0)) {
      sum_level += i + 1;
      cpt++;
      continue;
    }

    // Valey detection
    if ((prev_slope < 0) && (next_slope > 0)) {
      sum_level += i;
      cpt++;

      unsigned int level = sum_level / cpt;
      p.set((unsigned char)level, histogram[level]);
      //      vpTRACE("add %d %d", p.getLevel(), p.getValue());
      valey.push_back(p);

      nbvaley++;
    }

    prev_slope = next_slope;
    sum_level = 0;
    cpt = 0;
  }
  if (prev_slope < 0) {
    p.set((unsigned char)size - 1u, histogram[size - 1]);
    //      vpTRACE("add %d %d", p.getLevel(), p.getValue());
    valey.push_back(p);
    nbvaley++;
  }

  return nbvaley;
}

/*!

  Find the valey between two peaks. It starts at the lowest peak and
  works its way up to the highest peak. Along the way, it looks at
  each point in the histogram to find the location of the smallest
  histogram point which is the valey point.

  \param peak1 : A peak in the histogram.
  \param peak2 : A other peak in the histogram.

  \param valey : Low point of the valey between two peaks in a histogram.

  \return true if a valey was found, false otherwise.
*/
bool vpHistogram::getValey(const vpHistogramPeak &peak1, const vpHistogramPeak &peak2, vpHistogramValey &valey)
{

  // Set the left and right peaks
  vpHistogramPeak peakl, peakr;
  if (peak1.getLevel() > peak2.getLevel()) {
    peakl = peak2;
    peakr = peak1;
  } else {
    peakl = peak1;
    peakr = peak2;
  }

  // Search the valey
  unsigned int nbmini;     // Minimum numbers
  unsigned int sumindmini; // Sum
  unsigned int mini;       // current minimum

  mini = peakl.getValue();
  sumindmini = 0;
  nbmini = 0;
  for (unsigned i = peakl.getLevel(); i <= peakr.getLevel(); i++) {
    if (histogram[i] < mini) {
      mini = histogram[i];
      nbmini = 1;
      sumindmini = i;
      continue;
    }
    if (histogram[i] == mini) {
      sumindmini += i;
      nbmini++;
    }
  }

  if (nbmini == 0) {
    // no valey found
    valey.set(0, 0);

    return false;
  } else {
    unsigned int minipos = sumindmini / nbmini; // position of the minimum

    valey.set((unsigned char)minipos, histogram[minipos]);
    return true;
  }
}
/*!

  Find the two valey around an histogram peak. It starts at the peak
  position and works its way down and up to find the left and right
  valey around the peak.


  \param dist : Minimal distance between two significative histogram peaks.
  \param peak : A peak in the histogram.
  \param valeyl : The valey on the left of the peak.
  \param valeyr : The valey on the right of the peak.

  \return 0x00 : If no valey was found.
  \return 0x01 : If only the right valey was found.
  \return 0x10 : If only the left valey was found.
  \return 0x11 : If two valeys around the peak were found.

*/
unsigned vpHistogram::getValey(unsigned char dist, const vpHistogramPeak &peak, vpHistogramValey &valeyl,
                               vpHistogramValey &valeyr)
{
  unsigned int ret = 0x11;
  unsigned int nbmini;              // Minimum numbers
  unsigned int sumindmini;          // Sum
  unsigned int mini;                // current minimum
  vpHistogramPeak peakr, peakl;     // Left and right peaks of peak
  std::list<vpHistogramPeak> peaks; // list of histogram peaks
  //   unsigned int nbpeaks=0; // Number of peaks in the histogram (ie local
  //   maxima)

  if (peak.getLevel() == 0) {
    valeyl.set(0, 0);
    ret &= 0x01;
  }
  if (peak.getLevel() == size - 1) {
    valeyr.set((unsigned char)(size - 1), 0);
    ret &= 0x10;
  }

  if (ret >> 1) // consider the left part of the requested peak
  {
    // If the list of peaks is empty, compute it
    if (peaks.empty()) {
      /* nbpeaks = */ getPeaks(peaks);
    }

    //     if (1) {
    //       //      vpTRACE("nb peaks: %d", nbpeaks);
    //       peaks.front();
    //       for (unsigned i=0; i < nbpeaks; i ++) {
    // 	vpHistogramPeak p = peaks.value();
    // // 	vpTRACE("peak index %d: pos %d value: %d",
    // // 		i, p.getLevel(), p.getValue());
    // 	peaks.next();
    //       }
    //     }
    // Go to the requested peak in the list
    std::list<vpHistogramPeak>::const_iterator it;
    unsigned index = 0;
    for (it = peaks.begin(); it != peaks.end(); ++it) {
      if (peak == *it) {
        // we are on the peak.
        break;
      }
      index++;
    }

    bool found = false;
    if (index == 0) {
      // No chance to get a peak on the left
      // should not occur !
      peakl.set(0, 0);
    } else {
      // search for the nearest peak on the left that matches the distance
      std::list<vpHistogramPeak>::const_iterator lit; // left iterator
      for (lit = peaks.begin(); lit != it; ++lit) {
        if (abs((*lit).getLevel() - peak.getLevel()) > dist) {
          // peakl found
          peakl = *lit;
          found = true; // we cannot stop here, since the other peaks on the
                        // right may exist
        }
      }
    }
    if (!found)
      peakl.set(0, 0);

    // Search the valey on the left
    mini = peak.getValue();
    sumindmini = 0;
    nbmini = 0;
    for (unsigned i = peakl.getLevel(); i < peak.getLevel(); i++) {
      if (histogram[i] < mini) {
        mini = histogram[i];
        nbmini = 1;
        sumindmini = i;
        continue;
      }
      if (histogram[i] == mini) {
        sumindmini += i;
        nbmini++;
      }
    }
    if (nbmini == 0) {
      valeyl.set(0, 0);
      ret &= 0x01;
    } else {
      unsigned int minipos = sumindmini / nbmini; // position of the minimum
      valeyl.set((unsigned char)minipos, histogram[minipos]);
      ret &= 0x11;
    }
  }

  if (ret << 1) {
    // If the list of peaks is empty, compute it
    if (peaks.empty()) {
      /* nbpeaks = */ getPeaks(peaks);
    }
    // Go to the requested peak in the list
    std::list<vpHistogramPeak>::const_iterator it;
    unsigned index = 0;
    for (it = peaks.begin(); it != peaks.end(); ++it) {
      if (peak == *it) {
        // we are on the peak.
        break;
      }
      index++;
    }

    bool found = false;
    // search for the nearest peak on the right that matches the distance
    std::list<vpHistogramPeak>::const_iterator rit; // right iterator
    for (rit = it; rit != peaks.end(); ++rit)

      if (abs((*rit).getLevel() - peak.getLevel()) > dist) {
        // peakr found
        peakr = *rit;
        found = true;
        break; // we can stop here
      }

    if (!found)
      peakr.set((unsigned char)(size - 1), 0);

    // Search the valey on the right
    mini = peak.getValue();
    sumindmini = 0;
    nbmini = 0;
    for (unsigned i = (unsigned int)peak.getLevel() + 1; i <= (unsigned int)peakr.getLevel(); i++) {
      if (histogram[i] < mini) {
        mini = histogram[i];
        nbmini = 1;
        sumindmini = i;
        continue;
      }
      if (histogram[i] == mini) {
        sumindmini += i;
        nbmini++;
      }
    }
    if (nbmini == 0) {
      valeyr.set((unsigned char)(size - 1), 0);
      ret &= 0x10;
    } else {
      unsigned int minipos = sumindmini / nbmini; // position of the minimum
      valeyr.set((unsigned char)minipos, histogram[minipos]);
      ret &= 0x11;
    }
  }

  return ret;
}

/*!
  Sort a list of histogram peaks from highest to the lowest.

  \param peaks :  List of histogram peaks.
  \return The number of peaks in the histogram.

  \sa getPeak()
*/
unsigned vpHistogram::sort(std::list<vpHistogramPeak> &peaks)
{

  if (peaks.empty()) {
    return 0;
  }

  peaks.sort(compare_vpHistogramPeak);

  return (unsigned int)peaks.size();
}

/*!

  Write the histogram values in a file.

  In the file, on each line you will find first the gray level and
  than the number of pixels which have this gray level.

  \param filename : File name to write with histogram values.

  \return true if the file could be written, false otherwise.
*/

bool vpHistogram::write(const std::string &filename) { return (this->write(filename.c_str())); }

/*!

  Write the histogram values in a file.

  In the file, on each line you will find first the gray level and
  than the number of pixels which have this gray level.

  \param filename : File name to write with histogram values.

  \return true if the file could be written, false otherwise.
*/

bool vpHistogram::write(const char *filename)
{
  FILE *fd = fopen(filename, "w");
  if (fd == NULL)
    return false;
  for (unsigned i = 0; i < size; i++)
    fprintf(fd, "%u %u\n", i, histogram[i]);
  fclose(fd);

  return true;
}
