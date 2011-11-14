/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Gray level histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/



/*!
  \file vpHistogram.h
  \brief Declaration of the vpHistogram class.
  Class vpHistogram defines gray level image histograms

*/

#ifndef vpHistogram_h
#define vpHistogram_h

#include <list>

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpHistogramPeak.h>
#include <visp/vpHistogramValey.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#  include <visp/vpList.h>
#endif


/*!
  \class vpHistogram
  \ingroup Histogram
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
  vpHistogram();
  vpHistogram(const vpHistogram &h);
  vpHistogram(const vpImage<unsigned char> &I);
  virtual ~vpHistogram();

  vpHistogram & operator=(const vpHistogram &h);

  /*!

    Return the number of pixels having the gray \e level.

    \param level : Gray level in the histogram.

    \return Number of pixels having the gray level.

    \code
    vpImage<unsigned char> I; // A gray level image
    
    vpHistogram h; 
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    for (int i=0; i < h.getSize(); i ++)
      printf("%d: %d\n", i, h[i]);
    \endcode

  */
  inline unsigned operator[](const unsigned char level) const
    {
      return histogram[level];

    };
  /*!

    Return the number of pixels having the gray \e level.

    \param level : Gray level in the histogram.

    \return Number of pixels having the gray level.

    \code
    vpImage<unsigned char> I; // A gray level image
    
    vpHistogram h; 
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    for (int i=0; i < h.getSize(); i ++)
      printf("%d: %d\n", i, h(i));
    \endcode

  */
  inline unsigned operator()(const unsigned char level) const
    {
      return histogram[level];
    };
  /*!

    Return the number of pixels having the gray \e level.

    \param level : Gray level in the histogram.

    \return Number of pixels having the gray level.

    \code
    vpImage<unsigned char> I; // A gray level image
    
    vpHistogram h; 
    h.calculate(I); // Histogram of the gray level image

    // Print the histogram values
    for (int i=0; i < h.getSize(); i ++)
      printf("%d: %d\n", i, h.get(i));
    \endcode

  */
  inline unsigned get(const unsigned char level) const
    {
      return histogram[level];
    };

  /*!

    Set the number of pixels having the gray \e level.

    \param level : Gray level in the histogram. Level is in [0:255]

    \param value : Number of pixels having the gray level.

    \code
    vpHistogram h; 

    // Set histogram values
    for (int i=0; i < h.getSize(); i ++)
      h.set(i, i*2); // for each level i, set a value of 2*i
    \endcode

  */
  inline void set(const unsigned char level, unsigned int value)
    {
      histogram[level] = value;
    };

  void     calculate(const vpImage<unsigned char> &I);
  void     smooth(const unsigned int fsize = 3);
  unsigned getPeaks(std::list<vpHistogramPeak> & peaks);
  unsigned getPeaks(unsigned char dist, 
                    vpHistogramPeak & peak1,
                    vpHistogramPeak & peak2);
  bool     getPeaks(unsigned char dist, 
                    vpHistogramPeak & peakl,
                    vpHistogramPeak & peakr,
                    vpHistogramValey & valey);
  unsigned getValey(std::list<vpHistogramValey> & valey);
  bool     getValey(const vpHistogramPeak & peak1, 
                    const vpHistogramPeak & peak2,
                    vpHistogramValey & valey);
  unsigned getValey(unsigned char dist,
                    const vpHistogramPeak & peak,
                    vpHistogramValey & valeyl,
                    vpHistogramValey & valeyr);
  unsigned sort(std::list<vpHistogramPeak> & peaks);

  bool     write(const std::string &filename);
  bool     write(const char *filename);

  /*!
    Get the histogram size.

    \return The size of the histogram, or the image maximum gray
    levels numbers.

    \sa getValues()
  */
  inline unsigned getSize()
    { 
      return size; 
    };

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
    for (int i=0; i < h.getSize(); i ++)
      printf("%d: %d\n", i, values[i]);
    \endcode

    \sa getSize()
  */
  inline unsigned * getValues() 
    {
      return histogram;
    };

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  vp_deprecated unsigned getPeaks(vpList<vpHistogramPeak> & peaks);
  vp_deprecated unsigned getValey(vpList<vpHistogramValey> & valey);
  vp_deprecated unsigned sort(vpList<vpHistogramPeak> & peaks);
#endif

private:
  void init(unsigned size = 256);

  unsigned int *histogram;
  unsigned size; // Histogram size (max allowed 256)
};



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
