/****************************************************************************
 *
 * $Id: vpHistogram.cpp,v 1.1 2007-03-02 13:05:11 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <visp/vpHistogram.h>



/*!
  Defaut constructor for a gray level histogram.
*/
vpHistogram::vpHistogram()
{
  histogram = NULL;
  size = 256;
  init();
}

/*!
  Copy constructor of a gray level histogram.
*/
vpHistogram::vpHistogram(const vpHistogram &h)
{
  //  vpTRACE("copy");
  histogram = NULL;
  size = 256;
  init(h.size);
  memcpy(histogram, h.histogram, size * sizeof(unsigned));
}

/*!
  Calculates the histrogram from a gray level image.

  \param I : Gray level image.

  \sa calculate()
*/
vpHistogram::vpHistogram(const vpImage<unsigned char> &I)
{
  histogram = NULL;
  size = 256;
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
    delete [] histogram;
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
vpHistogram &
vpHistogram::operator=(const vpHistogram &h)
{
  histogram = NULL;
  init(h.size);
  memcpy(histogram, h.histogram, size * sizeof(unsigned));

  return *this;
}

/*!
  Initialise the histogram structure.
  - Allocates the array
  - Initialise all the values to zero.
*/
void
vpHistogram::init(unsigned size)
{
  if (histogram != NULL) {
    delete [] histogram;
    histogram = NULL;
  }
  
  this->size = size;
  histogram = new unsigned [size];

  memset(histogram, 0, size * sizeof(unsigned));

  //  vpTRACE("alloc: %p", &histogram);
}


/*!

  Calculate the histogram from a gray level image.

  \param I : Gray level image.

*/
void vpHistogram::calculate(const vpImage<unsigned char> &I)
{
  memset(histogram, 0, size * sizeof(unsigned));

  for (unsigned i=0; i < I.getHeight(); i ++) {
    for (unsigned j=0; j < I.getWidth(); j ++) {
      histogram[ I[i][j] ] ++;
    }
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
void
vpHistogram::smooth(unsigned fsize)
{
  if (histogram == NULL) {
    vpERROR_TRACE("Histogram array not initialised\n");
    throw (vpImageException(vpImageException::notInitializedError,
			    "Histogram array not initialised")) ;
  }

  vpHistogram h;
  h = *this;

  int hsize = fsize / 2; // half filter size
  unsigned sum;
  unsigned nb;

  for (unsigned i=0; i < size; i ++) {
    sum = 0;
    nb = 0;
    for (int j=-hsize; j <= hsize; j ++) {
      if ( (i + j) >= 0 && (i + j) < size ) {
	sum += h.histogram[i + j];
	nb ++;
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
unsigned vpHistogram::getPeaks(vpList<vpHistogramPeak> & peaks)
{
  if (histogram == NULL) {
    vpERROR_TRACE("Histogram array not initialised\n");
    throw (vpImageException(vpImageException::notInitializedError,
			    "Histogram array not initialised")) ;
  }

  int prev_slope;              // Previous histogram inclination
  int next_slope;              // Next histogram inclination
  vpHistogramPeak p;           // An histogram peak
  unsigned nbpeaks; // Number of peaks in the histogram (ie local maxima)

  if ( ! peaks.empty() )
    peaks.kill();

  peaks.front();

  // Parse the histogram to get the local maxima
  unsigned cpt = 0;
  unsigned sum_level = 0;
  nbpeaks = 0;
  prev_slope = 1;

  for (unsigned i = 0; i < size-1; i++) {
    next_slope = histogram[i+1] - histogram[i];

//     if ((prev_slope < 0) && (next_slope > 0) ) {
//       sum_level += i;
//       cpt ++;
//       continue;
//     }

    if ((prev_slope > 0) && (next_slope == 0) ) {
      sum_level += i + 1;
      cpt ++;
      continue;
    }


    // Peak detection
    if ( (prev_slope > 0) && (next_slope < 0) ) {
      sum_level += i;
      cpt ++;
      
      unsigned level = sum_level / cpt;
      p.set(level, histogram[level]);
      //      vpTRACE("add %d %d", p.getLevel(), p.getValue());
      peaks.addRight(p);

      nbpeaks ++;

    }
   
    prev_slope = next_slope;   
    sum_level = 0;
    cpt = 0;
  }
  if (prev_slope > 0) {
    p.set(size-1, histogram[size-1]);
    //      vpTRACE("add %d %d", p.getLevel(), p.getValue());
    peaks.addRight(p);
    nbpeaks ++;
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
unsigned vpHistogram::getPeaks(unsigned char dist, 
			       vpHistogramPeak & peak1, 
			       vpHistogramPeak & peak2)
{
  vpList<vpHistogramPeak> peaks;
  unsigned nbpeaks; // Number of peaks in the histogram (ie local maxima)

  nbpeaks = getPeaks(peaks);
  sort(peaks);
  peaks.front();
  
  if (nbpeaks == 0) {
    peak1.set(0, 0);
    peak2.set(0, 0);
    return 0;
  }

  if (nbpeaks == 1) {
    peak1 = peaks.value();
    peak2.set(0, 0);
    return 1;
  }

  // Parse the peaks list to get the peak with a distance greater
  // than dist to the highest
  peak1 = peaks.value();
  peaks.next();
  while (! peaks.outside() ) {
    vpHistogramPeak p;
    p = peaks.value();
    if (abs(p.getLevel() - peak1.getLevel()) > dist) {
      // The second peak is found
      peak2 = p;
      return 2;
    }
    peaks.next();
  }

  // No second peak found
  peak2.set(0, 0);
  return 1;
}


/*!

  Determine the two highest peaks in the histogram and compute a
  threshold to separate the two objects. Here we dont now which is the
  highest peak. It could be \e peakl or \e peakr.

  \param dist  : Distance between two significative histogram maxima
  \param peakl : Position of the left histogram peak.
  \param peakr : Position of the right histogram peak.

  \param valey : Valey between the two peaks \e peakl and \e peakr.

  \return true if the histogram is bimodal, false otherwise.
*/

bool
vpHistogram::getPeaks(unsigned char dist, 
		      vpHistogramPeak & peakl,
		      vpHistogramPeak & peakr, 
		      vpHistogramValey & valey)
{
  unsigned char *peak;              // Local maxima values
  int prev_slope;              // Previous histogram inclination
  int next_slope;              // Next histogram inclination
  unsigned index_highest_peak; // Index in peak[] array of the highest peak 
  unsigned index_second_peak;  // Index in peak[] array of the second peak 

  unsigned int prof;             
  unsigned int maxprof;       	       // Nb pixels difference between 2 maxi peaks
  unsigned int nbmini;             // Minimum numbers
  unsigned int sumindmini;         // Sum 
  unsigned int mini;                    // current minimum
  unsigned int nbpeaks; // Number of peaks in the histogram (ie local maxima)

  // Init the valey
  valey.set(0, 0);

  // Allocation for the 
  peak = new unsigned char [size];

  // Parse the histogram to get the local maxima
  nbpeaks = 0;
  prev_slope = 1;
  for (unsigned i = 0; i < size-1; i++) {
    next_slope = histogram[i+1] - histogram[i];
    if (next_slope == 0) 
      continue;
    // Peak detection
    if ( (prev_slope > 0) && (next_slope < 0) )
      peak[nbpeaks ++] = i;
    
    prev_slope = next_slope;   
  }
  if (prev_slope > 0)
    peak[nbpeaks ++] = size-1;

//   vpTRACE("nb peaks: %d", nbpeaks);
//   for (unsigned i=0; i < nbpeaks; i ++)
//     vpTRACE("peak %d: pos %d value: %d", i, peak[i], histogram[ peak[i] ]);
  
  // Get the global maximum
  index_highest_peak = 0;
  for (unsigned int i=0; i < nbpeaks; i++) {
    if (histogram[ peak[i] ] > histogram[ peak[index_highest_peak] ]) {
      index_highest_peak = i;
    }
  }

//   vpTRACE("highest peak index: %d pos: %d value: %d", 
// 	  index_highest_peak, peak[index_highest_peak],
// 	  histogram[ peak[index_highest_peak] ]);

  maxprof=0;
  index_second_peak=index_highest_peak;

  // Search second local maximum on the left of the global maximum
  for (unsigned i = 0; i < index_highest_peak; i++) {
    if (peak[index_highest_peak] - peak[i] > dist) {
      prof=0;
      for (int j=peak[i]; j <= peak[index_highest_peak]-dist; j++)
	if((histogram[peak[i]] - histogram[j]) > prof) 
	  prof = histogram[peak[i]] - histogram[j];

      if (prof > maxprof) {
	maxprof = prof;
	index_second_peak = i;
      }
    }
  }

  // Search second local maximum on the right of the global maximum
  for (unsigned i = index_highest_peak+1; i < nbpeaks; i++) {
    if (peak[i] - peak[index_highest_peak] > dist) {
      prof=0;
      for (unsigned j=peak[index_highest_peak]+dist; j <= peak[i]; j++)
	if((histogram[peak[i]] - histogram[j]) > prof)
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
  if (peak[index_highest_peak] < peak[index_second_peak])
  {
    peakr.set(peak[index_second_peak],  histogram[ peak[index_second_peak] ]);
    peakl.set(peak[index_highest_peak], histogram[ peak[index_highest_peak] ]);
  }
  else
  {
    peakl.set(peak[index_second_peak],  histogram[ peak[index_second_peak] ]);
    peakr.set(peak[index_highest_peak], histogram[ peak[index_highest_peak]]); 
  }

  if (peakl == peakr) {

    delete [] peak;

    return (false); // Not a bimodal histogram
  }

  // Search the valey
  mini = peakl.getValue();
  sumindmini=0;
  nbmini=0;
  for (unsigned i=peakl.getLevel(); i <= peakr.getLevel(); i++) {
    if (histogram[i] < mini)  {
      mini=histogram[i];
      nbmini=1;
      sumindmini=i;
      continue;
    }
    if (histogram[i] == mini) {
      sumindmini += i;
      nbmini ++;
    }
  }
  // vpTRACE("nbmini %d", nbmini);
  // vpTRACE("sumindmini %d", sumindmini);
  // vpTRACE("mini: indmini: %d", sumindmini/nbmini);

  if (nbmini == 0) {
    // no valey found
    valey.set(0, 0); 

    delete [] peak;
  
    return false;
  }
  else {
    mini = sumindmini/nbmini; // mean
    valey.set(mini, histogram[mini]); 

    delete [] peak;
  
    return (true);
  }
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
bool
vpHistogram::getValey(const vpHistogramPeak & peak1, 
		      const vpHistogramPeak & peak2,
		      vpHistogramValey & valey)
{

  // Set the left and right peaks
  vpHistogramPeak peakl, peakr;
  if (peak1.getLevel() > peak2.getLevel()) {
    peakl = peak2;
    peakr = peak1;
  }
  else {
    peakl = peak1;
    peakr = peak2;
  }

  // Search the valey
  unsigned int nbmini;             // Minimum numbers
  unsigned int sumindmini;         // Sum 
  unsigned int mini;                    // current minimum

  mini = peakl.getValue();
  sumindmini=0;
  nbmini=0;
  for (unsigned i=peakl.getLevel(); i <= peakr.getLevel(); i++) {
    if (histogram[i] < mini)  {
      mini=histogram[i];
      nbmini=1;
      sumindmini=i;
      continue;
    }
    if (histogram[i] == mini) {
      sumindmini += i;
      nbmini ++;
    }
  }

  if (nbmini == 0) {
    // no valey found
    valey.set(0, 0);
  
    return false;
  }
  else {
    unsigned char minipos = sumindmini/nbmini; // position of the minimum
    
    valey.set(minipos, histogram[minipos]);
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
unsigned    
vpHistogram::getValey(unsigned char dist,
		      const vpHistogramPeak & peak, 
		      vpHistogramValey & valeyl, 
		      vpHistogramValey & valeyr)
{
  unsigned int ret = 0x11;
  unsigned int nbmini;             // Minimum numbers
  unsigned int sumindmini;         // Sum 
  unsigned int mini;                    // current minimum
  vpHistogramPeak peakr, peakl; // Left and right peaks of peak
  vpList<vpHistogramPeak> peaks; // list of histogram peaks
  unsigned int nbpeaks; // Number of peaks in the histogram (ie local maxima)

  if (peak.getLevel() == 0) {
    valeyl.set(0, 0);
    ret &= 0x01;
  }
  if (peak.getLevel() == size -1) {
    valeyr.set(size-1, 0);
    ret &= 0x10;
  }

  if (ret >> 1) {
    // If the list of peaks is empty, compute it
    if (peaks.empty()) {
      nbpeaks = getPeaks(peaks);
    }

    if (1) {
      //      vpTRACE("nb peaks: %d", nbpeaks);
      peaks.front();
      for (unsigned i=0; i < nbpeaks; i ++) {
	vpHistogramPeak p = peaks.value();
// 	vpTRACE("peak index %d: pos %d value: %d", 
// 		i, p.getLevel(), p.getValue());
	peaks.next();
      }
    }
    // Go to the requested peak in the list
    peaks.front();
    unsigned index = 0;
    while ( ! peaks.outside() ) {
      peakl = peaks.value();
      if (peak == peakl) {
	// we are on the peak.
	break;
      }
      peaks.next();
      index ++;
    }

    bool found = false;
    if (index == 0) {
      // No chance to get a peak on the left 
      // should not occur !
      peakl.set(0,0);
    }
    else {
      while ( ! peaks.previousOutside() ) {
	peakl = peaks.previousValue();
	if (abs(peakl.getLevel() - peak.getLevel()) > dist) {
	  // peakl found
	  found = true;
	  break;
	}
	peaks.previous();
      }
    }
    if ( ! found)
      peakl.set(0,0);

    // Search the valey on the left 
    mini = peak.getValue();
    sumindmini=0;
    nbmini=0;
    for (unsigned i=peakl.getLevel(); i < peak.getLevel(); i++) {
      if (histogram[i] < mini)  {
	mini=histogram[i];
	nbmini=1;
	sumindmini=i;
	continue;
      }
      if (histogram[i] == mini) {
	sumindmini += i;
	nbmini ++;
      }
    }
    if (nbmini == 0) {
      valeyl.set(0, 0);
      ret &= 0x01;
    }
    else {
      unsigned char minipos = sumindmini/nbmini; // position of the minimum
      valeyl.set(minipos, histogram[minipos]);
      ret &= 0x11;
    }
  }

  if (ret << 1) {
    // If the list of peaks is empty, compute it
    if (peaks.empty()) {
      nbpeaks = getPeaks(peaks);
    }
    // Go to the requested peak in the list
    peaks.front();
    unsigned index = 0;
    while ( ! peaks.outside() ) {
      peakr = peaks.value();
      if (peak == peakr) {
	// we are on the peak.
	break;
      }
      peaks.next();
      index ++;
    }
    bool found = false;
    while ( !peaks.outside() ) {
      peakr = peaks.value();
      if (abs(peakr.getLevel() - peak.getLevel()) > dist) {
	// peakr found
	found = true;
	break;
      }
      peaks.next();
    }
    
    if ( ! found)
      peakr.set(size-1,0);

    // Search the valey on the right
    mini = peak.getValue();
    sumindmini=0;
    nbmini=0;
    for (unsigned i=peak.getLevel()+1; i <= peakr.getLevel(); i++) {
      if (histogram[i] < mini)  {
	mini=histogram[i];
	nbmini=1;
	sumindmini=i;
	continue;
      }
      if (histogram[i] == mini) {
	sumindmini += i;
	nbmini ++;
      }
    }
    if (nbmini == 0) {
      valeyr.set(size-1, 0);
      ret &= 0x10;
    }
    else {
      unsigned char minipos = sumindmini/nbmini; // position of the minimum
      valeyr.set(minipos, histogram[minipos]);
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
unsigned vpHistogram::sort(vpList<vpHistogramPeak> & peaks)
{

  vpList<vpHistogramPeak> _peaks;
  vpHistogramPeak p, _p;       // A peak
  unsigned nbpeaks = 0;

  if ( peaks.empty() ) {
    nbpeaks = 0;
    return nbpeaks;
  }

  _peaks = peaks;
  peaks.kill(); // erase the output list
  // Sort the list of peaks from highest to the lowest
  peaks.front();
  _p = _peaks.value();
  // Add first value in the list
  peaks.addRight(_p);
  nbpeaks ++;
  // parse the rest of the list
  _peaks.next();
  while (! _peaks.outside() ) {
    _p = _peaks.value();
    peaks.front();
    while (! peaks.outside() ) {
      p = peaks.value();
      if (_p.getValue() > p.getValue()) {
	peaks.addLeft(_p);
	nbpeaks ++;
	break;
      }
      if( peaks.nextOutside()) {
	peaks.addRight(_p);
	nbpeaks ++;
	break;
      }
	
      peaks.next();
    }
    
    _peaks.next();
  }
  return nbpeaks;
}

/*!

  Write the histogram values in a file.

  In the file, on each line you will find first the gray level and
  than the number of pixels which have this gray level.

  \param filename : File name to write with histogram values.
  
  \return true if the file could be written, false otherwise.
*/

bool
vpHistogram::write(const string &filename)
{
  return ( this->write(filename.c_str()) );
}

/*!

  Write the histogram values in a file.

  In the file, on each line you will find first the gray level and
  than the number of pixels which have this gray level.

  \param filename : File name to write with histogram values.
  
  \return true if the file could be written, false otherwise.
*/

bool
vpHistogram::write(const char *filename)
{
  string opath;

  FILE *fd = fopen(filename, "w");
  if (fd == NULL)
    return false;
  for (unsigned i=0; i < size; i ++)
    fprintf(fd, "%d %d\n", i, histogram[i]);
  fclose(fd);

  return true;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
