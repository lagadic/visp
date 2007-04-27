/****************************************************************************
 *
 * $Id: histogram.cpp,v 1.3 2007-04-27 16:40:14 fspindle Exp $
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
 * Example of Histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file histogram.cpp

  \brief Histogram manipulation.
*/


/*!
  \example histogram.cpp

  Example of a gray level histogram manipulation.
*/

#include <stdio.h>
#include <sstream>
#include <iomanip>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>


#include <visp/vpImage.h>
#include <visp/vpHistogram.h>



int
main()
{
  // Create a test image.
  // 3 areas were defines;
  // - one is the background with gray level bg
  // - another for a big rectangle with gray level fg
  // - the last one for a small rectangle with gray level mg
  unsigned char fg = 255, bg = 0, mg = 128;
  vpImage<unsigned char> I(384, 288);
  I = bg;

  for (int i=100; i < 150; i ++)
  for (int j=100; j < 150; j ++)
    I[i][j] = fg;
  for (int i=10; i < 15; i ++)
  for (int j=10; j < 15; j ++)
    I[i][j] = mg;


  unsigned distance = 60;
  vpHistogram h;

  // Computes the histogram from the image
  h.calculate(I);

  if (1){
    std::cout << "Histogram image: " << std::endl;
    for (unsigned int i=0; i < h.getSize(); i ++)
      printf("%d: %d\n", i, h[i]);
  }

  // Smooth the histogram
  h.smooth();

  if (1){
    std::cout << "Smoothed histogram image: " << std::endl;
    unsigned *values = h.getValues();
    for (unsigned int i=0; i < h.getSize(); i ++)
      printf("%d: %d\n", i, values[i]);
  }

  vpList<vpHistogramPeak> peaks;
  unsigned int nbpeaks = 0;

  // get all the histogram peaks
  nbpeaks = h.getPeaks(peaks);

  vpTRACE("List of peaks");
  vpTRACE("Nb peaks: %d", nbpeaks);
  if (nbpeaks) {
    peaks.front();
    while (! peaks.outside() ) {
      vpHistogramPeak p;
      p = peaks.value();
      vpTRACE("Peak: gray level: %d value: %d", p.getLevel(), p.getValue());
      peaks.next();
    }
  }

  // sort all the histogram peaks list to have the highest peak at the
  // beginning of the list, the smallest at the end.
  nbpeaks = h.sort(peaks);

  vpTRACE("Sorted list of peaks");
  vpTRACE("Nb peaks: %d", nbpeaks);
  if (nbpeaks) {
    peaks.front();
    while (! peaks.outside() ) {
      vpHistogramPeak p;
      p = peaks.value();
      vpTRACE("Peak: gray level: %d value: %d", p.getLevel(), p.getValue());
      peaks.next();
    }
  }

  // Get the two highest histogram peaks. peak1 is the highest
  vpHistogramPeak peak1, peak2;
  nbpeaks = h.getPeaks(distance, peak1, peak2);
  if (nbpeaks != 2) {
    std::cout << "Not a bimodal histogram..." << std::endl;
  }
  else {
    vpTRACE("Bimodal histogram: main peak1: %d-%d second peak2: %d-%d",
	    peak1.getLevel(), peak1.getValue(),
	    peak2.getLevel(), peak2.getValue());
  }

  // Get the valey between the two highest peaks
  vpHistogramValey valey;
  if (h.getValey(peak1, peak2, valey) == false) {
    vpTRACE("No valey found...");
  }
  else {
    vpTRACE("Valey: %d-%d", valey.getLevel(), valey.getValue());
  }


  vpHistogramValey valeyl, valeyr;

  {
    // Search the two valeys around peak1
    unsigned ret = h.getValey(distance, peak1, valeyl, valeyr);
    if (ret == 0x00) {
      vpTRACE("No left and right valey for peak %d-%d...",
	      peak1.getLevel(), peak1.getValue());
    }
    else if (ret == 0x10) {
      vpTRACE("No right valey for peak %d-%d...",
	      peak1.getLevel(), peak1.getValue());
      vpTRACE("Left valey: %d-%d", valeyl.getLevel(), valeyl.getValue());
    }
    else if (ret == 0x01) {
      vpTRACE("No left valey for peak %d-%d...",
	      peak1.getLevel(), peak1.getValue());
      vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
    }
    else if (ret == 0x11) {
      vpTRACE("Left valey: %d-%d",  valeyl.getLevel(), valeyl.getValue());
      vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
    }
  }
  {
    // Search the two valeys around peak2
    unsigned ret = h.getValey(distance, peak2, valeyl, valeyr);
    if (ret == 0x00) {
      vpTRACE("No left and right valey for peak %d-%d...",
	      peak2.getLevel(), peak2.getValue());
    }
    else if (ret == 0x10) {
      vpTRACE("No right valey for peak %d-%d...",
	      peak2.getLevel(), peak2.getValue());
      vpTRACE("Left valey: %d-%d", valeyl.getLevel(), valeyl.getValue());
    }
    else if (ret == 0x01) {
      vpTRACE("No left valey for peak %d-%d...",
	      peak2.getLevel(), peak2.getValue());
      vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
    }
    else if (ret == 0x11) {
      vpTRACE("Left valey: %d-%d",  valeyl.getLevel(), valeyl.getValue());
      vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
    }
  }

  ////////////////////////////////////////////////////////////
  // get the valey between the two highest peaks. Here we don't know
  // which of peakl or peakr is the highest.
  vpHistogramPeak peakl, peakr;
  if (h.getPeaks(distance, peakl, peakr, valey) == false) {
    std::cout << "Not a bimodal histogram..." << std::endl;
  }
  else {
    vpTRACE("Bimodal histogram: valey %d-%d for peakl: %d-%d peakr: %d-%d",
	    valey.getLevel(), valey.getValue(),
	    peakl.getLevel(), peakl.getValue(),
	    peakr.getLevel(), peakr.getValue());
  }

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
