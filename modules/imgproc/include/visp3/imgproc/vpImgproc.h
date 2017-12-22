/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Static functions for basic image processing functions.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpImgproc.h
  \brief Basic image processing functions.

*/

#ifndef __vpImgproc_h__
#define __vpImgproc_h__

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageMorphology.h>
#include <visp3/imgproc/vpContours.h>

#define USE_OLD_FILL_HOLE 0

namespace vp
{
enum RETINEX_LEVEL { RETINEX_UNIFORM = 0, RETINEX_LOW = 1, RETINEX_HIGH = 2 };

typedef enum {
  AUTO_THRESHOLD_HUANG,      /*!< Huang L.-K. and Wang M.-J.J. (1995) "Image
                                Thresholding by Minimizing the Measures of
                                Fuzziness" Pattern Recognition, 28(1): 41-51
                                \cite Huang_imagethresholding */
  AUTO_THRESHOLD_INTERMODES, /*!< Prewitt, JMS & Mendelsohn, ML (1966), "The
                                analysis of cell images", Annals of the New
                                York Academy of Sciences 128: 1035-1053
                                \cite NYAS:NYAS1035 */
  AUTO_THRESHOLD_ISODATA,    /*!< Ridler, TW & Calvard, S (1978), "Picture
                                thresholding using an iterative selection
                                method", IEEE Transactions on Systems, Man and
                                Cybernetics 8: 630-632 \cite article4310039, */
  AUTO_THRESHOLD_MEAN,       /*!< Glasbey, CA (1993), "An analysis of
                                histogram-based thresholding algorithms", CVGIP:
                                Graphical Models and Image Processing 55: 532-537
                                \cite Glasbey:1993:AHT:167725.167747 */
  AUTO_THRESHOLD_OTSU,       /*!< Otsu, N (1979), "A threshold selection method from
                                gray-level histograms", IEEE Trans. Sys., Man.,
                                Cyber. 9: 62-66, doi:10.1109/TSMC.1979.4310076
                                 \cite article4310076 */
  AUTO_THRESHOLD_TRIANGLE    /*!< Zack GW, Rogers WE, Latt SA (1977), "Automatic
                                measurement of sister chromatid exchange
                                frequency", J. Histochem. Cytochem. 25 (7):
                                741–53, PMID 70454 \cite doi:10.1177/25.7.70454
                              */
} vpAutoThresholdMethod;

VISP_EXPORT void adjust(vpImage<unsigned char> &I, const double alpha, const double beta);
VISP_EXPORT void adjust(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const double alpha,
                        const double beta);
VISP_EXPORT void adjust(vpImage<vpRGBa> &I, const double alpha, const double beta);
VISP_EXPORT void adjust(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const double alpha, const double beta);

VISP_EXPORT void clahe(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const int blockRadius = 150,
                       const int bins = 256, const float slope = 3.0f, const bool fast = true);
VISP_EXPORT void clahe(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const int blockRadius = 150,
                       const int bins = 256, const float slope = 3.0f, const bool fast = true);

VISP_EXPORT void equalizeHistogram(vpImage<unsigned char> &I);
VISP_EXPORT void equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2);
VISP_EXPORT void equalizeHistogram(vpImage<vpRGBa> &I, const bool useHSV = false);
VISP_EXPORT void equalizeHistogram(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const bool useHSV = false);

VISP_EXPORT void gammaCorrection(vpImage<unsigned char> &I, const double gamma);
VISP_EXPORT void gammaCorrection(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const double gamma);
VISP_EXPORT void gammaCorrection(vpImage<vpRGBa> &I, const double gamma);
VISP_EXPORT void gammaCorrection(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const double gamma);

VISP_EXPORT void retinex(vpImage<vpRGBa> &I, const int scale = 240, const int scaleDiv = 3,
                         const int level = RETINEX_UNIFORM, const double dynamic = 1.2, const int kernelSize = -1);
VISP_EXPORT void retinex(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const int scale = 240, const int scaleDiv = 3,
                         const int level = RETINEX_UNIFORM, const double dynamic = 1.2, const int kernelSize = -1);

VISP_EXPORT void stretchContrast(vpImage<unsigned char> &I);
VISP_EXPORT void stretchContrast(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2);
VISP_EXPORT void stretchContrast(vpImage<vpRGBa> &I);
VISP_EXPORT void stretchContrast(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2);

VISP_EXPORT void stretchContrastHSV(vpImage<vpRGBa> &I);
VISP_EXPORT void stretchContrastHSV(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2);

VISP_EXPORT void unsharpMask(vpImage<unsigned char> &I, const unsigned int size = 7, const double weight = 0.6);
VISP_EXPORT void unsharpMask(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires, const unsigned int size = 7,
                             const double weight = 0.6);
VISP_EXPORT void unsharpMask(vpImage<vpRGBa> &I, const unsigned int size = 7, const double weight = 0.6);
VISP_EXPORT void unsharpMask(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, const unsigned int size = 7,
                             const double weight = 0.6);

VISP_EXPORT void
connectedComponents(const vpImage<unsigned char> &I, vpImage<int> &labels, int &nbComponents,
                    const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4);

VISP_EXPORT void fillHoles(vpImage<unsigned char> &I
#if USE_OLD_FILL_HOLE
                           ,
                           const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4
#endif
);

VISP_EXPORT void floodFill(vpImage<unsigned char> &I, const vpImagePoint &seedPoint, const unsigned char oldValue,
                           const unsigned char newValue,
                           const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4);

VISP_EXPORT void reconstruct(const vpImage<unsigned char> &marker, const vpImage<unsigned char> &mask,
                             vpImage<unsigned char> &I,
                             const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4);

VISP_EXPORT unsigned char autoThreshold(vpImage<unsigned char> &I, const vp::vpAutoThresholdMethod &method,
                                        const unsigned char backgroundValue = 0,
                                        const unsigned char foregroundValue = 255);
}

#endif
