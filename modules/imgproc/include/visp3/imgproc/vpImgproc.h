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
 * Static functions for basic image processing functions.
 */

/*!
  \file vpImgproc.h
  \brief Basic image processing functions.

*/

#ifndef _vpImgproc_h_
#define _vpImgproc_h_

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageMorphology.h>
#include <visp3/imgproc/vpContours.h>

#define USE_OLD_FILL_HOLE 0

namespace vp
{
enum RETINEX_LEVEL { RETINEX_UNIFORM = 0, RETINEX_LOW = 1, RETINEX_HIGH = 2 };

typedef enum
{
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

/*!
 * \ingroup group_imgproc_brightness
 *
 * Adjust the brightness of a grayscale image such as the new intensity is
 * alpha x old_intensity + beta.
 *
 * \param I : The grayscale image to adjust the brightness.
 * \param alpha : Multiplication coefficient.
 * \param beta : Constant value added to the old intensity.
 */
VISP_EXPORT void adjust(vpImage<unsigned char> &I, double alpha, double beta);

/*!
 * \ingroup group_imgproc_brightness
 *
 * Adjust the brightness of a grayscale image such as the new intensity is
 * alpha x old_intensity + beta.
 *
 * \param I1 : The original grayscale image.
 * \param I2 : The grayscale image after adjusting pixel intensities.
 * \param alpha : Multiplication coefficient.
 * \param beta : Constant value added to the old intensity.
 */
VISP_EXPORT void adjust(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, double alpha, double beta);

/*!
 * \ingroup group_imgproc_brightness
 *
 * Adjust the brightness of a color image such as the new intensity is alpha x
 * old_intensity + beta.
 *
 * \param I : The color image to adjust the brightness.
 * \param alpha : Multiplication coefficient.
 * \param beta : Constant value added to the old intensity.
 */
VISP_EXPORT void adjust(vpImage<vpRGBa> &I, const double alpha, double beta);

/*!
 * \ingroup group_imgproc_brightness
 *
 * Adjust the brightness of a color image such as the new intensity is alpha x
 * old_intensity + beta.
 *
 * \param I1 : The original color image.
 * \param I2 : The color image after adjusting pixel intensities.
 * \param alpha : Multiplication coefficient.
 * \param beta : Constant value added to the old intensity.
 */
VISP_EXPORT void adjust(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, double alpha, double beta);

/*!
 * \ingroup group_imgproc_brightness
 *
 * Adjust the contrast of a grayscale image locally using the Contrast Limited
 * Adaptive Histogram Equalization method. The limit parameter allows to
 * limit the slope of the transformation function to prevent the
 * over amplification of noise. This method is a transcription of the CLAHE
 * ImageJ plugin code by Stephan Saalfeld.
 *
 * \param I1 : The first grayscale image.
 * \param I2 : The second grayscale image after application of the CLAHE
 * method.
 * \param blockRadius : The size (2*blockRadius+1) of the local region
 * around a pixel for which the histogram is equalized. This size should be
 * larger than the size of features to be preserved.
 * \param bins : The number
 * of histogram bins used for histogram equalization (between 1 and 256). The
 * number of histogram bins should be smaller than the number of pixels in a
 * block.
 * \param slope : Limits the contrast stretch in the intensity transfer
 * function. Very large values will let the histogram equalization do whatever
 * it wants to do, that is result in maximal local contrast. The value 1 will
 * result in the original image.
 * \param fast : Use the fast but less accurate
 * version of the filter. The fast version does not evaluate the intensity
 * transfer function for each pixel independently but for a grid of adjacent
 * boxes of the given block size only and interpolates for locations in
 * between.
 */
VISP_EXPORT void clahe(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, int blockRadius = 150,
                       int bins = 256, float slope = 3.0f, bool fast = true);

/*!
 * \ingroup group_imgproc_brightness
 *
 * Adjust the contrast of a color image locally using the Contrast Limited
 * Adaptive Histogram Equalization method. The limit parameter allows to
 * limit the slope of the transformation function to prevent the
 * over amplification of noise. This method is a transcription of the CLAHE
 * ImageJ plugin code by Stephan Saalfeld.
 *
 * \param I1 : The first color image.
 * \param I2 : The second color image after application of the CLAHE method.
 * \param blockRadius : The size (2*blockRadius+1) of the local region around a
 * pixel for which the histogram is equalized. This size should be larger than
 * the size of features to be preserved.
 * \param  bins : The number of histogram
 * bins used for histogram equalization (between 1 and 256). The number of
 * histogram bins should be smaller than the number of pixels in a block.
 * \param slope : Limits the contrast stretch in the intensity transfer
 * function. Very large values will let the histogram equalization do whatever
 * it wants to do, that is result in maximal local contrast. The value 1 will
 * result in the original image.
 * \param fast : Use the fast but less accurate
 * version of the filter. The fast version does not evaluate the intensity
 * transfer function for each pixel independently but for a grid of adjacent
 * boxes of the given block size only and interpolates for locations in
 * between.
*/
VISP_EXPORT void clahe(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, int blockRadius = 150, int bins = 256,
                       float slope = 3.0f, bool fast = true);

/*!
 * \ingroup group_imgproc_histogram
 *
 * Adjust the contrast of a grayscale image by performing an histogram
 * equalization. The intensity distribution is redistributed over the full [0 -
 * 255] range such as the cumulative histogram distribution becomes linear.
 *
 * \param I : The grayscale image to apply histogram equalization.
*/
VISP_EXPORT void equalizeHistogram(vpImage<unsigned char> &I);

/*!
 * \ingroup group_imgproc_histogram
 *
 * Adjust the contrast of a grayscale image by performing an histogram
 * equalization. The intensity distribution is redistributed over the full [0 -
 * 255] range such as the cumulative histogram distribution becomes linear.
 *
 * \param I1 : The first grayscale image.
 * \param I2 : The second grayscale image after histogram equalization.
 */
VISP_EXPORT void equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2);

/*!
 * \ingroup group_imgproc_histogram
 *
 * Adjust the contrast of a color image by performing an histogram
 * equalization. The intensity distribution is redistributed over the full [0 -
 * 255] range such as the cumulative histogram distribution becomes linear. The
 * alpha channel is ignored / copied from the source alpha channel.
 *
 * \param I : The color image to apply histogram equalization.
 * \param useHSV : If true, the histogram equalization is performed on the
 * value channel (in HSV space), otherwise the histogram equalization is
 * performed independently on the RGB channels.
 */
VISP_EXPORT void equalizeHistogram(vpImage<vpRGBa> &I, bool useHSV = false);

/*!
 * \ingroup group_imgproc_histogram
 *
 * Adjust the contrast of a color image by performing an histogram
 * equalization. The intensity distribution is redistributed over the full [0 -
 * 255] range such as the cumulative histogram distribution becomes linear. The
 * alpha channel is ignored / copied from the source alpha channel.
 *
 * \param I1 : The first color image.
 * \param I2 : The second color image after histogram equalization.
 * \param useHSV : If true, the histogram equalization is performed on the
 * value channel (in HSV space), otherwise the histogram equalization is
 * performed independently on the RGB channels.
 */
VISP_EXPORT void equalizeHistogram(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, bool useHSV = false);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a grayscale image.
 *
 * \param I : The grayscale image to apply gamma correction.
 * \param gamma : Gamma value.
 */
VISP_EXPORT void gammaCorrection(vpImage<unsigned char> &I, double gamma);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a grayscale image.
 *
 * \param I1 : The first grayscale image.
 * \param I2 : The second grayscale image after gamma correction.
 * \param gamma : Gamma value.
 */
VISP_EXPORT void gammaCorrection(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, double gamma);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a color image.
 *
 * \param I : The color image to apply gamma correction.
 * \param gamma : Gamma value.
 */
VISP_EXPORT void gammaCorrection(vpImage<vpRGBa> &I, double gamma);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a color image.
 *
 * \param I1 : The first color image.
 * \param I2 : The second color image after gamma correction.
 * \param gamma : Gamma value.
 */
VISP_EXPORT void gammaCorrection(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, double gamma);

/*!
 * \ingroup group_imgproc_retinex
 *
 * Apply the Retinex algorithm (the input image is modified).
 * \param I : The color image after application of the Retinex technique.
 * \param scale : Specifies the depth of the retinex effect. Minimum value is
 * 16, a value providing gross, unrefined filtering. Maximum value is 250.
 * Optimal and default value is 240.
 * \param scaleDiv : Specifies the number of
 * iterations of the multi scale filter. Values larger than 2 exploit the
 * "multiscale" nature of the algorithm.
 * \param level : Specifies distribution
 * of the Gaussian blurring kernel sizes for Scale division values > 2:
 *   - 0, tends to treat all image intensities similarly,
 *   - 1, enhances dark regions of the image,
 *   - 2, enhances the bright regions of the image.
 * \param dynamic : Adjusts the color of the result. Large values produce less
 * saturated images.
 * \param kernelSize : Kernel size for the gaussian blur
 * operation. If -1, the kernel size is calculated from the image size.
 */
VISP_EXPORT void retinex(vpImage<vpRGBa> &I, int scale = 240, int scaleDiv = 3, int level = RETINEX_UNIFORM,
                         double dynamic = 1.2, int kernelSize = -1);

/*!
 * \ingroup group_imgproc_retinex
 *
 * Apply the Retinex algorithm.
 * \param I1 : The input color image.
 * \param I2 : The output color image after application of the Retinex technique.
 * \param scale : Specifies the depth of the retinex effect. Minimum
 * value is 16, a value providing gross, unrefined filtering. Maximum value is
 * 250. Optimal and default value is 240.
 * \param scaleDiv : Specifies the
 * number of iterations of the multiscale filter. Values larger than 2 exploit
 * the "multiscale" nature of the algorithm.
 * \param level : Specifies distribution of the Gaussian blurring kernel sizes
 * for Scale division values > 2:
 *   - 0, tends to treat all image intensities similarly,
 *   - 1, enhances dark regions of the image,
 *   - 2, enhances the bright regions of the image.
 * \param dynamic : Adjusts the color of the result. Large values produce less
 * saturated images.
 * \param kernelSize : Kernel size for the gaussian blur
 * operation. If -1, the kernel size is calculated from the image size.
 */
VISP_EXPORT void retinex(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, int scale = 240, int scaleDiv = 3,
                         int level = RETINEX_UNIFORM, double dynamic = 1.2, int kernelSize = -1);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a grayscale image.
 *
 * \param I : The grayscale image to stretch the contrast.
*/
VISP_EXPORT void stretchContrast(vpImage<unsigned char> &I);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a grayscale image.
 *
 * \param I1 : The first input grayscale image.
 * \param I2 : The second output grayscale image.
 */
VISP_EXPORT void stretchContrast(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image.
 *
 * \param I : The color image to stretch the contrast.
 */
VISP_EXPORT void stretchContrast(vpImage<vpRGBa> &I);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image.
 *
 * \param I1 : The first input color image.
 * \param I2 : The second output color image.
 */
VISP_EXPORT void stretchContrast(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image in the HSV color space.
 * The saturation and value components are stretch so the hue is preserved.
 *
 * \param I : The color image to stretch the contrast in the HSV color space.
 */
VISP_EXPORT void stretchContrastHSV(vpImage<vpRGBa> &I);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image in the HSV color space.
 * The saturation and value components are stretch so the hue is preserved.
 *
 * \param I1 : The first input color image.
 * \param I2 : The second output color image.
 */
VISP_EXPORT void stretchContrastHSV(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  @name Deprecated functions
*/
//@{
vp_deprecated VISP_EXPORT void unsharpMask(vpImage<unsigned char> &I, unsigned int size = 7, double weight = 0.6);
vp_deprecated VISP_EXPORT void unsharpMask(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires,
                                           unsigned int size = 7, double weight = 0.6);
vp_deprecated VISP_EXPORT void unsharpMask(vpImage<vpRGBa> &I, unsigned int size = 7, double weight = 0.6);
vp_deprecated VISP_EXPORT void unsharpMask(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, unsigned int size = 7,
                                           double weight = 0.6);
//@}
#endif

/*!
 * \ingroup group_imgproc_sharpening
 *
 * Sharpen a grayscale image using the unsharp mask technique.
 *
 * \param I : The grayscale image to sharpen.
 * \param sigma : Standard deviation for Gaussian kernel.
 * \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
VISP_EXPORT void unsharpMask(vpImage<unsigned char> &I, float sigma, double weight = 0.6);

/*!
 * \ingroup group_imgproc_sharpening
 *
 * Sharpen a grayscale image using the unsharp mask technique.
 *
 * \param I : The input grayscale image.
 * \param Ires : The output grayscale image.
 * \param sigma : Standard deviation for Gaussian kernel.
 * \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
VISP_EXPORT void unsharpMask(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires, float sigma,
                             double weight = 0.6);

/*!
 * \ingroup group_imgproc_sharpening
 *
 * Sharpen a color image using the unsharp mask technique.
 *
 * \param I : The color image to sharpen.
 * \param sigma : Standard deviation for Gaussian kernel.
 * \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
VISP_EXPORT void unsharpMask(vpImage<vpRGBa> &I, float sigma, double weight = 0.6);

/*!
 * \ingroup group_imgproc_sharpening
 *
 * Sharpen a color image using the unsharp mask technique.
 *
 * \param I : The input color image.
 * \param Ires : The output color image.
 * \param sigma : Standard deviation for Gaussian kernel.
 * \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
VISP_EXPORT void unsharpMask(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, float sigma, double weight = 0.6);

/*!
 * \ingroup group_imgproc_connected_components
 *
 * Perform connected components detection.
 *
 * \param I : Input image (0 means background).
 * \param labels : Label image that contain for each position the component label.
 * \param nbComponents : Number of connected components.
 * \param connexity : Type of connexity.
 */
VISP_EXPORT void connectedComponents(const vpImage<unsigned char> &I, vpImage<int> &labels, int &nbComponents,
                                     const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4);

/*!
 * \ingroup group_imgproc_morph
 *
 * Fill the holes in a binary image.
 *
 * \param I : Input binary image (0 means background, 255 means foreground).
 */
VISP_EXPORT void fillHoles(vpImage<unsigned char> &I
#if USE_OLD_FILL_HOLE
                           ,
                           const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4
#endif
);

/*!
 * \ingroup group_imgproc_connected_components
 *
 * Perform the flood fill algorithm.
 *
 * \param I : Input image to flood fill.
 * \param seedPoint : Seed position in the image.
 * \param oldValue : Old value to replace.
 * \param newValue : New value to flood fill.
 * \param connexity : Type of connexity.
 */
VISP_EXPORT void floodFill(vpImage<unsigned char> &I, const vpImagePoint &seedPoint, const unsigned char oldValue,
                           const unsigned char newValue,
                           const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4);

/*!
 * \ingroup group_imgproc_morph
 *
 * Perform morphological reconstruction of the image \a marker under the image
 * \a mask. Definition from Gleb V. Tcheslavsk: > The morphological
 * reconstruction by dilation of a grayscale image \f$ g \f$ by a grayscale
 * marker image \f$ f \f$ > is defined as the geodesic dilation of \f$ f \f$
 * with respect to \f$ g \f$ repeated (iterated) until stability is reached:
 * \f[
 *   R_{g}^{D} \left ( f \right ) = D_{g}^{\left ( k \right )} \left ( f \right
 * ) \f] with \f$ k \f$ such that: \f$ D_{g}^{\left ( k \right )} \left ( f
 * \right ) = D_{g}^{\left ( k+1 \right )} \left ( f \right ) \f$
 *
 * \param marker : Grayscale image marker.
 * \param mask : Grayscale image mask.
 * \param h_kp1 : Image morphologically reconstructed.
 * \param connexity : Type of connexity.
 */
VISP_EXPORT void reconstruct(const vpImage<unsigned char> &marker, const vpImage<unsigned char> &mask,
                             vpImage<unsigned char> &h_kp1 /*alias I */,
                             const vpImageMorphology::vpConnexityType &connexity = vpImageMorphology::CONNEXITY_4);

/*!
 * \ingroup group_imgproc_threshold
 *
 * Automatic thresholding.
 *
 * \param I : Input grayscale image.
 * \param method : Automatic thresholding method.
 * \param backgroundValue : Value to set to the background.
 * \param foregroundValue : Value to set to the foreground.
 */
VISP_EXPORT unsigned char autoThreshold(vpImage<unsigned char> &I, const vp::vpAutoThresholdMethod &method,
                                        const unsigned char backgroundValue = 0,
                                        const unsigned char foregroundValue = 255);
} // namespace vp

#endif
