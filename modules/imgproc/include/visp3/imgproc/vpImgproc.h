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
 * Static functions for basic image processing functions.
 */

/*!
  \file vpImgproc.h
  \brief Basic image processing functions.

*/

#ifndef VP_IMGPROC_H
#define VP_IMGPROC_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageMorphology.h>
#include <visp3/imgproc/vpContours.h>

namespace VISP_NAMESPACE_NAME
{

#define USE_OLD_FILL_HOLE 0

/*!
 * Retinex level that allows to specifies distribution
 * of the Gaussian blurring kernel sizes for scale division values > 2.
 */
enum RETINEX_LEVEL
{
  RETINEX_UNIFORM = 0, //!< Tends to treat all image intensities similarly.
  RETINEX_LOW = 1, //!< Enhances dark regions of the image.
  RETINEX_HIGH = 2 //!< Enhances the bright regions of the image
};

/*!
 * Automatic thresholding method.
 */
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

/**
 * \brief Gamma Correction automatic methods.
 */
typedef enum vpGammaMethod
{
  GAMMA_MANUAL = 0, /*!< User-defined constant positive gamma factor.*/
  GAMMA_LOG_BASED = 1, /*!< Scott, J & Pusateri M (2009)"Towards Real-time Hardware
                        Gamma Correction for Dynamic Contrast Enhancement"
                        IEEE Applied Imagery Pattern Recognition Workshop (AIPR 2009)*/
  GAMMA_NONLINEAR_BASED = 2, /*!< Shi, Y et al. (2007), "Reducing Illumination Based On Nonlinear Gamma Correction",
                              International Conference on Image Processing */
  GAMMA_CDF_BASED = 3, /*!< Huang, SC  et al. (2013),"Efficient Contrast Enhancement Using Adaptive
                        Gamma Correction With Weighting Distribution",
                        IEEE Trans. on Image Processing, VOL. 22, NO. 3, MARCH 2013. */
  GAMMA_CLASSIFICATION_BASED = 4, /*!< Rahman, S  et al. (2016),  "An adaptive gamma correction for image
                                   enhancement", EURASIP Journal on Image and Video Processing*/
  GAMMA_SPATIAL_VARIANT_BASED = 5,  /*!< Lee, S et al. (2010), "A Space-Variant Luminance Map based
                                    Color Image Enhancement",
                                    IEEE Trans. on Consumer Electronics, Vol. 56, No. 4, November 2010.*/
  GAMMA_METHOD_COUNT = 6
} vpGammaMethod;

/**
 * \brief Get the list of available vpGammaMethod.
 *
 * \param[in] pref The prefix of the list.
 * \param[in] sep The separator between two elements of the list.
 * \param[in] suf The suffix of the list.
 * \return std::string The list of available items.
 */
VISP_EXPORT std::string vpGammaMethodList(const std::string &pref = "<", const std::string &sep = " , ",
                                            const std::string &suf = ">");

/**
 * \brief Cast a \b vpGammaMethod into a string, to know its name.
 *
 * \param[in] type The type that must be casted into a string.
 * \return std::string The corresponding name.
 */
VISP_EXPORT std::string vpGammaMethodToString(const vpGammaMethod &type);

/**
 * \brief Cast a string into a \b vpGammaMethod.
 *
 * \param[in] name The name of the backend.
 * \return vpGammaMethod The corresponding enumeration value.
 */
VISP_EXPORT vpGammaMethod vpGammaMethodFromString(const std::string &name);

/**
 * \brief How to handle color images when applying Gamma Correction.
 */
typedef enum vpGammaColorHandling
{
  GAMMA_RGB = 0, /*!< Gamma correction is apply to Red, Blue and Green channels individually.*/
  GAMMA_HSV = 1, /*!< The input image is converted into HSV space, Gamma Correction is applied to Value channel and
                 then the image is converted back into RGBa space.*/
  GAMMA_COLOR_HANDLING_COUNT = 2
} vpGammaColorHandling;

/**
 * \brief Get the list of available vpGammaColorHandling.
 *
 * \param[in] pref The prefix of the list.
 * \param[in] sep The separator between two elements of the list.
 * \param[in] suf The suffix of the list.
 * \return std::string The list of available items.
 */
VISP_EXPORT std::string vpGammaColorHandlingList(const std::string &pref = "<", const std::string &sep = " , ",
                                            const std::string &suf = ">");

/**
 * \brief Cast a \b vpGammaColorHandling into a string, to know its name.
 *
 * \param[in] type The type that must be casted into a string.
 * \return std::string The corresponding name.
 */
VISP_EXPORT std::string vpGammaColorHandlingToString(const vpGammaColorHandling &type);

/**
 * \brief Cast a string into a \b vpGammaColorHandling.
 *
 * \param[in] name The name of the backend.
 * \return vpGammaColorHandling The corresponding enumeration value.
 */
VISP_EXPORT vpGammaColorHandling vpGammaColorHandlingFromString(const std::string &name);

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
VISP_EXPORT void adjust(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, double alpha, double beta);

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
VISP_EXPORT void adjust(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I1, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I2, double alpha, double beta);

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
VISP_EXPORT void adjust(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, const double alpha, double beta);

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
VISP_EXPORT void adjust(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2, double alpha, double beta);

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
VISP_EXPORT void clahe(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I1, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I2, int blockRadius = 150,
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
VISP_EXPORT void clahe(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2, int blockRadius = 150, int bins = 256,
                       float slope = 3.0f, bool fast = true);

/*!
 * \ingroup group_imgproc_histogram
 *
 * Adjust the contrast of a grayscale image by performing an histogram
 * equalization. The intensity distribution is redistributed over the full [0 -
 * 255] range such as the cumulative histogram distribution becomes linear.
 *
 * \param I : The grayscale image to apply histogram equalization.
 * \param p_mask : If set, a boolean mask to take into account only the points for which the mask is true.
 */
VISP_EXPORT void equalizeHistogram(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const VISP_NAMESPACE_ADDRESSING vpImage<bool> *p_mask = nullptr);

/*!
 * \ingroup group_imgproc_histogram
 *
 * Adjust the contrast of a grayscale image by performing an histogram
 * equalization. The intensity distribution is redistributed over the full [0 -
 * 255] range such as the cumulative histogram distribution becomes linear.
 *
 * \param I1 : The first grayscale image.
 * \param I2 : The second grayscale image after histogram equalization.
 * \param p_mask : If set, a boolean mask to take into account only the points for which the mask is true.
 */
VISP_EXPORT void equalizeHistogram(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I1, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I2,
                                   const VISP_NAMESPACE_ADDRESSING vpImage<bool> *p_mask = nullptr);

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
VISP_EXPORT void equalizeHistogram(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, bool useHSV = false);

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
VISP_EXPORT void equalizeHistogram(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2, bool useHSV = false);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a grayscale image.
 *
 * \param[inout] I : The grayscale image to apply gamma correction.
 * \param[in] gamma : Gamma value. If equals to -1, use automatic Gamma correction based on a non-linear
 * technique. If equals to -2, use automatic Gamma correction based on a logarithmic technique.
 * If equals to -3, uses automatic Gamma correction based on classification. If equals to -4, uses automatic Gamma
 * correction based on probabilistic.
 * \param[in] method : The method to use: either \b GAMMA_MANUAL if the user wants to use a positive constant \b gamma
 * factor, or one of the automatic method if \b gamma is negative.
 * \param[in] p_mask : If different from nullptr, permits to indicate which points must be taken into account by setting
 * them to true.
 */
VISP_EXPORT void gammaCorrection(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const float &gamma, const vpGammaMethod &method = GAMMA_MANUAL,
                                 const VISP_NAMESPACE_ADDRESSING vpImage<bool> *p_mask = nullptr);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a grayscale image.
 *
 * \param[in] I1 : The first grayscale image.
 * \param[out] I2 : The second grayscale image after gamma correction.
 * \param[in] gamma : Gamma value. If equals to -1, use automatic Gamma correction based on a non-linear
 * technique. If equals to -2, use automatic Gamma correction based on a logarithmic technique.
 * If equals to -3, uses automatic Gamma correction based on classification. If equals to -4, uses automatic Gamma
 * correction based on probabilistic.
 * \param[in] method : The method to use: either \b GAMMA_MANUAL if the user wants to use a positive constant \b gamma
 * factor, or one of the automatic method if \b gamma is negative.
 * \param[in] p_mask : If different from nullptr, permits to indicate which points must be taken into account by setting
 * them to true.
 */
VISP_EXPORT void gammaCorrection(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I1, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I2, const float &gamma,
                                 const vpGammaMethod &method = GAMMA_MANUAL, const VISP_NAMESPACE_ADDRESSING vpImage<bool> *p_mask = nullptr);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a color image.
 *
 * \param[inout] I : The color image to apply gamma correction.
 * \param[in] gamma : Gamma value.
 * \param[in] colorHandling : How to handle the colors of the image.
 * \param[in] method : The method to use: either \b GAMMA_MANUAL if the user wants to use a positive constant \b gamma factor,
 * or one of the automatic method if \b gamma is negative.
 * \param[in] p_mask : If different from nullptr, permits to indicate which points must be taken into account by setting
 * them to true.
 */
VISP_EXPORT void gammaCorrection(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, const float &gamma, const vpGammaColorHandling &colorHandling = GAMMA_RGB,
                                 const vpGammaMethod &method = GAMMA_MANUAL, const VISP_NAMESPACE_ADDRESSING vpImage<bool> *p_mask = nullptr);

/*!
 * \ingroup group_imgproc_gamma
 *
 * Perform a gamma correction on a color image.
 *
 * \param[in] I1 : The first color image.
 * \param[out] I2 : The second color image after gamma correction.
 * \param[in] gamma : Gamma value.
 * \param[in] colorHandling : How to handle the colors of the image.
 * \param[in] method : The method to use: either \b GAMMA_MANUAL if the user wants to use a positive constant \b gamma factor,
 * or one of the automatic method if \b gamma is negative.
 * \param[in] p_mask : If different from nullptr, permits to indicate which points must be taken into account by setting
 * them to true.
 */
VISP_EXPORT void gammaCorrection(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2, const float &gamma,
                                 const vpGammaColorHandling &colorHandling = GAMMA_RGB,
                                 const vpGammaMethod &method = GAMMA_MANUAL, const VISP_NAMESPACE_ADDRESSING vpImage<bool> *p_mask = nullptr);

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
VISP_EXPORT void retinex(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, int scale = 240, int scaleDiv = 3, int level = RETINEX_UNIFORM,
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
VISP_EXPORT void retinex(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2, int scale = 240, int scaleDiv = 3,
                         int level = RETINEX_UNIFORM, double dynamic = 1.2, int kernelSize = -1);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a grayscale image.
 *
 * \param I : The grayscale image to stretch the contrast.
*/
VISP_EXPORT void stretchContrast(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a grayscale image.
 *
 * \param I1 : The first input grayscale image.
 * \param I2 : The second output grayscale image.
 */
VISP_EXPORT void stretchContrast(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I1, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I2);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image.
 *
 * \param I : The color image to stretch the contrast.
 */
VISP_EXPORT void stretchContrast(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image.
 *
 * \param I1 : The first input color image.
 * \param I2 : The second output color image.
 */
VISP_EXPORT void stretchContrast(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image in the HSV color space.
 * The saturation and value components are stretch so the hue is preserved.
 *
 * \param I : The color image to stretch the contrast in the HSV color space.
 */
VISP_EXPORT void stretchContrastHSV(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I);

/*!
 * \ingroup group_imgproc_contrast
 *
 * Stretch the contrast of a color image in the HSV color space.
 * The saturation and value components are stretch so the hue is preserved.
 *
 * \param I1 : The first input color image.
 * \param I2 : The second output color image.
 */
VISP_EXPORT void stretchContrastHSV(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I1, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I2);

/*!
 * \ingroup group_imgproc_sharpening
 *
 * Sharpen a grayscale image using the unsharp mask technique.
 *
 * \param I : The grayscale image to sharpen.
 * \param sigma : Standard deviation for Gaussian kernel.
 * \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
VISP_EXPORT void unsharpMask(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, float sigma, double weight = 0.6);

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
VISP_EXPORT void unsharpMask(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &Ires, float sigma,
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
VISP_EXPORT void unsharpMask(VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, float sigma, double weight = 0.6);

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
VISP_EXPORT void unsharpMask(const VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &I, VISP_NAMESPACE_ADDRESSING vpImage<VISP_NAMESPACE_ADDRESSING vpRGBa> &Ires, float sigma, double weight = 0.6);

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
VISP_EXPORT void connectedComponents(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, VISP_NAMESPACE_ADDRESSING vpImage<int> &labels, int &nbComponents,
                                     const VISP_NAMESPACE_ADDRESSING vpImageMorphology::vpConnexityType &connexity = VISP_NAMESPACE_ADDRESSING vpImageMorphology::CONNEXITY_4);

/*!
 * \ingroup group_imgproc_morph
 *
 * Fill the holes in a binary image.
 *
 * \param I : Input binary image (0 means background, 255 means foreground).
 */
VISP_EXPORT void fillHoles(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I
#if USE_OLD_FILL_HOLE
                           ,
                           const VISP_NAMESPACE_ADDRESSING vpImageMorphology::vpConnexityType &connexity = VISP_NAMESPACE_ADDRESSING vpImageMorphology::CONNEXITY_4
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
VISP_EXPORT void floodFill(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const VISP_NAMESPACE_ADDRESSING vpImagePoint &seedPoint, const unsigned char oldValue,
                           const unsigned char newValue,
                           const VISP_NAMESPACE_ADDRESSING vpImageMorphology::vpConnexityType &connexity = VISP_NAMESPACE_ADDRESSING vpImageMorphology::CONNEXITY_4);

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
VISP_EXPORT void reconstruct(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &marker, const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &mask,
                             VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &h_kp1 /*alias I */,
                             const VISP_NAMESPACE_ADDRESSING vpImageMorphology::vpConnexityType &connexity = VISP_NAMESPACE_ADDRESSING vpImageMorphology::CONNEXITY_4);

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
VISP_EXPORT unsigned char autoThreshold(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const vpAutoThresholdMethod &method,
                                        const unsigned char backgroundValue = 0,
                                        const unsigned char foregroundValue = 255);

} // namespace

#endif
