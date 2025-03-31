#include <iostream>
#include <limits>
#include <type_traits>

#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageFilter.h>

#include "hsvUtils.h"

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace vpOldImageFilter
{
/**
   * \brief Resize the image \b I to the desired size and, if \b p_mask is different from nullptr, initialize
   * \b I with 0s.
   *
   * @tparam ImageType Any numerical type (int, float, ...)
   * @param p_mask If different from nullptr, a boolean mask that tells which pixels must be computed.
   * @param height The desired height.
   * @param width The desired width.
   * @param I The image that must be resized and potentially initialized.
   */
template<typename ImageType>
static void resizeAndInitializeIfNeeded(const vpImage<bool> *p_mask, const unsigned int height, const unsigned int width, vpImage<ImageType> &I)
{
  if (p_mask == nullptr) {
    // Just need to resize the output image, values will be computed and overwrite what is inside the image
    I.resize;
  }
  else {
    // Need to reset the image because some points will not be computed
    I.resize(height, width, static_cast<ImageType>(0));
  }
}

/**
 * \brief Indicates if the boolean mask is true at the desired coordinates.
 *
 * \param[in] p_mask Pointer towards the boolean mask if any or nullptr.
 * \param[in] r The row index in the boolean mask.
 * \param[in] c The column index in the boolean mask.
 * \return true If the boolean mask is true at the desired coordinates or if \b p_mask is equal to \b nullptr.
 * \return false False otherwise.
 */
static bool checkBooleanMask(const vpImage<bool> *p_mask, const unsigned int &r, const unsigned int &c)
{
  bool computeVal = true;
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  if (p_mask != nullptr)
#else
  if (p_mask != NULL)
#endif
  {
    computeVal = (*p_mask)[r][c];
  }
  return computeVal;
}

double filterXR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;
  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][c - i].R);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double filterXG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][c - i].G);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double filterXB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][c - i].B);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double filterXLeftBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][c - i].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][i - c].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double filterXLeftBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][c - i].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][i - c].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double filterXLeftBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][c - i].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][i - c].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double filterXRightBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].R + I[r][c - i].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][((val_2 * width) - c) - i - 1].R + I[r][c - i].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double filterXRightBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].G + I[r][c - i].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][((val_2 * width) - c) - i - 1].G + I[r][c - i].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double filterXRightBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].B + I[r][c - i].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][(val_2 * width) - c - i - 1].B + I[r][c - i].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double filterYR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].R + I[r - i][c].R);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double filterYG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].G + I[r - i][c].G);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double filterYB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].B + I[r - i][c].B);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double filterYTopBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].R + I[r - i][c].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].R + I[i - r][c].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double filterYTopBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].G + I[r - i][c].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].G + I[i - r][c].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double filterYTopBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].B + I[r - i][c].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].B + I[i - r][c].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

double filterYBottomBorderR(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].R + I[r - i][c].R);
    }
    else {
      result += filter[i] * static_cast<double>(I[((val_2 * height) - r) - i - 1][c].R + I[r - i][c].R);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].R));
}

double filterYBottomBorderG(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].G + I[r - i][c].G);
    }
    else {
      result += filter[i] * static_cast<double>(I[((val_2 * height) - r) - i - 1][c].G + I[r - i][c].G);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].G));
}

double filterYBottomBorderB(const vpImage<vpRGBa> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].B + I[r - i][c].B);
    }
    else {
      result += filter[i] * static_cast<double>(I[((val_2 * height) - r) - i - 1][c].B + I[r - i][c].B);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].B));
}

void filterX(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr);

template<typename ImageType, typename FilterType>
inline FilterType filterX(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template <typename ImageType, typename FilterType>
inline FilterType filterXLeftBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                           const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][i - c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template <typename ImageType, typename FilterType>
inline FilterType filterXRightBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                            const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  FilterType result = static_cast<FilterType>(0.);
  const unsigned int twice = 2;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<FilterType>(I[r][c + i] + I[r][c - i]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r][((twice * width) - c) - i - 1] + I[r][c - i]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template <typename ImageType, typename FilterType>
void filterX(const vpImage<ImageType> &I, vpImage<FilterType> &dIx, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight();
  const unsigned int width = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = width - ((size - 1) / 2);
  resizeAndInitializeIfNeeded(p_mask, height, width, dIx);

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = filterXLeftBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = filterX<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
    for (unsigned int j = stop2J; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j] = filterXRightBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
}

void filterY(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &dIx, const double *filter, unsigned int size, const vpImage<bool> *p_mask = nullptr);

template<typename ImageType, typename FilterType>
inline FilterType filterY(const vpImage<ImageType> &I, unsigned int r, unsigned int c, const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
inline FilterType filterYTopBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                          const FilterType *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[i - r][c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
inline FilterType filterYBottomBorder(const vpImage<ImageType> &I, unsigned int r, unsigned int c,
                                             const FilterType *filter, unsigned int size)
{
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  FilterType result = static_cast<FilterType>(0.);
  const unsigned int twiceHeight = 2 * height;
  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<FilterType>(I[r + i][c] + I[r - i][c]);
    }
    else {
      result += filter[i] * static_cast<FilterType>(I[(twiceHeight - r) - i - 1][c] + I[r - i][c]);
    }
  }
  return result + (filter[0] * static_cast<FilterType>(I[r][c]));
}

template<typename ImageType, typename FilterType>
void filterY(const vpImage<ImageType> &I, vpImage<FilterType> &dIy, const FilterType *filter, unsigned int size,
                    const vpImage<bool> *p_mask = nullptr)
{
  const unsigned int height = I.getHeight(), width = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = height - ((size - 1) / 2);
  resizeAndInitializeIfNeeded(p_mask, height, width, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = filterYTopBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = filterY<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
  for (unsigned int i = stop2I; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      // We have to compute the value for each pixel if we don't have a mask or for
      // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j] = filterYBottomBorder<ImageType, FilterType>(I, i, j, filter, size);
      }
    }
  }
}

void gaussianBlur(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &GI, unsigned int size = 7, double sigma = 0., bool normalize = true,
  const vpImage<bool> *p_mask = nullptr)
{
  if (size-1 > I.getWidth() || size-1 > I.getHeight()) {
    std::ostringstream oss;
    oss << "Image size (" << I.getWidth() << "x" << I.getHeight() << ") is too small for the Gaussian kernel ("
      << "size=" << size << "), min size is " << (size-1);
    throw vpException(vpException::dimensionError, oss.str());
  }

  double *fg = new double[(size + 1) / 2];
  vpImageFilter::getGaussianKernel(fg, size, sigma, normalize);
  vpImage<vpRGBa> GIx;
  vpImageFilter::filterX(I, GIx, fg, size, p_mask);
  vpImageFilter::filterY(GIx, GI, fg, size, p_mask);
  GIx.destroy();
  delete[] fg;
}

template<typename HSVType, bool useFullScale>
double filterXH(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;
  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].H + I[r][c - i].H);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].H));
}

template<typename HSVType, bool useFullScale>
double filterXS(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].S + I[r][c - i].S);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].S));
}

template<typename HSVType, bool useFullScale>
double filterXV(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r][c + i].V + I[r][c - i].V);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].V));
}

template<typename HSVType, bool useFullScale>
double filterXLeftBorderH(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].H + I[r][c - i].H);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].H + I[r][i - c].H);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].H));
}

template<typename HSVType, bool useFullScale>
double filterXLeftBorderS(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].S + I[r][c - i].S);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].S + I[r][i - c].S);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].S));
}

template<typename HSVType, bool useFullScale>
double filterXLeftBorderV(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                         const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (c > i) {
      result += filter[i] * static_cast<double>(I[r][c + i].V + I[r][c - i].V);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][c + i].V + I[r][i - c].V);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].V));
}

template<typename HSVType, bool useFullScale>
double filterXRightBorderH(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].H + I[r][c - i].H);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][((val_2 * width) - c) - i - 1].H + I[r][c - i].H);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].H));
}

template<typename HSVType, bool useFullScale>
double filterXRightBorderS(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].S + I[r][c - i].S);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][((val_2 * width) - c) - i - 1].S + I[r][c - i].S);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].S));
}

template<typename HSVType, bool useFullScale>
double filterXRightBorderV(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                        const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int stop = (size - 1) / 2;
  const unsigned int width = I.getWidth();
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((c + i) < width) {
      result += filter[i] * static_cast<double>(I[r][c + i].V + I[r][c - i].V);
    }
    else {
      result += filter[i] * static_cast<double>(I[r][(val_2 * width) - c - i - 1].V + I[r][c - i].V);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].V));
}

template<typename HSVType, bool useFullScale>
void filterX(const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<vpHSV<HSVType, useFullScale>> &dIx, const double *filter, unsigned int size,
  const vpImage<bool> *p_mask)
{
  const unsigned int heightI = I.getHeight(), widthI = I.getWidth();
  const unsigned int stop1J = (size - 1) / 2;
  const unsigned int stop2J = widthI - ((size - 1) / 2);
  resizeAndInitializeIfNeeded(p_mask, heightI, widthI, dIx);

  for (unsigned int i = 0; i < heightI; ++i) {
    for (unsigned int j = 0; j < stop1J; ++j) {
    // We have to compute the value for each pixel if we don't have a mask or for
    // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j].H = static_cast<HSVType>(filterXLeftBorderH(I, i, j, filter, size));
        dIx[i][j].S = static_cast<HSVType>(filterXLeftBorderS(I, i, j, filter, size));
        dIx[i][j].V = static_cast<HSVType>(filterXLeftBorderV(I, i, j, filter, size));
      }
    }
    for (unsigned int j = stop1J; j < stop2J; ++j) {
    // We have to compute the value for each pixel if we don't have a mask or for
    // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j].H = static_cast<HSVType>(filterXH(I, i, j, filter, size));
        dIx[i][j].S = static_cast<HSVType>(filterXS(I, i, j, filter, size));
        dIx[i][j].V = static_cast<HSVType>(filterXV(I, i, j, filter, size));
      }
    }
    for (unsigned int j = stop2J; j < widthI; ++j) {
    // We have to compute the value for each pixel if we don't have a mask or for
    // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIx[i][j].H = static_cast<HSVType>(filterXRightBorderH(I, i, j, filter, size));
        dIx[i][j].S = static_cast<HSVType>(filterXRightBorderS(I, i, j, filter, size));
        dIx[i][j].V = static_cast<HSVType>(filterXRightBorderV(I, i, j, filter, size));
      }
    }
  }
}

// ------------- Filtering along Y for HSV images ----------------

template<typename HSVType, bool useFullScale>
double filterYH(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].H + I[r - i][c].H);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].H));
}

template<typename HSVType, bool useFullScale>
double filterYS(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].S + I[r - i][c].S);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].S));
}

template<typename HSVType, bool useFullScale>
double filterYV(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    result += filter[i] * static_cast<double>(I[r + i][c].V + I[r - i][c].V);
  }
  return result + (filter[0] * static_cast<double>(I[r][c].V));
}

template<typename HSVType, bool useFullScale>
double filterYTopBorderH(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].H + I[r - i][c].H);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].H + I[i - r][c].H);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].H));
}

template<typename HSVType, bool useFullScale>
double filterYTopBorderS(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].S + I[r - i][c].S);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].S + I[i - r][c].S);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].S));
}

template<typename HSVType, bool useFullScale>
double filterYTopBorderV(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c, const double *filter, unsigned int size)
{
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if (r > i) {
      result += filter[i] * static_cast<double>(I[r + i][c].V + I[r - i][c].V);
    }
    else {
      result += filter[i] * static_cast<double>(I[r + i][c].V + I[i - r][c].V);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].V));
}

template<typename HSVType, bool useFullScale>
double filterYBottomBorderH(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].H + I[r - i][c].H);
    }
    else {
      result += filter[i] * static_cast<double>(I[((val_2 * height) - r) - i - 1][c].H + I[r - i][c].H);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].H));
}

template<typename HSVType, bool useFullScale>
double filterYBottomBorderS(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].S + I[r - i][c].S);
    }
    else {
      result += filter[i] * static_cast<double>(I[((val_2 * height) - r) - i - 1][c].S + I[r - i][c].S);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].S));
}

template<typename HSVType, bool useFullScale>
double filterYBottomBorderV(const vpImage<vpHSV<HSVType, useFullScale>> &I, unsigned int r, unsigned int c,
                                          const double *filter, unsigned int size)
{
  const unsigned int val_2 = 2;
  const unsigned int height = I.getHeight();
  const unsigned int stop = (size - 1) / 2;
  double result = 0.;

  for (unsigned int i = 1; i <= stop; ++i) {
    if ((r + i) < height) {
      result += filter[i] * static_cast<double>(I[r + i][c].V + I[r - i][c].V);
    }
    else {
      result += filter[i] * static_cast<double>(I[((val_2 * height) - r) - i - 1][c].V + I[r - i][c].V);
    }
  }
  return result + (filter[0] * static_cast<double>(I[r][c].V));
}

template<typename HSVType, bool useFullScale>
void filterY(const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<vpHSV<HSVType, useFullScale>> &dIy, const double *filter, unsigned int size,
  const vpImage<bool> *p_mask)
{
  const unsigned int heightI = I.getHeight(), widthI = I.getWidth();
  const unsigned int stop1I = (size - 1) / 2;
  const unsigned int stop2I = heightI - ((size - 1) / 2);
  resizeAndInitializeIfNeeded(p_mask, heightI, widthI, dIy);

  for (unsigned int i = 0; i < stop1I; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
    // We have to compute the value for each pixel if we don't have a mask or for
    // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j].H = static_cast<HSVType>(filterYTopBorderH(I, i, j, filter, size));
        dIy[i][j].S = static_cast<HSVType>(filterYTopBorderS(I, i, j, filter, size));
        dIy[i][j].V = static_cast<HSVType>(filterYTopBorderV(I, i, j, filter, size));
      }
    }
  }
  for (unsigned int i = stop1I; i < stop2I; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
    // We have to compute the value for each pixel if we don't have a mask or for
    // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j].H = static_cast<HSVType>(filterYH(I, i, j, filter, size));
        dIy[i][j].S = static_cast<HSVType>(filterYS(I, i, j, filter, size));
        dIy[i][j].V = static_cast<HSVType>(filterYV(I, i, j, filter, size));
      }
    }
  }
  for (unsigned int i = stop2I; i < heightI; ++i) {
    for (unsigned int j = 0; j < widthI; ++j) {
    // We have to compute the value for each pixel if we don't have a mask or for
    // pixels for which the mask is true otherwise
      bool computeVal = checkBooleanMask(p_mask, i, j);
      if (computeVal) {
        dIy[i][j].H = static_cast<HSVType>(filterYBottomBorderH(I, i, j, filter, size));
        dIy[i][j].S = static_cast<HSVType>(filterYBottomBorderS(I, i, j, filter, size));
        dIy[i][j].V = static_cast<HSVType>(filterYBottomBorderV(I, i, j, filter, size));
      }
    }
  }
}

/*!
   Apply a Gaussian blur to RGB color image.
   \param[in] I : Input image.
   \param[out] GI : Filtered image.
   \param[in] size : Filter size. This value should be odd.
   \param[in] sigma : Gaussian standard deviation. If it is equal to zero or
   negative, it is computed from filter size as sigma = (size-1)/6.
   \param[in] normalize : Flag indicating whether to normalize the filter coefficients or not.
   \param[in] p_mask : If different from nullptr, mask indicating which points to consider (true) or to ignore(false).

   \sa getGaussianKernel() to know which kernel is used.
  */
template<typename HSVType, bool useFullScale>
void gaussianBlur(const vpImage<vpHSV<HSVType, useFullScale>> &I, vpImage<vpHSV<HSVType, useFullScale>> &GI, unsigned int size = 7, double sigma = 0., bool normalize = true,
  const vpImage<bool> *p_mask = nullptr)
{
  double *fg = new double[(size + 1) / 2];
  vpImageFilter::getGaussianKernel(fg, size, sigma, normalize);
  vpImage<vpHSV<HSVType, useFullScale>> GIx;
  vpImageFilter::filterX(I, GIx, fg, size, p_mask);
  vpImageFilter::filterY(GIx, GI, fg, size, p_mask);
  GIx.destroy();
  delete[] fg;
}

template <typename ImageType, typename FilterType>
static void gaussianBlur(const vpImage<ImageType> &I, vpImage<FilterType> &GI, unsigned int size = 7, FilterType sigma = 0., bool normalize = true,
                         const vpImage<bool> *p_mask = nullptr)
{
  FilterType *fg = new FilterType[(size + 1) / 2];
  vpImageFilter::getGaussianKernel<FilterType>(fg, size, sigma, normalize);
  vpImage<FilterType> GIx;
  vpImageFilter::filterX<ImageType, FilterType>(I, GIx, fg, size, p_mask);
  vpImageFilter::filterY<FilterType, FilterType>(GIx, GI, fg, size, p_mask);
  GIx.destroy();
  delete[] fg;
}
}

template<typename ArithmeticType, bool useFullScale>
void print(const vpImage<vpHSV<ArithmeticType, useFullScale>> &I, const std::string &name)
{
  std::cout << name << " = " << std::endl;
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      std::string character;
      if (vpMath::nul(I[r][c].H, 1e-3)) {
        character = '0';
      }
      else {
        double val = static_cast<double>(I[r][c].H);
        if (val > 0 && val < 1.) {
          val *= 10.;
        }
        character = std::to_string(static_cast<unsigned int>(val));
      }
      std::cout << character << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

template<typename ArithmeticType>
void print(const vpImage<ArithmeticType> &I, const std::string &name)
{
  std::cout << name << " = " << std::endl;
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      char character;
      if (vpMath::nul(I[r][c], 1e-3)) {
        character = '0';
      }
      else if (I[r][c] > 0) {
        character = '+';
      }
      else {
        character = '-';
      }
      std::cout << character << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl << std::flush;
}

void print(const vpImage<unsigned char> &I, const std::string &name)
{
  std::cout << name << " = " << std::endl;
  const unsigned int nbRows = I.getRows(), nbCols = I.getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      std::cout << std::to_string(I[r][c]) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl << std::flush;
}
#endif

int main()
{
  bool isSuccess = true;

  vpRGBa rgba;
  vpHSV<unsigned char> hsv_uc;
  vpHSV<double> hsv_double;

  // Inputs
  vpHSVTests::vpInputDataset dataset;

  // Outputs
  vpImage<double> Iuc_filtered_old, Iuc_filtered_new;
  vpImage<vpRGBa> Irgba(11, 11, vpRGBa(125U, 125U, 125U)), Irgba_filtered_old, Irgba_filtered_new;
  vpImage<vpHSV<unsigned char, true>> Ihsv_uc_filtered_old_true, Ihsv_uc_filtered_new_true;
  vpImage<vpHSV<unsigned char, false>> Ihsv_uc_filtered_old_false, Ihsv_uc_filtered_new_false;
  vpImage<vpHSV<double>> Ihsv_double_filtered_old, Ihsv_double_filtered_new;

  for (unsigned int size = 3; (size < 7) && isSuccess; size += 2) {
    for (auto input: dataset.m_ucImages) {
      print(input.second.m_I, input.first);
      vpOldImageFilter::gaussianBlur(input.second.m_I, Iuc_filtered_old, size);
      vpImageFilter::gaussianBlur(input.second.m_I, Iuc_filtered_new, size, 0.);
      bool ucSuccess = vpHSVTests::areAlmostEqual(Iuc_filtered_old, Iuc_filtered_new);
      isSuccess = isSuccess && ucSuccess;
      if (!isSuccess) {
        std::cerr << "ERROR: filter on uchar failed ! " << std::endl;
      }
    }

    vpOldImageFilter::gaussianBlur(Irgba, Irgba_filtered_old, size);
    vpImageFilter::gaussianBlur(Irgba, Irgba_filtered_new, size, 0.);
    bool rgbaSuccess = vpHSVTests::areAlmostEqual(Irgba_filtered_old, Irgba_filtered_new);
    isSuccess = isSuccess && rgbaSuccess;
    if (!rgbaSuccess) {
      std::cerr << "ERROR: filter on RGBa failed ! " << std::endl;
    }

    for (auto input: dataset.m_hsvUCtrue) {
      print(input.second.m_I, input.first);
      vpOldImageFilter::gaussianBlur(input.second.m_I, Ihsv_uc_filtered_old_true, size);
      vpImageFilter::gaussianBlur(input.second.m_I, Ihsv_uc_filtered_new_true, size, 0.);
      bool hsvucSuccessTrue = vpHSVTests::areAlmostEqual(Ihsv_uc_filtered_old_true, Ihsv_uc_filtered_new_true);
      isSuccess = isSuccess && hsvucSuccessTrue;
      if (!hsvucSuccessTrue) {
        std::cerr << "ERROR: filter on HSV<uchar, true> failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvUCfalse) {
      print(input.second.m_I, input.first);
      vpOldImageFilter::gaussianBlur(input.second.m_I, Ihsv_uc_filtered_old_false, size);
      vpImageFilter::gaussianBlur(input.second.m_I, Ihsv_uc_filtered_new_false, size, 0.);
      bool hsvucSuccessTrue = vpHSVTests::areAlmostEqual(Ihsv_uc_filtered_old_false, Ihsv_uc_filtered_new_false);
      isSuccess = isSuccess && hsvucSuccessTrue;
      if (!hsvucSuccessTrue) {
        std::cerr << "ERROR: filter on HSV<uchar, false> failed ! " << std::endl;
      }
    }

    for (auto input: dataset.m_hsvDouble) {
      print(input.second.m_I, input.first);
      vpOldImageFilter::gaussianBlur(input.second.m_I, Ihsv_double_filtered_old, size);
      vpImageFilter::gaussianBlur(input.second.m_I, Ihsv_double_filtered_new, size, 0.);
      bool hsvucSuccessTrue = vpHSVTests::areAlmostEqual(Ihsv_double_filtered_old, Ihsv_double_filtered_new);
      isSuccess = isSuccess && hsvucSuccessTrue;
      if (!hsvucSuccessTrue) {
        std::cerr << "ERROR: filter on HSV<double> failed ! " << std::endl;
      }
    }
  }

  if (isSuccess) {
    std::cout << "All tests were successful !" << std::endl;
    return EXIT_SUCCESS;
  }
  std::cerr << "ERROR: Something went wrong !" << std::endl;
  return EXIT_FAILURE;
}
#else
int main()
{
  std::cout << "vpHSV class is not available, please use CXX 11 standard" << std::endl;
  return EXIT_SUCCESS;
}
#endif
