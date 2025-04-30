#ifndef HSV_UTILS_H
#define HSV_UTILS_H
#include <iostream>
#include <map>
#include <type_traits>

#include <visp3/core/vpColorGetter.h>
#include <visp3/core/vpImage.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
namespace vpHSVTests
{
/**
 * \brief Compute the error for an arithmetic type (uchar, float, double, ...).
 *
 * \tparam T An arithmetic type.
 * \param[in] a The first value.
 * \param[in] b The second value.
 * \return The absolute difference.
 */
template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value, double>::type
error(const T &a, const T &b)
{
  return std::abs(a - b);
}

/**
 * \brief Initialize the error to 0.
 *
 * \tparam Tp The type, that must have a nbChannels constexpr.
 * \tparam I The iterator.
 * \param[in] t1 The first value.
 * \param[in] t2 The second value.
 * \return double 0. to initialize the error.
 */
template<typename Tp, int I = 0>
inline typename std::enable_if<I == Tp::nbChannels, double>::type
error(const Tp &t1, const Tp &t2)
{
  (void)t1;
  (void)t2;
  return 0.;
}

/**
 * \brief Compute the error for a channel and recursively call error for the other channels.
 *
 * \tparam Tp The type, that must have a nbChannels constexpr.
 * \tparam I The iterator.
 * \param[in] t1 The first value.
 * \param[in] t2 The second value.
 * \return double The weighted sum of the absolute error for all the channels.
 */
template<typename Tp, int I = 0>
inline typename std::enable_if<I < Tp::nbChannels, double>::type
  error(const Tp &t1, const Tp &t2)
{
  const double &val1 = vpColorGetter<I>::get(t1);
  const double &val2 = vpColorGetter<I>::get(t2);
  const double den = 1. / Tp::nbChannels;
  double err = den * std::abs(val1 - val2) + error<Tp, I + 1>(t1, t2);
  return err;
}

/**
 * \brief Indicates if 2 vpImage are almost equal.
 *
 * \tparam T The type of the image.
 * \param[in] I1 The first image.
 * \param[in] I2 The second image.
 * \param[in] thresh The maximum tolerated error.
 * \return true The images are almost equal.
 * \return false Otherwise.
 */
template<typename T>
bool areAlmostEqual(const vpImage<T> &I1, const std::string &nameI1, const vpImage<T> &I2, const std::string &nameI2, const double &thresh = 1e-3)
{
  bool areEqual = true;
  if (I1.getWidth() != I2.getWidth()) {
    std::cerr << "ERROR: Image width differ." << std::endl;
  }

  if (I1.getHeight() != I2.getHeight()) {
    std::cerr << "ERROR: Image height differ." << std::endl;
  }

  unsigned int width = I1.getWidth(), height = I1.getHeight();
  for (unsigned int r = 0; (r < height) && areEqual; ++r) {
    for (unsigned int c = 0; (c < width) && areEqual; ++c) {
      double err = error(I1[r][c], I2[r][c]);
      if (err > thresh) {
        std::cerr << "ERROR: Error (" << err << ") > thresh (" << thresh << ")" << std::endl;
        std::cerr << "\t" << nameI1 << "[" << r << "][" << c << "] = (" << I1[r][c] << ") , " << nameI2 << "[" << r << "][" << c << "] = (" << I2[r][c] << ")" << std::endl;
        areEqual = false;
      }
    }
  }
  return areEqual;
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

template <typename ImageType>
struct vpInputImage
{
  std::string m_errorMsg;
  vpImage<ImageType> m_I;

  vpInputImage(const std::string &errMsg = "", const vpImage<ImageType> &I = vpImage<ImageType>())
    : m_errorMsg(errMsg)
    , m_I(I)
  { }
};

typedef struct vpInputDataset
{
  std::vector<std::pair<std::string, vpInputImage<unsigned char>>> m_ucImages;
  std::vector<std::pair<std::string, vpInputImage<vpHSV<unsigned char, true>>>> m_hsvUCtrue;
  std::vector<std::pair<std::string, vpInputImage<vpHSV<unsigned char, false>>>> m_hsvUCfalse;
  std::vector<std::pair<std::string, vpInputImage<vpHSV<double>>>> m_hsvDouble;
  vpImage<bool> m_Imask;

  vpInputDataset()
  {
    vpImage<vpHSV<unsigned char, true>> Ihsv_uc_true_square(11, 11, vpHSV<unsigned char, true>(0U, 0U, 0U));
    vpImage<vpHSV<unsigned char, true>> Ihsv_uc_true_inversed = Ihsv_uc_true_square;
    vpImage<vpHSV<unsigned char, true>> Ihsv_uc_true_horizontal = Ihsv_uc_true_square;
    vpImage<vpHSV<unsigned char, true>> Ihsv_uc_true_vertical = Ihsv_uc_true_square;
    vpImage<vpHSV<unsigned char, true>> Ihsv_uc_true_pyramid = Ihsv_uc_true_square;

    vpImage<vpHSV<unsigned char, false>> Ihsv_uc_false_square(11, 11, vpHSV<unsigned char, false>(0U, 0U, 0U));
    vpImage<vpHSV<unsigned char, false>> Ihsv_uc_false_inversed = Ihsv_uc_false_square;
    vpImage<vpHSV<unsigned char, false>> Ihsv_uc_false_horizontal = Ihsv_uc_false_square;
    vpImage<vpHSV<unsigned char, false>> Ihsv_uc_false_vertical = Ihsv_uc_false_square;
    vpImage<vpHSV<unsigned char, false>> Ihsv_uc_false_pyramid = Ihsv_uc_false_square;

    vpImage<vpHSV<double>> Ihsv_square(11, 11, vpHSV<double>(0., 0., 0.));
    vpImage<vpHSV<double>> Ihsv_square_inversed = Ihsv_square;
    vpImage<vpHSV<double>> Ihsv_square_horizontal = Ihsv_square;
    vpImage<vpHSV<double>> Ihsv_square_vertical = Ihsv_square;
    vpImage<vpHSV<double>> Ihsv_pyramid = Ihsv_square;

    vpImage<unsigned char> Iuc_square(11, 11, 0);
    vpImage<unsigned char> Iuc_square_inversed = Iuc_square;
    vpImage<unsigned char> Iuc_square_horizontal = Iuc_square;
    vpImage<unsigned char> Iuc_square_vertical = Iuc_square;
    vpImage<unsigned char> Iuc_pyramid = Iuc_square;

    m_Imask.resize(11, 11, false);

    for (unsigned int r = 2; r <=8; ++r) {
      for (unsigned int c = 2; c <= 8; ++c) {
        if ((r == 2) || (r == 3) || (r==7) || (r == 8)) {
          m_Imask[r][c] = true;
        }
        if ((c == 2) || (c == 3) || (c==7) || (c == 8)) {
          m_Imask[r][c] = true;
        }
        double val = 0., val_inversed = 0., val_pyramid = 0.;
        unsigned char val_c = 0, val_c_inversed = 0, val_c_pyramid = 0;
        if (c <= 5) {
          val = 0.1 * static_cast<double>(c - 1);
          val_inversed = 0.1 * static_cast<double>(6 - c);
          val_c = c - 1;
          val_c_inversed = 6 - c;
        }
        else {
          val = 0.1 * static_cast<double>(9 - c);
          val_inversed = 0.1 * static_cast<double>(c - 4);
          val_c = 9 - c;
          val_c_inversed = c - 4;
        }
        if (r <= 5) {
          val_pyramid = val + 0.1 * (r - 2);
          val_c_pyramid = val_c + r - 2;
        }
        else {
          val_pyramid = val + 0.1 * (8 - r);
          val_c_pyramid = val_c + 8 - r;
        }
        if ((r == 2) || (r == 8) /* || (((r == 4) || (r == 6)) && ((c >=4) && (c <= 6))) */) {
          // Horizontal lines of the square
          Ihsv_square[r][c] = vpHSV<double>(val, val, val);
          Ihsv_square_inversed[r][c] = vpHSV<double>(val_inversed, val_inversed, val_inversed);
          Ihsv_square[c][r] = vpHSV<double>(val, val, val);
          Ihsv_square_inversed[c][r] = vpHSV<double>(val_inversed, val_inversed, val_inversed);

          Ihsv_uc_false_square[r][c] = vpHSV<unsigned char, false>(val_c, val_c, val_c);
          Ihsv_uc_false_inversed[r][c] = vpHSV<unsigned char, false>(val_c_inversed, val_c_inversed, val_c_inversed);
          Ihsv_uc_false_square[c][r] = vpHSV<unsigned char, false>(val_c, val_c, val_c);
          Ihsv_uc_false_inversed[c][r] = vpHSV<unsigned char, false>(val_c_inversed, val_c_inversed, val_c_inversed);

          Ihsv_uc_true_square[r][c] = vpHSV<unsigned char, true>(val_c, val_c, val_c);
          Ihsv_uc_true_inversed[r][c] = vpHSV<unsigned char, true>(val_c_inversed, val_c_inversed, val_c_inversed);
          Ihsv_uc_true_square[c][r] = vpHSV<unsigned char, true>(val_c, val_c, val_c);
          Ihsv_uc_true_inversed[c][r] = vpHSV<unsigned char, true>(val_c_inversed, val_c_inversed, val_c_inversed);

          Iuc_square[r][c] = val_c;
          Iuc_square_inversed[r][c] = val_c_inversed;
          Iuc_square[c][r] = val_c;
          Iuc_square_inversed[c][r] = val_c_inversed;
        }
        Ihsv_square_horizontal[r][c] = vpHSV<double>(val, val, val);
        Ihsv_uc_true_horizontal[r][c] = vpHSV<unsigned char, true>(val_c, val_c, val_c);
        Ihsv_uc_false_horizontal[r][c] = vpHSV<unsigned char, false>(val_c, val_c, val_c);
        Iuc_square_horizontal[r][c] = val_c;

        Ihsv_square_vertical[c][r] = vpHSV<double>(val, val, val);
        Ihsv_uc_true_vertical[c][r] = vpHSV<unsigned char, true>(val_c, val_c, val_c);
        Ihsv_uc_false_vertical[c][r] = vpHSV<unsigned char, false>(val_c, val_c, val_c);
        Iuc_square_vertical[c][r] = val_c;

        Ihsv_pyramid[r][c] = vpHSV<double>(val_pyramid, val_pyramid, val_pyramid);
        Ihsv_uc_true_pyramid[r][c] = vpHSV<unsigned char, true>(val_c_pyramid, val_c_pyramid, val_c_pyramid);
        Ihsv_uc_false_pyramid[r][c] = vpHSV<unsigned char, false>(val_c_pyramid, val_c_pyramid, val_c_pyramid);
        Iuc_pyramid[r][c] = val_c_pyramid;
      }
    }
    m_ucImages.emplace_back("Iuc_square", vpInputImage<unsigned char>("", Iuc_square));
    m_ucImages.emplace_back("Iuc_square_inversed", vpInputImage<unsigned char>("", Iuc_square_inversed));
    m_ucImages.emplace_back("Iuc_square_horizontal", vpInputImage<unsigned char>("GX is expected to have values on the whole surface of the square, GY only on the borders", Iuc_square_horizontal));
    m_ucImages.emplace_back("Iuc_square_vertical", vpInputImage<unsigned char>("GY is expected to have values on the whole surface of the square, GX only on the borders", Iuc_square_vertical));
    m_ucImages.emplace_back("Iuc_pyramid", vpInputImage<unsigned char>("GX and GY are expected to have values on the whole square", Iuc_pyramid));

    m_hsvUCtrue.emplace_back("Ihsv_uc_true_square", vpInputImage<vpHSV<unsigned char, true>>("", Ihsv_uc_true_square));
    m_hsvUCtrue.emplace_back("Ihsv_uc_true_inversed", vpInputImage<vpHSV<unsigned char, true>>("", Ihsv_uc_true_inversed));
    m_hsvUCtrue.emplace_back("Ihsv_uc_true_horizontal", vpInputImage<vpHSV<unsigned char, true>>("GX is expected to have values on the whole surface of the square, GY only on the borders", Ihsv_uc_true_horizontal));
    m_hsvUCtrue.emplace_back("Ihsv_uc_true_vertical", vpInputImage<vpHSV<unsigned char, true>>("GY is expected to have values on the whole surface of the square, GX only on the borders", Ihsv_uc_true_vertical));
    m_hsvUCtrue.emplace_back("Ihsv_uc_true_pyramid", vpInputImage<vpHSV<unsigned char, true>>("GX and GY are expected to have values on the whole square", Ihsv_uc_true_pyramid));

    m_hsvUCfalse.emplace_back("Ihsv_uc_false_square", vpInputImage<vpHSV<unsigned char, false>>("", Ihsv_uc_false_square));
    m_hsvUCfalse.emplace_back("Ihsv_uc_false_inversed", vpInputImage<vpHSV<unsigned char, false>>("", Ihsv_uc_false_inversed));
    m_hsvUCfalse.emplace_back("Ihsv_uc_false_horizontal", vpInputImage<vpHSV<unsigned char, false>>("GX is expected to have values on the whole surface of the square, GY only on the borders", Ihsv_uc_false_horizontal));
    m_hsvUCfalse.emplace_back("Ihsv_uc_false_vertical", vpInputImage<vpHSV<unsigned char, false>>("GY is expected to have values on the whole surface of the square, GX only on the borders", Ihsv_uc_false_vertical));
    m_hsvUCfalse.emplace_back("Ihsv_uc_false_pyramid", vpInputImage<vpHSV<unsigned char, false>>("GX and GY are expected to have values on the whole square", Ihsv_uc_false_pyramid));

    m_hsvDouble.emplace_back("I_hsv_double_square", vpInputImage<vpHSV<double>>("", Ihsv_square));
    m_hsvDouble.emplace_back("Ihsv_double_square_inversed", vpInputImage<vpHSV<double>>("", Ihsv_square_inversed));
    m_hsvDouble.emplace_back("Ihsv_double_horizontal", vpInputImage<vpHSV<double>>("GX is expected to have values on the whole surface of the square, GY only on the borders", Ihsv_square_horizontal));
    m_hsvDouble.emplace_back("Ihsv_double_vertical", vpInputImage<vpHSV<double>>("GY is expected to have values on the whole surface of the square, GX only on the borders", Ihsv_square_vertical));
    m_hsvDouble.emplace_back("Ihsv_double_pyramid", vpInputImage<vpHSV<double>>("GX and GY are expected to have values on the whole square", Ihsv_pyramid));
  }
}vpInputDataset;
}
#endif
#endif
