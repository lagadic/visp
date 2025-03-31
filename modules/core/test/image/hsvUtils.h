#include <iostream>
#include <type_traits>

#include <visp3/core/vpColorGetter.h>
#include <visp3/core/vpImage.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
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
bool areAlmostEqual(const vpImage<T> &I1, const vpImage<T> &I2, const double &thresh = 1e-3)
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
        std::cerr << "\tIold[" << r << "][" << c << "] = (" << I1[r][c] << ") , Inew[" << r << "][" << c << "] = (" << I2[r][c] << ")" << std::endl;
        areEqual = false;
      }
    }
  }
  return areEqual;
}
#endif
