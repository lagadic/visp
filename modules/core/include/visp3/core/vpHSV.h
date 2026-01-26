/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * HSV color scale.
 */

#ifndef VP_HSV_H
#define VP_HSV_H

#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRGBa.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <type_traits>

BEGIN_VISP_NAMESPACE
template<typename T, bool = true>
class vpHSV;

template <typename T, bool useFullScale>
std::ostream &operator<<(std::ostream &os, const vpHSV<T, useFullScale> &hsv);

#ifndef VISP_PYTHON_PREPROCESSOR_RUNNING
namespace
{
/**
 * \brief Structure that gives the variance of the different channels of a HSV pixel assuming that
 * each channel follows a continuous uniform law defined on the interval [0.; 1.].
 *
 * \tparam T : The type used to encode the channels.
 * \tparam otherUseFullScale : Unused.
 * \tparam EnableIf : Enable this structure only for floating point types.
 */
template <typename T, bool otherUseFullScale, typename EnableIf = void>
struct UniformLawVariance;

/**
 * \brief Structure that gives the variance of the different channels of a HSV pixel assuming that
 * each channel follows a continuous uniform law defined on the interval [0.; 1.].
 * Enable this structure only for floating point types.
 * \tparam T : The type used to encode the channels.
 * \tparam useFullScale : Unused.
 */
template <typename T, bool useFullScale>
struct UniformLawVariance<T, useFullScale, typename std::enable_if<std::is_floating_point<T>::value>::type>
{
  // Variance of a continuous uniform law defined on the interval [a; b] = (b - a)^2 / 12
  // Here, a = 0, b = 1
  static constexpr float hueVariance = 1.f/12.f;
  static constexpr float otherChannelsVariance = 1.f/12.f;
};

/**
 * \brief Structure that gives the variance of the different channels of a HSV pixel assuming that
 * each channel follows a discrete uniform law defined on the interval {0; 1; ...; 255}.
 */
template <>
struct UniformLawVariance<unsigned char, true>
{
  // Variance of a discrete uniform law defined on the interval {a; a + 1; ... b} = ((b - a + 1)^2 - 1)/ 12
  // Here, a = 0, b = 255
  static constexpr float hueVariance = (256.f * 256.f - 1.f)/12.f;
  static constexpr float otherChannelsVariance = (256.f * 256.f - 1.f)/12.f;
};

/**
 * \brief Structure that gives the variance of the different channels of a HSV pixel assuming that
 * each channel follows a discrete uniform law defined on the interval {0; 1; ...; 255} for the Saturation and
 * Value channels and {0; 1; ...; maxHueUsingLimitedRange} for the Hue channel.
 */
template <>
struct UniformLawVariance<unsigned char, false>
{
  // Variance of a discrete uniform law defined on the interval {a; a + 1; ... b} = ((b - a + 1)^2 - 1)/ 12
  // Here, a = 0, b = 255 for the Saturation and Value channels
  // and {0; 1; ...; maxHueUsingLimitedRange} for the Hue channel.
  static constexpr float hueVariance = (180.f * 180.f - 1.f)/12.f;
  static constexpr float otherChannelsVariance = (256.f * 256.f - 1.f)/12.f;
};
}
#endif

/**
 * \brief Class implementing the HSV pixel format.
 *
 * \tparam T The type of the channels. Either a floating point type (float, double) or unsigned char.
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; maxHueUsingLimitedRange].
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you are interested in how you can convert vpHSV to and from other type of data, or
 * how to use it for color segmentation, you may have a look at:</span><br>

 * - \ref tutorial-hsv-segmentation-intro
 */
template<typename T, bool useFullScale>
class vpHSV
{
public:
  /**
   * \brief Construct a new vpHSV object using floating point channels.
   *
   * \param[in] H_ The value of the Hue channel.
   * \param[in] S_ The value of the Saturation channel.
   * \param[in] V_ The value of the Value channel.
   */
  explicit vpHSV(const double &H_ = 0., const double &S_ = 0., const double &V_ = 0.)
    : H(static_cast<T>(H_))
    , S(static_cast<T>(S_))
    , V(static_cast<T>(V_))
  { }

  /**
   * \brief Construct a new vpHSV object from a vpColVector.
   *
   * \param[in] v The values must be in the range that corresponds to the type
   * used to encode the channels.
   */
  vpHSV(const vpColVector &v)
  {
    this->set(v);
  }

  /**
   * \brief Default copy constructor.
   */
  vpHSV(const vpHSV<T, useFullScale> &) = default;

#ifndef VISP_PYTHON_PREPROCESSOR_RUNNING
  /**
   * \brief Construct a new vpHSV object using unsigned char channels and the full range [0; 255] from a vpHSV object
   * whose channels are in floating point format.
   *
   * Enable the method only if the constructed object uses unsigned char format and uses the full range
   * [0; 255] and the object that is used as reference uses floating point format.
   *
   * \tparam U The format of the constructed object.
   * \tparam V The format of the base object.
   * \param[in] other A floating point format vpHSV.
   */
  template<typename U = T, typename V, typename std::enable_if<std::is_same<T, unsigned char>::value &&std::is_floating_point<V>::value &&useFullScale, U>::type = 0 >
  vpHSV(const vpHSV<V> &other)
  {
    buildFrom(other);
  }

  /**
   * \brief Construct a new vpHSV object using unsigned char channels and the limited range [0; maxHueUsingLimitedRange]
   * from a vpHSV object whose channels are in floating point format.
   *
   * Enable the method only if the constructed object uses unsigned char format and uses the limited range
   * [0; maxHueUsingLimitedRange] and the object that is used as reference uses floating point format.
   *
   * \tparam U The format of the constructed object.
   * \tparam V The format of the base object.
   * \param[in] other A floating point format vpHSV.
   */
  template<typename U = T, typename V, typename std::enable_if<std::is_same<T, unsigned char>::value &&std::is_floating_point<V>::value && !useFullScale, U>::type = 0 >
  vpHSV(const vpHSV<V> &other)
  {
    buildFrom(other);
  }

  /**
   * \brief Construct a new floating point vpHSV object from an unsigned char vpHSV object.
   *
   * Enable the method only if the constructed object uses the floating point format for its channels.
   *
   * \tparam U The type of the channels of the constructed vpHSV pixels.
   * \tparam otherUseFullScale True if the reference object uses unsigned char and the full range [0; 255], false if it
   * uses unsigned char and the limited range [0; maxHueUsingLimitedRange].
   * \param[in] other The reference object.
   */
  template<typename U = T, bool otherUseFullScale, typename std::enable_if<std::is_floating_point<U>::value>::type...>
  vpHSV(const vpHSV<unsigned char, otherUseFullScale> &other)
  {
    buildFrom(other);
  }
#endif

  /**
  * \brief Construct a new vpHSV object from a vpRGBa object.
  *
  * \param[in] rgba The reference vpRGBa object.
  */
  vpHSV(const vpRGBa &rgba)
  {
    buildFrom(rgba);
  }

  /**
   * Default destructor.
   */
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  virtual ~vpHSV() = default;
#else
  virtual ~vpHSV() { }
#endif

  /**
   * \brief Modify the object to be the result of the conversion of the vpRGBa object into HSV format.
   *
   * \param[in] rgba The vpRGBa object that serves as model/
   * \return Reference to the modified object.
   */
  vpHSV<T, useFullScale> &buildFrom(const vpRGBa &rgba);

  /**
   * \brief Convert a floating point HSV into a unsigned char HSV using the full range [0; 255].
   * Enable the method only if the modified object uses unsigned char and full range [0; 255] and the base
   * object uses a floating point format.
   * \tparam V The type of the channels of the vpHSV pixels that serves as reference.
   * \param[in] other The floating point HSV.
   * \return Reference to the modified object.
   */
  template<typename V, bool otherUseFullScale>
  typename std::enable_if<std::is_same<T, unsigned char>::value &&std::is_floating_point<V>::value &&useFullScale, vpHSV<T, useFullScale> &>::type
    buildFrom(const vpHSV<V, otherUseFullScale> &other)
  {
    H = static_cast<T>(other.H * 255.);
    S = static_cast<T>(other.S * 255.);
    this->V = static_cast<T>(other.V * 255.);
    return *this;
  }

  /**
   * \brief Convert a floating point HSV into a unsigned char HSV using the limited range [0; maxHueUsingLimitedRange].
   * Enable the method only if the modified object uses unsigned char and limited range
   * [0; maxHueUsingLimitedRange] and the base object uses a floating point format.
   * \tparam U The type of the channels of the vpHSV pixels that is modified.
   * \tparam V The type of the channels of the vpHSV pixels that serves as reference.
   * \param[in] other The floating point HSV.
   * \return Reference to the modified object.
   */
  template<typename U = T, typename V, bool otherUseFullScale>
  typename std::enable_if<std::is_same<U, unsigned char>::value &&std::is_floating_point<V>::value && !useFullScale, vpHSV<T, useFullScale> &>::type
    buildFrom(const vpHSV<V, otherUseFullScale> &other)
  {
    H = static_cast<T>(other.H * static_cast<double>(maxHueUsingLimitedRange));
    S = static_cast<T>(other.S * 255.);
    this->V = static_cast<T>(other.V * 255.);
    return *this;
  }

  /**
   * \brief Convert a vpHSV that uses unsigned char for its channels into a vpHSV that uses floating point for its channels.
   *
   * Enable the method only if the modified object uses floating point format.
   *
   * \tparam U The type of the channels of the vpHSV pixels.
   * \tparam otherUseFullScale True if the reference object uses unsigned char and the full range [0; 255], false if it
   * uses unsigned char and the limited range [0; maxHueUsingLimitedRange].
   * \param[in] other The unsigned char vpHSV.
   * \return Reference to the modified object.
   */
  template<typename U = T, bool otherUseFullScale>
  typename std::enable_if<std::is_floating_point<U>::value, vpHSV<T, useFullScale> &>::type
    buildFrom(const vpHSV<unsigned char, otherUseFullScale> &other)
  {
    if (otherUseFullScale) {
      H = static_cast<T>(other.H) / static_cast<T>(255.);
    }
    else {
      H = static_cast<T>(other.H) / static_cast<T>(maxHueUsingLimitedRange);
    }
    S = static_cast<T>(other.S) / static_cast<T>(255.);
    V = static_cast<T>(other.V) / static_cast<T>(255.);
    return *this;
  }

  /**
   * \brief Convert a floating point HSV into another floating point type HSV.
   * Enable the method only if the modified object uses is a floating point format, the base object too
   * but the formats are different. The type "int" is not used, it is here only because float and doubles are
   * "not [a] valid type for a template non-type parameter"
   * \tparam U The type of the channels of the vpHSV pixels that is modified.
   * \tparam V The type of the channels of the vpHSV pixels that serves as reference.
   * \tparam otherUseFullScale To avoid problem if one was created with true and the other false (even it is not used for
   * floating point types).

   * \param[in] other The floating point HSV.
   * \return vpHSV<T, useFullScale>& Reference to the modified object.
   */
  template<typename U = T, typename V, bool otherUseFullScale>
  typename std::enable_if<std::is_floating_point<U>::value &&std::is_floating_point<V>::value && !std::is_same<T, V>::value, vpHSV<T, useFullScale> &>::type
    buildFrom(const vpHSV<V, otherUseFullScale> &other)
  {
    H = static_cast<T>(other.H);
    S = static_cast<T>(other.S);
    this->V = static_cast<T>(other.V);
    return *this;
  }

  /**
   * \brief Compute the normalized HSV values (i.e. in the range [0; 1]) that correspond to a vpRGBa object.
   *
   * \param[in] rgba The RGB pixel.
   * \return vpColVector Vector of normalized HSV values.
   */
  static vpColVector computeNormalizedHSV(const vpRGBa &rgba)
  {
    double red, green, blue;
    double h, s, v;
    double min, max;

    red = rgba.R / 255.0;
    green = rgba.G / 255.0;
    blue = rgba.B / 255.0;

    if (red > green) {
      max = std::max<double>(red, blue);
      min = std::min<double>(green, blue);
    }
    else {
      max = std::max<double>(green, blue);
      min = std::min<double>(red, blue);
    }

    v = max;

    if (!vpMath::equal(max, 0.0, std::numeric_limits<double>::epsilon())) {
      s = (max - min) / max;
    }
    else {
      s = 0.0;
    }

    if (vpMath::equal(s, 0.0, std::numeric_limits<double>::epsilon())) {
      h = 0.0;
    }
    else {
      double delta = max - min;

      if (vpMath::equal(red, max, std::numeric_limits<double>::epsilon())) {
        h = (green - blue) / delta;
      }
      else if (vpMath::equal(green, max, std::numeric_limits<double>::epsilon())) {
        h = 2.0 + ((blue - red) / delta);
      }
      else {
        h = 4.0 + ((red - green) / delta);
      }

      h /= 6.0;
      if (h < 0.0) {
        h += 1.0;
      }
      else if (h > 1.0) {
        h -= 1.0;
      }
    }

    vpColVector hsv(3);
    hsv[0] = h;
    hsv[1] = s;
    hsv[2] = v;
    return hsv;
  }

  /**
   * \brief Compute the square of the Mahalanobis distance between two HSV pixels.
   * It is assumed that the channels are independent and follow a uniform distribution law.
   *
   * \param[in] a The first pixel to compare.
   * \param[in] b The second pixel to compare.
   * \param[out] diff The vector (b - a).
   * \return float The squared Mahalanobis distance between a and b.
   */
  template <typename ArithmeticType>
  inline static ArithmeticType squaredMahalanobisDistance(const vpHSV<T, useFullScale> &a, const vpHSV<T, useFullScale> &b, vpColVector &diff)
  {
    static const ArithmeticType invHueVariance = 1.f / UniformLawVariance<T, useFullScale>::hueVariance;
    static const ArithmeticType invOtherChannelsVariance = 1.f / UniformLawVariance<T, useFullScale>::otherChannelsVariance;
    diff.resize(3);
    diff[0] = b.H - a.H;
    diff[1] = b.S - a.S;
    diff[2] = b.V - a.V;
    ArithmeticType distance = diff[0] * diff[0] * invHueVariance + invOtherChannelsVariance * (diff[1] * diff[1] + diff[2] * diff[2]);
    return distance;
  }

  /**
   * \brief Compute the square of the Mahalanobis distance between two HSV pixels.
   * It is assumed that the channels are independent and follow a uniform distribution law.
   *
   * \param[in] a The first pixel to compare.
   * \param[in] b The second pixel to compare.
   * \return float The squared Mahalanobis distance between a and b.
   */
  template <typename ArithmeticType>
  inline static ArithmeticType squaredMahalanobisDistance(const vpHSV<T, useFullScale> &a, const vpHSV<T, useFullScale> &b)
  {
    vpColVector diff;
    return squaredMahalanobisDistance<ArithmeticType>(a, b, diff);
  }

  /**
   * \brief Compute the Mahalanobis distance between two HSV pixels.
   * It is assumed that the channels are independent and follow a uniform distribution law.
   *
   * \param[in] a The first pixel to compare.
   * \param[in] b The second pixel to compare.
   * \return float The Mahalanobis distance between a and b.
   */
  template <typename ArithmeticType>
  inline static ArithmeticType mahalanobisDistance(const vpHSV<T, useFullScale> &a, const vpHSV<T, useFullScale> &b)
  {
    return std::sqrt(squaredMahalanobisDistance<ArithmeticType>(a, b));
  }

  /**
   * \brief Compute the Mahalanobis distance between two HSV pixels.
   * It is assumed that the channels are independent and follow a uniform distribution law.
   *
   * \param[in] a The first pixel to compare.
   * \param[in] b The second pixel to compare.
   * \param[out] diff The vector (b - a).
   * \return float The Mahalanobis distance between a and b.
   */
  template <typename ArithmeticType>
  inline static ArithmeticType mahalanobisDistance(const vpHSV<T, useFullScale> &a, const vpHSV<T, useFullScale> &b, vpColVector &diff)
  {
    return std::sqrt(squaredMahalanobisDistance<ArithmeticType>(a, b, diff));
  }

  // Operators
  vpHSV<T, useFullScale> &operator=(vpHSV<T, useFullScale> &&) = default;
  vpHSV<T, useFullScale> &operator=(const vpHSV<T, useFullScale> &) = default;

  vpHSV<T, useFullScale> &operator=(const vpColVector &v)
  {
    vpHSV<T, useFullScale> vAsHSV(v);
    *this = vAsHSV;
    return *this;
  }


  bool operator==(const vpHSV<T, useFullScale> &v) const
  {
    return(vpMath::equal(v.H, H, 1e-6) && vpMath::equal(v.S, S, 1e-6) && vpMath::equal(v.V, V, 1e-6));
  }

  bool operator!=(const vpHSV<T, useFullScale> &v) const
  {
    return !(*this == v);
  }

  vpColVector operator-(const vpHSV<T, useFullScale> &v) const
  {
    return this->toColVector() - v.toColVector();
  }

  vpHSV<T, useFullScale> operator+(const vpHSV<T, useFullScale> &v) const
  {
    vpHSV<T, useFullScale> result;
    result.H = H + v.H;
    result.S = S + v.S;
    result.V = V + v.V;
    return result;
  }

  vpColVector operator-(const vpColVector &v) const
  {
    return this->toColVector() - v;
  }

  vpColVector operator+(const vpColVector &v) const
  {
    vpColVector result(3);
    result[0] = H + v[0];
    result[1] = S + v[1];
    result[2] = V + v[2];
    return result;
  }

  /**
   * \brief Cast a vpHSV into a vpColVector.
   *
   * \return vpColVector
   */
  vpColVector toColVector() const
  {
    vpColVector color(3);
    color[0] = H;
    color[1] = S;
    color[2] = V;
    return color;
  }

  /**
   * \brief Cast a vpHSV into a string, for display purpose.
   *
   * \return std::string
   */
  virtual std::string toString() const;

  friend std::ostream &operator<< <>(std::ostream &os, const vpHSV<T, useFullScale> &hsv);

public:
  T H; /*!< The Hue channel.*/
  T S; /*!< The Saturation channel.*/
  T V; /*!< The Value channel.*/

  /**
   * \brief Number of channels a HSV pixel is made of.
   */
  static constexpr unsigned char nbChannels = 3;

  /**
   * \brief Maximum value of the Hue channel when using unsigned char and the limited range.
   */
  static constexpr unsigned char maxHueUsingLimitedRange = 179;

private:
  /**
   * \brief Permit to initialize a vpHSV object using a vector.
   *
   * \tparam Tp The type of the channels of the vpHSV pixels.
   * \param[in] v A vector whose size must be equal to 3.
   */
  template<typename Tp = T>
  inline
    typename std::enable_if<std::is_floating_point<Tp>::value, void>::type  set(const vpColVector &v)
  {
    // const Tp limMin = 0.;
    // const Tp limMax = 1.;
    // if ((v[0] < limMin) || (v[0] > limMax)) {
    //   // throw exception
    // }
    H = v[0];
    // if ((v[1] < limMin) || (v[1] > limMax)) {
    //   // throw exception
    // }
    S = v[1];
    // if ((v[2] < limMin) || (v[2] > limMax)) {
    //   // throw exception
    // }
    V = v[2];
  }

  template<typename Tp = T>
  inline
    typename std::enable_if<std::is_same<Tp, unsigned char>::value, void>::type  set(const vpColVector &v)
  {
    // const Tp otherMax = std::numeric_limits<U>::max();
    // Tp hmax;
    // if (useFullScale) {
    //   hmax = std::numeric_limits<U>::max();
    // }
    // else {
    //   hmax = maxHueUsingLimitedRange;
    // }

    // if (v[0] > hmax) {
    //   // throw exception
    // }
    H = static_cast<T>(v[0]);

    // if (v[1] > otherMax) {
    //   // throw exception
    // }
    S = static_cast<T>(v[1]);

    // if (v[2] > otherMax) {
    //   // throw exception
    // }
    V = static_cast<T>(v[2]);
  }
};

template <typename T, bool useFullScale>
std::string vpHSV<T, useFullScale>::toString() const
{
  std::stringstream ss;
  ss << "vpHSV<";
  using CastType = typename std::conditional<std::is_integral<T>::value, int, T>::type;
  if (std::is_same<T, unsigned char>::value) {
    ss << "uchar";
  }
  else if (std::is_same<T, double>::value) {
    ss << "double";
  }
  else {
    ss << "other";
  }

  if (useFullScale) {
    std::cout << ", full";
  }
  else {
    std::cout << ", partial";
  }
  ss << " scale> (H, S, V): (" << (CastType)H << " , " << (CastType)S << " , " << (CastType)V << ")";
  return ss.str();
}

template <typename T, bool useFullScale>
std::ostream &operator<<(std::ostream &os, const vpHSV<T, useFullScale> &hsv)
{
  os << hsv.toString();
  return os;
}

template<>
VISP_EXPORT vpHSV<unsigned char, false> &vpHSV<unsigned char, false>::buildFrom(const vpRGBa &rgba);

template<>
VISP_EXPORT vpHSV<unsigned char, true> &vpHSV<unsigned char, true>::buildFrom(const vpRGBa &rgba);

template<>
VISP_EXPORT vpHSV<double> &vpHSV<double>::buildFrom(const vpRGBa &rgba);

END_VISP_NAMESPACE
#endif
#endif
