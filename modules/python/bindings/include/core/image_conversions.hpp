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
 * Python bindings.
 */

#ifndef VISP_PYTHON_CORE_IMAGE_CONVERT_HPP
#define VISP_PYTHON_CORE_IMAGE_CONVERT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>


namespace
{
using ConversionFunction1D = void(*)(unsigned char *, unsigned char *, unsigned int);
using ConversionFunction2D = void(*)(unsigned char *, unsigned char *, unsigned int, unsigned int);
using ConversionFunction2DWithFlip = void(*)(unsigned char *, unsigned char *, unsigned int, unsigned int, bool);
using ConversionFunction2DWithFlipAndNThreads = void(*)(unsigned char *, unsigned char *, unsigned int, unsigned int, bool, unsigned int);


using ComputeBytesFunction = unsigned(*)(unsigned int, unsigned int);

void call_conversion_fn(ConversionFunction2D fn, unsigned char *src, unsigned char *dest, unsigned int h, unsigned int w)
{
  fn(src, dest, h, w);
}
void call_conversion_fn(ConversionFunction2DWithFlip fn, unsigned char *src, unsigned char *dest, unsigned int h, unsigned int w, bool flip)
{
  fn(src, dest, h, w, flip);
}
void call_conversion_fn(ConversionFunction1D fn, unsigned char *src, unsigned char *dest, unsigned int h, unsigned int w)
{
  fn(src, dest, h * w);
}

template <typename ConversionFn>
struct SimpleConversionStruct
{
  SimpleConversionStruct(const std::string &name, ConversionFn fn, unsigned int srcBytesPerPixel, unsigned int destBytesPerPixel) :
    name(name), fn(fn), srcBytesPerPixel(srcBytesPerPixel), destBytesPerPixel(destBytesPerPixel)
  { }
  std::string name;
  ConversionFn fn;
  unsigned int srcBytesPerPixel;
  unsigned int destBytesPerPixel;

  void add_conversion_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert)
  {
    pyImageConvert.def_static(name.c_str(), [this](py::array_t<unsigned char, py::array::c_style> &src,
                                                   py::array_t<unsigned char, py::array::c_style> &dest) {
                                                     py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                     if (bufsrc.ndim < 2 || bufdest.ndim < 2) {
                                                       throw std::runtime_error("Expected to have src and dest arrays with at least two dimensions.");
                                                     }
                                                     if (bufsrc.shape[0] != bufdest.shape[0] || bufsrc.shape[1] != bufdest.shape[1]) {
                                                       std::stringstream ss;
                                                       ss << "src and dest must have the same number of pixels, but got src = " << shape_to_string(bufsrc.shape);
                                                       ss << "and dest = " << shape_to_string(bufdest.shape);
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     if (srcBytesPerPixel > 1 && (bufsrc.ndim != 3 || bufsrc.shape[2] != srcBytesPerPixel)) {
                                                       std::stringstream ss;
                                                       ss << "Source array should be a 3D array of shape (H, W, " << srcBytesPerPixel << ")";
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     else if (srcBytesPerPixel == 1 && bufsrc.ndim == 3 && bufsrc.shape[2] > 1) {
                                                       throw std::runtime_error("Source array should be a either a 2D array of shape H x W or a 3D array of shape (H, W, 1)");
                                                     }
                                                     if (destBytesPerPixel > 1 && (bufdest.ndim != 3 || bufdest.shape[2] != destBytesPerPixel)) {
                                                       std::stringstream ss;
                                                       ss << "Destination array should be a 3D array of shape (H, W, " << destBytesPerPixel << ")";
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     else if (destBytesPerPixel == 1 && bufdest.ndim == 3 && bufdest.shape[2] > 1) {
                                                       throw std::runtime_error("Destination should be a either a 2D array of shape H x W or a 3D array of shape (H, W, 1)");
                                                     }
                                                     unsigned char *src_ptr = static_cast<unsigned char *>(bufsrc.ptr);
                                                     unsigned char *dest_ptr = static_cast<unsigned char *>(bufdest.ptr);
                                                     call_conversion_fn(fn, src_ptr, dest_ptr, bufsrc.shape[0], bufsrc.shape[1]);
    }, "See C++ documentation of the function for more info", py::arg("src"), py::arg("dest"));
  }
};

template <>
struct SimpleConversionStruct<ConversionFunction2DWithFlip>
{
  SimpleConversionStruct(const std::string &name, ConversionFunction2DWithFlip fn, unsigned int srcBytesPerPixel, unsigned int destBytesPerPixel) :
    name(name), fn(fn), srcBytesPerPixel(srcBytesPerPixel), destBytesPerPixel(destBytesPerPixel)
  { }
  std::string name;
  ConversionFunction2DWithFlip fn;
  unsigned int srcBytesPerPixel;
  unsigned int destBytesPerPixel;

  void add_conversion_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert)
  {
    pyImageConvert.def_static(name.c_str(), [this](py::array_t<unsigned char, py::array::c_style> &src,
                                                   py::array_t<unsigned char, py::array::c_style> &dest, bool flip) {
                                                     py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                     if (bufsrc.ndim < 2 || bufdest.ndim < 2) {
                                                       throw std::runtime_error("Expected to have src and dest arrays with at least two dimensions.");
                                                     }
                                                     if (bufsrc.shape[0] != bufdest.shape[0] || bufsrc.shape[1] != bufdest.shape[1]) {
                                                       std::stringstream ss;
                                                       ss << "src and dest must have the same number of pixels, but got src = " << shape_to_string(bufsrc.shape);
                                                       ss << "and dest = " << shape_to_string(bufdest.shape);
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     if (srcBytesPerPixel > 1 && (bufsrc.ndim != 3 || bufsrc.shape[2] != srcBytesPerPixel)) {
                                                       std::stringstream ss;
                                                       ss << "Source array should be a 3D array of shape (H, W, " << srcBytesPerPixel << ")";
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     else if (srcBytesPerPixel == 1 && bufsrc.ndim == 3 && bufsrc.shape[2] > 1) {
                                                       throw std::runtime_error("Source array should be a either a 2D array of shape H x W or a 3D array of shape (H, W, 1)");
                                                     }
                                                     if (destBytesPerPixel > 1 && (bufdest.ndim != 3 || bufdest.shape[2] != destBytesPerPixel)) {
                                                       std::stringstream ss;
                                                       ss << "Destination array should be a 3D array of shape (H, W, " << destBytesPerPixel << ")";
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     else if (destBytesPerPixel == 1 && bufdest.ndim == 3 && bufdest.shape[2] > 1) {
                                                       throw std::runtime_error("Destination should be a either a 2D array of shape H x W or a 3D array of shape (H, W, 1)");
                                                     }
                                                     unsigned char *src_ptr = static_cast<unsigned char *>(bufsrc.ptr);
                                                     unsigned char *dest_ptr = static_cast<unsigned char *>(bufdest.ptr);
                                                     call_conversion_fn(fn, src_ptr, dest_ptr, bufsrc.shape[0], bufsrc.shape[1], flip);
    }, "See C++ documentation of the function for more info", py::arg("src"), py::arg("dest"), py::arg("flip") = false);
  }
};

template <typename ConversionFn>
struct ConversionFromYUVLike
{
  ConversionFromYUVLike(const std::string &name, ConversionFn fn, ComputeBytesFunction sourceBytesFn, unsigned int destBytesPerPixel) :
    name(name), fn(fn), sourceBytesFn(sourceBytesFn), destBytesPerPixel(destBytesPerPixel)
  { }
  std::string name;
  ConversionFn fn;
  ComputeBytesFunction sourceBytesFn;

  unsigned int destBytesPerPixel;

  void add_conversion_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert)
  {
    pyImageConvert.def_static(name.c_str(), [this](py::array_t<unsigned char, py::array::c_style> &src,
                                                   py::array_t<unsigned char, py::array::c_style> &dest) {
                                                     py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                     if (bufdest.ndim < 2) {
                                                       throw std::runtime_error("Expected to have dest array with at least two dimensions.");
                                                     }

                                                     unsigned int height = bufdest.shape[0], width = bufdest.shape[1];

                                                     unsigned expectedSourceBytes = sourceBytesFn(height, width);

                                                     unsigned actualBytes = 1;
                                                     for (unsigned int i = 0; i < bufsrc.ndim; ++i) {
                                                       actualBytes *= bufsrc.shape[i];
                                                     }

                                                     if (actualBytes != expectedSourceBytes) {
                                                       std::stringstream ss;
                                                       ss << "Expected to have " << expectedSourceBytes << " bytes in the input array, but got " << actualBytes << " elements.";
                                                       throw std::runtime_error(ss.str());
                                                     }

                                                     if (destBytesPerPixel > 1 && (bufdest.ndim != 3 || bufdest.shape[2] != destBytesPerPixel)) {
                                                       std::stringstream ss;
                                                       ss << "Destination array should be a 3D array of shape (H, W, " << destBytesPerPixel << ")";
                                                       throw std::runtime_error(ss.str());
                                                     }
                                                     else if (destBytesPerPixel == 1 && bufdest.ndim == 3 && bufdest.shape[2] > 1) {
                                                       throw std::runtime_error("Destination should be a either a 2D array of shape H x W or a 3D array of shape (H, W, 1)");
                                                     }


                                                     unsigned char *src_ptr = static_cast<unsigned char *>(bufsrc.ptr);
                                                     unsigned char *dest_ptr = static_cast<unsigned char *>(bufdest.ptr);
                                                     call_conversion_fn(fn, src_ptr, dest_ptr, bufdest.shape[0], bufdest.shape[1]);
    }, py::arg("src"), py::arg("dest"));
  }
};

unsigned size422(unsigned h, unsigned w)
{
  return h * w + (h * (w / 2)) * 2;
}
unsigned size420(unsigned h, unsigned w)
{
  return h * w + ((h / 2) * (w / 2)) * 2;
}
unsigned size411(unsigned h, unsigned w)
{
  return h * w + ((h / 4) * (w / 4)) * 2;
}

void rgb_or_rgba_to_hsv_verification(const py::buffer_info &bufsrc, const py::buffer_info &bufdest, const unsigned destBytes, const unsigned height, const unsigned width)
{
  if (bufsrc.ndim != 3 || bufdest.ndim != 3) {
    throw std::runtime_error("Expected to have src and dest arrays with at least two dimensions.");
  }
  if (bufdest.shape[0] != 3) {
    throw std::runtime_error("Source array should be a 3D array of shape (3, H, W) ");
  }
  if (bufsrc.shape[2] != destBytes) {
    std::stringstream ss;
    ss << "Target array should be a 3D array of shape (H, W, " << destBytes << ")";
    throw std::runtime_error(ss.str());
  }
  if (bufsrc.shape[0] != height || bufsrc.shape[1] != width) {
    std::stringstream ss;
    ss << "src and dest must have the same number of pixels, but got HSV planes with dimensions (" << height << ", " << width << ")";
    ss << "and RGB array with dimensions (" << bufdest.shape[0] << ", " << bufdest.shape[1] << ")";
    throw std::runtime_error(ss.str());
  }
}

void add_hsv_double_to_rgb_or_rgba_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert,
                                    void (*fn)(const double *, const double *, const double *, unsigned char *, unsigned int), const char *name, const unsigned destBytes)
{
  pyImageConvert.def_static(name, [fn, destBytes](py::array_t<double, py::array::c_style> &src,
                                                  py::array_t<unsigned char, py::array::c_style> &dest) {
                                                    py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                    const unsigned height = bufsrc.shape[1];
                                                    const unsigned width = bufsrc.shape[2];
                                                    rgb_or_rgba_to_hsv_verification(bufdest, bufsrc, destBytes, height, width);

                                                    const double *h = static_cast<double *>(bufsrc.ptr);
                                                    const double *s = h + (height * width);
                                                    const double *v = s + (height * width);
                                                    unsigned char *dest_ptr = static_cast<unsigned char *>(bufdest.ptr);
                                                    fn(h, s, v, dest_ptr, height * width);

  }, "Convert from HSV Planes (as a 3 x H x W array) to a an RGB/RGBA array (as an H x W x 3 or H x W x 4 array)", py::arg("hsv"), py::arg("rgb"));
}

void add_hsv_uchar_to_rgb_or_rgba_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert,
                                    void (*fn)(const unsigned char *, const unsigned char *, const unsigned char *, unsigned char *, unsigned int, bool), const char *name, const unsigned destBytes)
{
  pyImageConvert.def_static(name, [fn, destBytes](py::array_t<unsigned char, py::array::c_style> &src,
                                                  py::array_t<unsigned char, py::array::c_style> &dest, bool h_full) {
                                                    py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                    const unsigned height = bufsrc.shape[1];
                                                    const unsigned width = bufsrc.shape[2];
                                                    rgb_or_rgba_to_hsv_verification(bufdest, bufsrc, destBytes, height, width);

                                                    const unsigned char *h = static_cast<unsigned char *>(bufsrc.ptr);
                                                    const unsigned char *s = h + (height * width);
                                                    const unsigned char *v = s + (height * width);
                                                    unsigned char *dest_ptr = static_cast<unsigned char *>(bufdest.ptr);
                                                    fn(h, s, v, dest_ptr, height * width, h_full);

  }, "Convert from HSV Planes (as a 3 x H x W array) to a an RGB/RGBA array (as an H x W x 3 or H x W x 4 array)", py::arg("hsv"), py::arg("rgb"), py::arg("h_full") = true);
}

void add_rgb_or_rgba_uchar_to_hsv_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert,
                                    void (*fn)(const unsigned char *, unsigned char *, unsigned char *, unsigned char *, unsigned int, bool), const char *name, const unsigned destBytes)
{
  pyImageConvert.def_static(name, [fn, destBytes](py::array_t<unsigned char, py::array::c_style> &src,
                                                  py::array_t<unsigned char, py::array::c_style> &dest,
                                                  bool h_full) {
                                                    py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                    const unsigned height = bufdest.shape[1];
                                                    const unsigned width = bufdest.shape[2];
                                                    rgb_or_rgba_to_hsv_verification(bufsrc, bufdest, destBytes, height, width);

                                                    unsigned char *h = static_cast<unsigned char *>(bufdest.ptr);
                                                    unsigned char *s = h + (height * width);
                                                    unsigned char *v = s + (height * width);
                                                    const unsigned char *rgb = static_cast<unsigned char *>(bufsrc.ptr);
                                                    fn(rgb, h, s, v, height * width, h_full);

  }, "Convert from HSV Planes (as a 3 x H x W array) to a an RGB/RGBA array (as an H x W x 3 or H x W x 4 array)", py::arg("rgb"), py::arg("hsv"), py::arg("h_full") = true);
}

void add_rgb_or_rgba_double_to_hsv_binding(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert,
                                    void (*fn)(const unsigned char *, double *, double *, double *, unsigned int), const char *name, const unsigned destBytes)
{
  pyImageConvert.def_static(name, [fn, destBytes](py::array_t<unsigned char, py::array::c_style> &src,
                                                  py::array_t<double, py::array::c_style> &dest) {
                                                    py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                                    const unsigned height = bufdest.shape[1];
                                                    const unsigned width = bufdest.shape[2];
                                                    rgb_or_rgba_to_hsv_verification(bufsrc, bufdest, destBytes, height, width);

                                                    double *h = static_cast<double *>(bufdest.ptr);
                                                    double *s = h + (height * width);
                                                    double *v = s + (height * width);
                                                    const unsigned char *rgb = static_cast<unsigned char *>(bufsrc.ptr);
                                                    fn(rgb, h, s, v, height * width);

  }, "Convert from HSV Planes (as a 3 x H x W array) to a an RGB/RGBA array (as an H x W x 3 or H x W x 4 array)", py::arg("rgb"), py::arg("hsv"));
}

/* Demosaicing implem */
template <class DataType>
void add_demosaic_to_rgba_fn(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert, void (*fn)(const DataType *, DataType *, unsigned int, unsigned int, unsigned int), const char *name)
{
  pyImageConvert.def_static(name, [fn](py::array_t<DataType, py::array::c_style> &src,
                                       py::array_t<DataType, py::array::c_style> &dest,
                                       unsigned int num_threads) {
                                         py::buffer_info bufsrc = src.request(), bufdest = dest.request();
                                         const unsigned destBytes = 4;

                                         if (bufsrc.ndim != 2 || bufdest.ndim != 3) {
                                           throw std::runtime_error("Expected to have source array with two dimensions and destination RGBA array with 3.");
                                         }
                                         if (bufdest.shape[2] != destBytes) {
                                           std::stringstream ss;
                                           ss << "Target array should be a 3D array of shape (H, W, " << destBytes << ")";
                                           throw std::runtime_error(ss.str());
                                         }
                                         const unsigned height = bufdest.shape[0];
                                         const unsigned width = bufdest.shape[1];
                                         if (bufsrc.shape[0] != height || bufsrc.shape[1] != width) {
                                           std::stringstream ss;
                                           ss << "src and dest must have the same number of pixels, but got source with dimensions (" << height << ", " << width << ")";
                                           ss << "and RGB array with dimensions (" << bufdest.shape[0] << ", " << bufdest.shape[1] << ")";
                                           throw std::runtime_error(ss.str());
                                         }

                                         const DataType *bayer = static_cast<DataType *>(bufsrc.ptr);
                                         DataType *rgba = static_cast<DataType *>(bufdest.ptr);
                                         fn(bayer, rgba, height, width, num_threads);

  }, "Demosaic function implementation, see C++ documentation.", py::arg("bayer_data"), py::arg("rgba"), py::arg("num_threads") = 0);
}

}

void bindings_vpImageConvert(py::class_<vpImageConvert, std::shared_ptr<vpImageConvert>> &pyImageConvert)
{
  // Simple conversions where the size input is a single argument
  {
    std::vector<SimpleConversionStruct<ConversionFunction1D>> conversions = {
      SimpleConversionStruct("YUV444ToGrey", &vpImageConvert::YUV444ToGrey, 3, 1),
      SimpleConversionStruct("YUV444ToRGB", &vpImageConvert::YUV444ToRGB, 3, 3),
      SimpleConversionStruct("YUV444ToRGBa", &vpImageConvert::YUV444ToRGBa, 3, 4),
      SimpleConversionStruct("RGBToRGBa", static_cast<ConversionFunction1D>(&vpImageConvert::RGBToRGBa), 3, 4),
      SimpleConversionStruct("RGBaToRGB", &vpImageConvert::RGBaToRGB, 4, 3),
      SimpleConversionStruct("RGBaToGrey", static_cast<ConversionFunction1D>(&vpImageConvert::RGBaToGrey), 4, 1),
      SimpleConversionStruct("GreyToRGB", &vpImageConvert::GreyToRGB, 1, 3),
      SimpleConversionStruct("GreyToRGBa", static_cast<ConversionFunction1D>(&vpImageConvert::GreyToRGBa), 1, 4),
      SimpleConversionStruct("RGBToGrey", static_cast<ConversionFunction1D>(&vpImageConvert::RGBToGrey), 3, 1),
      SimpleConversionStruct("MONO16ToGrey", static_cast<ConversionFunction1D>(&vpImageConvert::MONO16ToGrey), 2, 1),
      SimpleConversionStruct("MONO16ToRGBa", static_cast<ConversionFunction1D>(&vpImageConvert::MONO16ToRGBa), 2, 4)

    };
    for (auto &conversion: conversions) {
      conversion.add_conversion_binding(pyImageConvert);
    }
  }

  // Simple conversions with flip
  {
    std::vector<SimpleConversionStruct<ConversionFunction2DWithFlip>> conversions = {
      SimpleConversionStruct("BGRToRGBa", static_cast<ConversionFunction2DWithFlip>(&vpImageConvert::BGRToRGBa), 3, 4),
      SimpleConversionStruct("BGRaToRGBa", static_cast<ConversionFunction2DWithFlip>(&vpImageConvert::BGRaToRGBa), 4, 4)
    };
    for (auto &conversion: conversions) {
      conversion.add_conversion_binding(pyImageConvert);
    }
  }

  // YUV conversions
  {
    using Conv = ConversionFromYUVLike<ConversionFunction2D>;
    std::vector<Conv> conversions = {
      Conv("YUYVToRGBa",    &vpImageConvert::YUYVToRGBa,    &size422, 4),
      Conv("YUYVToRGB",     &vpImageConvert::YUYVToRGB,     &size422, 3),
      Conv("YV12ToRGBa",    &vpImageConvert::YV12ToRGBa,    &size420, 4),
      Conv("YV12ToRGB",     &vpImageConvert::YV12ToRGB,     &size420, 3),
      Conv("YUV420ToRGBa",  &vpImageConvert::YUV420ToRGBa,  &size420, 4),
      Conv("YUV420ToRGB",   &vpImageConvert::YUV420ToRGB,   &size420, 3),
      Conv("YVU9ToRGBa",    &vpImageConvert::YVU9ToRGBa,    &size411, 4),
      Conv("YVU9ToRGB",     &vpImageConvert::YVU9ToRGB,     &size411, 3),
    };
    for (auto &conversion: conversions) {
      conversion.add_conversion_binding(pyImageConvert);
    }
  }
  {
    using Conv = ConversionFromYUVLike<ConversionFunction1D>;
    std::vector<Conv> conversions = {

      Conv("YUYVToGrey",    &vpImageConvert::YUYVToGrey,    &size422, 1),
      Conv("YUV422ToRGBa",  &vpImageConvert::YUV422ToRGBa,  &size422, 4),
      Conv("YUV422ToRGB",   &vpImageConvert::YUV422ToRGB,   &size422, 3),
      Conv("YUV422ToGrey",  &vpImageConvert::YUV422ToGrey,  &size422, 1),
      Conv("YCbCrToRGBa",   &vpImageConvert::YCbCrToRGBa,   &size422, 4),
      Conv("YCbCrToRGB",    &vpImageConvert::YCbCrToRGB,    &size422, 3),
      Conv("YCbCrToGrey",   &vpImageConvert::YCbCrToGrey,   &size422, 1),
      Conv("YCrCbToRGBa",   &vpImageConvert::YCrCbToRGBa,   &size422, 4),
      Conv("YCrCbToRGB",    &vpImageConvert::YCrCbToRGB,    &size422, 3),
      Conv("YUV420ToGrey",  &vpImageConvert::YUV420ToGrey,  &size420, 1),
      Conv("YUV411ToRGBa",  &vpImageConvert::YUV411ToRGBa,  &size411, 4),
      Conv("YUV411ToRGB",   &vpImageConvert::YUV411ToRGB,   &size411, 3),
      Conv("YUV411ToGrey",  &vpImageConvert::YUV411ToGrey,  &size411, 1),
    };
    for (auto &conversion: conversions) {
      conversion.add_conversion_binding(pyImageConvert);
    }
  }

  // HSV <-> RGB/a
  add_hsv_uchar_to_rgb_or_rgba_binding(pyImageConvert, vpImageConvert::HSVToRGB, "HSVToRGB", 3);
  add_hsv_double_to_rgb_or_rgba_binding(pyImageConvert, vpImageConvert::HSVToRGB, "HSVToRGB", 3);
  add_hsv_uchar_to_rgb_or_rgba_binding(pyImageConvert, vpImageConvert::HSVToRGBa, "HSVToRGBa", 4);
  add_hsv_double_to_rgb_or_rgba_binding(pyImageConvert, vpImageConvert::HSVToRGBa, "HSVToRGBa", 4);

  add_rgb_or_rgba_uchar_to_hsv_binding(pyImageConvert, vpImageConvert::RGBToHSV, "RGBToHSV", 3);
  add_rgb_or_rgba_double_to_hsv_binding(pyImageConvert, vpImageConvert::RGBToHSV, "RGBToHSV", 3);
  add_rgb_or_rgba_uchar_to_hsv_binding(pyImageConvert, vpImageConvert::RGBaToHSV, "RGBaToHSV", 4);
  add_rgb_or_rgba_double_to_hsv_binding(pyImageConvert, vpImageConvert::RGBaToHSV, "RGBaToHSV", 4);


  // uint8_t implems
  {
    using DemosaicFn = void (*)(const uint8_t *, uint8_t *, unsigned int, unsigned int, unsigned int);
    std::vector<std::pair<DemosaicFn, const char *>> functions = {
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicRGGBToRGBaMalvar), "demosaicRGGBToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGRBGToRGBaMalvar), "demosaicGRBGToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGBRGToRGBaMalvar), "demosaicGBRGToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicBGGRToRGBaMalvar), "demosaicBGGRToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicRGGBToRGBaBilinear), "demosaicRGGBToRGBaBilinear"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGRBGToRGBaBilinear), "demosaicGRBGToRGBaBilinear"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGBRGToRGBaBilinear), "demosaicGBRGToRGBaBilinear"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicBGGRToRGBaBilinear), "demosaicBGGRToRGBaBilinear"}
    };
    for (const auto &pair: functions) {
      add_demosaic_to_rgba_fn(pyImageConvert, pair.first, pair.second);
    }
  }
  //UInt16_t implems
  {
    using DemosaicFn = void (*)(const uint16_t *, uint16_t *, unsigned int, unsigned int, unsigned int);
    std::vector<std::pair<DemosaicFn, const char *>> functions = {
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicRGGBToRGBaMalvar), "demosaicRGGBToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGRBGToRGBaMalvar), "demosaicGRBGToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGBRGToRGBaMalvar), "demosaicGBRGToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicBGGRToRGBaMalvar), "demosaicBGGRToRGBaMalvar"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicRGGBToRGBaBilinear), "demosaicRGGBToRGBaBilinear"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGRBGToRGBaBilinear), "demosaicGRBGToRGBaBilinear"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicGBRGToRGBaBilinear), "demosaicGBRGToRGBaBilinear"},
      {static_cast<DemosaicFn>(&vpImageConvert::demosaicBGGRToRGBaBilinear), "demosaicBGGRToRGBaBilinear"}
    };
    for (const auto &pair: functions) {
      add_demosaic_to_rgba_fn(pyImageConvert, pair.first, pair.second);
    }
  }

}

#endif
