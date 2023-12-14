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
 * Python bindings.
 */

#ifndef VISP_PYTHON_CORE_IMAGE_CONVERT_HPP
#define VISP_PYTHON_CORE_IMAGE_CONVERT_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <visp3/core/vpImageConvert.h>

namespace
{
using ConversionFunction1D = void(*)(unsigned char *, unsigned char *, unsigned int);
using ConversionFunction2D = void(*)(unsigned char *, unsigned char *, unsigned int, unsigned int);
using ComputeBytesFunction = unsigned(*)(unsigned int, unsigned int);

void call_conversion_fn(ConversionFunction2D fn, unsigned char *src, unsigned char *dest, unsigned int h, unsigned int w)
{
  fn(src, dest, h, w);
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

  void add_conversion_binding(py::class_<vpImageConvert> &pyImageConvert)
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
    }, py::arg("src"), py::arg("dest"));
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

  void add_conversion_binding(py::class_<vpImageConvert> &pyImageConvert)
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

}



void bindings_vpImageConvert(py::class_<vpImageConvert> &pyImageConvert)
{
  // Simple conversions where the size input is a single argument
  {
    std::vector<SimpleConversionStruct<ConversionFunction1D>> conversions = {
      SimpleConversionStruct("YUV444ToGrey", &vpImageConvert::YUV444ToGrey, 3, 1),
      SimpleConversionStruct("YUV444ToRGB", &vpImageConvert::YUV444ToRGB, 3, 3),
      SimpleConversionStruct("YUV444ToRGBa", &vpImageConvert::YUV444ToRGBa, 3, 4),
      SimpleConversionStruct("RGBToRGBa", static_cast<ConversionFunction1D>(&vpImageConvert::RGBToRGBa), 3, 4),
      SimpleConversionStruct("RGBaToRGB", &vpImageConvert::RGBaToRGB, 4, 3),
      SimpleConversionStruct("GreyToRGB", &vpImageConvert::GreyToRGB, 1, 3),
      SimpleConversionStruct("GreyToRGBa", static_cast<ConversionFunction1D>(&vpImageConvert::GreyToRGBa), 1, 4),
      SimpleConversionStruct("RGBToGrey", static_cast<ConversionFunction1D>(&vpImageConvert::RGBToGrey), 3, 1),
    };
    for (auto &conversion: conversions) {
      conversion.add_conversion_binding(pyImageConvert);
    }
  }

  //YUV conversions
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
}

#endif
