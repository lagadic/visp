/****************************************************************************
 *
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
*****************************************************************************/

//! \example tutorial-npz.cpp

#include <visp3/core/vpConfig.h>
#include <iostream>

// Check if cxx14 or higher
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L))) \
    && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_MINIZ)

#include <memory>
#include <complex>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  {
    //! [Save_string_init]
    const std::string save_string = "Open Source Visual Servoing Platform";
    std::vector<char> vec_save_string(save_string.begin(), save_string.end());
    //! [Save_string_init]

    //! [Save_string_save]
    const std::string npz_filename = "tutorial_npz_read_write.npz";
    const std::string identifier = "My string data";
    visp::cnpy::npz_save(npz_filename, identifier, &vec_save_string[0], { vec_save_string.size() }, "w");
    //! [Save_string_save]
  }
  {
    //! [Read_string_load]
    const std::string npz_filename = "tutorial_npz_read_write.npz";
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    //! [Read_string_load]
    //! [Read_string]
    const std::string identifier = "My string data";
    if (npz_data.find(identifier) != npz_data.end()) {
      visp::cnpy::NpyArray arr_string_data = npz_data[identifier];
      std::vector<char> vec_arr_string_data = arr_string_data.as_vec<char>();
      const std::string read_string = std::string(vec_arr_string_data.begin(), vec_arr_string_data.end());
      std::cout << "Read string: " << read_string << std::endl;
    }
    //! [Read_string]
  }

  {
    //! [Save_basic_types]
    const std::string npz_filename = "tutorial_npz_read_write.npz";
    const std::string int_identifier = "My int data";
    int int_data = 99;
    visp::cnpy::npz_save(npz_filename, int_identifier, &int_data, { 1 }, "w");

    const std::string double_identifier = "My double data";
    double double_data = 3.14;
    visp::cnpy::npz_save(npz_filename, double_identifier, &double_data, { 1 }, "a");

    const std::string complex_identifier = "My complex data";
    std::complex<double> complex_data(int_data, double_data);
    visp::cnpy::npz_save(npz_filename, complex_identifier, &complex_data, { 1 }, "a");
    //! [Save_basic_types]
  }
  {
    //! [Read_basic_types]
    const std::string npz_filename = "tutorial_npz_read_write.npz";
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    const std::string int_identifier = "My int data";
    const std::string double_identifier = "My double data";
    const std::string complex_identifier = "My complex data";

    visp::cnpy::npz_t::iterator it_int = npz_data.find(int_identifier);
    visp::cnpy::npz_t::iterator it_double = npz_data.find(double_identifier);
    visp::cnpy::npz_t::iterator it_complex = npz_data.find(complex_identifier);
    if (it_int != npz_data.end() && it_double != npz_data.end() && it_complex != npz_data.end()) {
      visp::cnpy::NpyArray arr_data_int = it_int->second;
      visp::cnpy::NpyArray arr_data_double = it_double->second;
      visp::cnpy::NpyArray arr_data_complex = it_complex->second;

      int int_data = *arr_data_int.data<int>();
      double double_data = *arr_data_double.data<double>();
      std::complex<double> complex_data = *arr_data_complex.data<std::complex<double>>();
      std::cout << "Read int data: " << int_data << std::endl;
      std::cout << "Read double data: " << double_data << std::endl;
      std::cout << "Read complex data, real: " << complex_data.real() << " ; imag: " << complex_data.imag() << std::endl;
    }
    //! [Read_basic_types]
  }

  {
    //! [Save_image_read]
    vpImage<vpRGBa> img;
    const std::string img_filename = "ballons.jpg";
    vpImageIo::read(img, img_filename);
    //! [Save_image_read]
    //! [Save_image]
    if (img.getSize() != 0) {
      const std::string npz_filename = "tutorial_npz_read_write.npz";
      const std::string img_identifier = "My color image";
      visp::cnpy::npz_save(npz_filename, img_identifier, &img.bitmap[0], { img.getRows(), img.getCols() }, "w");
    }
    //! [Save_image]
  }
  {
    //! [Read_image]
    const std::string npz_filename = "tutorial_npz_read_write.npz";
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    const std::string img_identifier = "My color image";
    visp::cnpy::npz_t::iterator it_img = npz_data.find(img_identifier);

    if (it_img != npz_data.end()) {
      visp::cnpy::NpyArray arr_data_img = it_img->second;
      const bool copy_data = false;
      vpImage<vpRGBa> img(arr_data_img.data<vpRGBa>(), arr_data_img.shape[0], arr_data_img.shape[1], copy_data);
      std::cout << "Img: " << img.getWidth() << "x" << img.getHeight() << std::endl;

      std::unique_ptr<vpDisplay> ptr_display;
#if defined(VISP_HAVE_X11)
      ptr_display = std::make_unique<vpDisplayX>(img);
#elif defined(VISP_HAVE_GDI)
      ptr_display = std::make_unique<vpDisplayGDI>(img);
#endif

      vpDisplay::display(img);
      vpDisplay::displayText(img, 20, 20, "vpImage<vpRGBa>", vpColor::red);
      vpDisplay::flush(img);
      vpDisplay::getClick(img);
    }
    //! [Read_image]
  }

  {
    //! [Save_multi_array]
    vpImage<vpRGBa> img;
    const std::string img_filename = "ballons.jpg";
    vpImageIo::read(img, img_filename);
    if (img.getSize() != 0) {
      std::vector<unsigned char> vec_data_img;
      vec_data_img.resize(3*img.getSize());
      vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(img.bitmap), vec_data_img.data(),
        img.getSize());

      const std::string npz_filename = "tutorial_npz_read_write.npz";
      const std::string img_identifier = "My RGB image";
      visp::cnpy::npz_save(npz_filename, img_identifier, &vec_data_img[0], { img.getRows(), img.getCols(), 3 }, "w");
    }
    //! [Save_multi_array]
  }
  {
    //! [Read_multi_array]
    const std::string npz_filename = "tutorial_npz_read_write.npz";
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    const std::string img_identifier = "My RGB image";
    visp::cnpy::npz_t::iterator it_img = npz_data.find(img_identifier);

    if (it_img != npz_data.end()) {
      visp::cnpy::NpyArray arr_data_img = it_img->second;
      vpImage<vpRGBa> img(arr_data_img.shape[0], arr_data_img.shape[1]);
      vpImageConvert::RGBToRGBa(arr_data_img.data<unsigned char>(), reinterpret_cast<unsigned char *>(img.bitmap),
        img.getSize());

      std::unique_ptr<vpDisplay> ptr_display;
#if defined(VISP_HAVE_X11)
      ptr_display = std::make_unique<vpDisplayX>(img);
#elif defined(VISP_HAVE_GDI)
      ptr_display = std::make_unique<vpDisplayGDI>(img);
#endif

      vpDisplay::display(img);
      vpDisplay::displayText(img, 20, 20, "RGBToRGBa", vpColor::red);
      vpDisplay::flush(img);
      vpDisplay::getClick(img);
    }
    //! [Read_multi_array]
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cerr << "This tutorial requires C++ version >= C++14." << std::endl;
  std::cerr << "This tutorial requires display (X11 or GDI) capability." << std::endl;
#ifndef VISP_HAVE_MINIZ
  std::cerr << "This tutorial requires having enabled npz I/O functions." << std::endl;
#endif
  return EXIT_FAILURE;
}
#endif
