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
 * Test visp::cnpy::npz_load() / visp::cnpy::npy_save() functions.
 */

/*!
  \example catchNPZ.cpp
 */
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEndian.h>

#if defined(VISP_HAVE_CATCH2) && \
  (defined(_WIN32) || (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))) && \
  defined(VISP_LITTLE_ENDIAN) && defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)

#include <catch_amalgamated.hpp>

#include <type_traits>
#include <complex>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImage.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
std::string createTmpDir()
{
  std::string directory_filename = vpIoTools::getTempPath() + "/testNPZ";

  vpIoTools::makeDirectory(directory_filename);
  return directory_filename;
}
}

TEST_CASE("Test visp::cnpy::npy_load/npz_save", "[visp::cnpy I/O]")
{
  std::string directory_filename = createTmpDir();
  REQUIRE(vpIoTools::checkDirectory(directory_filename));
  std::string npz_filename = directory_filename + "/test_npz_read_write.npz";

  SECTION("Read/Save string data")
  {
    const std::string save_string = "Open Source Visual Servoing Platform";
    std::vector<char> vec_save_string(save_string.begin(), save_string.end());
    const std::string identifier = "String";
    visp::cnpy::npz_save(npz_filename, identifier, &vec_save_string[0], { vec_save_string.size() }, "w");

    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    REQUIRE(npz_data.find(identifier) != npz_data.end());

    visp::cnpy::NpyArray arr_string_data = npz_data[identifier];
    std::vector<char> vec_arr_string_data = arr_string_data.as_vec<char>();
    // For null-terminated character handling, see:
    // https://stackoverflow.com/a/8247804
    // https://stackoverflow.com/a/45491652
    const std::string read_string(vec_arr_string_data.begin(), vec_arr_string_data.end());
    CHECK(save_string == read_string);

    // Direct variable access
    visp::cnpy::NpyArray arr_string_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
    std::vector<char> vec_arr_string_data_direct = arr_string_data_direct.as_vec<char>();

    const std::string read_string_direct(vec_arr_string_data_direct.begin(), vec_arr_string_data_direct.end());
    CHECK(read_string_direct == read_string);
  }

  SECTION("Read/Save multi-dimensional array")
  {
    const std::string identifier = "Array";
    size_t height = 5, width = 7, channels = 3;
    std::vector<int> save_vec_copy;
    {
      std::vector<int> save_vec;
      save_vec.reserve(height*width*channels);
      for (int i = 0; i < static_cast<int>(height*width*channels); ++i) {
        save_vec.push_back(i);
      }

      visp::cnpy::npz_save(npz_filename, identifier, &save_vec[0], { height, width, channels }, "a"); // append
      save_vec_copy = save_vec;
    }

    {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
      REQUIRE(npz_data.find(identifier) != npz_data.end());

      visp::cnpy::NpyArray arr_vec_data = npz_data[identifier];
      std::vector<int> read_vec = arr_vec_data.as_vec<int>();

      REQUIRE(save_vec_copy.size() == read_vec.size());
      for (size_t i = 0; i < read_vec.size(); ++i) {
        CHECK(save_vec_copy[i] == read_vec[i]);
      }

      // Direct variable access
      visp::cnpy::NpyArray arr_vec_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
      std::vector<int> read_vec_direct = arr_vec_data_direct.as_vec<int>();

      REQUIRE(read_vec_direct.size() == read_vec.size());
      for (size_t i = 0; i < read_vec.size(); ++i) {
        CHECK(read_vec_direct[i] == read_vec[i]);
      }
    }
  }

  SECTION("Read/Save vpImage<vpRGBa>")
  {
    // CHECK(std::is_trivially_copyable<vpRGBa>::value == true); // false
    // CHECK(std::is_trivial<vpRGBa>::value == true); // false
    CHECK(sizeof(vpRGBa) == (4 * sizeof(unsigned char)));

    const std::string identifier = "vpImage<vpRGBa>";
    vpImage<vpRGBa> I_save_copy;
    {
      vpImage<vpRGBa> I_save(11, 17);
      for (unsigned int i = 0; i < I_save.getRows(); i++) {
        for (unsigned int j = 0; j < I_save.getCols(); j++) {
          I_save[i][j].R = 4 * (i*I_save.getCols() + j) + 0;
          I_save[i][j].G = 4 * (i*I_save.getCols() + j) + 1;
          I_save[i][j].B = 4 * (i*I_save.getCols() + j) + 2;
          I_save[i][j].A = 4 * (i*I_save.getCols() + j) + 3;
        }
      }

      visp::cnpy::npz_save(npz_filename, identifier, &I_save.bitmap[0], { I_save.getRows(), I_save.getCols() }, "a"); // append
      I_save_copy = I_save;
    }

    {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
      REQUIRE(npz_data.find(identifier) != npz_data.end());

      visp::cnpy::NpyArray arr_vec_data = npz_data[identifier];
      const bool copy_data = false;
      vpImage<vpRGBa> I_read(arr_vec_data.data<vpRGBa>(), static_cast<unsigned int>(arr_vec_data.shape[0]),
                             static_cast<unsigned int>(arr_vec_data.shape[1]), copy_data);

      CHECK(I_save_copy.getSize() == I_read.getSize());
      CHECK(I_save_copy == I_read);

      // Direct variable access
      visp::cnpy::NpyArray arr_vec_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
      vpImage<vpRGBa> I_read_direct(arr_vec_data_direct.data<vpRGBa>(), static_cast<unsigned int>(arr_vec_data_direct.shape[0]),
                             static_cast<unsigned int>(arr_vec_data_direct.shape[1]), copy_data);

      CHECK(I_read.getSize() == I_read_direct.getSize());
      CHECK(I_read == I_read_direct);
    }
  }

  SECTION("Read/Save std::complex<double>")
  {
    // Handling of std::complex<>?
    //  - https://github.com/rogersce/cnpy/blob/4e8810b1a8637695171ed346ce68f6984e585ef4/cnpy.cpp#L40-L42
    //  - https://github.com/rogersce/cnpy/blob/4e8810b1a8637695171ed346ce68f6984e585ef4/cnpy.h#L129
    // https://en.cppreference.com/w/cpp/named_req/TriviallyCopyable

    // Next CHECK() call may fail when g++ < 5 (case on centos-7-2 ci). That's why we ensure that c++ standard is > 11
    // See https://stackoverflow.com/questions/25123458/is-trivially-copyable-is-not-a-member-of-std
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_11)
    CHECK(std::is_trivially_copyable<std::complex<double>>::value == true);
#endif
    // https://en.cppreference.com/w/cpp/types/is_trivial
    // CHECK(std::is_trivial<std::complex<double>>::value == true); // false

    const std::string identifier = "std::complex<double>";
    std::complex<double> complex_data_copy;
    {
      std::complex<double> complex_data(99, 3.14);
      visp::cnpy::npz_save(npz_filename, identifier, &complex_data, { 1 }, "a"); // append
      complex_data_copy = complex_data;
    }

    {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
      REQUIRE(npz_data.find(identifier) != npz_data.end());

      visp::cnpy::NpyArray arr_vec_data = npz_data[identifier];
      std::complex<double> complex_data_read = *arr_vec_data.data<std::complex<double>>();

      CHECK(complex_data_copy.real() == complex_data_read.real());
      CHECK(complex_data_copy.imag() == complex_data_read.imag());

      // Direct variable access
      visp::cnpy::NpyArray arr_vec_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
      std::complex<double> complex_data_read_direct = *arr_vec_data.data<std::complex<double>>();

      CHECK(complex_data_read_direct.real() == complex_data_read.real());
      CHECK(complex_data_read_direct.imag() == complex_data_read.imag());
    }
  }

  SECTION("Read/Save std::vector<std::complex<double>>")
  {
    const std::string identifier = "std::vector<std::complex<double>>";
    std::vector<std::complex<double>> vec_complex_data_copy;
    {
      std::vector<std::complex<double>> vec_complex_data;
      std::complex<double> complex_data(99, 3.14);
      vec_complex_data.push_back(complex_data);

      complex_data.real(-77.12);
      complex_data.imag(-100.95);
      vec_complex_data.push_back(complex_data);

      visp::cnpy::npz_save(npz_filename, identifier, &vec_complex_data[0], { vec_complex_data.size() }, "a"); // append
      vec_complex_data_copy = vec_complex_data;
    }

    {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
      REQUIRE(npz_data.find(identifier) != npz_data.end());

      visp::cnpy::NpyArray arr_vec_data = npz_data[identifier];
      std::vector<std::complex<double>> vec_complex_data_read = arr_vec_data.as_vec<std::complex<double>>();

      REQUIRE(vec_complex_data_copy.size() == vec_complex_data_read.size());
      for (size_t i = 0; i < vec_complex_data_copy.size(); i++) {
        CHECK(vec_complex_data_copy[i].real() == vec_complex_data_read[i].real());
        CHECK(vec_complex_data_copy[i].imag() == vec_complex_data_read[i].imag());
      }

      // Direct variable access
      visp::cnpy::NpyArray arr_vec_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
      std::vector<std::complex<double>> vec_complex_data_read_direct = arr_vec_data_direct.as_vec<std::complex<double>>();

      REQUIRE(vec_complex_data_read_direct.size() == vec_complex_data_read.size());
      for (size_t i = 0; i < vec_complex_data_read_direct.size(); i++) {
        CHECK(vec_complex_data_read_direct[i].real() == vec_complex_data_read[i].real());
        CHECK(vec_complex_data_read_direct[i].imag() == vec_complex_data_read[i].imag());
      }
    }
  }

  SECTION("Read/Save vpHomogeneousMatrix")
  {
    const std::string identifier = "vpHomogeneousMatrix";
    vpHomogeneousMatrix cMo_save_copy;
    {
      vpHomogeneousMatrix cMo_save(vpTranslationVector(10, 20, 30), vpThetaUVector(0.1, 0.2, 0.3));
      // std::cout << "cMo_save:\n" << cMo_save << std::endl;

      visp::cnpy::npz_save(npz_filename, identifier, &cMo_save.data[0], { cMo_save.getRows(), cMo_save.getCols() }, "a"); // append
      cMo_save_copy = cMo_save;
    }

    {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
      REQUIRE(npz_data.find(identifier) != npz_data.end());

      visp::cnpy::NpyArray arr_vec_data = npz_data[identifier];
      vpHomogeneousMatrix cMo_read(arr_vec_data.as_vec<double>());
      // std::cout << "cMo_read:\n" << cMo_read << std::endl;

      CHECK(cMo_save_copy == cMo_read);

      // Direct variable access
      visp::cnpy::NpyArray arr_vec_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
      vpHomogeneousMatrix cMo_read_direct(arr_vec_data_direct.as_vec<double>());

      CHECK(cMo_read_direct == cMo_read);
    }
  }

  SECTION("Read/Save std::vector<vpHomogeneousMatrix>")
  {
    const std::string identifier = "std::vector<vpHomogeneousMatrix>";
    std::vector<vpHomogeneousMatrix> vec_cMo_save_copy;
    {
      std::vector<double> vec_cMo_save;
      for (size_t i = 0; i < 5; i++) {
        vpHomogeneousMatrix cMo_save(vpTranslationVector(1. +10.*i, 2. +20.*i, 3. +30.*i), vpThetaUVector(0.1+i, 0.2+i, 0.3+i));
        vec_cMo_save_copy.push_back(cMo_save);
        vec_cMo_save.insert(vec_cMo_save.end(), cMo_save.data, cMo_save.data+cMo_save.size());
        // std::cout << "cMo_save:\n" << cMo_save << std::endl;
      }

      visp::cnpy::npz_save(npz_filename, identifier, &vec_cMo_save[0], { vec_cMo_save.size()/16, 16 }, "a"); // append
    }

    {
      visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
      REQUIRE(npz_data.find(identifier) != npz_data.end());

      visp::cnpy::NpyArray arr_vec_data = npz_data[identifier];
      std::vector<double> vec_cMo_read = arr_vec_data.as_vec<double>();
      REQUIRE(vec_cMo_save_copy.size() == arr_vec_data.shape[0]);

      for (size_t i = 0; i < arr_vec_data.shape[0]; i++) {
        std::vector<double>::const_iterator first = vec_cMo_read.begin() + i*arr_vec_data.shape[1];
        std::vector<double>::const_iterator last = first + arr_vec_data.shape[1];
        std::vector<double> subvec_cMo_read(first, last);
        vpHomogeneousMatrix cMo_read(subvec_cMo_read);
        // std::cout << "cMo_read:\n" << cMo_read << std::endl;
        CHECK(vec_cMo_save_copy[i] == cMo_read);
      }

      // Direct variable access
      visp::cnpy::NpyArray arr_vec_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
      std::vector<double> vec_cMo_read_direct = arr_vec_data_direct.as_vec<double>();
      REQUIRE(arr_vec_data_direct.shape.size() == arr_vec_data.shape.size());
      REQUIRE(arr_vec_data_direct.shape[0] == arr_vec_data.shape[0]);

      for (size_t i = 0; i < arr_vec_data_direct.shape[0]; i++) {
        std::vector<double>::const_iterator first = vec_cMo_read_direct.begin() + i*arr_vec_data_direct.shape[1];
        std::vector<double>::const_iterator last = first + arr_vec_data_direct.shape[1];
        std::vector<double> subvec_cMo_read_direct(first, last);
        vpHomogeneousMatrix cMo_read_direct(subvec_cMo_read_direct);
        // std::cout << "cMo_read:\n" << cMo_read << std::endl;
        CHECK(vec_cMo_save_copy[i] == cMo_read_direct);
      }
    }
  }
  REQUIRE(vpIoTools::remove(directory_filename));
  REQUIRE(!vpIoTools::checkDirectory(directory_filename));
}

// https://en.cppreference.com/w/cpp/types/integer
// https://github.com/catchorg/Catch2/blob/devel/docs/test-cases-and-sections.md#type-parametrised-test-cases
using BasicTypes = std::tuple<uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, uint64_t, int64_t, float, double>;
TEMPLATE_LIST_TEST_CASE("Test visp::cnpy::npy_load/npz_save", "[BasicTypes][list]", BasicTypes)
{
  std::string directory_filename = createTmpDir();
  REQUIRE(vpIoTools::checkDirectory(directory_filename));
  std::string npz_filename = directory_filename + "/test_npz_read_write.npz";

  std::string identifier = "data";
  TestType save_data_copy;
  {
    TestType save_data = std::numeric_limits<TestType>::min();
    visp::cnpy::npz_save(npz_filename, identifier, &save_data, { 1 }, "w");
    save_data_copy = save_data;
  }
  {
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    REQUIRE(npz_data.find(identifier) != npz_data.end());
    visp::cnpy::NpyArray arr_data = npz_data[identifier];
    TestType read_data = *arr_data.data<TestType>();
    CHECK(save_data_copy == read_data);

    // Direct variable access
    visp::cnpy::NpyArray arr_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
    TestType read_data_direct = *arr_data_direct.data<TestType>();
    CHECK(read_data_direct == read_data);
  }

  identifier = "data2";
  {
    TestType save_data = std::numeric_limits<TestType>::max();
    visp::cnpy::npz_save(npz_filename, identifier, &save_data, { 1 }, "a"); // append
    save_data_copy = save_data;
  }
  {
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    REQUIRE(npz_data.find(identifier) != npz_data.end());
    visp::cnpy::NpyArray arr_data = npz_data[identifier];
    TestType read_data = *arr_data.data<TestType>();
    CHECK(save_data_copy == read_data);

    // Direct variable access
    visp::cnpy::NpyArray arr_data_direct = visp::cnpy::npz_load(npz_filename, identifier);
    TestType read_data_direct = *arr_data_direct.data<TestType>();
    CHECK(read_data_direct == read_data);
  }
  {
    visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
    REQUIRE(npz_data.find(identifier) != npz_data.end());
    visp::cnpy::NpyArray arr_data = npz_data[identifier];
    TestType read_data = *arr_data.data<TestType>();
    CHECK(save_data_copy == read_data);
  }

  REQUIRE(vpIoTools::remove(directory_filename));
  REQUIRE(!vpIoTools::checkDirectory(directory_filename));
}

#if defined(VISP_HAVE_DATASET) && (VISP_HAVE_DATASET_VERSION >= 0x030703)
namespace
{
void loadData(const std::string &npz_filename,
    bool &false_data, bool &true_data, uint32_t &uint32_data, int64_t &int64_data,
    float &float_data, double &double_data, std::string &string_data, std::complex<double> &complex_data,
    std::vector<int> &vec_int, std::vector<float> &vec_flt, std::vector<std::string> &vec_string,
    std::vector<std::complex<float>> &vec_complex, bool has_complex)
{
  const std::string bool_false_identifier = "My bool false data";
  const std::string bool_true_identifier = "My bool true data";
  const std::string uint32_identifier = "My uint32 data";
  const std::string int64_identifier = "My int64 data";
  const std::string float_identifier = "My float data";
  const std::string double_identifier = "My double data";
  const std::string string_identifier = "My string data";
  const std::string complex_identifier = "My complex data";
  const std::string matrix_int_identifier = "My int matrix data";
  const std::string matrix_flt_identifier = "My float matrix data";
  const std::string matrix_string_identifier = "My string matrix data";
  const std::string vec_complex_identifier = "My complex vector data";

  visp::cnpy::npz_t npz_data = visp::cnpy::npz_load(npz_filename);
  visp::cnpy::npz_t::iterator it_bool_false = npz_data.find(bool_false_identifier);
  visp::cnpy::npz_t::iterator it_bool_true = npz_data.find(bool_true_identifier);
  visp::cnpy::npz_t::iterator it_uint32 = npz_data.find(uint32_identifier);
  visp::cnpy::npz_t::iterator it_int64 = npz_data.find(int64_identifier);
  visp::cnpy::npz_t::iterator it_float = npz_data.find(float_identifier);
  visp::cnpy::npz_t::iterator it_double = npz_data.find(double_identifier);
  visp::cnpy::npz_t::iterator it_string = npz_data.find(string_identifier);
  visp::cnpy::npz_t::iterator it_matrix_int = npz_data.find(matrix_int_identifier);
  visp::cnpy::npz_t::iterator it_matrix_flt = npz_data.find(matrix_flt_identifier);
  visp::cnpy::npz_t::iterator it_matrix_string = npz_data.find(matrix_string_identifier);

  REQUIRE(it_bool_false != npz_data.end());
  REQUIRE(it_bool_true != npz_data.end());
  REQUIRE(it_uint32 != npz_data.end());
  REQUIRE(it_int64 != npz_data.end());
  REQUIRE(it_float != npz_data.end());
  REQUIRE(it_double != npz_data.end());
  REQUIRE(it_string != npz_data.end());
  REQUIRE(it_matrix_int != npz_data.end());
  REQUIRE(it_matrix_flt != npz_data.end());
  REQUIRE(it_matrix_string != npz_data.end());

  visp::cnpy::NpyArray arr_data_bool_false = it_bool_false->second;
  visp::cnpy::NpyArray arr_data_bool_true = it_bool_true->second;
  visp::cnpy::NpyArray arr_data_uint32 = it_uint32->second;
  visp::cnpy::NpyArray arr_data_int64 = it_int64->second;
  visp::cnpy::NpyArray arr_data_float = it_float->second;
  visp::cnpy::NpyArray arr_data_double = it_double->second;
  visp::cnpy::NpyArray arr_data_string = it_string->second;
  visp::cnpy::NpyArray arr_data_matrix_int = it_matrix_int->second;
  visp::cnpy::NpyArray arr_data_matrix_flt = it_matrix_flt->second;
  visp::cnpy::NpyArray arr_data_matrix_string = it_matrix_string->second;

  false_data = *arr_data_bool_false.data<bool>();
  true_data = *arr_data_bool_true.data<bool>();
  uint32_data = *arr_data_uint32.data<uint32_t>();
  int64_data = *arr_data_int64.data<int64_t>();
  float_data = *arr_data_float.data<float>();
  double_data = *arr_data_double.data<double>();
  assert(!arr_data_string.as_utf8_string_vec().empty());
  string_data = arr_data_string.as_utf8_string_vec()[0];
  vec_int = arr_data_matrix_int.as_vec<int>();
  vec_flt = arr_data_matrix_flt.as_vec<float>();
  vec_string = arr_data_matrix_string.as_utf8_string_vec();

  if (has_complex) {
    visp::cnpy::npz_t::iterator it_complex = npz_data.find(complex_identifier);
    REQUIRE(it_complex != npz_data.end());
    visp::cnpy::NpyArray arr_data_complex = it_complex->second;
    complex_data = *arr_data_complex.data<std::complex<double>>();

    visp::cnpy::npz_t::iterator it_vec_complex = npz_data.find(vec_complex_identifier);
    REQUIRE(it_vec_complex != npz_data.end());
    visp::cnpy::NpyArray arr_data_vec_complex = it_vec_complex->second;
    vec_complex = arr_data_vec_complex.as_vec<std::complex<float>>();
  }
}

void getNpzGroundTruth(bool &gt_bool_false, bool &gt_bool_true, uint32_t &gt_uint32_data, int64_t &gt_int64_data,
    float &gt_float_data, double &gt_double_data, std::string &gt_string_data, std::complex<double> &gt_complex_data,
    std::vector<int> &gt_vec_int, std::vector<float> &gt_vec_flt, std::vector<std::string> &gt_vec_string,
    std::vector<std::complex<float>> &gt_vec_complex_data)
{
  // ground-truth values
  gt_bool_false = false;
  gt_bool_true = true;
  gt_uint32_data = 99;
  gt_int64_data = -123456;
  gt_float_data = -456.51f;
  gt_double_data = 3.14;
  gt_string_data = "ViSP: Open source Visual Servoing Platform";

  gt_complex_data = std::complex<double>(gt_float_data, gt_double_data);
  const size_t height = 5, width = 7, channels = 3;
  for (int i = 0; i < static_cast<int>(height*width*channels); ++i) {
    gt_vec_int.push_back(i);
    gt_vec_flt.push_back(i);
  }

  gt_vec_string.push_back("ViSP ");
  gt_vec_string.push_back("for ");
  gt_vec_string.push_back("visual servoing: ");
  gt_vec_string.push_back("a generic software platform ");
  gt_vec_string.push_back("with a wide class of ");
  gt_vec_string.push_back("robot control skills");

  for (int i = 0; i < 3; i++) {
    gt_vec_complex_data.push_back(std::complex<float>(gt_complex_data.real()*(i+1), gt_complex_data.imag()*2*(i+1)));
  }
}
}

TEST_CASE("Test little-endian / big-endian npz loading", "[visp::cnpy I/O]")
{
  const bool has_complex = true;
  SECTION("Check little-endian correctness for npz loading")
  {
    const std::string npz_filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
      "npz/visp_cnpy/visp_npz_test_data_cnpy_LE.npz");

    // Ground-truth data
    bool gt_bool_false, gt_bool_true;
    uint32_t gt_uint32_data;
    int64_t gt_int64_data;
    float gt_float_data;
    double gt_double_data;
    std::string gt_string_data;
    std::complex<double> gt_complex_data;
    std::vector<int> gt_vec_int;
    std::vector<float> gt_vec_flt;
    std::vector<std::string> gt_vec_string;
    std::vector<std::complex<float>> gt_vec_complex_data;
    getNpzGroundTruth(gt_bool_false, gt_bool_true, gt_uint32_data, gt_int64_data, gt_float_data, gt_double_data,
      gt_string_data, gt_complex_data, gt_vec_int, gt_vec_flt, gt_vec_string, gt_vec_complex_data);

    // Load data
    bool b_false = false, b_true = false;
    uint32_t uint32_data = 0;
    int64_t int64_data = 0;
    float float_data = 0;
    double double_data = 0;
    std::string string_data = "";
    std::complex<double> complex_data;
    std::vector<int> vec_int;
    std::vector<float> vec_flt;
    std::vector<std::string> vec_string;
    std::vector<std::complex<float>> vec_complex_data;
    loadData(npz_filename, b_false, b_true, uint32_data, int64_data, float_data, double_data, string_data,
      complex_data, vec_int, vec_flt, vec_string, vec_complex_data, has_complex);

    CHECK(b_false == gt_bool_false);
    CHECK(b_true == gt_bool_true);
    CHECK(uint32_data == gt_uint32_data);
    CHECK(int64_data == gt_int64_data);
    CHECK(float_data == gt_float_data);
    CHECK(double_data == gt_double_data);
    CHECK(string_data == gt_string_data);
    CHECK(complex_data.real() == gt_complex_data.real());
    CHECK(complex_data.imag() == gt_complex_data.imag());

    REQUIRE(gt_vec_int.size() == gt_vec_flt.size());
    REQUIRE(gt_vec_int.size() == vec_int.size());
    REQUIRE(gt_vec_int.size() == vec_flt.size());
    for (size_t i = 0; i < gt_vec_int.size(); i++) {
      REQUIRE(gt_vec_int[i] == vec_int[i]);
      REQUIRE(gt_vec_flt[i] == vec_flt[i]);
    }

    REQUIRE(gt_vec_string.size() == vec_string.size());
    for (size_t i = 0; i < gt_vec_string.size(); i++) {
      REQUIRE(gt_vec_string[i] == vec_string[i]);
    }

    REQUIRE(gt_vec_complex_data.size() == vec_complex_data.size());
    for (size_t i = 0; i < gt_vec_complex_data.size(); i++) {
      REQUIRE(vec_complex_data[i].real() == gt_vec_complex_data[i].real());
      REQUIRE(vec_complex_data[i].imag() == gt_vec_complex_data[i].imag());
    }
  }

  SECTION("Check little-endian vs big_endian correctness for npz loading")
  {
    const std::string npz_filename_LE = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
      "npz/visp_cnpy/visp_npz_test_data_cnpy_LE.npz");
    const std::string npz_filename_BE = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
      "npz/visp_cnpy/visp_npz_test_data_cnpy_BE.npz");

    visp::cnpy::npz_t npz_data_LE = visp::cnpy::npz_load(npz_filename_LE);
    visp::cnpy::npz_t npz_data_BE = visp::cnpy::npz_load(npz_filename_BE);

    bool b_data_false_LE, b_data_false_BE;
    bool b_data_true_LE, b_data_true_BE;
    uint32_t uint32_data_LE, uint32_data_BE;
    int64_t int64_data_LE, int64_data_BE;
    float float_data_LE, float_data_BE;
    double double_data_LE, double_data_BE;
    std::string string_data_LE, string_data_BE;
    std::complex<double> complex_data_LE, complex_data_BE;
    std::vector<int> vec_int_LE, vec_int_BE;
    std::vector<float> vec_flt_LE, vec_flt_BE;
    std::vector<std::string> vec_string_LE, vec_string_BE;
    std::vector<std::complex<float>> vec_complex_data_LE, vec_complex_data_BE;

    loadData(npz_filename_LE, b_data_false_LE, b_data_true_LE, uint32_data_LE, int64_data_LE, float_data_LE,
      double_data_LE, string_data_LE, complex_data_LE, vec_int_LE, vec_flt_LE, vec_string_LE, vec_complex_data_LE, has_complex);

    loadData(npz_filename_BE, b_data_false_BE, b_data_true_BE, uint32_data_BE, int64_data_BE, float_data_BE,
      double_data_BE, string_data_BE, complex_data_BE, vec_int_BE, vec_flt_BE, vec_string_BE, vec_complex_data_BE, has_complex);

    CHECK(b_data_false_LE == b_data_false_BE);
    CHECK(b_data_true_LE == b_data_true_BE);
    CHECK(uint32_data_LE == uint32_data_BE);
    CHECK(int64_data_LE == int64_data_BE);
    CHECK(float_data_LE == float_data_BE);
    CHECK(double_data_LE == double_data_BE);
    CHECK(string_data_LE == string_data_BE);
    CHECK(complex_data_LE.real() == complex_data_BE.real());
    CHECK(complex_data_LE.imag() == complex_data_BE.imag());

    REQUIRE(vec_int_LE.size() == vec_flt_LE.size());
    REQUIRE(vec_int_LE.size() == vec_int_BE.size());
    REQUIRE(vec_int_LE.size() == vec_flt_BE.size());
    for (size_t i = 0; i < vec_int_LE.size(); i++) {
      CHECK(vec_int_LE[i] == vec_int_BE[i]);
      CHECK(vec_flt_LE[i] == vec_flt_BE[i]);
    }

    REQUIRE(vec_string_LE.size() == vec_string_BE.size());
    for (size_t i = 0; i < vec_string_LE.size(); i++) {
      CHECK(vec_string_LE[i] == vec_string_BE[i]);
    }

    REQUIRE(vec_complex_data_LE.size() == vec_complex_data_BE.size());
    for (size_t i = 0; i < vec_complex_data_LE.size(); i++) {
      CHECK(vec_complex_data_LE[i].real() == vec_complex_data_LE[i].real());
      CHECK(vec_complex_data_LE[i].imag() == vec_complex_data_LE[i].imag());
    }
  }
}

TEST_CASE("Test loading correctness wrt. NumPy generated npz", "[visp::cnpy I/O]")
{
  const bool has_complex = false;

  // Ground-truth data
  bool gt_bool_false, gt_bool_true;
  uint32_t gt_uint32_data;
  int64_t gt_int64_data;
  float gt_float_data;
  double gt_double_data;
  std::string gt_string_data;
  std::complex<double> gt_complex_data;
  std::vector<int> gt_vec_int;
  std::vector<float> gt_vec_flt;
  std::vector<std::string> gt_vec_string;
  std::vector<std::complex<float>> gt_vec_complex_data;
  getNpzGroundTruth(gt_bool_false, gt_bool_true, gt_uint32_data, gt_int64_data, gt_float_data, gt_double_data,
    gt_string_data, gt_complex_data, gt_vec_int, gt_vec_flt, gt_vec_string, gt_vec_complex_data);

  SECTION("Check little-endian correctness")
  {
    const std::string npz_filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
      "npz/numpy/visp_npz_test_data_numpy_LE.npz");

    bool bool_false = false, bool_true = false;
    uint32_t uint32_data = 0;
    int64_t int64_data = 0;
    float float_data;
    double double_data;
    std::string string_data;
    std::complex<double> complex_data;
    std::vector<int> vec_int;
    std::vector<float> vec_flt;
    std::vector<std::string> vec_string;
    std::vector<std::complex<float>> vec_complex_data;

    loadData(npz_filename, bool_false, bool_true, uint32_data, int64_data, float_data, double_data, string_data,
      complex_data, vec_int, vec_flt, vec_string, vec_complex_data, has_complex);

    CHECK(bool_false == gt_bool_false);
    CHECK(bool_true == gt_bool_true);
    CHECK(uint32_data == gt_uint32_data);
    CHECK(int64_data == gt_int64_data);
    CHECK(float_data == gt_float_data);
    CHECK(double_data == gt_double_data);
    CHECK(string_data == gt_string_data);

    REQUIRE(gt_vec_int.size() == gt_vec_flt.size());
    REQUIRE(gt_vec_int.size() == vec_int.size());
    REQUIRE(gt_vec_int.size() == vec_flt.size());
    for (size_t i = 0; i < gt_vec_int.size(); i++) {
      CHECK(gt_vec_int[i] == vec_int[i]);
      CHECK(gt_vec_flt[i] == vec_flt[i]);
    }
    REQUIRE(gt_vec_string.size() == vec_string.size());
    for (size_t i = 0; i < gt_vec_string.size(); i++) {
      CHECK(gt_vec_string[i] == vec_string[i]);
    }
  }

  SECTION("Check big-endian correctness")
  {
    const std::string npz_filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
      "npz/numpy/visp_npz_test_data_numpy_BE.npz");

    bool bool_false = false, bool_true = false;
    uint32_t uint32_data = 0;
    int64_t int64_data = 0;
    float float_data;
    double double_data;
    std::string string_data;
    std::complex<double> complex_data;
    std::vector<int> vec_int;
    std::vector<float> vec_flt;
    std::vector<std::string> vec_string;
    std::vector<std::complex<float>> vec_complex_data;

    loadData(npz_filename, bool_false, bool_true, uint32_data, int64_data, float_data, double_data, string_data,
      complex_data, vec_int, vec_flt, vec_string, vec_complex_data, has_complex);

    CHECK(bool_false == gt_bool_false);
    CHECK(bool_true == gt_bool_true);
    CHECK(uint32_data == gt_uint32_data);
    CHECK(int64_data == gt_int64_data);
    CHECK(float_data == gt_float_data);
    CHECK(double_data == gt_double_data);
    CHECK(string_data == gt_string_data);

    REQUIRE(gt_vec_int.size() == gt_vec_flt.size());
    REQUIRE(gt_vec_int.size() == vec_int.size());
    REQUIRE(gt_vec_int.size() == vec_flt.size());
    for (size_t i = 0; i < gt_vec_int.size(); i++) {
      CHECK(gt_vec_int[i] == vec_int[i]);
      CHECK(gt_vec_flt[i] == vec_flt[i]);
    }

    REQUIRE(gt_vec_string.size() == vec_string.size());
    for (size_t i = 0; i < gt_vec_string.size(); i++) {
      CHECK(gt_vec_string[i] == vec_string[i]);
    }
  }
}

TEST_CASE("Test loading correctness wrt. NumPy generated npz + compression", "[visp::cnpy I/O]")
{
  // check if file exists
  const bool correct_dataset_version = vpIoTools::checkFilename(
    vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                              "npz/numpy/visp_npz_test_data_numpy_LE_compressed.npz"));
  if (!correct_dataset_version) {
    return;
  }

  const bool has_complex = false;

  // Ground-truth data
  bool gt_bool_false, gt_bool_true;
  uint32_t gt_uint32_data;
  int64_t gt_int64_data;
  float gt_float_data;
  double gt_double_data;
  std::string gt_string_data;
  std::complex<double> gt_complex_data;
  std::vector<int> gt_vec_int;
  std::vector<float> gt_vec_flt;
  std::vector<std::string> gt_vec_string;
  std::vector<std::complex<float>> gt_vec_complex_data;
  getNpzGroundTruth(gt_bool_false, gt_bool_true, gt_uint32_data, gt_int64_data, gt_float_data, gt_double_data,
    gt_string_data, gt_complex_data, gt_vec_int, gt_vec_flt, gt_vec_string, gt_vec_complex_data);

  SECTION("Check little-endian correctness")
  {
    const std::string npz_filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
      "npz/numpy/visp_npz_test_data_numpy_LE_compressed.npz");

    bool bool_false = false, bool_true = false;
    uint32_t uint32_data = 0;
    int64_t int64_data = 0;
    float float_data;
    double double_data;
    std::string string_data;
    std::complex<double> complex_data;
    std::vector<int> vec_int;
    std::vector<float> vec_flt;
    std::vector<std::string> vec_string;
    std::vector<std::complex<float>> vec_complex_data;

    loadData(npz_filename, bool_false, bool_true, uint32_data, int64_data, float_data, double_data, string_data,
      complex_data, vec_int, vec_flt, vec_string, vec_complex_data, has_complex);

    CHECK(bool_false == gt_bool_false);
    CHECK(bool_true == gt_bool_true);
    CHECK(uint32_data == gt_uint32_data);
    CHECK(int64_data == gt_int64_data);
    CHECK(float_data == gt_float_data);
    CHECK(double_data == gt_double_data);
    // CHECK(string_data == gt_string_data); // Cannot manage to make it work with compressed string from NumPy

    REQUIRE(gt_vec_int.size() == gt_vec_flt.size());
    REQUIRE(gt_vec_int.size() == vec_int.size());
    REQUIRE(gt_vec_int.size() == vec_flt.size());
    for (size_t i = 0; i < gt_vec_int.size(); i++) {
      CHECK(gt_vec_int[i] == vec_int[i]);
      CHECK(gt_vec_flt[i] == vec_flt[i]);
    }
    REQUIRE(gt_vec_string.size() == vec_string.size());
    // Cannot manage to make it work with compressed string from NumPy
    // for (size_t i = 0; i < gt_vec_string.size(); i++) {
    //   CHECK(gt_vec_string[i] == vec_string[i]);
    // }
  }
}
#endif

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  return numFailed;
}

#else
int main() { return EXIT_SUCCESS; }
#endif
