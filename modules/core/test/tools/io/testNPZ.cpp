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
  \example testNPZ.cpp
 */
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEndian.h>

#if defined(VISP_HAVE_CATCH2) && \
  (defined(_WIN32) || (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))) && \
  defined(VISP_LITTLE_ENDIAN) && defined(VISP_HAVE_MINIZ)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <type_traits>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImage.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
std::string createTmpDir()
{
  std::string username;
  vpIoTools::getUserName(username);

#if defined(_WIN32)
  std::string tmp_dir = "C:/temp/" + username;
#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
  std::string tmp_dir = "/tmp/" + username;
#endif
  std::string directory_filename = tmp_dir + "/testNPZ/";

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
    const std::string read_string = std::string(vec_arr_string_data.begin(), vec_arr_string_data.end());
    CHECK(save_string == read_string);
  }

  SECTION("Read/Save multi-dimensional array")
  {
    const std::string identifier = "Array";
    size_t height = 5, width = 7, channels = 3;
    std::vector<int> save_vec_copy;
    {
      std::vector<int> save_vec;
      save_vec.reserve(height*width*channels);
      for (size_t i = 0; i < height*width*channels; i++) {
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
      for (size_t i = 0; i < read_vec.size(); i++) {
        CHECK(save_vec_copy[i] == read_vec[i]);
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
    }
  }

  SECTION("Read/Save std::complex<double>")
  {
    // Handling of std::complex<>?
    //  - https://github.com/rogersce/cnpy/blob/4e8810b1a8637695171ed346ce68f6984e585ef4/cnpy.cpp#L40-L42
    //  - https://github.com/rogersce/cnpy/blob/4e8810b1a8637695171ed346ce68f6984e585ef4/cnpy.h#L129
    // https://en.cppreference.com/w/cpp/named_req/TriviallyCopyable
    CHECK(std::is_trivially_copyable<std::complex<double>>::value == true);
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
    }
  }

  SECTION("Read/Save vpHomogeneousMatrix")
  {
    const std::string identifier = "vpHomogeneousMatrix";
    vpHomogeneousMatrix cMo_save_copy;
    {
      vpHomogeneousMatrix cMo_save(vpTranslationVector(10, 20, 30), vpThetaUVector(1, 2, 3));
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
    }
  }

  SECTION("Read/Save std::vector<vpHomogeneousMatrix>")
  {
    const std::string identifier = "std::vector<vpHomogeneousMatrix>";
    std::vector<vpHomogeneousMatrix> vec_cMo_save_copy;
    {
      std::vector<double> vec_cMo_save;
      for (size_t i = 0; i < 5; i++) {
        vpHomogeneousMatrix cMo_save(vpTranslationVector(1+10*i, 2+20*i, 3+30*i), vpThetaUVector(0.1+i, 0.2+i, 0.3+i));
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

  const std::string identifier = "data";
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
  }

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
  }

  REQUIRE(vpIoTools::remove(directory_filename));
  REQUIRE(!vpIoTools::checkDirectory(directory_filename));
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
