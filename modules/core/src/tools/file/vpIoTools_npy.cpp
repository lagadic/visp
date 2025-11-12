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
 * Io tools dedicated to npy.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>

#if defined(VISP_HAVE_MINIZ) && defined(VISP_HAVE_WORKING_REGEX)
#define USE_ZLIB_API 0

#if !USE_ZLIB_API
// See: https://github.com/BinomialLLC/basis_universal/blob/master/encoder/basisu_miniz.h
// Apache License, Version 2.0
#include "basisu_miniz.h"

using namespace buminiz;
#else
#include <zlib.h>
#endif

// To avoid warnings such as: warning: unused variable ‘littleEndian’ [-Wunused-variable]
#define UNUSED(x) ((void)(x)) // see: https://stackoverflow.com/a/777359

// Copyright (C) 2011  Carl Rogers
// Released under MIT License
// license available in LICENSE file, or at http://www.opensource.org/licenses/mit-license.php


// anonymous namespace
namespace
{
#if defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME; // for vpEndian calls
#endif

void reverse_data(std::shared_ptr<std::vector<char> > &data_holder, const std::vector<size_t> &shape,
  size_t word_size, char data_type)
{
  if (!shape.empty()) {
    size_t total_size = shape[0];
    for (size_t i = 1; i < shape.size(); i++) {
      total_size *= shape[i];
    }

    // std::complex
    if (data_type == 'c') {
      const size_t half_word_size = word_size / 2;
      for (size_t i = 0; i < total_size; i++) {
        // real
        std::reverse(data_holder->begin() + i*word_size, data_holder->begin() + i*word_size + half_word_size);
        // imag
        std::reverse(data_holder->begin() + i*word_size + half_word_size, data_holder->begin() + (i+1)*word_size);
      }
    }
    else if (data_type != 'U') {
      for (size_t i = 0; i < total_size; i++) {
        std::reverse(data_holder->begin() + i*word_size, data_holder->begin() + (i+1)*word_size);
      }
    }
  }
  else {
    // 0d-array (number) with NumPy
    if (data_type == 'c') {
      const size_t half_word_size = word_size / 2;
      // real
      std::reverse(data_holder->begin(), data_holder->begin() + half_word_size);
      // imag
      std::reverse(data_holder->begin() + half_word_size, data_holder->begin() + word_size);
    }
    else if (data_type != 'U') {
      std::reverse(data_holder->begin(), data_holder->begin() + word_size);
    }
  }

  if (data_type == 'U') { // special case to handle UTF-32 string data
    size_t utf32_size = 4;
    for (size_t i = 0; i < data_holder->size(); i += utf32_size) {
      std::reverse(data_holder->begin() + i, data_holder->begin() + (i+utf32_size)); // NumPy saves string in UTF-32
    }
  }
}

uint16_t swap16bits_if(uint16_t val, bool swap)
{
  if (swap) {
    return vpEndian::swap16bits(val);
  }

  return val;
}

uint32_t swap32bits_if(uint32_t val, bool swap)
{
  if (swap) {
    return vpEndian::swap32bits(val);
  }

  return val;
}

// https://github.com/rogersce/cnpy/pull/78/files
// https://github.com/nmcclatchey/Bugfix-for-cnpy/blob/e148a5ce5db80fa3e28ce6d551343cfc8ebdc832/cnpy.cpp#L285
struct AutoCloser
{
  FILE *fp;
  AutoCloser() : fp(nullptr) { }
  ~AutoCloser()
  {
    if (fp != nullptr) {
      fclose(fp);
    }
  }
};
} // anonymous namespace

char visp::cnpy::BigEndianTest()
{
  int x = 1;
  return (((reinterpret_cast<char *>(&x))[0]) ? '<' : '>');
}

char visp::cnpy::map_type(const std::type_info &t)
{
  if (t == typeid(float)) { return 'f'; }
  if (t == typeid(double)) { return 'f'; }
  if (t == typeid(long double)) { return 'f'; }

  if (t == typeid(int)) { return 'i'; }
  if (t == typeid(char)) { return 'i'; }
  if (t == typeid(short)) { return 'i'; }
  if (t == typeid(long)) { return 'i'; }
  if (t == typeid(long long)) { return 'i'; }

  if (t == typeid(unsigned char)) { return 'u'; }
  if (t == typeid(unsigned short)) { return 'u'; }
  if (t == typeid(unsigned long)) { return 'u'; }
  if (t == typeid(unsigned long long)) { return 'u'; }
  if (t == typeid(unsigned int)) { return 'u'; }

  if (t == typeid(bool)) { return 'b'; }

  if (t == typeid(std::complex<float>)) { return 'c'; }
  if (t == typeid(std::complex<double>)) { return 'c'; }
  if (t == typeid(std::complex<long double>)) { return 'c'; }

  if (t == typeid(std::string)) { return 'U'; }

  else { return '?'; }
}

void visp::cnpy::parse_npy_header(unsigned char *buffer, size_t &word_size, std::vector<size_t> &shape,
  bool &fortran_order, bool &little_endian, char &data_type)
{
  uint16_t header_len = *reinterpret_cast<uint16_t *>(buffer+8);
  std::string header(reinterpret_cast<char *>(buffer+9), header_len);

  //fortran order
  size_t loc1 = header.find("fortran_order")+16;
  fortran_order = (header.substr(loc1, 4) == "True" ? true : false);

  //shape
  loc1 = header.find("(");
  size_t loc2 = header.find(")");

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1+1, loc2-loc1-1);
  while (std::regex_search(str_shape, sm, num_regex)) {
    // https://github.com/rogersce/cnpy/commit/ca6c0ce5bed57e3b5b64aede4f39aa07a9e71f5e
    shape.push_back(std::stoll(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  //endian, word size, data type
  //byte order code | stands for not applicable.
  //not sure when this applies except for byte array
  loc1 = header.find("descr")+9;
  little_endian = ((header[loc1] == '<') || (header[loc1] == '|') ? true : false);
  data_type = header[loc1+1];

  std::string str_ws = header.substr(loc1+2);
  loc2 = str_ws.find("'");
  word_size = atoll(str_ws.substr(0, loc2).c_str());
}

void visp::cnpy::parse_npy_header(FILE *fp, size_t &word_size, std::vector<size_t> &shape,
  bool &fortran_order, bool &little_endian, char &data_type)
{
  char buffer[256];
  size_t res = fread(buffer, sizeof(char), 11, fp);
  if (res != 11) {
    std::ostringstream oss;
    oss << "parse_npy_header: failed fread, res=" << res;
    throw std::runtime_error(oss.str());
  }
  std::string header = fgets(buffer, 256, fp);
  assert(header[header.size()-1] == '\n');

  size_t loc1, loc2;

  //fortran order
  loc1 = header.find("fortran_order");
  if (loc1 == std::string::npos) {
    throw std::runtime_error("parse_npy_header: failed to find header keyword: 'fortran_order'");
  }
  loc1 += 16;
  fortran_order = (header.substr(loc1, 4) == "True" ? true : false);

  //shape
  loc1 = header.find("(");
  loc2 = header.find(")");
  if ((loc1 == std::string::npos) || (loc2 == std::string::npos)) {
    throw std::runtime_error("parse_npy_header: failed to find header keyword: '(' or ')'");
  }

  std::regex num_regex("[0-9][0-9]*");
  std::smatch sm;
  shape.clear();

  std::string str_shape = header.substr(loc1+1, loc2-loc1-1);
  while (std::regex_search(str_shape, sm, num_regex)) {
    // https://github.com/rogersce/cnpy/commit/ca6c0ce5bed57e3b5b64aede4f39aa07a9e71f5e
    shape.push_back(std::stoll(sm[0].str()));
    str_shape = sm.suffix().str();
  }

  //endian, word size, data type
  //byte order code | stands for not applicable.
  //not sure when this applies except for byte array
  loc1 = header.find("descr");
  if (loc1 == std::string::npos) {
    throw std::runtime_error("parse_npy_header: failed to find header keyword: 'descr'");
  }
  loc1 += 9;
  little_endian = ((header[loc1] == '<') || (header[loc1] == '|') ? true : false);
  data_type = header[loc1+1];

  std::string str_ws = header.substr(loc1+2);
  loc2 = str_ws.find("'");
  word_size = atoll(str_ws.substr(0, loc2).c_str());
  if (data_type == 'U') {
    word_size *= 4; // UTF-32 with NumPy
  }
}

void visp::cnpy::parse_zip_footer(FILE *fp, uint16_t &nrecs, size_t &global_header_size, size_t &global_header_offset)
{
  std::vector<char> footer(22);
  fseek(fp, -22, SEEK_END);
  size_t res = fread(&footer[0], sizeof(char), 22, fp);
  if (res != 22) {
    std::ostringstream oss;
    oss << "parse_zip_footer: failed fread, res=" << res;
    throw std::runtime_error(oss.str());
  }

  uint16_t disk_no, disk_start, nrecs_on_disk, comment_len;
#ifdef VISP_BIG_ENDIAN
  disk_no = vpEndian::swap16bits(*(uint16_t *)&footer[4]);
  disk_start = vpEndian::swap16bits(*(uint16_t *)&footer[6]);
  nrecs_on_disk = vpEndian::swap16bits(*(uint16_t *)&footer[8]);
  nrecs = vpEndian::swap16bits(*(uint16_t *)&footer[10]);
  global_header_size = vpEndian::swap32bits(*(uint32_t *)&footer[12]);
  global_header_offset = vpEndian::swap32bits(*(uint32_t *)&footer[16]);
  comment_len = vpEndian::swap16bits(*(uint16_t *)&footer[20]);
#else
  disk_no = *(uint16_t *)&footer[4];
  disk_start = *(uint16_t *)&footer[6];
  nrecs_on_disk = *(uint16_t *)&footer[8];
  nrecs = *(uint16_t *)&footer[10];
  global_header_size = *(uint32_t *)&footer[12];
  global_header_offset = *(uint32_t *)&footer[16];
  comment_len = *(uint16_t *)&footer[20];
#endif

  UNUSED(disk_no); assert(disk_no == 0);
  UNUSED(disk_start); assert(disk_start == 0);
  UNUSED(nrecs_on_disk); assert(nrecs_on_disk == nrecs);
  UNUSED(comment_len); assert(comment_len == 0);
}

visp::cnpy::NpyArray load_the_npy_file(FILE *fp)
{
  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order, little_endian;
  char data_type = 'i'; // integer type
  visp::cnpy::parse_npy_header(fp, word_size, shape, fortran_order, little_endian, data_type);

  visp::cnpy::NpyArray arr(shape, word_size, fortran_order, data_type);
  size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp);
  if (nread != arr.num_bytes()) {
    std::ostringstream oss;
    oss << "load_the_npy_file: failed fread, nread=" << nread << " ; num_bytes=" << arr.num_bytes();
    throw std::runtime_error(oss.str());
  }

#ifdef VISP_LITTLE_ENDIAN
  if (!little_endian) {
    reverse_data(arr.data_holder, arr.shape, arr.word_size, data_type);
  }
#else
  if (little_endian) {
    reverse_data(arr.data_holder, arr.shape, arr.word_size, data_type);
  }
#endif
  return arr;
}

visp::cnpy::NpyArray load_the_npz_array(FILE *fp, uint32_t compr_bytes, uint32_t uncompr_bytes)
{
  std::vector<unsigned char> buffer_compr(compr_bytes);
  std::vector<unsigned char> buffer_uncompr(uncompr_bytes);
  size_t nread = fread(&buffer_compr[0], 1, compr_bytes, fp);
  if (nread != compr_bytes) {
    std::ostringstream oss;
    oss << "load_the_npz_array: failed fread, nread=" << nread << " ; compr_bytes=" << compr_bytes;
    throw std::runtime_error(oss.str());
  }

  z_stream d_stream;

  d_stream.zalloc = Z_NULL;
  d_stream.zfree = Z_NULL;
  d_stream.opaque = Z_NULL;
  d_stream.avail_in = 0;
  d_stream.next_in = Z_NULL;
  int err = inflateInit2(&d_stream, -MAX_WBITS);
  // https://github.com/rogersce/cnpy/commit/3ed2bc4063c455269b37af63442c595ee1bd60e1
  if (err != Z_OK) {
    std::ostringstream oss;
    oss << "load_the_npz_array: zlib inflateInit2 failed ; err=" << err;
    throw std::runtime_error(oss.str());
  }

  d_stream.avail_in = compr_bytes;
  d_stream.next_in = &buffer_compr[0];
  d_stream.avail_out = uncompr_bytes;
  d_stream.next_out = &buffer_uncompr[0];

  err = inflate(&d_stream, Z_FINISH);
  if (err != Z_OK) {
    std::ostringstream oss;
    oss << "load_the_npz_array: zlib inflate failed ; err=" << err;
    throw std::runtime_error(oss.str());
  }
  err = inflateEnd(&d_stream);
  if (err != Z_OK) {
    std::ostringstream oss;
    oss << "load_the_npz_array: zlib inflateEnd failed ; err=" << err;
    throw std::runtime_error(oss.str());
  }

  std::vector<size_t> shape;
  size_t word_size;
  bool fortran_order;
  bool little_endian = true;
  char data_type = 'i'; // integer type
  visp::cnpy::parse_npy_header(&buffer_uncompr[0], word_size, shape, fortran_order, little_endian, data_type);

  visp::cnpy::NpyArray array(shape, word_size, fortran_order, data_type);

  size_t offset = uncompr_bytes - array.num_bytes();
  memcpy(array.data<unsigned char>(), &buffer_uncompr[0]+offset, array.num_bytes());

#ifdef VISP_LITTLE_ENDIAN
  if (!little_endian) {
    reverse_data(array.data_holder, array.shape, array.word_size, data_type);
  }
#else
  if (little_endian) {
    reverse_data(array.data_holder, array.shape, array.word_size, data_type);
  }
#endif

  return array;
}

/*!
  Load the specified \p fname filepath as arrays of data. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.load.html">numpy.load</a> function.
  \param[in] fname : Path to the npz file.
  \return A map of arrays data. The key represents the variable name, the value is an array of basic data type.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.

  \sa To see how to use it, you may have a look at \ref tutorial-npz
 */
visp::cnpy::npz_t visp::cnpy::npz_load(const std::string &fname)
{
  if (!vpIoTools::checkFilename(fname)) {
    throw vpException(vpException::ioError, "This file does not exist: " + fname);
  }

  AutoCloser closer;
  closer.fp = fopen(fname.c_str(), "rb");

  if (!closer.fp) {
    throw std::runtime_error("npz_load: Error! Unable to open file " + fname + "!");
  }

  visp::cnpy::npz_t arrays;
  bool quit = false;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_26 = 26;
  const unsigned int index_28 = 28;
  const unsigned int val_8 = 8;
  const unsigned int val_18 = 18;
  const unsigned int val_22 = 22;
  const unsigned int val_30 = 30;

  bool host_is_LE = true;
#ifndef VISP_LITTLE_ENDIAN
  host_is_LE = false;
#endif

  const bool header_file_is_LE = true;
  bool same_endianness = (host_is_LE == header_file_is_LE);
  while (!quit) {
    std::vector<char> local_header(val_30);
    size_t headerres = fread(&local_header[0], sizeof(char), val_30, closer.fp);
    if (headerres != 30) {
      throw std::runtime_error("npz_load: failed fread 1");
    }

    //if we've reached the global header, stop reading
    if ((local_header[index_2] != 0x03) || (local_header[index_3] != 0x04)) {
      quit = true;
    }
    else {
      //read in the variable name
      uint16_t name_len = swap16bits_if(*(uint16_t *)&local_header[index_26], !same_endianness);
      std::string varname(name_len, ' ');
      size_t vname_res = fread(&varname[0], sizeof(char), name_len, closer.fp);
      if (vname_res != name_len) {
        throw std::runtime_error("npz_load: failed fread 2");
      }

      //erase the lagging .npy
      varname.erase(varname.end()-4, varname.end());

      //read in the extra field
      uint16_t extra_field_len = swap16bits_if(*(uint16_t *)&local_header[index_28], !same_endianness);
      if (extra_field_len > 0) {
        std::vector<char> buff(extra_field_len);
        size_t efield_res = fread(&buff[0], sizeof(char), extra_field_len, closer.fp);
        if (efield_res != extra_field_len) {
          throw std::runtime_error("npz_load: failed fread 3");
        }
      }

      uint16_t compr_method = swap16bits_if(*reinterpret_cast<uint16_t *>(&local_header[0] + val_8), !same_endianness);
      uint32_t compr_bytes = swap32bits_if(*reinterpret_cast<uint32_t *>(&local_header[0] + val_18), !same_endianness);
      uint32_t uncompr_bytes = swap32bits_if(*reinterpret_cast<uint32_t *>(&local_header[0] + val_22), !same_endianness);

      if (compr_method == 0) {
        arrays[varname] = load_the_npy_file(closer.fp);
      }
      else {
        arrays[varname] = load_the_npz_array(closer.fp, compr_bytes, uncompr_bytes);
      }
    }
  }

  return arrays;
}

/*!
  Load the specified \p varname array of data from the \p fname npz file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.load.html">numpy.load</a> function.
  \param[in] fname : Path to the npz file.
  \param[in] varname : Identifier for the requested array of data.
  \return An array of basic data type.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.

  \sa To see how to use it, you may have a look at \ref tutorial-npz
 */
visp::cnpy::NpyArray visp::cnpy::npz_load(const std::string &fname, const std::string &varname)
{
  if (!vpIoTools::checkFilename(fname)) {
    throw vpException(vpException::ioError, "This file does not exist: " + fname);
  }

  AutoCloser closer;
  closer.fp = fopen(fname.c_str(), "rb");

  if (!closer.fp) {
    throw std::runtime_error("npz_load: Unable to open file " + fname + "!");
  }

  bool quit = false;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_26 = 26;
  const unsigned int index_28 = 28;
  const unsigned int val_8 = 8;
  const unsigned int val_18 = 18;
  const unsigned int val_22 = 22;
  const unsigned int val_30 = 30;

  bool host_is_LE = true;
#ifndef VISP_LITTLE_ENDIAN
  host_is_LE = false;
#endif

  const bool header_file_is_LE = true;
  bool same_endianness = (host_is_LE == header_file_is_LE);
  while (!quit) {
    std::vector<char> local_header(val_30);
    size_t header_res = fread(&local_header[0], sizeof(char), val_30, closer.fp);
    if (header_res != 30) {
      throw std::runtime_error("npz_load 2: failed fread");
    }

    //if we've reached the global header, stop reading
    if ((local_header[index_2] != 0x03) || (local_header[index_3] != 0x04)) {
      quit = true;
    }
    else {
      //read in the variable name
      uint16_t name_len = swap16bits_if(*(uint16_t *)&local_header[index_26], !same_endianness);
      std::string vname(name_len, ' ');
      size_t vname_res = fread(&vname[0], sizeof(char), name_len, closer.fp);
      if (vname_res != name_len) {
        throw std::runtime_error("npz_load 2: failed fread");
      }
      vname.erase(vname.end()-4, vname.end()); //erase the lagging .npy

      //read in the extra field
      uint16_t extra_field_len = swap16bits_if(*(uint16_t *)&local_header[index_28], !same_endianness);
      fseek(closer.fp, extra_field_len, SEEK_CUR); //skip past the extra field

      uint16_t compr_method = swap16bits_if(*reinterpret_cast<uint16_t *>(&local_header[0] + val_8), !same_endianness);
      uint32_t compr_bytes = swap32bits_if(*reinterpret_cast<uint32_t *>(&local_header[0] + val_18), !same_endianness);
      uint32_t uncompr_bytes = swap32bits_if(*reinterpret_cast<uint32_t *>(&local_header[0] + val_22), !same_endianness);

      if (vname == varname) {
        NpyArray array = (compr_method == 0) ? load_the_npy_file(closer.fp) : load_the_npz_array(closer.fp, compr_bytes, uncompr_bytes);
        return array;
      }
      else {
        //skip past the data
        uint32_t size = swap32bits_if(*(uint32_t *)&local_header[22], !same_endianness);
        fseek(closer.fp, size, SEEK_CUR);
      }
    }
  }

  //if we get here, we haven't found the variable in the file
  throw std::runtime_error("npz_load 2: Variable name " + varname + " not found in " + fname);
}

/*!
  Load the specified npy \p fname filepath as one array of data. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.load.html">numpy.load</a> function.
  \param[in] fname : Path to the npy file.
  \return An array of basic data type.
  \warning This function has only been tested on little endian platform.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.
 */
visp::cnpy::NpyArray visp::cnpy::npy_load(const std::string &fname)
{
  if (!vpIoTools::checkFilename(fname)) {
    throw vpException(vpException::ioError, "This file does not exist: " + fname);
  }

  AutoCloser closer;
  closer.fp = fopen(fname.c_str(), "rb");

  if (!closer.fp) {
    throw std::runtime_error("npy_load: Unable to open file " + fname + "!");
  }

  NpyArray arr = load_the_npy_file(closer.fp);

  return arr;
}

namespace visp
{
namespace cnpy
{
// 000000e0  ff ff 12 00 14 00 4d 79  20 73 74 72 69 6e 67 20  |......My string |
// 000000f0  64 61 74 61 2e 6e 70 79  01 00 10 00 8c 00 00 00  |data.npy........|
// 00000100  00 00 00 00 8c 00 00 00  00 00 00 00 93 4e 55 4d  |.............NUM|
// 00000110  50 59 01 00 76 00 7b 27  64 65 73 63 72 27 3a 20  |PY..v.{'descr': |
// 00000120  27 3c 55 33 27 2c 20 27  66 6f 72 74 72 61 6e 5f  |'<U3', 'fortran_|
// 00000130  6f 72 64 65 72 27 3a 20  46 61 6c 73 65 2c 20 27  |order': False, '|
// 00000140  73 68 61 70 65 27 3a 20  28 29 2c 20 7d 20 20 20  |shape': (), }   |
// 00000150  20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20  |                |
// 00000160  20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20  |                |
// 00000170  20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20  |                |
// 00000180  20 20 20 20 20 20 20 20  20 20 20 0a 61 00 00 00  |           .a...|
// 00000190  62 00 00 00 63 00 00 00  50 4b 03 04 2d 00 00 00  |b...c...PK..-...|
// 000001a0  00 00 00 00 21 00 76 38  d2 58 ff ff ff ff ff ff  |....!.v8.X......|
// 000001b0  ff ff 16 00 14 00 4d 79  20 76 65 63 20 73 74 72  |......My vec str|
// 000001c0  69 6e 67 20 64 61 74 61  2e 6e 70 79 01 00 10 00  |ing data.npy....|
// 000001d0  40 01 00 00 00 00 00 00  40 01 00 00 00 00 00 00  |@.......@.......|
// 000001e0  93 4e 55 4d 50 59 01 00  76 00 7b 27 64 65 73 63  |.NUMPY..v.{'desc|
// 000001f0  72 27 3a 20 27 3c 55 38  27 2c 20 27 66 6f 72 74  |r': '<U8', 'fort|
// 00000200  72 61 6e 5f 6f 72 64 65  72 27 3a 20 46 61 6c 73  |ran_order': Fals|
// 00000210  65 2c 20 27 73 68 61 70  65 27 3a 20 28 32 2c 20  |e, 'shape': (2, |
// 00000220  33 29 2c 20 7d 20 20 20  20 20 20 20 20 20 20 20  |3), }           |
// 00000230  20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20  |                |
// 00000240  20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20  |                |
// 00000250  20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 0a  |               .|
// 00000260  61 00 00 00 62 00 00 00  63 00 00 00 64 00 00 00  |a...b...c...d...|
// 00000270  20 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  | ...............|
// 00000280  65 00 00 00 66 00 00 00  67 00 00 00 68 00 00 00  |e...f...g...h...|
// 00000290  69 00 00 00 6a 00 00 00  6b 00 00 00 6c 00 00 00  |i...j...k...l...|
// 000002a0  6d 00 00 00 6e 00 00 00  00 00 00 00 00 00 00 00  |m...n...........|
// 000002b0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
// 000002c0  6f 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |o...............|
// 000002d0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
// 000002e0  70 00 00 00 71 00 00 00  00 00 00 00 00 00 00 00  |p...q...........|
// 000002f0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
// 00000300  72 00 00 00 73 00 00 00  74 00 00 00 00 00 00 00  |r...s...t.......|
// 00000310  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  |................|
// 00000320  50 4b 01 02 2d 03 2d 00  00 00 00 00 00 00 21 00  |PK..-.-.......!.|
// 00000330  4b f2 4d a7 84 00 00 00  84 00 00 00 12 00 00 00  |K.M.............|
//
// Create header for special string case
std::vector<char> create_npy_header_string(const std::vector<size_t> &shape, const std::vector<size_t> &lengths)
{
  size_t max_length = *std::max_element(lengths.begin(), lengths.end());

  std::vector<char> dict;
  dict += "{'descr': '";
  dict += BigEndianTest();
  dict += map_type(typeid(std::string));
  dict += std::to_string(max_length);
  if (shape.size() > 0) {
    dict += "', 'fortran_order': False, 'shape': (";
    dict += std::to_string(shape[0]);
    for (size_t i = 1; i < shape.size(); ++i) {
      dict += ", ";
      dict += std::to_string(shape[i]);
    }
    if (shape.size() == 1) dict += ",";
    dict += "), }";
  }
  else {
    dict += "', 'fortran_order': False, 'shape': (";
    dict += "), }";
  }
  //pad with spaces so that preamble+dict is modulo 16 bytes. preamble is 10 bytes. dict needs to end with \n
  int remainder = 16 - (10 + dict.size()) % 16;
  dict.insert(dict.end(), remainder, ' ');
  dict.back() = '\n';

  std::vector<char> header;
  header += static_cast<char>(0x93);
  header += "NUMPY";
  header += static_cast<char>(0x01); //major version of numpy format
  header += static_cast<char>(0x00); //minor version of numpy format
#ifdef VISP_BIG_ENDIAN
  header += vpEndian::swap16bits(static_cast<uint16_t>(dict.size()));
#else
  header += static_cast<uint16_t>(dict.size());
#endif
  header.insert(header.end(), dict.begin(), dict.end());

  return header;
}

// std::vector<char> utf8_to_utf32_vec(const std::string &utf8)
// {
//   // SO: https://stackoverflow.com/questions/52703630/convert-c-stdstring-to-utf-16-le-encoded-string/52703954#52703954
//   // SO and ChatGPT gives approximatively the same logic

//   // But since I don't know how C++ deals with utf-16/utf-32 and std::string and for simplicity we do something stupid
//   // instead (see utf8_to_utf32_vec() func below).

//   std::wstring_convert<std::codecvt_utf8<char32_t, 1114111UL, std::codecvt_mode::little_endian>, char32_t> cnv;
//   std::u32string s = cnv.from_bytes(utf8);
//   if (cnv.converted() < utf8.size()) {
//     throw std::runtime_error("incomplete conversion");
//   }

//   std::vector<char> utf32Vector;
//   for (char32_t uc : s) {
//     utf32Vector.push_back(static_cast<char>(uc & 0xFF));
//     utf32Vector.push_back(static_cast<char>((uc >> 8) & 0xFF));
//     utf32Vector.push_back(static_cast<char>((uc >> 16) & 0xFF));
//     utf32Vector.push_back(static_cast<char>((uc >> 24) & 0xFF));
//   }

//   return utf32Vector;
// }

// NumPy saves a string character using 32-bits:
// - https://github.com/numpy/numpy/issues/15347
// Here for simplicity we assume std::string is utf-8 and we simply pad the vector with 0-val (NULL character).
std::vector<char> utf8_to_utf32_vec_pad(const std::string &utf8, const std::size_t &max_size)
{
  std::vector<char> utf32Vector;
  utf32Vector.reserve(4*max_size);

  for (const auto &ch : utf8) {
#ifdef VISP_BIG_ENDIAN
    utf32Vector.push_back(0);
    utf32Vector.push_back(0);
    utf32Vector.push_back(0);
    utf32Vector.push_back(ch);
#else
    utf32Vector.push_back(ch);
    utf32Vector.push_back(0);
    utf32Vector.push_back(0);
    utf32Vector.push_back(0);
#endif
  }

  // padding
  size_t cur_size = utf32Vector.size();
  for (size_t i = cur_size; i < 4*max_size; i++) {
    utf32Vector.push_back(0);
  }

  return utf32Vector;
}
} // cnpy
} // visp

/*!
  Save the specified \p fname vector of std::string (\p data) into the \p zipname npz file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.savez.html">numpy.savez</a> function.
  \param[in] zipname : Path to the npz file.
  \param[in] fname : Identifier for the corresponding array of data.
  \param[in] data_vec : Vector of std::string.
  \param[in] shape : Shape of the array, e.g. Nz x Ny x Nx.
  \param[in] mode : Writing mode, i.e. overwrite (w) or append (a) to the file.
  \warning This function should also work on big-endian platform, without guarantee since it has not been tested extensively.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.

  \sa To see how to use it, you may have a look at \ref tutorial-npz
 */
void visp::cnpy::npz_save(const std::string &zipname, std::string fname, const std::vector<std::string> &data_vec, const std::vector<size_t> &shape, const std::string &mode)
{
  if (data_vec.empty()) {
    vpException(vpException::badValue, "Input string data is empty.");
  }
  if (shape.empty()) {
    vpException(vpException::dimensionError, "Input string data shape vec is empty.");
  }

  //first, append a .npy to the fname
  fname += ".npy";

  //now, on with the show
  FILE *fp = NULL;
  uint16_t nrecs = 0;
  size_t global_header_offset = 0;
  std::vector<char> global_header;

  if (mode == "a") fp = fopen(zipname.c_str(), "r+b");

  if (fp) {
    //zip file exists. we need to add a new npy file to it.
    //first read the footer. this gives us the offset and size of the global header
    //then read and store the global header.
    //below, we will write the the new data at the start of the global header then append the global header and footer below it
    size_t global_header_size;
    parse_zip_footer(fp, nrecs, global_header_size, global_header_offset);
    fseek(fp, static_cast<long>(global_header_offset), SEEK_SET);
    global_header.resize(global_header_size);
    size_t res = fread(&global_header[0], sizeof(char), global_header_size, fp);
    if (res != global_header_size) {
      throw std::runtime_error("npz_save: header read error while adding to existing zip");
    }
    fseek(fp, static_cast<long>(global_header_offset), SEEK_SET);
  }
  else {
    fp = fopen(zipname.c_str(), "wb");
  }

  std::vector<size_t> lengths;
  lengths.reserve(data_vec.size());
  for (auto data_str : data_vec) {
    lengths.push_back(data_str.length());
  }

  std::vector<char> npy_header = create_npy_header_string(shape, lengths);
  size_t max_length = *std::max_element(lengths.begin(), lengths.end());

  std::vector<char> data_str_utf32_LE;
  data_str_utf32_LE.reserve(max_length*4); // NumPy stores string as UTF-32: https://github.com/numpy/numpy/issues/15347
  size_t sub_idx = 0;
  for (size_t i = 0; i < lengths.size(); i++) {
    std::vector<char> substr_utf32 = utf8_to_utf32_vec_pad(data_vec[i], max_length);
    data_str_utf32_LE.insert(data_str_utf32_LE.end(), substr_utf32.begin(), substr_utf32.end());

    sub_idx += lengths[i];
  }

  // https://github.com/rogersce/cnpy/pull/58/files
  size_t nels = data_str_utf32_LE.size();
  size_t nbytes = nels*sizeof(char) + npy_header.size();

  //get the CRC of the data to be added
  uint32_t crc = vp_mz_crc32(0L, (uint8_t *)&npy_header[0], npy_header.size());
  if (nels > 0) {
    crc = vp_mz_crc32(crc, (uint8_t *)&data_str_utf32_LE[0], nels*sizeof(uint8_t));
  }

  //build the local header
  std::vector<char> local_header;
  local_header += "PK"; //first part of sig
#ifdef VISP_BIG_ENDIAN
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(0x0403)); //second part of sig
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(20)); //min version to extract
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(0)); //general purpose bit flag
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(0)); //compression method
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(0)); //file last mod time
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(0));     //file last mod date
  local_header += vpEndian::swap32bits(static_cast<uint32_t>(crc)); //crc
  local_header += vpEndian::swap32bits(static_cast<uint32_t>(nbytes)); //compressed size
  local_header += vpEndian::swap32bits(static_cast<uint32_t>(nbytes)); //uncompressed size
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(fname.size())); //fname length
  local_header += vpEndian::swap16bits(static_cast<uint16_t>(0)); //extra field length
#else
  local_header += static_cast<uint16_t>(0x0403); //second part of sig
  local_header += static_cast<uint16_t>(20); //min version to extract
  local_header += static_cast<uint16_t>(0); //general purpose bit flag
  local_header += static_cast<uint16_t>(0); //compression method
  local_header += static_cast<uint16_t>(0); //file last mod time
  local_header += static_cast<uint16_t>(0);     //file last mod date
  local_header += static_cast<uint32_t>(crc); //crc
  local_header += static_cast<uint32_t>(nbytes); //compressed size
  local_header += static_cast<uint32_t>(nbytes); //uncompressed size
  local_header += static_cast<uint16_t>(fname.size()); //fname length
  local_header += static_cast<uint16_t>(0); //extra field length
#endif
  local_header += fname;

  //build global header
  global_header += "PK"; //first part of sig
#ifdef VISP_BIG_ENDIAN
  global_header += vpEndian::swap16bits(static_cast<uint16_t>(0x0201)); //second part of sig
  global_header += vpEndian::swap16bits(static_cast<uint16_t>(20)); //version made by
  global_header.insert(global_header.end(), local_header.begin()+4, local_header.begin()+30);
  global_header += static_cast<uint16_t>(0); //file comment length
  global_header += static_cast<uint16_t>(0); //disk number where file starts
  global_header += static_cast<uint16_t>(0); //internal file attributes
  global_header += static_cast<uint32_t>(0); //external file attributes
  global_header += vpEndian::swap32bits(static_cast<uint32_t>(global_header_offset)); //relative offset of local file header, since it begins where the global header used to begin
#else
  global_header += static_cast<uint16_t>(0x0201); //second part of sig
  global_header += static_cast<uint16_t>(20); //version made by
  global_header.insert(global_header.end(), local_header.begin()+4, local_header.begin()+30);
  global_header += static_cast<uint16_t>(0); //file comment length
  global_header += static_cast<uint16_t>(0); //disk number where file starts
  global_header += static_cast<uint16_t>(0); //internal file attributes
  global_header += static_cast<uint32_t>(0); //external file attributes
  global_header += static_cast<uint32_t>(global_header_offset); //relative offset of local file header, since it begins where the global header used to begin
#endif
  global_header += fname;

  //build footer
  std::vector<char> footer;
  footer += "PK"; //first part of sig
#ifdef VISP_BIG_ENDIAN
  footer += vpEndian::swap16bits(static_cast<uint16_t>(0x0605)); //second part of sig
  footer += static_cast<uint16_t>(0); //number of this disk
  footer += static_cast<uint16_t>(0); //disk where footer starts
  footer += vpEndian::swap16bits(static_cast<uint16_t>(nrecs+1)); //number of records on this disk
  footer += vpEndian::swap16bits(static_cast<uint16_t>(nrecs+1)); //total number of records
  footer += vpEndian::swap32bits(static_cast<uint32_t>(global_header.size())); //nbytes of global headers
  footer += vpEndian::swap32bits(static_cast<uint32_t>(global_header_offset + nbytes + local_header.size())); //offset of start of global headers, since global header now starts after newly written array
#else
  footer += static_cast<uint16_t>(0x0605); //second part of sig
  footer += static_cast<uint16_t>(0); //number of this disk
  footer += static_cast<uint16_t>(0); //disk where footer starts
  footer += static_cast<uint16_t>(nrecs+1); //number of records on this disk
  footer += static_cast<uint16_t>(nrecs+1); //total number of records
  footer += static_cast<uint32_t>(global_header.size()); //nbytes of global headers
  footer += static_cast<uint32_t>(global_header_offset + nbytes + local_header.size()); //offset of start of global headers, since global header now starts after newly written array
#endif
  footer += static_cast<uint16_t>(0); //zip file comment length

  //write everything
  fwrite(&local_header[0], sizeof(char), local_header.size(), fp);
  fwrite(&npy_header[0], sizeof(char), npy_header.size(), fp);
  fwrite(&data_str_utf32_LE[0], sizeof(char), nels, fp);
  fwrite(&global_header[0], sizeof(char), global_header.size(), fp);
  fwrite(&footer[0], sizeof(char), footer.size(), fp);
  fclose(fp);
}

/*!
  Save the specified \p fname std::string (\p data_str) into the \p zipname npz file. This function is similar to the
  <a href="https://numpy.org/doc/stable/reference/generated/numpy.savez.html">numpy.savez</a> function.
  \param[in] zipname : Path to the npz file.
  \param[in] fname : Identifier for the corresponding array of data.
  \param[in] data_str : C++ std::string data.
  \param[in] mode : Writing mode, i.e. overwrite (w) or append (a) to the file.
  \warning This function should also work on big-endian platform, without guarantee since it has not been tested extensively.
  \note Original library: <a href="https://github.com/rogersce/cnpy">cnpy</a> with MIT license.

  \sa To see how to use it, you may have a look at \ref tutorial-npz
 */
void visp::cnpy::npz_save(const std::string &zipname, const std::string &fname, const std::string &data_str, const std::string &mode)
{
  std::vector<std::string> data_vec;
  data_vec.push_back(data_str);
  std::vector<size_t> shape { 1 };
  npz_save(zipname, fname, data_vec, shape, mode);
}

#endif
