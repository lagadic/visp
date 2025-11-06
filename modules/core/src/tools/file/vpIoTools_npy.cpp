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
    else {
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
    else {
      std::reverse(data_holder->begin(), data_holder->begin() + word_size);
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

#endif
