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
 * Backend for portable image format I/O operations.
 */

/*!
  \file vpImageIoPortable.cpp
  \brief Backend for portable image format I/O operations.
*/

#include "vpImageIoBackend.h"
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpEndian.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*!
 * Decode the PNM image header.
 * \param filename[in] : File name.
 * \param fd[in] : File desdcriptor.
 * \param magic[in] : Magic number for identifying the file type.
 * \param w[out] : Image width.
 * \param h[out] : Image height.
 * \param maxval[out] : Maximum pixel value.
 */
  void vp_decodeHeaderPNM(const std::string &filename, std::ifstream &fd, const std::string &magic, unsigned int &w,
                          unsigned int &h, unsigned int &maxval)
{
  std::string line;
  unsigned int nb_elt = 4, cpt_elt = 0;
  while (cpt_elt != nb_elt) {
    // Skip empty lines or lines starting with # (comment)
    while (std::getline(fd, line) && (line.compare(0, 1, "#") == 0 || line.size() == 0)) {
    }

    if (fd.eof()) {
      fd.close();
      throw(vpImageException(vpImageException::ioError, "Cannot read header of file \"%s\"", filename.c_str()));
    }

    std::vector<std::string> header = vpIoTools::splitChain(line, std::string(" "));

    if (header.size() == 0) {
      fd.close();
      throw(vpImageException(vpImageException::ioError, "Cannot read header of file \"%s\"", filename.c_str()));
    }

    if (cpt_elt == 0) { // decode magic
      if (header[0].compare(0, magic.size(), magic) != 0) {
        fd.close();
        throw(vpImageException(vpImageException::ioError, "\"%s\" is not a PNM file with magic number %s",
                               filename.c_str(), magic.c_str()));
      }
      ++cpt_elt;
      header.erase(header.begin(),
                   header.begin() + 1); // erase first element that is processed
    }
    while (header.size()) {
      if (cpt_elt == 1) { // decode width
        std::istringstream ss(header[0]);
        ss >> w;
        ++cpt_elt;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
      else if (cpt_elt == 2) {          // decode height
        std::istringstream ss(header[0]);
        ss >> h;
        ++cpt_elt;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
      else if (cpt_elt == 3) {          // decode maxval
        std::istringstream ss(header[0]);
        ss >> maxval;
        ++cpt_elt;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
    }
  }
}

void vp_decodeHeaderPFM(const std::string &filename, std::ifstream &fd, std::string &magic, unsigned int &w,
                        unsigned int &h, double &scale, bool &littleEndian)
{
  std::string line;
  const unsigned int nb_elt = 4;
  unsigned int cpt_elt = 0;
  while (cpt_elt != nb_elt) {
    // Skip empty lines or lines starting with # (comment)
    while (std::getline(fd, line) && (line.compare(0, 1, "#") == 0 || line.size() == 0)) {
    }

    if (fd.eof()) {
      fd.close();
      throw(vpImageException(vpImageException::ioError, "Cannot read header of file \"%s\"", filename.c_str()));
    }

    std::vector<std::string> header = vpIoTools::splitChain(line, std::string(" "));

    if (header.empty()) {
      fd.close();
      throw(vpImageException(vpImageException::ioError, "Cannot read header of file \"%s\"", filename.c_str()));
    }

    if (cpt_elt == 0) { // decode magic
      magic = header[0];
      if (magic != "PF" && magic != "Pf") {
        fd.close();
        throw(vpImageException(vpImageException::ioError,
                               "\"%s\" is not a PFM file with PF (RGB) or Pf (gray) magic number", filename.c_str()));
      }
      ++cpt_elt;
      header.erase(header.begin(),
                   header.begin() + 1); // erase first element that is processed
    }
    while (header.size()) {
      if (cpt_elt == 1) { // decode width
        std::istringstream ss(header[0]);
        ss >> w;
        ++cpt_elt;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
      else if (cpt_elt == 2) {          // decode height
        std::istringstream ss(header[0]);
        ss >> h;
        ++cpt_elt;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
      else if (cpt_elt == 3) {          // decode byte order
        std::istringstream ss(header[0]);
        ss >> scale;
        littleEndian = scale < 0;
        ++cpt_elt;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
    }
  }
}
#endif

//--------------------------------------------------------------------------
// PFM
//--------------------------------------------------------------------------

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function is built like portable gray pixmap (eg PGM P5) file.
  but considers float image data.

  \param I : Image to save as a (PFM P8) file.
  \param filename : Name of the file containing the image.
*/
void vp_writePFM(const vpImage<float> &I, const std::string &filename)
{
  FILE *fd;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot write PFM image: filename empty"));
  }

  fd = fopen(filename.c_str(), "wb");

  if (fd == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PFM file \"%s\"", filename.c_str()));
  }

  // Write the head
  fprintf(fd, "P8\n");                                 // Magic number
  fprintf(fd, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
  fprintf(fd, "255\n");                                // Max level

  // Write the bitmap
  size_t ierr;
  size_t nbyte = I.getWidth() * I.getHeight();

  ierr = fwrite(I.bitmap, sizeof(float), nbyte, fd);
  if (ierr != nbyte) {
    fclose(fd);
    throw(vpImageException(vpImageException::ioError, "Cannot save PFM file \"%s\": only %d bytes over %d saved ",
                           filename.c_str(), ierr, nbyte));
  }

  fflush(fd);
  fclose(fd);
}

void vp_writePFM_HDR(const vpImage<float> &I, const std::string &filename)
{
  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot write PFM image: filename empty"));
  }

  FILE *fd = fopen(filename.c_str(), "wb");
  if (fd == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PFM file \"%s\"", filename.c_str()));
  }

  // Write the head
  fprintf(fd, "Pf\n");                                 // Magic number
  fprintf(fd, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
#ifdef VISP_LITTLE_ENDIAN
  fprintf(fd, "%f\n", -1.0f);                          // Byte order
#else
  fprintf(fd, "%f\n", 1.0f);                           // Byte order
#endif

  // Write the bitmap
  size_t nbyte = I.getWidth();
  for (int i = static_cast<int>(I.getHeight()) - 1; i >= 0; i--) {
    size_t ierr = fwrite(I[i], sizeof(float), nbyte, fd);
    if (ierr != nbyte) {
      fclose(fd);
      throw(vpImageException(vpImageException::ioError, "Cannot save PFM file \"%s\": only %d bytes over %d saved ",
                             filename.c_str(), ierr, nbyte));
    }
  }

  fflush(fd);
  fclose(fd);
}

void vp_writePFM_HDR(const vpImage<vpRGBf> &I, const std::string &filename)
{
  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot write PFM image: filename empty"));
  }

  FILE *fd = fopen(filename.c_str(), "wb");
  if (fd == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PFM file \"%s\"", filename.c_str()));
  }

  // Write the head
  fprintf(fd, "PF\n");                                 // Magic number
  fprintf(fd, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
#ifdef VISP_LITTLE_ENDIAN
  fprintf(fd, "%f\n", -1.0f);                          // Byte order
#else
  fprintf(fd, "%f\n", 1.0f);                           // Byte order
#endif

  // Write the bitmap
  size_t nbyte = I.getWidth() * 3;
  for (int i = static_cast<int>(I.getHeight()) - 1; i >= 0; i--) {
    size_t ierr = fwrite(I[i], sizeof(float), nbyte, fd);
    if (ierr != nbyte) {
      fclose(fd);
      throw(vpImageException(vpImageException::ioError, "Cannot save PFM file \"%s\": only %d bytes over %d saved ",
                             filename.c_str(), ierr, nbyte));
    }
  }

  fflush(fd);
  fclose(fd);
}

//--------------------------------------------------------------------------
// PGM
//--------------------------------------------------------------------------

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \param I : Image to save as a (PGM P5) file.
  \param filename : Name of the file containing the image.
*/
void vp_writePGM(const vpImage<unsigned char> &I, const std::string &filename)
{
  FILE *fd;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PGM file: filename empty"));
  }

  fd = fopen(filename.c_str(), "wb");

  if (fd == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PGM file \"%s\"", filename.c_str()));
  }

  // Write the head
  fprintf(fd, "P5\n");                                 // Magic number
  fprintf(fd, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
  fprintf(fd, "255\n");                                // Max level

  // Write the bitmap
  size_t ierr;
  size_t nbyte = I.getWidth() * I.getHeight();

  ierr = fwrite(I.bitmap, sizeof(unsigned char), nbyte, fd);
  if (ierr != nbyte) {
    fclose(fd);
    throw(vpImageException(vpImageException::ioError, "Cannot save PGM file \"%s\": only %d over %d bytes saved",
                           filename.c_str(), ierr, nbyte));
  }

  fflush(fd);
  fclose(fd);
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \param I : Image to save as a (PGM P5) file.
  \param filename : Name of the file containing the image.
*/
void vp_writePGM(const vpImage<short> &I, const std::string &filename)
{
  vpImage<unsigned char> Iuc;
  unsigned int nrows = I.getHeight();
  unsigned int ncols = I.getWidth();

  Iuc.resize(nrows, ncols);

  for (unsigned int i = 0; i < nrows * ncols; ++i)
    Iuc.bitmap[i] = (unsigned char)I.bitmap[i];

  vp_writePGM(Iuc, filename);
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.
  Color image is converted into a grayscale image.

  \param I : Image to save as a (PGM P5) file.
  \param filename : Name of the file containing the image.
*/
void vp_writePGM(const vpImage<vpRGBa> &I, const std::string &filename)
{

  FILE *fd;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PGM file: filename empty"));
  }

  fd = fopen(filename.c_str(), "wb");

  if (fd == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PGM file \"%s\"", filename.c_str()));
  }

  // Write the head
  fprintf(fd, "P5\n");                                 // Magic number
  fprintf(fd, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
  fprintf(fd, "255\n");                                // Max level

  // Write the bitmap
  size_t ierr;
  size_t nbyte = I.getWidth() * I.getHeight();

  vpImage<unsigned char> Itmp;
  vpImageConvert::convert(I, Itmp);

  ierr = fwrite(Itmp.bitmap, sizeof(unsigned char), nbyte, fd);
  if (ierr != nbyte) {
    fclose(fd);
    throw(vpImageException(vpImageException::ioError, "Cannot save PGM file \"%s\": only %d over %d bytes saved",
                           filename.c_str(), ierr, nbyte));
  }

  fflush(fd);
  fclose(fd);
}

/*!
  Read a PFM P8 file and initialize a float image.

  Read the contents of the portable gray pixmap (PFM P8) filename, allocate
  memory for the corresponding image, and set the bitmap with the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPFM(vpImage<float> &I, const std::string &filename)
{
  unsigned int w = 0, h = 0, maxval = 0;
  const unsigned int w_max = 100000, h_max = 100000, maxval_max = 255;
  const std::string magic("P8");

  std::ifstream fd(filename.c_str(), std::ios::binary);

  // Open the filename
  if (!fd.is_open()) {
    throw(vpImageException(vpImageException::ioError, "Cannot open file \"%s\"", filename.c_str()));
  }

  vp_decodeHeaderPNM(filename, fd, magic, w, h, maxval);

  if (w > w_max || h > h_max) {
    fd.close();
    throw(vpException(vpException::badValue, "Bad image size in \"%s\"", filename.c_str()));
  }
  if (maxval > maxval_max) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Bad maxval in \"%s\"", filename.c_str()));
  }

  if ((h != I.getHeight()) || (w != I.getWidth())) {
    I.resize(h, w);
  }

  unsigned int nbyte = I.getHeight() * I.getWidth();
  fd.read((char *)I.bitmap, sizeof(float) * nbyte);
  if (!fd) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Read only %d of %d bytes in file \"%s\"", fd.gcount(), nbyte,
                           filename.c_str()));
  }

  fd.close();
}

/*!
  Read a PFM Pf file and initialize a float image.

  Read the contents of the portable gray pixmap (PFM Pf) filename, allocate
  memory for the corresponding image, and set the bitmap with the content of
  the file.
  Ranges of the pixels are not restricted to the [0 - 255] range.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPFM_HDR(vpImage<float> &I, const std::string &filename)
{
  std::ifstream fd(filename.c_str(), std::ios::binary);

  // Open the filename
  if (!fd.is_open()) {
    throw(vpImageException(vpImageException::ioError, "Cannot open file \"%s\"", filename.c_str()));
  }

  const unsigned int w_max = 100000, h_max = 100000;
  const std::string magicRGB("PF"), magicGray("Pf");
  std::string magic;
  unsigned int w = 0, h = 0;
  double scale = 1;
  bool littleEndian = true;
  vp_decodeHeaderPFM(filename, fd, magic, w, h, scale, littleEndian);

  if (w > w_max || h > h_max) {
    fd.close();
    throw(vpException(vpException::badValue, "Bad image size in \"%s\"", filename.c_str()));
  }

  unsigned int channels = (magic == magicRGB) ? 3 : 1;
  if (h != I.getHeight() || channels * w != I.getWidth()) {
    I.resize(h, channels * w);
  }

#ifdef VISP_LITTLE_ENDIAN
  bool swapEndianness = !littleEndian;
#else
  bool swapEndianness = littleEndian;
#endif
  for (int i = I.getHeight() - 1; i >= 0; i--) {
    fd.read((char *)I[i], sizeof(float) * w * channels);
    if (swapEndianness) {
      for (unsigned int j = 0; j < w * channels; ++j) {
        I[i][j] = vpEndian::swapFloat(I[i][j]);
      }
    }
  }

  if (!fd) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Read only %d bytes in file \"%s\"", fd.gcount(),
                           filename.c_str()));
  }
  fd.close();

  if (std::fabs(scale) > 0.0f) {
    for (unsigned int i = 0; i < I.getHeight(); ++i) {
      for (unsigned int j = 0; j < I.getWidth(); ++j) {
        I[i][j] *= 1.0f / static_cast<float>(std::fabs(scale));
      }
    }
  }
}

/*!
  Read a PFM PF file and initialize a 3-channels float image.

  Read the contents of the portable color pixmap (PFM PF) filename, allocate
  memory for the corresponding image, and set the bitmap with the content of
  the file.
  Ranges of the pixels are not restricted to the [0 - 255] range.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPFM_HDR(vpImage<vpRGBf> &I, const std::string &filename)
{
  std::ifstream fd(filename.c_str(), std::ios::binary);

  // Open the filename
  if (!fd.is_open()) {
    throw(vpImageException(vpImageException::ioError, "Cannot open file \"%s\"", filename.c_str()));
  }

  const unsigned int w_max = 100000, h_max = 100000;
  const std::string magicRGB("PF"), magicGray("Pf");
  std::string magic;
  unsigned int w = 0, h = 0;
  double scale = 1;
  bool littleEndian = true;
  vp_decodeHeaderPFM(filename, fd, magic, w, h, scale, littleEndian);

  if (w > w_max || h > h_max) {
    fd.close();
    throw(vpException(vpException::badValue, "Bad image size in \"%s\"", filename.c_str()));
  }

  unsigned int channels = (magic == magicRGB) ? 3 : 1;
  if (magic != magicRGB) {
    throw(vpImageException(vpImageException::ioError, "Image \"%s\" is not an RGB image!", filename.c_str()));
  }
  if (h != I.getHeight() || w != I.getWidth()) {
    I.resize(h, w);
  }

#ifdef VISP_LITTLE_ENDIAN
  bool swapEndianness = !littleEndian;
#else
  bool swapEndianness = littleEndian;
#endif
  for (int i = I.getHeight() - 1; i >= 0; i--) {
    fd.read((char *)I[i], sizeof(float) * w * channels);
    if (swapEndianness) {
      for (unsigned int j = 0; j < w; ++j) {
        I[i][j].R = vpEndian::swapFloat(I[i][j].R);
        I[i][j].G = vpEndian::swapFloat(I[i][j].G);
        I[i][j].B = vpEndian::swapFloat(I[i][j].B);
      }
    }
  }

  if (!fd) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Read only %d bytes in file \"%s\"", fd.gcount(),
                           filename.c_str()));
  }
  fd.close();

  if (std::fabs(scale) > 0.0f) {
    for (unsigned int i = 0; i < I.getHeight(); ++i) {
      for (unsigned int j = 0; j < I.getWidth(); ++j) {
        I[i][j].R *= 1.0f / static_cast<float>(std::fabs(scale));
        I[i][j].G *= 1.0f / static_cast<float>(std::fabs(scale));
        I[i][j].B *= 1.0f / static_cast<float>(std::fabs(scale));
      }
    }
  }
}

/*!
  Read a PGM P5 file and initialize a scalar image.

  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap with the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPGM(vpImage<unsigned char> &I, const std::string &filename)
{
  unsigned int w = 0, h = 0, maxval = 0;
  unsigned int w_max = 100000, h_max = 100000, maxval_max = 255;
  std::string magic("P5");

  std::ifstream fd(filename.c_str(), std::ios::binary);

  // Open the filename
  if (!fd.is_open()) {
    throw(vpImageException(vpImageException::ioError, "Cannot open file \"%s\"", filename.c_str()));
  }

  vp_decodeHeaderPNM(filename, fd, magic, w, h, maxval);

  if (w > w_max || h > h_max) {
    fd.close();
    throw(vpException(vpException::badValue, "Bad image size in \"%s\"", filename.c_str()));
  }
  if (maxval > maxval_max) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Bad maxval in \"%s\"", filename.c_str()));
  }

  if ((h != I.getHeight()) || (w != I.getWidth())) {
    I.resize(h, w);
  }

  unsigned int nbyte = I.getHeight() * I.getWidth();
  fd.read((char *)I.bitmap, nbyte);
  if (!fd) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Read only %d of %d bytes in file \"%s\"", fd.gcount(), nbyte,
                           filename.c_str()));
  }

  fd.close();
}

/*!
  Read a PGM P5 file and initialize a scalar image.

  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap with the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  The gray level image contained in the \e filename is converted in a
  color image in \e I.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPGM(vpImage<vpRGBa> &I, const std::string &filename)
{
  vpImage<unsigned char> Itmp;

  vp_readPGM(Itmp, filename);

  vpImageConvert::convert(Itmp, I);
}

//--------------------------------------------------------------------------
// PPM
//--------------------------------------------------------------------------

/*!
  Read the contents of the portable pixmap (PPM P6) filename, allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap with the gray level data. That means that the image \e I is
  a "black and white" rendering of the original image in \e filename, as in a
  black and white photograph. The quantization formula used is \f$0,299 r +
  0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPPM(vpImage<unsigned char> &I, const std::string &filename)
{
  vpImage<vpRGBa> Itmp;

  vp_readPPM(Itmp, filename);

  vpImageConvert::convert(Itmp, I);
}

/*!
  Read the contents of the portable pixmap (PPM P6) filename,
  allocate memory for the corresponding vpRGBa image.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vp_readPPM(vpImage<vpRGBa> &I, const std::string &filename)
{
  unsigned int w = 0, h = 0, maxval = 0;
  unsigned int w_max = 100000, h_max = 100000, maxval_max = 255;
  std::string magic("P6");

  std::ifstream fd(filename.c_str(), std::ios::binary);

  // Open the filename
  if (!fd.is_open()) {
    throw(vpImageException(vpImageException::ioError, "Cannot open file \"%s\"", filename.c_str()));
  }

  vp_decodeHeaderPNM(filename, fd, magic, w, h, maxval);

  if (w > w_max || h > h_max) {
    fd.close();
    throw(vpException(vpException::badValue, "Bad image size in \"%s\"", filename.c_str()));
  }
  if (maxval > maxval_max) {
    fd.close();
    throw(vpImageException(vpImageException::ioError, "Bad maxval in \"%s\"", filename.c_str()));
  }

  if ((h != I.getHeight()) || (w != I.getWidth())) {
    I.resize(h, w);
  }

  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      unsigned char rgb[3];
      fd.read((char *)&rgb, 3);

      if (!fd) {
        fd.close();
        throw(vpImageException(vpImageException::ioError, "Read only %d of %d bytes in file \"%s\"",
                               (i * I.getWidth() + j) * 3 + fd.gcount(), I.getSize() * 3, filename.c_str()));
      }

      I[i][j].R = rgb[0];
      I[i][j].G = rgb[1];
      I[i][j].B = rgb[2];
      I[i][j].A = vpRGBa::alpha_default;
    }
  }

  fd.close();
}

/*!
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PPM P6) file.
  grayscale image is converted into a color image vpRGBa.

  \param I : Image to save as a (PPM P6) file.
  \param filename : Name of the file containing the image.
*/
void vp_writePPM(const vpImage<unsigned char> &I, const std::string &filename)
{
  vpImage<vpRGBa> Itmp;

  vpImageConvert::convert(I, Itmp);

  vp_writePPM(Itmp, filename);
}

/*!
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PPM P6) file.

  \param I : Image to save as a (PPM P6) file.
  \param filename : Name of the file containing the image.
*/
void vp_writePPM(const vpImage<vpRGBa> &I, const std::string &filename)
{
  FILE *f;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PPM file: filename empty"));
  }

  f = fopen(filename.c_str(), "wb");

  if (f == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PPM file \"%s\"", filename.c_str()));
  }

  fprintf(f, "P6\n");                                 // Magic number
  fprintf(f, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
  fprintf(f, "%d\n", 255);                            // Max level

  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      vpRGBa v = I[i][j];
      unsigned char rgb[3];
      rgb[0] = v.R;
      rgb[1] = v.G;
      rgb[2] = v.B;

      size_t res = fwrite(&rgb, 1, 3, f);
      if (res != 3) {
        fclose(f);
        throw(vpImageException(vpImageException::ioError, "cannot write file \"%s\"", filename.c_str()));
      }
    }
  }

  fflush(f);
  fclose(f);
}

END_VISP_NAMESPACE
