/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Read/write images.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpImageIo.cpp
  \brief Read/write images
*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h> //image  conversion
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

void vp_decodeHeaderPNM(const std::string &filename, std::ifstream &fd, const std::string &magic, unsigned int &w,
                        unsigned int &h, unsigned int &maxval);

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
    };

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
      cpt_elt++;
      header.erase(header.begin(),
                   header.begin() + 1); // erase first element that is processed
    }
    while (header.size()) {
      if (cpt_elt == 1) { // decode width
        std::istringstream ss(header[0]);
        ss >> w;
        cpt_elt++;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      } else if (cpt_elt == 2) {          // decode height
        std::istringstream ss(header[0]);
        ss >> h;
        cpt_elt++;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      } else if (cpt_elt == 3) {          // decode maxval
        std::istringstream ss(header[0]);
        ss >> maxval;
        cpt_elt++;
        header.erase(header.begin(),
                     header.begin() + 1); // erase first element that is processed
      }
    }
  }
}
#endif

vpImageIo::vpImageFormatType vpImageIo::getFormat(const std::string &filename)
{
  std::string ext = vpImageIo::getExtension(filename);

  if (ext.compare(".PGM") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".pgm") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".PPM") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".ppm") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".JPG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".JPEG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpeg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".PNG") == 0)
    return FORMAT_PNG;
  else if (ext.compare(".png") == 0)
    return FORMAT_PNG;
  // Formats supported by opencv
  else if (ext.compare(".TIFF") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".tiff") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".TIF") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".tif") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".BMP") == 0)
    return FORMAT_BMP;
  else if (ext.compare(".bmp") == 0)
    return FORMAT_BMP;
  else if (ext.compare(".DIB") == 0)
    return FORMAT_DIB;
  else if (ext.compare(".dib") == 0)
    return FORMAT_DIB;
  else if (ext.compare(".PBM") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".pbm") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".SR") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".sr") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".RAS") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".ras") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".JP2") == 0)
    return FORMAT_JPEG2000;
  else if (ext.compare(".jp2") == 0)
    return FORMAT_JPEG2000;
  else
    return FORMAT_UNKNOWN;
}

// return the extension of the file including the dot
std::string vpImageIo::getExtension(const std::string &filename)
{
  // extract the extension
  size_t dot = filename.find_last_of(".");
  std::string ext = filename.substr(dot, filename.size() - 1);
  return ext;
}

/*!
  Read the contents of the image filename, allocate memory for the
  corresponding greyscale image, update its content, and return a reference to
  the image.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  Always supported formats are *.pgm and *.ppm.
  If \c libjpeg 3rd party is used, we support also *.jpg and *.jpeg files.
  If \c libpng 3rd party is used, we support also *.png files.
  If OpenCV 3rd party is used, we support *.jpg, *.jpeg, *.jp2, *.rs, *.ras,
  *.tiff, *.tif, *.png, *.bmp, *.pbm files.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
 */
void vpImageIo::read(vpImage<unsigned char> &I, const std::string &filename)
{
  bool exist = vpIoTools::checkFilename(filename);
  if (!exist) {
    std::string message = "Cannot read file: \"" + std::string(filename) + "\" doesn't exist";
    throw(vpImageException(vpImageException::ioError, message));
  }

  // Allows to use ~ symbol or env variables in path
  std::string final_filename = vpIoTools::path(filename);

  bool try_opencv_reader = false;

  switch (getFormat(final_filename)) {
  case FORMAT_PGM:
    readPGM(I, final_filename);
    break;
  case FORMAT_PPM:
    readPPM(I, final_filename);
    break;
  case FORMAT_JPEG:
#ifdef VISP_HAVE_JPEG
    readJPEG(I, final_filename);
#else
    try_opencv_reader = true;
#endif
    break;
  case FORMAT_PNG:
#if defined(VISP_HAVE_PNG)
    readPNG(I, final_filename);
#else
    try_opencv_reader = true;
#endif
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_reader = true;
    break;
  }

  if (try_opencv_reader) {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    // std::cout << "Use opencv to read the image" << std::endl;
    cv::Mat cvI = cv::imread(final_filename, cv::IMREAD_GRAYSCALE);
    if (cvI.cols == 0 && cvI.rows == 0) {
      std::string message = "Cannot read file \"" + std::string(final_filename) + "\": Image format not supported";
      throw(vpImageException(vpImageException::ioError, message));
    }
    vpImageConvert::convert(cvI, I);
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    // std::cout << "Use opencv to read the image" << std::endl;
    cv::Mat cvI = cv::imread(final_filename, CV_LOAD_IMAGE_GRAYSCALE);
    if (cvI.cols == 0 && cvI.rows == 0) {
      std::string message = "Cannot read file \"" + std::string(final_filename) + "\": Image format not supported";
      throw(vpImageException(vpImageException::ioError, message));
    }
    vpImageConvert::convert(cvI, I);
#else
    std::string message = "Cannot read file \"" + std::string(final_filename) + "\": Image format not supported";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Read the contents of the image filename, allocate memory for the
  corresponding color image, update its content, and return a reference to the
  image.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  Always supported formats are *.pgm and *.ppm.
  If \c libjpeg 3rd party is used, we support also *.jpg and *.jpeg files.
  If \c libpng 3rd party is used, we support also *.png files.
  If OpenCV 3rd party is used, we support *.jpg, *.jpeg, *.jp2, *.rs, *.ras,
  *.tiff, *.tif, *.png, *.bmp, *.pbm files.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
 */
void vpImageIo::read(vpImage<vpRGBa> &I, const std::string &filename)
{
  bool exist = vpIoTools::checkFilename(filename);
  if (!exist) {
    std::string message = "Cannot read file: \"" + std::string(filename) + "\" doesn't exist";
    throw(vpImageException(vpImageException::ioError, message));
  }
  // Allows to use ~ symbol or env variables in path
  std::string final_filename = vpIoTools::path(filename);

  bool try_opencv_reader = false;

  switch (getFormat(final_filename)) {
  case FORMAT_PGM:
    readPGM(I, final_filename);
    break;
  case FORMAT_PPM:
    readPPM(I, final_filename);
    break;
  case FORMAT_JPEG:
#ifdef VISP_HAVE_JPEG
    readJPEG(I, final_filename);
#else
    try_opencv_reader = true;
#endif
    break;
  case FORMAT_PNG:
#if defined(VISP_HAVE_PNG)
    readPNG(I, final_filename);
#else
    try_opencv_reader = true;
#endif
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_reader = true;
    break;
  }

  if (try_opencv_reader) {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    // std::cout << "Use opencv to read the image" << std::endl;
    cv::Mat cvI = cv::imread(final_filename, cv::IMREAD_COLOR);
    if (cvI.cols == 0 && cvI.rows == 0) {
      std::string message = "Cannot read file \"" + std::string(final_filename) + "\": Image format not supported";
      throw(vpImageException(vpImageException::ioError, message));
    }
    vpImageConvert::convert(cvI, I);
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    // std::cout << "Use opencv to read the image" << std::endl;
    cv::Mat cvI = cv::imread(final_filename, CV_LOAD_IMAGE_COLOR);
    if (cvI.cols == 0 && cvI.rows == 0) {
      std::string message = "Cannot read file \"" + std::string(final_filename) + "\": Image format not supported";
      throw(vpImageException(vpImageException::ioError, message));
    }
    vpImageConvert::convert(cvI, I);
#else
    std::string message = "Cannot read file \"" + std::string(final_filename) + "\": Image format not supported";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Write the content of the image in the file which name is given by \e
  filename.

  Always supported formats are *.pgm and *.ppm.
  If \c libjpeg 3rd party is used, we support also *.jpg and *.jpeg files.
  If \c libpng 3rd party is used, we support also *.png files.
  If OpenCV 3rd party is used, we support *.jpg, *.jpeg, *.jp2, *.rs, *.ras,
  *.tiff, *.tif, *.png, *.bmp, *.pbm files.

  \param I : Image to write.
  \param filename : Name of the file containing the image.
 */
void vpImageIo::write(const vpImage<unsigned char> &I, const std::string &filename)
{
  bool try_opencv_writer = false;

  switch (getFormat(filename)) {
  case FORMAT_PGM:
    writePGM(I, filename);
    break;
  case FORMAT_PPM:
    writePPM(I, filename);
    break;
  case FORMAT_JPEG:
#ifdef VISP_HAVE_JPEG
    writeJPEG(I, filename);
#else
    try_opencv_writer = true;
#endif
    break;
  case FORMAT_PNG:
#ifdef VISP_HAVE_PNG
    writePNG(I, filename);
#else
    try_opencv_writer = true;
#endif
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_writer = true;
    break;
  }

  if (try_opencv_writer) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    // std::cout << "Use opencv to write the image" << std::endl;
    cv::Mat cvI;
    vpImageConvert::convert(I, cvI);
    cv::imwrite(filename, cvI);
#else
    vpCERROR << "Cannot write file: Image format not supported..." << std::endl;
    throw(vpImageException(vpImageException::ioError, "Cannot write file: Image format not supported"));
#endif
  }
}

/*!
  Write the content of the image in the file which name is given by \e
  filename.

  Always supported formats are *.pgm and *.ppm.
  If \c libjpeg 3rd party is used, we support also *.jpg and *.jpeg files.
  If \c libpng 3rd party is used, we support also *.png files.
  If OpenCV 3rd party is used, we support *.jpg, *.jpeg, *.jp2, *.rs, *.ras,
  *.tiff, *.tif, *.png, *.bmp, *.pbm files.

  \param I : Image to write.
  \param filename : Name of the file containing the image.
 */
void vpImageIo::write(const vpImage<vpRGBa> &I, const std::string &filename)
{
  bool try_opencv_writer = false;

  switch (getFormat(filename)) {
  case FORMAT_PGM:
    writePGM(I, filename);
    break;
  case FORMAT_PPM:
    writePPM(I, filename);
    break;
  case FORMAT_JPEG:
#ifdef VISP_HAVE_JPEG
    writeJPEG(I, filename);
#else
    try_opencv_writer = true;
#endif
    break;
  case FORMAT_PNG:
#ifdef VISP_HAVE_PNG
    writePNG(I, filename);
#else
    try_opencv_writer = true;
#endif
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_writer = true;
    break;
  }

  if (try_opencv_writer) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    // std::cout << "Use opencv to write the image" << std::endl;
    cv::Mat cvI;
    vpImageConvert::convert(I, cvI);
    cv::imwrite(filename, cvI);
#else
    vpCERROR << "Cannot write file: Image format not supported..." << std::endl;
    throw(vpImageException(vpImageException::ioError, "Cannot write file: Image format not supported"));
#endif
  }
}

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

void vpImageIo::writePFM(const vpImage<float> &I, const std::string &filename)
{
  FILE *fd;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot write PFM image: filename empty"));
  }

  fd = fopen(filename.c_str(), "wb");

  if (fd == NULL) {
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
//--------------------------------------------------------------------------
// PGM
//--------------------------------------------------------------------------

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.

  \param I : Image to save as a (PGM P5) file.
  \param filename : Name of the file containing the image.
*/

void vpImageIo::writePGM(const vpImage<unsigned char> &I, const std::string &filename)
{

  FILE *fd;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PGM file: filename empty"));
  }

  fd = fopen(filename.c_str(), "wb");

  if (fd == NULL) {
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
void vpImageIo::writePGM(const vpImage<short> &I, const std::string &filename)
{
  vpImage<unsigned char> Iuc;
  unsigned int nrows = I.getHeight();
  unsigned int ncols = I.getWidth();

  Iuc.resize(nrows, ncols);

  for (unsigned int i = 0; i < nrows * ncols; i++)
    Iuc.bitmap[i] = (unsigned char)I.bitmap[i];

  vpImageIo::writePGM(Iuc, filename);
}
/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PGM P5) file.
  Color image is converted into a grayscale image.

  \param I : Image to save as a (PGM P5) file.
  \param filename : Name of the file containing the image.
*/

void vpImageIo::writePGM(const vpImage<vpRGBa> &I, const std::string &filename)
{

  FILE *fd;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PGM file: filename empty"));
  }

  fd = fopen(filename.c_str(), "wb");

  if (fd == NULL) {
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
  memory for the corresponding image, and set the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/

void vpImageIo::readPFM(vpImage<float> &I, const std::string &filename)
{
  unsigned int w = 0, h = 0, maxval = 0;
  unsigned int w_max = 100000, h_max = 100000, maxval_max = 255;
  std::string magic("P8");

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
  Read a PGM P5 file and initialize a scalar image.

  Read the contents of the portable gray pixmap (PGM P5) filename, allocate
  memory for the corresponding image, and set the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/

void vpImageIo::readPGM(vpImage<unsigned char> &I, const std::string &filename)
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
  memory for the corresponding image, and set the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  The gray level image contained in the \e filename is converted in a
  color image in \e I.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/

void vpImageIo::readPGM(vpImage<vpRGBa> &I, const std::string &filename)
{
  vpImage<unsigned char> Itmp;

  vpImageIo::readPGM(Itmp, filename);

  vpImageConvert::convert(Itmp, I);
}

//--------------------------------------------------------------------------
// PPM
//--------------------------------------------------------------------------

/*!
  Read the contents of the portable pixmap (PPM P6) filename, allocate memory
  for the corresponding gray level image, convert the data in gray level, and
  set the bitmap whith the gray level data. That means that the image \e I is
  a "black and white" rendering of the original image in \e filename, as in a
  black and white photograph. The quantization formula used is \f$0,299 r +
  0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/
void vpImageIo::readPPM(vpImage<unsigned char> &I, const std::string &filename)
{
  vpImage<vpRGBa> Itmp;

  vpImageIo::readPPM(Itmp, filename);

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
void vpImageIo::readPPM(vpImage<vpRGBa> &I, const std::string &filename)
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

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
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

void vpImageIo::writePPM(const vpImage<unsigned char> &I, const std::string &filename)
{
  vpImage<vpRGBa> Itmp;

  vpImageConvert::convert(I, Itmp);

  vpImageIo::writePPM(Itmp, filename);
}

/*!
  Write the content of the bitmap in the file which name is given by \e
  filename. This function writes a portable gray pixmap (PPM P6) file.

  \param I : Image to save as a (PPM P6) file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writePPM(const vpImage<vpRGBa> &I, const std::string &filename)
{
  FILE *f;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PPM file: filename empty"));
  }

  f = fopen(filename.c_str(), "wb");

  if (f == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PPM file \"%s\"", filename.c_str()));
  }

  fprintf(f, "P6\n");                                 // Magic number
  fprintf(f, "%u %u\n", I.getWidth(), I.getHeight()); // Image size
  fprintf(f, "%d\n", 255);                            // Max level

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
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

//--------------------------------------------------------------------------
// JPEG
//--------------------------------------------------------------------------

#if defined(VISP_HAVE_JPEG)

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writeJPEG(const vpImage<unsigned char> &I, const std::string &filename)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file \"%s\"", filename.c_str()));
  }

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  jpeg_stdio_dest(&cinfo, file);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 1;
  cinfo.in_color_space = JCS_GRAYSCALE;
  jpeg_set_defaults(&cinfo);

  jpeg_start_compress(&cinfo, TRUE);

  unsigned char *line;
  line = new unsigned char[width];
  unsigned char *input = (unsigned char *)I.bitmap;
  while (cinfo.next_scanline < cinfo.image_height) {
    for (unsigned int i = 0; i < width; i++) {
      line[i] = *(input);
      input++;
    }
    jpeg_write_scanlines(&cinfo, &line, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  delete[] line;
  fclose(file);
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writeJPEG(const vpImage<vpRGBa> &I, const std::string &filename)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file \"%s\"", filename.c_str()));
  }

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  jpeg_stdio_dest(&cinfo, file);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);

  jpeg_start_compress(&cinfo, TRUE);

  unsigned char *line;
  line = new unsigned char[3 * width];
  unsigned char *input = (unsigned char *)I.bitmap;
  while (cinfo.next_scanline < cinfo.image_height) {
    for (unsigned int i = 0; i < width; i++) {
      line[i * 3] = *(input);
      input++;
      line[i * 3 + 1] = *(input);
      input++;
      line[i * 3 + 2] = *(input);
      input++;
      input++;
    }
    jpeg_write_scanlines(&cinfo, &line, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  delete[] line;
  fclose(file);
}

/*!
  Read the contents of the JPEG file, allocate memory
  for the corresponding gray level image, if necessary convert the data in
  gray level, and set the bitmap whith the gray level data. That means that
  the image \e I is a "black and white" rendering of the original image in \e
  filename, as in a black and white photograph. If necessary, the quantization
  formula used is \f$0,299 r + 0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/
void vpImageIo::readJPEG(vpImage<unsigned char> &I, const std::string &filename)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG file \"%s\"", filename.c_str()));
  }

  jpeg_stdio_src(&cinfo, file);
  jpeg_read_header(&cinfo, TRUE);

  unsigned int width = cinfo.image_width;
  unsigned int height = cinfo.image_height;

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  jpeg_start_decompress(&cinfo);

  unsigned int rowbytes = cinfo.output_width * (unsigned int)(cinfo.output_components);
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, rowbytes, 1);

  if (cinfo.out_color_space == JCS_RGB) {
    vpImage<vpRGBa> Ic(height, width);
    unsigned char *output = (unsigned char *)Ic.bitmap;
    while (cinfo.output_scanline < cinfo.output_height) {
      jpeg_read_scanlines(&cinfo, buffer, 1);
      for (unsigned int i = 0; i < width; i++) {
        *(output++) = buffer[0][i * 3];
        *(output++) = buffer[0][i * 3 + 1];
        *(output++) = buffer[0][i * 3 + 2];
        *(output++) = vpRGBa::alpha_default;
      }
    }
    vpImageConvert::convert(Ic, I);
  }

  else if (cinfo.out_color_space == JCS_GRAYSCALE) {
    while (cinfo.output_scanline < cinfo.output_height) {
      unsigned int row = cinfo.output_scanline;
      jpeg_read_scanlines(&cinfo, buffer, 1);
      memcpy(I[row], buffer[0], rowbytes);
    }
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(file);
}

/*!
  Read a JPEG file and initialize a scalar image.

  Read the contents of the JPEG file, allocate
  memory for the corresponding image, and set
  the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a grayscaled image, a conversion is done to deal
  with \e I which is a color image.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::readJPEG(vpImage<vpRGBa> &I, const std::string &filename)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG file \"%s\"", filename.c_str()));
  }

  jpeg_stdio_src(&cinfo, file);

  jpeg_read_header(&cinfo, TRUE);

  unsigned int width = cinfo.image_width;
  unsigned int height = cinfo.image_height;

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  jpeg_start_decompress(&cinfo);

  unsigned int rowbytes = cinfo.output_width * (unsigned int)(cinfo.output_components);
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, rowbytes, 1);

  if (cinfo.out_color_space == JCS_RGB) {
    unsigned char *output = (unsigned char *)I.bitmap;
    while (cinfo.output_scanline < cinfo.output_height) {
      jpeg_read_scanlines(&cinfo, buffer, 1);
      for (unsigned int i = 0; i < width; i++) {
        *(output++) = buffer[0][i * 3];
        *(output++) = buffer[0][i * 3 + 1];
        *(output++) = buffer[0][i * 3 + 2];
        *(output++) = vpRGBa::alpha_default;
      }
    }
  }

  else if (cinfo.out_color_space == JCS_GRAYSCALE) {
    vpImage<unsigned char> Ig(height, width);

    while (cinfo.output_scanline < cinfo.output_height) {
      unsigned int row = cinfo.output_scanline;
      jpeg_read_scanlines(&cinfo, buffer, 1);
      memcpy(Ig[row], buffer[0], rowbytes);
    }

    vpImageConvert::convert(Ig, I);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(file);
}

#elif defined(VISP_HAVE_OPENCV)

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writeJPEG(const vpImage<unsigned char> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);
  cv::imwrite(filename.c_str(), Ip);
#else
  IplImage *Ip = NULL;
  vpImageConvert::convert(I, Ip);

  cvSaveImage(filename.c_str(), Ip);

  cvReleaseImage(&Ip);
#endif
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writeJPEG(const vpImage<vpRGBa> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);
  cv::imwrite(filename.c_str(), Ip);
#else
  IplImage *Ip = NULL;
  vpImageConvert::convert(I, Ip);

  cvSaveImage(filename.c_str(), Ip);

  cvReleaseImage(&Ip);
#endif
}

/*!
  Read the contents of the JPEG file, allocate memory
  for the corresponding gray level image, if necessary convert the data in
  gray level, and set the bitmap whith the gray level data. That means that
  the image \e I is a "black and white" rendering of the original image in \e
  filename, as in a black and white photograph. If necessary, the quantization
  formula used is \f$0,299 r + 0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/
void vpImageIo::readJPEG(vpImage<unsigned char> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  cv::Mat Ip = cv::imread(filename.c_str(), cv::IMREAD_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  IplImage *Ip = NULL;
  Ip = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (Ip != NULL)
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
  cvReleaseImage(&Ip);
#endif
}

/*!
  Read a JPEG file and initialize a scalar image.

  Read the contents of the JPEG file, allocate
  memory for the corresponding image, and set
  the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a grayscaled image, a conversion is done to deal
  with \e I which is a color image.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::readJPEG(vpImage<vpRGBa> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  cv::Mat Ip = cv::imread(filename.c_str(), cv::IMREAD_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  IplImage *Ip = NULL;
  Ip = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR);
  if (Ip != NULL)
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
  cvReleaseImage(&Ip);
#endif
}

#endif

//--------------------------------------------------------------------------
// PNG
//--------------------------------------------------------------------------

#if defined(VISP_HAVE_PNG)

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \param I : Image to save as a PNG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writePNG(const vpImage<unsigned char> &I, const std::string &filename)
{
  FILE *file;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file \"%s\"", filename.c_str()));
  }

  /* create a png info struct */
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr) {
    fclose(file);
    vpERROR_TRACE("Error during png_create_write_struct()\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, NULL);
    vpERROR_TRACE("Error during png_create_info_struct()\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* initialize the setjmp for returning properly after a libpng error occured
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    vpERROR_TRACE("Error during init_io\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* setup libpng for using standard C fwrite() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();
  int bit_depth = 8;
  int color_type = PNG_COLOR_TYPE_GRAY;
  /* set some useful information from header */

  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    vpERROR_TRACE("Error during write header\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);

  png_bytep *row_ptrs = new png_bytep[height];
  for (unsigned int i = 0; i < height; i++)
    row_ptrs[i] = new png_byte[width];

  unsigned char *input = (unsigned char *)I.bitmap;

  for (unsigned int i = 0; i < height; i++) {
    png_byte *row = row_ptrs[i];
    for (unsigned int j = 0; j < width; j++) {
      row[j] = *(input);
      input++;
    }
  }

  png_write_image(png_ptr, row_ptrs);

  png_write_end(png_ptr, NULL);

  for (unsigned int j = 0; j < height; j++)
    delete[] row_ptrs[j];

  delete[] row_ptrs;

  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(file);
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \param I : Image to save as a PNG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writePNG(const vpImage<vpRGBa> &I, const std::string &filename)
{
  FILE *file;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file \"%s\"", filename.c_str()));
  }

  /* create a png info struct */
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr) {
    fclose(file);
    vpERROR_TRACE("Error during png_create_write_struct()\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, NULL);
    vpERROR_TRACE("Error during png_create_info_struct()\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* initialize the setjmp for returning properly after a libpng error occured
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    vpERROR_TRACE("Error during init_io\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* setup libpng for using standard C fwrite() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();
  int bit_depth = 8;
  int color_type = PNG_COLOR_TYPE_RGB;
  /* set some useful information from header */

  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    vpERROR_TRACE("Error during write header\n");
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);

  png_bytep *row_ptrs = new png_bytep[height];
  for (unsigned int i = 0; i < height; i++)
    row_ptrs[i] = new png_byte[3 * width];

  unsigned char *input = (unsigned char *)I.bitmap;
  ;

  for (unsigned int i = 0; i < height; i++) {
    png_byte *row = row_ptrs[i];
    for (unsigned int j = 0; j < width; j++) {
      row[3 * j] = *(input);
      input++;
      row[3 * j + 1] = *(input);
      input++;
      row[3 * j + 2] = *(input);
      input++;
      input++;
    }
  }

  png_write_image(png_ptr, row_ptrs);

  png_write_end(png_ptr, NULL);

  for (unsigned int j = 0; j < height; j++)
    delete[] row_ptrs[j];

  delete[] row_ptrs;

  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(file);
}

/*!
  Read the contents of the PNG file, allocate memory
  for the corresponding gray level image, if necessary convert the data in
  gray level, and set the bitmap whith the gray level data. That means that
  the image \e I is a "black and white" rendering of the original image in \e
  filename, as in a black and white photograph. If necessary, the quantization
  formula used is \f$0,299 r + 0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/
void vpImageIo::readPNG(vpImage<unsigned char> &I, const std::string &filename)
{
  FILE *file;
  png_byte magic[8];
  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot read file \"%s\"", filename.c_str()));
  }

  /* read magic number */
  if (fread(magic, 1, sizeof(magic), file) != sizeof(magic)) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read magic number in file \"%s\"", filename.c_str()));
  }

  /* check for valid magic number */
  if (png_sig_cmp(magic, 0, sizeof(magic))) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG file: \"%s\" is not a valid PNG image",
                           filename.c_str()));
  }

  /* create a png read struct */
  // printf("version %s\n", PNG_LIBPNG_VER_STRING);
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (png_ptr == NULL) {
    fprintf(stderr, "error: can't create a png read structure!\n");
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "error reading png file"));
  }

  /* create a png info struct */
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == NULL) {
    fprintf(stderr, "error: can't create a png info structure!\n");
    fclose(file);
    png_destroy_read_struct(&png_ptr, NULL, NULL);
    throw(vpImageException(vpImageException::ioError, "error reading png file"));
  }

  /* initialize the setjmp for returning properly after a libpng error occured
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    vpERROR_TRACE("Error during init io\n");
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* setup libpng for using standard C fread() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  /* tell libpng that we have already read the magic number */
  png_set_sig_bytes(png_ptr, sizeof(magic));

  /* read png info */
  png_read_info(png_ptr, info_ptr);

  unsigned int width = png_get_image_width(png_ptr, info_ptr);
  unsigned int height = png_get_image_height(png_ptr, info_ptr);

  unsigned int bit_depth, channels, color_type;
  /* get some useful information from header */
  bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  channels = png_get_channels(png_ptr, info_ptr);
  color_type = png_get_color_type(png_ptr, info_ptr);

  /* convert index color images to RGB images */
  if (color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png_ptr);

  /* convert 1-2-4 bits grayscale images to 8 bits grayscale. */
  if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand(png_ptr);

  //  if (png_get_valid (png_ptr, info_ptr, PNG_INFO_tRNS))
  //    png_set_tRNS_to_alpha (png_ptr);

  if (color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_strip_alpha(png_ptr);

  if (bit_depth == 16)
    png_set_strip_16(png_ptr);
  else if (bit_depth < 8)
    png_set_packing(png_ptr);

  /* update info structure to apply transformations */
  png_read_update_info(png_ptr, info_ptr);

  channels = png_get_channels(png_ptr, info_ptr);

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  png_bytep *rowPtrs = new png_bytep[height];

  unsigned int stride = png_get_rowbytes(png_ptr, info_ptr);
  unsigned char *data = new unsigned char[stride * height];

  for (unsigned int i = 0; i < height; i++)
    rowPtrs[i] = (png_bytep)data + (i * stride);

  png_read_image(png_ptr, rowPtrs);

  vpImage<vpRGBa> Ic(height, width);
  unsigned char *output;

  switch (channels) {
  case 1:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i];
    }
    break;

  case 2:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i * 2];
    }
    break;

  case 3:
    output = (unsigned char *)Ic.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i * 3];
      *(output++) = data[i * 3 + 1];
      *(output++) = data[i * 3 + 2];
      *(output++) = vpRGBa::alpha_default;
    }
    vpImageConvert::convert(Ic, I);
    break;

  case 4:
    output = (unsigned char *)Ic.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i * 4];
      *(output++) = data[i * 4 + 1];
      *(output++) = data[i * 4 + 2];
      *(output++) = data[i * 4 + 3];
    }
    vpImageConvert::convert(Ic, I);
    break;
  }

  delete[](png_bytep) rowPtrs;
  delete[] data;
  png_read_end(png_ptr, NULL);
  png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
  fclose(file);
}

/*!
  Read a PNG file and initialize a scalar image.

  Read the contents of the PNG file, allocate
  memory for the corresponding image, and set
  the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a grayscaled image, a conversion is done to deal
  with \e I which is a color image.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::readPNG(vpImage<vpRGBa> &I, const std::string &filename)
{
  FILE *file;
  png_byte magic[8];

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == NULL) {
    throw(vpImageException(vpImageException::ioError, "Cannot read file \"%s\"", filename.c_str()));
  }

  /* read magic number */
  if (fread(magic, 1, sizeof(magic), file) != sizeof(magic)) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read magic number in file \"%s\"", filename.c_str()));
  }

  /* check for valid magic number */
  if (png_sig_cmp(magic, 0, sizeof(magic))) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG file: \"%s\" is not a valid PNG image",
                           filename.c_str()));
  }

  /* create a png read struct */
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr) {
    fclose(file);
    vpERROR_TRACE("Error during png_create_read_struct()\n");
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* create a png info struct */
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    fclose(file);
    png_destroy_read_struct(&png_ptr, NULL, NULL);
    vpERROR_TRACE("Error during png_create_info_struct()\n");
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* initialize the setjmp for returning properly after a libpng error occured
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    vpERROR_TRACE("Error during init io\n");
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* setup libpng for using standard C fread() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  /* tell libpng that we have already read the magic number */
  png_set_sig_bytes(png_ptr, sizeof(magic));

  /* read png info */
  png_read_info(png_ptr, info_ptr);

  unsigned int width = png_get_image_width(png_ptr, info_ptr);
  unsigned int height = png_get_image_height(png_ptr, info_ptr);

  unsigned int bit_depth, channels, color_type;
  /* get some useful information from header */
  bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  channels = png_get_channels(png_ptr, info_ptr);
  color_type = png_get_color_type(png_ptr, info_ptr);

  /* convert index color images to RGB images */
  if (color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png_ptr);

  /* convert 1-2-4 bits grayscale images to 8 bits grayscale. */
  if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand(png_ptr);

  //  if (png_get_valid (png_ptr, info_ptr, PNG_INFO_tRNS))
  //    png_set_tRNS_to_alpha (png_ptr);

  if (color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_strip_alpha(png_ptr);

  if (bit_depth == 16)
    png_set_strip_16(png_ptr);
  else if (bit_depth < 8)
    png_set_packing(png_ptr);

  /* update info structure to apply transformations */
  png_read_update_info(png_ptr, info_ptr);

  channels = png_get_channels(png_ptr, info_ptr);

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  png_bytep *rowPtrs = new png_bytep[height];

  unsigned int stride = png_get_rowbytes(png_ptr, info_ptr);
  unsigned char *data = new unsigned char[stride * height];

  for (unsigned int i = 0; i < height; i++)
    rowPtrs[i] = (png_bytep)data + (i * stride);

  png_read_image(png_ptr, rowPtrs);

  vpImage<unsigned char> Ig(height, width);
  unsigned char *output;

  switch (channels) {
  case 1:
    output = (unsigned char *)Ig.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i];
    }
    vpImageConvert::convert(Ig, I);
    break;

  case 2:
    output = (unsigned char *)Ig.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i * 2];
    }
    vpImageConvert::convert(Ig, I);
    break;

  case 3:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i * 3];
      *(output++) = data[i * 3 + 1];
      *(output++) = data[i * 3 + 2];
      *(output++) = vpRGBa::alpha_default;
    }
    break;

  case 4:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; i++) {
      *(output++) = data[i * 4];
      *(output++) = data[i * 4 + 1];
      *(output++) = data[i * 4 + 2];
      *(output++) = data[i * 4 + 3];
    }
    break;
  }

  delete[](png_bytep) rowPtrs;
  delete[] data;
  png_read_end(png_ptr, NULL);
  png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
  fclose(file);
}

#elif defined(VISP_HAVE_OPENCV)

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \param I : Image to save as a PNG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writePNG(const vpImage<unsigned char> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);
  cv::imwrite(filename.c_str(), Ip);
#else
  IplImage *Ip = NULL;
  vpImageConvert::convert(I, Ip);

  cvSaveImage(filename.c_str(), Ip);

  cvReleaseImage(&Ip);
#endif
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \param I : Image to save as a PNG file.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::writePNG(const vpImage<vpRGBa> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);
  cv::imwrite(filename.c_str(), Ip);
#else
  IplImage *Ip = NULL;
  vpImageConvert::convert(I, Ip);

  cvSaveImage(filename.c_str(), Ip);

  cvReleaseImage(&Ip);
#endif
}

/*!
  Read the contents of the PNG file, allocate memory
  for the corresponding gray level image, if necessary convert the data in
  gray level, and set the bitmap whith the gray level data. That means that
  the image \e I is a "black and white" rendering of the original image in \e
  filename, as in a black and white photograph. If necessary, the quantization
  formula used is \f$0,299 r + 0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/
void vpImageIo::readPNG(vpImage<unsigned char> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  cv::Mat Ip = cv::imread(filename.c_str(), cv::IMREAD_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  IplImage *Ip = NULL;
  Ip = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (Ip != NULL)
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
  cvReleaseImage(&Ip);
#endif
}

/*!
  Read a PNG file and initialize a scalar image.

  Read the contents of the PNG file, allocate
  memory for the corresponding image, and set
  the bitmap whith the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a grayscaled image, a conversion is done to deal
  with \e I which is a color image.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void vpImageIo::readPNG(vpImage<vpRGBa> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  cv::Mat Ip = cv::imread(filename.c_str(), cv::IMREAD_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  IplImage *Ip = NULL;
  Ip = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR);
  if (Ip != NULL)
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
  cvReleaseImage(&Ip);
#endif
}

#endif
