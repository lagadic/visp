/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Backend functions implementation for image I/O operations.
 *
 *****************************************************************************/

/*!
  \file vpImageIoBackend.h
  \brief Backend functions implementation for image I/O operations.
*/

#ifndef vpIMAGEIOBACKEND_H
#define vpIMAGEIOBACKEND_H

#include <visp3/core/vpImage.h>

// Portable FloatMap format (PFM)
// Portable Graymap format (PGM)
// Portable Pixmap format (PPM)
void vp_writePFM(const vpImage<float> &I, const std::string &filename);
void vp_writePFM_HDR(const vpImage<float> &I, const std::string &filename);
void vp_writePFM_HDR(const vpImage<vpRGBf> &I, const std::string &filename);
void vp_writePGM(const vpImage<unsigned char> &I, const std::string &filename);
void vp_writePGM(const vpImage<short> &I, const std::string &filename);
void vp_writePGM(const vpImage<vpRGBa> &I, const std::string &filename);
void vp_readPFM(vpImage<float> &I, const std::string &filename);
void vp_readPFM_HDR(vpImage<float> &I, const std::string &filename);
void vp_readPFM_HDR(vpImage<vpRGBf> &I, const std::string &filename);
void vp_readPGM(vpImage<unsigned char> &I, const std::string &filename);
void vp_readPGM(vpImage<vpRGBa> &I, const std::string &filename);
void vp_readPPM(vpImage<unsigned char> &I, const std::string &filename);
void vp_readPPM(vpImage<vpRGBa> &I, const std::string &filename);
void vp_writePPM(const vpImage<unsigned char> &I, const std::string &filename);
void vp_writePPM(const vpImage<vpRGBa> &I, const std::string &filename);

// libjpeg
void readJPEGLibjpeg(vpImage<unsigned char> &I, const std::string &filename);
void readJPEGLibjpeg(vpImage<vpRGBa> &I, const std::string &filename);

void writeJPEGLibjpeg(const vpImage<unsigned char> &I, const std::string &filename, int quality);
void writeJPEGLibjpeg(const vpImage<vpRGBa> &I, const std::string &filename, int quality);

// libpng
void readPNGLibpng(vpImage<unsigned char> &I, const std::string &filename);
void readPNGLibpng(vpImage<vpRGBa> &I, const std::string &filename);

void writePNGLibpng(const vpImage<unsigned char> &I, const std::string &filename);
void writePNGLibpng(const vpImage<vpRGBa> &I, const std::string &filename);

// OpenCV
void readOpenCV(vpImage<unsigned char> &I, const std::string &filename);
void readOpenCV(vpImage<vpRGBa> &I, const std::string &filename);
void readOpenCV(vpImage<float> &I, const std::string &filename);
void readOpenCV(vpImage<vpRGBf> &I, const std::string &filename);

void writeOpenCV(const vpImage<unsigned char> &I, const std::string &filename, int quality);
void writeOpenCV(const vpImage<vpRGBa> &I, const std::string &filename, int quality);
void writeOpenCV(const vpImage<float> &I, const std::string &filename);
void writeOpenCV(const vpImage<vpRGBf> &I, const std::string &filename);

// Simd lib
void readSimdlib(vpImage<unsigned char> &I, const std::string &filename);
void readSimdlib(vpImage<vpRGBa> &I, const std::string &filename);

// TinyEXR lib
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
void readEXRTiny(vpImage<float> &I, const std::string &filename);
void readEXRTiny(vpImage<vpRGBf> &I, const std::string &filename);
#endif

void writeJPEGSimdlib(const vpImage<unsigned char> &I, const std::string &filename, int quality);
void writeJPEGSimdlib(const vpImage<vpRGBa> &I, const std::string &filename, int quality);

void writePNGSimdlib(const vpImage<unsigned char> &I, const std::string &filename);
void writePNGSimdlib(const vpImage<vpRGBa> &I, const std::string &filename);

// stb lib
void readStb(vpImage<unsigned char> &I, const std::string &filename);
void readStb(vpImage<vpRGBa> &I, const std::string &filename);

void writeJPEGStb(const vpImage<unsigned char> &I, const std::string &filename, int quality);
void writeJPEGStb(const vpImage<vpRGBa> &I, const std::string &filename, int quality);

void writePNGStb(const vpImage<unsigned char> &I, const std::string &filename);
void writePNGStb(const vpImage<vpRGBa> &I, const std::string &filename);

// TinyEXR lib
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
void writeEXRTiny(const vpImage<float> &I, const std::string &filename);
void writeEXRTiny(const vpImage<vpRGBf> &I, const std::string &filename);
#endif

#endif
