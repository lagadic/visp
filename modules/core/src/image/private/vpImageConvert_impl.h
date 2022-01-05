/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
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
 * Image conversion tools.
 *
 *****************************************************************************/

/*!
  \file vpImageConvert_impl.h
  \brief Image conversion tools
*/

#ifndef vpIMAGECONVERT_impl_H
#define vpIMAGECONVERT_impl_H

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>

/*!
  Convert the input float depth image to a 8-bits depth image. The input
  depth value is assigned a value proportional to its frequency.
  \param[in] src_depth : Input float depth image.
  \param[out] dest_depth : Output grayscale depth image.
 */
void
vp_createDepthHistogram( const vpImage< float > &src_depth, vpImage< unsigned char > &dest_depth )
{
  dest_depth.resize( src_depth.getHeight(), src_depth.getWidth() );
  uint32_t histogram[0x10000];
  memset( histogram, 0, sizeof( histogram ) );

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i ) {
    if(!vpMath::isNaN(src_depth.bitmap[i])) {
      ++histogram[static_cast<uint32_t>(src_depth.bitmap[i])];
    }
  }

  for ( int i = 2; i < 0x10000; ++i )
    histogram[i] += histogram[i - 1]; // Build a cumulative histogram for the indices in [1,0xFFFF]

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i )
  {
    uint16_t d = static_cast<uint16_t>(src_depth.bitmap[i]);
    if ( d )
    {
      unsigned char f =
          static_cast< unsigned char >( histogram[d] * 255 / histogram[0xFFFF] ); // 0-255 based on histogram location
      dest_depth.bitmap[i] = f;
    }
    else
    {
      dest_depth.bitmap[i] = 0;
    }
  }
}

/*!
  Convert the input uint16_t depth image to a 8-bits depth image. The input
  depth value is assigned a value proportional to its frequency.
  \param[in] src_depth : Input uint16_t depth image.
  \param[out] dest_depth : Output grayscale depth image.
 */
void
vp_createDepthHistogram( const vpImage< uint16_t > &src_depth, vpImage< unsigned char > &dest_depth )
{
  dest_depth.resize( src_depth.getHeight(), src_depth.getWidth() );
  uint32_t histogram[0x10000];
  memset( histogram, 0, sizeof( histogram ) );

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i ) {
    ++histogram[static_cast<uint32_t>(src_depth.bitmap[i])];
  }

  for ( int i = 2; i < 0x10000; ++i )
    histogram[i] += histogram[i - 1]; // Build a cumulative histogram for the indices in [1,0xFFFF]

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i )
  {
    uint16_t d = src_depth.bitmap[i];
    if ( d )
    {
      unsigned char f =
          static_cast< unsigned char >( histogram[d] * 255 / histogram[0xFFFF] ); // 0-255 based on histogram location
      dest_depth.bitmap[i] = f;
    }
    else
    {
      dest_depth.bitmap[i] = 0;
    }
  }
}

/*!
  Convert the input float depth image to a 3 channels 8-bits depth image. The input
  depth value is assigned a value proportional to its frequency.
  \param[in] src_depth : Input float depth image.
  \param[out] dest_depth : Output RGB depth image.
 */
void
vp_createDepthHistogram( const vpImage< float > &src_depth, vpImage< vpRGBa > &dest_depth )
{
  dest_depth.resize( src_depth.getHeight(), src_depth.getWidth() );
  uint32_t histogram[0x10000];
  memset( histogram, 0, sizeof( histogram ) );

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i ) {
    if(!vpMath::isNaN(src_depth.bitmap[i])) {
      ++histogram[static_cast<uint32_t>(src_depth.bitmap[i])];
    }
  }

  for ( int i = 2; i < 0x10000; ++i )
    histogram[i] += histogram[i - 1]; // Build a cumulative histogram for the indices in [1,0xFFFF]

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i )
  {
    uint16_t d = static_cast<uint16_t>(src_depth.bitmap[i]);
    if ( d )
    {
      unsigned char f = (unsigned char)( histogram[d] * 255 / histogram[0xFFFF] ); // 0-255 based on histogram location
      dest_depth.bitmap[i].R = 255 - f;
      dest_depth.bitmap[i].G = 0;
      dest_depth.bitmap[i].B = f;
      dest_depth.bitmap[i].A = vpRGBa::alpha_default;
    }
    else
    {
      dest_depth.bitmap[i].R = 20;
      dest_depth.bitmap[i].G = 5;
      dest_depth.bitmap[i].B = 0;
      dest_depth.bitmap[i].A = vpRGBa::alpha_default;
    }
  }
}

/*!
  Convert the input uint16_t depth image to a 3 channels 8-bits depth image. The input
  depth value is assigned a value proportional to its frequency.
  \param[in] src_depth : Input uint16_t depth image.
  \param[out] dest_depth : Output RGB depth image.
 */
void
vp_createDepthHistogram( const vpImage< uint16_t > &src_depth, vpImage< vpRGBa > &dest_depth )
{
  dest_depth.resize( src_depth.getHeight(), src_depth.getWidth() );
  uint32_t histogram[0x10000];
  memset( histogram, 0, sizeof( histogram ) );

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i ) {
    ++histogram[static_cast<uint32_t>(src_depth.bitmap[i])];
  }

  for ( unsigned int i = 2; i < 0x10000; ++i )
    histogram[i] += histogram[i - 1]; // Build a cumulative histogram for the indices in [1,0xFFFF]

  #ifdef VISP_HAVE_OPENMP
  #pragma omp parallel for
  #endif
  for ( int i = 0; i < static_cast<int>(src_depth.getSize()); ++i )
  {
    uint16_t d = src_depth.bitmap[i];
    if ( d )
    {
      unsigned char f = (unsigned char)( histogram[d] * 255 / histogram[0xFFFF] ); // 0-255 based on histogram location
      dest_depth.bitmap[i].R = 255 - f;
      dest_depth.bitmap[i].G = 0;
      dest_depth.bitmap[i].B = f;
      dest_depth.bitmap[i].A = vpRGBa::alpha_default;
    }
    else
    {
      dest_depth.bitmap[i].R = 20;
      dest_depth.bitmap[i].G = 5;
      dest_depth.bitmap[i].B = 0;
      dest_depth.bitmap[i].A = vpRGBa::alpha_default;
    }
  }
}

#endif
