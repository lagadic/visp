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
 * Luminance based feature.
 */

/*!
 * \file vpFeatureLuminance.h
 * \brief Class that defines the image luminance visual feature
 *
 * For more details see \cite Collewet08c.
 */

#ifndef vpFeatureLuminance_h
#define vpFeatureLuminance_h

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

BEGIN_VISP_NAMESPACE
/*!
 * \class vpLuminance
 * \brief Class that defines the luminance and gradient of a point.
 *
 * \sa vpFeatureLuminance
*/
class VISP_EXPORT vpLuminance
{
public:
  double x, y;   // point coordinates (in meter)
  double I;      // pixel intensity
  double Ix, Iy; // pixel gradient
  double Z;      // pixel depth
};
#endif

/*!
 * \class vpFeatureLuminance
 * \ingroup group_visual_features
 * \brief Class that defines the image luminance visual feature
 *
 * For more details see \cite Collewet08c.
 */
class VISP_EXPORT vpFeatureLuminance : public vpBasicFeature
{
protected:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z;

  //! Number of rows.
  unsigned int nbr;
  //! Number of column.
  unsigned int nbc;
  //! Border size.
  unsigned int bord;

  //! Store the image (as a vector with intensity and gradient I, Ix, Iy)
  vpLuminance *pixInfo;
  int firstTimeIn;

public:
  vpFeatureLuminance();
  vpFeatureLuminance(const vpFeatureLuminance &f);

  //! Destructor.
  virtual ~vpFeatureLuminance() VP_OVERRIDE;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  void buildFrom(vpImage<unsigned char> &I);
#endif
  vpFeatureLuminance &build(vpImage<unsigned char> &I);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;

  vpFeatureLuminance *duplicate() const VP_OVERRIDE;

  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) VP_OVERRIDE;
  void error(const vpBasicFeature &s_star, vpColVector &e);

  double get_Z() const;
  unsigned int getBorder() const;


  void init(unsigned int _nbr, unsigned int _nbc, double _Z);

  void init() VP_OVERRIDE;
  vpMatrix interaction(unsigned int select = FEATURE_ALL) VP_OVERRIDE;
  void interaction(vpMatrix &L);
  void print(unsigned int select = FEATURE_ALL) const VP_OVERRIDE;

  vpFeatureLuminance &operator=(const vpFeatureLuminance &f);


  void setCameraParameters(const vpCameraParameters &_cam);
  void set_Z(double Z);

  static const int DEFAULT_BORDER;

public:
  vpCameraParameters cam;
};
END_VISP_NAMESPACE
#endif
