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
 * 2D vanishing point visual feature (Z coordinate in 3D space is infinity)
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/

#ifndef vpFeatureVanishingPoint_H
#define vpFeatureVanishingPoint_H

/*!
  \file vpFeatureVanishingPoint.h \brief Class that defines 2D vanishing
  point visual feature (Z coordinate in 3D space is infinity)
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpFeatureVanishingPoint
  \ingroup group_visual_features

  Class that defines 2D vanishing point visual features. Various features can be considered:

  - Either the cartesian coordinates \f$ (x, y) \f$ of the vanishing point obtained from the intersection of two lines;
    in that case \f$ {\bf s} = (x, y) \f$ and the corresponding interaction matrices are:
    \f[ L_x = \left[ \begin{array}{cccccc} 0 & 0 & 0 & x y & -(1 + x^2) & y \end{array} \right] \f]
    \f[ L_y = \left[ \begin{array}{cccccc} 0 & 0 & 0 & 1 + y * y & -xy & -x \end{array} \right] \f]

  - Rather features fonction of the polar coordinates of the vanishing point obtained themselves from the polar coordinates of the two lines
    \f$(\rho_1, \theta_1)\f$ and \f$(\rho_2, \theta_2)\f$;
    in that case \f$ {\bf s} = (\arctan(1/\rho), 1/\rho, \alpha) \f$ with:
    \f[ 1/\rho = \frac{\sin(\theta_1 - \theta_2)}{\sqrt{\rho_1^2 + \rho_2^2 - 2 \rho_1 \rho_2 cos(\theta_1 - \theta_2)}} \f]
    \f[ \alpha = \frac{\rho_1 \cos \theta_2 - \rho_2 cos \theta_1}{\sqrt{\rho_1^2 + \rho_2^2 - 2 \rho_1 \rho_2 cos(\theta_1 - \theta_2)}} \f]
    The corresponding interaction matrices are:
    \f[ L_{\arctan(\frac{1}{\rho})} = \left[ \begin{array}{cccccc} 0 & 0 & 0 & - \sin \alpha & \cos \alpha & 0 \end{array} \right] \f]
    \f[ L_{\frac{1}{\rho}} = \left[ \begin{array}{cccccc} 0 & 0 & 0 & -(1 + \frac{1}{\rho^2}) \sin \alpha & (1 + \frac{1}{\rho^2}) \cos \alpha & 0 \end{array} \right] \f]
    \f[ L_{\alpha} = \left[ \begin{array}{cccccc} 0 & 0 & 0 & \frac{\cos \alpha}{\rho} & \frac{\sin \alpha}{\rho}  & -1 \end{array} \right] \f]
*/
class VISP_EXPORT vpFeatureVanishingPoint : public vpBasicFeature
{
public:
  static unsigned int selectAlpha();
  static unsigned int selectAtanOneOverRho();
  static unsigned int selectOneOverRho();
  static unsigned int selectX();
  static unsigned int selectY();

public:
  vpFeatureVanishingPoint();
  //! Destructor.
  virtual ~vpFeatureVanishingPoint() {}

  void buildFrom(const double x, const double y);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  vpFeatureVanishingPoint *duplicate() const;

  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = (selectX() | selectY()));

  double get_x() const;
  double get_y() const;
  double getAtanOneOverRho() const;
  double getOneOverRho() const;
  double getAlpha() const;

  void init();
  vpMatrix interaction(const unsigned int select = (selectX() | selectY()));

  void print(const unsigned int select = (selectX() | selectY())) const;

  void set_x(const double x);
  void set_y(const double y);
  void set_xy(const double x, const double y);
  void setAtanOneOverRho(const double atan_one_over_rho);
  void setOneOverRho(const double one_over_rho);
  void setAlpha(const double alpha);

protected:
  unsigned int m_select; // Memory to know which features are used for display;
};

#endif
