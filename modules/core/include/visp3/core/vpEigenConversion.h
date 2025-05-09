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
 * ViSP <--> Eigen conversion.
 */

#ifndef VP_EIGEN_CONVERSION_H
#define VP_EIGEN_CONVERSION_H

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_EIGEN3
#include <Eigen/Dense>
#endif
#include <visp3/core/vpMatrix.h>

namespace VISP_NAMESPACE_NAME
{
#ifdef VISP_HAVE_EIGEN3
/* Eigen to ViSP */
VISP_EXPORT void eigen2visp(const Eigen::MatrixXd &src, vpMatrix &dst);

VISP_EXPORT void eigen2visp(const Eigen::MatrixXd &src, vpHomogeneousMatrix &dst);

template <typename Type>
void eigen2visp(const Eigen::Quaternion<Type> &src, vpQuaternionVector &dst)
{
  dst.buildFrom(src.x(), src.y(), src.z(), src.w());
}

template <typename Type>
void eigen2visp(const Eigen::AngleAxis<Type> &src, vpThetaUVector &dst)
{
  const unsigned int val_2 = 2;
  dst.buildFrom(src.angle() * src.axis()(0), src.angle() * src.axis()(1), src.angle() * src.axis()(val_2));
}

VISP_EXPORT void eigen2visp(const Eigen::VectorXd &src, vpColVector &dst);

VISP_EXPORT void eigen2visp(const Eigen::RowVectorXd &src, vpRowVector &dst);

/* ViSP to Eigen */
template <typename Derived>
void visp2eigen(const  vpMatrix &src, Eigen::MatrixBase<Derived> &dst)
{
  dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(src.data, src.getRows(),
                                                                                            src.getCols());
}

template <typename Derived>
void visp2eigen(const  vpHomogeneousMatrix &src, Eigen::MatrixBase<Derived> &dst)
{
  dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(src.data, src.getRows(),
                                                                                            src.getCols());
}

template <typename Type>
void visp2eigen(const  vpQuaternionVector &src, Eigen::Quaternion<Type> &dst)
{
  dst.w() = static_cast<Type>(src.w());
  dst.x() = static_cast<Type>(src.x());
  dst.y() = static_cast<Type>(src.y());
  dst.z() = static_cast<Type>(src.z());
}

template <typename Type>
void visp2eigen(const  vpThetaUVector &src, Eigen::AngleAxis<Type> &dst)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  dst.angle() = static_cast<Type>(src.getTheta());
  dst.axis()(index_0) = static_cast<Type>(src.getU()[index_0]);
  dst.axis()(index_1) = static_cast<Type>(src.getU()[index_1]);
  dst.axis()(index_2) = static_cast<Type>(src.getU()[index_2]);
}

VISP_EXPORT void visp2eigen(const  vpColVector &src, Eigen::VectorXd &dst);

VISP_EXPORT void visp2eigen(const  vpRowVector &src, Eigen::RowVectorXd &dst);
#endif
} // namespace VISP_NAMESPACE_NAME
#endif
