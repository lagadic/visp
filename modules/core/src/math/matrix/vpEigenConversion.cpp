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

#include <new>                                        // for operator new

#include <visp3/core/vpConfig.h>                      // for VISP_HAVE_EIGEN3
#include <visp3/core/vpEigenConversion.h>             // for eigen2visp, vis...
#include <visp3/core/vpException.h>                   // for vpException
#include <visp3/core/vpMatrix.h>                      // for vpMatrix
#include <visp3/core/vpColVector.h>                   // for vpColVector
#include <visp3/core/vpHomogeneousMatrix.h>           // for vpHomogeneousMa...
#include <visp3/core/vpRowVector.h>                   // for vpRowVector

#ifdef VISP_HAVE_EIGEN3
#include <Eigen/src/Core/Assign.h>                    // for MatrixBase::ope...
#include <Eigen/src/Core/AssignEvaluator.h>           // for call_assignment
#include <Eigen/src/Core/CoreEvaluators.h>            // for evaluator<>::Flags
#include <Eigen/src/Core/DenseBase.h>                 // for DenseBase<>::Ma...
#include <Eigen/src/Core/DenseCoeffsBase.h>           // for inner_stride_at...
#include <Eigen/src/Core/GenericPacketMath.h>         // for ploadt, pstoret
#include <Eigen/src/Core/Map.h>                       // for traits<>::Flags
#include <Eigen/src/Core/MapBase.h>                   // for MapBase<>::Size...
#include <Eigen/src/Core/Matrix.h>                    // for traits<>::ColsA...
#include <Eigen/src/Core/NumTraits.h>                 // for GenericNumTrait...
#include <Eigen/src/Core/Stride.h>                    // for Stride, Stride<...
#include <Eigen/src/Core/arch/NEON/PacketMath.h>      // for unpacket_traits...
#include <Eigen/src/Core/util/Constants.h>            // for AlignmentType
#include <Eigen/src/Core/util/ForwardDeclarations.h>  // for accessors_level...
#include <Eigen/src/Core/util/Macros.h>               // for EIGEN_MAJOR_VER...
#include <Eigen/src/Core/util/Memory.h>               // for conditional_ali...
#include <Eigen/src/Core/util/Meta.h>                 // for is_arithmetic<>...
#include <Eigen/src/Core/util/XprHelper.h>            // for size_at_compile...
#endif

namespace VISP_NAMESPACE_NAME
{
#ifdef VISP_HAVE_EIGEN3
/* Eigen to ViSP */
void eigen2visp(const Eigen::MatrixXd &src, vpMatrix &dst)
{
  dst.resize(static_cast<unsigned int>(src.rows()), static_cast<unsigned int>(src.cols()), false, false);
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(&dst.data[0], src.rows(),
                                                                                      src.cols()) = src;
}

void eigen2visp(const Eigen::MatrixXd &src, vpHomogeneousMatrix &dst)
{
  if ((src.rows() != 4) || (src.cols() != 4)) {
    throw  vpException(vpException::dimensionError, "Input Eigen Matrix must be of size (4,4)!");
  }

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(&dst.data[0], src.rows(),
                                                                                      src.cols()) = src;
}

void eigen2visp(const Eigen::VectorXd &src, vpColVector &dst)
{
  dst.resize(static_cast<unsigned int>(src.rows()));
#if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
  Eigen::DenseIndex src_rows = src.rows();
  for (Eigen::DenseIndex i = 0; i < src_rows; i++) {
#else
  Eigen::Index src_rows = src.rows();
  for (Eigen::Index i = 0; i < src_rows; ++i) {
#endif
    dst[static_cast<unsigned int>(i)] = src(i);
  }
}

void eigen2visp(const Eigen::RowVectorXd &src, vpRowVector &dst)
{
  dst.resize(static_cast<unsigned int>(src.cols()));
#if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
  Eigen::DenseIndex src_cols = src.cols();
  for (Eigen::DenseIndex i = 0; i < src_cols; ++i) {
#else
  Eigen::Index src_cols = src.cols();
  for (Eigen::Index i = 0; i < src_cols; ++i) {
#endif
    dst[static_cast<unsigned int>(i)] = src(i);
  }
}

void visp2eigen(const  vpColVector &src, Eigen::VectorXd &dst) { dst = Eigen::VectorXd::Map(src.data, src.size()); }

void visp2eigen(const  vpRowVector &src, Eigen::RowVectorXd &dst)
{
  dst = Eigen::RowVectorXd::Map(src.data, src.size());
}
#endif
} // namespace VISP_NAMESPACE_NAME
