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
 * Test conversion between ViSP and Eigen type.
 */

/*!
  \example testEigenConversion.cpp

  Test conversion between ViSP and Eigen type.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEigenConversion.h>

#if defined(VISP_HAVE_EIGEN3) && defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
template <typename Type> std::ostream &operator<<(std::ostream &os, const Eigen::Quaternion<Type> &q)
{
  return os << "qw: " << q.w() << " ; qx: " << q.x() << " ; qy: " << q.y() << " ; qz: " << q.z();
}

template <typename Type> std::ostream &operator<<(std::ostream &os, const Eigen::AngleAxis<Type> &aa)
{
  return os << "angle: " << aa.angle() << " ; axis: " << aa.axis()(0) << " ; " << aa.axis()(1) << " ; " << aa.axis()(2)
    << " ; thetau: " << aa.angle() * aa.axis()(0) << " ; " << aa.angle() * aa.axis()(1) << " ; "
    << aa.angle() * aa.axis()(2);
}
} // namespace

TEST_CASE("vpMatrix <--> Eigen::MatrixXd/Matrix3Xd conversion", "[eigen_conversion]")
{
  vpMatrix visp_m(3, 4);
  for (unsigned int i = 0; i < visp_m.size(); i++) {
    visp_m.data[i] = i;
  }

  {
    Eigen::MatrixXd eigen_m;
    VISP_NAMESPACE_NAME::visp2eigen(visp_m, eigen_m);
    std::cout << "Eigen MatrixXd:\n" << eigen_m << std::endl;

    vpMatrix visp_m2;
    VISP_NAMESPACE_NAME::eigen2visp(eigen_m, visp_m2);
    std::cout << "ViSP vpMatrix:\n" << visp_m2 << std::endl;

    REQUIRE(visp_m == visp_m2);
    std::cout << std::endl;
  }
  {
    Eigen::Matrix3Xd eigen_m;
    VISP_NAMESPACE_NAME::visp2eigen(visp_m, eigen_m);
    std::cout << "Eigen Matrix3Xd:\n" << eigen_m << std::endl;

    vpMatrix visp_m2;
    VISP_NAMESPACE_NAME::eigen2visp(eigen_m, visp_m2);
    std::cout << "ViSP vpMatrix:\n" << visp_m2 << std::endl;

    REQUIRE(visp_m == visp_m2);
    std::cout << std::endl;
  }
}

TEST_CASE("Eigen::MatrixXd <--> vpMatrix conversion", "[eigen_conversion]")
{
  Eigen::MatrixXd eigen_m(3, 5);
#if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
  for (Eigen::DenseIndex i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::DenseIndex j = 0; j < eigen_m.cols(); j++) {
#else
  for (Eigen::Index i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::Index j = 0; j < eigen_m.cols(); j++) {
#endif
      eigen_m(i, j) = static_cast<double>(i * eigen_m.cols() + j);
    }
  }
  std::cout << "Eigen Matrix (row major: " << eigen_m.IsRowMajor << "):\n" << eigen_m << std::endl;

  vpMatrix visp_m;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m, visp_m);
  std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

  Eigen::MatrixXd eigen_m2;
  VISP_NAMESPACE_NAME::visp2eigen(visp_m, eigen_m2);
  std::cout << "Eigen MatrixXd (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

  vpMatrix visp_m2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m2, visp_m2);
  REQUIRE(visp_m == visp_m2);
  std::cout << std::endl;
    }

TEST_CASE("Eigen::MatrixX4d <--> vpMatrix conversion", "[eigen_conversion]")
{
  Eigen::MatrixX4d eigen_m(2, 4);
#if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
  for (Eigen::DenseIndex i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::DenseIndex j = 0; j < eigen_m.cols(); j++) {
#else
  for (Eigen::Index i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::Index j = 0; j < eigen_m.cols(); j++) {
#endif
      eigen_m(i, j) = static_cast<double>(i * eigen_m.cols() + j);
    }
  }
  std::cout << "Eigen MatrixX4d (row major: " << eigen_m.IsRowMajor << "):\n" << eigen_m << std::endl;

  vpMatrix visp_m;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m, visp_m);
  std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

  Eigen::MatrixX4d eigen_m2;
  VISP_NAMESPACE_NAME::visp2eigen(visp_m, eigen_m2);
  std::cout << "Eigen MatrixX4d (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

  vpMatrix visp_m2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m2, visp_m2);
  REQUIRE(visp_m == visp_m2);
  std::cout << std::endl;
    }

TEST_CASE("Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> <--> vpMatrix conversion", "[eigen_conversion]")
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eigen_m(3, 5);
#if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
  for (Eigen::DenseIndex i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::DenseIndex j = 0; j < eigen_m.cols(); j++) {
#else
  for (Eigen::Index i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::Index j = 0; j < eigen_m.cols(); j++) {
#endif
      eigen_m(i, j) = static_cast<double>(i * eigen_m.cols() + j);
    }
  }
  std::cout << "Eigen Matrix (RowMajor):\n" << eigen_m << std::endl;

  vpMatrix visp_m;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m, visp_m);
  std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

  Eigen::MatrixXd eigen_m2;
  VISP_NAMESPACE_NAME::visp2eigen(visp_m, eigen_m2);
  std::cout << "Eigen MatrixXd (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

  vpMatrix visp_m2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m2, visp_m2);
  REQUIRE(visp_m == visp_m2);
  std::cout << std::endl;
    }

TEST_CASE("Eigen::Matrix<double, Dynamic, Dynamic, ColMajor> <--> vpMatrix conversion", "[eigen_conversion]")
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> eigen_m(3, 5);
#if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
  for (Eigen::DenseIndex i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::DenseIndex j = 0; j < eigen_m.cols(); j++) {
#else
  for (Eigen::Index i = 0; i < eigen_m.rows(); i++) {
    for (Eigen::Index j = 0; j < eigen_m.cols(); j++) {
#endif
      eigen_m(i, j) = static_cast<double>(i * eigen_m.cols() + j);
    }
  }
  std::cout << "Eigen Matrix (ColMajor):\n" << eigen_m << std::endl;

  vpMatrix visp_m;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m, visp_m);
  std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

  Eigen::MatrixXd eigen_m2;
  VISP_NAMESPACE_NAME::visp2eigen(visp_m, eigen_m2);
  std::cout << "Eigen MatrixXd (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

  vpMatrix visp_m2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_m2, visp_m2);
  REQUIRE(visp_m == visp_m2);
  std::cout << std::endl;
    }

TEST_CASE("vpHomogeneousMatrix <--> Eigen::Matrix4d conversion", "[eigen_conversion]")
{
  vpHomogeneousMatrix visp_cMo(0.1, 0.2, 0.3, 0.1, 0.2, 0.3);
  Eigen::Matrix4d eigen_cMo;
  VISP_NAMESPACE_NAME::visp2eigen(visp_cMo, eigen_cMo);
  std::cout << "Eigen Matrix4d cMo:\n" << eigen_cMo << std::endl;

  vpHomogeneousMatrix visp_cMo2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_cMo, visp_cMo2);
  std::cout << "ViSP  vpHomogeneousMatrix cMo:\n" << visp_cMo2 << std::endl;
  REQUIRE(visp_cMo == visp_cMo2);
  std::cout << std::endl;
}

TEST_CASE("vpHomogeneousMatrix <--> Eigen::Matrix4f + double casting conversion", "[eigen_conversion]")
{
  vpHomogeneousMatrix visp_cMo; // identity for float to double casting
  Eigen::Matrix4d eigen_cMo_tmp;
  VISP_NAMESPACE_NAME::visp2eigen(visp_cMo, eigen_cMo_tmp);
  Eigen::Matrix4f eigen_cMo = eigen_cMo_tmp.cast<float>();
  std::cout << "Eigen Matrix4f cMo:\n" << eigen_cMo << std::endl;

  vpHomogeneousMatrix visp_cMo2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_cMo.cast<double>(), visp_cMo2);
  std::cout << "ViSP  vpHomogeneousMatrix cMo:\n" << visp_cMo2 << std::endl;
  REQUIRE(visp_cMo == visp_cMo2);
  std::cout << std::endl;
}

TEST_CASE("vpQuaternionVector <--> Eigen::Quaternionf conversion", "[eigen_conversion]")
{
  vpQuaternionVector visp_quaternion(0, 1, 2, 3);
  Eigen::Quaternionf eigen_quaternion;
  VISP_NAMESPACE_NAME::visp2eigen(visp_quaternion, eigen_quaternion);
  std::cout << "Eigen quaternion: " << eigen_quaternion << std::endl;

  vpQuaternionVector visp_quaternion2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_quaternion, visp_quaternion2);
  std::cout << "ViSP quaternion: " << visp_quaternion2.t() << std::endl;
  REQUIRE(visp_quaternion == visp_quaternion2);
  std::cout << std::endl;
}

TEST_CASE("vpThetaUVector <--> Eigen::AngleAxisf conversion", "[eigen_conversion]")
{
  vpThetaUVector visp_thetau(0, 1, 2);
  Eigen::AngleAxisf eigen_angle_axis;
  VISP_NAMESPACE_NAME::visp2eigen(visp_thetau, eigen_angle_axis);
  std::cout << "Eigen AngleAxis: " << eigen_angle_axis << std::endl;

  vpThetaUVector visp_thetau2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_angle_axis, visp_thetau2);
  std::cout << "ViSP AngleAxis: " << visp_thetau2.t() << std::endl;
  REQUIRE(visp_thetau == visp_thetau2);
  std::cout << std::endl;
}

TEST_CASE("vpColVector <--> Eigen::VectorXd conversion", "[eigen_conversion]")
{
  vpColVector visp_col(4, 4);
  visp_col = 10;
  Eigen::VectorXd eigen_col;
  VISP_NAMESPACE_NAME::visp2eigen(visp_col, eigen_col);
  std::cout << "Eigen VectorXd: " << eigen_col.transpose() << std::endl;

  vpColVector visp_col2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_col, visp_col2);
  std::cout << "ViSP vpColVector: " << visp_col2.t() << std::endl;
  REQUIRE(visp_col == visp_col2);
  std::cout << std::endl;
}

TEST_CASE("vpRowVector <--> Eigen::RowVectorXd conversion", "[eigen_conversion]")
{
  vpRowVector visp_row(4, 10);
  visp_row = 10;
  Eigen::RowVectorXd eigen_row;
  VISP_NAMESPACE_NAME::visp2eigen(visp_row, eigen_row);
  std::cout << "Eigen RowVectorXd: " << eigen_row << std::endl;

  vpRowVector visp_row2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_row, visp_row2);
  std::cout << "ViSP vpRowVector: " << visp_row2 << std::endl;
  REQUIRE(visp_row == visp_row2);
  std::cout << std::endl;
}

TEST_CASE("Eigen::RowVector4d <--> vpRowVector conversion", "[eigen_conversion]")
{
  Eigen::RowVector4d eigen_row;
  eigen_row << 9, 8, 7, 6;
  vpRowVector visp_row;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_row, visp_row);
  std::cout << "ViSP vpRowVector: " << visp_row << std::endl;

  Eigen::RowVector4d eigen_row2;
  VISP_NAMESPACE_NAME::visp2eigen(static_cast<vpMatrix>(visp_row), eigen_row2);
  std::cout << "Eigen RowVector4d: " << eigen_row2 << std::endl;

  vpRowVector visp_row2;
  VISP_NAMESPACE_NAME::eigen2visp(eigen_row2, visp_row2);
  REQUIRE(visp_row == visp_row2);
  std::cout << std::endl;
}

TEST_CASE("vpRowVector <--> Eigen::RowVector4d conversion", "[eigen_conversion]")
{
  vpRowVector visp_row(4, 10);
  visp_row = 10;
  Eigen::RowVector4d eigen_row;
  VISP_NAMESPACE_NAME::visp2eigen(static_cast<vpMatrix>(visp_row), eigen_row);
  std::cout << "Eigen RowVector4d: " << eigen_row << std::endl;

  vpRowVector visp_row2;
  VISP_NAMESPACE_NAME::eigen2visp(static_cast<Eigen::RowVectorXd>(eigen_row), visp_row2);
  std::cout << "ViSP vpRowVector: " << visp_row2 << std::endl;
  REQUIRE(visp_row == visp_row2);
  std::cout << std::endl;
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
