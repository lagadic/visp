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
 * Test conversion between ViSP and Eigen type.
 *
 *****************************************************************************/

/*!
  \example testEigenConversion.cpp

  Test conversion between ViSP and Eigen type.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpEigenConversion.h>

#ifdef VISP_HAVE_EIGEN3
namespace
{
template<typename Type>
std::ostream &operator<<(std::ostream &os, const Eigen::Quaternion<Type> &q) {
  return os << "qw: " << q.w() << " ; qx: " << q.x() << " ; qy: "
            << q.y() << " ; qz: " << q.z();
}

template<typename Type>
std::ostream &operator<<(std::ostream &os, const Eigen::AngleAxis<Type> &aa) {
  return os << "angle: " << aa.angle() << " ; axis: " << aa.axis()(0)
            << " ; " << aa.axis()(1) << " ; " << aa.axis()(2)
            << " ; thetau: " << aa.angle()*aa.axis()(0)
            << " ; " << aa.angle()*aa.axis()(1)
            << " ; " << aa.angle()*aa.axis()(2);
}

bool isEqual(const vpArray2D<double> &a1, const vpArray2D<double> &a2) {
  if (a1.size() != a2.size()) {
    return false;
  }

  for (size_t i = 0; i < a1.size(); i++) {
    if (!vpMath::equal(a1.data[i], a2.data[i], std::numeric_limits<double>::epsilon())) {
      return false;
    }
  }

  return true;
}
}
#endif

int main()
{
#ifdef VISP_HAVE_EIGEN3
  {
    vpMatrix visp_m(3, 4);
    for (unsigned int i = 0; i < visp_m.size(); i++) {
      visp_m.data[i] = i;
    }

    Eigen::MatrixXd eigen_m;
    vp::visp2eigen(visp_m, eigen_m);
    std::cout << "Eigen MatrixXd:\n" << eigen_m << std::endl;

    vpMatrix visp_m2;
    vp::eigen2visp(eigen_m, visp_m2);
    std::cout << "ViSP vpMatrix:\n" << visp_m2 << std::endl;

    if (!isEqual(visp_m, visp_m2)) {
      std::cerr << "Issue with vpMatrix <--> Eigen::MatrixXd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpMatrix visp_m(3, 4);
    for (unsigned int i = 0; i < visp_m.size(); i++) {
      visp_m.data[i] = i;
    }

    Eigen::Matrix3Xd eigen_m;
    vp::visp2eigen(visp_m, eigen_m);
    std::cout << "Eigen Matrix3Xd:\n" << eigen_m << std::endl;

    vpMatrix visp_m2;
    vp::eigen2visp(eigen_m, visp_m2);
    std::cout << "ViSP vpMatrix:\n" << visp_m2 << std::endl;

    if (!isEqual(visp_m, visp_m2)) {
      std::cerr << "Issue with vpMatrix <--> Eigen::Matrix3Xd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

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
    vp::eigen2visp(eigen_m, visp_m);
    std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

    Eigen::MatrixXd eigen_m2;
    vp::visp2eigen(visp_m, eigen_m2);
    std::cout << "Eigen MatrixXd (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

    vpMatrix visp_m2;
    vp::eigen2visp(eigen_m2, visp_m2);

    if (!isEqual(visp_m, visp_m2)) {
      std::cerr << "Issue with vpMatrix <--> Eigen::MatrixXd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

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
    vp::eigen2visp(eigen_m, visp_m);
    std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

    Eigen::MatrixX4d eigen_m2;
    vp::visp2eigen(visp_m, eigen_m2);
    std::cout << "Eigen MatrixX4d (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

    vpMatrix visp_m2;
    vp::eigen2visp(eigen_m2, visp_m2);

    if (!isEqual(visp_m, visp_m2)) {
      std::cerr << "Issue with vpMatrix <--> Eigen::MatrixX4d conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

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
    vp::eigen2visp(eigen_m, visp_m);
    std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

    Eigen::MatrixXd eigen_m2;
    vp::visp2eigen(visp_m, eigen_m2);
    std::cout << "Eigen MatrixXd (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

    vpMatrix visp_m2;
    vp::eigen2visp(eigen_m2, visp_m2);

    if (!isEqual(visp_m, visp_m2)) {
      std::cerr << "Issue with vpMatrix <--> Eigen::MatrixXd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

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
    vp::eigen2visp(eigen_m, visp_m);
    std::cout << "ViSP vpMatrix:\n" << visp_m << std::endl;

    Eigen::MatrixXd eigen_m2;
    vp::visp2eigen(visp_m, eigen_m2);
    std::cout << "Eigen MatrixXd (row major: " << eigen_m2.IsRowMajor << "):\n" << eigen_m2 << std::endl;

    vpMatrix visp_m2;
    vp::eigen2visp(eigen_m2, visp_m2);

    if (!isEqual(visp_m, visp_m2)) {
      std::cerr << "Issue with vpMatrix <--> Eigen::MatrixXd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpHomogeneousMatrix visp_cMo;
    Eigen::Matrix4d eigen_cMo;
    vp::visp2eigen(visp_cMo, eigen_cMo);
    std::cout << "Eigen Matrix4d cMo:\n" << eigen_cMo << std::endl;

    vpHomogeneousMatrix visp_cMo2;
    vp::eigen2visp(eigen_cMo, visp_cMo2);
    std::cout << "ViSP  vpHomogeneousMatrix cMo:\n" << visp_cMo2 << std::endl;

    if (!isEqual(visp_cMo, visp_cMo2)) {
      std::cerr << "Issue with vpHomogeneousMatrix <--> Eigen::Matrix4d conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpHomogeneousMatrix visp_cMo;
    Eigen::Matrix4d eigen_cMo_tmp;
    vp::visp2eigen(visp_cMo, eigen_cMo_tmp);
    Eigen::Matrix4f eigen_cMo = eigen_cMo_tmp.cast<float>();
    std::cout << "Eigen Matrix4f cMo:\n" << eigen_cMo << std::endl;

    vpHomogeneousMatrix visp_cMo2;
    vp::eigen2visp(eigen_cMo.cast<double>(), visp_cMo2);
    std::cout << "ViSP  vpHomogeneousMatrix cMo:\n" << visp_cMo2 << std::endl;

    if (!isEqual(visp_cMo, visp_cMo2)) {
      std::cerr << "Issue with vpHomogeneousMatrix <--> Eigen::Matrix4d conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpQuaternionVector visp_quaternion(0, 1, 2, 3);
    Eigen::Quaternionf eigen_quaternion;
    vp::visp2eigen(visp_quaternion, eigen_quaternion);
    std::cout << "Eigen quaternion: " << eigen_quaternion << std::endl;

    vpQuaternionVector visp_quaternion2;
    vp::eigen2visp(eigen_quaternion, visp_quaternion2);
    std::cout << "ViSP quaternion: " << visp_quaternion2.t() << std::endl;

    if (!isEqual(visp_quaternion, visp_quaternion2)) {
      std::cerr << "Issue with vpQuaternionVector <--> Eigen::Quaternion conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpThetaUVector visp_thetau(0, 1, 2);
    Eigen::AngleAxisf eigen_angle_axis;
    vp::visp2eigen(visp_thetau, eigen_angle_axis);
    std::cout << "Eigen AngleAxis: " << eigen_angle_axis << std::endl;

    vpThetaUVector visp_thetau2;
    vp::eigen2visp(eigen_angle_axis, visp_thetau2);
    std::cout << "ViSP AngleAxis: " << visp_thetau2.t() << std::endl;

    if (!isEqual(visp_thetau, visp_thetau2)) {
      std::cerr << "Issue with vpThetaUVector <--> Eigen::AngleAxis conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpColVector visp_col(4, 4);
    Eigen::VectorXd eigen_col;
    vp::visp2eigen(visp_col, eigen_col);
    std::cout << "Eigen VectorXd: " << eigen_col.transpose() << std::endl;

    vpColVector visp_col2;
    vp::eigen2visp(eigen_col, visp_col2);
    std::cout << "ViSP vpColVector: " << visp_col2.t() << std::endl;

    if (!isEqual(visp_col, visp_col2)) {
      std::cerr << "Issue with vpColVector <--> Eigen::VectorXd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpRowVector visp_row(4, 10);
    Eigen::RowVectorXd eigen_row;
    vp::visp2eigen(visp_row, eigen_row);
    std::cout << "Eigen RowVectorXd: " << eigen_row << std::endl;

    vpRowVector visp_row2;
    vp::eigen2visp(eigen_row, visp_row2);
    std::cout << "ViSP vpRowVector: " << visp_row2 << std::endl;

    if (!isEqual(visp_row, visp_row2)) {
      std::cerr << "Issue with vpRowVector <--> Eigen::RowVectorXd conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    Eigen::RowVector4d eigen_row;
    eigen_row << 9, 8, 7, 6;
    vpRowVector visp_row;
    vp::eigen2visp(eigen_row, visp_row);
    std::cout << "ViSP vpRowVector: " << visp_row << std::endl;

    Eigen::RowVector4d eigen_row2;
    vp::visp2eigen(visp_row, eigen_row2);
    std::cout << "Eigen RowVector4d: " << eigen_row2 << std::endl;

    vpRowVector visp_row2;
    vp::eigen2visp(eigen_row2, visp_row2);

    if (!isEqual(visp_row, visp_row2)) {
      std::cerr << "Issue with vpRowVector <--> Eigen::RowVector4d conversion!" << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << std::endl;
  }

  {
    vpRowVector visp_row(4, 10);
    Eigen::RowVector4d eigen_row;
    vp::visp2eigen(visp_row, eigen_row);
    std::cout << "Eigen RowVector4d: " << eigen_row << std::endl;

    vpRowVector visp_row2;
    vp::eigen2visp(eigen_row, visp_row2);
    std::cout << "ViSP vpRowVector: " << visp_row2 << std::endl;

    if (!isEqual(visp_row, visp_row2)) {
      std::cerr << "Issue with vpRowVector <--> Eigen::RowVector4d conversion!" << std::endl;
      return EXIT_FAILURE;
    }
  }
#endif

  return  EXIT_SUCCESS;
}
