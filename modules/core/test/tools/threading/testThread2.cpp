/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test threading capabilities (extended).
 *
 *****************************************************************************/

/*!
  \example testThread2.cpp

  \brief Test threading capabilities (extended).
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))

#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <visp3/core/vpThread.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpColVector.h>


namespace {
  //! [functor-thread-example declaration]
  class ArithmFunctor {
  public:
    ArithmFunctor(const vpColVector &v1, const vpColVector &v2, const unsigned int start, const unsigned int end) :
      m_add(), m_mul(), m_v1(v1), m_v2(v2), m_indexStart(start), m_indexEnd(end) {
    }

    ArithmFunctor() : m_add(), m_mul(), m_v1(), m_v2(), m_indexStart(0), m_indexEnd(0) {
    }

    void operator()() {
      computeImpl();
    }

    vpColVector getVectorAdd() const {
      return m_add;
    }

    vpColVector getVectorMul() const {
      return m_mul;
    }

  private:
    vpColVector m_add;
    vpColVector m_mul;
    vpColVector m_v1;
    vpColVector m_v2;
    unsigned int m_indexStart;
    unsigned int m_indexEnd;

    void computeImpl() {
      m_add.resize(m_indexEnd - m_indexStart);
      m_mul.resize(m_indexEnd - m_indexStart);

      //to simulate a long computation
      for (int iter = 0; iter < 100; iter++) {
        for (unsigned int i = m_indexStart, cpt = 0; i < m_indexEnd; i++, cpt++) {
          m_add[cpt] = m_v1[i] + m_v2[i];
          m_mul[cpt] = m_v1[i] * m_v2[i];
        }
      }
    }
  };
  //! [functor-thread-example declaration]

  //! [functor-thread-example threadFunction]
  vpThread::Return arithmThread(vpThread::Args args) {
    ArithmFunctor* f = static_cast<ArithmFunctor*>(args);
    (*f)();
    return 0;
  }
  //! [functor-thread-example threadFunction]

  void insert(vpColVector &v1, const vpColVector &v2) {
    unsigned int size = v1.size();
    v1.resize(size + v2.size(), false);

    for (unsigned int i = 0, cpt = size; i < v2.size(); i++, cpt++) {
      v1[cpt] = v2[i];
    }
  }

  bool check(const vpColVector &v1, const vpColVector &v2, const vpColVector &res_add, const vpColVector &res_mul) {
    double add = 0.0, mul = 0.0;
    for (unsigned int i = 0; i < v1.size(); i++) {
      add += v1[i] + v2[i];
      mul += v1[i] * v2[i];
    }

    double add_th = res_add.sum();
    double mul_th = res_mul.sum();

    std::cout << "add=" << add << " ; add_th=" << add_th << std::endl;
    std::cout << "mul=" << mul << " ; mul_th=" << mul_th << std::endl;

    if (!vpMath::equal(add, add_th, std::numeric_limits<double>::epsilon())) {
      std::cerr << "Problem: add=" << add << " ; add_th=" << add_th << std::endl;
      return false;
    }

    if (!vpMath::equal(mul, mul_th, std::numeric_limits<double>::epsilon())) {
      std::cerr << "Problem: mul=" << mul << " ; mul_th=" << mul_th << std::endl;
      return false;
    }

    return true;
  }
}

int main() {
  std::string appveyor_threading = "";
  try {
    appveyor_threading = vpIoTools::getenv("APPVEYOR_THREADING");
  } catch (...) {}

  if (appveyor_threading == "true") {
    unsigned int nb_threads = 4;
    unsigned int size = 1000007;
    srand((unsigned int)time(NULL));

    vpColVector v1(size), v2(size);
    for (unsigned int i = 0; i < size; i++) {
      v1[i] = rand() % 101;
      v2[i] = rand() % 101;
    }

    //! [functor-thread-example threadCreation]
    std::vector<vpThread> threads(nb_threads);
    std::vector<ArithmFunctor> functors(nb_threads);
    unsigned int split = size / nb_threads;
    for (unsigned int i = 0; i < nb_threads; i++) {
      if (i < nb_threads - 1) {
        functors[i] = ArithmFunctor(v1, v2, i*split, (i + 1)*split);
      }
      else {
        functors[i] = ArithmFunctor(v1, v2, i*split, size);
      }

      std::cout << "Create thread: " << i << std::endl;
      threads[i].create((vpThread::Fn) arithmThread, (vpThread::Args) &functors[i]);
    }
    //! [functor-thread-example threadCreation]

    //! [functor-thread-example getResults]
    vpColVector res_add, res_mul;
    for (size_t i = 0; i < nb_threads; i++) {
      std::cout << "Join thread: " << i << std::endl;
      threads[i].join();

      insert(res_add, functors[i].getVectorAdd());
      insert(res_mul, functors[i].getVectorMul());
    }
    //! [functor-thread-example getResults]

    if (!check(v1, v2, res_add, res_mul)) {
      return EXIT_FAILURE;
    }
  }

  std::cout << "testThread2 is ok!" << std::endl;
  return EXIT_SUCCESS;
}

#else

#include <iostream>
#include <cstdlib>

int main()
{
#  if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#  else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#  endif
  return EXIT_SUCCESS;
}
#endif
