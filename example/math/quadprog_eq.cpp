/****************************************************************************
 *
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
 * Example of sequential calls to QP solver with constant equality constraint
 *
*****************************************************************************/
/*!
  \file quadprog_eq.cpp

  \brief Example of sequential calls to QP solver with constant equality constraint
*/

/*!
  \example quadprog_eq.cpp

  Example of sequential calls to QP solver with constant equality constraint
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_LAPACK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#include "qp_plot.h"
#include <visp3/core/vpQuadProg.h>
#include <visp3/core/vpTime.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  const int n = 20; // x dim
  const int m = 10; // equality m < n
  const int p = 30; // inequality
  const int o = 16; // cost function
#ifdef VISP_HAVE_DISPLAY
  bool opt_display = true;
  bool opt_click_allowed = true;
#endif

  for (int i = 0; i < argc; i++) {
#ifdef VISP_HAVE_DISPLAY
    if (std::string(argv[i]) == "-d")
      opt_display = false;
    else if (std::string(argv[i]) == "-c")
      opt_click_allowed = false;
    else
#endif
      if (std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0] << " [-d] [-c] [-h] [--help]" << std::endl;
        std::cout << "\nOptions: \n"
#ifdef VISP_HAVE_DISPLAY
          "  -d \n"
          "     Disable the image display. This can be useful \n"
          "     for automatic tests using crontab under Unix or \n"
          "     using the task manager under Windows.\n"
          "\n"
          "  -c \n"
          "     Disable the mouse click. Useful to automate the \n"
          "     execution of this program without human intervention.\n"
          "\n"
#endif
          "  -h, --help\n"
          "     Print the help.\n"
          << std::endl;

        return EXIT_SUCCESS;
      }
  }
  std::srand((long)vpTime::measureTimeMs());

  vpMatrix A, Q, C;
  vpColVector b, d, r;

  A = randM(m, n) * 5;
  b = randV(m) * 5;
  Q = randM(o, n) * 5;
  r = randV(o) * 5;
  C = randM(p, n) * 5;

  // make sure Cx <= d has a solution within Ax = b

  vpColVector x = A.solveBySVD(b);
  d = C * x;
  for (int i = 0; i < p; ++i)
    d[i] += (5. * rand()) / RAND_MAX;

  // solver with stored equality and warm start
  vpQuadProg qp_WS;
  qp_WS.setEqualityConstraint(A, b);

  vpQuadProg qp_ineq_WS;
  qp_ineq_WS.setEqualityConstraint(A, b);

  // timing
  int total = 100;
  double t_WS(0), t_noWS(0), t_ineq_WS(0), t_ineq_noWS(0);
  const double eps = 1e-2;

#ifdef VISP_HAVE_DISPLAY
  QPlot *plot = nullptr;
  if (opt_display)
    plot = new QPlot(2, total,
                     { "only equalities", "pre-solving", "equalities + inequalities", "pre-solving / warm start" });
#endif

  for (int k = 0; k < total; ++k) {
    // small change on QP data (A and b are constant)
    Q += eps * randM(o, n);
    r += eps * randV(o);
    C += eps * randM(p, n);
    d += eps * randV(p);

    // solve only equalities
    // without warm start
    x = 0;
    double t = vpTime::measureTimeMs();
    vpQuadProg::solveQPe(Q, r, A, b, x);

    t_noWS += vpTime::measureTimeMs() - t;
#ifdef VISP_HAVE_DISPLAY
    if (opt_display)
      plot->plot(0, 0, k, t);
#endif

    // with pre-solved Ax = b
    x = 0;
    t = vpTime::measureTimeMs();
    qp_WS.solveQPe(Q, r, x);

    t_WS += vpTime::measureTimeMs() - t;
#ifdef VISP_HAVE_DISPLAY
    if (opt_display)
      plot->plot(0, 1, k, t);
#endif

    // with inequalities
    // without warm start
    x = 0;
    vpQuadProg qp;
    t = vpTime::measureTimeMs();
    qp.solveQP(Q, r, A, b, C, d, x);

    t_ineq_noWS += vpTime::measureTimeMs() - t;
#ifdef VISP_HAVE_DISPLAY
    if (opt_display)
      plot->plot(1, 0, k, t);
#endif

    // with warm start + pre-solving
    x = 0;
    t = vpTime::measureTimeMs();
    qp_ineq_WS.solveQPi(Q, r, C, d, x, true);

    t_ineq_WS += vpTime::measureTimeMs() - t;
#ifdef VISP_HAVE_DISPLAY
    if (opt_display)
      plot->plot(1, 1, k, t);
#endif
  }

  std::cout.precision(3);
  std::cout << "With only equality constraints\n";
  std::cout << "   pre-solving: t = " << t_WS << " ms (for 1 QP = " << t_WS / total << " ms)\n";
  std::cout << "   no pre-solving: t = " << t_noWS << " ms (for 1 QP = " << t_noWS / total << " ms)\n\n";

  std::cout << "With inequality constraints\n";
  std::cout << "   Warm start: t = " << t_ineq_WS << " ms (for 1 QP = " << t_ineq_WS / total << " ms)\n";
  std::cout << "   No warm start: t = " << t_ineq_noWS << " ms (for 1 QP = " << t_ineq_noWS / total << " ms)"
    << std::endl;

#ifdef VISP_HAVE_DISPLAY
  if (opt_display) {
    if (opt_click_allowed) {
      std::cout << "Click in the graph to exit..." << std::endl;
      plot->wait();
    }
    delete plot;
  }
#endif
}
#elif !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
int main()
{
  std::cout << "You did not build ViSP with c++11 or higher compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CXX_STANDARD=11, and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You did not build ViSP with Lapack support" << std::endl;
  return EXIT_SUCCESS;
}
#endif
