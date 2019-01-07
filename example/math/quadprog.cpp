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
 * Example of sequential calls to QP solver
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/
/*!
  \file quadprog.cpp

  \brief Example of sequential calls to QP solver
*/

/*!
  \example quadprog.cpp

  Example of sequential calls to QP solver
*/

#include <iostream>
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CPP11_COMPATIBILITY

#include <visp3/core/vpQuadProg.h>
#include <visp3/core/vpTime.h>

#include "qp_plot.h"

int main (int argc, char **argv)
{
  const int n = 20;   // x dim
  const int m = 10;   // equality m < n
  const int p = 30;   // inequality
  const int o = 16;   // cost function
#ifdef VISP_HAVE_DISPLAY
  bool opt_display = true;
#endif

  for (int i = 0; i < argc; i++) {
#ifdef VISP_HAVE_DISPLAY
    if (std::string(argv[i]) == "-d")
      opt_display = false;
    else
#endif
    if (std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0] << " [-d] [-h]" << std::endl;
      std::cout << "\nOptions: \n"
#ifdef VISP_HAVE_DISPLAY
                   "  -d \n"
                   "     Disable the image display. This can be useful \n"
                   "     for automatic tests using crontab under Unix or \n"
                   "     using the task manager under Windows.\n"
                   "\n"
#endif
                   "  -h\n"
                   "     Print the help.\n"<< std::endl;

      return EXIT_SUCCESS;
    }
  }
  std::srand((long) vpTime::measureTimeMs());

  vpMatrix A, Q, C;
  vpColVector b, d, r;

  A = randM(m,n)*5;
  b = randV(m)*5;
  Q = randM(o,n)*5;
  r = randV(o)*5;
  C = randM(p,n)*5;

  // make sure Cx <= d has a solution within Ax = b
  vpColVector x = A.solveBySVD(b);
  d = C*x;
  for(int i = 0; i < p; ++i)
    d[i] += (5.*rand())/RAND_MAX;

  // solver with warm start
  vpQuadProg qp_WS;

  // timing
  int total = 100;
  double t_WS(0), t_noWS(0);
  const double eps = 1e-2;

#ifdef VISP_HAVE_DISPLAY
  QPlot *plot = NULL;
  if (opt_display)
    plot = new QPlot(1, total, {"time to solveQP", "warm start"});
#endif

  for(int k = 0; k < total; ++k)
  {
    // reset active set at some point
    if(k == total/2)
      qp_WS.resetActiveSet();

    // small change on QP data
    Q += eps * randM(o,n);
    r += eps * randV(o);
    A += eps * randM(m,n);
    b += eps * randV(m);
    C += eps * randM(p,n);
    d += eps * randV(p);

    // solver without warm start
    vpQuadProg qp;
    x = 0;
    double t = vpTime::measureTimeMs();
    qp.solveQP(Q, r, A, b, C, d, x);

    t_noWS += vpTime::measureTimeMs() - t;
#ifdef VISP_HAVE_DISPLAY
    if (opt_display)
      plot->plot(0,0,k,t);
#endif

    // with warm start
    x = 0;
    t = vpTime::measureTimeMs();
    qp_WS.solveQP(Q, r, A, b, C, d, x);

    t_WS += vpTime::measureTimeMs() - t;
#ifdef VISP_HAVE_DISPLAY
    if (opt_display)
      plot->plot(0, 1, k, t);
#endif
  }

  std::cout.precision(3);
  std::cout << "Warm start: t = " << t_WS << " ms (for 1 QP = " << t_WS/total << " ms)\n";
  std::cout << "No warm start: t = " << t_noWS << " ms (for 1 QP = " << t_noWS/total << " ms)" << std::endl;

#ifdef VISP_HAVE_DISPLAY
  if (opt_display) {
    plot->wait();
    delete plot;
  }
#endif
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You did not build ViSP with C++11 compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CPP11=ON, and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
