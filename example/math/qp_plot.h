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
  \file qp_plot.h

  \brief Include to plot solver time
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpPlot.h>

vpMatrix randM(int n, int m)
{
  vpMatrix M(n,m);
  for(int i = 0; i < n; ++i)
    for(int j = 0; j < m; ++j)
      M[i][j] = (2.*rand())/RAND_MAX -1;
  return M;
}

vpColVector randV(int n)
{
  vpColVector M(n);
  for(int i = 0; i < n; ++i)
    M[i] = (2.*rand())/RAND_MAX -1;
  return M;
}

#ifdef VISP_HAVE_DISPLAY
class QPlot
{
public:
  virtual ~QPlot()  { delete P; }
  QPlot(int graphNum, int total, std::vector<std::string> legend)
  {
    P = new vpPlot(graphNum, 700, 700, 100, 200, "Resolution time");

    for(int i = 0; i < graphNum; ++i)
    {
      P->initGraph(i,2);
      P->setColor(i,0,vpColor::red);
      P->setColor(i,1,vpColor::blue);
      P->setGraphThickness(i,2);
      P->initRange(i, 0, total, 0, 0.1);
      P->setUnitY(i,"ms");
      P->setTitle(i, legend[2*i]);
      P->setLegend(i, 0, "without " + legend[2*i+1]);
      P->setLegend(i, 1, legend[2*i+1]);
    }
  }

  void plot(int g, int c, int i, double t)
  {
    P->plot(g,c,i,vpTime::measureTimeMs() - t);
  }

  void wait()
  {
    P->I.display->getClick();
  }
  vpPlot* P;

private:
  // Copy constructor not allowed.
  QPlot(const QPlot &qplot);
};
#else
class VISP_EXPORT QPPlot
{
public:
  QPPlot(int, int , std::vector<std::string> ) {}
  void plot(int , int , int , double ) {}
  void wait() {}
};
#endif
