/****************************************************************************
 *
 * $Id: vpMePath.h,v 1.10 2007-12-20 09:09:25 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Andrea Cherubini
 *
 *****************************************************************************/


#ifndef vpMePath_HH
#define vpMePath_HH


#include <math.h>


#include <visp/vpConfig.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include <visp/vpMeTracker.h>
#include <visp/vpMeSite.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpColor.h>


/*!
  \class vpMePath

  \brief class that tracks a path (white road line) moving edges

*/
class VISP_EXPORT vpMePath : public vpMeTracker
{

public:

  vpMePath();
  virtual ~vpMePath();
  void display(vpImage<unsigned char> &I, vpColor::vpColorType col);
  void track(vpImage<unsigned char>& Im);
  void initTracking(vpImage<unsigned char> &I);
  void initTracking(vpImage<unsigned char> &I, int n,
		    unsigned *i, unsigned *j);
  inline void setVerboseMode(bool on=false) {this->verbose = on;};
  inline void setTrackParabolas(bool on=true) {this->trackParabolas = on;};

private:

  void getParameters(vpImage<unsigned char> &I);
  void sample(vpImage<unsigned char>&image);
  void leastSquare(vpImage<unsigned char> &I);
  void leastSquareParabola(vpImage<unsigned char> &I);
  void leastSquareParabolaGivenOrientation(vpImage<unsigned char> &I);
  void leastSquareLine(vpImage<unsigned char> &I);
  void updateNormAng();
  void suppressPoints();
  void seekExtremities(vpImage<unsigned char> &I);
  void getParabolaPoints();
  void displayList(vpImage<unsigned char> &I);
  void computeNormAng(double &norm_ang, vpColVector &K,
		      double i, double j, bool isLine);
  void reduceList(vpList<vpMeSite> &list, int newSize);

public:
  //line OR parabola parameters y = ax^2+bx+c in frame (rotated by thetaFin)
  double aFin, bFin, cFin, thetaFin;
  //indicates that the path is a straight line
  bool line;
  //indicates that the line si horizontal
  bool horLine;
  //image plane coordinates of curve extremity points
  double i1, j1, i2, j2;
  //image plane parabola point coordinates used to find circle
  double *i_par, *j_par;
  //number of parabola points used to find circle
  int numPointPar;
  //number of initial points used to find parabola
  int numPoints;
  //image plane coordinates of initial points
  unsigned *i_ref, *j_ref;
  //number of points used for tracking at every iteration
  int n_points;

private:
  //conic (parabola or line) parameters
  vpColVector K;
  //line parameters
  vpColVector K_line;
  //parabola parameters
  vpColVector K_par;
  //parabola parameters y = ax^2+bx+c in frame (rotated by thetaPar)
  double aPar, bPar, cPar, thetaPar;
  //cos and sine of thetaFin
  double ct, st;
  //indicates that it is the first iteration
  bool firstIter;
  //least square errors
  double line_error, parab_error, parab_errorTot;
  //number of points with small error in least squares
  int lineGoodPoints, parGoodPoints, parGoodPointsTot;
  //indicates verbose mode
  bool verbose;
  //indicates thta parabolas should be tracked
  bool trackParabolas;

  //number of least square iterations
  int LSiter;
  //threshold on least square line error
  double good_point_thresh;
  //every sampleIter iterations sample the curve
  int sampleIter;
  //percent of samples within extremities
  double pointPercentageWithinExtremes;
  //number of times extremities are seeked at each iteration
  int seekLoops;
  //number of points seeked outside each extremity
  int numExtr;
  //gain for considering good points when selecting curve
  int goodPointGain;
  //max error tolerated on line before selecting parabola
  int maxLineScore;
  //conic determinant threshold for selecting a line
  double par_det_threshold;
  //aPar threshold (if below, select line)
  double aParThreshold;
};




#endif


