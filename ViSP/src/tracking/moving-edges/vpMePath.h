/****************************************************************************
 *
 * $Id: vpMePath.h,v 1.1 2007-07-06 15:46:37 fspindle Exp $
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
#include <visp/vpColor.h>



class VISP_EXPORT vpMePath : public vpMeTracker
{

public:

  vpMePath() ;
  virtual ~vpMePath() ;
  void display(vpImage<unsigned char>&I, vpColor::vpColorType col) ;
  void track(vpImage<unsigned char>& Im);
  void initTracking(vpImage<unsigned char> &I) ;
  void initTracking(vpImage<unsigned char> &I, int n,
		    unsigned *i, unsigned *j) ;
  
  inline void setVerboseMode(bool on=true) {this->verbose = on;};
private:

  void getParameters() ;
  void sample(vpImage<unsigned char>&image);
  void leastSquare() ;
  void leastSquareParabola();
  //given K0 and K1, find the three others
  void leastSquareParabolaGivenOrientation() ;
  void leastSquareLine() ;//andrea: find parameters in the case of a line
  void updateNormAng();
  void suppressPoints();
  void seekExtremities(vpImage<unsigned char> &I);
  void getCirclePoints();
  void displayList(vpImage<unsigned char> &I);
  void computeNormAng(double &norm_ang, vpColVector &K, 
		      double i, double j, bool isLine);

public:
  
  double aFin, bFin, cFin, thetaFin;//line OR parabola params andrea
  int dataSet; //indicates which data set is being used
  bool line; //indicates that parabola degenerates into line
  double i1,j1, i2, j2 ;
  double *i_cir, *j_cir;
  static const int numPointCir = 100;	//points used to find circle
  
private:
  
  //! vecteur de parametres de la quadrique
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  vpColVector K, K_line, K_par ;
  vpColVector w_line, w_par;
  double aPar, bPar, cPar, thetaPar;
  unsigned *i_ref, *j_ref;//ref parabola point coords andrea

  //flag for leastSquareParabola(): at first iter only use 5 points,
  //then numberOfSignal() points
  bool firstIter;
  double line_error, parab_error, parab_errorTot;
  int lineGoodPoints, parGoodPoints, parGoodPointsTot;
  //flag indicating that sampling is currently being done, hence
  //previous curve should be kept
  bool sampling;
  //sampling is currently being done, hence previous curve should be kept
  bool keepLine;

  // verbose mode
  bool verbose;

  // parameters
  int numPoints; // initial points used to find parabola
};




#endif


