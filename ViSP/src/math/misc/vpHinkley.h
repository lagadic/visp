/****************************************************************************
 *
 *       Copyright (c) 2001 by IRISA/INRIA Rennes.
 *       All Rights Reserved.
 *
 *       IRISA/INRIA Rennes
 *       Campus Universitaire de Beaulieu
 *       35042 Rennes Cedex
 *
 ****************************************************************************/

#ifndef vpHinkley_H
#define vpHinkley_H

/*!
  \file vpHinkley.h
*/

/*! \enum EHinkleyJump
  Indicates if a jump is detected by the Hinkley test.
*/
typedef enum {
  noJump, /*!< No jump is detected by the Hinkley test. */
  downwardJump, /*!< A downward jump is detected by the Hinkley test. */
  upwardJump /*!< An upward jump is detected by the Hinkley test. */
} hinkleyJump;

/*!
  \brief Test de Hinckley
*/
class vpHinkley
{
 public:
  vpHinkley();
  ~vpHinkley();
  vpHinkley(double alpha, double delta);

  void init();
  void init(double alpha, double delta) ;

  void setDelta(double delta);
  void setAlpha(double alpha);
  hinkleyJump testDownwardJump(double data);
  hinkleyJump testUpwardJump(double data);
  hinkleyJump testDownUpwardJump(double data);
  void setIter(int iter); // For debug only

  static void print(hinkleyJump jump) ;
 private:
  void computeMean(double data);
  void computeSk(double data);
  void computeMk();
  void computeTk(double data);
  void computeNk();

 private:
  double dmin2;
  double alpha;
  int    ndata;	// Signal lenght
  double mean;	// Data mean value
  double Sk;
  double Mk;
  double Tk;
  double Nk;

  int iter; // for debug only
};

#endif
