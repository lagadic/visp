
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: vpLevenbergMarquartd.h,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpLevenbergMarquartd.h,v 1.1 2005-09-02 14:04:14 fspindle Exp $
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef vpLevenbergMarquartd_h
#define vpLevenbergMarquartd_h

#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

#include <visp/vpMath.h>

int
qrsolv (int n, double *r, int ldr, int *ipvt, double *diag, double *qtb,
	double *x, double *sdiag, double *wa) ;

double
enorm (const double *x, int n);


int	
lmpar(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb,
      double *delta, double *par, double *x, double *sdiag, double *wa1, double *wa2);

double 
pythag (double a, double b);

int	
qrfac(int m, int n, double *a, int lda, int *pivot, int *ipvt,
	      int lipvt, double *rdiag, double *acnorm, double *wa);

int	qrsolv (int n, double *r, int ldr, int *ipvt, double *diag, double *qtb,
		double *x, double *sdiag, double *wa);

int	lmder (void (*ptr_fcn)(int m, int n, double *xc, double *fvecc,
			       double *jac, int ldfjac, int iflag), int m, int n, double *x,
	       double *fvec, double *fjac, int ldfjac, double ftol, double xtol,
	       double gtol, int maxfev, double *diag, int mode,
	       const double factor, int nprint, int *info, int *nfev,
	       int *njev, int *ipvt, double *qtf, double *wa1, double *wa2,
	       double *wa3, double *wa4);

int lmder1 (void (*ptr_fcn)(int m, int n, double *xc, double *fvecc,
			    double *jac, int ldfjac, int iflag),
	    int m, int n, double *x, double *fvec, double *fjac,
	    int ldfjac, double tol, int *info, int *ipvt, int lwa, double *wa);


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

