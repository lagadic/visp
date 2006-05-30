/****************************************************************************
 *
 * $Id: vpLevenbergMarquartd.h,v 1.2 2006-05-30 08:40:42 fspindle Exp $
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
 * This file is part of the ViSP toolkit
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
 * Levenberg Marquartd.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 *
 *****************************************************************************/


#ifndef vpLevenbergMarquartd_h
#define vpLevenbergMarquartd_h

#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

#include <visp/vpConfig.h>
#include <visp/vpMath.h>

int VISP_EXPORT
qrsolv (int n, double *r, int ldr, int *ipvt, double *diag,
	double *qtb, double *x, double *sdiag, double *wa) ;

double VISP_EXPORT
enorm (const double *x, int n);

int VISP_EXPORT
lmpar(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb,
      double *delta, double *par, double *x, double *sdiag, double *wa1,
      double *wa2);

double VISP_EXPORT
pythag (double a, double b);

int VISP_EXPORT
qrfac(int m, int n, double *a, int lda, int *pivot, int *ipvt,
      int lipvt, double *rdiag, double *acnorm, double *wa);

int VISP_EXPORT
qrsolv (int n, double *r, int ldr, int *ipvt, double *diag, double *qtb,
	double *x, double *sdiag, double *wa);

int VISP_EXPORT
lmder (void (*ptr_fcn)(int m, int n, double *xc, double *fvecc,
		       double *jac, int ldfjac, int iflag),
       int m, int n, double *x,
       double *fvec, double *fjac, int ldfjac, double ftol, double xtol,
       double gtol, int maxfev, double *diag, int mode,
       const double factor, int nprint, int *info, int *nfev,
       int *njev, int *ipvt, double *qtf, double *wa1, double *wa2,
       double *wa3, double *wa4);

int VISP_EXPORT
lmder1 (void (*ptr_fcn)(int m, int n, double *xc, double *fvecc,
			double *jac, int ldfjac, int iflag),
	int m, int n, double *x, double *fvec, double *fjac,
	int ldfjac, double tol, int *info, int *ipvt, int lwa, double *wa);


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

