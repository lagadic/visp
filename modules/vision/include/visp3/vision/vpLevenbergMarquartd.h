/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Levenberg Marquartd.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 *
 *****************************************************************************/

#ifndef vpLevenbergMarquartd_h
#define vpLevenbergMarquartd_h

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int VISP_EXPORT qrsolv(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *x, double *sdiag,
                       double *wa);

double VISP_EXPORT enorm(const double *x, int n);

int VISP_EXPORT lmpar(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *delta, double *par,
                      double *x, double *sdiag, double *wa1, double *wa2);

double VISP_EXPORT pythag(double a, double b);

int VISP_EXPORT qrfac(int m, int n, double *a, int lda, int *pivot, int *ipvt, int lipvt, double *rdiag, double *acnorm,
                      double *wa);

int VISP_EXPORT qrsolv(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *x, double *sdiag,
                       double *wa);

int VISP_EXPORT lmder(void (*ptr_fcn)(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag),
                      int m, int n, double *x, double *fvec, double *fjac, int ldfjac, double ftol, double xtol,
                      double gtol, unsigned int maxfev, double *diag, int mode, const double factor, int nprint,
                      int *info, unsigned int *nfev, int *njev, int *ipvt, double *qtf, double *wa1, double *wa2,
                      double *wa3, double *wa4);

int VISP_EXPORT lmder1(void (*ptr_fcn)(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag),
                       int m, int n, double *x, double *fvec, double *fjac, int ldfjac, double tol, int *info,
                       int *ipvt, int lwa, double *wa);

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
