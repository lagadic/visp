/* dlas2.f -- translated by f2c (version 20061008).
   You must link the resulting object file with libf2c:
	on Microsoft Windows system, link with libf2c.lib;
	on Linux or Unix systems, link with .../path/to/libf2c.a -lm
	or, if you install libf2c.a in a standard place, with -lf2c -lm
	-- in that order, at the end of the command line, as in
		cc *.o -lf2c -lm
	Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

		http://www.netlib.org/f2c/libf2c.zip
*/

#include "f2c.h"
#include "blaswrap.h"

/* Subroutine */ int dlas2_(doublereal *f, doublereal *g, doublereal *h__, 
	doublereal *ssmin, doublereal *ssmax)
{
    /* System generated locals */
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    doublereal c__, fa, ga, ha, as, at, au, fhmn, fhmx;


/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*     .. Scalar Arguments .. */
/*     .. */

/*  Purpose */
/*  ======= */

/*  DLAS2  computes the singular values of the 2-by-2 matrix */
/*     [  F   G  ] */
/*     [  0   H  ]. */
/*  On return, SSMIN is the smaller singular value and SSMAX is the */
/*  larger singular value. */

/*  Arguments */
/*  ========= */

/*  F       (input) DOUBLE PRECISION */
/*          The (1,1) element of the 2-by-2 matrix. */

/*  G       (input) DOUBLE PRECISION */
/*          The (1,2) element of the 2-by-2 matrix. */

/*  H       (input) DOUBLE PRECISION */
/*          The (2,2) element of the 2-by-2 matrix. */

/*  SSMIN   (output) DOUBLE PRECISION */
/*          The smaller singular value. */

/*  SSMAX   (output) DOUBLE PRECISION */
/*          The larger singular value. */

/*  Further Details */
/*  =============== */

/*  Barring over/underflow, all output quantities are correct to within */
/*  a few units in the last place (ulps), even in the absence of a guard */
/*  digit in addition/subtraction. */

/*  In IEEE arithmetic, the code works correctly if one matrix element is */
/*  infinite. */

/*  Overflow will not occur unless the largest singular value itself */
/*  overflows, or is within a few ulps of overflow. (On machines with */
/*  partial overflow, like the Cray, overflow may occur if the largest */
/*  singular value is within a factor of 2 of overflow.) */

/*  Underflow is harmless if underflow is gradual. Otherwise, results */
/*  may correspond to a matrix modified by perturbations of size near */
/*  the underflow threshold. */

/*  ==================================================================== */

/*     .. Parameters .. */
/*     .. */
/*     .. Local Scalars .. */
/*     .. */
/*     .. Intrinsic Functions .. */
/*     .. */
/*     .. Executable Statements .. */

    fa = abs(*f);
    ga = abs(*g);
    ha = abs(*h__);
    fhmn = min(fa,ha);
    fhmx = max(fa,ha);
    if (fhmn == 0.) {
	*ssmin = 0.;
	if (fhmx == 0.) {
	    *ssmax = ga;
	} else {
/* Computing 2nd power */
	    d__1 = min(fhmx,ga) / max(fhmx,ga);
	    *ssmax = max(fhmx,ga) * sqrt(d__1 * d__1 + 1.);
	}
    } else {
	if (ga < fhmx) {
	    as = fhmn / fhmx + 1.;
	    at = (fhmx - fhmn) / fhmx;
/* Computing 2nd power */
	    d__1 = ga / fhmx;
	    au = d__1 * d__1;
	    c__ = 2. / (sqrt(as * as + au) + sqrt(at * at + au));
	    *ssmin = fhmn * c__;
	    *ssmax = fhmx / c__;
	} else {
	    au = fhmx / ga;
	    if (au == 0.) {

/*              Avoid possible harmful underflow if exponent range */
/*              asymmetric (true SSMIN may not underflow even if */
/*              AU underflows) */

		*ssmin = fhmn * fhmx / ga;
		*ssmax = ga;
	    } else {
		as = fhmn / fhmx + 1.;
		at = (fhmx - fhmn) / fhmx;
/* Computing 2nd power */
		d__1 = as * au;
/* Computing 2nd power */
		d__2 = at * au;
		c__ = 1. / (sqrt(d__1 * d__1 + 1.) + sqrt(d__2 * d__2 + 1.));
		*ssmin = fhmn * c__ * au;
		*ssmin += *ssmin;
		*ssmax = ga / (c__ + c__);
	    }
	}
    }
    return 0;

/*     End of DLAS2 */

} /* dlas2_ */
