/* dlauu2.f -- translated by f2c (version 20061008).
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

/* Table of constant values */

static doublereal c_b7 = 1.;
static integer c__1 = 1;

/* Subroutine */ int dlauu2_(char *uplo, integer *n, doublereal *a, integer *
	lda, integer *info)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2, i__3;

    /* Local variables */
    integer i__;
    doublereal aii;
    extern doublereal ddot_(integer *, doublereal *, integer *, doublereal *, 
	    integer *);
    extern /* Subroutine */ int dscal_(integer *, doublereal *, doublereal *, 
	    integer *);
    extern logical lsame_(char *, char *);
    extern /* Subroutine */ int dgemv_(char *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, doublereal *, integer *, 
	    doublereal *, doublereal *, integer *);
    logical upper;
    extern /* Subroutine */ int xerbla_(char *, integer *);


/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*     .. Scalar Arguments .. */
/*     .. */
/*     .. Array Arguments .. */
/*     .. */

/*  Purpose */
/*  ======= */

/*  DLAUU2 computes the product U * U' or L' * L, where the triangular */
/*  factor U or L is stored in the upper or lower triangular part of */
/*  the array A. */

/*  If UPLO = 'U' or 'u' then the upper triangle of the result is stored, */
/*  overwriting the factor U in A. */
/*  If UPLO = 'L' or 'l' then the lower triangle of the result is stored, */
/*  overwriting the factor L in A. */

/*  This is the unblocked form of the algorithm, calling Level 2 BLAS. */

/*  Arguments */
/*  ========= */

/*  UPLO    (input) CHARACTER*1 */
/*          Specifies whether the triangular factor stored in the array A */
/*          is upper or lower triangular: */
/*          = 'U':  Upper triangular */
/*          = 'L':  Lower triangular */

/*  N       (input) INTEGER */
/*          The order of the triangular factor U or L.  N >= 0. */

/*  A       (input/output) DOUBLE PRECISION array, dimension (LDA,N) */
/*          On entry, the triangular factor U or L. */
/*          On exit, if UPLO = 'U', the upper triangle of A is */
/*          overwritten with the upper triangle of the product U * U'; */
/*          if UPLO = 'L', the lower triangle of A is overwritten with */
/*          the lower triangle of the product L' * L. */

/*  LDA     (input) INTEGER */
/*          The leading dimension of the array A.  LDA >= max(1,N). */

/*  INFO    (output) INTEGER */
/*          = 0: successful exit */
/*          < 0: if INFO = -k, the k-th argument had an illegal value */

/*  ===================================================================== */

/*     .. Parameters .. */
/*     .. */
/*     .. Local Scalars .. */
/*     .. */
/*     .. External Functions .. */
/*     .. */
/*     .. External Subroutines .. */
/*     .. */
/*     .. Intrinsic Functions .. */
/*     .. */
/*     .. Executable Statements .. */

/*     Test the input parameters. */

    /* Parameter adjustments */
    a_dim1 = *lda;
    a_offset = 1 + a_dim1;
    a -= a_offset;

    /* Function Body */
    *info = 0;
    upper = lsame_(uplo, "U");
    if (! upper && ! lsame_(uplo, "L")) {
	*info = -1;
    } else if (*n < 0) {
	*info = -2;
    } else if (*lda < max(1,*n)) {
	*info = -4;
    }
    if (*info != 0) {
	i__1 = -(*info);
	xerbla_("DLAUU2", &i__1);
	return 0;
    }

/*     Quick return if possible */

    if (*n == 0) {
	return 0;
    }

    if (upper) {

/*        Compute the product U * U'. */

	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    aii = a[i__ + i__ * a_dim1];
	    if (i__ < *n) {
		i__2 = *n - i__ + 1;
		a[i__ + i__ * a_dim1] = ddot_(&i__2, &a[i__ + i__ * a_dim1], 
			lda, &a[i__ + i__ * a_dim1], lda);
		i__2 = i__ - 1;
		i__3 = *n - i__;
		dgemv_("No transpose", &i__2, &i__3, &c_b7, &a[(i__ + 1) * 
			a_dim1 + 1], lda, &a[i__ + (i__ + 1) * a_dim1], lda, &
			aii, &a[i__ * a_dim1 + 1], &c__1);
	    } else {
		dscal_(&i__, &aii, &a[i__ * a_dim1 + 1], &c__1);
	    }
/* L10: */
	}

    } else {

/*        Compute the product L' * L. */

	i__1 = *n;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    aii = a[i__ + i__ * a_dim1];
	    if (i__ < *n) {
		i__2 = *n - i__ + 1;
		a[i__ + i__ * a_dim1] = ddot_(&i__2, &a[i__ + i__ * a_dim1], &
			c__1, &a[i__ + i__ * a_dim1], &c__1);
		i__2 = *n - i__;
		i__3 = i__ - 1;
		dgemv_("Transpose", &i__2, &i__3, &c_b7, &a[i__ + 1 + a_dim1], 
			 lda, &a[i__ + 1 + i__ * a_dim1], &c__1, &aii, &a[i__ 
			+ a_dim1], lda);
	    } else {
		dscal_(&i__, &aii, &a[i__ + a_dim1], lda);
	    }
/* L20: */
	}
    }

    return 0;

/*     End of DLAUU2 */

} /* dlauu2_ */
