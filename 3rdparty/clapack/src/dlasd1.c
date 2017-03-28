/* dlasd1.f -- translated by f2c (version 20061008).
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

static integer c__0 = 0;
static doublereal c_b7 = 1.;
static integer c__1 = 1;
static integer c_n1 = -1;

/* Subroutine */ int dlasd1_(integer *nl, integer *nr, integer *sqre, 
	doublereal *d__, doublereal *alpha, doublereal *beta, doublereal *u, 
	integer *ldu, doublereal *vt, integer *ldvt, integer *idxq, integer *
	iwork, doublereal *work, integer *info)
{
    /* System generated locals */
    integer u_dim1, u_offset, vt_dim1, vt_offset, i__1;
    doublereal d__1, d__2;

    /* Local variables */
    integer i__, k, m, n, n1, n2, iq, iz, iu2, ldq, idx, ldu2, ivt2, idxc, 
	    idxp, ldvt2;
    extern /* Subroutine */ int dlasd2_(integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *, 
	     doublereal *, integer *, doublereal *, integer *, doublereal *, 
	    doublereal *, integer *, doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, integer *), dlasd3_(
	    integer *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, integer *, doublereal *, doublereal *, integer *, 
	    doublereal *, integer *, doublereal *, integer *, doublereal *, 
	    integer *, integer *, integer *, doublereal *, integer *), 
	    dlascl_(char *, integer *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, doublereal *, integer *, integer *),
	     dlamrg_(integer *, integer *, doublereal *, integer *, integer *, 
	     integer *);
    integer isigma;
    extern /* Subroutine */ int xerbla_(char *, integer *);
    doublereal orgnrm;
    integer coltyp;


/*  -- LAPACK auxiliary routine (version 3.2) -- */
/*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
/*     November 2006 */

/*     .. Scalar Arguments .. */
/*     .. */
/*     .. Array Arguments .. */
/*     .. */

/*  Purpose */
/*  ======= */

/*  DLASD1 computes the SVD of an upper bidiagonal N-by-M matrix B, */
/*  where N = NL + NR + 1 and M = N + SQRE. DLASD1 is called from DLASD0. */

/*  A related subroutine DLASD7 handles the case in which the singular */
/*  values (and the singular vectors in factored form) are desired. */

/*  DLASD1 computes the SVD as follows: */

/*                ( D1(in)  0    0     0 ) */
/*    B = U(in) * (   Z1'   a   Z2'    b ) * VT(in) */
/*                (   0     0   D2(in) 0 ) */

/*      = U(out) * ( D(out) 0) * VT(out) */

/*  where Z' = (Z1' a Z2' b) = u' VT', and u is a vector of dimension M */
/*  with ALPHA and BETA in the NL+1 and NL+2 th entries and zeros */
/*  elsewhere; and the entry b is empty if SQRE = 0. */

/*  The left singular vectors of the original matrix are stored in U, and */
/*  the transpose of the right singular vectors are stored in VT, and the */
/*  singular values are in D.  The algorithm consists of three stages: */

/*     The first stage consists of deflating the size of the problem */
/*     when there are multiple singular values or when there are zeros in */
/*     the Z vector.  For each such occurence the dimension of the */
/*     secular equation problem is reduced by one.  This stage is */
/*     performed by the routine DLASD2. */

/*     The second stage consists of calculating the updated */
/*     singular values. This is done by finding the square roots of the */
/*     roots of the secular equation via the routine DLASD4 (as called */
/*     by DLASD3). This routine also calculates the singular vectors of */
/*     the current problem. */

/*     The final stage consists of computing the updated singular vectors */
/*     directly using the updated singular values.  The singular vectors */
/*     for the current problem are multiplied with the singular vectors */
/*     from the overall problem. */

/*  Arguments */
/*  ========= */

/*  NL     (input) INTEGER */
/*         The row dimension of the upper block.  NL >= 1. */

/*  NR     (input) INTEGER */
/*         The row dimension of the lower block.  NR >= 1. */

/*  SQRE   (input) INTEGER */
/*         = 0: the lower block is an NR-by-NR square matrix. */
/*         = 1: the lower block is an NR-by-(NR+1) rectangular matrix. */

/*         The bidiagonal matrix has row dimension N = NL + NR + 1, */
/*         and column dimension M = N + SQRE. */

/*  D      (input/output) DOUBLE PRECISION array, */
/*                        dimension (N = NL+NR+1). */
/*         On entry D(1:NL,1:NL) contains the singular values of the */
/*         upper block; and D(NL+2:N) contains the singular values of */
/*         the lower block. On exit D(1:N) contains the singular values */
/*         of the modified matrix. */

/*  ALPHA  (input/output) DOUBLE PRECISION */
/*         Contains the diagonal element associated with the added row. */

/*  BETA   (input/output) DOUBLE PRECISION */
/*         Contains the off-diagonal element associated with the added */
/*         row. */

/*  U      (input/output) DOUBLE PRECISION array, dimension(LDU,N) */
/*         On entry U(1:NL, 1:NL) contains the left singular vectors of */
/*         the upper block; U(NL+2:N, NL+2:N) contains the left singular */
/*         vectors of the lower block. On exit U contains the left */
/*         singular vectors of the bidiagonal matrix. */

/*  LDU    (input) INTEGER */
/*         The leading dimension of the array U.  LDU >= max( 1, N ). */

/*  VT     (input/output) DOUBLE PRECISION array, dimension(LDVT,M) */
/*         where M = N + SQRE. */
/*         On entry VT(1:NL+1, 1:NL+1)' contains the right singular */
/*         vectors of the upper block; VT(NL+2:M, NL+2:M)' contains */
/*         the right singular vectors of the lower block. On exit */
/*         VT' contains the right singular vectors of the */
/*         bidiagonal matrix. */

/*  LDVT   (input) INTEGER */
/*         The leading dimension of the array VT.  LDVT >= max( 1, M ). */

/*  IDXQ  (output) INTEGER array, dimension(N) */
/*         This contains the permutation which will reintegrate the */
/*         subproblem just solved back into sorted order, i.e. */
/*         D( IDXQ( I = 1, N ) ) will be in ascending order. */

/*  IWORK  (workspace) INTEGER array, dimension( 4 * N ) */

/*  WORK   (workspace) DOUBLE PRECISION array, dimension( 3*M**2 + 2*M ) */

/*  INFO   (output) INTEGER */
/*          = 0:  successful exit. */
/*          < 0:  if INFO = -i, the i-th argument had an illegal value. */
/*          > 0:  if INFO = 1, an singular value did not converge */

/*  Further Details */
/*  =============== */

/*  Based on contributions by */
/*     Ming Gu and Huan Ren, Computer Science Division, University of */
/*     California at Berkeley, USA */

/*  ===================================================================== */

/*     .. Parameters .. */

/*     .. */
/*     .. Local Scalars .. */
/*     .. */
/*     .. External Subroutines .. */
/*     .. */
/*     .. Intrinsic Functions .. */
/*     .. */
/*     .. Executable Statements .. */

/*     Test the input parameters. */

    /* Parameter adjustments */
    --d__;
    u_dim1 = *ldu;
    u_offset = 1 + u_dim1;
    u -= u_offset;
    vt_dim1 = *ldvt;
    vt_offset = 1 + vt_dim1;
    vt -= vt_offset;
    --idxq;
    --iwork;
    --work;

    /* Function Body */
    *info = 0;

    if (*nl < 1) {
	*info = -1;
    } else if (*nr < 1) {
	*info = -2;
    } else if (*sqre < 0 || *sqre > 1) {
	*info = -3;
    }
    if (*info != 0) {
	i__1 = -(*info);
	xerbla_("DLASD1", &i__1);
	return 0;
    }

    n = *nl + *nr + 1;
    m = n + *sqre;

/*     The following values are for bookkeeping purposes only.  They are */
/*     integer pointers which indicate the portion of the workspace */
/*     used by a particular array in DLASD2 and DLASD3. */

    ldu2 = n;
    ldvt2 = m;

    iz = 1;
    isigma = iz + m;
    iu2 = isigma + n;
    ivt2 = iu2 + ldu2 * n;
    iq = ivt2 + ldvt2 * m;

    idx = 1;
    idxc = idx + n;
    coltyp = idxc + n;
    idxp = coltyp + n;

/*     Scale. */

/* Computing MAX */
    d__1 = abs(*alpha), d__2 = abs(*beta);
    orgnrm = max(d__1,d__2);
    d__[*nl + 1] = 0.;
    i__1 = n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if ((d__1 = d__[i__], abs(d__1)) > orgnrm) {
	    orgnrm = (d__1 = d__[i__], abs(d__1));
	}
/* L10: */
    }
    dlascl_("G", &c__0, &c__0, &orgnrm, &c_b7, &n, &c__1, &d__[1], &n, info);
    *alpha /= orgnrm;
    *beta /= orgnrm;

/*     Deflate singular values. */

    dlasd2_(nl, nr, sqre, &k, &d__[1], &work[iz], alpha, beta, &u[u_offset], 
	    ldu, &vt[vt_offset], ldvt, &work[isigma], &work[iu2], &ldu2, &
	    work[ivt2], &ldvt2, &iwork[idxp], &iwork[idx], &iwork[idxc], &
	    idxq[1], &iwork[coltyp], info);

/*     Solve Secular Equation and update singular vectors. */

    ldq = k;
    dlasd3_(nl, nr, sqre, &k, &d__[1], &work[iq], &ldq, &work[isigma], &u[
	    u_offset], ldu, &work[iu2], &ldu2, &vt[vt_offset], ldvt, &work[
	    ivt2], &ldvt2, &iwork[idxc], &iwork[coltyp], &work[iz], info);
    if (*info != 0) {
	return 0;
    }

/*     Unscale. */

    dlascl_("G", &c__0, &c__0, &c_b7, &orgnrm, &n, &c__1, &d__[1], &n, info);

/*     Prepare the IDXQ sorting permutation. */

    n1 = k;
    n2 = n - k;
    dlamrg_(&n1, &n2, &d__[1], &c__1, &c_n1, &idxq[1]);

    return 0;

/*     End of DLASD1 */

} /* dlasd1_ */
