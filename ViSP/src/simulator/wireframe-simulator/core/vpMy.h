/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Module de Macros et de Types de bases en langage "C".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef	NULL
#define	NULL		0
#endif
#ifndef	FALSE
#define	FALSE		0
#endif
#ifndef	TRUE
#define	TRUE		1
#endif

#ifndef	STDIN
#define	STDIN		0
#endif
#ifndef	STDOUT
#define	STDOUT		1
#endif
#ifndef	STDERR
#define	STDERR		2
#endif

#define	NAMESIZE	80
#define	LINESIZE	256

#ifdef	gould
#define	BUFSIZE		1024

#define	M_E		2.7182818284590452354
#define M_LN10		2.30258509299404568402
#define	M_PI		3.14159265358979323846
#define	M_PI_2		1.57079632679489661923
#define	M_PI_4		0.78539816339744830962
#endif

#ifdef	sun
#define	BUFSIZE		1024
#endif

#ifdef	vax
#define	BUFSIZE		512

#define	M_E		2.7182818284590452354
#define M_LN10		2.30258509299404568402
#define	M_PI		3.14159265358979323846
#define	M_PI_2		1.57079632679489661923
#define	M_PI_4		0.78539816339744830962
#endif

#define	M_EPSILON	1E-06

//#define	ABS(X)		(((X) < 0)   ? -(X) : (X))
#define	FABS(X)		(((X) < 0.0) ? -(X) : (X))
//#define	MAX(A,B)	(((A) > (B)) ? (A) : (B))
//#define	MAX3(A,B,C)	(MAX(MAX(A,B),C))
//#define	MIN(A,B)	(((A) < (B)) ? (A) : (B))
//#define	MIN3(A,B,C)	(MIN(MIN(A,B),C))

#define	MIN_MAX(M,MIN,MAX)	if ((M) < (MIN)) (MIN) = (M);\
			 	else if ((M) > (MAX)) (MAX) = (M) 

#define	TWO_POWER(P)	(((P) > 0) ? 1 << (P) : 1)
#define	SWAP(A,B,T)	{ (T) = (A); (A) = (B); (B) = (T); }

typedef	unsigned char	Byte;
typedef	unsigned short	Index;
typedef	char		Type;

extern void fscanf_float (float *fp);

#endif
