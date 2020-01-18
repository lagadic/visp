/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Module de Macros et de Types de bases en langage "C".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/
#ifndef vpMy_H
#define vpMy_H

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/robot/vpWireFrameSimulatorTypes.h>

#ifndef NULL
#define NULL 0
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef STDIN
#define STDIN 0
#endif
#ifndef STDOUT
#define STDOUT 1
#endif
#ifndef STDERR
#define STDERR 2
#endif

#define NAMESIZE 80
#define LINESIZE 256

#define M_EPSILON 1E-06

//#define	ABS(X)		(((X) < 0)   ? -(X) : (X))
#define FABS(X) (((X) < 0.0) ? -(X) : (X))
//#define	MAX(A,B)	(((A) > (B)) ? (A) : (B))
//#define	MAX3(A,B,C)	(MAX(MAX(A,B),C))
//#define	MIN(A,B)	(((A) < (B)) ? (A) : (B))
//#define	MIN3(A,B,C)	(MIN(MIN(A,B),C))

#define MIN_MAX(M, MIN, MAX)                                                                                           \
  if ((M) < (MIN))                                                                                                     \
    (MIN) = (M);                                                                                                       \
  else if ((M) > (MAX))                                                                                                \
  (MAX) = (M)

#define TWO_POWER(P) (((P) > 0) ? 1 << (P) : 1)
#define SWAP(A, B, T)                                                                                                  \
  {                                                                                                                    \
    (T) = (A);                                                                                                         \
    (A) = (B);                                                                                                         \
    (B) = (T);                                                                                                         \
  }

typedef unsigned char Byte;

#endif
#endif
