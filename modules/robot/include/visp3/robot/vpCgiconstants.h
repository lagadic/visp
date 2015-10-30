/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#ifndef vpCgiconstants_H
#define vpCgiconstants_H
 
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/* errors */
#define NO_ERROR   0
#define ENOTCGCL   1
#define ENOTCGOP   2
#define	ENOTVSOP   3
#define	ENOTVSAC   4
#define	ENOTOPOP   5
#define	EVSIDINV  10
#define	ENOWSTYP  11
#define	EMAXVSOP  12
#define	EVSNOTOP  13
#define	EVSISACT  14
#define	EVSNTACT  15
#define	EINQALTL  16
#define	EBADRCTD  20
#define	EBDVIEWP  21
#define	ECLIPTOL  22
#define	ECLIPTOS  23
#define	EVDCSDIL  24
#define	EBTBUNDL  30
#define	EBBDTBDI  31
#define	EBTUNDEF  32
#define	EBADLINX  33
#define	EBDWIDTH  34
#define	ECINDXLZ  35
#define	EBADCOLX  36
#define	EBADMRKX  37
#define	EBADSIZE  38
#define	EBADFABX  39
#define	EPATARTL  40
#define	EPATSZTS  41
#define	ESTYLLEZ  42
#define	ENOPATNX  43
#define	EPATITOL  44
#define	EBADTXTX  45
#define	EBDCHRIX  46
#define	ETXTFLIN  47
#define	ECEXFOOR  48
#define	ECHHTLEZ  49
#define	ECHRUPVZ  50
#define	ECOLRNGE  51
#define	ENMPTSTL  60
#define	EPLMTWPT  61
#define	EPLMTHPT  62
#define	EGPLISFL  63
#define	EARCPNCI  64
#define	EARCPNEL  65
#define	ECELLATS  66
#define	ECELLPOS  67
#define	ECELLTLS  68
#define	EVALOVWS  69
#define	EPXNOTCR  70
#define	EINDNOEX  80
#define	EINDINIT  81
#define	EINDALIN  82
#define	EINASAEX  83
#define	EINAIIMP  84
#define	EINNTASD  85
#define	EINTRNEX  86
#define	EINNECHO  87
#define	EINECHON  88
#define	EINEINCP  89
#define	EINERVWS  90
#define	EINETNSU  91
#define	EINENOTO  92
#define	EIAEVNEN  93
#define	EINEVNEN  94
#define	EBADDATA  95
#define	ESTRSIZE  96
#define	EINQOVFL  97
#define	EINNTRQE  98
#define	EINNTRSE  99
#define	EINNTQUE 100
#define	EMEMSPAC 110
#define	ENOTCSTD 111
#define	ENOTCCPW 112
#define	EFILACC  113
#define	ECGIWIN  114

/* devices */
#define BW1DD 	      1
#define BW2DD	      2
#define CG1DD 	      3
#define BWPIXWINDD    4
#define CGPIXWINDD    5
#define GP1DD 	      6
#define CG2DD 	      7
#define CG4DD 	      8
#define PIXWINDD      9

#define VWSURF_NEWFLG  1
#define _CGI_KEYBORDS  1
#define _CGI_LOCATORS  4
#define _CGI_STROKES   3
#define _CGI_VALUATRS  3
#define _CGI_CHOICES   3
#define _CGI_PICKS     3
#define MAXVWS 	       5
#define MAXTRIG        6
#define MAXASSOC       5
#define MAXEVENTS      1024

/* limits */
#define MAXAESSIZE	  10	/* maximum number of AES table entries */
#define MAXNUMPATS	  50	/* maximum number of pattern table entries */
#define MAXPATSIZE	 256	/* maximum pattern size */

#define MAXPTS		1024	/* maximum number of pts per polygon */
#define MAXCHAR		 256	/* maximum number of chars in a string */

#define OUTFUNS		  67	/* number of output functions */
#define INFUNS		  22	/* number of input functions */

/* attributes */

/* fonts */
#define ROMAN		0
#define GREEK		1
#define SCRIPT		2
#define OLDENGLISH	3
#define STICK		4
#define SYMBOLS		5

#define DEVNAMESIZE	20
/* Warning: Because of the ; separators,
 * do not use the following macro unless it is surrounded with { }s.
 * Beware interactions with if-else and ?: constructs.
 */
#define NORMAL_VWSURF(dev,surf) \
	dev.screenname[0] = '\0'; \
	dev.windowname[0] = '\0'; \
	dev.windowfd = 0;  \
	dev.retained = 0; \
	dev.dd = surf; \
	dev.cmapsize = 0; \
	dev.cmapname[0] ='\0';  \
	dev.flags = 0; \
	dev.ptr= '\0' ;


#endif
#endif
