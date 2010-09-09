/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpCgiconstants.h>	/* defines constants   */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

typedef char    Cchar;

typedef float   Cfloat;

typedef int     Cint;

typedef enum
{
    ACTIVE, INACTIVE
}               Cactstate;

typedef enum
{
    CLEAR, NO_OP, RETAIN
}               Cacttype;


typedef enum
{
    INDIVIDUAL, BUNDLED
}               Casptype;

typedef enum
{
    VALID_DATA,
    TIMED_OUT,
    DISABLED,
    WRONG_STATE,
    NOT_SUPPORTED
}               Cawresult;

typedef enum
{
    TRANSPARENT, OPAQUE
}               Cbmode;

typedef enum
{
    BITNOT, BITTRUE
}               Cbitmaptype;

typedef enum
{
    HOLLOW, SOLIDI, PATTERN, HATCH
}               Cintertype;

typedef enum
{
    STRING, CHARACTER, STROKE
}               Cprectype;

typedef enum
{
    SOLID, DOTTED, DASHED, DASHED_DOTTED, DASH_DOT_DOTTED, LONG_DASHED
}               Clintype;

typedef enum
{
    DOT, PLUS, ASTERISK, CIRCLE, X
}               Cmartype;

typedef struct
{
    Clintype        line_type;
    Cfloat          line_width;
    Cint            line_color;
    Cmartype        marker_type;
    Cfloat          marker_size;
    Cint            marker_color;
    Cintertype      interior_style;
    Cint            hatch_index;
    Cint            pattern_index;
    Cint            fill_color;
    Clintype        perimeter_type;
    Cfloat          perimeter_width;
    Cint            perimeter_color;
    Cint            text_font;
    Cprectype       text_precision;
    Cfloat          character_expansion;
    Cfloat          character_spacing;
    Cint            text_color;
}               Cbunatt;

typedef struct
{
    unsigned char  *ra;
    unsigned char  *ga;
    unsigned char  *ba;
    Cint            n;
}               Ccentry;

typedef enum
{
    OPEN, CLOSE
}               Ccflag;

typedef enum
{
    YES, NO
}               Cchangetype;

typedef enum
{
    NOCLIP,
    CLIP,
    CLIP_RECTANGLE
}               Cclip;

typedef enum
{
    CHORD, PIE
}               Cclosetype;

typedef enum
{
    REPLACE, AND, OR, NOT, XOR
}               Ccombtype;

typedef struct
{
    Cint            x;
    Cint            y;
}               Ccoor;

typedef struct
{
    Ccoor          *ptlist;
    Cint            n;
}               Ccoorlist;

typedef struct
{
    Ccoor          *upper;
    Ccoor          *lower;
}               Ccoorpair;

typedef enum
{
    IC_LOCATOR,
    IC_STROKE,
    IC_VALUATOR,
    IC_CHOICE,
    IC_STRING,
    IC_PICK
}               Cdevoff;

typedef enum
{
    E_TRACK,
    E_ECHO,
    E_TRACK_OR_ECHO,
    E_TRACK_AND_ECHO
}               Cechoav;

typedef enum
{
    ECHO_OFF, ECHO_ON, TRACK_ON
}               Cechostate;

typedef enum
{
    NO_ECHO, PRINTERS_FIST, HIGHLIGHT, RUBBER_BAND_BOX,
    DOTTED_LINE, SOLID_LINE, STRING_ECHO, XLINE, YLINE
}               Cechotype;

typedef struct
{
    Cint            n;
    Cechoav        *elements;
    Cechotype      *echos;
}               Cechotypelst;

typedef struct
{
    Cechostate     *echos;
    Cint            n;
}               Cechostatelst;

typedef struct
{
    int             segid;	/* segment */
    int             pickid;	/* pick id */
}               Cpick;

typedef struct
{
    Ccoor          *xypt;	/* LOCATOR */
    Ccoorlist      *points;	/* STROKE devices */
    Cfloat          val;	/* VALUATOR device */
    Cint            choice;	/* CHOICE devices */
    Cchar          *string;	/* STRING device */
    Cpick          *pick;	/* PICK devices */
}               Cinrep;

typedef struct
{
    Cinrep         *echos;
    Cint            n;
}               Cechodatalst;

typedef enum
{
    NATURAL, POINT, BEST_FIT
}               Cendstyle;

typedef enum
{
    NO_OFLO, OFLO
}               Ceqflow;

typedef Cint    Cerror;

typedef enum
{
    INTERRUPT, NO_ACTION, POLL
}               Cerrtype;

typedef enum
{
    CLIP_RECT, VIEWPORT, VIEWSURFACE
}               Cexttype;

typedef enum
{
    OFF, ON
}               Cflag;

typedef struct
{
    Cintertype      style;
    Cflag           visible;
    Cint            color;
    Cint            hatch_index;
    Cint            pattern_index;
    Cint            index;
    Clintype        pstyle;
    Cfloat          pwidth;
    Cint            pcolor;
}               Cfillatt;

typedef struct
{
    Cint            n;
    Cint           *num;
    Casptype       *value;
}               Cflaglist;

typedef enum
{
    FREEZE, REMOVE
}               Cfreeze;

typedef struct
{
    Clintype        style;
    Cfloat          width;
    Cint            color;
    Cint            index;
}               Clinatt;

typedef enum
{
    L_FALSE, L_TRUE
}               Clogical;

typedef struct pixrect Cpixrect;

typedef enum
{
    RELEASE, NO_EVENTS, REQUEST_EVENT, RESPOND_EVENT, QUEUE_EVENT
}               Clidstate;

typedef struct
{
    Cmartype        type;
    Cfloat          size;
    Cint            color;
    Cint            index;
}               Cmarkatt;

typedef enum
{
    NO_INPUT, ALWAYS_ON, SETTABLE, DEPENDS_ON_LID
}               Cinputability;

typedef struct
{
    Cint            cur_index;
    Cint            row;
    Cint            column;
    Cint           *colorlist;
    Ccoor          *point;
    Cint            dx;
    Cint            dy;
}               Cpatternatt;

typedef enum
{
    PROMPT_OFF, PROMPT_ON
}               Cpromstate;

typedef enum
{
    ACK_ON, ACK_OFF
}               Cackstate;


typedef struct
{
    Cint            n;
    Cdevoff        *class;
    Cint           *assoc;
}               Cassoclid;

typedef struct
{
    Clidstate       state;
    Cpromstate      prompt;
    Cackstate       acknowledgement;
    Cinrep         *current;
    Cint            n;
    Cint           *triggers;
    Cechotype       echotyp;
    Cechostate      echosta;
    Cint            echodat;
}               Cstatelist;

typedef struct
{
    Clogical        sample;
    Cchangetype     change;
    Cint            numassoc;
    Cint           *trigassoc;
    Cinputability   prompt;
    Cinputability   acknowledgement;
    Cechotypelst   *echo;
    Cchar          *classdep;
    Cstatelist      state;
}               Cliddescript;

typedef enum
{
    SIMULTANEOUS_EVENT_FOLLOWS, SINGLE_EVENT
}               Cmesstype;

typedef enum
{
    RIGHT, LEFT, UP, DOWN
}               Cpathtype;

typedef enum
{
    LFT, CNTER, RGHT, NRMAL, CNT
}               Chaligntype;

typedef enum
{
    TOP, CAP, HALF, BASE, BOTTOM, NORMAL, CONT
}               Cvaligntype;

typedef enum
{
    NOT_VALID, EMPTY, NON_EMPTY, ALMOST_FULL, FULL
}               Cqtype;

typedef enum
{
    ABSOLUTE, SCALED
}               Cspecmode;

typedef enum
{
    NONE, REQUIRED_FUNCTIONS_ONLY, SOME_NON_REQUIRED_FUNCTIONS,
    ALL_NON_REQUIRED_FUNCTIONS
}               Csuptype;

typedef struct
{
    Cint            fontset;
    Cint            index;
    Cint            current_font;
    Cprectype       precision;
    Cfloat          exp_factor;
    Cfloat          space;
    Cint            color;
    Cint            height;
    Cfloat          basex;
    Cfloat          basey;
    Cfloat          upx;
    Cfloat          upy;
    Cpathtype       path;
    Chaligntype     halign;
    Cvaligntype     valign;
    Cfloat          hcalind;
    Cfloat          vcalind;
}               Ctextatt;

typedef enum
{
    NOT_FINAL, FINAL
}               Ctextfinal;

typedef struct
{
    Cchangetype     change;
    Cassoclid      *numassoc;
    Cint            maxassoc;
    Cpromstate      prompt;
    Cackstate       acknowledgement;
    Cchar          *name;
    Cchar          *description;
}               Ctrigdis;

typedef struct
{
    Cactstate       state;
    Cassoclid      *assoc;
}               Ctrigstate;

typedef enum
{
    INTEGER, REAL, BOTH
}               Cvdctype;

typedef struct
{
    Cint            numloc;
    Cint            numval;
    Cint            numstrk;
    Cint            numchoice;
    Cint            numstr;
    Cint            numtrig;
    Csuptype        event_queue;
    Csuptype        asynch;
    Csuptype        coord_map;
    Csuptype        echo;
    Csuptype        tracking;
    Csuptype        prompt;
    Csuptype        acknowledgement;
    Csuptype        trigger_manipulation;
}               Ccgidesctab;

typedef struct
{
    Cchar           screenname[DEVNAMESIZE];	/* physical screen */
    Cchar           windowname[DEVNAMESIZE];	/* window */
    Cint            windowfd;	/* window file */
    Cint            retained;	/* retained flag */
    Cint            dd;		/* device */
    Cint            cmapsize;	/* color map size */
    Cchar           cmapname[DEVNAMESIZE];	/* color map name */
    Cint            flags;	/* new flag */
    Cchar         **ptr;	/* CGI tool descriptor */
}               Cvwsurf;



/* define abnormal function calls */

Cpixrect       *inquire_device_bitmap();
Clinatt        *inquire_line_attributes();
Cmarkatt       *inquire_marker_attributes();
Cfillatt       *inquire_fill_area_attributes();
Cpatternatt    *inquire_pattern_attributes();
Ctextatt       *inquire_text_attributes();
Cflaglist      *inquire_aspect_source_flags();

/* define abnormal CGIPW function calls */
Clinatt        *cgipw_inquire_line_attributes();
Cmarkatt       *cgipw_inquire_marker_attributes();
Cfillatt       *cgipw_inquire_fill_area_attributes();
Cpatternatt    *cgipw_inquire_pattern_attributes();
Ctextatt       *cgipw_inquire_text_attributes();
Cflaglist      *cgipw_inquire_aspect_source_flags();
#endif

