/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      debug.h
 * Project:   ViSP
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id $
 *
 * Description
 * ============
 *
 * Macro de trace et de debugage
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdio.h>
#include <iostream>
using namespace std;


#define vpIN_FCT(niv, a...)   do {\
    if (DEBUG_MODE >= niv) {\
    cout <<"begin" << __FILE__ << ": " << __FUNCTION__ <<  "(#" << __LINE__ << ") :"; \
    printf (a); printf ("\n"); \
    fflush (stdout); } \
    } while (0)


#define vpOUT_FCT(niv, a...)   do {\
    if (DEBUG_MODE >= niv) {\
    cout <<"begin" << __FILE__ << ": " << __FUNCTION__ <<  "(#" << __LINE__ << ") :"; \
    printf (a); printf ("\n"); \
    fflush (stdout); } \
    } while (0)



#define ERROR_TRACE(a...)   do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0)
#define cerror cerr << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"
#define TRACE(a...)    do {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a); \
    printf ("\n"); \
    fflush (stdout); } while (0)
#define ctrace cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

#ifdef DEBUG
#define DERROR_TRACE(niv, a...)  do {\
    if (DEBUG_MODE >= niv) {\
    cerr <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } \
    } while (0)
#define DEBUG_TRACE(niv, a...)   do {\
    if (DEBUG_MODE >= niv) {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a); printf ("\n"); \
    fflush (stdout); } \
    } while (0)
#define cdebug(niv) if (DEBUG_MODE < niv) ; else \
    cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

#else
#define DERROR_TRACE(niv, a...)  do {} while (0)
#define DEBUG_TRACE(niv, a...)   do {} while (0)
#define cdebug(niv) if (1) ; else cout
#endif

#ifdef DEFENSIF
#define DEFENSIF(a)  (a)
#else
#define DEFENSIF(a)  (0)
#endif



#endif /* __SEQUENCE_HH */

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
