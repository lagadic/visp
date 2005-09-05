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

#ifndef WIN32
////////////////////////////////////////////////////////////////////////////
// Unix system
//
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
#define CERROR cerr << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"
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

////////////////////////////////////////////////////////////////////////////
// Windows system
//
#else

#ifndef __FUNCTION__
#define __FUNCTION__ " "
#endif

#define vpIN_FCT(niv, a, b)   do {\
    if (DEBUG_MODE >= niv) {\
    cout <<"begin" << __FILE__ << ": " << __FUNCTION__ <<  "(#" << __LINE__ << ") :"; \
    printf (a); printf ("\n"); \
    fflush (stdout); } \
    } while (0)


#define vpOUT_FCT(niv, a, b)   do {\
    if (DEBUG_MODE >= niv) {\
    cout <<"begin" << __FILE__ << ": " << __FUNCTION__ <<  "(#" << __LINE__ << ") :"; \
    printf (a); printf ("\n"); \
    fflush (stdout); } \
    } while (0)



inline void ERROR_TRACE(char *a) {  do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0); };
inline void ERROR_TRACE(char *a, int b) {  do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a, b); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0); };
inline void ERROR_TRACE(char *a, double b) {  do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a, b); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0); };
inline void ERROR_TRACE(char *a, const char *b) {  do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a, b); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0); };
inline void ERROR_TRACE(char *a, unsigned b, const char *c) {  do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a, b, c); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0); };
inline void ERROR_TRACE(char *a, double b, double c) {  do {\
    cerr << "!!\t" << __FILE__ << ": " <<__FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    fprintf (stderr, a, b, c); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } while (0); };
#define cerror cerr << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"
inline void TRACE(char *a) {   do {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a); \
    printf ("\n"); \
    fflush (stdout); } while (0); };
inline void TRACE(char *a, int b) {   do {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a, b); \
    printf ("\n"); \
    fflush (stdout); } while (0); };
inline void TRACE(char *a, double b, double c) {   do {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a, b, c); \
    printf ("\n"); \
    fflush (stdout); } while (0); };
inline void TRACE(char *a, double b, double c, double d) {   do {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a, b, c, d); \
    printf ("\n"); \
    fflush (stdout); } while (0); };
#define ctrace cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

#ifdef DEBUG
#define DERROR_TRACE(niv, a, b)  do {\
    if (DEBUG_MODE >= niv) {\
    cerr <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a); \
    fprintf (stderr, "\n"); \
    fflush (stderr); } \
    } while (0)
#define DEBUG_TRACE(niv, a)   do {\
    if (DEBUG_MODE >= niv) {\
    cout <<__FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :" ; \
    printf (a); printf ("\n"); \
    fflush (stdout); } \
    } while (0)
#define cdebug(niv) if (DEBUG_MODE < niv) ; else \
    cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

#else
#define DERROR_TRACE(niv, a, b)  do {} while (0)
inline void DEBUG_TRACE(int niv, char *a) {  do {} while (0); };
inline void DEBUG_TRACE(int niv, char *a, char *b) { do {} while (0); };
inline void DEBUG_TRACE(int niv, char *a, unsigned char *b) { do {} while (0); };
inline void DEBUG_TRACE(int niv, char *a, unsigned char **b) { do {} while (0); };
inline void DEBUG_TRACE(int niv, char *a, int b, int c) { do {} while (0); };
#define cdebug(niv) if (1) ; else cout
#endif

#endif
////////////////////////////////////////////////////////////////////////////

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
