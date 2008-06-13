/****************************************************************************
 *
 * $Id: vpDebug.h,v 1.10 2008-06-13 13:37:37 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Debug and trace macro.
 *
 *   - TRACING:    vpTRACE and vpERROR_TRACE work like printf with carreer return at the end of the string.
 *                 vpCERROR et vpCTRACE work like the C++ output streams std::cout and std::cerr.
 *   - DEBUGING:   vpDEBUG_TRACE(niv,  and vpDERROR_TRACE(niv, work like printf, but print only if the
 *                 tracing level niv is greater than the debug level VP_DEBUG_MODE.
 *                 vpCDEBUG(niv) work like the C++ output stream std::cout.
 *                 vpDEBUG_ENABLE(niv) is equal to 1 if the debug level niv is greater than the debug mode
 *                 VP_DEBUG_MODE, 0 else.
 *   - PROG DEFENSIVE: DEFENSIF(a) is equal to a if defensive mode is active, 0 else.
 *
 * Authors:
 * Nicolas Mansard, Bruno Renier
 *
 *****************************************************************************/

#ifndef __VP_DEBUG_HH
#define __VP_DEBUG_HH

#include <stdio.h>
#include <stdarg.h>
#include <iostream>


#ifdef WIN32
#  ifndef __FUNCTION__
#    define __FUNCTION__ " "
#  endif
#endif

#ifndef VP_DEBUG_MODE
#  define VP_DEBUG_MODE 0
#endif


/*!
  \class vpTraceOutput
  \brief This class is used to display debug or error messages.
  It needs to be initialized with the file name, function name and line, of
  the place where it is created.
  It is best used by first instanciating the object and directly calling the () operator.
  This is used to mimic variadic macros (not supported in MSVC prior to version 8)
*/
class vpTraceOutput
{
private:
    const char* currentFile; //Name of the file to use in the displays
    const char* currentFunc; //Name of the function to use in the displays
    int currentLine;		 //Line to use in the displays

    //if true, output to std::cerr/stderr else std::cout/stdout
    bool err;
    //string to display before anything else
    const char* header;

public:
    /*!
      Constructor.
      \param file Should be the name of the file where this constructor was called.
      \param line Should be the line in file where this constructor was called.
      \param func Should be the name of the function where this constructor was called.
      \param error If true, writes to the error stream.
      \param s String to print before any other message (acts like a header).
      \note Call the constructor with something like vpTraceOutput(__FILE__,__LINE__, __FUNCTION__).
    */
    vpTraceOutput(const char* file, int line, const char* func, bool error=false, const char * s=NULL) :
        currentFile(file),
	currentFunc(func),
	currentLine(line),
	err(error),
	header(s)
    {}

    /*!
      Displays a string if the debug level is inferior to VP_DEBUG_MODE.
      \param niv Level of this message.
      \param format Formating string.
    */
    void operator()(int niv, const char* format, ...)
    {
	//if the niv level is inferior to VP_DEBUG_MODE
	if(VP_DEBUG_MODE >= niv)
	{
	    //gets the variable list of arguments
	    va_list args;
	    va_start(args, format);

	    std::cout << "(N" << niv << ") " ;
	    //calls display with it
	    display(format, args);

	    va_end(args);
	}
    }

    /*!
      Displays a string.
      \param format Formating string.
    */
    void operator()(const char* format, ...)
    {
	//gets the variable list of arguments
	va_list args;
	va_start(args, format);

#ifdef VP_DEBUG
	std::cout<<"(N0) ";
#endif

	//calls display with it
	display(format, args);

	va_end(args);
    }

    /*!

      Displays a message to either stdout/std::cout or stderr/std::cerr (based on error
      boolean).

      \param format Formating string.
      \param args List of arguments.

    */
    void display(const char* format, va_list args)
    {
	//if we want to write to std::cerr/stderr
	if(err)
	{
	    //first writes the header if there is one
	    if(header != NULL) std::cerr<<header;
	    //then writes the recorded namefile, function and line
	    std::cerr << "!!\t" << currentFile << ": " <<currentFunc << "(#" << currentLine << ") :" ;
	    //and finally writes the message passed to () operator.
	    vfprintf (stderr, format, args);
	    fprintf (stderr, "\n");
	    //flushes the buffer
	    fflush (stderr);
	}
	else
	{
	    //first writes the header if there is one
	    if(header != NULL) std::cout<<header;
	    //then writes the recorded namefile, function and line
	    std::cout <<currentFile << ": " << currentFunc << "(#" << currentLine << ") :" ;
	    //and finally writes the message passed to () operator.
	    vprintf (format, args);
	    printf ("\n");
	    //flushes the buffer
	    fflush (stdout);
	}
    }

};




/* -------------------------------------------------------------------------- */
/* --- vpTRACE -------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Used to display trace messages on the standard stream (C++).
  Use like this : vpCTRACE<<"my message"<<std::endl;
*/
#define vpCTRACE std::cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"


/*!
  Used to display error messages on the error stream (C++).
  Use like this : vpCERROR<<"my message"<<std::endl;
*/
#define vpCERROR std::cerr << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

/*!
  Used to display error messages on the error stream.
  Prints the name of the file, the function name and the line where
  it was used.
  Use like this : vpERRORTRACE("my error message number %d", i);
  with any "printf" string.
*/
#define vpERROR_TRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, true))

/*!
  Used to display trace messages on the standard stream.
  Prints the name of the file, the function name and the line where
  it was used.
  Use like this : vpTRACE("my debug message number %d", i);
  with any "printf" string.
*/
#define vpTRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, false))


/* -------------------------------------------------------------------------- */
/* --- VP_DEBUG ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#ifdef VP_DEBUG

/*!
  vpDERROR_TRACE works like printf, but prints only if the
  tracing level niv is greater than the debug level VP_DEBUG_MODE.
*/
#define vpDERROR_TRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, true))

/*!
  vpDEBUG_TRACE works like printf, but prints only if the
  tracing level niv is greater than the debug level VP_DEBUG_MODE.
*/
#define vpDEBUG_TRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, false))

/*!
  vpCDEBUG(niv) work like the C++ output stream std::cout.
*/
#define vpCDEBUG(niv) if (VP_DEBUG_MODE < niv) ; else \
		std::cout << "(N" << niv << ") "<<  __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

/*!

  vpDEBUG_ENABLE(niv) is equal to 1 if the debug level niv is greater than
  the debug mode VP_DEBUG_MODE, 0 else.
*/
#define vpDEBUG_ENABLE(niv) (VP_DEBUG_MODE >= niv)

#else

inline void vpDERROR_TRACE(int /* niv */, const char * /* a */, ...){};
inline void vpDEBUG_TRACE(int /* niv */, const char * /* a */, ...){};
#define vpCDEBUG(niv) if (1) ; else std::cout
#define vpDEBUG_ENABLE(niv) (0)

#endif



/* -------------------------------------------------------------------------- */
/* --- vpTRACE IN/OUT FONCTION ---------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Works like vpTRACE and should be used at the beginning of a function.
*/
#define vpIN_FCT (vpTraceOutput(__FILE__,__LINE__, __FUNCTION__, false, "begin "))


/*!
  Works like vpTRACE and should be used at the end of a function.
*/
#define vpOUT_FCT (vpTraceOutput(__FILE__,__LINE__, __FUNCTION__, false, "end "))


/* -------------------------------------------------------------------------- */
/* --- DEFENSIF ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#ifdef VP_DEFENSIF
#define DEFENSIF(a)  (a)
#else
#define DEFENSIF(a)  (0)
#endif  /*#ifdef DEFENSIF*/


#endif /* #ifdef __DEBUG_HH */

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
