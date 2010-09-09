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
 * Debug and trace macro.
 *
 *   - TRACING:    vpTRACE and vpERROR_TRACE work like printf with carreer return at the end of the string.
 *                 vpCERROR et vpCTRACE work like the C++ output streams std::cout and std::cerr.
 *   - DEBUGING:   vpDEBUG_TRACE(niv) and vpDERROR_TRACE(niv), work like printf, but print only if the
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

  \ingroup Debug

  \brief This class is used to display debug or error messages.

  It needs to be initialized with the file name, function name and
  line, of the place where it is created.  It is best used by first
  instanciating the object and directly calling the () operator.  This
  is used to mimic variadic macros.

  This class is used to define the following macros:

  - Macros for tracing: vpTRACE() and vpERROR_TRACE() work like printf
    with carreer return at the end of the string.  vpCERROR() et
    vpCTRACE() work like the C++ output streams std::cout and
    std::cerr.

  - Macros for debuging: vpDEBUG_TRACE(niv) and vpDERROR_TRACE(niv)
    work like printf, but print only if the tracing level \e niv is
    greater than the debug level VP_DEBUG_MODE macro. vpCDEBUG(niv)
    work like the C++ output stream std::cout. vpDEBUG_ENABLE(niv) is
    equal to 1 if the debug level niv is greater than the debug mode
    VP_DEBUG_MODE, 0 else.

  The example below shows how to use these macros.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  vpIN_FCT("main()");

  // Check the active debug levels
  std::cout << "Debug level 1 active: " << vpDEBUG_ENABLE(1) << std::endl;
  std::cout << "Debug level 2 active: " << vpDEBUG_ENABLE(2) << std::endl;
  std::cout << "Debug level 3 active: " << vpDEBUG_ENABLE(3) << std::endl;

  // C-like debug printings
  vpTRACE("C-like trace"); // stdout

  // Printing depend only VP_DEBUG_MODE value is >= 1
  vpTRACE(1, "C-like trace level 1");              // stdout
  vpERROR_TRACE(1, "C-like error trace level 1");  // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpDEBUG_TRACE(2, "C-like debug trace level 2");  // stdout
  vpDERROR_TRACE(2, "C-like error trace level 2"); // stderr

  // C++-like debug printings
  vpCTRACE << "C++-like debug trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout

  vpOUT_FCT("main()");
}
  \endcode

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
	    
	    if (err)
		std::cerr << "(N" << niv << ") " ;
	    else
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

      Displays a message to either stdout/std::cout or
      stderr/std::cerr (based on error boolean).

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


/* ------------------------------------------------------------------------- */
/* --- vpTRACE IN/OUT FONCTION --------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*!
  \ingroup Debug
  Works like vpTRACE() and should be used at the beginning of a function.

  \code
#include <visp/vpDebug.h>

int main()
{
  vpIN_FCT("main()");
  // the body of the main() function
  vpOUT_FCT("main()");
}
  \endcode

  \sa vpOUT_FCT 
*/
#define vpIN_FCT (vpTraceOutput(__FILE__,__LINE__, __FUNCTION__, false, "begin "))


/*!
  \ingroup Debug
  Works like vpTRACE() and should be used at the end of a function.

  \code
#include <visp/vpDebug.h>

int main()
{
  vpIN_FCT("main()");
  // the body of the main() function
  vpOUT_FCT("main()");
}
  \endcode

  \sa vpIN_FCT 
*/
#define vpOUT_FCT (vpTraceOutput(__FILE__,__LINE__, __FUNCTION__, false, "end "))



/* -------------------------------------------------------------------------- */
/* --- vpTRACE -------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  \ingroup Debug
  Used to display trace messages on the standard stream (C++).
  Use like this : vpCTRACE<<"my message"<<std::endl;

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // C++-like debug printings
  vpCTRACE << "C++-like debug trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout
}
  \endcode

  \sa vpTRACE(), vpCERROR(), vpCDEBUG()
*/
#define vpCTRACE std::cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"


/*!
  \ingroup Debug
  Used to display error messages on the error stream (C++).
  Use like this : vpCERROR<<"my message"<<std::endl;

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // C++-like debug printings
  vpCTRACE << "C++-like debug trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout
}
  \endcode

  \sa vpCTRACE(), vpCDEBUG()
*/
#define vpCERROR std::cerr << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

/*!
  \ingroup Debug
  Used to display error messages on the error stream.
  Prints the name of the file, the function name and the line where
  it was used.
  Use like this : vpERROR_TRACE("my error message number %d", i);
  with any "printf" string.

  \code
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // Printing depend only VP_DEBUG_MODE value is >= 1
  vpTRACE(1, "C-like trace level 1");              // stdout
  vpERROR_TRACE(1, "C-like error trace level 1");  // stderr
}
  \endcode

  \sa vpTRACE()
*/
#define vpERROR_TRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, true))

/*!
  \ingroup Debug
  Used to display trace messages on the standard stream.
  Prints the name of the file, the function name and the line where
  it was used.
  Use like this : vpTRACE("my debug message number %d", i);
  with any "printf" string.

  \code
#include <visp/vpDebug.h>

int main()
{
  // C-like debug printings
  vpTRACE("C-like trace"); // stdout
}
  \endcode

  \sa vpCTRACE(), vpERROR_TRACE()
*/
#define vpTRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, false))


/* ------------------------------------------------------------------------- */
/* --- VP_DEBUG ------------------------------------------------------------ */
/* ------------------------------------------------------------------------- */

#ifdef VP_DEBUG

/*!
  \ingroup Debug
  vpDERROR_TRACE works like printf, but prints only if the
  tracing level niv is greater than the debug level VP_DEBUG_MODE.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpDEBUG_TRACE(2, "C-like debug trace level 2");  // stdout
  vpDERROR_TRACE(2, "C-like error trace level 2"); // stderr
}
  \endcode

  \sa vpDEBUG_TRACE()
*/
#define vpDERROR_TRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, true))

/*!
  \ingroup Debug
  vpDEBUG_TRACE works like printf, but prints only if the
  tracing level niv is greater than the debug level VP_DEBUG_MODE.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpDEBUG_TRACE(2, "C-like debug trace level 2");  // stdout
  vpDERROR_TRACE(2, "C-like error trace level 2"); // stderr
}
  \endcode

  \sa vpDERROR_TRACE()
*/
#define vpDEBUG_TRACE (vpTraceOutput( __FILE__,__LINE__, __FUNCTION__, false))

/*!
  \ingroup Debug
  vpCDEBUG(niv) work like the C++ output stream std::cout.
  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // C++-like debug printings
  vpCTRACE << "C++-like debug trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout
}
  \endcode

  \sa vpCTRACE(), vpCERROR()
*/
#define vpCDEBUG(niv) if (VP_DEBUG_MODE < niv) ; else \
		std::cout << "(N" << niv << ") "<<  __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

/*!
  \ingroup Debug

  vpDEBUG_ENABLE(niv) is equal to 1 if the debug level \e niv is greater than
  the debug mode VP_DEBUG_MODE, 0 else.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp/vpDebug.h>

int main()
{
  // Check the active debug levels
  std::cout << "Debug level 1 active: " << vpDEBUG_ENABLE(1) << std::endl;
  std::cout << "Debug level 2 active: " << vpDEBUG_ENABLE(2) << std::endl;
  std::cout << "Debug level 3 active: " << vpDEBUG_ENABLE(3) << std::endl;
}  
  \endcode
*/
#define vpDEBUG_ENABLE(niv) (VP_DEBUG_MODE >= niv)

#else

inline void vpDERROR_TRACE(int /* niv */, const char * /* a */, ...){};
inline void vpDEBUG_TRACE(int /* niv */, const char * /* a */, ...){};
#define vpCDEBUG(niv) if (1) ; else std::cout
#define vpDEBUG_ENABLE(niv) (0)

#endif

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
