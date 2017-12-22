/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Debug and trace macro.
 *
 *   - TRACING:    vpTRACE and vpERROR_TRACE work like printf with carreer
 *return at the end of the string. vpCERROR et vpCTRACE work like the C++
 *output streams std::cout and std::cerr.
 *   - DEBUGING:   vpDEBUG_TRACE(niv) and vpDERROR_TRACE(niv), work like
 *printf, but print only if the tracing level niv is greater than the debug
 *level VP_DEBUG_MODE. vpCDEBUG(niv) work like the C++ output
 *stream std::cout. vpDEBUG_ENABLE(niv) is equal to 1 if the
 *debug level niv is greater than the debug mode
 *                 VP_DEBUG_MODE, 0 else.
 *   - PROG DEFENSIVE: DEFENSIF(a) is equal to a if defensive mode is active,
 *0 else.
 *
 * Authors:
 * Nicolas Mansard, Bruno Renier
 *
 *****************************************************************************/

#ifndef __VP_DEBUG_HH
#define __VP_DEBUG_HH

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <visp3/core/vpConfig.h>

#if defined(_WIN32)
#ifndef __FUNCTION__
#define __FUNCTION__ " "
#endif
#endif

#ifndef VP_DEBUG_MODE
#define VP_DEBUG_MODE 0
#endif

/*!
  \class vpTraceOutput

  \ingroup group_core_debug

  \brief This class is used to display debug or error messages.

  It needs to be initialized with the file name, function name and
  line, of the place where it is created.  It is best used by first
  instanciating the object and directly calling the () operator.  This
  is used to mimic variadic macros.

  This class is used to define the following macros:

  - Macros for tracing: vpTRACE(), vpERROR_TRACE(), vpIN_FCT() and
    vpOUT_FCT() work like printf
    with carreer return at the end of the string, while vpCTRACE() and
    vpCERROR() work like the C++ output streams std::cout and
    std::cerr.

  - Macros for debuging: vpDEBUG_TRACE(level) and vpDERROR_TRACE(level)
    work like printf, but print only if the tracing level \e level is
    greater than the debug level VP_DEBUG_MODE macro. vpCDEBUG(level)
    work like the C++ output stream std::cout. vpDEBUG_ENABLE(level) is
    equal to 1 if the debug level level is greater than the debug mode
    VP_DEBUG_MODE, 0 else.

  The example below shows how to use these macros.

  \code
#define VP_TRACE        // Activate the trace mode
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

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
  vpCTRACE << "C++-like trace" << std::endl; // stdout
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
  const char *currentFile; // Name of the file to use in the displays
  const char *currentFunc; // Name of the function to use in the displays
  int currentLine;         // Line to use in the displays

  // if true, output to std::cerr/stderr else std::cout/stdout
  bool err;
  // string to display before anything else
  const char *header;

public:
  /*!
    Constructor.
    \param file Should be the name of the file where this constructor was
    called. \param line Should be the line in file where this constructor was
    called. \param func Should be the name of the function where this
    constructor was called. \param error If true, writes to the error stream.
    \param s String to print before any other message (acts like a header).
    \note Call the constructor with something like
    vpTraceOutput(__FILE__,__LINE__, __FUNCTION__).
  */
  vpTraceOutput(const char *file, int line, const char *func, bool error = false, const char *s = NULL)
    : currentFile(file), currentFunc(func), currentLine(line), err(error), header(s)
  {
  }

  /*!
    Displays a string if the debug level is inferior to VP_DEBUG_MODE.
    \param level Level of this message.
    \param format Formating string.
  */
  void operator()(int level, const char *format, ...)
  {
    // if the level is inferior to VP_DEBUG_MODE
    if (VP_DEBUG_MODE >= level) {
      // gets the variable list of arguments
      va_list args;
      va_start(args, format);

      if (err)
        std::cerr << "(L" << level << ") ";
      else
        std::cout << "(L" << level << ") ";

      // calls display with it
      display(format, args);

      va_end(args);
    }
  }

  /*!
    Displays a string.
    \param format Formating string.
  */
  void operator()(const char *format, ...)
  {
    // gets the variable list of arguments
    va_list args;
    va_start(args, format);

#ifdef VP_DEBUG
    std::cout << "(L0) ";
#endif

    // calls display with it
    display(format, args);

    va_end(args);
  }

  /*!

    Displays a message to either stdout or
    stderr (based on error boolean).

    \param format Formating string.
    \param args List of arguments.

  */
  void display(const char *format, va_list args)
  {
    // if we want to write to std::cerr/stderr
    if (err) {
      // first writes the header if there is one
      if (header != NULL)
        std::cerr << header;
      // then writes the recorded namefile, function and line
      std::cerr << "!!\t" << currentFile << ": " << currentFunc << "(#" << currentLine << ") : ";
      // and finally writes the message passed to () operator.
      vfprintf(stderr, format, args);
      fprintf(stderr, "\n");
      // flushes the buffer
      fflush(stderr);
    } else {
      // first writes the header if there is one
      if (header != NULL)
        std::cout << header;
      // then writes the recorded namefile, function and line
      std::cout << currentFile << ": " << currentFunc << "(#" << currentLine << ") : ";
      // and finally writes the message passed to () operator.
      vprintf(format, args);
      printf("\n");
      // flushes the buffer
      fflush(stdout);
    }
  }
};

/* -------------------------------------------------------------------------
 */
/* --- vpTRACE IN/OUT FONCTION ---------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

#ifdef VP_TRACE // Activate the trace mode

/*!
  \ingroup group_core_debug
  Works like vpTRACE() and should be used at the beginning of a function.

  \code
#define VP_TRACE // To activate the trace mode
#include <visp3/core/vpDebug.h>

int main()
{
  vpIN_FCT("main()");
  // the body of the main() function
  vpOUT_FCT("main()");
}
  \endcode

  \sa vpOUT_FCT
*/
#define vpIN_FCT (vpTraceOutput(__FILE__, __LINE__, __FUNCTION__, false, "begin "))

/*!
  \ingroup group_core_debug
  Works like vpTRACE() and should be used at the end of a function.

  \code
#define VP_TRACE // To activate the trace mode
#include <visp3/core/vpDebug.h>

int main()
{
  vpIN_FCT("main()");
  // the body of the main() function
  vpOUT_FCT("main()");
}
  \endcode

  \sa vpIN_FCT
*/
#define vpOUT_FCT (vpTraceOutput(__FILE__, __LINE__, __FUNCTION__, false, "end "))

#else // #ifdef VP_TRACE

inline void vpIN_FCT(const char * /* a */, ...) {}
inline void vpOUT_FCT(const char * /* a */, ...) {}

#endif // #ifdef VP_TRACE

/* --------------------------------------------------------------------------
 */
/* --- vpTRACE --------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

#ifdef VP_TRACE

/*!
  \ingroup group_core_debug
  Used to display trace messages on the standard stream (C++).
  Use like this : vpCTRACE<<"my message"<<std::endl;

  \code
#define VP_TRACE        // To activate trace mode
#define VP_DEBUG        // To activate the debug mode
#define VP_DEBUG_MODE 2 // To activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // C++-like debug printings
  vpCTRACE << "C++-like trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout
}
  \endcode

  \sa vpTRACE(), vpCERROR(), vpCDEBUG()
*/
#define vpCTRACE std::cout << "(L0) " << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") : "

/*!
  \ingroup group_core_debug
  Used to display error messages on the error stream (C++).
  Use like this : vpCERROR<<"my message"<<std::endl;

  \code
#define VP_TRACE        // To activate trace mode
#define VP_DEBUG        // To activate the debug mode
#define VP_DEBUG_MODE 2 // To activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // C++-like debug printings
  vpCTRACE << "C++-like trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout
}
  \endcode

  \sa vpCTRACE(), vpCDEBUG()
*/
#define vpCERROR                                                                                                       \
  std::cerr << "(L0) "                                                                                                 \
            << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") : "

/*!
  \ingroup group_core_debug
  Used to display error messages on the error stream.
  Prints the name of the file, the function name and the line where
  it was used.
  Use like this : vpERROR_TRACE("my error message number %d", i);
  with any "printf" string.

  \code
#define VP_TRACE // To activate trace mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // Printing depend only VP_DEBUG_MODE value is >= 1
  vpTRACE(1, "C-like trace level 1");              // stdout
  vpERROR_TRACE(1, "C-like error trace level 1");  // stderr
}
  \endcode

  \sa vpTRACE()
*/
#define vpERROR_TRACE (vpTraceOutput(__FILE__, __LINE__, __FUNCTION__, true))

/*!
  \ingroup group_core_debug
  Used to display trace messages on the standard stream.
  Prints the name of the file, the function name and the line where
  it was used.
  Use like this : vpTRACE("my debug message number %d", i);
  with any "printf" string.

  \code
#define VP_TRACE // To activate trace mode
#include <visp3/core/vpDebug.h>

int main()
{
  // C-like debug printings
  vpTRACE("C-like trace"); // stdout
}
  \endcode

  \sa vpCTRACE(), vpERROR_TRACE()
*/
#define vpTRACE (vpTraceOutput(__FILE__, __LINE__, __FUNCTION__, false))

#else // #ifdef VP_TRACE

#define vpCTRACE                                                                                                       \
  if (false)                                                                                                           \
  std::cout // Warning C4127
#define vpCERROR                                                                                                       \
  if (false)                                                                                                           \
  std::cerr // Warning C4127

inline void vpERROR_TRACE(const char * /* a */, ...) {}
inline void vpERROR_TRACE(int /* level */, const char * /* a */, ...) {}
inline void vpTRACE(const char * /* a */, ...) {}
inline void vpTRACE(int /* level */, const char * /* a */, ...) {}

#endif // #ifdef VP_TRACE

/* -------------------------------------------------------------------------
 */
/* --- VP_DEBUG ------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

#ifdef VP_DEBUG

/*!
  \ingroup group_core_debug
  vpDERROR_TRACE works like printf, but prints only if the
  tracing level is smaller than the debug level VP_DEBUG_MODE.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpDEBUG_TRACE(2, "C-like debug trace level 2");  // stdout
  vpDERROR_TRACE(2, "C-like error trace level 2"); // stderr
}
  \endcode

  \sa vpDEBUG_TRACE()
*/
#define vpDERROR_TRACE (vpTraceOutput(__FILE__, __LINE__, __FUNCTION__, true))

/*!
  \ingroup group_core_debug
  vpDEBUG_TRACE works like printf, but prints only if the
  tracing level level is greater than the debug level VP_DEBUG_MODE.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpDEBUG_TRACE(2, "C-like debug trace level 2");  // stdout
  vpDERROR_TRACE(2, "C-like error trace level 2"); // stderr
}
  \endcode

  \sa vpDERROR_TRACE()
*/
#define vpDEBUG_TRACE (vpTraceOutput(__FILE__, __LINE__, __FUNCTION__, false))

/*!
  \ingroup group_core_debug
  vpCDEBUG(level) work like the C++ output stream std::cout.
  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // C++-like debug printings
  vpCTRACE << "C++-like trace" << std::endl; // stdout
  vpCERROR << "C++-like error trace" << std::endl; // stderr

  // Printing if VP_DEBUG defined and VP_DEBUG_MODE value >= 2
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // stdout
}
  \endcode

  \sa vpCTRACE(), vpCERROR()
*/
#define vpCDEBUG(level)                                                                                                \
  if (VP_DEBUG_MODE < level)                                                                                           \
    ;                                                                                                                  \
  else                                                                                                                 \
    std::cout << "(L" << level << ") " << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") : "

/*!
  \ingroup group_core_debug

  vpDEBUG_ENABLE(level) is equal to 1 if the debug level \e level is greater
than the debug mode VP_DEBUG_MODE, 0 else.

  \code
#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  // Check the active debug levels
  std::cout << "Debug level 1 active: " << vpDEBUG_ENABLE(1) << std::endl;
  std::cout << "Debug level 2 active: " << vpDEBUG_ENABLE(2) << std::endl;
  std::cout << "Debug level 3 active: " << vpDEBUG_ENABLE(3) << std::endl;
}
  \endcode
*/
#define vpDEBUG_ENABLE(level) (VP_DEBUG_MODE >= level)

#else // #ifdef VP_DEBUG

inline void vpDERROR_TRACE(const char * /* a */, ...) {}
inline void vpDEBUG_TRACE(const char * /* a */, ...) {}
inline void vpDERROR_TRACE(int /* level */, const char * /* a */, ...) {}
inline void vpDEBUG_TRACE(int /* level */, const char * /* a */, ...) {}

#define vpCDEBUG(level)                                                                                                \
  if (false)                                                                                                           \
  std::cout                           // Warning C4127
#define vpDEBUG_ENABLE(level) (false) // Warning C4127

#endif // #ifdef VP_DEBUG

/* --------------------------------------------------------------------------
 */
/* --- DEFENSIF -------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */
#ifdef VP_DEFENSIF
#define DEFENSIF(a) (a)
#else
#define DEFENSIF(a) (0)
#endif /*#ifdef DEFENSIF*/

#endif /* #ifdef __DEBUG_HH */
