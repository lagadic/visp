/****************************************************************************
 *
 * $Id: vpParseArgv.h,v 1.5 2007-05-02 16:41:43 fspindle Exp $
 *
 * Declarations for Tk-related things that are visible
 * outside of the Tk module itself.
 *
 * Copyright 1989-1992 Regents of the University of California.
 * Permission to use, copy, modify, and distribute this
 * software and its documentation for any purpose and without
 * fee is hereby granted, provided that the above copyright
 * notice appear in all copies.  The University of California
 * makes no representations about the suitability of this
 * software for any purpose.  It is provided "as is" without
 * express or implied warranty.
 *
 * This file has been modified to be used only for argv parsing without
 * reference to tk, tcl or X11. Base on tk.h from tk2.3
 *
 * Description:
 * Command line argument parsing.
 *
 * Authors:
 * Fabien Spindler (modification of the original version)
 *
 *****************************************************************************/

/*!
  \file vpParseArgv.h
  \brief Command line argument parsing.
*/

#ifndef vpParseArgv_h
#define vpParseArgv_h


#include <visp/vpConfig.h>

/*!
  \class vpParseArgv
  \brief Command line argument parsing.

*/

/*!
  ArgvType
  Legal values for the type field of a ArgvInfo: see the user
  documentation for details.
 */
typedef enum  {
  ARGV_CONSTANT,
  ARGV_INT,
  ARGV_STRING,
  ARGV_REST,
  ARGV_FLOAT,
  ARGV_DOUBLE,
  ARGV_FUNC,
  ARGV_GENFUNC,
  ARGV_HELP,
  ARGV_END
} vpArgvType;

/*!

  Structure used to specify how to handle argv options.
*/
typedef struct {
    char *key;		/*!< The key string that flags the option in the
			 * argv array. */
    vpArgvType type;	/*!< Indicates option type;  see below. */
    char *src;		/*!< Value to be used in setting dst;  usage
			 * depends on type. */
    char *dst;		/*!< Address of value to be modified;  usage
			 * depends on type. */
    char *help;		/*!< Documentation message describing this option. */
} vpArgvInfo;


class VISP_EXPORT vpParseArgv
{

 public:
  static vpArgvInfo defaultTable[2];
  static bool parse(int *argcPtr, char **argv,
		    vpArgvInfo *argTable, int flags);
  static int  parse(int argc, char** argv, char* validOpts, char** param);

 private:
  static void printUsage (vpArgvInfo *argTable, int flags);


  /* \enum ArgvFlags
    Flag bits for passing to vpParseArgv:
   */
  enum ArgvFlags {
    ARGV_NO_DEFAULTS		= 0x1,
    ARGV_NO_LEFTOVERS		= 0x2,
    ARGV_NO_ABBREV		= 0x4,
    ARGV_DONT_SKIP_FIRST_ARG	= 0x8,
    ARGV_NO_PRINT		= 0x10
  };

} ;


#endif
