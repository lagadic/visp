/****************************************************************************
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

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

/*!
  \class vpParseArgv
  \ingroup module_io_cmd_parser
  \brief Command line argument parsing.

  The code below shows a first way to parse command line arguments
  using vpParseArgv class. It allows to specify an option
  name with more than one character.

  \code
#include <stdio.h>
#include <visp3/core/vpMath.h>
#include <visp3/io/vpParseArgv.h>

// Usage : [-bool] [-int <integer value>] [-long <long value>]
//         [-float <float value>] [-double <double value>] [-string <string value>] [-h]
int main(int argc, const char ** argv)
{
  // Variables to set by command line parsing
  bool   b_val = false;
  int    i_val = 10;
  long   l_val = 123456;
  float  f_val = 0.1f;
  double d_val = M_PI;
  char   *s_val;

  // Parse the command line to set the variables
  vpParseArgv::vpArgvInfo argTable[] =
  {
    {"-bool", vpParseArgv::ARGV_CONSTANT_BOOL, 0, (char *) &b_val,
     "Flag enabled."},
    {"-int", vpParseArgv::ARGV_INT, (char*) NULL, (char *) &i_val,
     "An integer value."},
    {"-long", vpParseArgv::ARGV_LONG, (char*) NULL, (char *) &l_val,
     "An integer value."},
    {"-float", vpParseArgv::ARGV_FLOAT, (char*) NULL, (char *) &f_val,
     "A float value."},
    {"-double", vpParseArgv::ARGV_DOUBLE, (char*) NULL, (char *) &d_val,
     "A double value."},
    {"-string", vpParseArgv::ARGV_STRING, (char*) NULL, (char *) &s_val,
     "A string value."},
    {"-h", vpParseArgv::ARGV_HELP, (char*) NULL, (char *) NULL,
     "Print the help."},
    {(char*) NULL, vpParseArgv::ARGV_END, (char*) NULL, (char*) NULL, (char*) NULL} } ;

  // Read the command line options
  if(vpParseArgv::parse(&argc, argv, argTable,
                        vpParseArgv::ARGV_NO_LEFTOVERS |
                        vpParseArgv::ARGV_NO_ABBREV |
                        vpParseArgv::ARGV_NO_DEFAULTS)) {
    return (false);
  }

  // b_val, i_val, l_val, f_val, d_val, s_val may have new values
}
  \endcode

  The code below shows an other way to parse command line arguments using
  vpParseArgv class. Here command line options are only one character long.
  \code
#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpMath.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"bi:l:f:d:h" // double point mean here that the preceding option request an argument

// Usage : [-b] [-i <integer value>] [-l <long value>]
//         [-f <float value>] [-d <double value>] [-s <string value>] [-h]
int main(int argc, const char ** argv)
{
  // Variables to set by command line parsing
  bool   b_val = false;
  int    i_val = 10;
  long   l_val = 123456;
  float  f_val = 0.1f;
  double d_val = M_PI;
  char   *s_val;

  // Parse the command line to set the variables
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'b': b_val = true; break;
    case 'i': i_val = atoi(optarg); break;
    case 'l': l_val = atol(optarg); break;
    case 'f': f_val = (float) atof(optarg); break;
    case 'd': d_val = atof(optarg); break;
    case 's': sprintf(s_val, "%s", optarg); break;
    case 'h': printf("Usage: ...\n"); return true; break;

    default:
      printf("Usage: ...\n"); return true; break;
    }
  }
  if ((c == 1) || (c == -1)) {
    // standalone param or error
    printf("Usage: ...\n");
    return false;
  }

  // b_val, i_val, l_val, f_val, d_val, s_val may have new values
}
  \endcode


*/

class VISP_EXPORT vpParseArgv
{
public:
  /*!
    Legal values for the type field of a vpArgvInfo.
  */
  typedef enum {
    ARGV_CONSTANT,      /*!< Stand alone argument. Same as ARGV_CONSTANT_INT. */
    ARGV_CONSTANT_INT,  /*!< Stand alone argument associated to an int var that
                           is set to 1. */
    ARGV_CONSTANT_BOOL, /*!< Stand alone argument associated to a bool var
                           that is set to true. */
    ARGV_INT,           /*!< Argument is associated to an int. */
    ARGV_LONG,          /*!< Argument is associated to a long. */
    ARGV_STRING,        /*!< Argument is associated to a char * string. */
    ARGV_REST,
    ARGV_FLOAT,  /*!< Argument is associated to a float. */
    ARGV_DOUBLE, /*!< Argument is associated to a double. */
    ARGV_FUNC,
    ARGV_GENFUNC,
    ARGV_HELP, /*!< Argument is for help displaying. */
    ARGV_END   /*!< End of the argument list. */
  } vpArgvType;

  /*!
    Flag bits.
   */
  typedef enum {
    ARGV_NO_DEFAULTS = 0x1,         /*!< No default options like -help. */
    ARGV_NO_LEFTOVERS = 0x2,        /*!< Print an error message if an option is not
                                       in the argument list. */
    ARGV_NO_ABBREV = 0x4,           /*!< No abrevation. Print an error message if an
                                       option is abrevated (ie "-i" in place of "-int"
                                       which is requested). */
    ARGV_DONT_SKIP_FIRST_ARG = 0x8, /*!< Don't skip first argument. */
    ARGV_NO_PRINT = 0x10            /*!< No printings. */
  } vpArgvFlags;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!

    Structure used to specify how to handle argv options.
  */
  typedef struct {
    const char *key;  /*!< The key string that flags the option in the
                       * argv array. */
    vpArgvType type;  /*!< Indicates option type;  see below. */
    const char *src;  /*!< Value to be used in setting dst;  usage
                       * depends on type. */
    const char *dst;  /*!< Address of value to be modified;  usage
                       * depends on type. */
    const char *help; /*!< Documentation message describing this option. */
  } vpArgvInfo;
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

public:
  static vpArgvInfo defaultTable[2];
  static bool parse(int *argcPtr, const char **argv, vpArgvInfo *argTable, int flags);
  static int parse(int argc, const char **argv, const char *validOpts, const char **param);

private:
  static void printUsage(vpArgvInfo *argTable, int flags);
};

#endif
