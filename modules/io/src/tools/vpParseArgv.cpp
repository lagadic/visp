/****************************************************************************
 *
 * This file contains a procedure that handles table-based
 * argv-argc parsing.
 *
 * Copyright 1990 Regents of the University of California
 * Permission to use, copy, modify, and distribute this
 * software and its documentation for any purpose and without
 * fee is hereby granted, provided that the above copyright
 * notice appear in all copies.  The University of California
 * makes no representations about the suitability of this
 * software for any purpose.  It is provided "as is" without
 * express or implied warranty.
 *
 * This file has been modified to not rely on tcl, tk or X11.
 * Based on tkArgv.c from tk2.3 :
 *
 * Modifications by Peter Neelin (November 27, 1992)
 * Modifications by Fabien Spindler (June 20, 2006)
 */

/*!
  \file vpParseArgv.cpp
  \brief Command line argument parsing.
*/



#include <visp3/io/vpParseArgv.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

/*
 * Default table of argument descriptors.  These are normally available
 * in every application.
 */

vpParseArgv::vpArgvInfo vpParseArgv::defaultTable[2] = {
    {"-help", ARGV_HELP, (char *) NULL, (char *) NULL,
	"Print summary of command-line options and abort.\n"},
    {NULL, ARGV_END, (char *) NULL, (char *) NULL,
	(char *) NULL}
};

int (*handlerProc1)(const char *dst, const char *key, const char *argument);
int (*handlerProc2)(const char *dst, const char *key, int valargc, const char **argument);


/*!
  Process an argv array according to a table of expectedvcommand-line options. 
 
  The return value is a boolean value with true indicating an
  error. If an error occurs then an error message is printed on
  stderr. Under normal conditions, both *argcPtr and *argv are
  modified to return the arguments that couldn't be processed here
  (they didn't match the option table, or followed an
  vpParseArgv::ARGV_REST argument).
 
  \param argcPtr: Pointer to the count of command line arguments.

  \param argv: Array of command line argument strings.

  \param argTable: Array of command-specific argument descriptions.

  \param flags: This parameter is to set with vpParseArgv::vpArgvFlags
  values or combinations of these values using the OR operator
  (vpParseArgv::ARGV_NO_LEFTOVERS | vpParseArgv::ARGV_NO_DEFAULTS). If
  the vpParseArgv::ARGV_NO_DEFAULTS bit is set, then
  don't generate information for default options.
*/
bool
vpParseArgv::parse(int *argcPtr, const char **argv, vpArgvInfo *argTable, 
		   int flags)

{
   vpArgvInfo *infoPtr;	/* Pointer to the current entry in the
				 * table of argument descriptions. */
   vpArgvInfo *matchPtr;	        /* Descriptor that matches current argument. */
   const char *curArg;		/* Current argument */
   char c;		/* Second character of current arg (used for
				 * quick check for matching;  use 2nd char.
				 * because first char. will almost always
				 * be '-'). */
   int srcIndex;		/* Location from which to read next argument
				 * from argv. */
   int dstIndex;		/* Index into argv to which next unused
				 * argument should be copied (never greater
				 * than srcIndex). */
   int argc;			/* # arguments in argv still to process. */
   size_t length;			/* Number of characters in current argument. */
   unsigned long long nargs;                   /* Number of following arguments to get. */

/* Macro to optionally print errors */
#define FPRINTF if (!(flags&ARGV_NO_PRINT)) (void) fprintf

   if (flags & ARGV_DONT_SKIP_FIRST_ARG) {
      srcIndex = dstIndex = 0;
      argc = *argcPtr;
   } else {
      srcIndex = dstIndex = 1;
      argc = *argcPtr-1;
   }

   while (argc > 0) {
      curArg = argv[srcIndex];
      srcIndex++;
      argc--;
      c = curArg[1];
      length = strlen(curArg);

      /*
       * Loop throught the argument descriptors searching for one with
       * the matching key string.  If found, leave a pointer to it in
       * matchPtr.
       */

      matchPtr = NULL;
      for (unsigned int i = 0; i < 2; i++) {
         if (i == 0) {
            infoPtr = argTable;
         } else {
            infoPtr = defaultTable;
         }
         for (; infoPtr->type != ARGV_END; infoPtr++) {
            if (infoPtr->key == NULL) {
               continue;
            }
            if ((infoPtr->key[1] != c)
                || (strncmp(infoPtr->key, curArg, length) != 0)) {
               continue;
            }
            if (infoPtr->key[length] == 0) {
               matchPtr = infoPtr;
               goto gotMatch;
            }
            if (flags & ARGV_NO_ABBREV) {
               continue;
            }
            if (matchPtr != NULL) {
               FPRINTF(stderr, "ambiguous option \"%s\"\n", curArg);
               return true;
            }
            matchPtr = infoPtr;
         }
      }
      if (matchPtr == NULL) {

         /*
          * Unrecognized argument.  Just copy it down, unless the caller
          * prefers an error to be registered.
          */

         if (flags & ARGV_NO_LEFTOVERS) {
            FPRINTF(stderr, "unrecognized argument \"%s\"\n", curArg);
         }
         argv[dstIndex] = curArg;
         dstIndex++;
         continue;
      }

      /*
       * Take the appropriate action based on the option type
       */
	gotMatch:
      infoPtr = matchPtr;
      switch (infoPtr->type) {
      case ARGV_CONSTANT:
        *((int *) infoPtr->dst) = 1;
        break;
      case ARGV_INT:
         nargs = (uintptr_t) infoPtr->src;
         if (nargs<1) nargs=1;
         for (unsigned long i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
               char *endPtr=NULL;

               *(((int *) infoPtr->dst)+i) =
                  (int)strtol(argv[srcIndex], &endPtr, 0);
               if ((endPtr == argv[srcIndex]) || (*endPtr != 0)) {
                  FPRINTF(stderr,
                  "expected integer argument for \"%s\" but got \"%s\"\n",
                          infoPtr->key, argv[srcIndex]);
                  return true;
               }
               srcIndex++;
               argc--;
            }
         }
         break;
      case ARGV_LONG:
         nargs = (uintptr_t) infoPtr->src;
         if (nargs<1) nargs=1;
         for (unsigned long i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
               char *endPtr=NULL;

               *(((long *) infoPtr->dst)+i) =
                  strtol(argv[srcIndex], &endPtr, 0);
               if ((endPtr == argv[srcIndex]) || (*endPtr != 0)) {
                  FPRINTF(stderr,
                  "expected long argument for \"%s\" but got \"%s\"\n",
                          infoPtr->key, argv[srcIndex]);
                  return true;
               }
               srcIndex++;
               argc--;
            }
         }
         break;
      case ARGV_STRING:
         nargs = (uintptr_t) infoPtr->src;
         if (nargs<1) nargs=1;
         for (unsigned long i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
               *(((const char **)infoPtr->dst)+i) = argv[srcIndex];
               srcIndex++;
               argc--;
            }
         }
         break;
      case ARGV_REST:
         *((int *) infoPtr->dst) = dstIndex;
         goto argsDone;
      case ARGV_FLOAT:
         nargs = (uintptr_t) infoPtr->src;
         if (nargs<1) nargs=1;
         for (unsigned long i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
	      char *endPtr;

               *(((float *) infoPtr->dst)+i) =
                  (float)strtod(argv[srcIndex], &endPtr); // Here we use strtod
               if ((endPtr == argv[srcIndex]) || (*endPtr != 0)) {
                  FPRINTF(stderr,
       "expected floating-point argument for \"%s\" but got\"%s\"\n",
                          infoPtr->key, argv[srcIndex]);
                  return true;
               }
               srcIndex++;
               argc--;
            }
         }
         break;
      case ARGV_DOUBLE:
         nargs = (uintptr_t) infoPtr->src;
         if (nargs<1) nargs=1;
         for (unsigned long i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
	      char *endPtr;

               *(((double *) infoPtr->dst)+i) =
                  strtod(argv[srcIndex], &endPtr);
               if ((endPtr == argv[srcIndex]) || (*endPtr != 0)) {
                  FPRINTF(stderr,
       "expected double-point argument for \"%s\" but got\"%s\"\n",
                          infoPtr->key, argv[srcIndex]);
                  return true;
               }
               srcIndex++;
               argc--;
            }
         }
         break;

      case ARGV_FUNC: {
         handlerProc1 = (int (*)(const char *dst, const char *key, const char *argument))infoPtr->src;

         if ((*handlerProc1)(infoPtr->dst, infoPtr->key, argv[srcIndex]))
	 {
            srcIndex += 1;
            argc -= 1;
         }
         break;
      }
      case ARGV_GENFUNC: {
         handlerProc2 = (int (*)(const char *dst, const char *key, int valargc, const char **argument))infoPtr->src;

         argc = (*handlerProc2)(infoPtr->dst, infoPtr->key, argc, argv+srcIndex);
         if (argc < 0) {
            return true;
         }
         break;
      }

      case ARGV_HELP:
         printUsage (argTable, flags);
         return true;
      case ARGV_END:
      default:
         FPRINTF(stderr, "bad argument type %d in vpArgvInfo",
                 infoPtr->type);
         return true;
      }
   }

   /*
    * If we broke out of the loop because of an OPT_REST argument,
    * copy the remaining arguments down.
    */

 argsDone:
   while (argc) {
      argv[dstIndex] = argv[srcIndex];
      srcIndex++;
      dstIndex++;
      argc--;
   }
   argv[dstIndex] = (char *) NULL;
   *argcPtr = dstIndex;
   return false;

 missingArg:
   FPRINTF(stderr, "\"%s\" option requires an additional argument\n", curArg);
   return true;
	
#undef FPRINTF
}

/*!
  Generate a help string describing command-line options.
 
  Prints on stderr (unless vpParseArgv::ARGV_NO_PRINT is specified in
  flags) a help string describing all the options in argTable, plus
  all those in the default table unless vpParseArgv::ARGV_NO_DEFAULTS
  is specified in flags.
 
  \param argTable: Array of command-specific argument.descriptions.

  \param flags: If the vpParseArgv::ARGV_NO_DEFAULTS bit is set in
  this word, then don't generate information for default options.

*/
void
vpParseArgv::printUsage(vpArgvInfo * argTable, int flags)
{
   vpArgvInfo *infoPtr;
   int width;
   int numSpaces;
#define NUM_SPACES 20
   static char spaces[] = "                    ";
/*   char tmp[30]; */
   unsigned long long nargs;

/* Macro to optionally print errors */
#define FPRINTF if (!(flags&ARGV_NO_PRINT)) (void) fprintf

   /*
    * First, compute the width of the widest option key, so that we
    * can make everything line up.
    */

   width = 4;
   for (unsigned int i = 0; i < 2; i++) {
      for (infoPtr = i ? defaultTable : argTable;
           infoPtr->type != ARGV_END; infoPtr++) {
         int length;
         if (infoPtr->key == NULL) {
            continue;
         }
         length = (int)strlen(infoPtr->key);
         if (length > width) {
            width = length;
         }
      }
   }

   FPRINTF(stderr, "Command-specific options:");
   for (unsigned int i = 0; ; i++) {
      for (infoPtr = i ? defaultTable : argTable;
           infoPtr->type != ARGV_END; infoPtr++) {
         if ((infoPtr->type == ARGV_HELP) && (infoPtr->key == NULL)) {
            FPRINTF(stderr, "\n%s", infoPtr->help);
            continue;
         }
         FPRINTF(stderr, "\n %s:", infoPtr->key);
         numSpaces = width + 1 - (int)strlen(infoPtr->key);
         while (numSpaces > 0) {
            if (numSpaces >= NUM_SPACES) {
               FPRINTF(stderr, "%s",spaces);
            } else {
               FPRINTF(stderr, "%s",spaces+NUM_SPACES-numSpaces);
            }
            numSpaces -= NUM_SPACES;
         }
         FPRINTF(stderr, "%s",infoPtr->help);
         switch (infoPtr->type) {
         case ARGV_INT: {
            FPRINTF(stderr, "\n\t\tDefault value:");
            nargs = (uintptr_t) infoPtr->src;
            if (nargs<1) nargs=1;
            for (unsigned long j=0; j<nargs; j++) {
               FPRINTF(stderr, " %d", *(((int *) infoPtr->dst)+j));
            }
            break;
         }
         case ARGV_LONG: {
            FPRINTF(stderr, "\n\t\tDefault value:");
            nargs = (uintptr_t) infoPtr->src;
            if (nargs<1) nargs=1;
            for (unsigned long j=0; j<nargs; j++) {
               FPRINTF(stderr, " %ld", *(((long *) infoPtr->dst)+j));
            }
            break;
         }
         case ARGV_FLOAT: {
            FPRINTF(stderr, "\n\t\tDefault value:");
            nargs = (uintptr_t) infoPtr->src;
            if (nargs<1) nargs=1;
            for (unsigned long j=0; j<nargs; j++) {
               FPRINTF(stderr, " %f", *(((float *) infoPtr->dst)+j));
            }
            break;
         }
         case ARGV_DOUBLE: {
            FPRINTF(stderr, "\n\t\tDefault value:");
            nargs = (uintptr_t) infoPtr->src;
            if (nargs<1) nargs=1;
            for (unsigned long j=0; j<nargs; j++) {
               FPRINTF(stderr, " %g", *(((double *) infoPtr->dst)+j));
            }
            break;
         }
         case ARGV_STRING: {
            const char *string;

            nargs = (uintptr_t) infoPtr->src;
            if (nargs<1) nargs=1;
            string = *((const char **) infoPtr->dst);
            if ((nargs==1) && (string == NULL)) break;
            for (unsigned long j=0; j<nargs; j++) {
               string = *(((const char **) infoPtr->dst)+j);
               if (string != NULL) {
                  FPRINTF(stderr, " \"%s\"", string);
               }
            }

            break;
         }
         case ARGV_END:
         case ARGV_HELP:
         case ARGV_GENFUNC:
         case ARGV_FUNC:
         case ARGV_REST:
         case ARGV_CONSTANT:
         default: {
            break;
         }
         }
      }

      if ((flags & ARGV_NO_DEFAULTS) || (i > 0)) {
         break;
      }
      FPRINTF(stderr, "\nGeneric options for all commands:");
   }

   FPRINTF(stderr, "\n");
#undef FPRINTF
}

/*!

  Get next command line option and parameter.

  \param argc: Count of command line arguments.

  \param argv: Array of command line argument strings.

  \param validOpts: String of valid case-sensitive option characters, a
  ':' following a given character means that option can take a parameter.

  \param param: Pointer to a pointer to a string for output.

  \return If valid option is found, the character value of that option is
  returned, and *param points to the parameter if given, or is NULL if no
  param.

  \return If standalone parameter (with no option) is found, 1 is returned, and
  *param points to the standalone parameter

  \return If option is found, but it is not in the list of valid options, -1 is
  returned, and *param points to the invalid argument.

  \return When end of argument list is reached, 0 is returned, and *param
  is NULL.

*/
int 
vpParseArgv::parse(int argc, const char** argv, const char* validOpts, 
		   const char** param)
{
  static int iArg = 1;
  int chOpt;
  const char* psz = NULL;
  const char* pszParam = NULL;

  if (iArg < argc) {
    psz = &(argv[iArg][0]);
    if (*psz == '-') { // || *psz == '/')  {
      // we have an option specifier
      chOpt = argv[iArg][1];
      if (isalnum(chOpt) || ispunct(chOpt)) {
	// we have an option character
	psz = strchr(validOpts, chOpt);
	if (psz != NULL) {
	  // option is valid, we want to return chOpt
	  if (psz[1] == ':') {
	    // option can have a parameter
	    psz = &(argv[iArg][2]);
	    if (*psz == '\0') {
	      // must look at next argv for param
	      if (iArg+1 < argc) {
		psz = &(argv[iArg+1][0]);
		// next argv is the param
		iArg++;
		pszParam = psz;
	      }
	      else {
		// reached end of args looking for param
		// option specified without parameter
		chOpt = -1;
		pszParam = &(argv[iArg][0]);
	      }

	    }
	    else {
	      // param is attached to option
	      pszParam = psz;
	    }
	  }
	  else {
	    // option is alone, has no parameter
	  }
	}
	else {
	  // option specified is not in list of valid options
	  chOpt = -1;
	  pszParam = &(argv[iArg][0]);
	}
      }
      else {
	// though option specifier was given, option character
	// is not alpha or was was not specified
	chOpt = -1;
	pszParam = &(argv[iArg][0]);
      }
    }
    else {
      // standalone arg given with no option specifier
      chOpt = 1;
      pszParam = &(argv[iArg][0]);
    }
  }
  else {
    // end of argument list
    chOpt = 0;
  }

  iArg++;
  *param = pszParam;
  return (chOpt);
}
