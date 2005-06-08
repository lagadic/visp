/*
 * vpParseArgv.c --
 *
 *	This file contains a procedure that handles table-based
 *	argv-argc parsing.
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
 *
 * This file has been modified to not rely on tcl, tk or X11.
 * Based on tkArgv.c from tk2.3 :
static char rcsid[] = "$Header: /udd/fspindle/poub/cvs2svn/ViSP/cvsroot/visp/ViSP/src/tools/io/vpParseArgv.cpp,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $ SPRITE (Berkeley)";
 *
 * Modifications by Peter Neelin (November 27, 1992)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <visp/vpParseArgv.h>

#define TRUE  1
#define FALSE 0

/*
 * Default table of argument descriptors.  These are normally available
 * in every application.
 */

static vpArgvInfo defaultTable[] = {
    {"-help",	ARGV_HELP,	(char *) NULL,	(char *) NULL,
	"Print summary of command-line options and abort.\n"},
    {NULL,	ARGV_END,	(char *) NULL,	(char *) NULL,
	(char *) NULL}
};
#ifdef HP
int (*handlerProc1)();
int (*handlerProc2)();
#else
int (*handlerProc1)(char *dst, char *key, char *argument);
int (*handlerProc2)(char *dst, char *key, int valargc, char **argument);
#endif

/*
 * Forward declarations for procedures defined in this file:
 */

static void	PrintUsage _ANSI_ARGS_((vpArgvInfo *argTable, int flags));

/*
 *----------------------------------------------------------------------
 *
 * vpParseArgv --
 *
 *	Process an argv array according to a table of expected
 *	command-line options.  See the manual page for more details.
 *
 * Results:
 *	The return value is a Boolean value with TRUE indicating an error.
 *	If an error occurs then an error message is printed on stderr.
 *	Under normal conditions, both *argcPtr and *argv are modified
 *	to return the arguments that couldn't be processed here (they
 *	didn't match the option table, or followed an ARGV_REST
 *	argument).
 *
 * Side effects:
 *
 *----------------------------------------------------------------------
 */


#ifdef HP
int vpParseArgv (argcPtr, argv, argTable, flags)
    int *argcPtr;		 /*Number of arguments in argv.  Modified */
				 /*to hold # args left in argv at end.  */
    char **argv;		 /*Array of arguments.  Modified to hold */
				 /* those that couldn't be processed here. */
    vpArgvInfo *argTable;	 	 /*Array of option descriptions  */

    int flags;			 /*Or'ed combination of various flag bits, */
				 /* such as ARGV_NO_DEFAULTS. */
#else
int vpParseArgv(int *argcPtr, char **argv, vpArgvInfo *argTable, int flags)
#endif
{
   register vpArgvInfo *infoPtr;	/* Pointer to the current entry in the
				 * table of argument descriptions. */
   vpArgvInfo *matchPtr;	        /* Descriptor that matches current argument. */
   char *curArg;		/* Current argument */
   register char c;		/* Second character of current arg (used for
				 * quick check for matching;  use 2nd char.
				 * because first char. will almost always
				 * be '-'). */
   int srcIndex;		/* Location from which to read next argument
				 * from argv. */
   int dstIndex;		/* Index into argv to which next unused
				 * argument should be copied (never greater
				 * than srcIndex). */
   int argc;			/* # arguments in argv still to process. */
   int length;			/* Number of characters in current argument. */
   int nargs;                   /* Number of following arguments to get. */
   int i;

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
      for (i = 0; i < 2; i++) {
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
               return TRUE;
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
         *((int *) infoPtr->dst) = (int) infoPtr->src;
         break;
      case ARGV_INT:
         nargs = (int) infoPtr->src;
         if (nargs<1) nargs=1;
         for (i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
               char *endPtr;

               *(((int *) infoPtr->dst)+i) =
                  strtol(argv[srcIndex], &endPtr, 0);
               if ((endPtr == argv[srcIndex]) || (*endPtr != 0)) {
                  FPRINTF(stderr,
                  "expected integer argument for \"%s\" but got \"%s\"",
                          infoPtr->key, argv[srcIndex]);
                  return TRUE;
               }
               srcIndex++;
               argc--;
            }
         }
         break;
      case ARGV_STRING:
         nargs = (int) infoPtr->src;
         if (nargs<1) nargs=1;
         for (i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
               *(((char **)infoPtr->dst)+i) = argv[srcIndex];
               srcIndex++;
               argc--;
            }
         }
         break;
      case ARGV_REST:
         *((int *) infoPtr->dst) = dstIndex;
         goto argsDone;
      case ARGV_FLOAT:
         nargs = (int) infoPtr->src;
         if (nargs<1) nargs=1;
         for (i=0; i<nargs; i++) {
            if (argc == 0) {
               goto missingArg;
            } else {
               char *endPtr;

               *(((double *) infoPtr->dst)+i) =
                  strtod(argv[srcIndex], &endPtr);
               if ((endPtr == argv[srcIndex]) || (*endPtr != 0)) {
                  FPRINTF(stderr,
       "expected floating-point argument for \"%s\" but got\"%s\"\n",
                          infoPtr->key, argv[srcIndex]);
                  return TRUE;
               }
               srcIndex++;
               argc--;
            }
         }
         break;

      case ARGV_FUNC: {
#ifdef HP
        handlerProc1 = (int (*)())infoPtr->src;
#else
         handlerProc1 = (int (*)(char *dst, char *key, char *argument))infoPtr->src;
#endif
         if ((*handlerProc1)(infoPtr->dst, infoPtr->key, argv[srcIndex]))
	 {
            srcIndex += 1;
            argc -= 1;
         }
         break;
      }
      case ARGV_GENFUNC: {
#ifdef HP
        handlerProc2 = (int (*)())infoPtr->src;
#else
         handlerProc2 = (int (*)(char *dst, char *key, int valargc, char **argument))infoPtr->src;
#endif
         argc = (*handlerProc2)(infoPtr->dst, infoPtr->key, argc, argv+srcIndex);
         if (argc < 0) {
            return TRUE;
         }
         break;
      }

      case ARGV_HELP:
         PrintUsage (argTable, flags);
         return TRUE;
      default:
         FPRINTF(stderr, "bad argument type %d in vpArgvInfo",
                 infoPtr->type);
         return TRUE;
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
   return FALSE;

 missingArg:
   FPRINTF(stderr, "\"%s\" option requires an additional argument\n", curArg);
   return TRUE;
}

/*
 *----------------------------------------------------------------------
 *
 * PrintUsage --
 *
 *	Generate a help string describing command-line options.
 *
 * Results:
 *	Prints on stderr (unless ARGV_NO_PRINT is specified in flags)
 *	a help string describing all the options in argTable, plus all those
 *	in the default table unless ARGV_NO_DEFAULTS is
 *	specified in flags.
 *
 * Side effects:
 *	None.
 *
 *----------------------------------------------------------------------
 */
#ifdef HP
static void
PrintUsage(argTable, flags)

     vpArgvInfo *argTable;	/* Array of command-specific argument */
				/* descriptions. */
     int flags;			/* If the ARGV_NO_DEFAULTS bit is set */
				/* in this word, then don't generate */
				/*information for default options. */
#else
static void
PrintUsage(vpArgvInfo * argTable, int flags)
#endif
{
   register vpArgvInfo *infoPtr;
   int width, i, j, numSpaces;
#define NUM_SPACES 20
   static char spaces[] = "                    ";
/*   char tmp[30]; */
   int nargs;

/* Macro to optionally print errors */
#define FPRINTF if (!(flags&ARGV_NO_PRINT)) (void) fprintf

   /*
    * First, compute the width of the widest option key, so that we
    * can make everything line up.
    */

   width = 4;
   for (i = 0; i < 2; i++) {
      for (infoPtr = i ? defaultTable : argTable;
           infoPtr->type != ARGV_END; infoPtr++) {
         int length;
         if (infoPtr->key == NULL) {
            continue;
         }
         length = strlen(infoPtr->key);
         if (length > width) {
            width = length;
         }
      }
   }

   FPRINTF(stderr, "Command-specific options:");
   for (i = 0; ; i++) {
      for (infoPtr = i ? defaultTable : argTable;
           infoPtr->type != ARGV_END; infoPtr++) {
         if ((infoPtr->type == ARGV_HELP) && (infoPtr->key == NULL)) {
            FPRINTF(stderr, "\n%s", infoPtr->help);
            continue;
         }
         FPRINTF(stderr, "\n %s:", infoPtr->key);
         numSpaces = width + 1 - strlen(infoPtr->key);
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
            nargs = (int) infoPtr->src;
            if (nargs<1) nargs=1;
            for (j=0; j<nargs; j++) {
               FPRINTF(stderr, " %d", *(((int *) infoPtr->dst)+j));
            }
            break;
         }
         case ARGV_FLOAT: {
            FPRINTF(stderr, "\n\t\tDefault value:");
            nargs = (int) infoPtr->src;
            if (nargs<1) nargs=1;
            for (j=0; j<nargs; j++) {
               FPRINTF(stderr, " %g", *(((double *) infoPtr->dst)+j));
            }
            break;
         }
         case ARGV_STRING: {
            char *string;

            nargs = (int) infoPtr->src;
            if (nargs<1) nargs=1;
            string = *((char **) infoPtr->dst);
            if ((nargs==1) && (string == NULL)) break;
            for (j=0; j<nargs; j++) {
               string = *(((char **) infoPtr->dst)+j);
               if (string != NULL) {
                  FPRINTF(stderr, " \"%s\"", string);
               }
               else {
                  FPRINTF(stderr, " \"%s\"", string);
               }
            }

            break;
         }
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
}

