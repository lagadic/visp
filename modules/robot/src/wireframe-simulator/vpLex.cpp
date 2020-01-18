/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Le module "lex.c" contient les procedures de gestion
 * de l'analyse lexicale de l'analyseur lexicale "lex"
 * d'un fichier source dont la grammaire possede
 * les symboles terminaux suivants (ecrit en "LEX", UNIX) :
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include "vpKeyword.h"
#include "vpMy.h"
#include "vpToken.h"

#include <ctype.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS

static void count(void);
static void next_source(void);

void lexerr(const char *path, ...);

/* Codes des symboles terminaux	*/

#define NULT 0  /* caractere non valide	*/
#define EOBT 1  /* fin de buffer	*/
#define EOFT 2  /* fin de fichier	*/
#define EOLT 3  /* fin de ligne		*/
#define CMTT 4  /* commentaire		*/
#define IDNT 5  /* identificateur	*/
#define INTT 6  /* nombre entier	*/
#define FPTT 7  /* nombre flottant	*/
#define SGNT 8  /* signe +/-		*/
#define SPCT 9  /* caractere blanc	*/
#define STGT 10 /* caractere de chaine	*/
#define NBRT 11 /* nombre de codes	*/

/* Drapeaux des caracteres	*/

#define _NULT 0x00 /* caractere non valide	*/
#define _CMTT 0x01 /* commentaire		*/
#define _FPTT 0x02 /* nombre flottant	*/
#define _IDNT 0x04 /* identificateur	*/
#define _INTT 0x08 /* nombre entier	*/
#define _SGNT 0x10 /* signe +/-		*/
#define _STGT 0x20 /* caractere de chaine	*/

/* Caracteres sentinelles	*/

#define ASCII_NBR 128 /* nombre de codes ASCII*/

#ifndef EOB
#define EOB (-2) /* fin de buffer	*/
#endif
#ifndef EOF
#define EOF (-1) /* fin de fichier	*/
#endif
#ifndef EOL
#define EOL 10 /* fin de ligne		*/
#endif

#define CHAR_NBR 130 /* nombre de caracteres	*/

/* Tests des drapeaux		*/

#define isnult(c) (scantbl[c] == _NULT)
#define iscmtt(c) (scantbl[c] & _CMTT)
#define isfptt(c) (scantbl[c] & _FPTT)
#define isidnt(c) (scantbl[c] & _IDNT)
#define isintt(c) (scantbl[c] & _INTT)
#define issgnt(c) (scantbl[c] & _SGNT)
#define isstgt(c) (scantbl[c] & _STGT)

/*
 * Codes des messages d'erreur de l'analyseur lexicale.
 */
#define E_UNKNOWN 0
#define E_SYMBOL 1
#define E_CMT_EOF 2
#define E_FLOAT 3
#define E_INT 4
#define E_KEYWORD 5
#define E_STG_EOF 6
#define E_STG_EOL 7
#define E_STRING 8
#define E_9 9

const char *lex_errtbl[] = {/* table des messages d'erreur		*/
                            "error unknown",
                            "symbol undefined",
                            "unexpected EOF in comment",
                            "float expected",
                            "int expected",
                            "keyword expected",
                            "unexpected EOF in string or char constant",
                            "newline in string or char constant",
                            "string expected",
                            ""};

char *mytext = NULL;
int mylength = 0;
int mylineno = 1;
unsigned int mycolumno = 0;
float myfloat = 0.0;
int myint = 0;

static char *mysptr;   /* tete de lecture de la ligne courante	*/
static char *myline;   /* debut de la ligne courante		*/
static char *lastline; /* derniere ligne du buffer d'entree	*/

static Byte *chtbl;   /* premiers caracteres des terminaux	*/
static Byte *scantbl; /* caracteres suivants des terminaux	*/

/*
 * La procedure "open_lex" alloue et initialise les variables utilisees
 * par l'analyseur lexical "lex".
 */
void open_lex(void)
{
  if ((chtbl = (Byte *)malloc(CHAR_NBR * sizeof(Byte))) == NULL ||
      (scantbl = (Byte *)malloc(CHAR_NBR * sizeof(Byte))) == NULL) {
    static char proc_name[] = "open_lex";
    perror(proc_name);
    exit(1);
  }
  chtbl += 2; /* 2 sentinelles non affichables	*/
  scantbl += 2;

  /* initialise les premiers caracteres des symboles terminaux	*/

  for (int i = 0; i < ASCII_NBR; i++) {
    if (isalpha(i))
      chtbl[i] = IDNT;
    else if (isdigit(i))
      chtbl[i] = INTT;
    else if (isspace(i))
      chtbl[i] = SPCT;
    else
      switch (i) {
      case '"':
        chtbl[i] = STGT;
        break;
      case '+':
      case '-':
        chtbl[i] = SGNT;
        break;
      case '.':
        chtbl[i] = FPTT;
        break;
      case '/':
        chtbl[i] = CMTT;
        break;
      case '_':
        chtbl[i] = IDNT;
        break;
      default:
        chtbl[i] = NULT;
        break;
      }
  }

  /* Initialise les sentinelles comme des terminaux.		*/

  chtbl[EOB] = EOBT;
  chtbl[EOF] = EOFT;
  chtbl[EOL] = EOLT;

  /* Initialise les caracteres suivants des symboles terminaux.	*/

  for (int i = 0; i < ASCII_NBR; i++) {
    if (isalpha(i))
      scantbl[i] = _CMTT | _IDNT | _STGT;
    else if (isdigit(i))
      scantbl[i] = _CMTT | _IDNT | _INTT | _STGT;
    else
      switch (i) {
      case '"':
        scantbl[i] = _CMTT;
        break;
      case '+':
      case '-':
        scantbl[i] = _CMTT | _SGNT | _STGT;
        break;
      case '.':
        scantbl[i] = _CMTT | _FPTT | _STGT;
        break;
      case '/':
        scantbl[i] = _STGT;
        break;
      case '_':
        scantbl[i] = _CMTT | _IDNT | _STGT;
        break;
      default:
        scantbl[i] = _CMTT | _STGT;
        break;
      }
  }

  /* Initialise les sentinelles comme des terminaux.		*/

  scantbl[EOB] = _NULT;
  scantbl[EOF] = _NULT;
  scantbl[EOL] = _NULT;
}

/*
 * La procedure "close_lex" libere les variables utilisees
 * par l'analyseur lexical "lex".
 */
void close_lex(void)
{
  free((char *)(chtbl - 2)); /* voir "open_lex" pour "- 2"	*/
  free((char *)(scantbl - 2));
}

#define ECHO printf("%c", *(mysptr))
#define CURC (*((signed char *)mysptr))      /* caractere courant	*/
#define NEXTC (*((signed char *)mysptr + 1)) /* caractere suivant	*/
#define PREVC (*((signed char *)mysptr - 1)) /* caractere precedent	*/

/*
 * La procedure "lex" contient l'analyseur lexical.
 * Note :
 * La tete de lecture (mysptr) n'est pas systematiquement avancee apres
 *lecture. Le caractere courant est celui sous la tete de lecture. Ainsi on
 *accede de maniere symetrique aux caracteres precedent et suivant. Sortie :
 *		Code du symbole terminale analyse.
 */
int lex(void)
{
lex_loop:

  for (; chtbl[(int)CURC] == SPCT; mysptr++) {
  }; /* saute les espaces	*/

  switch (chtbl[(int)CURC]) {

  case NULT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    return (*mytext);
    break;
  case EOBT:
    next_source();
    goto lex_loop;
    break;
  case EOFT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    return (T_EOF);
    break;
  case EOLT:
    if (mysptr == lastline)
      next_source();
    else
      mysptr++;
    mylineno++;
    myline = mysptr;
    goto lex_loop;
    break;
  case CMTT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (CURC != '*')
      return (*mytext);
    mysptr++;
  comment:
    for (; iscmtt((int)CURC); mysptr++) {
    };
    switch (chtbl[(int)CURC]) {
    case EOBT:
      next_source();
      goto comment;
      break;
    case EOFT:
      lexerr("start", lex_errtbl[E_CMT_EOF], NULL);
      return (T_EOF);
      break;
    case EOLT:
      if (mysptr == lastline)
        next_source();
      else
        mysptr++;
      mylineno++;
      myline = mysptr;
      goto comment;
      break;
    case CMTT:
      if (PREVC == '*') { /* veritable fin	*/
        mysptr++;
        goto lex_loop;
      }
      mysptr++; /* pseudo fin 		*/
      goto comment;
      break;
    }
    break;
  case IDNT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    for (; isidnt((int)CURC); mysptr++) {
    };
    mylength = (int)(mysptr - mytext);
    return (get_symbol(mytext, mylength));
    break;
  case INTT:
    mytext = mysptr; /* sauvegarde le jeton	*/
  int_part:
    myint = (int)(CURC - '0');
    mysptr++;
    for (; isintt((int)CURC); mysptr++)
      myint = myint * 10 + (int)(CURC - '0');
    switch (CURC) {
    case '.': /* lecture fraction	*/
    float_part:
      mysptr++;
      for (; isintt((int)CURC); mysptr++) {
      };
      if (CURC != 'E' && CURC != 'e') {
        myfloat = (float)atof(mytext);
        /* FC
        printf("mytext %s, myfloat %f\n",mytext,myfloat);
        */
        return (T_FLOAT);
      }
      break;
    case 'E': /* lecture exposant	*/
    case 'e':
      mysptr++;
      if (isintt((int)CURC))
        mysptr++;
      else if (issgnt((int)CURC) && isintt((int)NEXTC))
        mysptr += 2;
      else {
        mysptr--;
        myfloat = (float)atof(mytext);
        return (T_FLOAT);
      }
      for (; isintt((int)CURC); mysptr++) {
      };
      myfloat = (float)atof(mytext);
      return (T_FLOAT);
      break;
    default:
      if (*mytext == '-')
        myint = -myint;
      return (T_INT);
      break;
    }
    break;
  case FPTT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (!isintt((int)CURC)) /* pas de fraction	*/
      return (*mytext);
    goto float_part;
    break;
  case SGNT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (isintt((int)CURC))
      goto int_part;
    if (isfptt((int)CURC) && isintt((int)NEXTC))
      goto float_part;
    return (*mytext);
    break;
  case STGT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
  string:
    for (; isstgt((int)CURC); mysptr++) {
    };
    switch (chtbl[(int)CURC]) {
    case EOBT:
      next_source();
      goto string;
      break;
    case EOFT:
      lexerr("start", lex_errtbl[E_STG_EOF], NULL);
      return ('\n');
      break;
    case EOLT:
      lexerr("start", lex_errtbl[E_STG_EOL], NULL);
      return ('\n');
      break;
    case STGT:
      if (PREVC != '\\') { /* veritable fin	*/
        mytext++;
        mylength = (int)(mysptr - mytext);
        mysptr++;
        return (T_STRING);
      }
      mysptr++; /* pseudo fin 		*/
      goto string;
      break;
    }
    break;
  default:
    ECHO;
    mysptr++;
    goto lex_loop;
    break;
  }
  return (T_EOF);
}

/*
 * La procedure "lexecho" contient l'analyseur lexical "lex" :
 * 1 Analyse le fichier source,
 * 2 Affiche le fichier source sur le fichier "f",
 * 3 Stoppe devant le jeton "token".
 * Note :
 * La tete de lecture (mysptr) n'est pas systematiquement avancee apres
 *lecture. Le caractere courant est celui sous la tete de lecture. Ainsi on
 *accede de maniere symetrique aux caracteres precedent et suivant. Entree :
 * f		Fichier en sortie.
 * token	Jeton de fin de rechercher.
 * Sortie :
 *		Code du symbole terminale analyse.
 */
int lexecho(FILE *f, int token)
{
lex_loop:
  for (; chtbl[(int)CURC] == SPCT; mysptr++) /* saute les espaces	*/
    fwrite(mysptr, 1, 1, f);

  switch (chtbl[(int)CURC]) {

  case NULT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (token != *mytext)
      fwrite(mytext, 1, 1, f);
    return (*mytext);
    break;
  case EOBT:
    next_source();
    goto lex_loop;
    break;
  case EOFT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    return (T_EOF);
    break;
  case EOLT:
    fwrite(mysptr, 1, 1, f);
    if (mysptr == lastline)
      next_source();
    else
      mysptr++;
    mylineno++;
    myline = mysptr;
    goto lex_loop;
    break;
  case CMTT:
    fwrite(mysptr, 1, 1, f);
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (CURC != '*')
      return (*mytext);
    fwrite(mysptr, 1, 1, f);
    mysptr++;
  comment:
    for (; iscmtt((int)CURC); mysptr++)
      fwrite(mysptr, 1, 1, f);
    switch (chtbl[(int)CURC]) {
    case EOBT:
      next_source();
      goto comment;
      break;
    case EOFT:
      lexerr("start", lex_errtbl[E_CMT_EOF], NULL);
      return (T_EOF);
      break;
    case EOLT:
      fwrite(mysptr, 1, 1, f);
      if (mysptr == lastline)
        next_source();
      else
        mysptr++;
      mylineno++;
      myline = mysptr;
      goto comment;
      break;
    case CMTT:
      fwrite(mysptr, 1, 1, f);
      if (PREVC == '*') { /* veritable fin	*/
        mysptr++;
        goto lex_loop;
      }
      mysptr++; /* pseudo fin 		*/
      goto comment;
      break;
    }
    break;
  case IDNT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    for (; isidnt((int)CURC); mysptr++) {
    };
    mylength = (int)(mysptr - mytext);
    if (token != get_symbol(mytext, mylength))
      fwrite(mytext, (size_t)mylength, 1, f);
    return (get_symbol(mytext, mylength));
    break;
  case INTT:
    mytext = mysptr; /* sauvegarde le jeton	*/
  int_part:
    mysptr++;
    for (; isintt((int)CURC); mysptr++) {
    };
    switch (CURC) {
    case '.': /* lecture fraction	*/
    float_part:
      mysptr++;
      for (; isintt((int)CURC); mysptr++) {
      };
      if (CURC != 'E' && CURC != 'e') {
        if (token != T_FLOAT)
          fwrite(mytext, (size_t)(mysptr - mytext), 1, f);
        return (T_FLOAT);
      }
      break;
    case 'E': /* lecture exposant	*/
    case 'e':
      mysptr++;
      if (isintt((int)CURC))
        mysptr++;
      else if (issgnt((int)CURC) && isintt((int)NEXTC))
        mysptr += 2;
      else {
        mysptr--;
        if (token != T_FLOAT)
          fwrite(mytext, (size_t)(mysptr - mytext), 1, f);
        return (T_FLOAT);
      }
      for (; isintt((int)CURC); mysptr++) {
      };
      if (token != T_FLOAT)
        fwrite(mytext, (size_t)(mysptr - mytext), 1, f);
      return (T_FLOAT);
      break;
    default:
      if (token != T_INT)
        fwrite(mytext, (size_t)(mysptr - mytext), 1, f);
      return (T_INT);
      break;
    }
    break;
  case FPTT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (!isintt((int)CURC)) { /* pas de fraction	*/
      if (token != *mytext)
        fwrite(mytext, 1, 1, f);
      return (*mytext);
    }
    goto float_part;
    break;
  case SGNT:
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
    if (isintt((int)CURC))
      goto int_part;
    if (isfptt((int)CURC) && isintt((int)NEXTC))
      goto float_part;
    if (token != *mytext)
      fwrite(mytext, 1, 1, f);
    return (*mytext);
    break;
  case STGT:
    fwrite(mysptr, 1, 1, f);
    mytext = mysptr; /* sauvegarde le jeton	*/
    mysptr++;
  string:
    for (; isstgt((int)CURC); mysptr++)
      fwrite(mysptr, 1, 1, f);
    switch (chtbl[(int)CURC]) {
    case EOBT:
      next_source();
      goto comment;
      break;
    case EOFT:
      lexerr("start", lex_errtbl[E_STG_EOF], NULL);
      return (T_EOF);
      break;
    case EOLT:
      lexerr("start", lex_errtbl[E_STG_EOL], NULL);
      return ('\n');
      break;
    case STGT:
      fwrite(mysptr, 1, 1, f);
      if (PREVC != '\\') { /* veritable fin	*/
        mytext++;
        mylength = (int)(mysptr - mytext);
        mysptr++;
        return (T_STRING);
      }
      mysptr++; /* pseudo fin 		*/
      goto string;
      break;
    }
    break;
  default:
    fwrite(mysptr, 1, 1, f);
    mysptr++;
    goto lex_loop;
    break;
  }
  return (T_EOF);
}

#undef BUFSIZE
#undef LINESIZE
#undef TEXTSIZE

#define BUFSIZE (BUFSIZ << 5)
#define LINESIZE (BUFSIZ - 1)
#define TEXTSIZE (1 + LINESIZE + BUFSIZE + 1)

static FILE *fds;    /* descripteur du fichier source	*/
static char *source; /* nom du fichier du programme source	*/
static char *botbuf; /* fond	  du buffer d'entree du fichier	*/
static char *buf;    /* base   du buffer d'entree du fichier	*/
static char *topbuf; /* sommet du buffer d'entree du fichier	*/

/*
 * La procedure "unlex" recule la tete de lecture devant le dernier jeton.
 */
void unlex(void) { mysptr = mytext; }

/*
 * La procedure "open_source" alloue et initialise les variables utilisees
 * pour la gestion des entrees du programme source.
 * Entree :
 * fd		Fichier du programme source.
 * sptr		Nom du fichier du programme source.
 */
void open_source(FILE *fd, const char *str)
{
  if ((source = (char *)malloc((strlen(str) + 1) * sizeof(char))) == NULL) {
    static char proc_name[] = "open_source";
    perror(proc_name);
    exit(1);
  }
  strcpy(source, str);
  if ((botbuf = (char *)malloc(TEXTSIZE * sizeof(char))) == NULL) {
    static char proc_name[] = "open_source";
    perror(proc_name);
    exit(1);
  }
  fds = fd;
  buf = botbuf + 1 + LINESIZE;
  topbuf = buf + 1;
  mylineno = 1;
  next_source();
}

/*
 * La procedure "close_source" libere les variables utilisees pour la gestion
 * des entrees du programme source.
 */
void close_source(void)
{
  free((char *)source);
  free((char *)botbuf);
}

/*
 * La procedure "next_source" remplit le buffer courant.
 */
static void next_source(void)
{
  size_t size;
  char *bot = buf;
  char *top = topbuf;

  /* recopie la derniere ligne devant "buf"	*/

  *bot = EOL; /* evite le debordement de "buf"	*/
  while ((*--bot = *--top) != EOL) {
  };
  myline = mysptr = bot + 1;

  size = fread(buf, sizeof(char), BUFSIZE, fds);
  if (size == 0) {
    topbuf = buf + 1;
    *buf = EOF;
    *topbuf = EOB; /* sentinelle de fin de fichier	*/
    mysptr = buf;
  } else {
    topbuf = buf + size;
    *topbuf = EOB; /* sentinelle de fin de buffer	*/

    /* recherche de la derniere ligne	*/
    top = topbuf;
    while (*--top != EOL) {
    };
    lastline = top;
  }
}

/*
 * ERR_STACK	: Pile des messages d'erreur.
 * La pile est geree par les procedures "poperr", "popuperr" et "pusherr".
 * Les messages sont affiches par les procedures "count" et "lexerr".
 */
#define ERR_STACK_MAX 32

static const char *err_stack[ERR_STACK_MAX];
static int size_stack = 0;

/*
 * La procedure "count" calcule la distance en espaces entre
 * le premier caractere "*mytext" et le caractere de debut de ligne "*myline".
 */
static void count(void)
{
  char *str;

  mycolumno = 0;
  for (str = myline; str <= mytext; str++) {
    (*str == '\t') ? mycolumno += 8 - (mycolumno % 8) : mycolumno++;
  }
}

/*
 * La procedure "lexerr" affiche les messages d'erreur.
 * 1 elle affiche la ligne d'erreur du fichier source.
 * 2 elle indique la position de l'erreur dans la ligne.
 * 3 elle affiche les messages d'erreur contenus dans la pile.
 * 4 elle affiche les messages d'erreur en parametre.
 * Entree :
 * va_list	Liste de messages d'erreur terminee par NULL.
 */

// lexerr (va_alist)
// va_dcl

void lexerr(const char *path, ...)
{
  va_list ap;
  char *cp;
  int i;

  /* Pointe sur le caractere fautif.	*/

  count();
  // write (STDERR, myline, mysptr - myline);
  fprintf(stderr, "\n%*c\n\"%s\", line %d:\n", mycolumno, '^', source, mylineno);

  /* Affiche les messages d'erreur de la pile.	*/

  for (i = 0; i < size_stack; i++)
    fprintf(stderr, "%s", err_stack[i]);

  /* Affiche les messages d'erreur en parametres.	*/

  va_start(ap, path);
  while ((cp = (char *)va_arg(ap, char *)) != NULL)
    fprintf(stderr, "%s", cp);
  fprintf(stderr, "\n");
  va_end(ap);

  exit(1);
}

/*
 * La procedure "poperr" depile le message d'erreur du sommet de pile.
 */
void poperr(void)
{
  if (--size_stack < 0) {
    static char proc_name[] = "poperr";
    fprintf(stderr, "%s: error stack underflow\n", proc_name);
    exit(1);
  }
}

/*
 * La procedure "popup_error" remplace le message d'erreur du sommet de pile.
 */
void popuperr(const char *str)
{
  if (size_stack <= 0) {
    static const char proc_name[] = "popuerr";
    fprintf(stderr, "%s: error stack underflow\n", proc_name);
    exit(1);
  }
  err_stack[size_stack - 1] = str;
}

/*
 * La procedure "pusherr" empile le message d'erreur.
 */
void pusherr(const char *str)
{
  if (size_stack >= ERR_STACK_MAX) {
    static const char proc_name[] = "pusherr";
    fprintf(stderr, "%s: error stack overflow\n", proc_name);
    exit(1);
  }
  err_stack[size_stack++] = str;
}

#endif
