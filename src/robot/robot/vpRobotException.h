/****************************************************************************
 *
 * $Id: vpRobotException.h,v 1.8 2008-09-26 15:20:57 fspindle Exp $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Exception that can be emited by the vpRobot class and its derivates.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef __vpRobotException_H
#define __vpRobotException_H


/* ------------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


/* \file vpRobotException.h
   \brief error that can be emited by the vpRobot class and its derivates
 */
/* Classes standards. */
#include <iostream>                /* Classe std::ostream.    */
#include <string>                  /* Classe string.     */
#include <visp/vpConfig.h>
#include <visp/vpException.h>



/* ------------------------------------------------------------------------- */
/* --- CLASS --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/*!
  \ingroup Exception
  \brief Error that can be emited by the vpRobot class and its derivates.
 */
class VISP_EXPORT vpRobotException : public vpException
{
public:
  /*!
    \brief Lists the possible error than can be emmited while calling
    vpRobot member
   */
  enum errorRobotCodeEnum
    {

	    /** Erreur lancee par le constructor. */
	    constructionError,

	    /** Erreur lancee lors de la construction d'un objet CRobot
	     * correspondant a un robot reel si l'objet de la classe en
	     * question doit etre unique. */
	    uniqueRobotError,

	    /** Erreur lancee par les fonctions de commande si le
	     * robot n'est pas dans le bon etat au moment du passage
	     * d'ordre. */
	    wrongStateError,

	    /** Erreur lancee par les fonctions de changement d'etat
	     * si le changement demandee n'est pas possible. */
	    stateModificationError,

	    /** Erreur generee lors d'un retour non nulle d'une fonction
	     * de communication de la lib Afma6. */
	    communicationError,

	    /** Erreur lancee apres un appel a une fonction de la lib
	     * bas-niveau de control de l'afma6 ayant renvoye une erreur. */
	    lowLevelError,

	    /** Erreur lancee par la fonction de parsing des parametres du
	     * robot, si le fichier donne en entree n'est pas valide.
	     */
	    readingParametersError,

	    /** Erreur lancee par les methodes d'une classe qui necessite
	     * un appel a une fonction d'initialisation apres la
	     * construction si l'init n'a pas ete fait. */
	    notInitializedError,

	    /** Erreur lancee par les fonctions decrites dans lAPI mais
	     * pas completement implementee. Dans ce cas, la fonction
	     * affiche simplement un message d'erreur avant de sortir
	     * par le 'throw'.
	     */
	    notImplementedError,
	    /** Position is out of range.
	     */
	    positionOutOfRangeError
    } ;

public:
  vpRobotException (const int code, const char * msg);
  vpRobotException (const int code, const std::string & msg);
  vpRobotException (const int code);

};





#endif /* #ifndef __vpRobotException_H */


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
