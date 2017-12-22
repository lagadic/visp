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
 * Exception that can be emited by the vpRobot class and its derivates.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpRobotException_H
#define __vpRobotException_H

/* -------------------------------------------------------------------------
 */
/* --- INCLUDE -------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/* \file vpRobotException.h
   \brief error that can be emited by the vpRobot class and its derivates
 */
/* Classes standards. */

#include <visp3/core/vpException.h>

#include <iostream> /* Classe std::ostream.    */
#include <string>   /* Classe string.     */

/* -------------------------------------------------------------------------
 */
/* --- CLASS ---------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/*!
  \class vpRobotException
  \brief Error that can be emited by the vpRobot class and its derivates.
 */
class VISP_EXPORT vpRobotException : public vpException
{
public:
  /*!
  \brief Lists the possible error than can be emmited while calling
  vpRobot member
  */
  enum errorRobotCodeEnum {

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
  };

public:
  vpRobotException(const int id, const char *format, ...)
  {
    this->code = id;
    va_list args;
    va_start(args, format);
    setMessage(format, args);
    va_end(args);
  }
  vpRobotException(const int id, const std::string &msg) : vpException(id, msg) {}
  explicit vpRobotException(const int id) : vpException(id) {}
};

#endif
