/****************************************************************************
 *
 * $Id: vpAfma4.cpp,v 1.8 2007-04-20 14:22:16 asaunier Exp $
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
 * Interface for the Irisa's Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA4
/* ----------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------ */
/* ----------------------------------------------------------------------- */


//#include "vpRobotError.h"       /* Classe d'erreur de la lib robot.        */
#include <visp/vpAfma4.h>      /* Header definissant la classe.           */

#include <visp/vpDebug.h>     /* Macros de trace et debugage.            */

#include <visp/vpRobotException.h>/* Classe d'erreur de la lib robot.     */


/* Inclusion des fichiers standards.		*/
#include <math.h>
#include <visp/vpMath.h>





/* ------------------------------------------------------------------------ */
/* --- COMPUTE ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
const int    vpAfma4::articulationsNb = 4;

void
vpAfma4::computeMGD (const vpColVector & q, vpHomogeneousMatrix & oMc)
{
  vpDEBUG_TRACE (3, "# Entree.");

  /* Passage de m en mm. */
  double            q0 = q[0]; // rot tourelle
  double            q1 = q[1]; // vertical translation
  double            q2 = q[2]; // pan
  double            q3 = q[3]; // tilt

  double            c1 = cos(q0);
  double            s1 = sin(q0);
  double            c2 = cos(q2);
  double            s2 = sin(q2);
  double            c3 = cos(q3);
  double            s3 = sin(q3);

  /* Calcul du modele d'apres les angles. */
  oMc[0][0] = -c1*s2*c3 - s1*c2*c3;
  oMc[0][1] = c1*s2*s3 + s1*c2*s3;
  oMc[0][2] = c1*c2 - s1*s2;
  oMc[0][3] = c1*l-s1*(regle+L);

  oMc[1][0] = -s1*s2*c3 + c1*c2*c3;
  oMc[1][1] = s1*s2*s3 - c1*c2*s3;
  oMc[1][2] = s1*c2+c1*s2;
  oMc[1][3] = s1*l+c1*(regle+L);

  oMc[2][0] = s3;
  oMc[2][1] = c3;
  oMc[2][2] = 0.f;
  oMc[2][3] = q1;

  oMc[3][0] = 0.f;
  oMc[3][1] = 0.f;
  oMc[3][2] = 0.f;
  oMc[3][3] = 1;

  vpCDEBUG (6) << "Position de l'effecteur: " << std::endl << oMc;

  /* Multiplication par rapport poignee-image. */
  double            t [3];
  for (int i = 0; i < 3; ++ i)
  {
    for (int j = 0; j < 3; ++ j)
    {
      t[j] = 0;
      for (int k = 0; k < 3; ++ k)
      {
	t[j] += oMc[i][k] * rpi[k][j];
      }
      oMc[i][3] += oMc[i][j] * rpi[j][3];
    }

    for (int j = 0; j < 3; ++ j)
    {
      oMc[i][j] = t[j];
    }
  }

  vpDEBUG_TRACE (3, "# Sortie.");
  return ;
}




/* ------------------------------------------------------------------------ */
/* --- AUTRES METHODES ---------------------------------------------------- */
/* ---------------------------------------------------------------------- */



/* Calcul le MGD.
 * Calcul le MGD du robot a partir de la position articulaire <q>
 * donnee en argument. Cree une matrice de taille 4x6 et y place le resultat.
 * INPUT:
 *   - q : vecteur articulaire.
 * OUTPUT:
 *   - matrice homogene representant les changementsde repere
 * pour passer du repere fixe du robot au repere camera.
 */

vpHomogeneousMatrix
vpAfma4::computeMGD (const vpColVector & q)
{
  vpDEBUG_TRACE (6, "# Entree.");

  vpDEBUG_TRACE (9, "Creation de la matrice resultat.");
  vpHomogeneousMatrix fMc;

  vpDEBUG_TRACE (9, "Appel du calcul de oMc.");
  computeMGD (q, fMc);

  vpDEBUG_TRACE (6, "# Sortie.");
  return fMc;
}


/* Calcul le MGD du robot sous forme d'un vecteur.
 * Calcul le MGD du robot a partir de la position articulaire <q>
 * donnee en argument. Le resultat est sous forme d'une matrice
 * homogene. On traduit alors sous forme vectoriel de dimension 6.
 * INPUT:
 *   - q         : vecteur articulaire.
 * OUTPUT:
 *   -  r  : torseur camera calcule a partir de la matrice
 * homogene correspondant au MGD.
 */
void
vpAfma4::computeMGD (const vpColVector & q, vpPoseVector & r)
{
  vpDEBUG_TRACE (6, "# Entree.");

  vpDEBUG_TRACE (9, "Creation de la matrice resultat.");
  vpHomogeneousMatrix fMc;

  vpDEBUG_TRACE (9, "Appel du calcul de oMc.");
  computeMGD (q, fMc);
  vpCDEBUG (15) << "fMc: " << std::endl << fMc;

  vpDEBUG_TRACE (9, "Conversion du cMf en vecteur.");
  r.buildFrom(fMc.inverse());

  vpDEBUG_TRACE (6, "# Sortie.");
  return ;
}





/* -------------------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */


/* Calcul le jacobien inverse (J+) du robot a partir de la position articulaire <q>
 * donnee en argument. Place le resultat dans la matrice passee en argument.
 * INPUT:
 *   - pos: vecteur articulaire.
 * OUTPUT:
 *   - Jinverse  : Jacobien direct (matrice 6x6). dq/dt = (J+) dq/dt.
 * ATTENTION:
 *   - la singularite porte sur le deuxieme axe de rotation.
 * ERROR:
 *   - ERRNotInitialized si l'objet CParametresAfma4 n'a pas ete
 * initialise avant l'appel a cette fonction.
 */

void
vpAfma4::computeInverseJacobian (const vpColVector & pos,
				 vpMatrix & Jinverse)
{

  vpERROR_TRACE ("Inverse jacobian non implemented");
  throw vpRobotException (vpRobotException::notInitializedError,
			  "Inverse jacobian non implemented");
}




/* ------------------------------------------------------------------------ */
/* --- AUTRES METHODES --------------------------------------------------- */
/* ------------------------------------------------------------------------ */



/* Calcul le jacobien inverse.
 * Calcul le jacobien inverse (J+) du robot a partir de la position articulaire <q>
 * donnee en argument. Cree une matrice de taille 6x6 et y place le resultat.
 * INPUT:
 *  - q: vecteur articulaire.
 * OUTPUT:
 *  - Jacobien inverse J+ (matrice 6x6). dq/dt = (J+) dr/dt.
 */
vpMatrix
vpAfma4::computeInverseJacobian (const vpColVector & q)
{
  vpDEBUG_TRACE (6, "# Entree.");

  vpDEBUG_TRACE (9, "Creation de la matrice resultat.");
  vpMatrix Jp (4,6);

  vpDEBUG_TRACE (9, "Appel du calcul de cMo.");
  computeInverseJacobian (q, Jp);

  vpDEBUG_TRACE (6, "# Sortie.");
  return Jp;
}

#undef CTE_L



/* ----------------------------------------------------------------------- */
/* --- VALEURS ----------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!
   Get articular lower joint limits.

   \param qmin : Articular lower joint limits.

*/
void
vpAfma4::getJointLimitsMin (vpColVector &qmin) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    qmin.resize(vpAfma4::articulationsNb);

    for (int i = 0; i < vpAfma4::articulationsNb ; ++ i)
	{
	    qmin [i] = jointMin [i] ;
	}


    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}

/*!
   Get articular upper joint limits.

   \param qmax : Articular upper joint limits.

*/
void
vpAfma4::getJointLimitsMax (vpColVector &qmax) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    qmax.resize(vpAfma4::articulationsNb);

    for (int i = 0; i < vpAfma4::articulationsNb ; ++ i)
	{
	    qmax [i] = jointMax [i] ;
	}

    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}


/* ----------------------------------------------------------------------- */
/* --- POSITION ---------------------------------------------------------- */
/* ----------------------------------------------------------------------- */


/* Modifie la position pour rester dans l'espace autorise.
 * Modifie la valeur de la position courrante pour rester dans les
 * limites des butees.
 * INPUT-OUTPUT
 *  - q: renvoie la position apres modification.
 */
void
vpAfma4::respectJointLimits (vpColVector &q) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    for (int i = 0 ; i < vpAfma4::articulationsNb ; ++ i)
    {
      if ( jointMin [i] > q [i])      { q [i] = jointMin [i]; }
      else if (jointMax [i] < q [i]) { q [i] = jointMax [i]; }
    }


    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}

/* Verifie si les limites des butees sont respectées.
 * Renvoie VRAI si le vecteur q se situe bien dans l'espace autorisé, FAUX
 * sinon, c'est a dire si au moins un axe de q se situe en butée.
 * INPUT:
 *   - q: vecteur articulaire (dim 6) a tester.
 */
bool
vpAfma4::areJointLimitsrespected (const vpColVector & q) const
{
  bool res = true;

  for (int i = 0 ; i < vpAfma4::articulationsNb ; ++ i)
    {
      if ( jointMin [i] > q [i])
	{
	  res = false;
	  break;
	}
      else if ( jointMax [i] < q [i])
	{
	  res = false;
	  break;
	}
    }

  return res;
}

/*! Renvoie la position courrante relative aux butees.
 * Donne un pourcentage pour chaque axe donnant la position dans
 * l'intervalle des valeurs possibles. On obtient pour chaque axe
 * une valeur r tq r * DeltaQ + JointMin = q.
 * Le resultat est place dans le tableau donne en entree.
 * INPUT:
 *   - q: position articulaire du robot.
 * OUTPUT:
 *   - jointPos: position courrante relative aux butees.
 */
void
vpAfma4::getPositionInJointInterval (const vpColVector &q,
				     vpColVector &jointPos) const
{
    vpDEBUG_TRACE (6, "# Entree.");
    
    jointPos.resize(vpAfma4::articulationsNb);

    for (int i = 0 ; i < vpAfma4::articulationsNb ; ++ i)
	{
	    jointPos [i]
		= ((q [i] - jointMin [i])
		   / (jointMax [i]- jointMin [i]));
	}

    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}


/* ----------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

const char * const vpAfma4::PARAMETRES_AFMA4_FILENAME
= "/udd/fspindle/robot/Afma4/current/include/const_Afma4.cnf";


/* Declaration des variables statiques.		*/
static char *opt_Afma4[] = {"KP","KD","KI","PMAX","PMIN","INIT","SENS",
  "ERIN","SATU","AMAX","VMAX","TRAI","TOPD", "TOPR",
  "LONl", "LONL", "REGL", "TETA", "TRPI", NULL};


/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTEUR ------------------------------------------------------ */
/* ----------------------------------------------------------------------- */

/* Constructeur vide. */
vpAfma4::vpAfma4 (void)
  : dejaInitialisee (false)

  /* Tous les autres attributs sont initialises dans l'init. */
{
  vpDEBUG_TRACE (25, "# Entree - Sortie.");
  init();
}
/* ---------------------------------------------------------------------- */
/* --- PRIVATE ---------------------------------------------------------- */
/* ---------------------------------------------------------------------- */


void
vpAfma4::initJointLimits (void)
{
  for (int i = 0; i < articulationsNb ; ++i)
  {
    this->jointMax [i] = (double)this->QMax[i] / this->top[i];
    this->jointMin [i] = (double)this->QMin[i] / this->top[i];

    if (jointMin[i] > jointMax[i])
    {
      double swap ;
      swap = this->jointMax[i];
      this->jointMax[i] = this->jointMin[i];
      this->jointMin[i] = swap;
    }
    this->jointMiddle[i] = (this->jointMax[i] + this->jointMin[i]) / 2;
  }

  /* Passage de mm en m. */
  this->jointMax[1] /= 1000;
  this->jointMin[1] /= 1000;
  this->jointMiddle[1] /= 1000;

  return ;
}


void
vpAfma4::initRpi (void)
{
  /* Calcul de la matric rpi (poignee - image). */

  vpRotationMatrix Rrpi ;
  Rrpi.buildFrom(this->rrpi) ;

  rpi.insert(Rrpi) ;
  rpi.insert(trpi) ;
}


/* ------------------------------------------------------------------------ */
/* --- INIT --------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void
vpAfma4::parseConfigFile (const char * filename)
{
  int               dim;
  int               code;
  char              Ligne[100];
  char              namoption[10];
  float             val;
  FILE *            fdtask;
  int               numLn = 0;

  vpDEBUG_TRACE (15, "Ouverture du fichier de parametres %s.", filename);
  if ((fdtask = fopen(filename, "r" )) == NULL)
  {
    vpERROR_TRACE ("Impossible d'ouvrir le fichier de constantes robot %s.",
		 filename);
    fclose(fdtask);
    throw vpRobotException (vpRobotException::readingParametersError,
			    "Impossible d'ouvrir le fichier de constantes robot.");
  }

  vpDEBUG_TRACE (15, "Parsage du fichier.");
  while (fgets(Ligne,100,fdtask) != NULL)
  {
    numLn ++;
    if ('#' == Ligne[0]) { continue; }
    sscanf(Ligne, "%s %f", namoption, &val);
    dim = strlen(namoption);

    for (code = 0;
	 NULL != opt_Afma4[code];
	 ++ code)
    {
      if (strncmp(opt_Afma4[code], namoption, dim) == 0)
      {
	break;
      }
    }

    switch(code)
    {
    case 0 :
      sscanf(Ligne, "%s %f %f %f %f", namoption,
	     &this->Kp[0], &this->Kp[1],
	     &this->Kp[2], &this->Kp[3]);
      break;

    case 1 :
      sscanf(Ligne, "%s %f %f %f %f", namoption,
	     &this->Kd[0], &this->Kd[1],
	     &this->Kd[2], &this->Kd[3]);
      break;

    case 2 :
      sscanf(Ligne, "%s %f %f %f %f", namoption,
	     &this->Ki[0], &this->Ki[1],
	     &this->Ki[2], &this->Ki[3]);
      break;

    case 3 :
      sscanf(Ligne, "%s %ld %ld %ld %ld",
	     namoption,
	     &this->QMax[0], &this->QMax[1],
	     &this->QMax[2], &this->QMax[3]);
      break;

    case 4 :
      sscanf(Ligne, "%s %ld %ld %ld %ld", namoption,
	     &this->QMin[0], &this->QMin[1],
	     &this->QMin[2], &this->QMin[3]);
      break;

    case 5 :
      sscanf(Ligne, "%s %ld %ld %ld %ld", namoption,
	     &this->RstQm[0], &this->RstQm[1],
	     &this->RstQm[2], &this->RstQm[3]);
      break;

    case 6 :
      sscanf(Ligne, "%s %ld %ld %ld %ld", namoption,
	     &this->SensDep[0], &this->SensDep[1],
	     &this->SensDep[2], &this->SensDep[3]);
      break;

    case 7 :
      sscanf(Ligne, "%s %ld %ld %ld %ld", namoption,
	     &this->EpsMax[0], &this->EpsMax[1],
	     &this->EpsMax[2], &this->EpsMax[3]);
      break;

    case 8 :
      sscanf(Ligne, "%s %ld %ld %ld %ld", namoption,
	     &this->TiMax[0], &this->TiMax[1],
	     &this->TiMax[2], &this->TiMax[3]);
      break;

    case 9 :
      sscanf(Ligne, "%s %ld %ld %ld %ld", namoption,
	     &this->AccMax[0], &this->AccMax[1],
	     &this->AccMax[2], &this->AccMax[3]);
      break;

    case 10 :
      sscanf(Ligne, "%s %ld %ld %ld %ld",
	     namoption,
	     &this->VitMax[0], &this->VitMax[1],
	     &this->VitMax[2], &this->VitMax[3]);
      break;

    case 11 :
      sscanf(Ligne, "%s %ld %ld %ld %ld",
	     namoption,
	     &this->ErrTMax[0], &this->ErrTMax[1],
	     &this->ErrTMax[2], &this->ErrTMax[3]);
      break;

    case 12 :
      sscanf(Ligne, "%s %f %f %f %f",
	     namoption,
	     &this->Unit[0], &this->Unit[1],
	     &this->Unit[2], &this->Unit[3]);
      break;

    case 13 :
      sscanf(Ligne, "%s %f %f %f %f",
	     namoption,
	     &this->top[0], &this->top[1],
	     &this->top[2], &this->top[3]);
      break;

    case 14 :
      sscanf(Ligne, "%s %f",
	     namoption,
	     &this->l);
      break;

    case 15 :
      sscanf(Ligne, "%s %f",
	     namoption,
	     &this->L);
      break;

    case 16 :
      sscanf(Ligne, "%s %f",
	     namoption,
	     &this->regle);
      break;

    case 17 :
      sscanf(Ligne, "%s %lf %lf %lf",
	     namoption,
	     &this->rrpi[0], &this->rrpi[1],
	     &this->rrpi[2]);
      this->rrpi[0] *= M_PI / 180.0 ;
      this->rrpi[1] *= M_PI / 180.0;
      this->rrpi[2] *= M_PI / 180.0;
      break;

    case 18 :
      sscanf(Ligne, "%s %lf %lf %lf",
	     namoption,
	     &this->trpi[0], &this->trpi[1],
	     &this->trpi[2]);
      this->trpi[0] /= 1000.0;
      this->trpi[1] /= 1000.0;
      this->trpi[2] /= 1000.0;
      break;

    default:
      vpERROR_TRACE ("Fichier de configuration %s incorrect: "
		   "ligne #%d.", filename, numLn);
    } /* SWITCH */
  } /* WHILE */

    /* Fin de la lecture du fichier. */
  vpDEBUG_TRACE (15, "On ferme le fichier de configuration.");
  fclose (fdtask);

  return;
}


void
vpAfma4::init (const char * paramAfma4)
{
  this->FlagMod = 0;
  this->FlagReset = 0;

  vpDEBUG_TRACE (15, "Parsage fichier robot.");
  this->parseConfigFile (paramAfma4);

  vpDEBUG_TRACE (15, "Calcul de la matrice RPI.");
  this->initRpi ();

  vpDEBUG_TRACE (15, "Calcul du domaine Joint Limits.");
  this->initJointLimits ();

  vpDEBUG_TRACE (15, "Objet initialise !");
  this->dejaInitialisee = true;

  return ;
}

/* Initialisation a l'aide du fichier par default.
 * Lance la methode init(char *) avec la constante
 * PARAMETRES_AFMA4_FILENAME en argument.
 */
void
vpAfma4::init (void)
{
  this->init (vpAfma4::PARAMETRES_AFMA4_FILENAME);
  return;
}
/* ----------------------------------------------------------------------- */
/* --- DISPLAY ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */





std::ostream & operator << (std::ostream & os,
		       const vpAfma4 & constant)
{
  os
    << "Kp: "
    << "\t" << constant.Kp[0]
    << "\t" << constant.Kp[1]
    << "\t" << constant.Kp[2]
    << "\t" << constant.Kp[3]
    << "\t" << std::endl

    << "Kd: "
    << "\t" << constant.Kd[0]
    << "\t" << constant.Kd[1]
    << "\t" << constant.Kd[2]
    << "\t" << constant.Kd[3]
    << "\t" << std::endl


    << "Ki: "
    << "\t" << constant.Ki[0]
    << "\t" << constant.Ki[1]
    << "\t" << constant.Ki[2]
    << "\t" << constant.Ki[3]
    << "\t" << std::endl

    << "QMax"
    << "\t" << constant.QMax[0]
    << "\t" << constant.QMax[1]
    << "\t" << constant.QMax[2]
    << "\t" << constant.QMax[3]
    << "\t" << std::endl

    << "QMin: "
    << "\t" << constant.QMin[0]
    << "\t" << constant.QMin[1]
    << "\t" << constant.QMin[2]
    << "\t" << constant.QMin[3]
    << "\t" << std::endl

    << "Unit: "
    << "\t" << constant.Unit[0]
    << "\t" << constant.Unit[1]
    << "\t" << constant.Unit[2]
    << "\t" << constant.Unit[3]
    << "\t" << std::endl

    << "top: "
    << "\t" << constant.top[0]
    << "\t" << constant.top[1]
    << "\t" << constant.top[2]
    << "\t" << constant.top[3]
    << "\t" << std::endl

    << "RstQm: "
    << "\t" << constant.RstQm[0]
    << "\t" << constant.RstQm[1]
    << "\t" << constant.RstQm[2]
    << "\t" << constant.RstQm[3]
    << "\t" << std::endl

    << "SensDep: "
    << "\t" << constant.SensDep[0]
    << "\t" << constant.SensDep[1]
    << "\t" << constant.SensDep[2]
    << "\t" << constant.SensDep[3]
    << "\t" << std::endl

    << "EpsMax: "
    << "\t" << constant.EpsMax[0]
    << "\t" << constant.EpsMax[1]
    << "\t" << constant.EpsMax[2]
    << "\t" << constant.EpsMax[3]
    << "\t" << std::endl

    << "TiMax: "
    << "\t" << constant.TiMax[0]
    << "\t" << constant.TiMax[1]
    << "\t" << constant.TiMax[2]
    << "\t" << constant.TiMax[3]
    << "\t" << std::endl

    << "AccMax: "
    << "\t" << constant.AccMax[0]
    << "\t" << constant.AccMax[1]
    << "\t" << constant.AccMax[2]
    << "\t" << constant.AccMax[3]
    << "\t" << std::endl

    << "VitMax: "
    << "\t" << constant.VitMax[0]
    << "\t" << constant.VitMax[1]
    << "\t" << constant.VitMax[2]
    << "\t" << constant.VitMax[3]
    << "\t" << std::endl

    << "ErrTMax: "
    << "\t" << constant.ErrTMax[0]
    << "\t" << constant.ErrTMax[1]
    << "\t" << constant.ErrTMax[2]
    << "\t" << constant.ErrTMax[3]
    << "\t" << std::endl

    << "l:"
    << "\t" << constant.l
    << "\t" << std::endl
    << "L: "
    << "\t" << constant.L
    << "\t" << std::endl
    << "regle: "
    << "\t" << constant.regle
    << "\t" << std::endl

    << "rrpi: "
    << "\t" << vpMath::deg(constant.rrpi[0])
    << "\t" << vpMath::deg(constant.rrpi[1])
    << "\t" << vpMath::deg(constant.rrpi[2])
    << "\t" << std::endl

    << "trpi: "
    << "\t" << constant.trpi[0]
    << "\t" << constant.trpi[1]
    << "\t" << constant.trpi[2]
    << "\t" << std::endl

    << "mat1: "
    << "\t" << constant.rpi[0][0]
    << "\t" << constant.rpi[0][1]
    << "\t" << constant.rpi[0][2]
    << "\t" << constant.rpi[0][3]
    << "\t" << std::endl
    << "mat2: "
    << "\t" << constant.rpi[1][0]
    << "\t" << constant.rpi[1][1]
    << "\t" << constant.rpi[1][2]
    << "\t" << constant.rpi[1][3]
    << "\t" << std::endl
    << "mat3:"
    << "\t" << constant.rpi[2][0]
    << "\t" << constant.rpi[2][1]
    << "\t" << constant.rpi[2][2]
    << "\t" << constant.rpi[2][3]
    << "\t" << std::endl
    << "mat4:"
    << "\t" << constant.rpi[3][0]
    << "\t" << constant.rpi[3][1]
    << "\t" << constant.rpi[3][2]
    << "\t" << constant.rpi[3][3]
    << "\t" << std::endl

    << "Joint Limit Max:"
    << "\t" << constant.jointMax[0]
    << "\t" << constant.jointMax[1]
    << "\t" << constant.jointMax[2]
    << "\t" << constant.jointMax[3]
    << "\t" << std::endl

    << "Joint Limit Min:"
    << "\t" << constant.jointMin[0]
    << "\t" << constant.jointMin[1]
    << "\t" << constant.jointMin[2]
    << "\t" << constant.jointMin[3]
    << "\t" << std::endl

    << "Joint Limit Middle:"
    << "\t" << constant.jointMiddle[0]
    << "\t" << constant.jointMiddle[1]
    << "\t" << constant.jointMiddle[2]
    << "\t" << constant.jointMiddle[3]
    << "\t" << std::endl ;

  return os;
}


void
vpAfma4::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

void
vpAfma4::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpHomogeneousMatrix eMc ;
  eMc = rpi; ;
  for(int i=0;i<3;i++)
    eMc[i][3] /=1000.0 ;

  cMe = eMc.inverse()  ;
}

/*!
  \brief get the robot Jacobian expressed in the end-effector frame

  Not implemneted

  \warning Re is not the embedded camera  frame (see also get_cMe)

*/
void
vpAfma4::get_eJe(const vpColVector &q, vpMatrix &eJe)
{

  eJe = 0;
  vpERROR_TRACE("Jacobian expressed in the end-effector frame not implemneted");
}

/*!
  \brief get the robot Jacobian expressed in the robot reference frame
  \exception vpRobotException (vpRobotException::ERRNotImplemented)
*/

void
vpAfma4::get_fJe(const vpColVector &q, vpMatrix &fJe)
{

  fJe.resize(6,4) ;

  try
  {
    double c1 = cos(q[0]);
    double s1 = sin(q[0]);
    double c13 = cos(q[0] + q[2]);
    double s13 = sin(q[0] + q[2]);

    fJe[0][0] = -s1*l - c1*L;
    fJe[0][1] = fJe[0][2] = fJe[0][3] = 0.0;;

    fJe[1][0] = c1*l - s1*L;
    fJe[1][1] = fJe[1][2] = fJe[1][3] = 0.0;;

    fJe[2][1] = 1.0;
    fJe[2][0] = fJe[2][2] = fJe[2][3] = 0.0;;

    fJe[3][0] = fJe[3][1] = fJe[3][2] = 0.0;;
    fJe[3][3] = c13;

    fJe[4][0] = fJe[4][1] = fJe[4][2] = 0.0;;
    fJe[4][3] = s13;

    fJe[5][0] = fJe[5][3] = 1.0;
    fJe[5][2] = fJe[5][4] = 0.0;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught");
    throw ;
  }

}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif


