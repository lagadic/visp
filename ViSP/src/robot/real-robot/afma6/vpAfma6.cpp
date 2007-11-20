/****************************************************************************
 *
 * $Id: vpAfma6.cpp,v 1.17 2007-11-20 16:51:36 fspindle Exp $
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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Nicolas Mansard
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_AFMA6
/* ----------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------ */
/* ----------------------------------------------------------------------- */


//#include "vpRobotError.h"       /* Classe d'erreur de la lib robot.        */
#include <visp/vpAfma6.h>      /* Header definissant la classe.           */

#include <visp/vpDebug.h>     /* Macros de trace et debugage.            */

#include <visp/vpRobotException.h>/* Classe d'erreur de la lib robot.     */

#include <visp/vpXmlParserCamera.h>/* Classe de la libxml2 pour lire les
  parametres intrinsèques de camera*/
/* Inclusion des fichiers standards.		*/
#include <math.h>
#include <visp/vpMath.h>





/* ------------------------------------------------------------------------ */
/* --- COMPUTE ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
const double vpAfma6::rhoDefault = 0.1;
const int    vpAfma6::articulationsNb = 6;
const vpAfma6::CameraRobotType vpAfma6::defaultCameraRobot = CAMERA_DRAGONFLY2_12MM;



void vpAfma6::
computeMGD (const vpColVector & q,
	    vpHomogeneousMatrix & oMc)
{
  vpDEBUG_TRACE (3, "# Entree.");

  /* Passage de m en mm. */
  double            q0 = q[0] * 1000;
  double            q1 = q[1] * 1000;
  double            q2 = q[2] * 1000;
  /* Decouplage liaisons 2 et 3. */
  double            q5 = q[5] - coupl * q[4];

  double            c1 = cos(q[3]);
  double            s1 = sin(q[3]);
  double            c2 = cos(q[4]);
  double            s2 = sin(q[4]);
  double            c3 = cos(q5);
  double            s3 = sin(q5);

  /* Calcul du modele d'apres les angles. */
  oMc[0][0] = s1*s2*c3 + c1*s3;
  oMc[0][1] = -s1*s2*s3 + c1*c3;
  oMc[0][2] = -s1*c2;
  oMc[0][3] = q0 + l*c1;

  oMc[1][0] = -c1*s2*c3 + s1*s3;
  oMc[1][1] = c1*s2*s3 + s1*c3;
  oMc[1][2] = c1*c2;
  oMc[1][3] = q1 + l*s1;

  oMc[2][0] = c2*c3;
  oMc[2][1] = -c2*s3;
  oMc[2][2] = s2;
  oMc[2][3] = q2;

  oMc[3][0] = 0;
  oMc[3][1] = 0;
  oMc[3][2] = 0;
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

  /* Passage de mm en m. */
  for (int i = 0; i < 3; ++ i) {  oMc [i][3] /= 1000;  }

  vpDEBUG_TRACE (3, "# Sortie.");
  return ;
}




/* ------------------------------------------------------------------------ */
/* --- AUTRES METHODES ---------------------------------------------------- */
/* ---------------------------------------------------------------------- */



/* Calcul le MGD.
 * Calcul le MGD du robot a partir de la position articulaire <q>
 * donnee en argument. Cree une matrice de taille 6x6 et y place le resultat.
 * INPUT:
 *   - q : vecteur articulaire.
 * OUTPUT:
 *   - matrice homogene representant les changementsde repere
 * pour passer du repere fixe du robot au repere camera.
 */

vpHomogeneousMatrix vpAfma6::
computeMGD (const vpColVector & q)
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
void vpAfma6::
computeMGD (const vpColVector & q,
	    vpPoseVector & r)
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



/* ---------------------------------------------------------------------- */
/* --- STATIC ---------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

#define CTE_L  -0.068825

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
 *   - ERRNotInitialized si l'objet CParametresAfma6 n'a pas ete
 * initialise avant l'appel a cette fonction.
 */

void vpAfma6::
computeInverseJacobian (const vpColVector & pos,
			vpMatrix & Jinverse)
{
  vpDEBUG_TRACE (9, "# Entree.");

  if (! dejaInitialisee)
  {
    vpERROR_TRACE ("L'objet n'a pas ete initialise correctement. Aucun "
		 "fichier de parametres n'a ete parse. La fonction "
		 "CParametresAfma6::init () doit etre appelee apres "
		 "la construction.");
    throw vpRobotException (vpRobotException::notInitializedError,
			    "L'objet n'a pas ete initialise correctement. Aucun "
			    "fichier de parametres n'a ete parse. La fonction "
			    "CParametresAfma6::init () doit etre appelee apres "
			    "la construction.");
  }

  int i;
  int j;

  //rotation du passage effecteur/camera
  vpMatrix           effcamR(3,3);
  //translation du passage effecteur/camera
  vpColVector        effcamT(3);

  //  std::cout << rpi << std::endl;
  for(i=0;i<3;i++)
  {
    // translation entre camera et effecteur en metre
    effcamT[i]= rpi [i][3]/1000;
    for(j=0;j<3;j++)
    {
      // rotation entre camera et effecteur
      effcamR[i][j] = rpi [i][j];
    }
  }

  /* Matrice de preproduit vectoriel associe a cameffT. */
  vpMatrix           preprodT(3,3);
  vpMatrix           temp(3,3);
  /* Passage camera effecteur rotation. */
  vpMatrix           cameffR(3,3);
  /* Passage camera effecteur translation. */
  vpColVector        cameffT(3);

  preprodT = 0.0;
  cameffR = effcamR.t();
  cameffT = cameffR*effcamT*(-1.0);
  //cameffT = effcamT;
  preprodT[0][1] = -cameffT[2];
  preprodT[0][2] =  cameffT[1];
  preprodT[1][0] =  cameffT[2];
  preprodT[1][2] = -cameffT[0];
  preprodT[2][0] = -cameffT[1];
  preprodT[2][1] =  cameffT[0];

  temp = effcamR*preprodT*(-1.0); // -R*[T]
  //temp = preprodT;

  /* Jacobien inverse dans le repere fixe articulaire. */
  vpMatrix           J1(3,3),J2(3,3);
  /* Passage repere fixe repere effecteur. */
  vpMatrix           R36(3,3);

  J1 = 0.0;
  J2 = 0.0;
  R36 = 0.0;

  double s1,c1,s2,c2,s3,c3,t2 ;
  s1=sin(pos[3]); c1=cos(pos[3]);
  s2=sin(pos[4]); c2=cos(pos[4]);
  s3=sin(pos[5]); c3=cos(pos[5]);
  //Singularite pour q4 = pi/2
  int sing = 0;

  if (fabs(c2)< 0.1)

  {

    if(sing == 0)
    {
      std::cerr << "Singularite : q5" << c2 << std::endl;
      sing = 1;
    }
    t2 = 0.0;
    J2[2][0] = 0.0;
    J2[2][1] = 0.0;

  }
  else
  {
    if(sing == 1)
    {
      std::cerr << "Sortie de singularite : q5" << c2 << std::endl;
      sing = 0;
    }
    t2 = tan(pos[4]) ;
    J2[2][0] = -s1/c2;
    J2[2][1] = c1/c2;
  }



  /* Inverse Jacobian Matrix  Hand Frame */


  J1[0][0] = CTE_L*s1*s1*t2;
  J1[0][1] = -CTE_L*s1*c1*t2;
  J1[0][2] = CTE_L*s1;

  J1[1][0] = -CTE_L*s1*c1*t2;
  J1[1][1] = CTE_L*c1*c1*t2;
  J1[1][2] = -CTE_L*c1;

  J2[0][0] = s1*t2;
  J2[0][1] = -c1*t2;
  J2[0][2] = 1.0;

  J2[1][0] = c1;
  J2[1][1] = s1;



  //calcul du passage repere fixe repere effecteur
  R36[0][0]= s1*s2*c3+c1*s3;
  R36[0][1]= -s1*s2*s3 + c1*c3;
  R36[0][2]= -s1*c2;
  R36[1][0]=-c1*s2*c3+s1*s3;
  R36[1][1]=c1*s2*s3+s1*c3;
  R36[1][2]=c1*c2;
  R36[2][0]=c2*c3;
  R36[2][1]=-c2*s3;
  R36[2][2]=s2;

  vpMatrix J11(3,3), J12(3,3),  J22(3,3);

  J11 = J12 = J22 = 0.0;
  J11 = R36*effcamR;
  J12 = R36*temp + J1*R36*effcamR;
  J22 = J2*R36*effcamR;


  Jinverse .resize(6,6) ; //jacobien inverse dans repere camera
  //calcul du jacobien inverse dans repere de la camera vitessearticulaire = Jinverse*Vitessecamera
  Jinverse = 0.0;
  for(i=0 ; i<3 ; ++ i)
  {
    for(j=0 ; j<3 ; ++ j)
    {
      Jinverse[i][j]     = J11[i][j];
      Jinverse[i][j+3]   = J12[i][j];
      Jinverse[i+3][j]   = 0 ;
      Jinverse[i+3][j+3] = J22[i][j];
    }
  }
  vpDEBUG_TRACE (9, "# Sortie.");
  return  ;
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
vpMatrix vpAfma6::
computeInverseJacobian (const vpColVector & q)
{
  vpDEBUG_TRACE (6, "# Entree.");

  vpDEBUG_TRACE (9, "Creation de la matrice resultat.");
  vpMatrix Jp (6,6);

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
vpAfma6::getJointLimitsMin (vpColVector &qmin) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    qmin.resize(vpAfma6::articulationsNb);

    for (int i = 0; i < vpAfma6::articulationsNb ; ++ i)
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
vpAfma6::getJointLimitsMax (vpColVector &qmax) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    qmax.resize(vpAfma6::articulationsNb);

    for (int i = 0; i < vpAfma6::articulationsNb ; ++ i)
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
void vpAfma6::
respectJointLimits (vpColVector &q) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    for (int i = 0 ; i < vpAfma6::articulationsNb ; ++ i)
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
bool vpAfma6::
areJointLimitsrespected (const vpColVector & q) const
{
  bool res = true;

  for (int i = 0 ; i < vpAfma6::articulationsNb ; ++ i)
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

/* Renvoie la position courrante relative aux butees.
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
vpAfma6::getPositionInJointInterval (const vpColVector &q,
				     vpColVector &jointPos) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    jointPos.resize(vpAfma6::articulationsNb);

    for (int i = 0 ; i < vpAfma6::articulationsNb ; ++ i)
	{
	    jointPos [i]
		= ((q [i] - jointMin [i])
		   / (jointMax [i]- jointMin [i]));
	}

    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}


/* ------------------------------------------------------------------------ */
/* --- EVITEMENT ---------------------------------------------------------- */
/* ----------------------------------------------------------------------- */

/* Modifie le seuil d'activation des butees.
 * L'evitement est active quand la position relative au butee (renvoye par
 * getPositionInJointInterval) est inferieur a ce seuil, ou superieur a
 * 1.0 moins ce seuil).
 * La valeur par default est donnee par la valeur de la constante rhoDefault.
 * INPUT:
 *  - rho: nouvelle valeur du seuil.
 */
void vpAfma6::
setActivationThreshold (const double __rho)
{
    vpDEBUG_TRACE (6, "# Entree.");

    rho = __rho;

    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}

/* Recupere le seuil d'activation des butees.
 * OUTPUT:
 *  - rho: valeur courrante du seuil.
 */
void vpAfma6::
getActivationThreshold (double &__rho) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    __rho = rho;

    vpDEBUG_TRACE (6, "# Sortie.");
    return ;
}

/* Recupere le seuil d'activation des butees.
 * OUTPUT:
 *   - valeur courrante du seuil.
 */
double vpAfma6::
getActivationThreshold (void) const
{
    vpDEBUG_TRACE (6, "# Entree.");

    vpDEBUG_TRACE (6, "# Sortie.");
    return rho;
}

/* Calcule le gradient de la fonction de potentiel d'evitement
 * d'obstacle.
 * La fonction de potentiel est nulle quand la position est hors du
 * domaine parametre par le seuil rho (cf getActivationThreshold()),
 * et est maximal au contact des butees.
 * La fonction retourne vrai ssi le gradient n'est pas nul (au moins une
 * articulation proche de sa butee.
 * INPUT:
 *   - q: position courrante du robot.
 * OUTPUT:
 *   - grad: valeur du gradient calcule.
 *   - retourne vrai ssi le gradient n'est pas nul (au moins une
 * articulation proche de sa butee.
 */
bool vpAfma6::
getAvoidanceGradiant (const vpColVector & q,
		      vpColVector & grad)
{
  /* Par default, on retournera la valeur faux (pas d'articulation proche des
   * butees. */
  bool              returnCode = false;

  double            deltaQ;
  double            distance;

  /* Redimensionnement a la bonne taille. Un test est effectue dans Resize()
   * pour ne pas faire de travail inutile si le vecteur est deja de la bonne
   * taille. */
  grad .resize (vpAfma6::articulationsNb);

  /* Calcule du gradient proprement dit, axe par axe. */
  for (int i = 0 ; i < vpAfma6::articulationsNb ; ++ i)
    {

      /* deltaQ est la taille de l'intervalle articulaire autorise. */
      deltaQ = jointMax [i] - jointMin [i];

      /* distance est la distance maximal aux butees autorisees
       * avant evitement. On evite dans [Jmin, Jmin+distance] et
       * [Jmax-distance, Jmax] */
      distance = deltaQ * rho;

      /* L'articulation est dans la partie interdite inferieure. */
      if (q[i] + distance > jointMax [i])
	{
	  vpDEBUG_TRACE (15,"L'articulation %d est dans la partie "
		       "interdite inferieure.", i);
	  grad [i] = (- q [i] - distance + jointMax [i])/deltaQ;
	  returnCode = true;
	}

      /* L'articulation est dans la partie interdite superieure. */
      else if (q[i] - distance < jointMin [i])
	{
	  vpDEBUG_TRACE (15,"L'articulation %d est dans la partie "
		       "interdite superieure.", i);
	  grad [i] = (jointMin [i] - (q [i] - distance))/deltaQ;
	  returnCode = true;
	}

      /* L'articulation est dans la partie autorisee. Pas d'evitement
       * sur cet axe. */
      else
	{
	  grad [i] = 0;
	}
    }

  vpDEBUG_TRACE (6, "# Sortie r=%s.", (returnCode)?"VRAI":"FAUX");
  return returnCode;
}


/* Renvoie le nombre d'iteration prevues avant de toucher une butée,
 * si on conserve la meme commande.
 * La fonction mesure combien d'iteration seront necessaires avant de toucher
 * la butees si la meme commande est conservée. Le calcul est fait de la
 * maniere la plus simple : on incremente la position articulaire, jusqu'a ce
 * qu'un axe parte en butee. On renvoie le nombre d'iteration.
 * INPUT:
 *   - q_dot: commande articulaire courrante, sur laquelle appliquer le test.
 *   - limiteIteration: nombre max d'iteration a tester.
 * OUTPUT:
 *   - nombre d'iterations necessaire avec la commande q_dot pour arriver
 * sur une butée. Dans le cas ou on a tester toutes les iterations demandées par
 * <limiteIteration>, on renvoie <limiteIteration>+1.
 */
int vpAfma6::
iterationsAvantButees (const vpColVector &q,
		       const vpColVector & q_dot,
		       const int limiteIteration)              const
{
  vpColVector             q_tmp = q;
  bool                   horsButee = true;
  int                    iter = 0;

  while (horsButee && (iter <= limiteIteration))
    {
      /* Verifie les butees. */
      horsButee = areJointLimitsrespected (q_tmp);

      /* Deplace le robot virtuellement. */
      for (int i = 0; i < vpAfma6::articulationsNb; ++ i)
	{
	  q_tmp [i] += q_dot [i];
	}

      /* Continue apres increment. */
      iter ++;
    }

  return iter;
}

/* ----------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

const char * const vpAfma6::PARAMETRES_AFMA6_FILENAME
= "/udd/fspindle/robot/Afma6/current/include/const_Afma6.cnf";


const char * const vpAfma6::CONST_MPI_DRAGONFLY2_WITHOUT_DISTORTION
= "/udd/fspindle/robot/Afma6/current/include/const_mpi_camera_Dragonfly2_without_distortion_Afma6.cnf";
const char * const vpAfma6::CONST_MPI_DRAGONFLY2_WITH_DISTORTION
= "/udd/fspindle/robot/Afma6/current/include/const_mpi_camera_Dragonfly2_with_distortion_Afma6.cnf";

const char * const vpAfma6::PARAMETRES_CAMERA_AFMA6_FILENAME
= "/udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml";

const char * const vpAfma6::CONST_LABEL_DRAGONFLY2 = "Dragonfly2";

/* Declaration des variables statiques.		*/
static char *opt_Afma6[] = {"KP","KD","KI","PMAX","PMIN","INIT","SENS",
			    "ERIN","SATU","AMAX","VMAX","TRAI",
			    "TOPC","LONG","COUP","TETA","TRPI","CAMERA",
			    NULL};



/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTEUR ------------------------------------------------------ */
/* ----------------------------------------------------------------------- */

/* Constructeur vide. */
vpAfma6::
vpAfma6 (void)
  :
  dejaInitialisee (false),
  rho (vpAfma6::rhoDefault)

  /* Tous les autres attributs sont initialises dans l'init. */
{
  vpDEBUG_TRACE (25, "# Entree - Sortie.");
  init();
}
/* ---------------------------------------------------------------------- */
/* --- PRIVATE ---------------------------------------------------------- */
/* ---------------------------------------------------------------------- */


void vpAfma6::
initJointLimits (void)
{
  for (int i = 0; i < 6 ; ++i)
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
  for (int i = 0; i < 3 ; ++i)
  {
    this->jointMax[i] /= 1000;
    this->jointMin[i] /= 1000;
    this->jointMiddle[i] /= 1000;
  }

  return ;
}


void vpAfma6::
initRpi (void)
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

void vpAfma6::
parseConfigFile (const char * filename)
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
	 NULL != opt_Afma6[code];
	 ++ code)
    {
      if (strncmp(opt_Afma6[code], namoption, dim) == 0)
      {
	break;
      }
    }

    switch(code)
    {
    case 0 :
      sscanf(Ligne, "%s %f %f %f %f %f %f", namoption,
	     &this->Kp[0], &this->Kp[1],
	     &this->Kp[2], &this->Kp[3],
	     &this->Kp[4], &this->Kp[5]);
      break;

    case 1 :
      sscanf(Ligne, "%s %f %f %f %f %f %f", namoption,
	     &this->Kd[0], &this->Kd[1],
	     &this->Kd[2], &this->Kd[3],
	     &this->Kd[4], &this->Kd[5]);
      break;

    case 2 :
      sscanf(Ligne, "%s %f %f %f %f %f %f", namoption,
	     &this->Ki[0], &this->Ki[1],
	     &this->Ki[2], &this->Ki[3],
	     &this->Ki[4], &this->Ki[5]);
      break;

    case 3 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld",
	     namoption,
	     &this->QMax[0], &this->QMax[1],
	     &this->QMax[2], &this->QMax[3],
	     &this->QMax[4], &this->QMax[5]);
      break;

    case 4 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld", namoption,
	     &this->QMin[0], &this->QMin[1],
	     &this->QMin[2], &this->QMin[3],
	     &this->QMin[4], &this->QMin[5]);
      break;

    case 5 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld", namoption,
	     &this->RstQm[0], &this->RstQm[1],
	     &this->RstQm[2], &this->RstQm[3],
	     &this->RstQm[4], &this->RstQm[5]);
      break;

    case 6 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld", namoption,
	     &this->SensDep[0], &this->SensDep[1],
	     &this->SensDep[2], &this->SensDep[3],
	     &this->SensDep[4], &this->SensDep[5]);
      break;

    case 7 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld", namoption,
	     &this->EpsMax[0], &this->EpsMax[1],
	     &this->EpsMax[2], &this->EpsMax[3],
	     &this->EpsMax[4], &this->EpsMax[5]);
      break;

    case 8 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld", namoption,
	     &this->TiMax[0], &this->TiMax[1],
	     &this->TiMax[2], &this->TiMax[3],
	     &this->TiMax[4], &this->TiMax[5]);
      break;

    case 9 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld", namoption,
	     &this->AccMax[0], &this->AccMax[1],
	     &this->AccMax[2], &this->AccMax[3],
	     &this->AccMax[4], &this->AccMax[5]);
      break;

    case 10 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld",
	     namoption,
	     &this->VitMax[0], &this->VitMax[1],
	     &this->VitMax[2], &this->VitMax[3],
	     &this->VitMax[4], &this->VitMax[5]);
      break;

    case 11 :
      sscanf(Ligne, "%s %ld %ld %ld %ld %ld %ld",
	     namoption,
	     &this->ErrTMax[0], &this->ErrTMax[1],
	     &this->ErrTMax[2], &this->ErrTMax[3],
	     &this->ErrTMax[4], &this->ErrTMax[5]);
      break;

    case 12 :
      sscanf(Ligne, "%s %f %f %f %f %f %f", namoption,
	     &this->top[0], &this->top[1],
	     &this->top[2], &this->top[3],
	     &this->top[4], &this->top[5]);
      break;

    case 13 :
      sscanf(Ligne, "%s %f", namoption, &this->l);
      break;

    case 14 :
      sscanf(Ligne, "%s %f", namoption, &this->coupl);
      break;

    case 15 :
      sscanf(Ligne, "%s %lf %lf %lf", namoption,
	     &this->rrpi[0], &this->rrpi[1],
	     &this->rrpi[2]);

      this->rrpi[0] *= M_PI / 180.0 ;
      this->rrpi[1] *= M_PI / 180.0;
      this->rrpi[2] *= M_PI / 180.0;
	break;

    case 16 :
      sscanf(Ligne, "%s %lf %lf %lf", namoption,
	     &this->trpi[0], &this->trpi[1],
	     &this->trpi[2]);
      break;

    case 17:
      break; // rien a faire (nom de la camera).

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


void vpAfma6::
init (const char * paramAfma6,
      const char * paramCamera)
{
  this->FlagMod = 0;
  this->FlagReset = 0;

  vpDEBUG_TRACE (15, "Parsage fichier robot.");
  this->parseConfigFile (paramAfma6);

  vpDEBUG_TRACE (15, "Parsage fichier camera.");
  this->parseConfigFile (paramCamera);

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
 * PARAMETRES_AFMA6_FILENAME en argument.
 */
void vpAfma6::
init (vpAfma6::CameraRobotType camera, bool usedistortion)
{
  char            filenameMPI [FILENAME_MAX];

  switch (camera)
  {
  case vpAfma6::CAMERA_DRAGONFLY2_12MM:
    {
      if(usedistortion == false){
        snprintf(filenameMPI, 100, "%s",
	         CONST_MPI_DRAGONFLY2_WITHOUT_DISTORTION);
      }
      else{
        snprintf(filenameMPI, 100, "%s",
           CONST_MPI_DRAGONFLY2_WITH_DISTORTION);
      }
      break;
    }
  default:
    {
      vpERROR_TRACE ("Cette erreur ne peut pas arriver.");
      vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
		   "que les specs de la classe ont ete modifiee, "
		   "et que le code n'a pas ete mis a jour "
		   "correctement.");
      vpERROR_TRACE ("Verifiez les valeurs possibles du type "
		   "vpAfma6::CameraRobotType, et controlez que "
		   "tous les cas ont ete pris en compte dans la "
		   "fonction init(camera).");
      break;
    }
  }

  this->init (vpAfma6::PARAMETRES_AFMA6_FILENAME,
	      filenameMPI);

  setCameraRobotType(camera);
  return ;
}


/* Initialisation a l'aide du fichier par default.
 * Lance la methode init(char *) avec la constante
 * PARAMETRES_AFMA6_FILENAME en argument.
 */
void vpAfma6::
init (void)
{
  this->init ( vpAfma6::defaultCameraRobot);
  return;
}
/* ----------------------------------------------------------------------- */
/* --- DISPLAY ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */





std::ostream & operator << (std::ostream & os,
		       const vpAfma6 & constant)
{

  os
    << "Kp: "
    << "\t" << constant.Kp[0]
    << "\t" << constant.Kp[1]
    << "\t" << constant.Kp[2]
    << "\t" << constant.Kp[3]
    << "\t" << constant.Kp[4]
    << "\t" << constant.Kp[5]
    << "\t" << std::endl

    << "Kd: "
    << "\t" << constant.Kd[0]
    << "\t" << constant.Kd[1]
    << "\t" << constant.Kd[2]
    << "\t" << constant.Kd[3]
    << "\t" << constant.Kd[4]
    << "\t" << constant.Kd[5]
    << "\t" << std::endl


    << "Ki: "
    << "\t" << constant.Ki[0]
    << "\t" << constant.Ki[1]
    << "\t" << constant.Ki[2]
    << "\t" << constant.Ki[3]
    << "\t" << constant.Ki[4]
    << "\t" << constant.Ki[5]
    << "\t" << std::endl

    << "QMax"
    << "\t" << constant.QMax[0]
    << "\t" << constant.QMax[1]
    << "\t" << constant.QMax[2]
    << "\t" << constant.QMax[3]
    << "\t" << constant.QMax[4]
    << "\t" << constant.QMax[5]
    << "\t" << std::endl

    << "QMin: "
    << "\t" << constant.QMin[0]
    << "\t" << constant.QMin[1]
    << "\t" << constant.QMin[2]
    << "\t" << constant.QMin[3]
    << "\t" << constant.QMin[4]
    << "\t" << constant.QMin[5]
    << "\t" << std::endl

    << "RstQm: "
    << "\t" << constant.RstQm[0]
    << "\t" << constant.RstQm[1]
    << "\t" << constant.RstQm[2]
    << "\t" << constant.RstQm[3]
    << "\t" << constant.RstQm[4]
    << "\t" << constant.RstQm[5]
    << "\t" << std::endl

    << "SensDep: "
    << "\t" << constant.SensDep[0]
    << "\t" << constant.SensDep[1]
    << "\t" << constant.SensDep[2]
    << "\t" << constant.SensDep[3]
    << "\t" << constant.SensDep[4]
    << "\t" << constant.SensDep[5]
    << "\t" << std::endl

    << "EpsMax: "
    << "\t" << constant.EpsMax[0]
    << "\t" << constant.EpsMax[1]
    << "\t" << constant.EpsMax[2]
    << "\t" << constant.EpsMax[3]
    << "\t" << constant.EpsMax[4]
    << "\t" << constant.EpsMax[5]
    << "\t" << std::endl

    << "TiMax: "
    << "\t" << constant.TiMax[0]
    << "\t" << constant.TiMax[1]
    << "\t" << constant.TiMax[2]
    << "\t" << constant.TiMax[3]
    << "\t" << constant.TiMax[4]
    << "\t" << constant.TiMax[5]
    << "\t" << std::endl

    << "AccMax: "
    << "\t" << constant.AccMax[0]
    << "\t" << constant.AccMax[1]
    << "\t" << constant.AccMax[2]
    << "\t" << constant.AccMax[3]
    << "\t" << constant.AccMax[4]
    << "\t" << constant.AccMax[5]
    << "\t" << std::endl

    << "VitMax: "
    << "\t" << constant.VitMax[0]
    << "\t" << constant.VitMax[1]
    << "\t" << constant.VitMax[2]
    << "\t" << constant.VitMax[3]
    << "\t" << constant.VitMax[4]
    << "\t" << constant.VitMax[5]
    << "\t" << std::endl

    << "ErrTMax: "
    << "\t" << constant.ErrTMax[0]
    << "\t" << constant.ErrTMax[1]
    << "\t" << constant.ErrTMax[2]
    << "\t" << constant.ErrTMax[3]
    << "\t" << constant.ErrTMax[4]
    << "\t" << constant.ErrTMax[5]
    << "\t" << std::endl

    << "top:"
    << "\t" <<  constant.top[0]
    << "\t" << constant.top[1]
    << "\t" << constant.top[2]
    << "\t" << constant.top[3]
    << "\t" << constant.top[4]
    << "\t" << constant.top[5]
    << "\t" << std::endl

    << "l:"
    << "\t" << constant.l
    << "\t" << std::endl
    << "co: "
    << "\t" << constant.coupl
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

    << "Joint Limit Sup:"
    << "\t" << constant.jointMax[0]
    << "\t" << constant.jointMax[1]
    << "\t" << constant.jointMax[2]
    << "\t" << constant.jointMax[3]
    << "\t" << constant.jointMax[4]
    << "\t" << constant.jointMax[5]
    << "\t" << std::endl

    << "Joint Limit Max:"
    << "\t" << constant.jointMin[0]
    << "\t" << constant.jointMin[1]
    << "\t" << constant.jointMin[2]
    << "\t" << constant.jointMin[3]
    << "\t" << constant.jointMin[4]
    << "\t" << constant.jointMin[5]
    << "\t" << std::endl

    << "Joint Limit Middle:"
    << "\t" << constant.jointMiddle[0]
    << "\t" << constant.jointMiddle[1]
    << "\t" << constant.jointMiddle[2]
    << "\t" << constant.jointMiddle[3]
    << "\t" << constant.jointMiddle[4]
    << "\t" << constant.jointMiddle[5]
    << "\t" << std::endl ;

  return os;
}


void
vpAfma6::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

void
vpAfma6::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpHomogeneousMatrix eMc ;
  eMc = rpi; ;
  for(int i=0;i<3;i++)
    eMc[i][3] /=1000.0 ;

  cMe = eMc.inverse()  ;
}

#define CTE_L -0.068826

/*!
  \brief get the robot Jacobian expressed in the end-effector frame

  \warning Re is not the embedded camera  frame (see also get_cMe)

*/
void
vpAfma6::get_eJe(const vpColVector &q, vpMatrix &eJe)
{


  eJe.resize(6,6) ;
  try
  {
    double s3,c3,s4,c4,s5,c5 ;

    s3=sin(q[3]); c3=cos(q[3]);
    s4=sin(q[4]); c4=cos(q[4]);
    s5=sin(q[5]); c5=cos(q[5]);

    double L = CTE_L ;

    eJe = 0;
    eJe[0][0] = s3*s4*c5+c3*s5;
    eJe[0][1] = -c3*s4*c5+s3*s5;
    eJe[0][2] = c4*c5;
    eJe[0][3] = -L*s4*c5;

    eJe[1][0] = -s3*s4*s5+c3*c5;
    eJe[1][1] = c3*s4*s5+s3*c5;
    eJe[1][2] = -c4*s5;
    eJe[1][3] = L*s4*s5;

    eJe[2][0] = -s3*c4;
    eJe[2][1] = c3*c4;
    eJe[2][2] = s4;
    eJe[2][3] = L*c4;

    eJe[3][3] = c4*c5;
    eJe[3][4] = s5;

    eJe[4][3] = -c4*s5;
    eJe[4][4] = c5;

    eJe[5][3] = s4;
    eJe[5][5] = 1;

  }
  catch(...)
  {
    vpERROR_TRACE("catch exception") ;
    throw ;
  }
}
/*!
  \brief get the robot Jacobian expressed in the robot reference frame
  \exception vpRobotException (vpRobotException::ERRNotImplemented)
*/

void
vpAfma6::get_fJe(const vpColVector &q, vpMatrix &fJe)
{

  fJe.resize(6,6) ;

  try
  {

    /*
      J0

           |  1     0     0   -Ls4    0     0     |
           |  0     1     0    Lc4    0     0     |
           |  0     0     1     0     0     0     |
      J0 = |  0     0     0     0     c4  -s4 c5  |
           |  0     0     0     0     s4   c4 c5  |
           |  0     0     0     1     0    s5     |

  */


   // block superieur gauche
   fJe[0][0] = fJe[1][1] = fJe[2][2] = 1 ;

   double s4 = sin(q[4]) ;
   double c4 = cos(q[4]) ;


   // block superieur droit
   fJe[0][3] = -CTE_L*s4 ;
   fJe[1][3] = CTE_L*c4 ;


   double s5 = sin(q[5]) ;
   double c5 = cos(q[5]) ;
   // block inferieur droit
   fJe[3][4] = c4 ;     fJe[3][5] = -s4*c5 ;
   fJe[4][4] = s4 ;     fJe[4][5] = c4*c5 ;
   fJe[5][3] = 1 ;      fJe[5][5] = s5 ;


  }
  catch(...)
  {
    vpERROR_TRACE("Error caught");
    throw ;
  }

}

/*!
  \brief get the current intrinsic camera parameters
  \param cam : output : camera parameters to fill.
  \param image_width : image width used to compute camera calibration.
  \param image_height : image height used to compute camera calibration.
*/

void vpAfma6::
getCameraParameters (vpCameraParameters &cam,
                      const unsigned int image_width,
                      const unsigned int image_height)
{
  vpXmlParserCamera parser;
  switch (getCameraRobotType())
  {
  case vpAfma6::CAMERA_DRAGONFLY2_12MM:
    {
      parser.parse(cam,
		   vpAfma6::PARAMETRES_CAMERA_AFMA6_FILENAME,
		   vpAfma6::CONST_LABEL_DRAGONFLY2,
		   image_width, image_height);
      break;
    }
  default:
    {
      vpERROR_TRACE ("Cette erreur ne peut pas arriver.");
      vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
       "que les specs de la classe ont ete modifiee, "
       "et que le code n'a pas ete mis a jour "
       "correctement.");
      vpERROR_TRACE ("Verifiez les valeurs possibles du type "
       "vpAfma6::CameraRobotType, et controlez que "
       "tous les cas ont ete pris en compte dans la "
       "fonction init(camera).");
      break;
    }
  }
}
/*!
  \brief get the current intrinsic camera parameters
  \param cam : output : camera parameters to fill.
  \param I : image send by the current used camera.
*/
void vpAfma6::
getCameraParameters (vpCameraParameters &cam, const vpImage<unsigned char> &I)
{
  getCameraParameters(cam,I.getWidth(),I.getHeight());
}
/*!
  \brief get the current intrinsic camera parameters
  \param cam : output : camera parameters to fill.
  \param I : image send by the current used camera.
*/
void vpAfma6::
getCameraParameters (vpCameraParameters &cam, const vpImage<vpRGBa> &I)
{
  getCameraParameters(cam,I.getWidth(),I.getHeight());
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif


