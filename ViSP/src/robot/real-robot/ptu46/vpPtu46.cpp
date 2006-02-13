/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet LAGADIC / IRISA, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPtu46.cpp
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *
 *
 *  $Id: vpPtu46.cpp,v 1.4 2006-02-13 09:27:54 fspindle Exp $
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <visp/vpConfig.h>


/* ----------------------------------------------------------------------- */
/* --- INCLUDE ------------------------------------------------------------ */
/* ----------------------------------------------------------------------- */


//#include "vpRobotError.h"       /* Classe d'erreur de la lib robot.        */
#include <visp/vpPtu46.h>      /* Header definissant la classe.           */

#include <visp/vpDebug.h>     /* Macros de trace et debugage.            */

#include <visp/vpRobotException.h>/* Classe d'erreur de la lib robot.     */


/* Inclusion des fichiers standards.		*/
#include <math.h>
#include <visp/vpMath.h>





/* ------------------------------------------------------------------------ */
/* --- COMPUTE ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
const int   vpPtu46::articulationsNb = 2;
const float vpPtu46::L               = 0.0765;
const float vpPtu46::h               = 0.068;
const float vpPtu46::l2              = 0.0;


/*!
  Compute the direct geometric model of the camera: fMc
 */
void vpPtu46::
computeMGD (const vpColVector & q,
	    vpHomogeneousMatrix & fMc)
{
  DEBUG_TRACE (3, "# Entree.");

  if (q.getRows() != 2) {
    ERROR_TRACE("Bad dimension for ptu-46 articular vector");
    throw;
  }

  double            q1 = q[0]; // pan
  double            q2 = q[1]; // tilt

  double            c1 = cos(q1);
  double            s1 = sin(q1);
  double            c2 = cos(q2);
  double            s2 = sin(q2);

  fMc[0][0] = s1;
  fMc[0][1] = c1*s2;
  fMc[0][2] = c1*c2;
  fMc[0][3] = -h*c1*s2 - L*s1 + l2*c1*c2;

  fMc[1][0] = -c1;
  fMc[1][1] = s1*s2;
  fMc[1][2] = s1*c2;
  fMc[1][3] = -h*s1*s2 + L*c1 + l2*s1*c2;

  fMc[2][0] = 0;
  fMc[2][1] = -c2;
  fMc[2][2] = s2;
  fMc[2][3] = h*c2 + l2*s2;

  fMc[3][0] = 0;
  fMc[3][1] = 0;
  fMc[3][2] = 0;
  fMc[3][3] = 1;

  CDEBUG (6) << "Position de la camera: " << endl << fMc;

  DEBUG_TRACE (3, "# Sortie.");
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

vpHomogeneousMatrix vpPtu46::
computeMGD (const vpColVector & q)
{
  DEBUG_TRACE (6, "# Entree.");

  DEBUG_TRACE (9, "Creation de la matrice resultat.");
  vpHomogeneousMatrix fMc;

  DEBUG_TRACE (9, "Appel du calcul de fMc.");
  computeMGD (q, fMc);

  DEBUG_TRACE (6, "# Sortie.");
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
void vpPtu46::
computeMGD (const vpColVector & q,
	    vpPoseVector & r)
{
  DEBUG_TRACE (6, "# Entree.");

  DEBUG_TRACE (9, "Creation de la matrice resultat.");
  vpHomogeneousMatrix fMc;

  DEBUG_TRACE (9, "Appel du calcul de oMc.");
  computeMGD (q, fMc);
  CDEBUG (15) << "fMc: " << endl << fMc;

  DEBUG_TRACE (9, "Conversion du cMf en vecteur.");
  r.buildFrom(fMc.inverse());

  DEBUG_TRACE (6, "# Sortie.");
  return ;
}




/* -------------------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */


/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTEUR ------------------------------------------------------ */
/* ----------------------------------------------------------------------- */

/* Constructeur vide. */
vpPtu46::
vpPtu46 (void)

  /* Tous les autres attributs sont initialises dans l'init. */
{
  DEBUG_TRACE (25, "# Entree - Sortie.");
  init();
}
/* ---------------------------------------------------------------------- */
/* --- PRIVATE ---------------------------------------------------------- */
/* ---------------------------------------------------------------------- */



void vpPtu46::
init ()
{
  DEBUG_TRACE (15, "Objet initialise !");

  return ;
}


/* ----------------------------------------------------------------------- */
/* --- DISPLAY ----------------------------------------------------------- */
/* ----------------------------------------------------------------------- */





ostream & operator << (ostream & os,
		       const vpPtu46 & constant)
{

  os
    << "Geometric parameters: " << endl
    << "L: "
    << "\t" << vpPtu46::L << endl
    << "h: "
    << "\t" << vpPtu46::h
    << "l2: "
    << "\t" << vpPtu46::l2
    << "\t" << endl;

  return os;
}


void
vpPtu46::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

void
vpPtu46::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpHomogeneousMatrix eMc ;

  eMc[0][0] = 0;
  eMc[0][1] = -1;
  eMc[0][2] = 0;
  eMc[0][3] = h;

  eMc[1][0] = 1;
  eMc[1][1] = 0;
  eMc[1][2] = 0;
  eMc[1][3] = -L;

  eMc[2][0] = 0;
  eMc[2][1] = 0;
  eMc[2][2] = 1;
  eMc[2][3] = l2;

  eMc[3][0] = 0;
  eMc[3][1] = 0;
  eMc[3][2] = 0;
  eMc[3][3] = 1;

  cMe = eMc.inverse()  ;
}

/*!
  \brief get the robot Jacobian expressed in the end-effector frame

  \warning Re is not the embedded camera  frame (see also get_cMe)

*/
void
vpPtu46::get_eJe(const vpColVector &q, vpMatrix &eJe)
{


  eJe.resize(6,2) ;

  if (q.getRows() != 2) {
    ERROR_TRACE("Bad dimension for ptu-46 articular vector");
    throw;
  }

  double s2 = sin(q[1]) ;
  double c2 = cos(q[1]) ;

  eJe = 0;

  eJe[3][0] = c2;
  eJe[4][1] = 1;
  eJe[5][0] = s2;

}
/*!
  \brief get the robot Jacobian expressed in the robot reference frame
  \exception vpRobotException (vpRobotException::ERRNotImplemented)
*/

void
vpPtu46::get_fJe(const vpColVector &q, vpMatrix &fJe)
{

  if (q.getRows() != 2) {
    ERROR_TRACE("Bad dimension for ptu-46 articular vector");
    throw;
  }

  fJe.resize(6,2) ;

  double s1 = sin(q[0]) ;
  double c1 = cos(q[0]) ;

  fJe = 0;

  fJe[3][1] = s1;
  fJe[4][1] = -c1;
  fJe[5][0] = 1;
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

