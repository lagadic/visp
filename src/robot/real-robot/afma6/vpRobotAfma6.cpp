/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA, 2004
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotAfma6.cpp
 * Project:   Visp2
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotAfma6.cpp,v 1.1.1.1 2005-06-08 07:08:08 fspindle Exp $
 *
 * Description
 * ============

 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <visp/vpConfig.h>
#ifdef HAVE_ROBOT_AFMA6

/* Headers des fonctions implementees. */
#include <visp/vpRobotAfma6.h>           /* Header de la classe.          */
#include <visp/vpRobotException.h>/* Erreurs lancees par les classes CRobot. */

#include <visp/vpDebug.h>           /* Macros de trace et debug.  */
// Fonctions AFMA6 bas niveau
#include <afma.h>          /* Communication avec le robot.      */

#include <signal.h>


#include "/udd/fspindle/robot/Afma6/current/src/robot/local/lib/sp_erreur_Afma6.h"

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

bool vpRobotAfma6::robotAlreadyCreated = false;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */



void emergencyStop(int signo)
{
  cout << "Arret de la tache par : SIGNAL= " << (char)7  ;
  switch(signo)
  {
  case  SIGINT:
    cout << "SIGINT (arret par ^C) " << endl ; break ;
  case SIGSEGV:
    cout <<"SIGSEGV (arret par segmentation fault) " << endl ; break ;
  case SIGBUS:
    cout <<"SIGBUS (arret par bus error) " << endl ; break ;
  default :
    cout << signo << endl ;
  }
  stop_mouvement_Afma6() ;
  vmeClose_Afma6();

  cout << "exit(1)" <<endl ;
  exit(1) ;
}


/*! constructor
 */
vpRobotAfma6::vpRobotAfma6 (void)
  :
  vpAfma6 (),
  vpRobot ()
{

 /*
#define	SIGHUP	1	// hangup
#define	SIGINT	2	// interrupt (rubout)
#define	SIGQUIT	3	// quit (ASCII FS)
#define	SIGILL	4	// illegal instruction (not reset when caught)
#define	SIGTRAP	5	// trace trap (not reset when caught)
#define	SIGIOT	6	// IOT instruction
#define	SIGABRT 6	// used by abort, replace SIGIOT in the future
#define	SIGEMT	7	// EMT instruction
#define	SIGFPE	8	// floating point exception
#define	SIGKILL	9	// kill (cannot be caught or ignored)
#define	SIGBUS	10	// bus error
#define	SIGSEGV	11	// segmentation violation
#define	SIGSYS	12	// bad argument to system call
#define	SIGPIPE	13	// write on a pipe with no one to read it
#define	SIGALRM	14	// alarm clock
#define	SIGTERM	15	// software termination signal from kill
  */

  signal(SIGINT, emergencyStop);
  signal(SIGBUS, emergencyStop) ;
  signal(SIGSEGV, emergencyStop) ;
  signal(SIGKILL, emergencyStop);
  signal(SIGQUIT, emergencyStop);

  DEBUG_TRACE (12, "Open communication with VME.");
  init();

  DEBUG_TRACE (12, "Read Config parameters.");
  vpAfma6::init ();

  try
  {
    setRobotState(vpRobot::STATE_STOP) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  positioningVelocity =  defaultPositioningVelocity ;
  return ;
}



/* -------------------------------------------------------------------------- */
/* --- INITIALISATION ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Initialise les connexions avec la carte VME et demare le robot.
 * ERROR:
 *   - ERRConstruction si une erreur survient lors de l'ouverture du VME.
 *   - ERRCommunication si une erreur survient lors de l'initialisation de
 * l'afma6.
 */
void
vpRobotAfma6::init (void)
{

  DEBUG_TRACE (12, "Open connection to the VME.");
  if (0 != vmeOpenA32D32_Afma6())
  {
    ERROR_TRACE ("Cannot open connexion between PC and VME.");
    throw vpRobotException (vpRobotException::ERRConstruction,
			    "Cannot open connexion between PC and VME");
  }

  if (0 != initialisation_Afma6())
  {
    ERROR_TRACE ("Error during robot initialization.");
    throw vpRobotException (vpRobotException::ERRCommunication,
			    "Error during robot initialization.");
  }

  return ;
}


/* ------------------------------------------------------------------------ */
/* --- DESTRUCTEUR -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

//! destructor
vpRobotAfma6::~vpRobotAfma6 (void)
{

  setRobotState(vpRobot::STATE_STOP) ;

  if (0 != vmeClose_Afma6())
  {
    ERROR_TRACE ("Error while closing communications with the robot.");
  }

  vpRobotAfma6::robotAlreadyCreated = FALSE;

  return;
}




/* Demarage du robot.
 * Change l'etat du robot, pour le placer en position arret, vitesse ou
 * position.
 * Les effets de bord sont:
 *   - si le robot est en arret, le changement est realise ssi le robot
 * est sous tension. Aucun effet de bord.
 *   - si le robot est en commande en position, le passage en arret stop
 * le robot a la position courrante, sans attendre la fin du deplacement.
 *   - si le robot est en commande en position, le passage en commande en
 * vitesse est impossible si le robot est en mouvement vers une nouvelle
 * position. Dans ce cas, une erreur ERRChangementEtat est lancee.
 *   - si le robot est en commande en vitesse, vitesse non nulle, un
 * changement d'etat arrete le robot (vitesse=0) a la position courrante.
 * INPUT:
 *   - newState: etat du robot apres l'appel a la fonction si aucune
 * erreur n'a ete lancee.
 * OUTPUT:
 *   - Retourne le precedent etat du robot.
 * ERROR:
 *   - ERRChangementEtat si le changement demande n'est pas possible.
 */
vpRobot::RobotStateType
vpRobotAfma6::setRobotState(vpRobot::RobotStateType newState)
{
  switch (newState)
  {
  case vpRobot::STATE_STOP:
    {
      if (vpRobot::STATE_STOP != getRobotState ())
      {
	 stop_mouvement_Afma6();
      }
      break;
    }
  case vpRobot::STATE_POSITION_CONTROL:
    {
      if (vpRobot::STATE_VELOCITY_CONTROL  == getRobotState ())
      {
	DEBUG_TRACE (12, "Passage vitesse -> position.");
	stop_mouvement_Afma6();
      }
      else
      {
	DEBUG_TRACE (1, "Passage arret -> position.");
      }
      break;
    }
  case vpRobot::STATE_VELOCITY_CONTROL:
    {
      if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ())
      {
	DEBUG_TRACE (10, "Robot en arret: demarage...");
	if (0 != init_mouvement_Afma6 ())
	{
	  ERROR_TRACE ("Cannot init velocity control.");
	  throw vpRobotException (vpRobotException::ERRLowLevel,
				  "Cannot init velocity control.");
	}
      }
      break;
    }
  default:
    break ;
  }

  return vpRobot::setRobotState (newState);
}


/* ------------------------------------------------------------------------ */
/* --- ARRET -------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* Arret du robot.
 * Envoye une commande pour arreter le robot a sa position actuelle, et
 * place l'objet en etat ETAT_ROBOT_ARRET.
 */
void
vpRobotAfma6::stopMotion(void)
{
  stop_mouvement_Afma6();
}


void
vpRobotAfma6::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  vpAfma6::get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

void
vpRobotAfma6::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpAfma6::get_cMe(cMe) ;
}

#define CTE_L -0.068826

//! get the robot Jacobian expressed in the end-effector frame
void
vpRobotAfma6::get_eJe(vpMatrix &eJe)
{

  vpColVector q(6) ;
  getPosition(vpRobot::ARTICULAR_FRAME, q) ;

  try
  {
    vpAfma6::get_eJe(q,eJe) ;
  }
  catch(...)
  {
    ERROR_TRACE("catch exception ") ;
    throw ;
  }
}
/*!
  \brief get the robot Jacobian expressed in the robot reference frame
  \warning this functionality is not implemented ont he Afma6
  \exception vpRobotException (vpRobotException::ERRNotImplemented)
*/

void
vpRobotAfma6::get_fJe(vpMatrix &fJe)
{

  vpColVector q(6) ;
  getPosition(vpRobot::ARTICULAR_FRAME, q) ;

  try
  {
    vpAfma6::get_fJe(q,fJe) ;
  }
  catch(...)
  {
    throw ;
  }

}



/*! Set the velocity for a positionning task

velocity in % of the maximum velocity between [0,100]
*/
void
vpRobotAfma6::setPositioningVelocity (const double velocity)
{
  positioningVelocity = velocity;
}
/*!
  Set the velocity for a positionning task
 */
double
vpRobotAfma6::getPositioningVelocity (void)
{
  return positioningVelocity;
}


/* Deplacement du robot en position.
 * Deplace le robot en position. Le robot se deplace jusqu'a avoir atteint
 * la position donnee en argument. La fonction est bloquante: elle rend la
 * main quand le deplacement est termine. Le robot doit etre dans l'etat
 * ETAT_ROBOT_COMMANDE_POSITION. Le repere de travail utilise est celui
 * donne par l'argument \a repere.
 * INPUT:
 *   - r: nouvelle position du robot apres l'appel a cette fonction.
 *   - repere: repere de travail utilise.
 * ATTENTION: Fonction bloquante.
 * ERROR:
 *   - ERRMauvaisEtatRobot si le robot n'est pas dans l'etat
 * ETAT_ROBOT_COMMANDE_POSITION.
 */
void
vpRobotAfma6::setPosition (const vpRobot::ControlFrameType frame,
			   const vpColVector & r )
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState ())
  {
    ERROR_TRACE ("Robot was not in position-based control\n"
		 "Modification of the robot state");
    setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  }

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME :
    {
      communicationPosition.repere = REPCAM;
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:

    {
      communicationPosition.repere = REPART;
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      communicationPosition.repere = REPFIX;
      break ;
    }
  }


  communicationPosition.mode = ABSOLU;

  vpRobotAfma6::VD6_mrad_mmrad (r, communicationPosition.pos);

  communicationPosition.vitesse = positioningVelocity;

  if (0 != positionnement_Afma6(& communicationPosition ))
  {
    ERROR_TRACE ("Positionning error.");
    throw vpRobotException (vpRobotException::ERRLowLevel,
			    "Positionning error.");
  }

  return ;
}


void vpRobotAfma6::setPosition (const vpRobot::ControlFrameType frame,
				const double x, const double y, const double z,
				const double rx, const double ry, const double rz
	     )
{
  try{
    vpColVector q(6) ;
    q[0] = x ;
    q[1] = y ;
    q[2] = z ;
    q[3] = rx ;
    q[4] = ry ;
    q[5] = rz ;

    setPosition(frame,q) ;
  }
  catch(...)
  {
    throw ;
  }
}


/* Recupere la position actuelle du robot.
 * Recupere la position actuelle du robot et place le resultat dans la
 * variable <r> donnee en argument. Le repere de travail dans lequel
 * est exprime le resultat est celui donne par l'argument \a repere.
 * OUTPUT:
 *   - r: reference dans laquelle est placee le resultat.
 * INPUT:
 *   - repere: repere de travail dans lequel est exprime le resultat.
 */
void
vpRobotAfma6::getPosition (const vpRobot::ControlFrameType frame,
			   vpColVector & r)
{
  DEBUG_TRACE (9, "# Entree.");

  switch (frame)
  {
  case vpRobot::CAMERA_FRAME :
    {
      r = 0;
      return ;
    }
  case vpRobot::ARTICULAR_FRAME :
    {
      communicationPosition.repere = REPART;
      break ;
    }
  case vpRobot::REFERENCE_FRAME :
    {
      communicationPosition.repere = REPFIX;
      break ;
    }
  }

  communicationPosition.mode=ABSOLU;
  if (0 != recup_posit_Afma6(& (communicationPosition) ) )
  {
    ERROR_TRACE ("Error when calling  recup_posit_Afma6.");
    throw vpRobotException (vpRobotException::ERRLowLevel,
			    "Error when calling  recup_posit_Afma6.");
  }

  r.resize (vpRobotAfma6::nbArticulations);
  vpRobotAfma6::DV6_mmrad_mrad (communicationPosition.pos, r);

}




void vpRobotAfma6::
DV6_mmrad_mrad(const double * input, vpColVector & output)
{
  output [0] = input [0] / 1000.0;
  output [1] = input [1] / 1000.0;
  output [2] = input [2] / 1000.0;
  output [3] = input [3];
  output [4] = input [4];
  output [5] = input [5];
}

void vpRobotAfma6::
VD6_mrad_mmrad (const vpColVector & input, double * output)
{
  output [0] = input [0] * 1000.0;
  output [1] = input [1] * 1000.0;
  output [2] = input [2] * 1000.0;
  output [3] = input [3];
  output [4] = input [4];
  output [5] = input [5];
}








/* Envoye une commande en vitesse au robot.
 *
 * Envoye une commande en vitesse au robot, exprime dans le repere
 * donne par l'argument \a repere. Le robot doit etre dans l'etat
 * ETAT_ROBOT_COMMANDE_VITESSE.
 * INPUT:
 *   - r_dot: vitesse envoyee au robot (mm/s et rad/s).
 *   - repere: repere de travail dans lequel est exprime le resultat.
 * ERROR:
 *   - ERRMauvaisEtatRobot si le robot n'est pas dans l'etat
 * ETAT_ROBOT_COMMANDE_VITESSE.
 */
void vpRobotAfma6::
setVelocity (const vpRobot::ControlFrameType frame,
	     const vpColVector & r_dot)
{

  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ())
  {
    ERROR_TRACE ("Cannot send a velocity to the robot "
		 "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException (vpRobotException::ERRWrongState,
			    "Cannot send a velocity to the robot "
			    "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME :
    {
      communicationVelocity.repere = REPCAM;
      break ;
    }
  case vpRobot::ARTICULAR_FRAME :
    {
      communicationVelocity.repere = REPART;
      break ;
    }
  case vpRobot::REFERENCE_FRAME :
    {
      communicationVelocity.repere = REPFIX;
      break ;
    }
  }

  for (int i = 0; i < 6; ++ i)
  {
    communicationVelocity.aserv[i] = VITESSE;
    communicationVelocity.mvt[i] = r_dot[i] ;
  }
  communicationVelocity.mvt[0] *= 1000.0 ;
  communicationVelocity.mvt[1] *= 1000.0 ;
  communicationVelocity.mvt[2] *= 1000.0 ;


  active_mouvement_Afma6(& (communicationVelocity) );
}



/* -------------------------------------------------------------------------- */
/* --- GET ------------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */


/* Recupere la vitesse actuelle du robot.
 * Recupere la vitesse actuelle du robot et place le resultat dans
 * la reference \a r_dot donnee en argument.
 * Le repere de travail dans lequel est exprime le resultat est celui
 * donne par l'argument \a repere
 * INPUT:
 *   - repere: repere de travail dans lequel est exprime le resultat.
 * OUTPUT:
 *   - r_dot: reference dans laquelle est placee le resultat (mm/s et rad/s).
 */
void vpRobotAfma6::
getVelocity (const vpRobot::ControlFrameType frame,
	     vpColVector & r_dot)
{

  long                 frameAfma6 = REPFIX;

  switch (frame)
  {
  case vpRobot::CAMERA_FRAME:
    {
      frameAfma6 = REPCAM;
      r_dot.resize (6);
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:
    {
      frameAfma6 = REPART;
      r_dot.resize (nbArticulations);
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      frameAfma6 = REPFIX;
      r_dot.resize (nbArticulations);
      break ;
    }
  }

  r_dot.resize(vpRobotAfma6::nbArticulations);

  mesure_vit_Afma6  	(frameAfma6, r_dot.data,
			 velocityMeasureTempo);

  vpRobotAfma6::V6_mmrad_mrad (r_dot);

}




/* Recupere la vitesse actuelle du robot.
 * Recupere la vitesse actuelle du robot et renvoie le resultat.
 * Le repere de travail dans lequel est exprime le resultat est celui
 * donne par l'argument \a repere
 * INPUT:
 *   - repere: repere de travail dans lequel est exprime le resultat.
 * OUTPUT:
 *   - Position actuelle du robot (mm/s et rad/s).
 */
vpColVector
vpRobotAfma6::getVelocity (vpRobot::ControlFrameType frame)
{
  vpColVector r_dot;
  getVelocity (frame, r_dot);

  return r_dot;
}

/* ------------------------------------------------------------------------ */
/* --- TEMPO -------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* Configure la duree d'attente pour la lecture de la vitesse
 * du robot.
 * La fonction bas niveau du robot mesure la vitesse du robot en
 * effectuant la difference entre deux positions. Le temps separant les
 * deux mesures est configure par la variable \a velocityMesureTempo. La
 * fonction #setVelocityMesureTempo permet de changer la valeur de cette
 * variable.
 * Par default, le temps d'attente est defaultVelocityMesureTempo.
 * INPUT:
 *  - tempo: nouvelle duree d'attente entre les deux mesures de
 * position de la fonction de lecteure de la vitesse (mm/s et rad/s).
 */
void
vpRobotAfma6::setVelocityMeasureTempo (const int tempo)
{
  velocityMeasureTempo = tempo;
}
/* Recupere la duree d'attente pour la lecture de la vitesse
 * du robot.
 * La fonction bas niveau du robot mesure la vitesse du robot en
 * effectuant la difference entre deux positions. Le temps separant les
 * deux mesures est configure par la variable \a velocityMeasureTempo. La
 * fonction #getVelocityMeasureTempo permet de lire la valeur de cette
 * variable.
 * Par default, le temps d'attente est defaultVelocityMeasureTempo.
 * OUTPUT:
 *   - Retourne la duree d'attente entre les deux mesures de
 * position de la fonction de lecteure de la vitesse (mm/s et rad/s).
 */
int vpRobotAfma6::
getVelocityMeasureTempo (void)
{
  return velocityMeasureTempo;
}


void vpRobotAfma6::
V6_mmrad_mrad (vpColVector & inoutput)
{
  inoutput [0] = inoutput [0] / 1000.0;
  inoutput [1] = inoutput [1] / 1000.0;
  inoutput [2] = inoutput [2] / 1000.0;
  inoutput [3] = inoutput [3] ;
  inoutput [4] = inoutput [4] ;
  inoutput [5] = inoutput [5] ;

  return ;
}

/* --- COPIES --------------------------------------------------------------- */

void vpRobotAfma6::
VD6_mdg_mrad (const vpColVector & input, double * output)
{
  output [0] = input [0];
  output [1] = input [1];
  output [2] = input [2];
  output [3] = input [3] / 180.0 * M_PI;
  output [4] = input [4] / 180.0 * M_PI;
  output [5] = input [5] / 180.0 * M_PI;

  return ;
}


/*
 * PROCEDURE: 	lit_pos
 *
 * ENTREE:
 * pt_fich	Pointeur de fichier a lire.
 *
 * SORTIE:
 * position	Positions sauvegardees du robot et de la camera.
 *
 * RESUME:
 * La procedure recupere les positions sauvegardees du robot dans le fichier
 * ayant pour pointeur de fichier "pt_fich".  La procedure retourne "OK" en cas
 * de succes, et "NO_AFMA6_POSITION" en cas d'echec.
 */

int
vpRobotAfma6::readPosFile(char *name, vpColVector &v)
//FILE *pt_fich, st_position_Afma6 *position)
{

  FILE * pt_fich ;
  pt_fich = fopen(name,"r") ;

  char line[FILENAME_MAX];
  char head[] = "R:";
  int  sortie = FALSE;

  do {
    // Saut des lignes commencant par #
    if (fgets (line, 100, pt_fich) != NULL) {
      if ( strncmp (line, "#", 1) != 0) {
	// La ligne n'est pas un commentaire
	if ( fscanf (pt_fich, "%s", line) != EOF)   {
	  if ( strcmp (line, head) == 0)
	    sortie = TRUE; 	// Position robot trouvee.
	}
	else
	  return (1); // fin fichier sans position robot.
      }
    }
    else {
      return (1);		/* fin fichier 	*/
    }

  }
  while ( sortie != TRUE );

  double x,y,z,rx,ry,rz ;
  // Lecture des positions
  fscanf(pt_fich, "%lf %lf %lf %lf %lf %lf",
	 &x, &y, &z,
	 &rx, &ry, &rz);
  v.resize(6) ;

  v[0] = x/1000 ;
  v[1] = y/1000 ;
  v[2] = z/1000 ;
  v[3] = vpMath::rad(rx) ;
  v[4] = vpMath::rad(ry) ;
  v[5] = vpMath::rad(rz) ;

  fclose(pt_fich) ;
  return (0);
}

void
vpRobotAfma6::move(char *name)
{
  vpColVector v ;
  readPosFile(name, v)  ;
  setPosition ( vpRobot::ARTICULAR_FRAME,  v) ;
}



void
vpRobotAfma6::openGripper()
{
  system("sudo /local/driver/pince/Afma6_pince_root -s 1") ;
}

void
vpRobotAfma6::closeGripper()
{
  system("sudo /local/driver/pince/Afma6_pince_root -s 0") ;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

