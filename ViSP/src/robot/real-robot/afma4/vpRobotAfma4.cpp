/****************************************************************************
 *
 * $Id: vpRobotAfma4.cpp,v 1.7 2007-04-18 16:14:28 asaunier Exp $
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

/* Headers des fonctions implementees. */
#include <visp/vpRobotAfma4Contrib.h>    /* Communication avec le robot. */
#include <visp/vpRobotAfma4.h>           /* Header de la classe.          */
#include <visp/vpRobotException.h>/* Erreurs lancees par les classes CRobot. */

#include <visp/vpDebug.h>           /* Macros de trace et debug.  */
// Fonctions AFMA4 bas niveau


#include <signal.h>

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

bool vpRobotAfma4::robotAlreadyCreated = false;
const double       vpRobotAfma4::defaultPositioningVelocity = 10.0;
const int          vpRobotAfma4::nbArticulations = 4;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */



void emergencyStopAfma4(int signo)
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
  stop_mouvement_Afma4() ;
  vmeClose_Afma4();
  close_rs232_servolens();

  cout << "exit(1)" <<endl ;
  exit(1) ;
}


/*! constructor
 */
vpRobotAfma4::vpRobotAfma4 (void)
  :
  vpAfma4 (),
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

  signal(SIGINT, emergencyStopAfma4);
  signal(SIGBUS, emergencyStopAfma4) ;
  signal(SIGSEGV, emergencyStopAfma4) ;
  signal(SIGKILL, emergencyStopAfma4);
  signal(SIGQUIT, emergencyStopAfma4);

  vpDEBUG_TRACE (12, "Open communication with VME.");
  init();

  vpDEBUG_TRACE (12, "Read Config parameters.");
  vpAfma4::init ();

  try
  {
    setRobotState(vpRobot::STATE_STOP) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  positioningVelocity = defaultPositioningVelocity ;
  return ;
}



/* -------------------------------------------------------------------------- */
/* --- INITIALISATION ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Initialise les connexions avec la carte VME et demare le robot.
 * ERROR:
 *   - ERRConstruction si une erreur survient lors de l'ouverture du VME.
 *   - ERRCommunication si une erreur survient lors de l'initialisation de
 * l'afma4.
 */
void
vpRobotAfma4::init (void)
{

  vpDEBUG_TRACE (12, "Open connection to the VME.");
  if (0 != vmeOpenA32D32_Afma4())
  {
    vpERROR_TRACE ("Cannot open connexion between PC and VME.");
    throw vpRobotException (vpRobotException::constructionError,
			    "Cannot open connexion between PC and VME");
  }

  if (0 != initialisation_Afma4())
  {
    vpERROR_TRACE ("Error during robot initialization.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Error during robot initialization.");
  }

  open_rs232_servolens(SERVOLENS_SERIAL_PORT) ;
  //supprime les 4 carres de l'image
  init_servolens() ;

  commande_servolens(SERVOLENS_ZOOM,"1000") ;
  mode_DA0_servolens() ;


  return ;
}


/* ------------------------------------------------------------------------ */
/* --- DESTRUCTEUR -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

//! destructor
vpRobotAfma4::~vpRobotAfma4 (void)
{

  setRobotState(vpRobot::STATE_STOP) ;

  if (0 != vmeClose_Afma4())
  {
    vpERROR_TRACE ("Error while closing communications with the robot.");
  }
  close_rs232_servolens();

  vpRobotAfma4::robotAlreadyCreated = FALSE;

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
vpRobotAfma4::setRobotState(vpRobot::RobotStateType newState)
{
  switch (newState)
  {
  case vpRobot::STATE_STOP:
    {
      if (vpRobot::STATE_STOP != getRobotState ())
      {
	stop_mouvement_Afma4();
      }
      break;
    }
  case vpRobot::STATE_POSITION_CONTROL:
    {
      if (vpRobot::STATE_VELOCITY_CONTROL  == getRobotState ())
      {
	vpDEBUG_TRACE (12, "Passage vitesse -> position.");
	stop_mouvement_Afma4();
      }
      else
      {
	vpDEBUG_TRACE (1, "Passage arret -> position.");
      }
      break;
    }
  case vpRobot::STATE_VELOCITY_CONTROL:
    {
      if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ())
      {
	vpDEBUG_TRACE (10, "Robot en arret: demarage...");
	if (0 != init_mouvement_Afma4 ())
	{
	  vpERROR_TRACE ("Cannot init velocity control.");
	  throw vpRobotException (vpRobotException::lowLevelError,
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
vpRobotAfma4::stopMotion(void)
{
  stop_mouvement_Afma4();
  setRobotState (vpRobot::STATE_STOP);
}


void
vpRobotAfma4::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  vpAfma4::get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

void
vpRobotAfma4::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpAfma4::get_cMe(cMe) ;
}


//! get the robot Jacobian expressed in the end-effector frame
void
vpRobotAfma4::get_eJe(vpMatrix &eJe)
{

  vpColVector q(6) ;
  getPosition(vpRobot::ARTICULAR_FRAME, q) ;

  try
  {
    vpAfma4::get_eJe(q,eJe) ;
  }
  catch(...)
  {
    vpERROR_TRACE("catch exception") ;
    throw ;
  }
}
/*!
  \brief get the robot Jacobian expressed in the robot reference frame
  \warning this functionality is not implemented ont he Afma4
  \exception vpRobotException (vpRobotException::ERRNotImplemented)
*/

void
vpRobotAfma4::get_fJe(vpMatrix &fJe)
{

  vpColVector q(4) ;
  getPosition(vpRobot::ARTICULAR_FRAME, q) ;

  try
  {
    vpAfma4::get_fJe(q,fJe) ;
  }
  catch(...)
  {
  	vpERROR_TRACE("Error caught")
    throw ;
  }

}



/*! Set the velocity for a positionning task

velocity in % of the maximum velocity between [0,100]
*/
void
vpRobotAfma4::setPositioningVelocity (const double velocity)
{
  positioningVelocity = velocity;
}
/*!
  Set the velocity for a positionning task
*/
double
vpRobotAfma4::getPositioningVelocity (void)
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
 *   - repere: repere de travail utilise. Suel le repere articulaire
 *             est disponible
 * ATTENTION: Fonction bloquante.
 * ERROR:
 *   - ERRMauvaisEtatRobot si le robot n'est pas dans l'etat
 * ETAT_ROBOT_COMMANDE_POSITION.
 */
void
vpRobotAfma4::setPosition (const vpRobot::ControlFrameType frame,
			   const vpColVector & r )
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState ())
  {
    vpERROR_TRACE ("Robot was not in position-based control\n"
		 "Modification of the robot state");
    setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  }

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME:
    vpERROR_TRACE ("Cannot move the robot in camera frame: not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot move the robot in camera frame: "
			    "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    vpERROR_TRACE ("Cannot move the robot in reference frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot move the robot in reference frame: "
			    "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    vpERROR_TRACE ("Cannot move the robot in mixt frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot move the robot in mixt frame: "
			    "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break ;
  }

  communicationPosition.mode = ABSOLU;

  vpRobotAfma4::VD4_mrad_mmrad (r, communicationPosition.pos);

  communicationPosition.vitesse = positioningVelocity;

  if (0 != positionnement_Afma4(& communicationPosition ))
  {
    vpERROR_TRACE ("Positionning error.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Positionning error.");
  }

  return ;
}


void vpRobotAfma4::setPosition (const vpRobot::ControlFrameType frame,
				const double q1, const double q2,
				const double q3, const double q4 )
{
  try{
    vpColVector q(4) ;
    q[0] = q1 ;
    q[1] = q2 ;
    q[2] = q3 ;
    q[3] = q4 ;

    setPosition(frame,q) ;
  }
  catch(...)
  {
  	vpERROR_TRACE("Error caught")
    throw ;
  }
}


/* Recupere la position articulaire actuelle du robot.
 * Recupere la position actuelle du robot et place le resultat dans la
 * variable <r> donnee en argument. Le repere de travail dans lequel
 * est exprime le resultat est celui donne par l'argument \a repere.
 * OUTPUT:
 *   - r: reference dans laquelle est placee le resultat.
 * INPUT:
 *   - repere: repere de travail dans lequel est exprime le resultat.
 Seul le repere articulaire est disponible

 \warning Functionnality only available in articular frame, with
 the axis order: rotation arround the vertical axis, vertical
 translation, pan and tilt of the camera
*/
void
vpRobotAfma4::getPosition (const vpRobot::ControlFrameType frame,
			   vpColVector & r)
{
  vpDEBUG_TRACE (9, "# Entree.");

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME :
    vpERROR_TRACE ("Cannot get position in camera frame: not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in camera frame: "
			    "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    vpERROR_TRACE ("Cannot get position in reference frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in reference frame: "
			    "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    vpERROR_TRACE ("Cannot get position in mixt frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in mixt frame: "
			    "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break ;
  }

  communicationPosition.mode=ABSOLU;
  if (0 != recup_posit_Afma4(& (communicationPosition) ) )
  {
    vpERROR_TRACE ("Error when calling  recup_posit_Afma4.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Error when calling  recup_posit_Afma4.");
  }

  r.resize (vpRobotAfma4::nbArticulations);
  vpRobotAfma4::VD4_mmrad_mrad (communicationPosition.pos, r);

}


void
vpRobotAfma4::setZoom(int zoom)
{
  char cha[10] ;
  sprintf(cha,"%d",zoom) ;
  commande_servolens(SERVOLENS_ZOOM, cha);
  wait_fin_mvt_servolens(SERVOLENS_ZOOM) ;
}

int
vpRobotAfma4::getZoom()
{
  char cha[10] ;
  lecture_posit_servolens(SERVOLENS_ZOOM, cha);
  return atoi(cha) ;
}

void
vpRobotAfma4::setIris(int iris)
{
  char cha[10] ;
  sprintf(cha,"%d", iris) ;
  commande_servolens(SERVOLENS_IRIS, cha);
  wait_fin_mvt_servolens(SERVOLENS_IRIS) ;
}
int
vpRobotAfma4::getIris()
{
  char cha[10] ;
  lecture_posit_servolens(SERVOLENS_IRIS, cha);
  return atoi(cha) ;
}
void
vpRobotAfma4::setAutoIris(bool on)
{
  if (on == true)
    mode_DA1_servolens();
  else
    mode_DA0_servolens();
}

void
vpRobotAfma4::setFocus(int focus)
{
  char cha[10] ;
  sprintf(cha,"%d",focus) ;
  commande_servolens(SERVOLENS_FOCUS, cha);
  while (!wait_fin_mvt_servolens(SERVOLENS_FOCUS)) ;
}

int
vpRobotAfma4::getFocus()
{
  char cha[10] ;//mettre_fin_mvt_servolens(SERVOLENS_FOCUS) ;
  lecture_posit_servolens(SERVOLENS_FOCUS, cha);
  return atoi(cha) ;
}


void
vpRobotAfma4::VD4_mmrad_mrad(const double * input, vpColVector & output)
{
  output [0] = input [0];
  output [1] = input [1] / 1000.0;
  output [2] = input [2];
  output [3] = input [3];
}

void
vpRobotAfma4::VD4_mrad_mmrad (const vpColVector & input, double * output)
{
  output [0] = input [0];
  output [1] = input [1] * 1000.0;
  output [2] = input [2];
  output [3] = input [3];
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

 \param frame Speed control frame type. Be aware, the REFERENCE_FRAME and
 MIXT_FRAME are not implemented

 \warning In CAMERA_FRAME, we control only the rx and ry camera velocities;
 r_dot dimension must be two: r_dot[0] correspond to rx, and r_dot[1] to ry

 \waning In ARTICULAR_FRAME, we control the 4 dof, r_dot dimension is
 4. r_dot[0] corresponds to the turret rotation (in radians), r_dot[1] to the
 vertical translation (in meters), r_dot[2] to the pan of the camera (in
 radians) and r_dot[3] to the tilt of the camera (in radians)
*/
void
vpRobotAfma4::setVelocity (const vpRobot::ControlFrameType frame,
			   const vpColVector & r_dot)
{

  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ())
  {
    vpERROR_TRACE ("Cannot send a velocity to the robot "
		 "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException (vpRobotException::wrongStateError,
			    "Cannot send a velocity to the robot "
			    "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME :
    {
      communicationVelocity.repere = REPCAM;
      if ( r_dot.getRows() != 2) {
	vpERROR_TRACE ("Bad dimension fo speed vector in camera frame");
	throw vpRobotException (vpRobotException::wrongStateError,
				"Bad dimension for speed vector "
				"in camera frame");
      }
      break ;
    }
  case vpRobot::ARTICULAR_FRAME :
    {
      communicationVelocity.repere = REPART;
      if ( r_dot.getRows() != 4) {
	vpERROR_TRACE ("Bad dimension fo speed vector in articular frame");
	throw vpRobotException (vpRobotException::wrongStateError,
				"Bad dimension for speed vector "
				"in articular frame");
      }
      break ;
    }
  case vpRobot::REFERENCE_FRAME :
    {
      communicationVelocity.repere = REPFIX;
      vpERROR_TRACE ("Cannot send a velocity to the robot "
		   "in the reference frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot send a velocity to the robot "
			      "in the reference frame:"
			      "functionality not implemented");
      break ;
    }
  case vpRobot::MIXT_FRAME :
    {
      vpERROR_TRACE ("Cannot send a velocity to the robot "
		   "in the mixt frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot send a velocity to the robot "
			      "in the mixt frame:"
			      "functionality not implemented");
      break ;
    }
  default:
    {
      vpERROR_TRACE ("Error in spec of vpRobot. "
		   "Case not taken in account.");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot send a velocity to the robot ");
    }
  }

  for (int i = 0; i < 6; ++ i)
  {
    communicationVelocity.asserv[i] = VITESSE;
    communicationVelocity.mvt[i] = 0.0;
  }

  vpDEBUG_TRACE (12, "Velocity limitation.");
  vpColVector v(6);

  switch(frame) {
  case vpRobot::CAMERA_FRAME : {
    double max = this ->maxRotationVelocity;
    for (int i = 0 ; i < 2; ++ i) // rx and ry of the camera
    {
      if (fabs (r_dot[i]) > max)
      {
	max = fabs (r_dot[i]);
	vpERROR_TRACE ("Excess velocity: ROTATION "
		     "(axe nr.%d).", i);
      }
    }
    max =  this ->maxRotationVelocity / max;
    for (int i = 0 ; i < 2; ++ i)
    { v [i] = r_dot[i]*max; }

    for (int i = 0; i < 2; ++ i)
    {
      communicationVelocity.asserv[i] = VITESSE;
      communicationVelocity.mvt[i] = v[i];
    }

    break;
  }
  case vpRobot::ARTICULAR_FRAME : {
    double max = this ->maxRotationVelocity;
    if (fabs (r_dot[0]) > max) // turret rotation
    {
      max = fabs (r_dot[0]);
      vpERROR_TRACE ("Excess velocity: ROTATION "
		   "(axe nr.%d).", 0);
    }
    for (int i = 2 ; i < 4; ++ i) // pan and tilt
    {
      if (fabs (r_dot[i]) > max)
      {
	max = fabs (r_dot[i]);
	vpERROR_TRACE ("Excess velocity: ROTATION "
		     "(axe nr.%d).", i);
      }
    }
    max =  this ->maxRotationVelocity / max;
    v [0] = r_dot[0]*max;
    v [2] = r_dot[2]*max;
    v [3] = r_dot[3]*max;

    max = this ->maxTranslationVelocity;
    if (fabs (r_dot[1]) > max)
    {
      max = fabs (r_dot[1]);
      vpERROR_TRACE ("Excess velocity: TRANSLATION "
		   "(axe nr.%d).", 1);
    }
    v [1] = r_dot[1]*max;

    for (int i = 0; i < 4; ++ i)
    {
      communicationVelocity.asserv[i] = VITESSE;
      communicationVelocity.mvt[i] = v[i];
    }
    communicationVelocity.mvt[1] *= 1000.0 ;

    break;
  }
  default:
    // Should never occur
    break;

  }

  vpCDEBUG(12) << "v: " << v.t() << endl;
  active_mouvement_Afma4(& (communicationVelocity) );
  return;
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
void
vpRobotAfma4::getVelocity (const vpRobot::ControlFrameType frame,
			   vpColVector & r_dot)
{

  long                 frameAfma4 = REPFIX;

  switch (frame)
  {
  case vpRobot::CAMERA_FRAME:
    {
      frameAfma4 = REPCAM;
      r_dot.resize (6);
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:
    {
      frameAfma4 = REPART;
      r_dot.resize (nbArticulations);
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      frameAfma4 = REPFIX;
      r_dot.resize (nbArticulations);
      break ;
    }
  case vpRobot::MIXT_FRAME:
    {

      vpERROR_TRACE ("Cannot get a velocity in the mixt frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot get a velocity in the reference frame:"
			      "functionality not implemented");
      break ;
    }
  }

  r_dot.resize(vpRobotAfma4::nbArticulations);

  mesure_vit_Afma4  	(frameAfma4, r_dot.data);

  vpRobotAfma4::VD4_mmrad_mrad (r_dot);

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
vpRobotAfma4::getVelocity (vpRobot::ControlFrameType frame)
{
  vpColVector r_dot;
  getVelocity (frame, r_dot);

  return r_dot;
}


void
vpRobotAfma4::VD4_mmrad_mrad (vpColVector & inoutput)
{
  inoutput [0] = inoutput [0];
  inoutput [1] = inoutput [1] / 1000.0;
  inoutput [2] = inoutput [2];
  inoutput [3] = inoutput [3] ;

  return ;
}

/* --- COPIES --------------------------------------------------------------- */

void
vpRobotAfma4::VD4_mdg_mrad (const vpColVector & input, double * output)
{
  output [0] = input [0] / 180.0 * M_PI;
  output [1] = input [1];
  output [2] = input [2] / 180.0 * M_PI;
  output [3] = input [3] / 180.0 * M_PI;

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
 * de succes, et "NO_AFMA4_POSITION" en cas d'echec.

 \return 0 if succes, 1 if an error occurs.
*/

int
vpRobotAfma4::readPosFile(char *name, vpColVector &v)
  //FILE *pt_fich, st_position_Afma4 *position)
{

  FILE * pt_fich ;
  pt_fich = fopen(name,"r") ;

  if (pt_fich == NULL) {
    vpERROR_TRACE ("Can not open file %s", name);
    return 1;
  }

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

  double q1,q2,q3,q4;
  // Lecture des positions
  fscanf(pt_fich, "%lf %lf %lf %lf",
	 &q1, &q2, &q3, &q4);
  v.resize(nbArticulations) ;

  v[0] = vpMath::rad(q1) ; // Rot tourelle
  v[1] = q2/1000.0 ;
  v[2] = vpMath::rad(q3) ;
  v[3] = vpMath::rad(q4) ;

  fclose(pt_fich) ;
  return (0);
}

/*!

  Get the robot displacement expressed in the camera frame since the last call
  of this method.

  \param v The measured displacement in camera frame. The dimension of v is 6
  (tx, ty, ty, rx, ry, rz). Translations are expressed in meters, rotations in
  radians.

  \sa getDisplacement(), getArticularDisplacement()

*/
void
vpRobotAfma4::getCameraDisplacement(vpColVector &v)
{
  getDisplacement(vpRobot::CAMERA_FRAME, v);

}
/*!

  Get the robot articular displacement since the last call of this method.

  \param qdot The measured articular displacement. The dimension of qdot is 4
  (the number of axis of the robot) with respectively qdot[1] (turret
  rotation), qdot[2] (vertical translation), qdot[3] (pan), qdot[4]
  (tilt). Translations are expressed in meters, rotations in radians.

  \sa getDisplacement(), getCameraDisplacement()

*/
void vpRobotAfma4::getArticularDisplacement(vpColVector  &qdot)
{
  getDisplacement(vpRobot::ARTICULAR_FRAME, qdot);
}

/*!

  Get the robot displacement since the last call of this method.

  \param frame The frame in which the measured displacement is expressed.

  \param q The displacement.
  . In articular, the dimension of q is 4 (the number of axis of the robot)
  with respectively q[1] (turret rotation), q[2] (vertical translation), q[3]
  (pan), q[4] (tilt).
  . In camera or reference frame, the dimension of q is 6 (tx, ty, ty, rx, ry,
  rz). Translations are expressed in meters, rotations in radians.

  \sa getArticularDisplacement(), getCameraDisplacement()

*/
void
vpRobotAfma4::getDisplacement(vpRobot::ControlFrameType frame,
			      vpColVector &q)
{
  double td[6];
  switch (frame)
  {
  case vpRobot::CAMERA_FRAME:
    {
      q.resize (6);
      mesure_dpl_Afma4(REPCAM,td);
      q[0]=td[0]/1000.0; // values are returned in mm
      q[1]=td[1]/1000.0;
      q[2]=td[2]/1000.0;
      q[3]=td[3];
      q[4]=td[4];
      q[5]=td[5];
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:
    {
      mesure_dpl_Afma4(REPART,td);
      q.resize (nbArticulations);
      q[0]=td[0];  // turret rotation
      q[1]=td[1]/1000.0;// vertical translation returned in mm
      q[2]=td[2]; // pan
      q[3]=td[3]; // tilt
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      mesure_dpl_Afma4(REPFIX,td);
      q.resize (6);
      q[0]=td[0]/1000.0; // values are returned in mm
      q[1]=td[1]/1000.0;
      q[2]=td[2]/1000.0;
      q[3]=td[3];
      q[4]=td[4];
      q[5]=td[5];
      break ;
    }
  case vpRobot::MIXT_FRAME:
    {
      vpERROR_TRACE ("Cannot get a displacement in the mixt frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot get a displacement in the reference frame:"
			      "functionality not implemented");

      break ;
    }
  }
}

void
vpRobotAfma4::move(char *name)
{
  vpColVector v ;
  readPosFile(name, v)  ;
  setPosition ( vpRobot::ARTICULAR_FRAME,  v) ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

