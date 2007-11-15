/****************************************************************************
 *
 * $Id: vpRobotAfma6.cpp,v 1.20 2007-11-15 14:47:29 fspindle Exp $
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

/* Headers des fonctions implementees. */
#include <visp/vpRobotAfma6.h>           /* Header de la classe.          */
#include <visp/vpRobotException.h>/* Erreurs lancees par les classes CRobot. */
#include <visp/vpDebug.h>           /* Macros de trace et debug.  */

#include <signal.h>

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

bool vpRobotAfma6::robotAlreadyCreated = false;
const double       vpRobotAfma6::defaultPositioningVelocity = 20.0;
const int          vpRobotAfma6::defaultVelocityMeasureTempo = 10;
const int          vpRobotAfma6::nbArticulations = 6;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */



void emergencyStop(int signo)
{
  std::cout << "Arret de la tache par : SIGNAL= " << (char)7  ;
  switch(signo)
  {
  case  SIGINT:
    std::cout << "SIGINT (arret par ^C) " << std::endl ; break ;
  case SIGSEGV:
    std::cout <<"SIGSEGV (arret par segmentation fault) " << std::endl ; break ;
  case SIGBUS:
    std::cout <<"SIGBUS (arret par bus error) " << std::endl ; break ;
  default :
    std::cout << signo << std::endl ;
  }
  stop_mouvement_Afma6() ;
  vmeClose_Afma6();

  std::cout << "exit(1)" <<std::endl ;
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

  vpDEBUG_TRACE (12, "Open communication with VME.");
  this->init();

  try
  {
    setRobotState(vpRobot::STATE_STOP) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  positioningVelocity =  defaultPositioningVelocity ;
  return ;
}


/* -------------------------------------------------------------------------- */
/* --- INITIALISATION ------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Initialise les connexions avec la carte VME et demarre le robot.
 * ERROR:
 *   - ERRConstruction si une erreur survient lors de l'ouverture du VME.
 *   - ERRCommunication si une erreur survient lors de l'initialisation de
 * l'afma6.
 */
void
vpRobotAfma6::init (void)
{

  vpDEBUG_TRACE (12, "Open connection to the VME.");
  if (0 != vmeOpenA32D32_Afma6())
  {
    vpERROR_TRACE ("Cannot open connexion between PC and VME.");
    throw vpRobotException (vpRobotException::constructionError,
			    "Cannot open connexion between PC and VME");
  }

  if (0 != initialisation_Afma6())
  {
    vpERROR_TRACE ("Error during robot initialization.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Error during robot initialization.");
  }
  // Charge sur le VME la matrice de passage pince-image correspondant
  // a la camera par defaut
  init(vpAfma6::defaultCameraRobot);

  return ;
}

/*!

  Charge la matrice-pince camera associee à la camera

*/
void
vpRobotAfma6::init (vpAfma6::CameraRobotType camera)
{
  ECameraAfma6 api_camera; // Interface with low level Afma6 api

  switch (camera)
  {
  case vpAfma6::CAMERA_XC77_12MM:
    api_camera = CAMERA_XC77;
    break;
  case vpAfma6::CAMERA_HF_8MM:
    api_camera = CAMERA_HF;
    break;
  case vpAfma6::CAMERA_IEEE1394_12MM:
    api_camera = CAMERA_IEEE1394;
    break;
  default:
    {
      api_camera = CAMERA_DEFAULT;
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


  update_mpi_Afma6(api_camera);


  vpDEBUG_TRACE (12, "Read Config parameters from the VME.");

  // The constant values on the VME
  STCONST_AFMA6 vme_constants;	/* Value of the joint limits */
  recup_all_const_Afma6(&vme_constants);

  // Update internal constant values
  for (int i=0; i < vpAfma6::articulationsNb; i ++) {
    this->Kp[i] = vme_constants.Kp[i];
    this->Kd[i] = vme_constants.Kd[i];
    this->Ki[i] = vme_constants.Ki[i];
    this->QMax[i] = vme_constants.QMax[i];
    this->QMin[i] = vme_constants.QMin[i];
    this->top[i] = vme_constants.top[i];
    this->RstQm[i] = vme_constants.RstQm[i];
    this->SensDep[i] = vme_constants.SensDep[i];
    this->EpsMax[i] = vme_constants.EpsMax[i];
    this->TiMax[i] = vme_constants.TiMax[i];
    this->AccMax[i] = vme_constants.AccMax[i];
    this->VitMax[i] = vme_constants.VitMax[i];
    this->ErrTMax[i] = vme_constants.ErrTMax[i];
  }
  this->l = vme_constants.l;
  this->coupl = vme_constants.coupl;
  this->FlagMod = vme_constants.FlagMod;
  this->FlagReset = vme_constants.FlagReset;
  for (int i=0; i < 3; i ++) {
    this->rrpi[i] = vme_constants.teta[i] * M_PI / 180.0;
    this->trpi[i] = vme_constants.trpi[i];
  }
  vpDEBUG_TRACE (15, "Compute homogeneous RPI matrix.");

  vpRotationMatrix Rrpi ;
  Rrpi.buildFrom(this->rrpi) ;

  rpi.insert(Rrpi) ;
  rpi.insert(trpi) ;

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
    vpERROR_TRACE ("Error while closing communications with the robot.");
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
	vpDEBUG_TRACE (12, "Passage vitesse -> position.");
	stop_mouvement_Afma6();
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
	if (0 != init_mouvement_Afma6 ())
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
vpRobotAfma6::stopMotion(void)
{
  stop_mouvement_Afma6();
  setRobotState (vpRobot::STATE_STOP);
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
    vpERROR_TRACE("catch exception ") ;
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
    vpERROR_TRACE("Error caught");
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
    vpERROR_TRACE ("Robot was not in position-based control\n"
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
  case vpRobot::MIXT_FRAME:
    {
      vpERROR_TRACE ("Positionning error. Mixt frame not implemented");
      throw vpRobotException (vpRobotException::lowLevelError,
			      "Positionning error: "
			      "Mixt frame not implemented.");
     break ;
    }
  }


  communicationPosition.mode = ABSOLU;

  vpRobotAfma6::VD6_mrad_mmrad (r, communicationPosition.pos);

  communicationPosition.vitesse = positioningVelocity;

  if (0 != positionnement_Afma6(& communicationPosition ))
  {
    vpERROR_TRACE ("Positionning error.");
    throw vpRobotException (vpRobotException::lowLevelError,
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
    vpERROR_TRACE("Error caught");
    throw ;
  }
}


/*!

  Get the current position of the robot.

  \param frame : Control frame type in which to get the position, either :
  - in the camera cartesien frame,
  - articular coordinates of each axes
  - in a reference or fixed cartesien frame attached to the robot base
  - in a mixt cartesien frame (not sure but probably translation in reference
    frame, and rotation in camera frame)

  \param r : Measured position of the robot:
  - in camera cartesien frame, a 6 dimension vector, set to 0,

  - in articular, a 6 dimension vector corresponding to the articular
    position of each dof, first the 3 translations, then the 3
    articular rotation positions.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
    the translation tx, ty, tz in meters (like a vpTranslationVector), and the
    last 3 values to the rx, ry, rz rotation (like a vpRxyzVector). The code
    below show how to convert this position into a vpHomogenousMatrix:

    \code
    vpRobotAfma6 robot;
    vpColVector r;
    robot.getPosition(vpRobot::REFERENCE_FRAME, r);
    vpTranslationVector rtc; // reference frame to camera frame translations
    vpRxyzVector rrc; // reference frame to camera frame rotations

    // Update the transformation between reference frame and camera frame
    for (int i=0; i < 3; i++) {
      rtc[i] = r[i];   // tx, ty, tz
      rrc[i] = r[i+3]; // ry, ry, rz
    }

    // Create a rotation matrix from the Rxyz rotation angles
    vpRotationMatrix rRc(rrc); // reference frame to camera frame rotation matrix

    // Create the camera to fix frame pose in terms of a homogenous matrix
    vpHomogeneousMatrix fMc(rRc, rtc);

    \endcode


  \sa setPosition(const vpRobot::ControlFrameType frame, const vpColVector & r)

*/
void
vpRobotAfma6::getPosition (const vpRobot::ControlFrameType frame,
			   vpColVector & r)
{
  vpDEBUG_TRACE (9, "# Entree.");

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
  case vpRobot::MIXT_FRAME:
    {
      vpERROR_TRACE ("Cannot get position in mixt frame: not implemented");
      throw vpRobotException (vpRobotException::lowLevelError,
			      "Cannot get position in mixt frame: "
			      "not implemented");
     break ;
    }
  }

  communicationPosition.mode=ABSOLU;
  if (0 != recup_posit_Afma6(& (communicationPosition) ) )
  {
    vpERROR_TRACE ("Error when calling  recup_posit_Afma6.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Error when calling  recup_posit_Afma6.");
  }

  r.resize (vpRobotAfma6::nbArticulations);
  vpRobotAfma6::DV6_mmrad_mrad (communicationPosition.pos, r);

}




void
vpRobotAfma6::DV6_mmrad_mrad(const double * input, vpColVector & output)
{
  output [0] = input [0] / 1000.0;
  output [1] = input [1] / 1000.0;
  output [2] = input [2] / 1000.0;
  output [3] = input [3];
  output [4] = input [4];
  output [5] = input [5];
}

void
vpRobotAfma6::VD6_mrad_mmrad (const vpColVector & input, double * output)
{
  output [0] = input [0] * 1000.0;
  output [1] = input [1] * 1000.0;
  output [2] = input [2] * 1000.0;
  output [3] = input [3];
  output [4] = input [4];
  output [5] = input [5];
}

/*!
  Apply a velocity to the robot.

  \param frame : Control frame in which the velocity is expressed. Velocities
  could be expressed in articular, camera frame, reference frame or mixt frame.

  \param r_dot : Velocity vector \f$ \dot {r} \f$. For translation speed \f$v,
  \dot{q}_1, \dot{q}_2, \dot{q}_3 \f$, units are m/s, for rotations speed \f$
  \omega, \dot{q}_4, \dot{q}_5, \dot{q}_6 \f$ rad/s. The size of this vector is
  always 6.

  - In articular, \f$ \dot {r} = [\dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4,
    \dot{q}_5, \dot{q}_6]^t \f$.

  - In camera frame, \f$ \dot {r} = [^{c} v_x, ^{c} v_y, ^{c} v_z, ^{c}
    \omega_x, ^{c} \omega_y, ^{c} \omega_z]^t \f$.

  - In reference frame, \f$ \dot {r} = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{r}
    \omega_x, ^{r} \omega_y, ^{r} \omega_z]^t \f$.

  - In mixt frame, \f$ \dot {r} = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{c} \omega_x,
    ^{c} \omega_y, ^{c} \omega_z]^t \f$.  In mixt frame, translations \f$ v_x,
    v_y, v_z \f$ are expressed in the reference frame and rotations \f$
    \omega_x, \omega_y, \omega_z \f$ in the camera frame.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \warning Velocities could be saturated if one of them exceed the maximal
  autorized speed (see vpRobot::maxTranslationVelocity and
  vpRobot::maxRotationVelocity).

 */
void
vpRobotAfma6::setVelocity (const vpRobot::ControlFrameType frame,
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
  case vpRobot::MIXT_FRAME :
    {
      communicationVelocity.repere = REPMIX;
      break ;
    }
  default:
    {
      vpERROR_TRACE ("Error in spec of vpRobot. "
		     "Case not taken in account.");
    }
  }


  vpDEBUG_TRACE (12, "Velocity limitation.");
  bool norm = false; // Flag to indicate when velocities need to be nomalized
  double max = this ->maxTranslationVelocity;
  vpColVector v(6);
  for (int i = 0 ; i < 3; ++ i) {
    if (fabs (r_dot[i]) > max) {
      norm = true;
      max = fabs (r_dot[i]);
      vpERROR_TRACE ("Excess velocity %g: TRANSLATION "
		     "(axe nr.%d).", r_dot[i], i);
    }
  }

  // Translations velocities normalisation
  if (norm == true)  {
    max =  this ->maxTranslationVelocity / max;
    for (int i = 0 ; i < 6; ++ i)
    { v [i] = r_dot[i]*max; }
  }
  else {
    for (int i = 0 ; i < 6; ++ i) {
      v [i] = r_dot[i];
    }
  }

  max = this ->maxRotationVelocity;
  for (int i = 3 ; i < 6; ++ i) {
    if (fabs (r_dot[i]) > max) {
      norm = true;
      max = fabs (r_dot[i]);
      vpERROR_TRACE ("Excess velocity %g: ROTATION "
		     "(axe nr.%d).", r_dot[i], i);
    }
  }
  // Rotations velocities normalisation
  if (norm == true) {
    max =  this ->maxRotationVelocity / max;
    for (int i = 3 ; i < 6; ++ i)
    { v [i] = r_dot[i]*max; }
  }

  for (int i = 0; i < 6; ++ i) {
    communicationVelocity.aserv[i] = VITESSE;
    communicationVelocity.mvt[i] = v[i] ;
  }
  communicationVelocity.mvt[0] *= 1000.0 ;
  communicationVelocity.mvt[1] *= 1000.0 ;
  communicationVelocity.mvt[2] *= 1000.0 ;


  active_mouvement_Afma6(& (communicationVelocity) );

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
vpRobotAfma6::getVelocity (const vpRobot::ControlFrameType frame,
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
  case vpRobot::MIXT_FRAME:
    {
      frameAfma6 = REPMIX;
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
int
vpRobotAfma6::getVelocityMeasureTempo (void)
{
  return velocityMeasureTempo;
}


void
vpRobotAfma6::V6_mmrad_mrad (vpColVector & inoutput)
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

void
vpRobotAfma6::VD6_mdg_mrad (const vpColVector & input, double * output)
{
  output [0] = input [0];
  output [1] = input [1];
  output [2] = input [2];
  output [3] = input [3] / 180.0 * M_PI;
  output [4] = input [4] / 180.0 * M_PI;
  output [5] = input [5] / 180.0 * M_PI;

  return ;
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
vpRobotAfma6::getCameraDisplacement(vpColVector &v)
{
  getDisplacement(vpRobot::CAMERA_FRAME, v);
}
/*!

  Get the robot articular displacement since the last call of this method.

  \param qdot The measured articular displacement. The dimension of qdot is 6
  (the number of axis of the robot). Translations are expressed in meters,
  rotations in radians.

  \sa getDisplacement(), getCameraDisplacement()

*/
void vpRobotAfma6::getArticularDisplacement(vpColVector  &qdot)
{
  getDisplacement(vpRobot::ARTICULAR_FRAME, qdot);
}

/*!

  Get the robot displacement since the last call of this method.

  \param frame The frame in which the measured displacement is expressed.

  \param q The displacement.
  . In articular, the dimension of q is 6.
  . In camera or reference frame, the dimension of q is 6 (tx, ty, ty, rx, ry,
  rz). Translations are expressed in meters, rotations in radians.

  \sa getArticularDisplacement(), getCameraDisplacement()

*/
void
vpRobotAfma6::getDisplacement(vpRobot::ControlFrameType frame,
			      vpColVector &q)
{
  double td[6];

  q.resize (6);
  switch (frame)
  {
  case vpRobot::CAMERA_FRAME:
    {
      mesure_dpl_Afma6(REPCAM,td);
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:
    {
      mesure_dpl_Afma6(REPART,td);
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      mesure_dpl_Afma6(REPFIX,td);
      break ;
    }
  case vpRobot::MIXT_FRAME:
    {
      mesure_dpl_Afma6(REPMIX,td);
      break ;
    }
  }
  q[0]=td[0]/1000.0; // values are returned in mm
  q[1]=td[1]/1000.0;
  q[2]=td[2]/1000.0;
  q[3]=td[3];
  q[4]=td[4];
  q[5]=td[5];
}


/*!

  Read articular positions in a specific Afma6 file.

  \param filename : Name of the position file to read.
  \param v : articular positions

  \return true if the positions were successfully readen in the file. false, if
  an error occurs.
*/

bool
vpRobotAfma6::readPosFile(const char *filename, vpColVector &v)
{

  FILE * fd ;
  fd = fopen(filename, "r") ;
  if (fd == NULL)
    return false;

  char line[FILENAME_MAX];
  char head[] = "R:";
  bool  sortie = false;

  do {
    // Saut des lignes commencant par #
    if (fgets (line, 100, fd) != NULL) {
      if ( strncmp (line, "#", 1) != 0) {
	// La ligne n'est pas un commentaire
	if ( fscanf (fd, "%s", line) != EOF)   {
	  if ( strcmp (line, head) == 0)
	    sortie = true; 	// Position robot trouvee.
	}
	else
	  return (false); // fin fichier sans position robot.
      }
    }
    else {
      return (false);		/* fin fichier 	*/
    }

  }
  while ( sortie != true );

  double x,y,z,rx,ry,rz ;
  // Lecture des positions
  fscanf(fd, "%lf %lf %lf %lf %lf %lf",
	 &x, &y, &z,
	 &rx, &ry, &rz);
  v.resize(6) ;

  v[0] = x/1000 ;
  v[1] = y/1000 ;
  v[2] = z/1000 ;
  v[3] = vpMath::rad(rx) ;
  v[4] = vpMath::rad(ry) ;
  v[5] = vpMath::rad(rz) ;

  fclose(fd) ;
  return (true);
}
/*!

  Save articular positions in a specific Afma6 file.

  \param filename : Name of the position file to create.
  \param v : articular positions

  \return true if the positions were successfully saved in the file. false, if
  an error occurs.
*/

bool
vpRobotAfma6::savePosFile(const char *filename, const vpColVector &v)
{

  FILE * fd ;
  fd = fopen(filename, "w") ;
  if (fd == NULL)
    return false;

  fprintf(fd, "\
#AFMA6 - Position - Version 2.01\n\
#\n\
# Septembre 2000 - Fabien SPINDLER\n\
# Fichier de sauvegarde des positions du robot cartésien.\n\
#\n\
# Ordre des axes:\n\
# R: axes du robot cartésien\n\
#     - translation sur x\n\
#     - translation sur y\n\
#     - translation sur z\n\
#     - rotation autour de x\n\
#     - rotation autour de y\n\
#     - rotation autour de z\n\
#\n");

  // Save positions in mm and deg
  fprintf(fd, "R: %lf %lf %lf %lf %lf %lf\n",
	  v[0]*1000, v[1]*1000, v[2]*1000,
	  vpMath::deg(v[3]), vpMath::deg(v[4]), vpMath::deg(v[5]));

  fclose(fd) ;
  return (true);
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
  system("rsh japet sudo /local/driver/pince/Afma6_pince_root -s 1") ;
}

void
vpRobotAfma6::closeGripper()
{
  system("rsh japet sudo /local/driver/pince/Afma6_pince_root -s 0") ;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

