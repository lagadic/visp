/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRobotPtu46.cpp
 * Project:   Visp2
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: vpRobotPtu46.cpp,v 1.2 2006-02-13 09:27:54 fspindle Exp $
 *
 * Description
 * ============

 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <visp/vpConfig.h>
#ifdef HAVE_ROBOT_PTUEVI

/* Headers des fonctions implementees. */
#include <visp/vpPtu46.h>           /* Header de la classe.          */
#include <visp/vpRobotPtu46.h>           /* Header de la classe.          */
#include <visp/vpRobotException.h>/* Erreurs lancees par les classes CRobot. */

#include <visp/vpDebug.h>           /* Macros de trace et debug.  */


#include <signal.h>

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

bool vpRobotPtu46::robotAlreadyCreated = false;
const double       vpRobotPtu46::defaultPositioningVelocity = 10.0;
const int          vpRobotPtu46::nbArticulations = 2;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */


/*! constructor
 */
vpRobotPtu46::vpRobotPtu46 (void)
  :
  vpRobot ()
{

  DEBUG_TRACE (12, "Open communication with Ptu-46.");
  init();

  try
  {
    setRobotState(vpRobot::STATE_STOP) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
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
vpRobotPtu46::init (void)
{

  DEBUG_TRACE (12, "Open connection Ptu-46.");
  if (0 != ptu.init("/dev/ttyS0") )
  {
    ERROR_TRACE ("Cannot open connexion with ptu-46.");
    throw vpRobotException (vpRobotException::constructionError,
			    "Cannot open connexion with ptu-46");
  }

  return ;
}


/* ------------------------------------------------------------------------ */
/* --- DESTRUCTEUR -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

//! destructor
vpRobotPtu46::~vpRobotPtu46 (void)
{

  setRobotState(vpRobot::STATE_STOP) ;

  if (0 != ptu.close())
  {
    ERROR_TRACE ("Error while closing communications with the robot ptu-46.");
  }

  vpRobotPtu46::robotAlreadyCreated = false;

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
vpRobotPtu46::setRobotState(vpRobot::RobotStateType newState)
{
  switch (newState)
  {
  case vpRobot::STATE_STOP:
    {
      if (vpRobot::STATE_STOP != getRobotState ())
      {
	ptu.stop();
      }
      break;
    }
  case vpRobot::STATE_POSITION_CONTROL:
    {
      if (vpRobot::STATE_VELOCITY_CONTROL  == getRobotState ())
      {
	DEBUG_TRACE (12, "Passage vitesse -> position.");
	ptu.stop();
      }
      else
      {
	DEBUG_TRACE (1, "Passage arret -> position.");
      }
      break;
    }
  case vpRobot::STATE_VELOCITY_CONTROL:
    {
      if (vpRobot::STATE_POSITION_CONTROL != getRobotState ())
      {
	DEBUG_TRACE (10, "Arret du robot...");
	ptu.stop();
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
vpRobotPtu46::stopMotion(void)
{
  ptu.stop();
  setRobotState (vpRobot::STATE_STOP);
}


void
vpRobotPtu46::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  vpPtu46::get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

void
vpRobotPtu46::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpPtu46::get_cMe(cMe) ;
}


//! get the robot Jacobian expressed in the end-effector frame
void
vpRobotPtu46::get_eJe(vpMatrix &eJe)
{
  vpColVector q(2) ;
  getPosition(vpRobot::ARTICULAR_FRAME, q) ;

  try
  {
    vpPtu46::get_eJe(q,eJe) ;
  }
  catch(...)
  {
    ERROR_TRACE("catch exception ") ;
    throw ;
  }
}
/*!
  \brief get the robot Jacobian expressed in the robot reference frame
  \warning this functionality is not implemented ont he Afma4
  \exception vpRobotException (vpRobotException::ERRNotImplemented)
*/

void
vpRobotPtu46::get_fJe(vpMatrix &fJe)
{
  vpColVector q(2) ;
  getPosition(vpRobot::ARTICULAR_FRAME, q) ;

  try
  {
    vpPtu46::get_fJe(q,fJe) ;
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
vpRobotPtu46::setPositioningVelocity (const double velocity)
{
  positioningVelocity = velocity;
}
/*!
  Set the velocity for a positionning task
*/
double
vpRobotPtu46::getPositioningVelocity (void)
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
vpRobotPtu46::setPosition (const vpRobot::ControlFrameType frame,
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
  case vpRobot::CAMERA_FRAME:
    ERROR_TRACE ("Cannot move the robot in camera frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot move the robot in camera frame: "
			    "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    ERROR_TRACE ("Cannot move the robot in reference frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot move the robot in reference frame: "
			    "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    ERROR_TRACE ("Cannot move the robot in mixt frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot move the robot in mixt frame: "
			    "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break ;
  }

  // Interface for the controller
  double artpos[2];

  artpos[0] = r[0];
  artpos[1] = r[1];

  if (0 != ptu.move(artpos, positioningVelocity, PTU_ABSOLUTE_MODE) )
  {
    ERROR_TRACE ("Positionning error.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Positionning error.");
  }

  return ;
}


void vpRobotPtu46::setPosition (const vpRobot::ControlFrameType frame,
				const double q1, const double q2)
{
  try{
    vpColVector q(2) ;
    q[0] = q1 ;
    q[1] = q2 ;

    setPosition(frame,q) ;
  }
  catch(...)
  {
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
vpRobotPtu46::getPosition (const vpRobot::ControlFrameType frame,
			   vpColVector & r)
{
  DEBUG_TRACE (9, "# Entree.");

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME :
    ERROR_TRACE ("Cannot get position in camera frame: not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in camera frame: "
			    "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    ERROR_TRACE ("Cannot get position in reference frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in reference frame: "
			    "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    ERROR_TRACE ("Cannot get position in mixt frame: "
		 "not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in mixt frame: "
			    "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break ;
  }

  double artpos[2];

  if (0 != ptu.getCurrentPosition( artpos ) )
  {
    ERROR_TRACE ("Error when calling  recup_posit_Afma4.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Error when calling  recup_posit_Afma4.");
  }

  r.resize (vpRobotPtu46::nbArticulations);

  r[0] = artpos[0];
  r[1] = artpos[1];
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
vpRobotPtu46::setVelocity (const vpRobot::ControlFrameType frame,
			   const vpColVector & r_dot)
{
  TPtuFrame ptuFrameInterface;

  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ())
  {
    ERROR_TRACE ("Cannot send a velocity to the robot "
		 "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException (vpRobotException::wrongStateError,
			    "Cannot send a velocity to the robot "
			    "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  switch(frame)
  {
  case vpRobot::CAMERA_FRAME :
    {
      ptuFrameInterface = PTU_CAMERA_FRAME;
      if ( r_dot.getRows() != 2) {
	ERROR_TRACE ("Bad dimension fo speed vector in camera frame");
	throw vpRobotException (vpRobotException::wrongStateError,
				"Bad dimension for speed vector "
				"in camera frame");
      }
      break ;
    }
  case vpRobot::ARTICULAR_FRAME :
    {
      ptuFrameInterface = PTU_ARTICULAR_FRAME;
      if ( r_dot.getRows() != 2) {
	ERROR_TRACE ("Bad dimension fo speed vector in articular frame");
	throw vpRobotException (vpRobotException::wrongStateError,
				"Bad dimension for speed vector "
				"in articular frame");
      }
      break ;
    }
  case vpRobot::REFERENCE_FRAME :
    {
      ERROR_TRACE ("Cannot send a velocity to the robot "
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
      ERROR_TRACE ("Cannot send a velocity to the robot "
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
      ERROR_TRACE ("Error in spec of vpRobot. "
		   "Case not taken in account.");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot send a velocity to the robot ");
    }
  }

  DEBUG_TRACE (12, "Velocity limitation.");
  double ptuSpeedInterface[2];

  switch(frame) {
  case vpRobot::ARTICULAR_FRAME :
  case vpRobot::CAMERA_FRAME : {
    double max = this ->maxRotationVelocity;
    for (int i = 0 ; i < 2; ++ i) // rx and ry of the camera
    {
      if (fabs (r_dot[i]) > max)
      {
	max = fabs (r_dot[i]);
	ERROR_TRACE ("Excess velocity: ROTATION "
		     "(axe nr.%d).", i);
      }
    }
    max =  this ->maxRotationVelocity / max;
    for (int i = 0 ; i < 2; ++ i)
     ptuSpeedInterface [i] = r_dot[i]*max;

    break;
  }
  default:
    // Should never occur
    break;

  }

  CDEBUG(12) << "v: " << ptuSpeedInterface[0]
	     << " " << ptuSpeedInterface[1] << endl;
  ptu.move(ptuSpeedInterface, ptuFrameInterface);
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
vpRobotPtu46::getVelocity (const vpRobot::ControlFrameType frame,
			   vpColVector & r_dot)
{

  TPtuFrame ptuFrameInterface = PTU_ARTICULAR_FRAME;

  switch (frame)
  {
  case vpRobot::CAMERA_FRAME:
    {
      ptuFrameInterface = PTU_CAMERA_FRAME;
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:
    {
      ptuFrameInterface = PTU_ARTICULAR_FRAME;
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      ERROR_TRACE ("Cannot get a velocity in the reference frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot get a velocity in the reference frame:"
			      "functionality not implemented");
      break ;
    }
  case vpRobot::MIXT_FRAME:
    {

      ERROR_TRACE ("Cannot get a velocity in the mixt frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot get a velocity in the mixt frame:"
			      "functionality not implemented");
      break ;
    }
  }

  r_dot.resize(vpRobotPtu46::nbArticulations);
  double ptuSpeedInterface[2];

  ptu.getCurrentSpeed(ptuSpeedInterface, ptuFrameInterface);

  r_dot[0] = ptuSpeedInterface[0];
  r_dot[1] = ptuSpeedInterface[1];

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
vpRobotPtu46::getVelocity (vpRobot::ControlFrameType frame)
{
  vpColVector r_dot;
  getVelocity (frame, r_dot);

  return r_dot;
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
vpRobotPtu46::readPosFile(char *name, vpColVector &v)
  //FILE *pt_fich, st_position_Afma4 *position)
{

  FILE * pt_fich ;
  pt_fich = fopen(name,"r") ;

  if (pt_fich == NULL) {
    ERROR_TRACE ("Can not open file %s", name);
    return 1;
  }

  char line[FILENAME_MAX];
  char head[] = "R:";
  bool  sortie = false;

  do {
    // Saut des lignes commencant par #
    if (fgets (line, 100, pt_fich) != NULL) {
      if ( strncmp (line, "#", 1) != 0) {
	// La ligne n'est pas un commentaire
	if ( fscanf (pt_fich, "%s", line) != EOF)   {
	  if ( strcmp (line, head) == 0)
	    sortie = true; 	// Position robot trouvee.
	}
	else
	  return (1); // fin fichier sans position robot.
      }
    }
    else {
      return (1);		/* fin fichier 	*/
    }

  }
  while ( sortie != true );

  double q1,q2;
  // Lecture des positions
  fscanf(pt_fich, "%lf %lf", &q1, &q2);
  v.resize(nbArticulations) ;

  v[0] = vpMath::rad(q1) ; // Rot tourelle
  v[1] = vpMath::rad(q2) ;

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
vpRobotPtu46::getCameraDisplacement(vpColVector &v)
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
void vpRobotPtu46::getArticularDisplacement(vpColVector  &qdot)
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
vpRobotPtu46::getDisplacement(vpRobot::ControlFrameType frame,
			      vpColVector &q)
{
  double d[6];

  switch (frame)
  {
  case vpRobot::CAMERA_FRAME:
    {
      q.resize (6);
      ptu.measureDpl(d, PTU_CAMERA_FRAME);
      q[0]=d[0];
      q[1]=d[1];
      q[2]=d[2];
      q[3]=d[3];
      q[4]=d[4];
      q[5]=d[5];
      break ;
    }
  case vpRobot::ARTICULAR_FRAME:
    {
      ptu.measureDpl(d, PTU_ARTICULAR_FRAME);
      q.resize (nbArticulations);
      q[0]=d[0];  // pan
      q[1]=d[1];  // tilt
      break ;
    }
  case vpRobot::REFERENCE_FRAME:
    {
      ERROR_TRACE ("Cannot get a displacement in the reference frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot get a displacement in the reference frame:"
			      "functionality not implemented");
      break ;
    }
  case vpRobot::MIXT_FRAME:
    {
      ERROR_TRACE ("Cannot get a displacement in the mixt frame: "
		   "functionality not implemented");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot get a displacement in the reference frame:"
			      "functionality not implemented");

      break ;
    }
  }
}

void
vpRobotPtu46::move(char *name)
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

