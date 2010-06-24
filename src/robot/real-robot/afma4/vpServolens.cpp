/****************************************************************************
 *
 * $Id$
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
 * Interface for the Servolens lens attached to the camera fixed on the 
 * Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpServolens.cpp

  Interface for the Servolens lens attached to the camera fixed on the 
  Afma4 robot.

*/

#ifdef UNIX

#include <termios.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <visp/vpServolens.h>
#include <visp/vpRobotException.h>
#include <visp/vpDebug.h>
#include <visp/vpTime.h>

/*!
  Default constructor. Does nothing.
  
  \sa open()

*/
vpServolens::vpServolens()
{
  isinit = false;
}

/*! 
  Open and initialize the Servolens serial link at 9600 bauds, 7
  data bits, even parity, 1 stop bit. The cariage return mode is not
  active, that means that each character is directly read without
  waitong for a cariage return.

  \sa open()
*/
vpServolens::vpServolens(const char *port)
{
  isinit = false;

  this->open(port);
}

/*!
  Destructor.

  Close the Servolens serial link.

  \sa close()
*/
vpServolens::~vpServolens()
{
  this->close();
}

/*!

  Open and initialize the Servolens serial link at 9600 bauds, 7
  data bits, even parity, 1 stop bit. The cariage return mode is not
  active, that means that each character is directly read without
  waitong for a cariage return.

  \param port : Serial device like /dev/ttyS0 or /dev/ttya.

  \exception vpRobotException::communicationError : If cannot open
  Servolens serial port or intialize the serial link.

  \sa close()
*/
void
vpServolens::open(const char *port)
{
  if (! isinit) {
    struct termios info;

    printf("\nOpen the Servolens serial port \"%s\"\n", port);

    if ((this->remfd=::open(port, O_RDWR|O_NONBLOCK)) < 0) {
      vpERROR_TRACE ("Cannot open Servolens serial port.");
      throw vpRobotException (vpRobotException::communicationError,
			      "Cannot open Servolens serial port.");
    }

    // Lecture des paramètres courants de la liaison série.
    if (tcgetattr(this->remfd, &info) < 0) {
      ::close(this->remfd);
      vpERROR_TRACE ("Error using TCGETS in ioctl.");
      throw vpRobotException (vpRobotException::communicationError,
			      "Error using TCGETS in ioctl");
    }

    //
    // Configuration de la liaison serie:
    // 9600 bauds, 1 bit de stop, parite paire, 7 bits de donnee
    //

    // Traitement sur les caractères recus
    info.c_iflag = 0;
    info.c_iflag |= INLCR;

    // Traitement sur les caractères envoyés sur la RS232.
    info.c_oflag = 0;  // idem

    // Traitement des lignes
    info.c_lflag = 0;

    // Controle materiel de la liaison
    info.c_cflag = 0;
    info.c_cflag |= CREAD;		// Validation reception
    info.c_cflag |= B9600 | CS7 | PARENB; // 9600 baus, 7 data, parite paire

    // Caractères immédiatement disponibles.
    //  info.c_cc[VMIN] = 1;
    //  info.c_cc[VTIME] = 0;

    if (tcsetattr(this->remfd, TCSANOW, &info) < 0) {
      ::close(this->remfd);
      vpERROR_TRACE ("Error using TCGETS in ioctl.");
      throw vpRobotException (vpRobotException::communicationError,
			      "Error using TCGETS in ioctl");
    }

    // Supprime tous les caracteres recus mais non encore lus par read()
    tcflush(this->remfd, TCIFLUSH);

    isinit = true;

    this->init();

    // Try to get the position of the zoom to check if the lens is really connected
    unsigned int izoom;
    if (this->getPosition(vpServolens::ZOOM, izoom) == false) {
      vpERROR_TRACE ("Cannot dial with the servolens. Check if the serial link is connected.");
      throw vpRobotException (vpRobotException::communicationError,
			      "Cannot dial with the servolens. Check if the serial link is connected.");

    }
   
  }
}

/*!
  Close the Servolens serial link.
  \sa open()
*/
void
vpServolens::close()
{
  if (isinit) {
    printf("\nClose the serial connexion with Servolens\n");
    ::close(this->remfd);
    isinit = false;
  }
}

/*!
  Reset the Servolens.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
void
vpServolens::reset()
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];

  /* suppression de l'echo */
  sprintf(commande, "SE1");
  this->write(commande);

  /* initialisation de l'objectif, idem qu'a la mise sous tension */
  sprintf(commande, "SR0");
  this->write(commande);

  vpTime::wait(25000);

  this->wait();

  /* suppression de l'echo */
  sprintf(commande, "SE0");
  this->write(commande);

  /* devalide l'incrustation de la fenetre sur l'ecran du moniteur */
  sprintf(commande, "VW0");
  this->write(commande);
}
/*!
  Initialize the Servolens lens.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

  \sa open()
*/
void
vpServolens::init()
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }

  char commande[10];

  /* suppression de l'echo */
  sprintf(commande, "SE0");
  this->write(commande);

  /* devalide l'incrustation de la fenetre sur l'ecran du moniteur */
  sprintf(commande, "VW0");
  this->write(commande);

  /* L'experience montre qu'une petite tempo est utile.		*/
  vpTime::wait(500);
}

/*!
  Set or remove the Servolens command complete status at the end of servoing.

  \param servo : Servolens servo motor.

  \param active : true to activate the emission of a command complete
  flag at the end of motion. false to disable this functionality.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
void 
vpServolens::enableCmdComplete(vpServoType servo, bool active)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];

  /* Envoie une commande pour qu'en fin de mouvement servolens renvoie
   * une information de fin de mouvement (ex: ZF, FF, DF).
   */
  switch(servo) {
  case ZOOM:
    if (active)
      sprintf(commande, "ZF1");
    else
      sprintf(commande, "ZF0");
    break;
  case FOCUS:
    if (active)
      sprintf(commande, "FF1");
    else
      sprintf(commande, "FF0");
    break;
  case IRIS:
    if (active)
      sprintf(commande, "DF1");
    else
      sprintf(commande, "DF0");
    break;
  }

  /* envoie de la commande */
  this->write(commande);   /* a la fin du mouvement envoie de ZF, FF, DF */
}

/*!
  Enable or disable the emission of the Servolens prompt "SERVOLENS>".

  \param active : true to activate the emission of the prompy. false
  to disable this functionality.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
void 
vpServolens::enablePrompt(bool active)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];

  /* suppression de l'echo */
  if (active == true)
    sprintf(commande, "SE1");
  else
    sprintf(commande, "SE0");
    
  this->write(commande);
}

/*!
  Set the controller type.

  \param controller : Controller type.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
void 
vpServolens::setController(vpControllerType controller)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];

  switch(controller) {
  case AUTO:
    /* Valide l'incrustation de la fenetre sur l'ecran du moniteur */
    sprintf(commande, "VW1");
    this->write(commande);
    break;
  case CONTROLLED:
    /* nettoyage : mot d'etat vide 0000 */
    sprintf(commande,"SX0842");
    this->write(commande);
    /* devalide l'incrustation de la fenetre sur l'ecran du moniteur */
    sprintf(commande, "VW0");
    this->write(commande);
    break;
  case RELEASED:
    sprintf(commande,"SX1084");
    this->write(commande);
    /* devalide l'incrustation de la fenetre sur l'ecran du moniteur */
    sprintf(commande, "VW0");
    this->write(commande);
    break;
  }

}

/*!
  Activates the auto iris mode of the Servolens.

  \param enable : true to activate the auto iris.

*/
void 
vpServolens::setAutoIris(bool enable)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];

  if (enable)
    sprintf(commande, "DA1");
  else
    sprintf(commande, "DA0");

  this->write(commande);
}

/*!
  Set the Servolens servo to the desired position.

  \param servo : Servolens servo motor to actuate.
  \param position : Desired position of the servo.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
void
vpServolens::setPosition(vpServoType servo, unsigned position)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];

  /* attente du prompt pour envoyer une commande */
  /*
  printf("attente prompt\n");
  this->wait();
  */

#if FINSERVO
  /* envoie des commandes pour qu'en fin de mouvement servolens renvoie */
  /* une commande de fin de mouvement (ex: ZF, FF, DF). */
  this->enableCommandComplete();
#endif	/* FINSERVO */

  // 08/08/00 Fabien S. - Correction de la consigne demandee
  // pour prendre en compte l'erreur entre la consigne demandée
  // et la consigne mesurée.
  // A la consigne du zoom on retranche 1.
  // A la consigne du focus on ajoute 1.
  // A la consigne du iris on ajoute 1.
  switch (servo) {
  case ZOOM:
    //printf("zoom demande: %d ", position);
    position --;
    if (position < ZOOM_MIN) position = ZOOM_MIN;
    //printf("zoom corrige: %d \n", position);
    break;
  case FOCUS:
    //printf("focus demande: %d ", position);
    position ++;
    if (position > FOCUS_MAX) position = FOCUS_MAX;
    //printf("focus corrige: %d \n", position);
    break;
  case IRIS:
    // printf("iris demande: %d ", position);
    position ++;
    if (position > IRIS_MAX) position = IRIS_MAX;
    //printf("iris corrige: %s \n", position);
    break;
  }

  /* commande a envoyer aux servomoteurs */
  switch(servo) {
    case ZOOM:
      sprintf(commande, "ZD%d", position);
      break;
    case FOCUS:
      sprintf(commande, "FD%d", position);
      break;
    case IRIS:
      sprintf(commande, "DD%d", position);
      break;
    }
  /* envoie de la commande */
#if PRINT
  printf("\ncommande: %s", commande);
#endif

  this->write(commande);

#if FINSERVO
  /* on attend la fin du mouvement des objectifs */
  this->wait(servo);  /* on attend les codes ZF, FF, DF */
#endif
}

/*!
  Get the Servolens current servo position.

  \param servo : Servolens servo motor to actuate.
  \param position : Measured position of the servo.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
bool
vpServolens::getPosition(vpServoType servo, unsigned &position)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
  char commande[10];
  char posit[10], *pt_posit;
  char c;
  short fin_lect_posit; /* indique si on a lu la position du servo-moteur */
  short posit_car;      /* donne la position du caractere lu */
  short lecture_posit_en_cours; /* indique si on lit la position courante */

  /* attente du prompt pour envoyer une commande */
  /*
  this->wait();
  */
  pt_posit = posit;

  /* envoie des commandes pour obtenir la position des servo-moteurs. */
  switch (servo) {
  case ZOOM:
    sprintf(commande, "ZD?");
    break;
  case FOCUS:
    sprintf(commande, "FD?");
    break;
  case IRIS:
    sprintf(commande, "DD?");
    break;
  default:
    break;
  }
  /* envoie de la commande */
  //    printf("\ncommande: %s", commande);

  this->write(commande);

  /* on cherche a lire la position du servo-moteur */
  /* Servolens renvoie une chaine de caractere du type ZD00400 ou FD00234 */
  fin_lect_posit = 0;
  posit_car = 0;
  lecture_posit_en_cours = 0;
  do {
    if (this->read(&c, 1) == true) {

      //    printf("caractere lu: %c\n", c);
      switch (posit_car){
	/* on lit le 1er caractere; (soit Z, soit F, soit D) */
      case 0:
	/* sauvegarde du pointeur */
	pt_posit = posit;

	switch (servo) {
	case ZOOM:
	  if( c == 'Z') posit_car = 1;
	  break;
	case FOCUS:
	  if( c == 'F') posit_car = 1;
	  break;
	case IRIS:
	  if( c == 'D') posit_car = 1;
	  break;
	}
	break;

	/* si le 1er caractere est correct, on lit le 2eme caractere */
	/* (toujours D) */
      case 1:
	if( c == 'D') posit_car = 2;
	else posit_car = 0; /* le 2eme caractere n'est pas correct */
	break;

	/* si on a lu les 2 premiers caracteres, on peut lire la */
	/* position du servo-moteur */
      case 2:
	if (c >= '0' && c <= '9')
	{
	  *pt_posit++ = c;        /* sauvegarde de la position */
	  lecture_posit_en_cours = 1;
	}
	else if (lecture_posit_en_cours)
	{
	  fin_lect_posit = 1;
	  *pt_posit = '\0';
	}
	else posit_car = 0;
	break;
      }

    }
    else {
      // Timout sur la lecture, on retoure FALSE
      return false;
    }
  }
  while ( !fin_lect_posit );

  //    printf("\nChaine lue: posit: %s", posit);

  /* toilettage de la position courantes lue */
  this->clean(posit, posit);

  //    printf("\nChaine toilettee: posit: %s", posit);
  position = atoi(posit);

  return(true);
}

/*!
  Waits for the Servolens promt '>'.

  \return The prompt character.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
char 
vpServolens::wait()
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }
    
  char c;
  ::write(this->remfd, "\r\n", strlen("\r\n"));
  do {
    ::read(this->remfd, &c, 1);
    c &= 0x7f;
  }
  while (c != '>');
  return c;

}

/*!

  Waits the end of motion of the corresponding servo motor.

  \param servo : Servolens servo motor.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
void
vpServolens::wait(vpServoType servo)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }

  char c;
  char fin_mvt[3];
  bool sortie = false;

  switch (servo) {
  case ZOOM:
    sprintf(fin_mvt, "ZF");
    break;
  case FOCUS:
    sprintf(fin_mvt, "FF");
    break;
  case IRIS:
  default:
    sprintf(fin_mvt, "DF");
    break;
    
  }

  /* lecture des caracteres recus */
  do {
    /* lecture des caracteres */
    ::read(this->remfd,&c,1);
    c &= 0x7f;

    /* tests si fin de mouvement */
    if (c == fin_mvt[0]) {
      /* lecture du caractere suivant */
      ::read(this->remfd,&c,1);
      c &= 0x7f;
      if (c == fin_mvt[1]) {
	sortie = true;
      }
    }
  }
  while ( !sortie);

  /*  printf("\nmouvement fini: chaine lue = %s", chaine); */
}

/*!
  Read one character form the serial link.

  \param c : The character that was read.
  \param timeout_s : Timeout in seconds.
  
  \return true if the reading was successfully achieved, false otherwise.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
bool 
vpServolens::read(char *c, long timeout_s)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }

  int n;
  fd_set         readfds; /* list of fds for select to listen to */
  struct timeval timeout = {timeout_s, 0}; // seconde, micro-sec

  FD_ZERO(&readfds); FD_SET(this->remfd, &readfds);

  if (select(FD_SETSIZE, &readfds, (fd_set *)NULL,
	     (fd_set *)NULL, &timeout) > 0) {
    n = ::read(this->remfd, c, 1); /* read one character at a time */
    *c &= 0x7f;
    //printf("lecture 1 car: %c\n", *c);
    return(true);
  }

  return (false);
}

/*!
  Write a string to the serial link.

  \param s : String to send to the Servolens.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
void 
vpServolens::write(const char *s)
{
  if (!isinit) {
    vpERROR_TRACE ("Cannot dial with Servolens.");
    throw vpRobotException (vpRobotException::communicationError,
			    "Cannot dial with Servolens.");
  }

  ::write(this->remfd,"\r", strlen("\r"));

  ::write(this->remfd, s, strlen(s));
  ::write(this->remfd,"\r", strlen("\r"));

  /*
   * Une petite tempo pour laisser le temps a la liaison serie de
   * digerer la commande envoyee. En fait, la liaison serie fonctionne
   * a 9600 bauds soit une transmission d'environ 9600 bits pas seconde.
   * Les plus longues commandes envoyees sur la liaison serie sont du type:
   * SX0842 soit 6 caracteres codes sur 8 bits chacuns = 48 bits pour
   * envoyer la commande SX0842.
   * Ainsi, le temps necessaire pour envoyer SX0842 est d'environ
   * 48 / 9600 = 0,0050 secondes = 5 milli secondes.
   * Ici on rajoute une marge pour amener la tempo a 20 ms.
   */
  vpTime::wait(20);
}

/*!
  Suppress all the zero characters on the left hand of \e in string.

  \param in : Input string.
  \param out : Output string without zero characters on the left.

*/
bool 
vpServolens::clean(const char *in, char *out)
{
  short nb_car, i=0;
  bool error = false;

  nb_car = strlen(in);

  /* on se positionne sur le 1er caractere different de zero */
  while( *(in) == '0' && i++ < nb_car ) {
    in++;
    if (i == nb_car)
      {
	error = true; /* la chaine ne contient pas une position */
	*(out++) = '0'; /* mise a zero de la position */
      }
  }

  /* copie de la position epuree des zeros de gauche */
  while( i++ <= nb_car ) { /* on a mis le = pour copier le caractere de fin */
    /* de chaine \0 */
    *(out++) = *(in++);
  }
  return (error);
}

#endif
