/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpServolens.h>

/*!
  Default constructor. Does nothing.

  \sa open()

*/
vpServolens::vpServolens() : remfd(0), isinit(false) {}

/*!
  Open and initialize the Servolens serial link at 9600 bauds, 7
  data bits, even parity, 1 stop bit. The cariage return mode is not
  active, that means that each character is directly read without
  waitong for a cariage return.

  \sa open()
*/
vpServolens::vpServolens(const char *port) : remfd(0), isinit(false) { this->open(port); }

/*!
  Destructor.

  Close the Servolens serial link.

  \sa close()
*/
vpServolens::~vpServolens() { this->close(); }

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
void vpServolens::open(const char *port)
{
  if (!isinit) {
    struct termios info;

    printf("\nOpen the Servolens serial port \"%s\"\n", port);

    if ((this->remfd = ::open(port, O_RDWR | O_NONBLOCK)) < 0) {
      vpERROR_TRACE("Cannot open Servolens serial port.");
      throw vpRobotException(vpRobotException::communicationError, "Cannot open Servolens serial port.");
    }

    // Lecture des parametres courants de la liaison serie.
    if (tcgetattr(this->remfd, &info) < 0) {
      ::close(this->remfd);
      vpERROR_TRACE("Error using TCGETS in ioctl.");
      throw vpRobotException(vpRobotException::communicationError, "Error using TCGETS in ioctl");
    }

    //
    // Configuration de la liaison serie:
    // 9600 bauds, 1 bit de stop, parite paire, 7 bits de donnee
    //

    // Traitement sur les caracteres recus
    info.c_iflag = 0;
    info.c_iflag |= INLCR;

    // Traitement sur les caracteres envoyes sur la RS232.
    info.c_oflag = 0; // idem

    // Traitement des lignes
    info.c_lflag = 0;

    // Controle materiel de la liaison
    info.c_cflag = 0;
    info.c_cflag |= CREAD;                // Validation reception
    info.c_cflag |= B9600 | CS7 | PARENB; // 9600 baus, 7 data, parite paire

    // Caracteres immediatement disponibles.
    //  info.c_cc[VMIN] = 1;
    //  info.c_cc[VTIME] = 0;

    if (tcsetattr(this->remfd, TCSANOW, &info) < 0) {
      ::close(this->remfd);
      vpERROR_TRACE("Error using TCGETS in ioctl.");
      throw vpRobotException(vpRobotException::communicationError, "Error using TCGETS in ioctl");
    }

    // Supprime tous les caracteres recus mais non encore lus par read()
    tcflush(this->remfd, TCIFLUSH);

    isinit = true;

    this->init();

    // Try to get the position of the zoom to check if the lens is really
    // connected
    unsigned int izoom;
    if (this->getPosition(vpServolens::ZOOM, izoom) == false) {
      vpERROR_TRACE("Cannot dial with the servolens. Check if the serial "
                    "link is connected.");
      throw vpRobotException(vpRobotException::communicationError, "Cannot dial with the servolens. Check if the "
                                                                   "serial link is connected.");
    }
  }
}

/*!
  Close the Servolens serial link.
  \sa open()
*/
void vpServolens::close()
{
  if (isinit) {
    printf("\nClose the serial connection with Servolens\n");
    ::close(this->remfd);
    isinit = false;
  }
}

/*!
  Reset the Servolens.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
void vpServolens::reset() const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
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
void vpServolens::init() const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
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
  Enable or disable the emission of the Servolens prompt "SERVOLENS>".

  \param active : true to activate the emission of the prompy. false
  to disable this functionality.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
void vpServolens::enablePrompt(bool active) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
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
void vpServolens::setController(vpControllerType controller) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }
  char commande[10];

  switch (controller) {
  case AUTO:
    /* Valide l'incrustation de la fenetre sur l'ecran du moniteur */
    sprintf(commande, "VW1");
    this->write(commande);
    break;
  case CONTROLLED:
    /* nettoyage : mot d'etat vide 0000 */
    sprintf(commande, "SX0842");
    this->write(commande);
    /* devalide l'incrustation de la fenetre sur l'ecran du moniteur */
    sprintf(commande, "VW0");
    this->write(commande);
    break;
  case RELEASED:
    sprintf(commande, "SX1084");
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
void vpServolens::setAutoIris(bool enable) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
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
void vpServolens::setPosition(vpServoType servo, unsigned int position) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }
  std::stringstream command;

/* attente du prompt pour envoyer une commande */
/*
printf("attente prompt\n");
this->wait();
*/

#ifdef FINSERVO
  /* envoie des commandes pour qu'en fin de mouvement servolens renvoie */
  /* une commande de fin de mouvement (ex: ZF, FF, DF). */
  this->enableCommandComplete();
#endif /* FINSERVO */

  // 08/08/00 Fabien S. - Correction de la consigne demandee
  // pour prendre en compte l'erreur entre la consigne demandee
  // et la consigne mesuree.
  // A la consigne du zoom on retranche 1.
  // A la consigne du focus on ajoute 1.
  // A la consigne du iris on ajoute 1.
  switch (servo) {
  case ZOOM:
    // printf("zoom demande: %d ", position);
    position--;
    if (position < ZOOM_MIN)
      position = ZOOM_MIN;
    // printf("zoom corrige: %d \n", position);
    break;
  case FOCUS:
    // printf("focus demande: %d ", position);
    position++;
    if (position > FOCUS_MAX)
      position = FOCUS_MAX;
    // printf("focus corrige: %d \n", position);
    break;
  case IRIS:
    // printf("iris demande: %d ", position);
    position++;
    if (position > IRIS_MAX)
      position = IRIS_MAX;
    // printf("iris corrige: %s \n", position);
    break;
  }

  /* commande a envoyer aux servomoteurs */
  switch (servo) {
  case ZOOM:
    command << "ZD" << position;
    break;
  case FOCUS:
    command << "FD" << position;
    break;
  case IRIS:
    command << "DD" << position;
    break;
  }
/* envoie de la commande */
#ifdef PRINT
  printf("\ncommand: %s", command.str());
#endif

  this->write(command.str().c_str());

#ifdef FINSERVO
  /* on attend la fin du mouvement des objectifs */
  this->wait(servo); /* on attend les codes ZF, FF, DF */
#endif
}

/*!
  Get the Servolens current servo position.

  \param servo : Servolens servo motor to actuate.
  \param position : Measured position of the servo.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
bool vpServolens::getPosition(vpServoType servo, unsigned int &position) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }
  char commande[10];
  char posit[10], *pt_posit;
  char c;
  short fin_lect_posit;         /* indique si on a lu la position du servo-moteur */
  short posit_car;              /* donne la position du caractere lu */
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
      switch (posit_car) {
      /* on lit le 1er caractere; (soit Z, soit F, soit D) */
      case 0:
        /* sauvegarde du pointeur */
        pt_posit = posit;

        switch (servo) {
        case ZOOM:
          if (c == 'Z')
            posit_car = 1;
          break;
        case FOCUS:
          if (c == 'F')
            posit_car = 1;
          break;
        case IRIS:
          if (c == 'D')
            posit_car = 1;
          break;
        }
        break;

      /* si le 1er caractere est correct, on lit le 2eme caractere */
      /* (toujours D) */
      case 1:
        if (c == 'D')
          posit_car = 2;
        else
          posit_car = 0; /* le 2eme caractere n'est pas correct */
        break;

      /* si on a lu les 2 premiers caracteres, on peut lire la */
      /* position du servo-moteur */
      case 2:
        if (c >= '0' && c <= '9') {
          *pt_posit++ = c; /* sauvegarde de la position */
          lecture_posit_en_cours = 1;
        } else if (lecture_posit_en_cours) {
          fin_lect_posit = 1;
          *pt_posit = '\0';
        } else
          posit_car = 0;
        break;
      }

    } else {
      // Timout sur la lecture, on retoure FALSE
      return false;
    }
  } while (!fin_lect_posit);

  //    printf("\nChaine lue: posit: %s", posit);

  /* toilettage de la position courantes lue */
  this->clean(posit, posit);

  //    printf("\nChaine toilettee: posit: %s", posit);
  position = (unsigned int)atoi(posit);

  return (true);
}

/*!

  These parameters are computed from the Dragonfly2 DR2-COL camera sensor
pixel size (7.4 um) and from the servolens zoom position.

  \param I : An image coming from the Dragonfly2 camera attached to the
  servolens.

\code
#include <visp3/vs/vpServolens.h>

int main()
{
  // UNIX vpServolens servolens("/dev/ttyS0");
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

  vpImage<unsigned char> I(240, 320);
  vpCameraParameters cam = servolens.getCameraParameters(I);
  std::cout << "Camera parameters: " << cam << std::endl;
#endif
  }
\endcode

  \exception vpRobotException::communicationError : If cannot dial with Servolens.

 */
vpCameraParameters vpServolens::getCameraParameters(vpImage<unsigned char> &I) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }
  vpCameraParameters cam;
  double pix_size = 7.4e-6; // Specific to the Dragonfly2 camera
  double px = 1000, py = 1000, u0 = 320, v0 = 240;
  // Determine if the image is subsampled.
  // Dragonfly2 native images are 640 by 480
  double subsample_factor = 1.;
  double width = I.getWidth();
  double height = I.getHeight();

  if (width > 300 && width < 340 && height > 220 && height < 260)
    subsample_factor = 2;
  else if (width > 140 && width < 1800 && height > 100 && height < 140)
    subsample_factor = 4;

  unsigned zoom;
  getPosition(vpServolens::ZOOM, zoom);
  // std::cout << "Actual zoom value: " << zoom << std::endl;

  // XSIZE_PIX_CAM_AFMA4 / focale et YSIZE_PIX_CAM_AFMA4 / focale
  // correspondent aux parametres de calibration de la camera (donnees
  // constructeur) pour des tailles d'images CCIR (768x576), donc avec scale
  // = 1.
  double focale = zoom * 1.0e-5;                       // Transformation en metres
  px = focale / (double)(subsample_factor * pix_size); // Taille des pixels en metres.
  py = focale / (double)(subsample_factor * pix_size); // Taille des pixels en metres.
  u0 = I.getWidth() / 2.;
  v0 = I.getHeight() / 2.;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

  return cam;
}

/*!
  Waits for the Servolens promt '>'.

  \return The prompt character.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
char vpServolens::wait() const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }

  ssize_t r;
  r = ::write(this->remfd, "\r\n", strlen("\r\n"));
  if (r != (ssize_t)(strlen("\r\n"))) {
    throw vpRobotException(vpRobotException::communicationError, "Cannot write on Servolens.");
  }
  char c;
  do {
    r = ::read(this->remfd, &c, 1);
    c &= 0x7f;
    if (r != 1) {
      throw vpRobotException(vpRobotException::communicationError, "Cannot read on Servolens.");
    }
  } while (c != '>');
  return c;
}

/*!

  Waits the end of motion of the corresponding servo motor.

  \param servo : Servolens servo motor.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.

*/
void vpServolens::wait(vpServoType servo) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
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
    if (::read(this->remfd, &c, 1) != 1) {
      throw vpRobotException(vpRobotException::communicationError, "Cannot read on Servolens.");
    }
    c &= 0x7f;

    /* tests si fin de mouvement */
    if (c == fin_mvt[0]) {
      /* lecture du caractere suivant */
      if (::read(this->remfd, &c, 1) != 1) {
        throw vpRobotException(vpRobotException::communicationError, "Cannot read on Servolens.");
      }

      c &= 0x7f;
      if (c == fin_mvt[1]) {
        sortie = true;
      }
    }
  } while (!sortie);

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
bool vpServolens::read(char *c, long timeout_s) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }

  fd_set readfds;                          /* list of fds for select to listen to */
  struct timeval timeout = {timeout_s, 0}; // seconde, micro-sec

  FD_ZERO(&readfds);
  FD_SET(static_cast<unsigned int>(this->remfd), &readfds);

  if (select(FD_SETSIZE, &readfds, (fd_set *)NULL, (fd_set *)NULL, &timeout) > 0) {
    ssize_t n = ::read(this->remfd, c, 1); /* read one character at a time */
    if (n != 1)
      return false;
    *c &= 0x7f;
    // printf("lecture 1 car: %c\n", *c);
    return (true);
  }

  return (false);
}

/*!
  Write a string to the serial link.

  \param s : String to send to the Servolens.

  \exception vpRobotException::communicationError : If cannot dial
  with Servolens.
*/
void vpServolens::write(const char *s) const
{
  if (!isinit) {
    vpERROR_TRACE("Cannot dial with Servolens.");
    throw vpRobotException(vpRobotException::communicationError, "Cannot dial with Servolens.");
  }
  ssize_t r = 0;
  r = ::write(this->remfd, "\r", strlen("\r"));
  r += ::write(this->remfd, s, strlen(s));
  r += ::write(this->remfd, "\r", strlen("\r"));
  if (r != (ssize_t)(2 * strlen("\r") + strlen(s))) {
    throw vpRobotException(vpRobotException::communicationError, "Cannot write on Servolens.");
  }

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
bool vpServolens::clean(const char *in, char *out) const
{
  short nb_car, i = 0;
  bool error = false;

  nb_car = strlen(in);

  /* on se positionne sur le 1er caractere different de zero */
  while (*(in) == '0' && i++ < nb_car) {
    in++;
    if (i == nb_car) {
      error = true;   /* la chaine ne contient pas une position */
      *(out++) = '0'; /* mise a zero de la position */
    }
  }

  /* copie de la position epuree des zeros de gauche */
  while (i++ <= nb_car) { /* on a mis le = pour copier le caractere de fin */
    /* de chaine \0 */
    *(out++) = *(in++);
  }
  return (error);
}

#endif
