/****************************************************************************
 *
 *       Copyright (c) 1999 by IRISA/INRIA Rennes.
 *       All Rights Reserved.
 *
 *       IRISA/INRIA Rennes
 *       Campus Universitaire de Beaulieu
 *       35042 Rennes Cedex
 *
 ****************************************************************************/

/*!

  \file vpHinkley.cpp

  \brief Definition of the vpHinkley class corresponding to the Hinkley's
  cumulative sum test.

*/

/*!
  \class vpHinkley

  \brief This class implements the Hinkley's cumulative sum test.

  \author Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

  The Hinkley's cumulative sum test is designed to detect jump in mean of an
  observed signal. It is known to be robust (by taking into account all the
  past of the observed quantity), efficient, and inducing a very low
  computational load. The other attractive features of this test are
  two-fold. First, it can straightforwardly and accurately provide the jump
  instant. Secondly, due to its formulation (cumulative sum test), it can
  simultaneously handle both very abrupt and important changes, and gradual
  smaller ones without adapting the involved thresholds.

  Two tests are performed in parallel to look for downwards or upwards jumps,
  respectively defined by:

  \f[ S_k = \sum_{t=0}^{k} (data_t - m_0 + \frac{\delta}{2}) \f]
  \f[ M_k = \max_{0 \leq i \leq k} S_i\f]
  \f[ T_k = \sum_{t=0}^{k} (data_t - m_0 - \frac{\delta}{2}) \f]
  \f[ N_k = \max_{0 \leq i \leq k} T_i\f]


  In which \f$m_o\f$ is computed on-line and corresponds to the mean of the
  signal \f$data_t\f$ we want to detect a jump. \f$m_o\f$ is re-initialized at
  zero after each jump detection. \f$\delta\f$ denotes the jump minimal
  magnitude that we want to detect, and \f$\alpha\f$ is a predefined threshold.

  A downward jump is detected if \f$ M_k - S_k > \alpha \f$.
  A upward jump is detected if \f$ T_k - S_k > \alpha \f$.

  If a jump is detected, the jump location is given by the last instant
  \f$k^{'}\f$ when \f$ M_{k^{'}} - S_{k^{'}} = 0 \f$, or \f$ T_{k^{'}} -
  N_{k^{'}} = 0 \f$.

*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <visp/vpHinkley.h>
#include <visp/vpDebug.h>
#include <visp/vpIoTools.h>


/* __DEBUG_LEVEL_ fixed by configure:
   1:
   2:
   3: Creation d'un fichier de donnees
*/

#ifdef VP_DEBUG
#  if (VP_DEBUG_MODE >= 3)
static FILE * f_hinkley;
#  endif
#endif

/*!

  Constructor.

*/
vpHinkley::vpHinkley()
{
  init();

  setAlpha(0.2);
  setDelta(0.2);
  setIter(0);

#ifdef VP_DEBUG
#  if (VP_DEBUG_MODE >= 3)
  char* dir,*file;
  dir = new char[FILENAME_MAX];
  file = new char[FILENAME_MAX];

  string user;
  vpIoTools::getUserName(user);

  sprintf(dir,"/tmp/%s", user.c_str());

  if (vpIoTools::checkDirectory(dir) == false)
    vpIoTools::makeDirectory(dir);


  sprintf(file, "%s/hinkley.dat", dir);

  f_hinkley = fopen(file, "w");
  fprintf(f_hinkley, "%s%s%s%s%s%s%s%s%s%s%s",
	  "# Contient des infos relatives au test de Hinkley\n",
	  "# avec dans l'ordre:\n",
	  "# colonne 1: iteration\n",
	  "# colonne 2: support\n",
	  "# colonne 3: Sk\n",
	  "# colonne 4: Mk\n",
	  "# colonne 5: Mk-Sk\n",
	  "# colonne 6: Tk\n",
	  "# colonne 7: Nk\n",
	  "# colonne 8: Tk-Nk\n",
	  "# colonne 9: jump\n");

  delete [] dir;
  delete [] file;

#  endif
#endif
}

/*!

  Constructor.

*/

vpHinkley::vpHinkley(double alpha, double delta)
{
  init();

  setAlpha(alpha);
  setDelta(delta);
  setIter(0);


}

void
vpHinkley::init(double alpha, double delta)
{
  init();

  setAlpha(alpha);
  setDelta(delta);
  setIter(0);


}
/*!

  Destructor.
*/
vpHinkley::~vpHinkley()
{
#ifdef VP_DEBUG
#  if (VP_DEBUG_MODE >= 3)
  fclose(f_hinkley);
#  endif
#endif
}

/*!

  initialise le test de Hinkley en positionnant la moyenne \f$m_0\f$ du signal
  et \f$S_k, M_k, T_k, N_k\f$ à zéro.

*/
void vpHinkley::init()
{
  ndata	= 0;
  mean  = 0.0;

  Sk = 0;
  Mk = 0;

  Tk = 0;
  Nk = 0;
}

/*!

  set the value of \f$\delta\f$, the jump minimal magnetude that we want to
  detect.

*/
void vpHinkley::setDelta(double delta)
{
  dmin2 = delta / 2;
}

/*!

  set the value of \f$\alpha\f$, a predefined threshold.

*/
void vpHinkley::setAlpha(double alpha)
{
  this->alpha = alpha;
}

/*!

  set an iteration value, only used to output debug information. Not used
  in the internal Hinkley test.

*/
void vpHinkley::setIter(int iter)
{
  this->iter = iter;
}

/*!

  Perform the Hinkley test. A downward jump is detected if
  \f$ M_k - S_k > \alpha \f$.

  \sa setDelta(), setAlpha(), testUpWardJump()

*/
hinkleyJump vpHinkley::testDownwardJump(double data)
{

  hinkleyJump jump = noJump;

  ndata ++; // Signal lenght

  if (ndata == 1) mean = data;

  // Calcul des variables cumulées
  computeSk(data);

  computeMk();

  vpCDEBUG(2) << "alpha: " << alpha << "dmin2: " << dmin2
	    << " data: " << data << " Sk: " << Sk << " Mk: " << Mk;

  // teste si les variables cumulées excèdent le seuil
  if ((Mk - Sk) > alpha)
    jump = downwardJump;

#ifdef VP_DEBUG
  if (VP_DEBUG_MODE >=2) {
    switch(jump) {
    case noJump:
      cout << "noJump " << endl;
     break;
    case downwardJump:
      cout << "downWardJump " << endl;
      break;
    case upwardJump:
      cout << "upwardJump " << endl;
      break;
    }
  }
#endif

  computeMean(data);

  if (jump == downwardJump)  {
    vpCDEBUG(2) << "\n*** DECROCHAGE  ***\n";

    Sk = 0; Mk = 0;  ndata = 0;
  }

  return (jump);
}

/*!

  Perform the Hinkley test. An upward jump is detected if \f$ T_k - N_k >
  \alpha \f$.

  \sa setDelta(), setAlpha(), testDownWardJump()

*/
hinkleyJump vpHinkley::testUpwardJump(double data)
{

  hinkleyJump jump = noJump;

  ndata ++; // Signal lenght

  if (ndata == 1) mean = data;

  // Calcul des variables cumulées
  computeTk(data);

  computeNk();

  vpCDEBUG(2) << "alpha: " << alpha << "dmin2: " << dmin2
	    << " data: " << data << " Tk: " << Tk << " Nk: " << Nk;

  // teste si les variables cumulées excèdent le seuil
  if ((Tk - Nk) > alpha)
    jump = upwardJump;

#ifdef VP_DEBUG
  if (VP_DEBUG_MODE >= 2) {
    switch(jump) {
    case noJump:
      cout << "noJump " << endl;
     break;
    case downwardJump:
      cout << "downWardJump " << endl;
      break;
    case upwardJump:
      cout << "upWardJump " << endl;
      break;
    }
  }
#endif
  computeMean(data);

  if (jump == upwardJump)  {
    vpCDEBUG(2) << "\n*** DECROCHAGE  ***\n";

    Tk = 0; Nk = 0;  ndata = 0;
  }

  return (jump);
}

/*!

  Perform the Hinkley test. A downward jump is detected if \f$ M_k - S_k >
  \alpha \f$. An upward jump is detected if \f$ T_k - S_k > \alpha \f$.

  \sa setDelta(), setAlpha(), testDownWardJump(), testUpWardJump()

*/
hinkleyJump vpHinkley::testDownUpwardJump(double data)
{

  hinkleyJump jump = noJump;

  ndata ++; // Signal lenght

  if (ndata == 1) mean = data;

  // Calcul des variables cumulées
  computeSk(data);
  computeTk(data);

  computeMk();
  computeNk();

  vpCDEBUG(2) << "alpha: " << alpha << "dmin2: " << dmin2 << " data: " << data
	    << " Sk: " << Sk << " Mk: " << Mk
		<< " Tk: " << Tk << " Nk: " << Nk << std::endl;

  // teste si les variables cumulées excèdent le seuil
  if ((Mk - Sk) > alpha)
    jump = downwardJump;
  else if ((Tk - Nk) > alpha)
    jump = upwardJump;

#ifdef VP_DEBUG
  if (VP_DEBUG_MODE >= 2) {
    switch(jump) {
    case noJump:
      cout << "noJump " << endl;
     break;
    case downwardJump:
      cout << "downWardJump " << endl;
      break;
    case upwardJump:
      cout << "upwardJump " << endl;
      break;
    }
  }

#  if (VP_DEBUG_MODE >= 3)
  fprintf(f_hinkley, "%d %f %f %f %f %f %f %f %d\n",
	  iter, data, Sk, Mk, Mk-Sk, Tk, Nk, Tk-Nk, jump);
  fflush(f_hinkley);
#  endif
#endif
  computeMean(data);

  if ((jump == upwardJump) || (jump == downwardJump)) {
    vpCDEBUG(2) << "\n*** DECROCHAGE  ***\n";

    Sk = 0; Mk = 0; Tk = 0; Nk = 0;  ndata = 0;
    // Debut modif FS le 03/09/2003
    mean = data;
    // Fin modif FS le 03/09/2003
  }

  return (jump);
}

/*!

  compute the mean value \f$m_0\f$ of the signal. The mean value must be
  computed before the jump is estimated on-line.

*/
void vpHinkley::computeMean(double data)
{
  // Debut modif FS le 03/09/2003
  // Lors d'une chute ou d'une remontée lente du signal, pariculièrement
  // après un saut, la moyenne a tendance à "dériver". Pour réduire ces
  // dérives de la moyenne, elle n'est remise à jour avec la valeur
  // courante du signal que si un début de saut potentiel n'est pas détecté.
  if ( ((Mk-Sk) == 0) && ((Tk-Nk) == 0) )
  // Fin modif FS le 03/09/2003

  // Mise a jour de la moyenne.
    mean = (mean * (ndata - 1) + data) / (ndata);

}
/*!

  compute \f$S_k = \sum_{t=0}^{k} (data_t - m_0 + \frac{\delta}{2})\f$

*/
void vpHinkley::computeSk(double data)
{

  // Calcul des variables cumulées
  Sk += data - mean + dmin2;
}
/*!

  compute \f$M_k\f$, the maximum value of \f$S_k\f$.

*/
void vpHinkley::computeMk()
{
  if (Sk > Mk) Mk = Sk;
}
/*!

  compute \f$T_k = \sum_{t=0}^{k} (data_t - m_0 - \frac{\delta}{2})\f$

*/
void vpHinkley::computeTk(double data)
{

  // Calcul des variables cumulées
  Tk += data - mean - dmin2;
}
/*!

  compute \f$N_k\f$, the minimum value of \f$T_k\f$.

*/
void vpHinkley::computeNk()
{
  if (Tk < Nk) Nk = Tk;
}

void vpHinkley::print(hinkleyJump jump) 
{
  switch(jump)
    {
    case noJump :
      std::cout << " No jump detected " << std::endl ;
      break ;
    case downwardJump :
      std::cout << " Jump downward detected " << std::endl ;
      break ; 
    case upwardJump :
      std::cout << " Jump upward detected " << std::endl ;
      break ;  
    default:
      std::cout << " Jump  detected " << std::endl ;
      break ;  

  }
}

