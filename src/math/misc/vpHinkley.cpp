/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Hinkley's cumulative sum test implementation.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpHinkley.cpp

  \brief Definition of the vpHinkley class corresponding to the Hinkley's
  cumulative sum test.

*/

#include <visp/vpHinkley.h>
#include <visp/vpDebug.h>
#include <visp/vpIoTools.h>
#include <visp/vpMath.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

/* VP_DEBUG_MODE fixed by configure:
   1:
   2:
   3: Print data
*/


/*!

  Constructor.

  Call init() to initialise the Hinkley's test and set \f$\alpha\f$
  and \f$\delta\f$ to default values.

  By default \f$ \delta = 0.2 \f$ and \f$ \alpha = 0.2\f$. Use
  setDelta() and setAlpha() to modify these values.

*/
vpHinkley::vpHinkley()
{
  init();

  setAlpha(0.2);
  setDelta(0.2);
}

/*!

  Constructor.

  Call init() to initialise the Hinkley's test and set \f$\alpha\f$
  and \f$\delta\f$ thresholds.

  \param alpha : \f$\alpha\f$ is a predefined threshold.

  \param delta : \f$\delta\f$ denotes the jump minimal magnitude that
  we want to detect.

  \sa setAlpha(), setDelta()

*/

vpHinkley::vpHinkley(double alpha, double delta)
{
  init();

  setAlpha(alpha);
  setDelta(delta);
}

/*!

  Call init() to initialise the Hinkley's test and set \f$\alpha\f$
  and \f$\delta\f$ thresholds.

  \param alpha : \f$\alpha\f$ is a predefined threshold.

  \param delta : \f$\delta\f$ denotes the jump minimal magnitude that
  we want to detect.

  \sa setAlpha(), setDelta()

*/
void
vpHinkley::init(double alpha, double delta)
{
  init();

  setAlpha(alpha);
  setDelta(delta);
}

/*!

  Destructor.

*/
vpHinkley::~vpHinkley()
{
}

/*!

  Initialise the Hinkley's test by setting the mean signal value
  \f$m_0\f$ to zero as well as \f$S_k, M_k, T_k, N_k\f$.

*/
void vpHinkley::init()
{
  nsignal = 0;
  mean  = 0.0;

  Sk = 0;
  Mk = 0;

  Tk = 0;
  Nk = 0;
}

/*!

  Set the value of \f$\delta\f$, the jump minimal magnetude that we want to
  detect.

  \sa setAlpha()

*/
void vpHinkley::setDelta(double delta)
{
  dmin2 = delta / 2;
}

/*!

  Set the value of \f$\alpha\f$, a predefined threshold.

  \sa setDelta()

*/
void vpHinkley::setAlpha(double alpha)
{
  this->alpha = alpha;
}

/*!

  Perform the Hinkley test. A downward jump is detected if
  \f$ M_k - S_k > \alpha \f$.

  \param signal : Observed signal \f$ s(t) \f$.

  \sa setDelta(), setAlpha(), testUpwardJump()

*/
vpHinkley::vpHinkleyJumpType vpHinkley::testDownwardJump(double signal)
{

  vpHinkleyJumpType jump = noJump;

  nsignal ++; // Signal lenght

  if (nsignal == 1) mean = signal;

  // Calcul des variables cumulées
  computeSk(signal);

  computeMk();

  vpCDEBUG(2) << "alpha: " << alpha << " dmin2: " << dmin2
	    << " signal: " << signal << " Sk: " << Sk << " Mk: " << Mk;

  // teste si les variables cumulées excèdent le seuil
  if ((Mk - Sk) > alpha)
    jump = downwardJump;

#ifdef VP_DEBUG
  if (VP_DEBUG_MODE >=2) {
    switch(jump) {
    case noJump:
      std::cout << "noJump " << std::endl;
     break;
    case downwardJump:
      std::cout << "downWardJump " << std::endl;
      break;
    case upwardJump:
      std::cout << "upwardJump " << std::endl;
      break;
    }
  }
#endif

  computeMean(signal);

  if (jump == downwardJump)  {
    vpCDEBUG(2) << "\n*** Reset the Hinkley test  ***\n";

    Sk = 0; Mk = 0;  nsignal = 0;
  }

  return (jump);
}

/*!

  Perform the Hinkley test. An upward jump is detected if \f$ T_k - N_k >
  \alpha \f$.

  \param signal : Observed signal \f$ s(t) \f$.

  \sa setDelta(), setAlpha(), testDownwardJump()

*/
vpHinkley::vpHinkleyJumpType vpHinkley::testUpwardJump(double signal)
{

  vpHinkleyJumpType jump = noJump;

  nsignal ++; // Signal lenght

  if (nsignal == 1) mean = signal;

  // Calcul des variables cumulées
  computeTk(signal);

  computeNk();

  vpCDEBUG(2) << "alpha: " << alpha << " dmin2: " << dmin2
	    << " signal: " << signal << " Tk: " << Tk << " Nk: " << Nk;

  // teste si les variables cumulées excèdent le seuil
  if ((Tk - Nk) > alpha)
    jump = upwardJump;

#ifdef VP_DEBUG
  if (VP_DEBUG_MODE >= 2) {
    switch(jump) {
    case noJump:
      std::cout << "noJump " << std::endl;
     break;
    case downwardJump:
      std::cout << "downWardJump " << std::endl;
      break;
    case upwardJump:
      std::cout << "upWardJump " << std::endl;
      break;
    }
  }
#endif
  computeMean(signal);

  if (jump == upwardJump)  {
    vpCDEBUG(2) << "\n*** Reset the Hinkley test  ***\n";

    Tk = 0; Nk = 0;  nsignal = 0;
  }

  return (jump);
}

/*!

  Perform the Hinkley test. A downward jump is detected if \f$ M_k - S_k >
  \alpha \f$. An upward jump is detected if \f$ T_k - N_k > \alpha \f$.

  \param signal : Observed signal \f$ s(t) \f$.

  \sa setDelta(), setAlpha(), testDownwardJump(), testUpwardJump()

*/
vpHinkley::vpHinkleyJumpType vpHinkley::testDownUpwardJump(double signal)
{

  vpHinkleyJumpType jump = noJump;

  nsignal ++; // Signal lenght

  if (nsignal == 1) mean = signal;

  // Calcul des variables cumulées
  computeSk(signal);
  computeTk(signal);

  computeMk();
  computeNk();

  vpCDEBUG(2) << "alpha: " << alpha << " dmin2: " << dmin2
	      << " signal: " << signal
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
      std::cout << "noJump " << std::endl;
     break;
    case downwardJump:
      std::cout << "downWardJump " << std::endl;
      break;
    case upwardJump:
      std::cout << "upwardJump " << std::endl;
      break;
    }
  }
#endif

  computeMean(signal);

  if ((jump == upwardJump) || (jump == downwardJump)) {
    vpCDEBUG(2) << "\n*** Reset the Hinkley test  ***\n";

    Sk = 0; Mk = 0; Tk = 0; Nk = 0;  nsignal = 0;
    // Debut modif FS le 03/09/2003
    mean = signal;
    // Fin modif FS le 03/09/2003
  }

  return (jump);
}

/*!

  Compute the mean value \f$m_0\f$ of the signal. The mean value must be
  computed before the jump is estimated on-line.

  \param signal : Observed signal \f$ s(t) \f$.

*/
void vpHinkley::computeMean(double signal)
{
  // Debut modif FS le 03/09/2003
  // Lors d'une chute ou d'une remontée lente du signal, pariculièrement
  // après un saut, la moyenne a tendance à "dériver". Pour réduire ces
  // dérives de la moyenne, elle n'est remise à jour avec la valeur
  // courante du signal que si un début de saut potentiel n'est pas détecté.
  //if ( ((Mk-Sk) == 0) && ((Tk-Nk) == 0) )
  if ( ( std::fabs(Mk-Sk) <= std::fabs(vpMath::maximum(Mk,Sk))*std::numeric_limits<double>::epsilon() ) 
       && 
       ( std::fabs(Tk-Nk) <= std::fabs(vpMath::maximum(Tk,Nk))*std::numeric_limits<double>::epsilon() ) )
  // Fin modif FS le 03/09/2003

  // Mise a jour de la moyenne.
    mean = (mean * (nsignal - 1) + signal) / (nsignal);

}
/*!

  Compute \f$S_k = \sum_{t=0}^{k} (s(t) - m_0 + \frac{\delta}{2})\f$

  \param signal : Observed signal \f$ s(t) \f$.

*/
void vpHinkley::computeSk(double signal)
{

  // Calcul des variables cumulées
  Sk += signal - mean + dmin2;
}
/*!

  Compute \f$M_k\f$, the maximum value of \f$S_k\f$.

*/
void vpHinkley::computeMk()
{
  if (Sk > Mk) Mk = Sk;
}
/*!

  Compute \f$T_k = \sum_{t=0}^{k} (s(t) - m_0 - \frac{\delta}{2})\f$

  \param signal : Observed signal \f$ s(t) \f$.
*/
void vpHinkley::computeTk(double signal)
{

  // Calcul des variables cumulées
  Tk += signal - mean - dmin2;
}
/*!

  Compute \f$N_k\f$, the minimum value of \f$T_k\f$.

*/
void vpHinkley::computeNk()
{
  if (Tk < Nk) Nk = Tk;
}

void vpHinkley::print(vpHinkley::vpHinkleyJumpType jump)
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

