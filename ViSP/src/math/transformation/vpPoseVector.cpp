/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Pose object. A pose is a size 6 vector [t, tu]^T where tu is
 * a rotation vector (theta u representation) and t is a translation vector.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file vpPoseVector.cpp
  \brief  Pose vector.

*/

#include <visp/vpPoseVector.h>
#include <visp/vpMath.h>
#include <visp/vpDebug.h>
#include <visp/vpMatrixException.h>
#include <visp/vpException.h>

/*!
  Set the size of the vector to 6.
*/
void 
vpPoseVector::init()
{
  resize(6) ;
}

/*!
  
  Default constructor.

  Construct a 6 dimension pose vector \f$ [\bf t, \Theta \bf
  u]^\top\f$ where \f$ \Theta \bf u\f$ is a rotation vector \f$[\Theta
  u_x, \Theta u_y, \Theta u_z]^\top\f$ and \f$ \bf t \f$ is a
  translation vector \f$[t_x, t_y, t_z]^\top\f$.

  The pose vector is initialized to zero.

*/
vpPoseVector::vpPoseVector()
{
  init() ;
}

/*!  

  Construct a 6 dimension pose vector \f$ [\bf{t}, \Theta
  \bf{u}]^\top\f$ from 3 translations and 3 \f$ \Theta \bf{u}\f$
  angles.

  Translations are expressed in meters, while rotations in radians.

  \param tx,ty,tz : Translations \f$[t_x, t_y, t_z]^\top\f$
  respectively along the x, y and z axis (in meters).

  \param tux,tuy,tuz : Rotations \f$[\Theta u_x, \Theta u_y, \Theta
  u_z]^\top\f$ respectively around the x, y and z axis (in radians).

*/
vpPoseVector::vpPoseVector(const double tx,
                           const double ty,
                           const double tz,
                           const double tux,
                           const double tuy,
                           const double tuz)
{
  init() ;

  (*this)[0] = tx ;
  (*this)[1] = ty ;
  (*this)[2] = tz ;

  (*this)[3] = tux ;
  (*this)[4] = tuy ;
  (*this)[5] = tuz ;
}

/*! 

  Construct a 6 dimension pose vector \f$ [\bf t, \Theta \bf
  u]^\top\f$ from a translation vector \f$ \bf t \f$ and a \f$\Theta
  \bf u\f$ vector.

  \param t : Translation vector \f$ \bf t \f$.
  \param tu : \f$\Theta \bf u\f$ rotation  vector.

*/
vpPoseVector::vpPoseVector(const vpTranslationVector& t,
                           const vpThetaUVector& tu)
{
  init() ;
  buildFrom(t,tu) ;
}

/*! 

  Construct a 6 dimension pose vector \f$ [\bf t, \Theta \bf
  u]^\top\f$ from a translation vector \f$ \bf t \f$ and a rotation
  matrix \f$ \bf R \f$.

  \param t : Translation vector \f$ \bf t \f$.

  \param R : Rotation matrix \f$ \bf R \f$ from which \f$\Theta \bf
  u\f$ vector is extracted to initialise the pose vector.

*/
vpPoseVector::vpPoseVector(const vpTranslationVector& t,
                           const vpRotationMatrix& R)
{
  init() ;
  buildFrom(t,R) ;
}

/*! 

  Construct a 6 dimension pose vector \f$ [\bf t, \Theta \bf
  u]^\top\f$ from an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation
  \f$ \bf t \f$ and \f$\Theta \bf u \f$ vectors are extracted to
  initialize the pose vector.

*/
vpPoseVector::vpPoseVector(const vpHomogeneousMatrix& M)
{
  init() ;
  buildFrom(M) ;
}

/*!
  Build a 6 dimension pose vector \f$ [\bf t, \Theta \bf u]^\top\f$ from
  an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation \f$
  \bf t \f$ and \f$\Theta \bf u \f$ vectors are extracted to initialize
  the pose vector.

  \return The build pose vector.

*/
vpPoseVector
vpPoseVector::buildFrom(const vpHomogeneousMatrix& M)
{
  vpRotationMatrix R ;    M.extract(R) ;
  vpTranslationVector t ; M.extract(t) ;
  buildFrom(t,R) ;
  return *this ;
}

/*! 

  Build a 6 dimension pose vector \f$ [\bf t, \Theta \bf u]^\top\f$
  from a translation vector \f$ \bf t \f$ and a \f$\Theta \bf u\f$
  vector.

  \param t : Translation vector \f$ \bf t \f$.
  \param tu : \f$\Theta \bf u\f$ rotation  vector.

  \return The build pose vector.
*/
vpPoseVector
vpPoseVector::buildFrom(const vpTranslationVector& t,
                        const vpThetaUVector& tu)
{
  for (unsigned int i =0  ; i < 3 ; i++)
    {
      (*this)[i] = t[i] ;
      (*this)[i+3] = tu[i] ;
    }
  return *this ;
}

/*! 

  Build a 6 dimension pose vector \f$ [\bf t, \Theta \bf u]^\top\f$
  from a translation vector \f$ \bf t \f$ and a rotation matrix \f$
  \bf R \f$.

  \param t : Translation vector \f$ \bf t \f$.

  \param R : Rotation matrix \f$ \bf R \f$ from which \f$\Theta \bf
  u\f$ vector is extracted to initialise the pose vector.

  \return The build pose vector.
*/
vpPoseVector
vpPoseVector::buildFrom(const vpTranslationVector& t,
                        const vpRotationMatrix& R)
{
  vpThetaUVector tu ;
  tu.buildFrom(R) ;

  buildFrom(t,tu) ;
  return *this ;
}



/*!

  Prints to the standart stream the pose vector.

  \warning Values concerning the \f$ \Theta \bf u\f$ rotation are
  converted in degrees.

  The following code
  \code
  // Create a pose vector
  vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);
  r.print();
  \endcode
  produces the output:

  \code
  1 2 3 180 -180 0
  \endcode
*/
void
vpPoseVector::print()
{
  for (unsigned int i =0  ; i < 6 ; i++)
    if (i<3) std::cout << (*this)[i] <<" " ;
    else  std::cout << vpMath::deg((*this)[i]) <<" " ;
  std::cout <<std::endl ;

}

/*!

  Save the pose vector in the output file stream.

  \param f : Output file stream. Should be open before entering in this method.

  \exception vpException::ioError : If the output stream is not open.

  \sa load()
*/
void
vpPoseVector::save(std::ofstream &f) const
{
  if (f != NULL)
    {
      f << *this ;
    }
  else
    {
      vpERROR_TRACE("\t\t file not open " );
      throw(vpException(vpException::ioError, "\t\t file not open")) ;
    }
}


/*!
  Read a pose vector from an input file stream. 

  \param f : The input file stream..Should be open before entering in
  this method.

  \exception vpException::ioError : If the input file stream is not open.

  \sa save()
*/
void
vpPoseVector::load(std::ifstream &f)
{
  if (f != NULL)
    {
      for (unsigned int i=0 ; i < 6 ; i++)
	{
	  f>>   (*this)[i] ;
	}
    }
  else
    {
      vpERROR_TRACE("\t\t file not open " );
      throw(vpException(vpException::ioError, "\t\t file not open")) ;
    }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
