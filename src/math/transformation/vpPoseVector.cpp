/****************************************************************************
 *
 * $Id: vpPoseVector.cpp,v 1.10 2008-07-28 16:46:45 fspindle Exp $
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
  for (int i =0  ; i < 3 ; i++)
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
  for (int i =0  ; i < 6 ; i++)
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
      for (int i=0 ; i < 6 ; i++)
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
/****************************************************************

           Deprecated functions

*****************************************************************/

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!  

  \deprecated Construct a 6 dimension pose vector \f$[ \Theta \bf
  u]^\top\f$ from a translation vector and an Euler rotation
  vector. Since vpEulerVector is deprecated (because ambiguous) this
  constructor should not be used.

  \param t : Translation vector \f$ \bf t \f$.

  \param e : Euler vector. This vector is converted into a \f$\Theta
  \bf u\f$ vector.

*/
vpPoseVector::vpPoseVector(const vpTranslationVector& t,
                           const vpEulerVector &e)
{
  init() ;
  vpRzyzVector rzyz;
  for (int i=0; i < 3; i ++)
    rzyz[i] = e[i];
  vpThetaUVector tu;
  tu.buildFrom(rzyz);
  buildFrom(t,tu) ;
}

/*! 

  \deprecated Constructor. Convert a "euler" vector and a translation
  vector into a pose.

*/
vpPoseVector::vpPoseVector(const vpEulerVector &e,
                           const vpTranslationVector& t)
{
  init() ;
  vpRzyzVector rzyz;
  for (int i=0; i < 3; i ++)
    rzyz[i] = e[i];
  vpThetaUVector tu;
  tu.buildFrom(rzyz);
  buildFrom(t,tu) ;
  vpTRACE("Warning : This function is deprecated : \
          use vpPoseVector(vpTranslationVector,vpThetaVector&) instead.");
}

/*! 

  \deprecated Constructor. Convert a "thetau" vector and a translation
  vector into a pose.

  This function is deprecated: use vpPoseVector(const
  vpTranslationVector&,const vpThetaUVector&) instead.

*/
vpPoseVector::vpPoseVector(const vpThetaUVector& tu,
                           const vpTranslationVector& t)
{
  init() ;
  buildFrom(t,tu) ;
  vpTRACE("Warning : This function is deprecated :  \
          use vpPoseVector(vpTranslationVector,vpThetaUVector&) instead.");
}

/*! 

  \deprecated Constructor. Convert a rotation matrix and a translation
  vector into a pose.

  This function is deprecated: use vpPoseVector(const
  vpTranslationVector&,const vpRotationMatrix&) instead.

*/
vpPoseVector::vpPoseVector(const vpRotationMatrix& R,
                           const vpTranslationVector& t)
{
  init() ;
  buildFrom(t,R) ;
  vpTRACE("Warning : This function is deprecated :  \
          use vpPoseVector(vpTranslationVector,vpRotationMatrix&) instead.");
}

/*!

  \deprecated Convert a translation and a "euler" vector into a pose.

*/
vpPoseVector
vpPoseVector::buildFrom(const vpTranslationVector& t,
                        const vpEulerVector &e)
{
  vpThetaUVector tu ;
  tu.buildFrom(e) ;
  buildFrom(t,tu) ;
  return *this ;
}

/*! 

  \deprecated Convert a "euler" vector and a translation vector into a pose.

  This function is deprecated: use buildFrom(const
  vpTranslationVector&,const vpEulerVector&) instead.

*/
vpPoseVector
vpPoseVector::buildFrom(const vpEulerVector &e,
                        const vpTranslationVector& t)
{
  vpThetaUVector tu ;
  tu.buildFrom(e) ;
  buildFrom(t,tu) ;
  vpTRACE("Warning : This function is deprecated :  \
          use buildFrom(vpTranslationVector,vpEulerVector&) instead.");
  return *this ;
}

/*!  

  \deprecated Convert a "thetau" vector and a translation vector into
  a pose.

  This function is deprecated: 
  use buildFrom(const vpTranslationVector&,const vpEulerVector&) instead.
*/

vpPoseVector
vpPoseVector::buildFrom(const vpThetaUVector& tu,
                        const vpTranslationVector& t)
{
  for (int i =0  ; i < 3 ; i++)
    {
      (*this)[i] = t[i] ;
      (*this)[i+3] = tu[i] ;
    }
  vpTRACE("Warning : This function is deprecated :  \
          use buildFrom(vpTranslationVector,vpThetaUVector&) instead.");
  return *this ;
}

/*!  

  \deprecated Convert a rotation matrix and a translation vector into a pose.

  This function is deprecated: 
  use buildFrom(const vpTranslationVector&,const vpEulerVector&) instead.
 
*/

vpPoseVector
vpPoseVector::buildFrom(const vpRotationMatrix& R,
                        const vpTranslationVector& t)
{
  vpThetaUVector tu ;
  tu.buildFrom(R) ;

  buildFrom(t,tu) ;
  vpTRACE("Warning : This function is deprecated :  \
          use buildFrom(vpTranslationVector,vpRotationMatrix&) instead.");
  return *this ;
}

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
