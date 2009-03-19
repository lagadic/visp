/****************************************************************************
 *
 * $Id: vpLine.cpp,v 1.9 2008-07-18 10:20:46 marchand Exp $
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
 * Line feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpLine.h>

#include <visp/vpDebug.h>
#include <visp/vpMath.h>

#include <visp/vpFeatureDisplay.h>

/*!
  \file vpLine.cpp
  \brief   class that defines what is a line
*/



/*!
  Initialize the memory space requested for 2D and 3D line parameters.
*/
void
vpLine::init()
{
  oP.resize(8) ;
  cP.resize(8) ;
  p.resize(2) ;
}

/*! 
  Default constructor.
*/
vpLine::vpLine()
{
  init() ;
}



/*!
 
  Sets the parameters which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two plans. The different parameters here define the
  equations of the two plans in the object frame.

  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in the object frame.

  \param A1 : The A parameter of the equation which defines the first plan.
  \param B1 : The B parameter of the equation which defines the first plan.
  \param C1 : The C parameter of the equation which defines the first plan.
  \param D1 : The D parameter of the equation which defines the first plan.
  \param A2 : The A parameter of the equation which defines the second plan.
  \param B2 : The B parameter of the equation which defines the second plan.
  \param C2 : The C parameter of the equation which defines the second plan.
  \param D2 : The D parameter of the equation which defines the second plan.
*/
void
vpLine::setWorldCoordinates(const double &A1, const double &B1,
			    const double &C1, const double &D1,
			    const double &A2, const double &B2,
			    const double &C2, const double &D2)
{
  oP[0] = A1 ;
  oP[1] = B1 ;
  oP[2] = C1 ;
  oP[3] = D1 ;

  oP[4] = A2 ;
  oP[5] = B2 ;
  oP[6] = C2 ;
  oP[7] = D2 ;
}


/*! 

  Sets the parameters which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two plans. Eight parameters are required to define
  the equations of the two plans in the object frame.

  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in the object frame.

  \param oP : The column vector which contains the height parameters
  needed to define the equations of the two plans. \f[ oP =
  \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1 \\ A2 \\ B2 \\ C2 \\ D2
  \end{array}\right] \f]

*/
void
vpLine::setWorldCoordinates(const vpColVector &oP)
{
  if (oP.getCols() != 8)
    throw vpException(vpException::dimensionError, "Size of oP is not equal to 8 as it should be");

  this->oP = oP ;
}


/*! 

  Sets the parameters which define the line in the object frame. As
  said in the class description, the line is defined as the
  intersection of two plans. Eight parameters are required to define
  the equations of the two plans in the object frame.

  \f[ A1 X + B1 Y + C1 Z +D1 = 0 \f]
  \f[ A2 X + B2 Y + C2 Z +D2 = 0 \f]
  Here \f$ (X, Y, Z) \f$ are the 3D coordinates in the object frame.

  \param oP1 : The column vector which contains the four parameters
  needed to define the equations of the first plan. \f[ oP1 =
  \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1 \end{array}\right] \f]
  
  \param oP2 : The column vector which contains the four parameters
  needed to define the equations of the second plan. \f[ oP2 =
  \left[\begin{array}{c} A2 \\ B2 \\ C2 \\ D2 \end{array}\right] \f]

*/
void
vpLine::setWorldCoordinates(const vpColVector &oP1,
			    const vpColVector &oP2)
{
    if (oP1.getCols() != 4)
      throw vpException(vpException::dimensionError, "Size of oP1 is not equal to 4 as it should be");

    if (oP2.getCols() != 4)
      throw vpException(vpException::dimensionError, "Size of oP2 is not equal to 4 as it should be");

  for (int i=0 ; i < 4 ; i++)
  {
    oP[i]   = oP1[i] ;
    oP[i+4] = oP2[i] ;
  }

}


/*!

  Computes the 2D equation of the line thanks to the 3D line features
  in the camera frame. The method used is the perspective projection.
  The computed parameters are \f$ \rho \f$ and \f$ \theta \f$.

  The code below shows how to use this method.
  \code
  //Create the line
  vpLine line;

  //Set the coordinates of the line in the object frame in meter.
  line.setWorldCoordinates( 1, 0, 0, -0.5, 0, 0, 1, 0.5)
  //Here the line is define by the intersection between the plan X = 0.5m and Z = 0.5m

  //Create the homogeneous matrix
  vpHomogeneousMatrix cMo;
  //Computes or set here the homogeneous matrix

  //Computes the equations of the two plans in the camera frame
  line.changeFrame(cMo);

  //Computes the line features in the camera frame( rho and theta)
  line.projection();
  \endcode
*/
void
vpLine::projection()
{
  projection(cP,p) ;
}


/*!

  Computes the 2D equation of the line thanks to the 3D line
  features. The method used is the perspective projection.  The
  computed parameters are \f$ \rho \f$ and \f$ \theta \f$.

  \param cP : The vector containing the line features relative to the
  camera frame. \f[ cP = \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1
  \\ A2 \\ B2 \\ C2 \\ D2 \end{array}\right] \f]
  
  \param p : The vector which contains the result of the computation
  ie the 2D line features. \f[ p = \left[\begin{array}{c} \rho \\
  \theta \end{array}\right] \f]
*/
void
vpLine::projection(const vpColVector &cP, vpColVector &p)
{
 //projection

  if (cP.getCols() != 8)
    throw vpException(vpException::dimensionError, "Size of cP is not equal to 8 as it should be");

  double A1, A2, B1, B2, C1, C2, D1, D2;

  A1=cP[0] ;
  B1=cP[1] ;
  C1=cP[2] ;
  D1=cP[3] ;

  A2=cP[4] ;
  B2=cP[5] ;
  C2=cP[6] ;
  D2=cP[7] ;

  double a, b, c, s;
  a = A1*D2 - A2*D1;
  b = B1*D2 - B2*D1;
  c = C1*D2 - C2*D1;
  s = sqrt( a*a+b*b );

  double rho = -c/s ;
  double theta = atan2( b, a);

  //while (theta > M_PI/2) { theta -= M_PI ; rho *= -1 ; }
  //while (theta < -M_PI/2) { theta += M_PI ; rho *= -1 ; }

  if (p.getCols() != 2)
    p.resize(2);

  p[0] = rho ;
  p[1] = theta ;
}


/*!

  Computes the line features given in the camera frame thanks to the
  line features given in the object frame and the homogeneous matrix
  relative to the difference between the object frame and the camera
  frame. Thus the computation gives the equations of the two plans
  needed to define the line in the camera frame thanks to the
  equations of the same two plans in the object frame.

  \param cMo : The homogeneous matrix relative to the difference
  between the object frame and the camera frame.

  The code below shows how to use this method.
  \code
  //Create the line
  vpLine line;

  //Set the coordinates of the line in the object frame in meter.
  line.setWorldCoordinates( 1, 0, 0, -0.5, 0, 0, 1, 0.5)
  //The line is define by the intersection between the plan X = 0.5m and Z = 0.5m

  //Create the homogeneous matrix
  vpHomogeneousMatrix cMo;
  //Computes or set here the homogeneous matrix

  //Computes the equations of the two plans in the camera frame
  line.changeFrame(cMo);
  \endcode
*/
void
vpLine::changeFrame(const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo,cP) ;
}


/*!

  Computes the line features given in a 3D frame of your choice thanks
  to the line features given in the object frame and the homogeneous
  matrix relative to the difference between the object frame and the
  frame you choose. Thus the computation gives the equations of the
  two plans needed to define the line in the desired frame thanks to
  the equations of the same two plans in the object frame.

  \param cMo : The homogeneous matrix relative to the difference
  between the object frame and the desired frame.

  \param cP : The vector which will contain the parameters of the two
  plans equations in the desired frame. \f[ cP =
  \left[\begin{array}{c}A1 \\ B1 \\ C1 \\ D1 \\ A2 \\ B2 \\ C2 \\ D2
  \end{array}\right] \f]

  The code below shows how to use this method.
  \code
  //Create the line
  vpLine line;

  //Set the coordinates of the line in the object frame in meter.
  line.setWorldCoordinates( 1, 0, 0, -0.5, 0, 0, 1, 0.5)
  //The line is define by the intersection between the plan X = 0.5m and Z = 0.5m

  //Create the homogeneous matrix
  vpHomogeneousMatrix cMo;
  //Computes or set here the homogeneous matrix

  //Creates the vector which will contain the line features
  vpColVector cP(8);

  //Computes the equations of the two plans in the camera frame
  line.changeFrame(cMo, cP);
  \endcode
*/
void
vpLine::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP)
{

  double a1, a2, b1, b2, c1, c2, d1, d2;
  double A1, A2, B1, B2, C1, C2, D1, D2;

  a1=oP[0] ;
  b1=oP[1] ;
  c1=oP[2] ;
  d1=oP[3] ;

  a2=oP[4] ;
  b2=oP[5] ;
  c2=oP[6] ;
  d2=oP[7] ;

  A1 = cMo[0][0]*a1 + cMo[0][1]*b1  + cMo[0][2]*c1;
  B1 = cMo[1][0]*a1 + cMo[1][1]*b1  + cMo[1][2]*c1;
  C1 = cMo[2][0]*a1 + cMo[2][1]*b1  + cMo[2][2]*c1;
  D1 = d1 - (cMo[0][3]*A1 + cMo[1][3]*B1  + cMo[2][3]*C1);

  A2 = cMo[0][0]*a2 + cMo[0][1]*b2  + cMo[0][2]*c2;
  B2 = cMo[1][0]*a2 + cMo[1][1]*b2  + cMo[1][2]*c2;
  C2 = cMo[2][0]*a2 + cMo[2][1]*b2  + cMo[2][2]*c2;
  D2 = d2 - (cMo[0][3]*A2 + cMo[1][3]*B2  + cMo[2][3]*C2);


  if (fabs(D2) < 1e-8)
  {
    //swap the two plane
    vpMath::swap(A1,A2) ;
    vpMath::swap(B1,B2) ;
    vpMath::swap(C1,C2) ;
    vpMath::swap(D1,D2) ;
  }

  //  vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
  //  vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;


  if ((fabs(D1) > 1e-8) || (fabs(D2) > 1e-8))
  {
    // Rajout des quatre contraintes sur la droite

    // Calcul du plan P1 passant par l'origine avec les contraintes
    // Contrainte d1 = 0
    double alpha1 ;
    double beta1  ;
    double gamma1 ;

    {
      alpha1 = D2*A1 - D1*A2 ;
      beta1  = D2*B1 - D1*B2 ;
      gamma1 = D2*C1 - D1*C2 ;
    }

    // Contrainte a1^2 + b1^2 + c1^2 = 1
    double s1 = sqrt (alpha1*alpha1 + beta1*beta1 + gamma1*gamma1);
    A1 =  alpha1/s1 ;
    B1 =  beta1/s1 ;
    C1 =  gamma1/s1 ;
    D1 = 0 ;

    //   vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;

    //std::cout <<"--> "<< A1 << "  " << B1 << "  " << C1<< "  " << D1 <<std::endl ;

    // ajout de la contrainte a1 a2 + b1 b2 + c1 c2 = 0
    double x1,y1 ;
    if (fabs(A1) > 0.01)
    {

      //    vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
      //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;
      x1 = A1*B2 - B1*A2;
      y1 = A1*C2 - C1*A2;
      A2 = -(B1*x1+C1*y1);
      B2= ((A1*A1+C1*C1)*x1-B1*C1*y1)/A1;
      C2 = (-B1*C1*x1+ (A1*A1+B1*B1)*y1)/A1;
    }
    else if (fabs(B1) > 0.01){

      //    vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
      //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

      x1 = A1*B2 - B1*A2;
      y1 = C1*B2 - B1*C2;
      A2 = -((B1*B1+C1*C1)*x1-A1*C1*y1)/B1;
      B2= A1*x1+C1*y1;
      C2 = -(-A1*C1*x1+(A1*A1+B1*B1)*y1)/B1;
    }
    else {

      //    vpERROR_TRACE("A1 B1 C1 D1 %f %f %f %f  ", A1, B1, C1, D1) ;
      //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

      x1 = A1*C2 - C1*A2;
      y1 = B1*C2 - C1*B2;
      A2= (-(B1*B1+C1*C1)*x1+A1*B1*y1)/C1;
      B2 = (A1*B1*x1-(A1*A1+C1*C1)*y1)/C1;
      C2 = A1*x1+B1*y1;
    }
    // Contrainte de normalisation
    //   vpERROR_TRACE("A2 B2 C2 D2 %f %f %f %f  ", A2, B2, C2, D2) ;

    double s2 = sqrt (A2*A2 +  B2*B2 + C2*C2);
    A2 = A2/s2;
    B2 = B2/s2;
    C2 = C2/s2;
    D2 = D2/s2;
  }
  else
  {
    // Cas degenere D1 = D2 = 0
  }

  if (cP.getCols() != 8)
    cP.resize(8);

  cP[0] =  A1;
  cP[1] =  B1;
  cP[2] =  C1;
  cP[3] =  D1;

  cP[4] =  A2;
  cP[5] =  B2;
  cP[6] =  C2;
  cP[7] =  D2;

  if (D2 < 0)
  {
    cP[4] *= -1 ;
    cP[5] *= -1 ;
    cP[6] *= -1 ;
    cP[7] *= -1 ;
  }
}


/*!

  Displays the line in the image I thanks to the 2D features and the
  camera parameters which enable to convert the features from meter to
  pixel.

  \param I : The image where the line must be displayed.

  \param cam : The camera parameters to enable the conversion from
  meter to pixel.

  \param color : The desired color to display the line in the image.
*/
void vpLine::display(vpImage<unsigned char> &I,
		     const vpCameraParameters &cam,
		     const vpColor::vpColorType color)
{
  vpFeatureDisplay::displayLine(p[0],p[1],
				cam, I, color) ;
}


/*!

  Displays the line in the image I thanks to the features in the
  object frame, the homogeneous matrix relative to the difference
  between the object frame and the camera frame and the camera
  parameters which enable to convert the features from meter to pixel.

  \param I : The image where the line must be displayed.

  \param cMo : The homogeneous matrix relative to the difference
  between the object frame and the camera frame.

  \param cam : The camera parameters to enable the conversion from
  meter to pixel.

  \param color : The desired color to display the line in the image.
*/
// non destructive wrt. cP and p
void vpLine::display(vpImage<unsigned char> &I,
		     const vpHomogeneousMatrix &cMo,
		     const vpCameraParameters &cam,
		     const vpColor::vpColorType color)
{
  vpColVector _cP, _p ;
  changeFrame(cMo,_cP) ;
  projection(_cP,_p) ;
  vpFeatureDisplay::displayLine(_p[0],_p[1],
				cam, I, color) ;

}


/*!
  Create an object with the same type.

  \code
  vpForwardProjection *fp;
  vpLine line;
  fp = line.duplicate(); // fp is now a vpLine
  \endcode

*/
vpLine *vpLine::duplicate() const
{
  vpLine *feature = new vpLine(*this) ;
  return feature ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
