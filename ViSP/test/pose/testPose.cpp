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
 * This file is part of the ViSP toolkit
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
 * Compute the pose of a 3D object using the Dementhon, Lagrange and
 * Non-Linear approach.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

#define L 0.035

/*!
  \example testPose.cpp

  Compute the pose of a 3D object using the Dementhon, Lagrange and
  Non-Linear approach.

*/
/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Compute the pose of a 3D object using the Dementhon, Lagrange and\n\
Non-Linear approach.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }


  vpPoint P[4]  ;  //  Point to be tracked
  vpPose pose ;
  pose.clearPoint() ;


  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;

  vpRotationMatrix cRo;
  vpTranslationVector cto;
  vpRxyzVector cro;
  vpHomogeneousMatrix cMo_ref(0.1,0.2,1,vpMath::rad(10),0,vpMath::rad(10)) ;
  int i ;
  for(i=0 ; i < 4 ; i++)
    P[i].project(cMo_ref) ;


  for (i=0 ; i < 4 ; i++)
  {
    pose.addPoint(P[i]) ; // and added to the pose computation class
  }

  // Let's go ...
  vpHomogeneousMatrix cMo ;

  std::cout <<"-------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  vpTRACE("LAGRANGE pose : ") ;
  std::cout << cMo << std::endl ;
  std::cout << "Lagrange residual term: " << pose.computeResidual(cMo) <<std::endl ;
  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  double residual = pose.computeResidual(cMo);

  std::cout << "\nPose Lagrange "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;


  std::cout <<"--------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  vpTRACE( "LOWE pose :" ) ;
  std::cout <<  cMo << std::endl ;
  std::cout << "Lowe residual term: " <<pose.computeResidual(cMo) <<std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose Lowe "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;

  std::cout <<"--------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  vpTRACE(  "DEMENTHON pose :" ) ;
  std::cout <<  cMo << std::endl ;
  std::cout << "Dementhon residual term: " << pose.computeResidual(cMo)
	    << std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose Dementhon "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;

  std::cout <<"--------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::LOWE, cMo) ;
  std::cout << "Lowe residual term: " <<pose.computeResidual(cMo) <<std::endl ;
  //  vpTRACE(  "Pose LOWE"  ) ;
  //  std::cout <<  cMo << std::endl ;
  //  std::cout << "residu Lowe " << pose.computeResidual(cMo) <<std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose lowe "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;

  std::cout <<std::endl << std::endl ;
  std::cout <<"--------------------------------------------------"<<std::endl ;
  std::cout << "Virtual Visual servoing " << std::endl ;

  std::cout <<"--------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  vpTRACE("LAGRANGE pose : ") ;
  std::cout << cMo << std::endl ;
  std::cout << "Lagrange residual term: " << pose.computeResidual(cMo)
	    <<std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose Lagrange "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;


  std::cout <<"--------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  vpTRACE( "VIRTUAL_VS pose :" ) ;
  std::cout <<  cMo << std::endl ;
  std::cout << "vvs residual term: " <<pose.computeResidual(cMo) <<std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose VVS "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;

  std::cout <<"-------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::DEMENTHON, cMo) ;
  vpTRACE(  "DEMENTHON pose :" ) ;
  std::cout <<  cMo << std::endl ;
  std::cout << "Dementhon residual term: " << pose.computeResidual(cMo)
	    <<std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose Dementhon "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;

  std::cout <<"-------------------------------------------------"<<std::endl ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  std::cout << "vvs residual term: " <<pose.computeResidual(cMo) <<std::endl ;

  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
  residual = pose.computeResidual(cMo);

  std::cout << "\nPose VVS "
	    << "(residual: " << residual << ")\n "
	    << "cdto[0] = " << cto[0] << ";\n "
	    << "cdto[1] = " << cto[1] << ";\n "
	    << "cdto[2] = " << cto[2] << ";\n "
	    << "cdro[0] = vpMath::rad(" << vpMath::deg(cro[0]) << ");\n "
	    << "cdro[1] = vpMath::rad(" << vpMath::deg(cro[1]) << ");\n "
	    << "cdro[2] = vpMath::rad(" << vpMath::deg(cro[2]) << ");\n"
	    << std::endl;
  return 0;
}
