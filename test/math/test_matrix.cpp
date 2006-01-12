/*!
  \example test_matrix.cpp

  Test some vpMatrix functionalities.
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpImageIo.h>

#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpPose.h>

int main()
{
  TRACE("------------------------");
  TRACE("--- TEST PRETTY PRINT---");
  TRACE("------------------------");
  vpMatrix M ;
  M.eye(4);

  TRACE("call std::cout << M;");
  std::cout << M << endl;

  TRACE("call M.print (std::cout, 4);");
  M.print (std::cout, 4);

  TRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);
  M[1][0]=1.235;
  M[1][1]=12.345;
  M[1][2]=.12345;
  TRACE("call std::cout << M;");
  std::cout << M;
  TRACE("call M.print (std::cout, 6);");
  M.print (std::cout, 6);

  TRACE("------------------------");
  M[0][0]=-1.235;
  M[1][0]=-12.235;

  TRACE("call std::cout << M;");
  std::cout << M;

  TRACE("call M.print (std::cout, 10);");
  M.print (std::cout, 10);

  TRACE("call M.print (std::cout, 2);");
  M.print (std::cout, 2);

  TRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);
  M[0][2]=-0.0000000876;
  TRACE("call std::cout << M;");
  std::cout << M;

  TRACE("call M.print (std::cout, 4);");
  M.print (std::cout, 4);
  TRACE("call M.print (std::cout, 10, \"M\");");
  M.print (std::cout, 10, "M");
  TRACE("call M.print (std::cout, 20, \"M\");");
  M.print (std::cout, 20, "M");


  TRACE("------------------------");
  TRACE("--- TEST RESIZE --------");
  TRACE("------------------------");
  CTRACE  << "5x5" << endl;
  M.resize(5,5,false);
  CTRACE << endl<< M;
  CTRACE  << "3x2" << endl;
  M.resize(3,2,false);
  CTRACE <<endl<< M;
  CTRACE  << "2x2" << endl;
  M.resize(2,2,false);
  CTRACE << endl<<M;
  TRACE("------------------------");


  vpTwistMatrix vMe;
  vpMatrix A,B;
  B=A*vMe;


}
