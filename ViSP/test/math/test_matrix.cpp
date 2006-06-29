/*!
  \example test_matrix.cpp

  Test some vpMatrix functionalities.
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTwistMatrix.h>


int main()
{
  vpTRACE("------------------------");
  vpTRACE("--- TEST PRETTY PRINT---");
  vpTRACE("------------------------");
  vpMatrix M ;
  M.eye(4);

  vpTRACE("call std::cout << M;");
  std::cout << M << endl;

  vpTRACE("call M.print (std::cout, 4);");
  M.print (std::cout, 4);

  vpTRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);
  M[1][0]=1.235;
  M[1][1]=12.345;
  M[1][2]=.12345;
  vpTRACE("call std::cout << M;");
  std::cout << M;
  vpTRACE("call M.print (std::cout, 6);");
  M.print (std::cout, 6);

  vpTRACE("------------------------");
  M[0][0]=-1.235;
  M[1][0]=-12.235;

  vpTRACE("call std::cout << M;");
  std::cout << M;

  vpTRACE("call M.print (std::cout, 10);");
  M.print (std::cout, 10);

  vpTRACE("call M.print (std::cout, 2);");
  M.print (std::cout, 2);

  vpTRACE("------------------------");
  M.resize(3,3) ;
  M.eye(3);
  M[0][2]=-0.0000000876;
  vpTRACE("call std::cout << M;");
  std::cout << M;

  vpTRACE("call M.print (std::cout, 4);");
  M.print (std::cout, 4);
  vpTRACE("call M.print (std::cout, 10, \"M\");");
  M.print (std::cout, 10, "M");
  vpTRACE("call M.print (std::cout, 20, \"M\");");
  M.print (std::cout, 20, "M");


  vpTRACE("------------------------");
  vpTRACE("--- TEST RESIZE --------");
  vpTRACE("------------------------");
  vpCTRACE  << "5x5" << endl;
  M.resize(5,5,false);
  vpCTRACE << endl<< M;
  vpCTRACE  << "3x2" << endl;
  M.resize(3,2,false);
  vpCTRACE <<endl<< M;
  vpCTRACE  << "2x2" << endl;
  M.resize(2,2,false);
  vpCTRACE << endl<<M;
  vpTRACE("------------------------");


  vpTwistMatrix vMe;
  vpMatrix A(1,6),B;

  A=1.0;
  //vMe=1.0;
  B=A*vMe;

  vpTRACE("------------------------");
  vpTRACE("--- TEST vpRowVector * vpColVector");
  vpTRACE("------------------------");
  vpRowVector r(3);
  r[0] = 2;
  r[1] = 3;
  r[2] = 4;

  vpColVector c(3);
  c[0] = 1;
  c[1] = 2;
  c[2] = -1;

  double rc = r * c;

  r.print(std::cout, 2, "r");
  c.print(std::cout, 2, "c");
  cout << "r * c = " << rc << endl;


}
