/*!
  \example test_matrix.cpp

  Test some vpMatrix functionalities.
*/


#include <visp/vpMatrix.h>
#include <visp/vpDebug.h>

int main()
{
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

}
