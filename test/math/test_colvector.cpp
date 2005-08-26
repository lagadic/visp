/*!
  \example test_colvector.cpp

  Test some vpColVector functionalities.
*/


#include <visp/vpColVector.h>
#include <visp/vpDebug.h>

int main()
{
  vpColVector V(4) ;
  V = 1.0;

  TRACE("------------------------");
  TRACE("call std::cout << V;");
  std::cout << V << endl;

  TRACE("------------------------");
  TRACE("call V.normalize();");
  V.normalize();

  TRACE("------------------------");
  TRACE("call std::cout << V;");
  std::cout << V << endl;

}
