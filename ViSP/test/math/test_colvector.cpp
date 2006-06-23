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

  vpTRACE("------------------------");
  vpTRACE("call std::cout << V;");
  std::cout << V << endl;

  vpTRACE("------------------------");
  vpTRACE("call V.normalize();");
  V.normalize();

  vpTRACE("------------------------");
  vpTRACE("call std::cout << V;");
  std::cout << V << endl;

}
