/*!
  \example test_matrix.cpp

  Test some vpMatrix functionalities.
*/


#include <visp/vpMatrix.h>

int main()
{
  vpMatrix M ;
  M.eye(4);
  std::cout << M << endl;

  M.resize(3,3) ;
  M.eye(3);
  M[1][0]=1.235;
  M[1][1]=12.345;
  M[1][2]=.12345;
  std::cout << M << endl;
}
