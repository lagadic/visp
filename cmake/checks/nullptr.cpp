#include <cstddef>
#include <iostream>

void g(int *)
{
  std::cout << "Function g called" << std::endl;
}

int main()
{
  g(nullptr);
  g(NULL);
  g(0);
}
