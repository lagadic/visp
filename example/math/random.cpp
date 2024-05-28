#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpUniRand.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpUniRand rng;
  for (int i = 0; i < 10; i++) {
    std::cout << rng.uniform(0, 6) << std::endl; // produces int values
    std::cout << rng.uniform(0.0, 6.0) << std::endl; // produces double values
  }

  std::vector<int> v;
  for (unsigned int i = 0; i < 10; i++) {
    v.push_back(i);
  }

  std::vector<int> shuffled_v = vpUniRand::shuffleVector<int>(v);
  std::cout << "Original vector = [\t";
  for (unsigned int i = 0; i < 10; i++) {
    std::cout << v[i] << "\t";
  }
  std::cout << "]" <<  std::endl;

  std::cout << "Shuffled vector = [\t";
  for (unsigned int i = 0; i < 10; i++) {
    std::cout << shuffled_v[i] << "\t";
  }
  std::cout << "]" <<  std::endl;
}
