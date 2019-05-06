#if __cplusplus >= 201402L || (defined(_MSC_VER) && _MSC_VER >= 1900)
// OK
#else
#error "C++14 is not supported"
#endif

#include <utility>

auto func(int i)
{
    return [i=std::move(i)](int b){return b+i;};
} 

int main() 
{ 
    int num = func(3)(5);
}
