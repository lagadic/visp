/* cpu_x86_Windows.ipp
 * 
 * Author           : Alexander J. Yee
 * Date Created     : 04/12/2014
 * Last Modified    : 04/12/2014
 *
 * Modification for ViSP:
 *   - _xgetbv (MinGW)
 *   - __cpuidex (MinGW)
 */

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Dependencies
#include <Windows.h>
#include <intrin.h>
#include "cpu_x86.h"
namespace FeatureDetector{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if defined(__MINGW32__)
unsigned __int64 _xgetbv(unsigned int index)
{
#if defined(__x86_64__) || defined(_AMD64_)
   unsigned __int64 val1, val2;
#else
   unsigned __LONG32 val1, val2;
#endif /* defined(__x86_64__) || defined(_AMD64_) */

   __asm__ __volatile__(
      "xgetbv"
      : "=a" (val1), "=d" (val2)
      : "c" (index));

   return (((unsigned __int64)val2) << 32) | val1;
}
#endif
#if defined(__MINGW32__)
void __cpuidex(int CPUInfo[4], int function_id, int subfunction_id) {
   __asm__ __volatile__ (
      "cpuid"
      : "=a" (CPUInfo [0]), "=b" (CPUInfo [1]), "=c" (CPUInfo [2]), "=d" (CPUInfo [3])
      : "a" (function_id), "c" (subfunction_id));
}
#endif
void cpu_x86::cpuid(int32_t out[4], int32_t x){
    __cpuidex(out, x, 0);
}
__int64 xgetbv(unsigned int x){
    return _xgetbv(x);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Detect 64-bit - Note that this snippet of code for detecting 64-bit has been copied from MSDN.
typedef BOOL (WINAPI *LPFN_ISWOW64PROCESS) (HANDLE, PBOOL);
BOOL IsWow64()
{
    BOOL bIsWow64 = FALSE;

    LPFN_ISWOW64PROCESS fnIsWow64Process = (LPFN_ISWOW64PROCESS) GetProcAddress(
        GetModuleHandle(TEXT("kernel32")), "IsWow64Process");

    if (NULL != fnIsWow64Process)
    {
        if (!fnIsWow64Process(GetCurrentProcess(), &bIsWow64))
        {
            printf("Error Detecting Operating System.\n");
            printf("Defaulting to 32-bit OS.\n\n");
            bIsWow64 = FALSE;
        }
    }
    return bIsWow64;
}
bool cpu_x86::detect_OS_x64(){
#ifdef _M_X64
    return true;
#else
    return IsWow64() != 0;
#endif
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}
