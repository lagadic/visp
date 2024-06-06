/* cpu_x86_Windows.ipp
 *
 * Author           : Alexander J. Yee
 * Date Created     : 04/12/2014
 * Last Modified    : 04/12/2014
 *
 * Modification for ViSP:
 *   - _xgetbv (MinGW)
 *   - __cpuidex (MinGW)
 *   - _xgetbv (Visual Studio 2010)
 */

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Dependencies
#include <Windows.h>
#include <intrin.h>
#include <memory.h>
#include "cpu_x86.h"
namespace FeatureDetector
{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//#if defined(_MSC_VER) && _MSC_VER <= 1600
#if defined(_MSC_FULL_VER) && _MSC_FULL_VER == 160040219 // == VS2010 SP1
//ref: https://github.com/webmproject/libwebp/blob/v0.6.0/src/dsp/cpu.c#L82
//ref: http://forums.codeguru.com/showthread.php?551499-xgetbv
//note: code to return the uint64_t value
//return ((uint64_t)edx_ << 32) | eax_;
//could be discard?
//ref: http://forums.codeguru.com/showthread.php?551499-xgetbv&s=aa59e5d6a36eb176c820406e707b42e4&p=2185193#post2185193
//ref: https://stackoverflow.com/a/25824252/6055233
uint64_t _xgetbv(unsigned int ext_ctrl_reg)
{
  uint32_t eax_, edx_;
  __asm {
    mov ecx, [ext_ctrl_reg]
    __asm _emit 0x0f __asm _emit 0x01 __asm _emit 0xd0 /* xgetbv() */
    mov eax_, eax
      mov edx_, edx
  }
  return ((uint64_t)edx_ << 32) | eax_;
}
#elif defined(_MSC_VER) && _MSC_VER <= 1600 // VS2010 too old
uint64_t _xgetbv(unsigned int)
{
  return 0U;
}
#endif
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
void __cpuidex(unsigned int CPUInfo[4], unsigned int function_id, unsigned int subfunction_id)
{
  __asm__ __volatile__(
     "cpuid"
     : "=a" (CPUInfo[0]), "=b" (CPUInfo[1]), "=c" (CPUInfo[2]), "=d" (CPUInfo[3])
     : "a" (function_id), "c" (subfunction_id));
}
#endif
void cpu_x86::cpuid(uint32_t out[4], uint32_t x)
{
#if defined(__MINGW32__)
  __cpuidex(out, x, 0U);
#else
  int32_t out_as_int[4];
  __cpuidex(out_as_int, x, 0U);
  memcpy(out, out_as_int, sizeof(int32_t) * 4);
#endif
}
__int64 xgetbv(unsigned int x)
{
  return _xgetbv(x);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Detect 64-bit - Note that this snippet of code for detecting 64-bit has been copied from MSDN.
typedef BOOL(WINAPI *LPFN_ISWOW64PROCESS) (HANDLE, PBOOL);
BOOL IsWow64()
{
  BOOL bIsWow64 = FALSE;

#if defined(__MINGW__) || defined(__MINGW32__) || defined(__MINGW64__)
  if (!IsWow64Process(GetCurrentProcess(), &bIsWow64)) {
    printf("Error Detecting Operating System.\n");
    printf("Defaulting to 32-bit OS.\n\n");
    bIsWow64 = FALSE;
  }
#elif !defined(WINRT) // Turned off on UWP where GetModuleHandle() doesn't exist
  LPFN_ISWOW64PROCESS fnIsWow64Process = (LPFN_ISWOW64PROCESS)GetProcAddress(
      GetModuleHandle(TEXT("kernel32")), "IsWow64Process");

  if (nullptr != fnIsWow64Process) {
    if (!fnIsWow64Process(GetCurrentProcess(), &bIsWow64)) {
      printf("Error Detecting Operating System.\n");
      printf("Defaulting to 32-bit OS.\n\n");
      bIsWow64 = FALSE;
    }
  }
#endif
  return bIsWow64;
}
bool cpu_x86::detect_OS_x64()
{
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
