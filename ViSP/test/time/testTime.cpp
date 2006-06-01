#if defined UNIX
#  include <unistd.h>
#elif defined WIN32
#  include <windows.h>
#  include <mmsystem.h>
#  include <winbase.h>
#endif
#include <iostream>
#include <time.h>

#include <visp/vpTime.h>

using namespace std;

int main()

{
  double v = 0;

  double t0 = vpTime::measureTimeMs();
  for (int i =0 ; i < 100000; i ++)
    for (int j =0 ; j < 100; j ++)
      v = i * 2 / 3. + j;

  double t1 = vpTime::measureTimeMs();
  vpTime::wait(t0, 40);

  double t2 = vpTime::measureTimeMs();

  // Sleep 10ms
#if defined UNIX
  usleep(10*1000);
#elif defined WIN32
  Sleep(10);
#endif

  double t3 = vpTime::measureTimeMs();

  // Sleep 2ms
#if defined UNIX
  usleep(2*1000);
#elif defined WIN32
  Sleep(2);
#endif
  double t4 = vpTime::measureTimeMs();

  vpTime::wait(t4, 19);

  double t5 = vpTime::measureTimeMs();

  vpTime::wait(5);

  double t6 = vpTime::measureTimeMs();

  vpTime::wait(21);

  double t7 = vpTime::measureTimeMs();

  cout << "t1-t0: computation: " << t1 - t0 << endl;
  cout << "t2-t0: wait(t, 40 ms): " << t2 - t0 << endl;
  cout << "t2-t1: waiting time(t, 40): " << t2 - t1 << endl;
  cout << "t3-t2: sleep(10 ms): " << t3 - t2 << endl;
  cout << "t4-t3: sleep(2 ms): " << t4 - t3 << endl;
  cout << "t5-t4: wait(t, 19 ms): " << t5 - t4 << endl;
  cout << "t6-t5: wait(5 ms): " << t6 - t5 << endl;
  cout << "t7-t7: wait(21 ms): " << t7 - t6 << endl;

  return 0;
}
