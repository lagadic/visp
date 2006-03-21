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

  usleep(10);

  double t3 = vpTime::measureTimeMs();

  struct timespec req;
  req.tv_sec = 0;
  req.tv_nsec = 2*1000*1000;
  nanosleep(&req, NULL);

  double t4 = vpTime::measureTimeMs();

  vpTime::wait(t4, 19);

  double t5 = vpTime::measureTimeMs();

  vpTime::wait(5);

  double t6 = vpTime::measureTimeMs();

  vpTime::wait(21);

  double t7 = vpTime::measureTimeMs();

  cout << "t1-t0: computation: " << t1 - t0 << endl;
  cout << "t2-t0: wait(t, 40): " << t2 - t0 << endl;
  cout << "t2-t1: waiting time(t, 40): " << t2 - t1 << endl;
  cout << "t3-t2: usleep(10): " << t3 - t2 << endl;
  cout << "t4-t3: nanosleep(2): " << t4 - t3 << endl;
  cout << "t5-t4: wait(t, 19): " << t5 - t4 << endl;
  cout << "t6-t5: wait(5): " << t6 - t5 << endl;
  cout << "t7-t7: wait(21): " << t7 - t6 << endl;

  return 0;
}
