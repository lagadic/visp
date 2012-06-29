set st da li

set title "Consigne vitesses camera [m/s m/s m/s rad/s rad/s rad/s]"
plot "/tmp/fspindle/log.dat" using 1 title "v1", "/tmp/fspindle/log.dat" using 2 title "v2", "/tmp/fspindle/log.dat" using 3 title "v3", "/tmp/fspindle/log.dat" using 4 title "v4", "/tmp/fspindle/log.dat" using 5 title "v5", "/tmp/fspindle/log.dat" using 6 title "v6"

pause -1

set title "Mesure vitesses articulaires [m/s m/s m/s rad/s rad/s rad/s]"
plot "/tmp/fspindle/log.dat" using 7 title "v1", "/tmp/fspindle/log.dat" using 8 title "v2", "/tmp/fspindle/log.dat" using 9 title "v3", "/tmp/fspindle/log.dat" using 10 title "v4", "/tmp/fspindle/log.dat" using 11 title "v5", "/tmp/fspindle/log.dat" using 12 title "v6"

pause -1

set title "Mesure positions articulaires [m m m rad rad rad]"
plot "/tmp/fspindle/log.dat" using 13 title "q1", "/tmp/fspindle/log.dat" using 14 title "q2", "/tmp/fspindle/log.dat" using 15 title "q3", "/tmp/fspindle/log.dat" using 16 title "q4", "/tmp/fspindle/log.dat" using 17 title "q5", "/tmp/fspindle/log.dat" using 18 title "q6"

pause -1

set title "s - s*"
plot "/tmp/fspindle/log.dat" using 19 title "x1", "/tmp/fspindle/log.dat" using 20 title "y1", "/tmp/fspindle/log.dat" using 21 title "x2", "/tmp/fspindle/log.dat" using 22 title "y2", "/tmp/fspindle/log.dat" using 23 title "x3", "/tmp/fspindle/log.dat" using 24 title "y3", "/tmp/fspindle/log.dat" using 25 title "x4", "/tmp/fspindle/log.dat" using 26 title "y4"

set title "cMo pose"
plot "/tmp/fspindle/log.dat" using 27 title "tx", "/tmp/fspindle/log.dat" using 28 title "ty", "/tmp/fspindle/log.dat" using 29 title "tz", "/tmp/fspindle/log.dat" using 30 title "rx", "/tmp/fspindle/log.dat" using 31 title "ry", "/tmp/fspindle/log.dat" using 32 title "rz"
