With TSD logging, main loop was 1025 per second.
Without TSD logging, main loop was 1047 per second.

With TSD logging, calling rclc_executor_spin_some every loop, 1025 per second.
calling every 10 loops, 11451 per second while doing nothing, 121 while moving with pd=10usec
calling every 10 loops, 11451 per second while doing nothing, 121 while moving with pd=20usec, about 8ms between loops
this is true whether sending a command or action request


With 1000usec pd, takes about 5sec to travel 0.4m
With 500usec pd, takes about 2.5 sec to travel 0.4m. Loop rate goes from 11363 down to about 825 loops/sec.