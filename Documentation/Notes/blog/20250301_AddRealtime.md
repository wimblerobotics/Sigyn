sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)

then append to /etc/security/limits.conf, before last line "#End of file"
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
