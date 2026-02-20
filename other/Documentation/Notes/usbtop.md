```bash
sudo modprobe usbmon
```

```bash
ros@sigyn7900:~$ lsusb
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 001 Device 003: ID 2109:2817 VIA Labs, Inc. USB2.0 Hub             
Bus 001 Device 004: ID 0b05:1a5c ASUSTek Computer, Inc. USB Audio
Bus 001 Device 005: ID 2109:2817 VIA Labs, Inc. USB2.0 Hub             
Bus 001 Device 006: ID 16c0:0483 Van Ooijen Technische Informatica Teensyduino Serial
Bus 001 Device 007: ID 0b05:19af ASUSTek Computer, Inc. AURA LED Controller
Bus 001 Device 008: ID 05e3:0608 Genesys Logic, Inc. Hub
Bus 001 Device 009: ID 0489:e0e2 Foxconn / Hon Hai Wireless_Device
Bus 001 Device 010: ID 1bcf:08a0 Sunplus Innovation Technology Inc. Gaming mouse [Philips SPK9304]
Bus 001 Device 011: ID 1c4f:0002 SiGma Micro Keyboard TRACER Gamma Ivory
Bus 001 Device 012: ID 03e7:f63b Intel Myriad VPU [Movidius Neural Compute Stick]
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 005 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 005 Device 002: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
Bus 006 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 007 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 008 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
```

```bash
usbmon
Bus ID 0 (Raw USB traffic, all USB buses)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
  Device ID 2 :			0.00 kb/s	0.00 kb/s
  Device ID 3 :			0.00 kb/s	0.00 kb/s
  Device ID 4 :			0.00 kb/s	0.00 kb/s
  Device ID 5 :			0.00 kb/s	0.00 kb/s
  Device ID 6 :			23.30 kb/s	45.23 kb/s            ## <<< TEENSY 4.1
  Device ID 8 :			0.00 kb/s	0.00 kb/s
  Device ID 9 :			0.00 kb/s	0.00 kb/s
  Device ID 10 :			1.22 kb/s	1.36 kb/s
  Device ID 11 :			0.00 kb/s	0.00 kb/s
  Device ID 12 :			20.72 kb/s	61.68 kb/s          ## <<< OAK-D
Bus ID 1 (Raw USB traffic, bus number 1)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
  Device ID 3 :			0.00 kb/s	0.00 kb/s
  Device ID 4 :			0.00 kb/s	0.00 kb/s
  Device ID 5 :			0.00 kb/s	0.00 kb/s
  Device ID 6 :			23.30 kb/s	45.23 kb/s           ## <<< TEENSY 4.1
  Device ID 7 :			0.00 kb/s	0.00 kb/s
  Device ID 8 :			0.00 kb/s	0.00 kb/s
  Device ID 9 :			0.00 kb/s	0.00 kb/s
  Device ID 10 :			1.22 kb/s	1.36 kb/s
  Device ID 11 :			0.00 kb/s	0.00 kb/s
  Device ID 12 :			20.72 kb/s	61.68 kb/s         ## <<< OAK-D
Bus ID 2 (Raw USB traffic, bus number 2)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
Bus ID 3 (Raw USB traffic, bus number 3)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
Bus ID 4 (Raw USB traffic, bus number 4)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
Bus ID 5 (Raw USB traffic, bus number 5)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
  Device ID 2 :			4.51 kb/s	22.13 kb/s            ## <<< CP210x UART Bridge (LIDAR)
Bus ID 6 (Raw USB traffic, bus number 6)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
Bus ID 7 (Raw USB traffic, bus number 7)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
Bus ID 8 (Raw USB traffic, bus number 8)	To device	From device
  Device ID 1 :			0.00 kb/s	0.00 kb/s
```