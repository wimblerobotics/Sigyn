EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLegal 14000 8500
encoding utf-8
Sheet 1 5
Title "Teensy Monitor"
Date "2021-02-08"
Rev "1"
Comp "Wimble Robotics"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 900  700  1350 300 
U 61D6939C
F0 "LevelShifters" 50
F1 "LevelShifters.sch" 50
$EndSheet
$Sheet
S 2400 700  1050 300 
U 620FF296
F0 "TimeOfFlightSensors" 50
F1 "TimeOfFlightSensors.sch" 50
$EndSheet
$Comp
L TeensyMonitorV5-rescue:Teensy4.1Wimble-teensy U1
U 1 1 621E1E02
P 4300 4450
F 0 "U1" H 4300 7015 50  0000 C CNN
F 1 "Teensy4.1Wimble" H 4300 6924 50  0000 C CNN
F 2 "teensy:Teensy41" H 3900 4850 50  0001 C CNN
F 3 "" H 3900 4850 50  0001 C CNN
	1    4300 4450
	1    0    0    -1  
$EndComp
Text GLabel 3200 2300 0    50   Input ~ 0
GND
Text GLabel 5400 2300 2    50   Input ~ 0
5V
Text GLabel 5400 2400 2    50   Input ~ 0
GND
Text GLabel 5400 2500 2    50   Input ~ 0
3V3
Text GLabel 5400 3000 2    50   Input ~ 0
SCL
Text GLabel 5400 3100 2    50   Input ~ 0
SDA
Text GLabel 3200 2400 0    50   Output ~ 0
RLY0
Text GLabel 3200 2500 0    50   Output ~ 0
RLY1
Text GLabel 3200 2600 0    50   Output ~ 0
RLY2
Text GLabel 3200 2700 0    50   Output ~ 0
RLY3
Text GLabel 3200 2800 0    50   Output ~ 0
RLY4
Text GLabel 3200 2900 0    50   Output ~ 0
RLY5
Text GLabel 3200 3000 0    50   Output ~ 0
RLY6
Text GLabel 3200 3100 0    50   Output ~ 0
RLY7
Text GLabel 5400 4500 2    50   Output ~ 0
TRG3
Text GLabel 5400 4400 2    50   Input ~ 0
ECHO3
Text GLabel 5400 4300 2    50   Output ~ 0
TRG2
Text GLabel 5400 3900 2    50   Output ~ 0
TRG1
Text GLabel 5400 3500 2    50   Output ~ 0
TRG0
Text GLabel 5400 4200 2    50   Input ~ 0
ECHO2
Text GLabel 5400 3800 2    50   Input ~ 0
ECHO1
Text GLabel 5400 3400 2    50   Input ~ 0
ECHO0
Text GLabel 5400 3700 2    50   Input ~ 0
GND
Text GLabel 5400 3600 2    50   Output ~ 0
SCK
Text GLabel 3200 3500 0    50   BiDi ~ 0
MOSI
Text GLabel 3200 3600 0    50   BiDi ~ 0
MISO
Text GLabel 3200 3700 0    50   Input ~ 0
3V3
Text GLabel 3200 3400 0    50   Output ~ 0
CS
Text GLabel 3200 3300 0    50   Output ~ 0
DC
Text GLabel 3200 4200 0    50   Input ~ 0
T_IRQ
Text GLabel 3200 4300 0    50   Output ~ 0
T_CS
Text GLabel 3200 3800 0    50   Input ~ 0
ANLG0
Text GLabel 3200 3900 0    50   Input ~ 0
ANLG1
Text GLabel 3200 4000 0    50   Input ~ 0
ANLG2
Text GLabel 3200 4100 0    50   Input ~ 0
ANLG3
Text GLabel 3200 4400 0    50   Output ~ 0
MOTOR0
Text GLabel 3200 4500 0    50   Output ~ 0
MOTOR1
Text GLabel 3200 3200 0    50   Output ~ 0
I2CRESET
$Sheet
S 3700 700  650  300 
U 6221804C
F0 "Relays" 50
F1 "Relays.sch" 50
$EndSheet
$Sheet
S 4600 700  1000 300 
U 623525E0
F0 "DisplayAndAnalog" 50
F1 "DisplayAndAnalog.sch" 50
$EndSheet
NoConn ~ 5400 2600
NoConn ~ 5400 2700
NoConn ~ 5400 2800
NoConn ~ 5400 2900
NoConn ~ 5400 3200
NoConn ~ 5400 3300
NoConn ~ 5400 4000
NoConn ~ 5400 4100
NoConn ~ 5400 4600
NoConn ~ 3200 4600
$Comp
L TeensyMonitorV5-rescue:Screw_Terminal_01x02-Connector J1
U 1 1 6241280D
P 7050 2300
F 0 "J1" H 7130 2292 50  0000 L CNN
F 1 "PWR" H 7130 2201 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 7050 2300 50  0001 C CNN
F 3 "~" H 7050 2300 50  0001 C CNN
F 4 "KF301-5.0-2P" H 7050 2300 50  0001 C CNN "LCSC"
	1    7050 2300
	1    0    0    -1  
$EndComp
Text GLabel 6850 2300 0    50   Input ~ 0
5V
Text GLabel 6850 2400 0    50   Input ~ 0
GND
$Comp
L TeensyMonitorV5-rescue:Conn_02x04_Odd_Even-Connector_Generic J2
U 1 1 624189CB
P 7000 3000
F 0 "J2" H 7050 3317 50  0000 C CNN
F 1 "5V" H 7050 3226 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 7000 3000 50  0001 C CNN
F 3 "~" H 7000 3000 50  0001 C CNN
F 4 "C124386" H 7000 3000 50  0001 C CNN "LCSC"
	1    7000 3000
	1    0    0    -1  
$EndComp
$Comp
L TeensyMonitorV5-rescue:Conn_02x04_Odd_Even-Connector_Generic J3
U 1 1 6241A719
P 7000 3700
F 0 "J3" H 7050 4017 50  0000 C CNN
F 1 "3V3" H 7050 3926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 7000 3700 50  0001 C CNN
F 3 "~" H 7000 3700 50  0001 C CNN
F 4 "C124386" H 7000 3700 50  0001 C CNN "LCSC"
	1    7000 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 3100 7300 3200
Wire Wire Line
	7300 3900 7300 3800
Wire Wire Line
	7300 3800 7300 3700
Connection ~ 7300 3800
Wire Wire Line
	7300 3700 7300 3600
Connection ~ 7300 3700
Wire Wire Line
	7300 3600 7300 3200
Connection ~ 7300 3600
Connection ~ 7300 3200
Wire Wire Line
	7300 3100 7300 3000
Connection ~ 7300 3100
Wire Wire Line
	7300 3000 7300 2900
Connection ~ 7300 3000
Wire Wire Line
	7300 2900 7300 2600
Connection ~ 7300 2900
Wire Wire Line
	6850 2400 6850 2600
Wire Wire Line
	6850 2600 7300 2600
Wire Wire Line
	6800 3200 6800 3100
Wire Wire Line
	6800 3100 6800 3000
Connection ~ 6800 3100
Wire Wire Line
	6800 3000 6800 2900
Connection ~ 6800 3000
Wire Wire Line
	6800 3900 6800 3800
Wire Wire Line
	6800 3800 6800 3700
Connection ~ 6800 3800
Wire Wire Line
	6800 3700 6800 3600
Connection ~ 6800 3700
Text GLabel 6800 2900 0    50   Input ~ 0
5V
Text GLabel 6800 3600 0    50   Input ~ 0
3V3
$EndSCHEMATC
