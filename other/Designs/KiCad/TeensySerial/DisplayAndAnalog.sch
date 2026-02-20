EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title "Display and Analog"
Date "2021-02-09"
Rev "1"
Comp "Wimble Robotics"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 2150 1250 0    50   Input ~ 0
GND
Text GLabel 2150 2550 0    50   Input ~ 0
5V
Text GLabel 2150 1850 0    50   Input ~ 0
SCK
Text GLabel 2150 1950 0    50   BiDi ~ 0
MISO
Text GLabel 2150 2050 0    50   BiDi ~ 0
MOSI
Text GLabel 2150 2150 0    50   Input ~ 0
CS
Text GLabel 2150 2250 0    50   Input ~ 0
DC
Text GLabel 4350 1950 2    50   Input ~ 0
T_IRQ
Text GLabel 4350 2050 2    50   BiDi ~ 0
MISO
Text GLabel 4350 2150 2    50   BiDi ~ 0
MOSI
Text GLabel 4350 2250 2    50   Input ~ 0
T_CS
Text GLabel 4350 2350 2    50   Input ~ 0
SCK
$Comp
L TeensyMonitorV5-rescue:R_Small-Device R?
U 1 1 62354DB7
P 2450 1750
AR Path="/62354DB7" Ref="R?"  Part="1" 
AR Path="/623525E0/62354DB7" Ref="R51"  Part="1" 
F 0 "R51" V 2254 1750 50  0000 C CNN
F 1 "100" V 2345 1750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2450 1750 50  0001 C CNN
F 3 "~" H 2450 1750 50  0001 C CNN
F 4 "C22775" H 2450 1750 50  0001 C CNN "LCSC"
	1    2450 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 1750 2550 1750
Wire Wire Line
	2350 1750 2300 1750
Wire Wire Line
	2150 2250 2750 2250
Wire Wire Line
	2750 2150 2150 2150
Wire Wire Line
	2150 2050 2750 2050
Wire Wire Line
	2750 1950 2150 1950
Wire Wire Line
	2150 1850 2750 1850
$Comp
L TeensyMonitorV5-rescue:Conn_01x04-Connector_Generic J51
U 1 1 623553A8
P 3050 3900
AR Path="/623553A8" Ref="J51"  Part="1" 
AR Path="/623525E0/623553A8" Ref="J51"  Part="1" 
F 0 "J51" H 3130 3892 50  0000 L CNN
F 1 "ANALOGS" H 3130 3801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3050 3900 50  0001 C CNN
F 3 "~" H 3050 3900 50  0001 C CNN
F 4 "C124378" H 3050 3900 50  0001 C CNN "LCSC"
	1    3050 3900
	1    0    0    -1  
$EndComp
$Comp
L TeensyMonitorV5-rescue:Conn_01x04-Connector_Generic J52
U 1 1 62355BAF
P 4550 3900
AR Path="/62355BAF" Ref="J52"  Part="1" 
AR Path="/623525E0/62355BAF" Ref="J52"  Part="1" 
F 0 "J52" H 4630 3892 50  0000 L CNN
F 1 "3V3" H 4630 3801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4550 3900 50  0001 C CNN
F 3 "~" H 4550 3900 50  0001 C CNN
F 4 "C124378" H 4550 3900 50  0001 C CNN "LCSC"
	1    4550 3900
	1    0    0    -1  
$EndComp
$Comp
L TeensyMonitorV5-rescue:Conn_01x04-Connector_Generic J53
U 1 1 6235656A
P 5950 3900
AR Path="/6235656A" Ref="J53"  Part="1" 
AR Path="/623525E0/6235656A" Ref="J53"  Part="1" 
F 0 "J53" H 6030 3892 50  0000 L CNN
F 1 "GND" H 6030 3801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5950 3900 50  0001 C CNN
F 3 "~" H 5950 3900 50  0001 C CNN
F 4 "C124378" H 5950 3900 50  0001 C CNN "LCSC"
	1    5950 3900
	1    0    0    -1  
$EndComp
Text GLabel 2850 3800 0    50   Input ~ 0
ANLG0
Text GLabel 2850 3900 0    50   Input ~ 0
ANLG1
Text GLabel 2850 4000 0    50   Input ~ 0
ANLG2
Text GLabel 2850 4100 0    50   Input ~ 0
ANLG3
Text GLabel 4150 4300 0    50   Input ~ 0
3V3
Text GLabel 5450 4300 0    50   Input ~ 0
GND
Wire Wire Line
	4350 3800 4350 3900
Wire Wire Line
	4350 3900 4350 4000
Connection ~ 4350 3900
Wire Wire Line
	4350 4000 4350 4100
Connection ~ 4350 4000
Wire Wire Line
	4350 4100 4350 4300
Wire Wire Line
	4350 4300 4150 4300
Connection ~ 4350 4100
Wire Wire Line
	5750 3800 5750 3900
Wire Wire Line
	5750 3900 5750 4000
Connection ~ 5750 3900
Wire Wire Line
	5750 4000 5750 4100
Connection ~ 5750 4000
Wire Wire Line
	5750 4100 5750 4300
Wire Wire Line
	5750 4300 5450 4300
Connection ~ 5750 4100
$Comp
L TeensyMonitorV5-rescue:Conn_01x05-Connector_Generic J55
U 1 1 6235EC97
P 6850 2750
F 0 "J55" H 6930 2792 50  0000 L CNN
F 1 "MOTOR1" H 6930 2701 50  0000 L CNN
F 2 "Connector_Molex:533750510" H 6850 2750 50  0001 C CNN
F 3 "~" H 6850 2750 50  0001 C CNN
F 4 "C124379" H 6850 2750 50  0001 C CNN "LCSC"
	1    6850 2750
	1    0    0    -1  
$EndComp
Text GLabel 5550 1900 0    50   Input ~ 0
MOTOR0
Text GLabel 5550 2550 0    50   Input ~ 0
MOTOR1
Text GLabel 5550 2000 0    50   Input ~ 0
MISO
Text GLabel 5550 2100 0    50   Input ~ 0
SCK
Text GLabel 5550 2200 0    50   Input ~ 0
3V3
Text GLabel 5550 2300 0    50   Input ~ 0
GND
Wire Wire Line
	5550 1900 6650 1900
Wire Wire Line
	6650 2000 6200 2000
Wire Wire Line
	6650 2200 6400 2200
Wire Wire Line
	5550 2300 6500 2300
Wire Wire Line
	6650 2550 5550 2550
Wire Wire Line
	6650 2950 6500 2950
Wire Wire Line
	6500 2950 6500 2300
Connection ~ 6500 2300
Wire Wire Line
	6500 2300 6650 2300
Wire Wire Line
	6650 2850 6400 2850
Wire Wire Line
	6400 2850 6400 2200
Connection ~ 6400 2200
Wire Wire Line
	6400 2200 5550 2200
Wire Wire Line
	6650 2750 6300 2750
Wire Wire Line
	6300 2750 6300 2100
Connection ~ 6300 2100
Wire Wire Line
	6300 2100 6650 2100
Wire Wire Line
	6650 2650 6200 2650
Wire Wire Line
	6200 2650 6200 2000
Connection ~ 6200 2000
Wire Wire Line
	6200 2000 5550 2000
$Comp
L TeensyMonitorV5-rescue:CR2013-MI2120-Driver_Display U?
U 1 1 62354DA5
P 3550 1950
AR Path="/62354DA5" Ref="U?"  Part="1" 
AR Path="/623525E0/62354DA5" Ref="U50"  Part="1" 
F 0 "U50" H 3550 1050 50  0000 C CNN
F 1 "CR2013-MI2120" H 3550 1150 50  0000 C CNN
F 2 "Connector_Molex:Molex-533751410" H 3550 1250 50  0001 C CNN
F 3 "http://pan.baidu.com/s/11Y990" H 2900 2450 50  0001 C CNN
	1    3550 1950
	-1   0    0    1   
$EndComp
Text GLabel 2150 1750 0    50   Input ~ 0
3V3
$Comp
L TeensyMonitorV5-rescue:R-Device R17
U 1 1 60389F43
P 5750 3000
F 0 "R17" H 5820 3046 50  0000 L CNN
F 1 "R" H 5820 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5680 3000 50  0001 C CNN
F 3 "~" H 5750 3000 50  0001 C CNN
	1    5750 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2100 5750 2100
Wire Wire Line
	5750 2850 5750 2100
Connection ~ 5750 2100
Wire Wire Line
	5750 2100 6300 2100
Wire Wire Line
	5750 3150 4350 3150
Wire Wire Line
	4350 3150 4350 3800
Connection ~ 4350 3800
Wire Wire Line
	2150 2550 3550 2550
$Comp
L TeensyMonitorV5-rescue:Conn_01x05-Connector_Generic J54
U 1 1 6235DAD1
P 6850 2100
F 0 "J54" H 6930 2142 50  0000 L CNN
F 1 "MOTOR0" H 6930 2051 50  0000 L CNN
F 2 "Connector_Molex:533750510" H 6850 2100 50  0001 C CNN
F 3 "~" H 6850 2100 50  0001 C CNN
F 4 "C124379" H 6850 2100 50  0001 C CNN "LCSC"
	1    6850 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1650 2300 1650
Wire Wire Line
	2300 1650 2300 1750
Wire Wire Line
	2300 1750 2150 1750
Connection ~ 2300 1750
Wire Wire Line
	3550 1250 3550 1350
Wire Wire Line
	2150 1250 3550 1250
$EndSCHEMATC
