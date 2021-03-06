EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Driver_FET
LIBS:ac_motor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Q_NMOS_GDS Q1
U 1 1 5C770B48
P 5850 1800
F 0 "Q1" H 6050 1850 50  0000 L CNN
F 1 "irf3808" H 6050 1750 50  0000 L CNN
F 2 "" H 6050 1900 50  0001 C CNN
F 3 "" H 5850 1800 50  0001 C CNN
	1    5850 1800
	1    0    0    -1  
$EndComp
$Comp
L IR2104 U1
U 1 1 5C770D71
P 4800 2000
F 0 "U1" H 4850 2525 50  0000 L CNN
F 1 "IR2104" H 4850 2450 50  0000 L CNN
F 2 "" H 4800 2000 50  0001 C CIN
F 3 "" H 4800 2000 50  0001 C CNN
	1    4800 2000
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5C770E1E
P 5400 1800
F 0 "R3" V 5480 1800 50  0000 C CNN
F 1 "10" V 5400 1800 50  0000 C CNN
F 2 "" V 5330 1800 50  0001 C CNN
F 3 "" H 5400 1800 50  0001 C CNN
	1    5400 1800
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GDS Q2
U 1 1 5C770E94
P 5850 2300
F 0 "Q2" H 6050 2350 50  0000 L CNN
F 1 "irf3808" H 6050 2250 50  0000 L CNN
F 2 "" H 6050 2400 50  0001 C CNN
F 3 "" H 5850 2300 50  0001 C CNN
	1    5850 2300
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5C771082
P 5400 2300
F 0 "R4" V 5480 2300 50  0000 C CNN
F 1 "10" V 5400 2300 50  0000 C CNN
F 2 "" V 5330 2300 50  0001 C CNN
F 3 "" H 5400 2300 50  0001 C CNN
	1    5400 2300
	0    1    1    0   
$EndComp
$Comp
L D D3
U 1 1 5C7710FE
P 5000 1300
F 0 "D3" H 5000 1400 50  0000 C CNN
F 1 "uf4007" H 5000 1200 50  0000 C CNN
F 2 "" H 5000 1300 50  0001 C CNN
F 3 "" H 5000 1300 50  0001 C CNN
	1    5000 1300
	-1   0    0    1   
$EndComp
$Comp
L CP C5
U 1 1 5C771277
P 5400 1300
F 0 "C5" H 5425 1400 50  0000 L CNN
F 1 "220mk" H 5425 1200 50  0000 L CNN
F 2 "" H 5438 1150 50  0001 C CNN
F 3 "" H 5400 1300 50  0001 C CNN
	1    5400 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5100 1700 5200 1700
Wire Wire Line
	5200 1700 5200 1300
Wire Wire Line
	5150 1300 5250 1300
Connection ~ 5200 1300
Wire Wire Line
	5100 1800 5250 1800
Wire Wire Line
	5100 2300 5250 2300
Wire Wire Line
	5550 2300 5650 2300
Wire Wire Line
	5550 1800 5650 1800
Wire Wire Line
	5100 2200 5200 2200
Wire Wire Line
	5200 2200 5200 2050
Wire Wire Line
	5200 2050 7050 2050
Wire Wire Line
	5950 2000 5950 2100
Connection ~ 5950 2050
Wire Wire Line
	5550 1300 5600 1300
Wire Wire Line
	5600 1300 5600 2050
Connection ~ 5600 2050
Wire Wire Line
	4800 1300 4800 1500
Wire Wire Line
	3900 1300 4850 1300
$Comp
L CP C1
U 1 1 5C7718B2
P 4200 1500
F 0 "C1" H 4225 1600 50  0000 L CNN
F 1 "100mk" H 4225 1400 50  0000 L CNN
F 2 "" H 4238 1350 50  0001 C CNN
F 3 "" H 4200 1500 50  0001 C CNN
	1    4200 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4200 1250 4200 1350
Text GLabel 3850 2000 0    60   Input ~ 0
in1
Wire Wire Line
	4500 2000 3850 2000
Wire Wire Line
	5950 1600 5950 1250
Wire Wire Line
	5950 1250 7250 1250
$Comp
L Q_NMOS_GDS Q3
U 1 1 5C77262A
P 5850 3350
F 0 "Q3" H 6050 3400 50  0000 L CNN
F 1 "irf3808" H 6050 3300 50  0000 L CNN
F 2 "" H 6050 3450 50  0001 C CNN
F 3 "" H 5850 3350 50  0001 C CNN
	1    5850 3350
	1    0    0    -1  
$EndComp
$Comp
L IR2104 U2
U 1 1 5C772630
P 4800 3550
F 0 "U2" H 4850 4075 50  0000 L CNN
F 1 "IR2104" H 4850 4000 50  0000 L CNN
F 2 "" H 4800 3550 50  0001 C CIN
F 3 "" H 4800 3550 50  0001 C CNN
	1    4800 3550
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5C772636
P 5400 3350
F 0 "R5" V 5480 3350 50  0000 C CNN
F 1 "10" V 5400 3350 50  0000 C CNN
F 2 "" V 5330 3350 50  0001 C CNN
F 3 "" H 5400 3350 50  0001 C CNN
	1    5400 3350
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GDS Q4
U 1 1 5C77263C
P 5850 3850
F 0 "Q4" H 6050 3900 50  0000 L CNN
F 1 "irf3808" H 6050 3800 50  0000 L CNN
F 2 "" H 6050 3950 50  0001 C CNN
F 3 "" H 5850 3850 50  0001 C CNN
	1    5850 3850
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5C772642
P 5400 3850
F 0 "R6" V 5480 3850 50  0000 C CNN
F 1 "10" V 5400 3850 50  0000 C CNN
F 2 "" V 5330 3850 50  0001 C CNN
F 3 "" H 5400 3850 50  0001 C CNN
	1    5400 3850
	0    1    1    0   
$EndComp
$Comp
L D D4
U 1 1 5C772648
P 5000 2850
F 0 "D4" H 5000 2950 50  0000 C CNN
F 1 "uf4007" H 5000 2750 50  0000 C CNN
F 2 "" H 5000 2850 50  0001 C CNN
F 3 "" H 5000 2850 50  0001 C CNN
	1    5000 2850
	-1   0    0    1   
$EndComp
$Comp
L CP C6
U 1 1 5C77264E
P 5400 2850
F 0 "C6" H 5425 2950 50  0000 L CNN
F 1 "220mk" H 5425 2750 50  0000 L CNN
F 2 "" H 5438 2700 50  0001 C CNN
F 3 "" H 5400 2850 50  0001 C CNN
	1    5400 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5100 3250 5200 3250
Wire Wire Line
	5200 3250 5200 2850
Wire Wire Line
	5150 2850 5250 2850
Connection ~ 5200 2850
Wire Wire Line
	5100 3350 5250 3350
Wire Wire Line
	5100 3850 5250 3850
Wire Wire Line
	5550 3850 5650 3850
Wire Wire Line
	5550 3350 5650 3350
Wire Wire Line
	5100 3750 5200 3750
Wire Wire Line
	5200 3750 5200 3600
Wire Wire Line
	5200 3600 7050 3600
Wire Wire Line
	5950 3550 5950 3650
Connection ~ 5950 3600
Wire Wire Line
	5550 2850 5600 2850
Wire Wire Line
	5600 2850 5600 3600
Connection ~ 5600 3600
Wire Wire Line
	4800 3050 4800 2850
Wire Wire Line
	4200 2850 4850 2850
$Comp
L CP C2
U 1 1 5C772666
P 4200 3050
F 0 "C2" H 4225 3150 50  0000 L CNN
F 1 "100mk" H 4225 2950 50  0000 L CNN
F 2 "" H 4238 2900 50  0001 C CNN
F 3 "" H 4200 3050 50  0001 C CNN
	1    4200 3050
	-1   0    0    -1  
$EndComp
Text GLabel 3850 3550 0    60   Input ~ 0
in2
Wire Wire Line
	4500 3550 3850 3550
Wire Wire Line
	5950 3150 5950 2800
Wire Wire Line
	5950 2800 7250 2800
$Comp
L Q_NMOS_GDS Q5
U 1 1 5C7729E4
P 5850 4900
F 0 "Q5" H 6050 4950 50  0000 L CNN
F 1 "irf3808" H 6050 4850 50  0000 L CNN
F 2 "" H 6050 5000 50  0001 C CNN
F 3 "" H 5850 4900 50  0001 C CNN
	1    5850 4900
	1    0    0    -1  
$EndComp
$Comp
L IR2104 U3
U 1 1 5C7729EA
P 4800 5100
F 0 "U3" H 4850 5625 50  0000 L CNN
F 1 "IR2104" H 4850 5550 50  0000 L CNN
F 2 "" H 4800 5100 50  0001 C CIN
F 3 "" H 4800 5100 50  0001 C CNN
	1    4800 5100
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5C7729F0
P 5400 4900
F 0 "R7" V 5480 4900 50  0000 C CNN
F 1 "10" V 5400 4900 50  0000 C CNN
F 2 "" V 5330 4900 50  0001 C CNN
F 3 "" H 5400 4900 50  0001 C CNN
	1    5400 4900
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GDS Q6
U 1 1 5C7729F6
P 5850 5400
F 0 "Q6" H 6050 5450 50  0000 L CNN
F 1 "irf3808" H 6050 5350 50  0000 L CNN
F 2 "" H 6050 5500 50  0001 C CNN
F 3 "" H 5850 5400 50  0001 C CNN
	1    5850 5400
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5C7729FC
P 5400 5400
F 0 "R8" V 5480 5400 50  0000 C CNN
F 1 "10" V 5400 5400 50  0000 C CNN
F 2 "" V 5330 5400 50  0001 C CNN
F 3 "" H 5400 5400 50  0001 C CNN
	1    5400 5400
	0    1    1    0   
$EndComp
$Comp
L D D5
U 1 1 5C772A02
P 5000 4400
F 0 "D5" H 5000 4500 50  0000 C CNN
F 1 "uf4007" H 5000 4300 50  0000 C CNN
F 2 "" H 5000 4400 50  0001 C CNN
F 3 "" H 5000 4400 50  0001 C CNN
	1    5000 4400
	-1   0    0    1   
$EndComp
$Comp
L CP C7
U 1 1 5C772A08
P 5400 4400
F 0 "C7" H 5425 4500 50  0000 L CNN
F 1 "220mk" H 5425 4300 50  0000 L CNN
F 2 "" H 5438 4250 50  0001 C CNN
F 3 "" H 5400 4400 50  0001 C CNN
	1    5400 4400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5100 4800 5200 4800
Wire Wire Line
	5200 4800 5200 4400
Wire Wire Line
	5150 4400 5250 4400
Connection ~ 5200 4400
Wire Wire Line
	5100 4900 5250 4900
Wire Wire Line
	5100 5400 5250 5400
Wire Wire Line
	5550 5400 5650 5400
Wire Wire Line
	5550 4900 5650 4900
Wire Wire Line
	5100 5300 5200 5300
Wire Wire Line
	5200 5300 5200 5150
Wire Wire Line
	5200 5150 7050 5150
Wire Wire Line
	5950 5100 5950 5200
Connection ~ 5950 5150
Wire Wire Line
	5550 4400 5600 4400
Wire Wire Line
	5600 4400 5600 5150
Connection ~ 5600 5150
Wire Wire Line
	4800 4600 4800 4400
Wire Wire Line
	4200 4400 4850 4400
$Comp
L CP C3
U 1 1 5C772A20
P 4200 4600
F 0 "C3" H 4225 4700 50  0000 L CNN
F 1 "100mk" H 4225 4500 50  0000 L CNN
F 2 "" H 4238 4450 50  0001 C CNN
F 3 "" H 4200 4600 50  0001 C CNN
	1    4200 4600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4200 4350 4200 4450
Text GLabel 3850 5100 0    60   Input ~ 0
in3
Wire Wire Line
	4500 5100 3850 5100
Wire Wire Line
	5950 4700 5950 4350
Wire Wire Line
	5950 4350 7250 4350
Wire Wire Line
	6750 1250 6750 4350
Connection ~ 6750 2800
Connection ~ 6750 1250
Connection ~ 6750 4350
Text GLabel 7250 1250 2    60   Input ~ 0
+A
Text GLabel 7250 2800 2    60   Input ~ 0
+B
Text GLabel 7250 4350 2    60   Input ~ 0
+C
Text GLabel 7050 5150 2    60   Input ~ 0
phase 3
Text GLabel 7050 3600 2    60   Input ~ 0
phase 2
Text GLabel 7050 2050 2    60   Input ~ 0
phase 1
Wire Wire Line
	4500 2100 4450 2100
Wire Wire Line
	4450 2100 4450 5200
Wire Wire Line
	4450 3650 4500 3650
Wire Wire Line
	3850 5200 4500 5200
Connection ~ 4450 3650
Text GLabel 3850 5200 0    60   Input ~ 0
sd
Connection ~ 4450 5200
$Comp
L GND #PWR12
U 1 1 5C775422
P 4800 5600
F 0 "#PWR12" H 4800 5350 50  0001 C CNN
F 1 "GND" H 4800 5450 50  0000 C CNN
F 2 "" H 4800 5600 50  0001 C CNN
F 3 "" H 4800 5600 50  0001 C CNN
	1    4800 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 1650 4200 1650
Wire Wire Line
	5950 2500 5950 2550
Wire Wire Line
	5950 2550 6400 2550
Wire Wire Line
	6400 2450 6400 5700
Wire Wire Line
	6400 5650 5950 5650
Wire Wire Line
	5950 5650 5950 5600
Wire Wire Line
	5950 4050 5950 4100
Wire Wire Line
	5950 4100 6400 4100
Connection ~ 6400 4100
$Comp
L R_Shunt R11
U 1 1 5C777CAF
P 6400 5900
F 0 "R11" V 6225 5900 50  0000 C CNN
F 1 "50A" V 6300 5900 50  0000 C CNN
F 2 "" V 6330 5900 50  0001 C CNN
F 3 "" H 6400 5900 50  0001 C CNN
	1    6400 5900
	-1   0    0    1   
$EndComp
Connection ~ 6400 5650
Wire Wire Line
	6400 6100 6400 6200
Wire Wire Line
	6400 6150 6950 6150
Text GLabel 6950 6150 2    60   Input ~ 0
-A,-B,-C
$Comp
L R R10
U 1 1 5C77831D
P 6400 2300
F 0 "R10" V 6480 2300 50  0000 C CNN
F 1 "47k" V 6400 2300 50  0000 C CNN
F 2 "" V 6330 2300 50  0001 C CNN
F 3 "" H 6400 2300 50  0001 C CNN
	1    6400 2300
	1    0    0    -1  
$EndComp
Connection ~ 6400 2550
Wire Wire Line
	6400 2150 6400 2050
Connection ~ 6400 2050
$Comp
L LM358 U4
U 1 1 5C779A74
P 4850 6150
F 0 "U4" H 4850 6350 50  0000 L CNN
F 1 "LM358" H 4850 5950 50  0000 L CNN
F 2 "" H 4850 6150 50  0001 C CNN
F 3 "" H 4850 6150 50  0001 C CNN
	1    4850 6150
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR15
U 1 1 5C779F54
P 6400 6200
F 0 "#PWR15" H 6400 5950 50  0001 C CNN
F 1 "GND" H 6400 6050 50  0000 C CNN
F 2 "" H 6400 6200 50  0001 C CNN
F 3 "" H 6400 6200 50  0001 C CNN
	1    6400 6200
	1    0    0    -1  
$EndComp
Connection ~ 6400 6150
$Comp
L R R9
U 1 1 5C77A8E3
P 5750 6050
F 0 "R9" V 5830 6050 50  0000 C CNN
F 1 "100" V 5750 6050 50  0000 C CNN
F 2 "" V 5680 6050 50  0001 C CNN
F 3 "" H 5750 6050 50  0001 C CNN
	1    5750 6050
	0    1    1    0   
$EndComp
$Comp
L POT RV1
U 1 1 5C77AA2E
P 5750 6800
F 0 "RV1" V 5575 6800 50  0000 C CNN
F 1 "1k" V 5650 6800 50  0000 C CNN
F 2 "" H 5750 6800 50  0001 C CNN
F 3 "" H 5750 6800 50  0001 C CNN
	1    5750 6800
	0    -1   -1   0   
$EndComp
$Comp
L CP C8
U 1 1 5C77AD08
P 5400 6250
F 0 "C8" H 5425 6350 50  0000 L CNN
F 1 "220mk" H 5425 6150 50  0000 L CNN
F 2 "" H 5438 6100 50  0001 C CNN
F 3 "" H 5400 6250 50  0001 C CNN
	1    5400 6250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 5C77B143
P 4950 6450
F 0 "#PWR13" H 4950 6200 50  0001 C CNN
F 1 "GND" H 4950 6300 50  0000 C CNN
F 2 "" H 4950 6450 50  0001 C CNN
F 3 "" H 4950 6450 50  0001 C CNN
	1    4950 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 6450 4950 6450
$Comp
L +5V #PWR8
U 1 1 5C77B250
P 4350 5800
F 0 "#PWR8" H 4350 5650 50  0001 C CNN
F 1 "+5V" H 4350 5940 50  0000 C CNN
F 2 "" H 4350 5800 50  0001 C CNN
F 3 "" H 4350 5800 50  0001 C CNN
	1    4350 5800
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5C77B4BA
P 4850 6800
F 0 "R2" V 4930 6800 50  0000 C CNN
F 1 "20k" V 4850 6800 50  0000 C CNN
F 2 "" V 4780 6800 50  0001 C CNN
F 3 "" H 4850 6800 50  0001 C CNN
	1    4850 6800
	0    1    1    0   
$EndComp
Wire Wire Line
	4500 6800 4700 6800
Wire Wire Line
	4500 6150 4500 6800
Wire Wire Line
	3700 6150 4550 6150
Wire Wire Line
	5000 6800 5600 6800
Wire Wire Line
	5200 6800 5200 6250
Wire Wire Line
	5200 6250 5150 6250
$Comp
L GND #PWR14
U 1 1 5C77B8A1
P 5400 6400
F 0 "#PWR14" H 5400 6150 50  0001 C CNN
F 1 "GND" H 5400 6250 50  0000 C CNN
F 2 "" H 5400 6400 50  0001 C CNN
F 3 "" H 5400 6400 50  0001 C CNN
	1    5400 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 6050 5600 6050
Wire Wire Line
	5400 6100 5400 6050
Connection ~ 5400 6050
Wire Wire Line
	5900 6050 5950 6050
Wire Wire Line
	5950 6050 5950 5800
Wire Wire Line
	5950 5800 6250 5800
Wire Wire Line
	6250 6000 6050 6000
Wire Wire Line
	6050 6000 6050 6800
Wire Wire Line
	6050 6800 5900 6800
Connection ~ 5200 6800
Wire Wire Line
	3750 5850 4950 5850
Wire Wire Line
	4350 5800 4350 6800
$Comp
L R R1
U 1 1 5C77CD5B
P 4100 6400
F 0 "R1" V 4180 6400 50  0000 C CNN
F 1 "120" V 4100 6400 50  0000 C CNN
F 2 "" V 4030 6400 50  0001 C CNN
F 3 "" H 4100 6400 50  0001 C CNN
	1    4100 6400
	0    1    1    0   
$EndComp
$Comp
L D D1
U 1 1 5C77D206
P 3850 6600
F 0 "D1" H 3850 6700 50  0000 C CNN
F 1 "1n4148" H 3850 6500 50  0000 C CNN
F 2 "" H 3850 6600 50  0001 C CNN
F 3 "" H 3850 6600 50  0001 C CNN
	1    3850 6600
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR1
U 1 1 5C77D5F4
P 3850 7100
F 0 "#PWR1" H 3850 6850 50  0001 C CNN
F 1 "GND" H 3850 6950 50  0000 C CNN
F 2 "" H 3850 7100 50  0001 C CNN
F 3 "" H 3850 7100 50  0001 C CNN
	1    3850 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 6800 3850 6750
Wire Wire Line
	3850 6450 3850 6400
Wire Wire Line
	3700 6400 3950 6400
Connection ~ 4500 6400
Connection ~ 4500 6150
Text GLabel 3700 6150 0    60   Input ~ 0
A1
Text GLabel 3700 6400 0    60   Input ~ 0
A2
Connection ~ 3850 6400
$Comp
L D D2
U 1 1 5C77DCCC
P 3850 6950
F 0 "D2" H 3850 7050 50  0000 C CNN
F 1 "1n4148" H 3850 6850 50  0000 C CNN
F 2 "" H 3850 6950 50  0001 C CNN
F 3 "" H 3850 6950 50  0001 C CNN
	1    3850 6950
	0    -1   -1   0   
$EndComp
$Comp
L C C4
U 1 1 5C77DFAA
P 4350 6950
F 0 "C4" H 4375 7050 50  0000 L CNN
F 1 "0.1mk" H 4375 6850 50  0000 L CNN
F 2 "" H 4388 6800 50  0001 C CNN
F 3 "" H 4350 6950 50  0001 C CNN
	1    4350 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR9
U 1 1 5C77E309
P 4350 7100
F 0 "#PWR9" H 4350 6850 50  0001 C CNN
F 1 "GND" H 4350 6950 50  0000 C CNN
F 2 "" H 4350 7100 50  0001 C CNN
F 3 "" H 4350 7100 50  0001 C CNN
	1    4350 7100
	1    0    0    -1  
$EndComp
Connection ~ 4350 5850
Wire Wire Line
	4250 6400 4500 6400
Wire Wire Line
	5750 6650 6050 6650
Connection ~ 6050 6650
$Comp
L C C9
U 1 1 5C773171
P 6450 1500
F 0 "C9" H 6475 1600 50  0000 L CNN
F 1 "0.1mk" H 6475 1400 50  0000 L CNN
F 2 "" H 6488 1350 50  0001 C CNN
F 3 "" H 6450 1500 50  0001 C CNN
	1    6450 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1350 6450 1250
Connection ~ 6450 1250
$Comp
L GND #PWR16
U 1 1 5C773747
P 6450 1650
F 0 "#PWR16" H 6450 1400 50  0001 C CNN
F 1 "GND" H 6450 1500 50  0000 C CNN
F 2 "" H 6450 1650 50  0001 C CNN
F 3 "" H 6450 1650 50  0001 C CNN
	1    6450 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 5C7837A0
P 4800 2500
F 0 "#PWR10" H 4800 2250 50  0001 C CNN
F 1 "GND" H 4800 2350 50  0000 C CNN
F 2 "" H 4800 2500 50  0001 C CNN
F 3 "" H 4800 2500 50  0001 C CNN
	1    4800 2500
	1    0    0    -1  
$EndComp
Connection ~ 4800 1300
Connection ~ 4800 2850
Connection ~ 4800 4400
$Comp
L GND #PWR3
U 1 1 5C7842CF
P 4200 1650
F 0 "#PWR3" H 4200 1400 50  0001 C CNN
F 1 "GND" H 4200 1500 50  0000 C CNN
F 2 "" H 4200 1650 50  0001 C CNN
F 3 "" H 4200 1650 50  0001 C CNN
	1    4200 1650
	1    0    0    -1  
$EndComp
$Comp
L +12P #PWR2
U 1 1 5C784524
P 4200 1250
F 0 "#PWR2" H 4200 1100 50  0001 C CNN
F 1 "+12P" H 4200 1390 50  0000 C CNN
F 2 "" H 4200 1250 50  0001 C CNN
F 3 "" H 4200 1250 50  0001 C CNN
	1    4200 1250
	1    0    0    -1  
$EndComp
Connection ~ 4200 1300
$Comp
L GND #PWR11
U 1 1 5C786D2B
P 4800 4050
F 0 "#PWR11" H 4800 3800 50  0001 C CNN
F 1 "GND" H 4800 3900 50  0000 C CNN
F 2 "" H 4800 4050 50  0001 C CNN
F 3 "" H 4800 4050 50  0001 C CNN
	1    4800 4050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 5C7870C8
P 4200 3200
F 0 "#PWR5" H 4200 2950 50  0001 C CNN
F 1 "GND" H 4200 3050 50  0000 C CNN
F 2 "" H 4200 3200 50  0001 C CNN
F 3 "" H 4200 3200 50  0001 C CNN
	1    4200 3200
	1    0    0    -1  
$EndComp
$Comp
L +12P #PWR4
U 1 1 5C787482
P 4200 2800
F 0 "#PWR4" H 4200 2650 50  0001 C CNN
F 1 "+12P" H 4200 2940 50  0000 C CNN
F 2 "" H 4200 2800 50  0001 C CNN
F 3 "" H 4200 2800 50  0001 C CNN
	1    4200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2800 4200 2900
Connection ~ 4200 2850
$Comp
L GND #PWR7
U 1 1 5C78977B
P 4200 4750
F 0 "#PWR7" H 4200 4500 50  0001 C CNN
F 1 "GND" H 4200 4600 50  0000 C CNN
F 2 "" H 4200 4750 50  0001 C CNN
F 3 "" H 4200 4750 50  0001 C CNN
	1    4200 4750
	1    0    0    -1  
$EndComp
$Comp
L +12P #PWR6
U 1 1 5C78995A
P 4200 4350
F 0 "#PWR6" H 4200 4200 50  0001 C CNN
F 1 "+12P" H 4200 4490 50  0000 C CNN
F 2 "" H 4200 4350 50  0001 C CNN
F 3 "" H 4200 4350 50  0001 C CNN
	1    4200 4350
	1    0    0    -1  
$EndComp
Connection ~ 4200 4400
Text GLabel 3750 5850 0    60   Input ~ 0
+5v
Text GLabel 3900 1300 0    60   Input ~ 0
+12v
$Sheet
S 8950 1150 1300 800 
U 5C7CF8C0
F0 "precharge" 60
F1 "precharge.sch" 60
F2 "+akk" I R 10250 1400 60 
F3 "-akk" I R 10250 1750 60 
F4 "+A" I L 8950 1250 60 
F5 "+B" I L 8950 1350 60 
F6 "+C" I L 8950 1450 60 
F7 "-B" I L 8950 1650 60 
F8 "-B" I L 8950 1750 60 
F9 "-C" I L 8950 1850 60 
$EndSheet
$EndSCHEMATC
