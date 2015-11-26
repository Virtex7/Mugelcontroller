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
LIBS:ph-motor
LIBS:Resonator_3pins
LIBS:A4960 Devboard-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L A4960 U2
U 1 1 55EC77AA
P 5950 3550
F 0 "U2" H 5400 2600 60  0000 C CNN
F 1 "A4960" H 5400 2500 60  0000 C CNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 5400 2500 60  0001 C CNN
F 3 "" H 5400 2500 60  0000 C CNN
	1    5950 3550
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 55EC79BC
P 8650 2700
F 0 "C17" H 8675 2800 50  0000 L CNN
F 1 "100n" H 8675 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8688 2550 30  0001 C CNN
F 3 "" H 8650 2700 60  0000 C CNN
	1    8650 2700
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 55EC7A21
P 8300 2700
F 0 "R14" V 8380 2700 50  0000 C CNN
F 1 "10" V 8300 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 2700 30  0001 C CNN
F 3 "" H 8300 2700 30  0000 C CNN
	1    8300 2700
	0    1    1    0   
$EndComp
$Comp
L R R15
U 1 1 55EC7A67
P 8300 3100
F 0 "R15" V 8380 3100 50  0000 C CNN
F 1 "10" V 8300 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 3100 30  0001 C CNN
F 3 "" H 8300 3100 30  0000 C CNN
	1    8300 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	8650 2900 9650 2900
Wire Wire Line
	8650 2850 8650 3000
Connection ~ 9150 2900
Wire Wire Line
	8850 2700 8450 2700
Wire Wire Line
	8850 3100 8450 3100
$Comp
L C C13
U 1 1 55EC7B45
P 6250 2050
F 0 "C13" H 6275 2150 50  0000 L CNN
F 1 "220n" H 6275 1950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6288 1900 30  0001 C CNN
F 3 "" H 6250 2050 60  0000 C CNN
	1    6250 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1900 6250 1850
Wire Wire Line
	6250 1850 6100 1850
Wire Wire Line
	6100 1850 6100 2200
$Comp
L +3.3V #PWR01
U 1 1 55EC7BD6
P 5950 2200
F 0 "#PWR01" H 5950 2050 50  0001 C CNN
F 1 "+3.3V" H 5950 2340 50  0000 C CNN
F 2 "" H 5950 2200 60  0000 C CNN
F 3 "" H 5950 2200 60  0000 C CNN
	1    5950 2200
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR02
U 1 1 55EC7BF8
P 5800 2200
F 0 "#PWR02" H 5800 2050 50  0001 C CNN
F 1 "VPP" H 5800 2350 50  0000 C CNN
F 2 "" H 5800 2200 60  0000 C CNN
F 3 "" H 5800 2200 60  0000 C CNN
	1    5800 2200
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 55EC7C28
P 6350 1400
F 0 "C12" H 6375 1500 50  0000 L CNN
F 1 "100n" H 6375 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6388 1250 30  0001 C CNN
F 3 "" H 6350 1400 60  0000 C CNN
	1    6350 1400
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 55EC7CCF
P 5350 1400
F 0 "C11" H 5375 1500 50  0000 L CNN
F 1 "100n" H 5375 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5388 1250 30  0001 C CNN
F 3 "" H 5350 1400 60  0000 C CNN
	1    5350 1400
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 55EC7D51
P 6650 1400
F 0 "C14" H 6675 1500 50  0000 L CNN
F 1 "22µ" H 6675 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6688 1250 30  0001 C CNN
F 3 "" H 6650 1400 60  0000 C CNN
	1    6650 1400
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR03
U 1 1 55EC7E3F
P 5350 1250
F 0 "#PWR03" H 5350 1100 50  0001 C CNN
F 1 "VPP" H 5350 1400 50  0000 C CNN
F 2 "" H 5350 1250 60  0000 C CNN
F 3 "" H 5350 1250 60  0000 C CNN
	1    5350 1250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 55EC7E68
P 6350 1250
F 0 "#PWR04" H 6350 1100 50  0001 C CNN
F 1 "+3.3V" H 6350 1390 50  0000 C CNN
F 2 "" H 6350 1250 60  0000 C CNN
F 3 "" H 6350 1250 60  0000 C CNN
	1    6350 1250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 55EC7E91
P 6650 1250
F 0 "#PWR05" H 6650 1100 50  0001 C CNN
F 1 "+3.3V" H 6650 1390 50  0000 C CNN
F 2 "" H 6650 1250 60  0000 C CNN
F 3 "" H 6650 1250 60  0000 C CNN
	1    6650 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 55EC7EB2
P 6650 1550
F 0 "#PWR06" H 6650 1300 50  0001 C CNN
F 1 "GND" H 6650 1400 50  0000 C CNN
F 2 "" H 6650 1550 60  0000 C CNN
F 3 "" H 6650 1550 60  0000 C CNN
	1    6650 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 55EC7EE2
P 6350 1550
F 0 "#PWR07" H 6350 1300 50  0001 C CNN
F 1 "GND" H 6350 1400 50  0000 C CNN
F 2 "" H 6350 1550 60  0000 C CNN
F 3 "" H 6350 1550 60  0000 C CNN
	1    6350 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 55EC7FBA
P 5350 1550
F 0 "#PWR08" H 5350 1300 50  0001 C CNN
F 1 "GND" H 5350 1400 50  0000 C CNN
F 2 "" H 5350 1550 60  0000 C CNN
F 3 "" H 5350 1550 60  0000 C CNN
	1    5350 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 55EC7FE3
P 5800 4900
F 0 "#PWR09" H 5800 4650 50  0001 C CNN
F 1 "GND" H 5800 4750 50  0000 C CNN
F 2 "" H 5800 4900 60  0000 C CNN
F 3 "" H 5800 4900 60  0000 C CNN
	1    5800 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 55EC8034
P 5950 4900
F 0 "#PWR010" H 5950 4650 50  0001 C CNN
F 1 "GND" H 5950 4750 50  0000 C CNN
F 2 "" H 5950 4900 60  0000 C CNN
F 3 "" H 5950 4900 60  0000 C CNN
	1    5950 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 55EC805D
P 6100 4900
F 0 "#PWR011" H 6100 4650 50  0001 C CNN
F 1 "GND" H 6100 4750 50  0000 C CNN
F 2 "" H 6100 4900 60  0000 C CNN
F 3 "" H 6100 4900 60  0000 C CNN
	1    6100 4900
	1    0    0    -1  
$EndComp
Text Label 9650 2900 2    60   ~ 0
MotorA
$Comp
L VPP #PWR012
U 1 1 55EC81BA
P 9150 2500
F 0 "#PWR012" H 9150 2350 50  0001 C CNN
F 1 "VPP" H 9150 2650 50  0000 C CNN
F 2 "" H 9150 2500 60  0000 C CNN
F 3 "" H 9150 2500 60  0000 C CNN
	1    9150 2500
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 55EC820B
P 7050 2350
F 0 "C15" H 7075 2450 50  0000 L CNN
F 1 "100n" H 7075 2250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7088 2200 30  0001 C CNN
F 3 "" H 7050 2350 60  0000 C CNN
	1    7050 2350
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 55EC826E
P 7300 2350
F 0 "C16" H 7325 2450 50  0000 L CNN
F 1 "22u" H 7325 2250 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 7338 2200 30  0001 C CNN
F 3 "" H 7300 2350 60  0000 C CNN
	1    7300 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2500 7050 2500
Wire Wire Line
	7050 2200 7550 2200
Wire Wire Line
	7550 2200 7550 2300
Connection ~ 7300 2200
$Comp
L GND #PWR013
U 1 1 55EC831A
P 7550 2300
F 0 "#PWR013" H 7550 2050 50  0001 C CNN
F 1 "GND" H 7550 2150 50  0000 C CNN
F 2 "" H 7550 2300 60  0000 C CNN
F 3 "" H 7550 2300 60  0000 C CNN
	1    7550 2300
	1    0    0    -1  
$EndComp
$Comp
L C C18
U 1 1 55EC860A
P 8650 3800
F 0 "C18" H 8675 3900 50  0000 L CNN
F 1 "100n" H 8675 3700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8688 3650 30  0001 C CNN
F 3 "" H 8650 3800 60  0000 C CNN
	1    8650 3800
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 55EC8610
P 8300 3800
F 0 "R16" V 8380 3800 50  0000 C CNN
F 1 "10" V 8300 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 3800 30  0001 C CNN
F 3 "" H 8300 3800 30  0000 C CNN
	1    8300 3800
	0    1    1    0   
$EndComp
$Comp
L R R17
U 1 1 55EC8616
P 8300 4200
F 0 "R17" V 8380 4200 50  0000 C CNN
F 1 "10" V 8300 4200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 4200 30  0001 C CNN
F 3 "" H 8300 4200 30  0000 C CNN
	1    8300 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 4000 9650 4000
Wire Wire Line
	8650 4000 8650 3950
Connection ~ 9150 4000
Wire Wire Line
	8850 3800 8450 3800
Wire Wire Line
	8850 4200 8450 4200
Text Label 9650 4000 2    60   ~ 0
MotorB
$Comp
L VPP #PWR014
U 1 1 55EC8622
P 9150 3600
F 0 "#PWR014" H 9150 3450 50  0001 C CNN
F 1 "VPP" H 9150 3750 50  0000 C CNN
F 2 "" H 9150 3600 60  0000 C CNN
F 3 "" H 9150 3600 60  0000 C CNN
	1    9150 3600
	1    0    0    -1  
$EndComp
$Comp
L C C19
U 1 1 55EC873C
P 8650 4900
F 0 "C19" H 8675 5000 50  0000 L CNN
F 1 "100n" H 8675 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8688 4750 30  0001 C CNN
F 3 "" H 8650 4900 60  0000 C CNN
	1    8650 4900
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 55EC8742
P 8300 4900
F 0 "R18" V 8380 4900 50  0000 C CNN
F 1 "10" V 8300 4900 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 4900 30  0001 C CNN
F 3 "" H 8300 4900 30  0000 C CNN
	1    8300 4900
	0    1    1    0   
$EndComp
$Comp
L R R19
U 1 1 55EC8748
P 8300 5300
F 0 "R19" V 8380 5300 50  0000 C CNN
F 1 "10" V 8300 5300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8230 5300 30  0001 C CNN
F 3 "" H 8300 5300 30  0000 C CNN
	1    8300 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 5100 9650 5100
Wire Wire Line
	8650 5100 8650 5050
Connection ~ 9150 5100
Wire Wire Line
	8850 4900 8450 4900
Wire Wire Line
	8850 5300 8450 5300
Text Label 9650 5100 2    60   ~ 0
MotorC
$Comp
L VPP #PWR015
U 1 1 55EC8754
P 9150 4700
F 0 "#PWR015" H 9150 4550 50  0001 C CNN
F 1 "VPP" H 9150 4850 50  0000 C CNN
F 2 "" H 9150 4700 60  0000 C CNN
F 3 "" H 9150 4700 60  0000 C CNN
	1    9150 4700
	1    0    0    -1  
$EndComp
Text Label 9650 3300 2    60   ~ 0
LSS
Text Label 9650 4400 2    60   ~ 0
LSS
Text Label 9650 5500 2    60   ~ 0
LSS
Wire Wire Line
	9650 5500 9150 5500
Wire Wire Line
	9150 4400 9650 4400
Wire Wire Line
	9650 3300 9150 3300
Text Label 8500 2550 0    60   ~ 0
CA
Text Label 8500 3650 0    60   ~ 0
CB
Text Label 8500 4750 0    60   ~ 0
CC
Wire Wire Line
	8500 4750 8650 4750
Wire Wire Line
	8650 3650 8500 3650
Wire Wire Line
	8500 2550 8650 2550
Wire Wire Line
	7050 2900 8050 2900
Wire Wire Line
	8050 2900 8050 2700
Wire Wire Line
	8050 2700 8150 2700
Wire Wire Line
	7050 3100 8150 3100
Wire Wire Line
	7050 3400 8050 3400
Wire Wire Line
	8050 3400 8050 3800
Wire Wire Line
	8050 3800 8150 3800
Wire Wire Line
	7800 5300 8150 5300
Wire Wire Line
	8650 3000 7050 3000
Connection ~ 8650 2900
Wire Wire Line
	7050 3500 8000 3500
Wire Wire Line
	8000 3500 8000 4000
Connection ~ 8650 4000
Wire Wire Line
	7050 3600 7950 3600
Wire Wire Line
	7950 3600 7950 4200
Wire Wire Line
	7950 4200 8150 4200
Wire Wire Line
	7050 3900 7900 3900
Wire Wire Line
	7900 3900 7900 4900
Wire Wire Line
	7900 4900 8150 4900
Wire Wire Line
	7850 5100 7850 4000
Wire Wire Line
	7850 4000 7050 4000
Connection ~ 8650 5100
Wire Wire Line
	7050 4100 7800 4100
Wire Wire Line
	7800 4100 7800 5300
Wire Wire Line
	7050 4300 7450 4300
Wire Wire Line
	7050 3800 7450 3800
Wire Wire Line
	7050 3300 7450 3300
Wire Wire Line
	7050 2800 7450 2800
Text Label 7450 2800 2    60   ~ 0
CA
Text Label 7450 3300 2    60   ~ 0
CB
Text Label 7450 3800 2    60   ~ 0
CC
Text Label 7450 4300 2    60   ~ 0
LSS
$Comp
L R R13
U 1 1 55EC9C3E
P 7300 4600
F 0 "R13" V 7380 4600 50  0000 C CNN
F 1 "R" V 7300 4600 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7230 4600 30  0001 C CNN
F 3 "" H 7300 4600 30  0000 C CNN
	1    7300 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4450 7300 4450
Wire Wire Line
	7300 4450 7300 4300
Connection ~ 7300 4300
Wire Wire Line
	7050 4600 7150 4600
Wire Wire Line
	7150 4600 7150 4750
Wire Wire Line
	7150 4750 7300 4750
Wire Wire Line
	7300 4750 7300 4850
$Comp
L GND #PWR016
U 1 1 55EC9DFD
P 7300 4850
F 0 "#PWR016" H 7300 4600 50  0001 C CNN
F 1 "GND" H 7300 4700 50  0000 C CNN
F 2 "" H 7300 4850 60  0000 C CNN
F 3 "" H 7300 4850 60  0000 C CNN
	1    7300 4850
	1    0    0    -1  
$EndComp
Text Notes 7550 4750 1    60   ~ 0
Shunt
$Comp
L VPP #PWR017
U 1 1 55EC9F84
P 9150 1600
F 0 "#PWR017" H 9150 1450 50  0001 C CNN
F 1 "VPP" H 9150 1750 50  0000 C CNN
F 2 "" H 9150 1600 60  0000 C CNN
F 3 "" H 9150 1600 60  0000 C CNN
	1    9150 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 55EC9FC1
P 9150 1900
F 0 "#PWR018" H 9150 1650 50  0001 C CNN
F 1 "GND" H 9150 1750 50  0000 C CNN
F 2 "" H 9150 1900 60  0000 C CNN
F 3 "" H 9150 1900 60  0000 C CNN
	1    9150 1900
	1    0    0    -1  
$EndComp
$Comp
L C C20
U 1 1 55ECA032
P 9150 1750
F 0 "C20" H 9175 1850 50  0000 L CNN
F 1 "100n" H 9175 1650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9188 1600 30  0001 C CNN
F 3 "" H 9150 1750 60  0000 C CNN
	1    9150 1750
	1    0    0    -1  
$EndComp
$Comp
L CP C21
U 1 1 55ECA0A0
P 9450 1750
F 0 "C21" H 9475 1850 50  0000 L CNN
F 1 "1000µ/25V" H 9475 1650 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D8_L11.5_P3.5" H 9488 1600 30  0001 C CNN
F 3 "" H 9450 1750 60  0000 C CNN
	1    9450 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 55ECA102
P 9450 1900
F 0 "#PWR019" H 9450 1650 50  0001 C CNN
F 1 "GND" H 9450 1750 50  0000 C CNN
F 2 "" H 9450 1900 60  0000 C CNN
F 3 "" H 9450 1900 60  0000 C CNN
	1    9450 1900
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR020
U 1 1 55ECA158
P 9450 1600
F 0 "#PWR020" H 9450 1450 50  0001 C CNN
F 1 "VPP" H 9450 1750 50  0000 C CNN
F 2 "" H 9450 1600 60  0000 C CNN
F 3 "" H 9450 1600 60  0000 C CNN
	1    9450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2600 7850 2600
Wire Wire Line
	7850 2600 7850 2500
$Comp
L VPP #PWR021
U 1 1 55ECA395
P 7850 2500
F 0 "#PWR021" H 7850 2350 50  0001 C CNN
F 1 "VPP" H 7850 2650 50  0000 C CNN
F 2 "" H 7850 2500 60  0000 C CNN
F 3 "" H 7850 2500 60  0000 C CNN
	1    7850 2500
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA8-AI IC1
U 1 1 55ECA482
P 2450 3400
F 0 "IC1" H 1700 4600 40  0000 L BNN
F 1 "ATMEGA8-AI" H 2950 1850 40  0000 L BNN
F 2 "Housings_QFP:LQFP-32_7x7mm_Pitch0.8mm" H 2450 3400 30  0000 C CIN
F 3 "" H 2450 3400 60  0000 C CNN
	1    2450 3400
	1    0    0    -1  
$EndComp
$Comp
L RESONATEUR Y1
U 1 1 55EC7DB9
P 1200 3400
F 0 "Y1" H 1220 3600 60  0000 C CNN
F 1 "16MHz" H 1450 3200 60  0000 C CNN
F 2 "toni:CerOsc_3,2x1,3" H 1200 3400 60  0001 C CNN
F 3 "" H 1200 3400 60  0000 C CNN
	1    1200 3400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR022
U 1 1 55EC7FB3
P 2500 5000
F 0 "#PWR022" H 2500 4750 50  0001 C CNN
F 1 "GND" H 2500 4850 50  0000 C CNN
F 2 "" H 2500 5000 60  0000 C CNN
F 3 "" H 2500 5000 60  0000 C CNN
	1    2500 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 55EC800F
P 2400 5000
F 0 "#PWR023" H 2400 4750 50  0001 C CNN
F 1 "GND" H 2400 4850 50  0000 C CNN
F 2 "" H 2400 5000 60  0000 C CNN
F 3 "" H 2400 5000 60  0000 C CNN
	1    2400 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 55EC806B
P 950 4100
F 0 "#PWR024" H 950 3850 50  0001 C CNN
F 1 "GND" H 950 3950 50  0000 C CNN
F 2 "" H 950 4100 60  0000 C CNN
F 3 "" H 950 4100 60  0000 C CNN
	1    950  4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3100 1250 3100
Wire Wire Line
	1550 3300 1500 3300
Wire Wire Line
	1500 3300 1500 3700
Wire Wire Line
	1500 3700 1250 3700
$Comp
L +3.3V #PWR025
U 1 1 55EC8187
P 2400 2100
F 0 "#PWR025" H 2400 1950 50  0001 C CNN
F 1 "+3.3V" H 2400 2240 50  0000 C CNN
F 2 "" H 2400 2100 60  0000 C CNN
F 3 "" H 2400 2100 60  0000 C CNN
	1    2400 2100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR026
U 1 1 55EC81E3
P 2500 2100
F 0 "#PWR026" H 2500 1950 50  0001 C CNN
F 1 "+3.3V" H 2500 2240 50  0000 C CNN
F 2 "" H 2500 2100 60  0000 C CNN
F 3 "" H 2500 2100 60  0000 C CNN
	1    2500 2100
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 55EC827D
P 950 2200
F 0 "R1" V 1030 2200 50  0000 C CNN
F 1 "100" V 950 2200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 880 2200 30  0001 C CNN
F 3 "" H 950 2200 30  0000 C CNN
	1    950  2200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR027
U 1 1 55EC8310
P 950 2050
F 0 "#PWR027" H 950 1900 50  0001 C CNN
F 1 "+3.3V" H 950 2190 50  0000 C CNN
F 2 "" H 950 2050 60  0000 C CNN
F 3 "" H 950 2050 60  0000 C CNN
	1    950  2050
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 55EC838B
P 950 2850
F 0 "C1" H 975 2950 50  0000 L CNN
F 1 "1u" H 975 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 988 2700 30  0001 C CNN
F 3 "" H 950 2850 60  0000 C CNN
	1    950  2850
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 55EC8480
P 1200 2850
F 0 "C2" H 1225 2950 50  0000 L CNN
F 1 "100n" H 1225 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1238 2700 30  0001 C CNN
F 3 "" H 1200 2850 60  0000 C CNN
	1    1200 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3000 1550 3000
Wire Wire Line
	950  3000 950  4100
Connection ~ 950  3400
Wire Wire Line
	1550 2700 1200 2700
Wire Wire Line
	800  2600 1550 2600
Wire Wire Line
	950  2350 950  2700
Connection ~ 950  2600
Wire Wire Line
	1550 3000 1550 2800
Connection ~ 1200 3000
$Comp
L CONN_02X03 P1
U 1 1 55EC8B48
P 2400 1050
F 0 "P1" H 2400 1250 50  0000 C CNN
F 1 "CONN_02X03" H 2400 850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 2400 -150 60  0001 C CNN
F 3 "" H 2400 -150 60  0000 C CNN
	1    2400 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 950  1850 950 
Wire Wire Line
	2150 1050 1850 1050
Wire Wire Line
	2150 1150 1850 1150
Wire Wire Line
	2650 1050 2950 1050
$Comp
L +3.3V #PWR028
U 1 1 55EC8D92
P 2750 950
F 0 "#PWR028" H 2750 800 50  0001 C CNN
F 1 "+3.3V" H 2750 1090 50  0000 C CNN
F 2 "" H 2750 950 60  0000 C CNN
F 3 "" H 2750 950 60  0000 C CNN
	1    2750 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 950  2650 950 
Wire Wire Line
	2650 1150 2750 1150
$Comp
L GND #PWR029
U 1 1 55EC8F02
P 2750 1150
F 0 "#PWR029" H 2750 900 50  0001 C CNN
F 1 "GND" H 2750 1000 50  0000 C CNN
F 2 "" H 2750 1150 60  0000 C CNN
F 3 "" H 2750 1150 60  0000 C CNN
	1    2750 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2900 3750 2900
Wire Wire Line
	3450 2800 3750 2800
Wire Wire Line
	3450 2700 3750 2700
$Comp
L R_PACK4 RP1
U 1 1 55EC90B2
P 4650 2850
F 0 "RP1" H 4650 3300 50  0000 C CNN
F 1 "R_PACK4" H 4650 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_Array_Concave_4x0603" H 4650 2850 60  0001 C CNN
F 3 "" H 4650 2850 60  0000 C CNN
	1    4650 2850
	1    0    0    -1  
$EndComp
Text Label 3750 2800 2    60   ~ 0
MISO
Text Label 3750 2700 2    60   ~ 0
MOSI
Text Label 3750 2900 2    60   ~ 0
SCK
Text Label 3750 2600 2    60   ~ 0
CS
Wire Wire Line
	3450 2600 3750 2600
Wire Wire Line
	4150 2800 4450 2800
Wire Wire Line
	4150 2700 4450 2700
Wire Wire Line
	4150 2600 4450 2600
Text Label 4150 2800 0    60   ~ 0
MISO
Text Label 4150 2700 0    60   ~ 0
MOSI
Text Label 4150 2600 0    60   ~ 0
SCK
Text Label 4150 2500 0    60   ~ 0
CS
Wire Wire Line
	4150 2500 4450 2500
Text Label 1850 950  0    60   ~ 0
MOSI
Text Label 1850 1050 0    60   ~ 0
SCK
Text Label 2950 1050 2    60   ~ 0
MISO
Text Label 1250 2400 0    60   ~ 0
Reset
Wire Wire Line
	1250 2400 1550 2400
Text Label 1850 1150 0    60   ~ 0
Reset
$Comp
L R R2
U 1 1 55EC9DAF
P 1550 1550
F 0 "R2" V 1630 1550 50  0000 C CNN
F 1 "4,7k" V 1550 1550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1480 1550 30  0001 C CNN
F 3 "" H 1550 1550 30  0000 C CNN
	1    1550 1550
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 55EC9EFA
P 1300 1850
F 0 "C3" H 1325 1950 50  0000 L CNN
F 1 "100n" H 1325 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1338 1700 30  0001 C CNN
F 3 "" H 1300 1850 60  0000 C CNN
	1    1300 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1700 1550 1700
$Comp
L +3.3V #PWR030
U 1 1 55ECA01D
P 1550 1400
F 0 "#PWR030" H 1550 1250 50  0001 C CNN
F 1 "+3.3V" H 1550 1540 50  0000 C CNN
F 2 "" H 1550 1400 60  0000 C CNN
F 3 "" H 1550 1400 60  0000 C CNN
	1    1550 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 55ECA08E
P 1300 2000
F 0 "#PWR031" H 1300 1750 50  0001 C CNN
F 1 "GND" H 1300 1850 50  0000 C CNN
F 2 "" H 1300 2000 60  0000 C CNN
F 3 "" H 1300 2000 60  0000 C CNN
	1    1300 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1700 1550 2400
Text Notes 1600 1750 0    60   ~ 0
Für leicht EMI gestörte Umgebungen\n
Wire Wire Line
	4850 3100 4000 3100
Wire Wire Line
	4000 3100 4000 2500
Wire Wire Line
	4000 2500 3450 2500
$Comp
L R R3
U 1 1 55ECA54E
P 3650 1000
F 0 "R3" V 3730 1000 50  0000 C CNN
F 1 "R" V 3650 1000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3580 1000 30  0001 C CNN
F 3 "" H 3650 1000 30  0000 C CNN
	1    3650 1000
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR032
U 1 1 55ECA5EA
P 3650 850
F 0 "#PWR032" H 3650 700 50  0001 C CNN
F 1 "VPP" H 3650 1000 50  0000 C CNN
F 2 "" H 3650 850 60  0000 C CNN
F 3 "" H 3650 850 60  0000 C CNN
	1    3650 850 
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 55ECA6B8
P 3650 1300
F 0 "R4" V 3730 1300 50  0000 C CNN
F 1 "R" V 3650 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3580 1300 30  0001 C CNN
F 3 "" H 3650 1300 30  0000 C CNN
	1    3650 1300
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 55ECA7C8
P 3950 1300
F 0 "C8" H 3975 1400 50  0000 L CNN
F 1 "100n" H 3975 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3988 1150 30  0001 C CNN
F 3 "" H 3950 1300 60  0000 C CNN
	1    3950 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR033
U 1 1 55ECA871
P 3650 1450
F 0 "#PWR033" H 3650 1200 50  0001 C CNN
F 1 "GND" H 3650 1300 50  0000 C CNN
F 2 "" H 3650 1450 60  0000 C CNN
F 3 "" H 3650 1450 60  0000 C CNN
	1    3650 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR034
U 1 1 55ECA8EB
P 3950 1450
F 0 "#PWR034" H 3950 1200 50  0001 C CNN
F 1 "GND" H 3950 1300 50  0000 C CNN
F 2 "" H 3950 1450 60  0000 C CNN
F 3 "" H 3950 1450 60  0000 C CNN
	1    3950 1450
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 55ECAA94
P 4300 1000
F 0 "R9" V 4380 1000 50  0000 C CNN
F 1 "R" V 4300 1000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4230 1000 30  0001 C CNN
F 3 "" H 4300 1000 30  0000 C CNN
	1    4300 1000
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 55ECAAA0
P 4300 1300
F 0 "R10" V 4380 1300 50  0000 C CNN
F 1 "R" V 4300 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4230 1300 30  0001 C CNN
F 3 "" H 4300 1300 30  0000 C CNN
	1    4300 1300
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 55ECAAA6
P 4600 1300
F 0 "C9" H 4625 1400 50  0000 L CNN
F 1 "100n" H 4625 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4638 1150 30  0001 C CNN
F 3 "" H 4600 1300 60  0000 C CNN
	1    4600 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR035
U 1 1 55ECAAAC
P 4300 1450
F 0 "#PWR035" H 4300 1200 50  0001 C CNN
F 1 "GND" H 4300 1300 50  0000 C CNN
F 2 "" H 4300 1450 60  0000 C CNN
F 3 "" H 4300 1450 60  0000 C CNN
	1    4300 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR036
U 1 1 55ECAAB2
P 4600 1450
F 0 "#PWR036" H 4600 1200 50  0001 C CNN
F 1 "GND" H 4600 1300 50  0000 C CNN
F 2 "" H 4600 1450 60  0000 C CNN
F 3 "" H 4600 1450 60  0000 C CNN
	1    4600 1450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR037
U 1 1 55ECAC01
P 4300 850
F 0 "#PWR037" H 4300 700 50  0001 C CNN
F 1 "+3.3V" H 4300 990 50  0000 C CNN
F 2 "" H 4300 850 60  0000 C CNN
F 3 "" H 4300 850 60  0000 C CNN
	1    4300 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 1150 3950 1150
Wire Wire Line
	4300 1150 4600 1150
Text Label 3950 1150 2    60   ~ 0
ADC0
Text Label 4600 1150 2    60   ~ 0
ADC1
Text Label 3750 3100 2    60   ~ 0
ADC0
Text Label 3750 3200 2    60   ~ 0
ADC1
Wire Wire Line
	3750 3200 3450 3200
Wire Wire Line
	3450 3100 3750 3100
$Comp
L R R6
U 1 1 55ECB5C7
P 3700 4000
F 0 "R6" V 3780 4000 50  0000 C CNN
F 1 "1k" V 3700 4000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3630 4000 30  0001 C CNN
F 3 "" H 3700 4000 30  0000 C CNN
	1    3700 4000
	0    -1   -1   0   
$EndComp
$Comp
L R R7
U 1 1 55ECB79E
P 3700 4100
F 0 "R7" V 3780 4100 50  0000 C CNN
F 1 "1k" V 3700 4100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3630 4100 30  0001 C CNN
F 3 "" H 3700 4100 30  0000 C CNN
	1    3700 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 4000 3550 4000
Wire Wire Line
	3450 4100 3550 4100
Wire Wire Line
	4300 3250 4850 3250
$Comp
L R R11
U 1 1 55ECBA30
P 4300 3950
F 0 "R11" V 4380 3950 50  0000 C CNN
F 1 "1k" V 4300 3950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4230 3950 30  0001 C CNN
F 3 "" H 4300 3950 30  0000 C CNN
	1    4300 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 4200 4300 4200
Wire Wire Line
	4300 4200 4300 4100
$Comp
L R R12
U 1 1 55ECBD13
P 4400 3950
F 0 "R12" V 4480 3950 50  0000 C CNN
F 1 "1k" V 4400 3950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4330 3950 30  0001 C CNN
F 3 "" H 4400 3950 30  0000 C CNN
	1    4400 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 4300 4400 4300
Wire Wire Line
	4400 4300 4400 4100
Wire Wire Line
	4300 3800 4300 3250
Wire Wire Line
	4400 3800 4400 3700
Wire Wire Line
	4400 3700 4850 3700
Wire Wire Line
	3450 4400 4550 4400
Wire Wire Line
	4550 4400 4550 3500
Wire Wire Line
	4550 3500 4850 3500
NoConn ~ 3450 3300
NoConn ~ 3450 3400
NoConn ~ 3450 3700
NoConn ~ 3450 3800
$Comp
L LED D3
U 1 1 55ECC0E0
P 3650 5000
F 0 "D3" H 3650 5100 50  0000 C CNN
F 1 "LED" H 3650 4900 50  0000 C CNN
F 2 "LEDs:LED-0603" H 3650 5000 60  0001 C CNN
F 3 "" H 3650 5000 60  0000 C CNN
	1    3650 5000
	0    -1   -1   0   
$EndComp
$Comp
L LED D4
U 1 1 55ECC2AA
P 3950 5000
F 0 "D4" H 3950 5100 50  0000 C CNN
F 1 "LED" H 3950 4900 50  0000 C CNN
F 2 "LEDs:LED-0603" H 3950 5000 60  0001 C CNN
F 3 "" H 3950 5000 60  0000 C CNN
	1    3950 5000
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 55ECC3DD
P 3650 5350
F 0 "R5" V 3730 5350 50  0000 C CNN
F 1 "100" V 3650 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3580 5350 30  0001 C CNN
F 3 "" H 3650 5350 30  0000 C CNN
	1    3650 5350
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 55ECC4BC
P 3950 5350
F 0 "R8" V 4030 5350 50  0000 C CNN
F 1 "100" V 3950 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3880 5350 30  0001 C CNN
F 3 "" H 3950 5350 30  0000 C CNN
	1    3950 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR038
U 1 1 55ECC68E
P 3650 5500
F 0 "#PWR038" H 3650 5250 50  0001 C CNN
F 1 "GND" H 3650 5350 50  0000 C CNN
F 2 "" H 3650 5500 60  0000 C CNN
F 3 "" H 3650 5500 60  0000 C CNN
	1    3650 5500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR039
U 1 1 55ECC729
P 3950 5500
F 0 "#PWR039" H 3950 5250 50  0001 C CNN
F 1 "GND" H 3950 5350 50  0000 C CNN
F 2 "" H 3950 5500 60  0000 C CNN
F 3 "" H 3950 5500 60  0000 C CNN
	1    3950 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4800 3650 4700
Wire Wire Line
	3650 4700 3450 4700
Wire Wire Line
	3450 4600 3950 4600
Wire Wire Line
	3950 4600 3950 4800
Text Notes 3300 5850 0    60   ~ 0
LEDs für Statusanzeige
$Comp
L ZENER D1
U 1 1 55ECE4C5
P 800 6650
F 0 "D1" H 800 6750 50  0000 C CNN
F 1 "TVS 18V" H 800 6550 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 800 6650 60  0001 C CNN
F 3 "" H 800 6650 60  0000 C CNN
	1    800  6650
	0    1    1    0   
$EndComp
$Comp
L GND #PWR040
U 1 1 55ECE631
P 800 6850
F 0 "#PWR040" H 800 6600 50  0001 C CNN
F 1 "GND" H 800 6700 50  0000 C CNN
F 2 "" H 800 6850 60  0000 C CNN
F 3 "" H 800 6850 60  0000 C CNN
	1    800  6850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P3
U 1 1 55ECF186
P 5300 7050
F 0 "P3" H 5300 7250 50  0000 C CNN
F 1 "CONN_01X03" V 5400 7050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 5300 7050 60  0001 C CNN
F 3 "" H 5300 7050 60  0000 C CNN
	1    5300 7050
	0    -1   1    0   
$EndComp
$Comp
L CONN_01X02 P2
U 1 1 55ECF2A1
P 4350 7050
F 0 "P2" H 4350 7200 50  0000 C CNN
F 1 "CONN_01X02" V 4450 7050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 4350 7050 60  0001 C CNN
F 3 "" H 4350 7050 60  0000 C CNN
	1    4350 7050
	0    -1   1    0   
$EndComp
$Comp
L CONN_01X03 P4
U 1 1 55ECF438
P 6200 7050
F 0 "P4" H 6200 7250 50  0000 C CNN
F 1 "CONN_01X03" V 6300 7050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 6200 7050 60  0001 C CNN
F 3 "" H 6200 7050 60  0000 C CNN
	1    6200 7050
	0    -1   1    0   
$EndComp
Text Label 6100 6450 3    60   ~ 0
MotorA
Text Label 6200 6450 3    60   ~ 0
MotorB
Text Label 6300 6450 3    60   ~ 0
MotorC
Text Label 4000 4000 2    60   ~ 0
RX
Text Label 4000 4100 2    60   ~ 0
TX
Wire Wire Line
	4000 4000 3850 4000
Wire Wire Line
	3850 4100 4000 4100
Text Label 5300 6450 3    60   ~ 0
RX
Text Label 5200 6450 3    60   ~ 0
TX
$Comp
L GND #PWR041
U 1 1 55ECFCCB
P 5400 6450
F 0 "#PWR041" H 5400 6200 50  0001 C CNN
F 1 "GND" H 5400 6300 50  0000 C CNN
F 2 "" H 5400 6450 60  0000 C CNN
F 3 "" H 5400 6450 60  0000 C CNN
	1    5400 6450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5400 6450 5400 6850
Wire Wire Line
	5300 6850 5300 6450
Wire Wire Line
	5200 6450 5200 6850
Wire Wire Line
	6100 6850 6100 6450
Wire Wire Line
	6200 6450 6200 6850
Wire Wire Line
	6300 6850 6300 6450
$Comp
L GND #PWR042
U 1 1 55ED01FB
P 4400 6450
F 0 "#PWR042" H 4400 6200 50  0001 C CNN
F 1 "GND" H 4400 6300 50  0000 C CNN
F 2 "" H 4400 6450 60  0000 C CNN
F 3 "" H 4400 6450 60  0000 C CNN
	1    4400 6450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4400 6450 4400 6850
$Comp
L VPP #PWR043
U 1 1 55ED0449
P 4300 6450
F 0 "#PWR043" H 4300 6300 50  0001 C CNN
F 1 "VPP" H 4300 6600 50  0000 C CNN
F 2 "" H 4300 6450 60  0000 C CNN
F 3 "" H 4300 6450 60  0000 C CNN
	1    4300 6450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4300 6450 4300 6850
Text Notes 4200 7450 0    60   ~ 0
Strom\n12V
Text Notes 5200 7400 0    60   ~ 0
UART
Text Notes 6100 7400 0    60   ~ 0
Motor
NoConn ~ 3450 3600
NoConn ~ 3450 3500
Wire Wire Line
	4550 2950 4850 2950
Wire Wire Line
	4600 2950 4600 4800
$Comp
L C C10
U 1 1 55ED141B
P 4600 4950
F 0 "C10" H 4625 5050 50  0000 L CNN
F 1 "100n" H 4625 4850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4638 4800 30  0001 C CNN
F 3 "" H 4600 4950 60  0000 C CNN
	1    4600 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR044
U 1 1 55ED152C
P 4600 5100
F 0 "#PWR044" H 4600 4850 50  0001 C CNN
F 1 "GND" H 4600 4950 50  0000 C CNN
F 2 "" H 4600 5100 60  0000 C CNN
F 3 "" H 4600 5100 60  0000 C CNN
	1    4600 5100
	1    0    0    -1  
$EndComp
NoConn ~ 3450 2400
$Comp
L PWR_FLAG #FLG045
U 1 1 55ED1AFC
P 4250 6750
F 0 "#FLG045" H 4250 6845 50  0001 C CNN
F 1 "PWR_FLAG" H 4250 6930 50  0000 C CNN
F 2 "" H 4250 6750 60  0000 C CNN
F 3 "" H 4250 6750 60  0000 C CNN
	1    4250 6750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4250 6750 4300 6750
Connection ~ 4300 6750
$Comp
L PWR_FLAG #FLG046
U 1 1 55ED1C87
P 4450 6750
F 0 "#FLG046" H 4450 6845 50  0001 C CNN
F 1 "PWR_FLAG" H 4450 6930 50  0000 C CNN
F 2 "" H 4450 6750 60  0000 C CNN
F 3 "" H 4450 6750 60  0000 C CNN
	1    4450 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 6750 4400 6750
Connection ~ 4400 6750
NoConn ~ 3450 4500
$Comp
L PWR_FLAG #FLG047
U 1 1 55ED2338
P 800 2600
F 0 "#FLG047" H 800 2695 50  0001 C CNN
F 1 "PWR_FLAG" H 800 2780 50  0000 C CNN
F 2 "" H 800 2600 60  0000 C CNN
F 3 "" H 800 2600 60  0000 C CNN
	1    800  2600
	0    -1   -1   0   
$EndComp
$Comp
L C C23
U 1 1 55ED2929
P 5650 1400
F 0 "C23" H 5675 1500 50  0000 L CNN
F 1 "4,7µ/25V" H 5675 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5688 1250 30  0001 C CNN
F 3 "" H 5650 1400 60  0000 C CNN
F 4 "C1206C475K3RACTU" H 5650 1400 60  0001 C CNN "Herstellername"
	1    5650 1400
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR048
U 1 1 55ED2AD1
P 5650 1250
F 0 "#PWR048" H 5650 1100 50  0001 C CNN
F 1 "VPP" H 5650 1400 50  0000 C CNN
F 2 "" H 5650 1250 60  0000 C CNN
F 3 "" H 5650 1250 60  0000 C CNN
	1    5650 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR049
U 1 1 55ED2B90
P 5650 1550
F 0 "#PWR049" H 5650 1300 50  0001 C CNN
F 1 "GND" H 5650 1400 50  0000 C CNN
F 2 "" H 5650 1550 60  0000 C CNN
F 3 "" H 5650 1550 60  0000 C CNN
	1    5650 1550
	1    0    0    -1  
$EndComp
$Comp
L LM2931AZ-5.0 U1
U 1 1 5653AFAA
P 2400 6725
F 0 "U1" H 2400 6975 40  0000 C CNN
F 1 "LM2931AZ-5.0" H 2400 6925 40  0000 C CNN
F 2 "toni:SOT223_GND_VO_VI" H 2400 6825 35  0001 C CIN
F 3 "" H 2400 6725 60  0000 C CNN
	1    2400 6725
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR050
U 1 1 5653B466
P 2800 6675
F 0 "#PWR050" H 2800 6525 50  0001 C CNN
F 1 "+3.3V" H 2800 6815 50  0000 C CNN
F 2 "" H 2800 6675 60  0000 C CNN
F 3 "" H 2800 6675 60  0000 C CNN
	1    2800 6675
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR051
U 1 1 5653BE8A
P 2400 6975
F 0 "#PWR051" H 2400 6725 50  0001 C CNN
F 1 "GND" H 2400 6825 50  0000 C CNN
F 2 "" H 2400 6975 60  0000 C CNN
F 3 "" H 2400 6975 60  0000 C CNN
	1    2400 6975
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR052
U 1 1 5653BF3A
P 1700 6675
F 0 "#PWR052" H 1700 6525 50  0001 C CNN
F 1 "VPP" H 1700 6825 50  0000 C CNN
F 2 "" H 1700 6675 60  0000 C CNN
F 3 "" H 1700 6675 60  0000 C CNN
	1    1700 6675
	0    -1   -1   0   
$EndComp
$Comp
L R R20
U 1 1 5653BFEC
P 1850 6675
F 0 "R20" V 1930 6675 50  0000 C CNN
F 1 "1" V 1850 6675 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 1780 6675 30  0001 C CNN
F 3 "" H 1850 6675 30  0000 C CNN
	1    1850 6675
	0    1    1    0   
$EndComp
$Comp
L VPP #PWR053
U 1 1 5653D0D0
P 800 6450
F 0 "#PWR053" H 800 6300 50  0001 C CNN
F 1 "VPP" H 800 6600 50  0000 C CNN
F 2 "" H 800 6450 60  0000 C CNN
F 3 "" H 800 6450 60  0000 C CNN
	1    800  6450
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS_old Q1
U 1 1 56540E55
P 9050 2700
F 0 "Q1" H 9350 2750 50  0000 R CNN
F 1 "Q_NMOS_GDS_old" H 9700 2650 50  0000 R CNN
F 2 "sven:SO8_WITH_Heatpipes" H 9250 2800 29  0001 C CNN
F 3 "" H 9050 2700 60  0000 C CNN
	1    9050 2700
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS_old Q2
U 1 1 565412E8
P 9050 3100
F 0 "Q2" H 9350 3150 50  0000 R CNN
F 1 "Q_NMOS_GDS_old" H 9700 3050 50  0000 R CNN
F 2 "sven:SO8_WITH_Heatpipes" H 9250 3200 29  0001 C CNN
F 3 "" H 9050 3100 60  0000 C CNN
	1    9050 3100
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS_old Q3
U 1 1 56541494
P 9050 3800
F 0 "Q3" H 9350 3850 50  0000 R CNN
F 1 "Q_NMOS_GDS_old" H 9700 3750 50  0000 R CNN
F 2 "sven:SO8_WITH_Heatpipes" H 9250 3900 29  0001 C CNN
F 3 "" H 9050 3800 60  0000 C CNN
	1    9050 3800
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS_old Q4
U 1 1 565416B9
P 9050 4200
F 0 "Q4" H 9350 4250 50  0000 R CNN
F 1 "Q_NMOS_GDS_old" H 9700 4150 50  0000 R CNN
F 2 "sven:SO8_WITH_Heatpipes" H 9250 4300 29  0001 C CNN
F 3 "" H 9050 4200 60  0000 C CNN
	1    9050 4200
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS_old Q5
U 1 1 5654183F
P 9050 4900
F 0 "Q5" H 9350 4950 50  0000 R CNN
F 1 "Q_NMOS_GDS_old" H 9700 4850 50  0000 R CNN
F 2 "sven:SO8_WITH_Heatpipes" H 9250 5000 29  0001 C CNN
F 3 "" H 9050 4900 60  0000 C CNN
	1    9050 4900
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GDS_old Q6
U 1 1 5654192E
P 9050 5300
F 0 "Q6" H 9350 5350 50  0000 R CNN
F 1 "Q_NMOS_GDS_old" H 9700 5250 50  0000 R CNN
F 2 "sven:SO8_WITH_Heatpipes" H 9250 5400 29  0001 C CNN
F 3 "" H 9050 5300 60  0000 C CNN
	1    9050 5300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 56577063
P 4550 2950
F 0 "#PWR?" H 4550 2800 50  0001 C CNN
F 1 "+3.3V" H 4550 3090 50  0000 C CNN
F 2 "" H 4550 2950 60  0000 C CNN
F 3 "" H 4550 2950 60  0000 C CNN
	1    4550 2950
	0    -1   -1   0   
$EndComp
Connection ~ 4600 2950
$EndSCHEMATC
