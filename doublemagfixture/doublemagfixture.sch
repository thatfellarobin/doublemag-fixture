EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Two-Magnet Fixture Electrical Schematic"
Date "2021-10-08"
Rev ""
Comp "Microrobotics Laboratory, University of Toronto"
Comment1 "Author: Robin Liu"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	2400 1600 2500 1600
Wire Wire Line
	2500 1600 2500 1500
$Comp
L power:+12V #PWR07
U 1 1 61620316
P 2500 1500
F 0 "#PWR07" H 2500 1350 50  0001 C CNN
F 1 "+12V" H 2515 1673 50  0000 C CNN
F 2 "" H 2500 1500 50  0001 C CNN
F 3 "" H 2500 1500 50  0001 C CNN
	1    2500 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 1800 2500 1800
Wire Wire Line
	2500 1800 2500 1900
$Comp
L power:GND #PWR08
U 1 1 61620F40
P 2500 1900
F 0 "#PWR08" H 2500 1650 50  0001 C CNN
F 1 "GND" H 2505 1727 50  0000 C CNN
F 2 "" H 2500 1900 50  0001 C CNN
F 3 "" H 2500 1900 50  0001 C CNN
	1    2500 1900
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR017
U 1 1 616215E6
P 2600 4900
F 0 "#PWR017" H 2600 4750 50  0001 C CNN
F 1 "+12V" H 2615 5073 50  0000 C CNN
F 2 "" H 2600 4900 50  0001 C CNN
F 3 "" H 2600 4900 50  0001 C CNN
	1    2600 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 616221D4
P 2700 7000
F 0 "#PWR023" H 2700 6750 50  0001 C CNN
F 1 "GND" H 2705 6827 50  0000 C CNN
F 2 "" H 2700 7000 50  0001 C CNN
F 3 "" H 2700 7000 50  0001 C CNN
	1    2700 7000
	1    0    0    -1  
$EndComp
Text Label 1950 5500 2    50   ~ 0
step0
Text Label 1950 5600 2    50   ~ 0
dir0
Text Label 1950 5700 2    50   ~ 0
step1
Text Label 1950 5800 2    50   ~ 0
dir1
Text Label 1950 5900 2    50   ~ 0
step2
Text Label 1950 6000 2    50   ~ 0
dir2
Text Label 1950 6100 2    50   ~ 0
step3
Text Label 1950 6200 2    50   ~ 0
dir3
Text Label 1950 6300 2    50   ~ 0
lim0
Text Label 1950 6400 2    50   ~ 0
lim1
$Comp
L MCU_Module:Arduino_UNO_R3 A1
U 1 1 61645E27
P 2700 5900
F 0 "A1" H 2700 4500 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 2700 4400 50  0000 C CNN
F 2 "" H 2700 5900 50  0001 C CIN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 2700 5900 50  0001 C CNN
	1    2700 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 7000 2700 7000
Connection ~ 2700 7000
$Comp
L Motor:Stepper_Motor_bipolar M1
U 1 1 61693277
P 4400 2800
F 0 "M1" V 4600 2850 50  0000 R CNN
F 1 "12V_350mA_Stepper_Motor" V 4700 3300 50  0000 R CNN
F 2 "" H 4410 2790 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 4410 2790 50  0001 C CNN
	1    4400 2800
	0    1    1    0   
$EndComp
Text Label 6700 1100 0    50   ~ 0
dir0
Text Label 6700 1200 0    50   ~ 0
step0
$Comp
L Device:R_Small R1
U 1 1 616932BB
P 7150 1100
F 0 "R1" V 6954 1100 50  0000 C CNN
F 1 "1k" V 7045 1100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7150 1100 50  0001 C CNN
F 3 "~" H 7150 1100 50  0001 C CNN
	1    7150 1100
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 616932C5
P 7450 1100
F 0 "D1" H 7450 893 50  0000 C CNN
F 1 "DIR0" H 7450 984 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 7450 1100 50  0001 C CNN
F 3 "~" V 7450 1100 50  0001 C CNN
	1    7450 1100
	-1   0    0    1   
$EndComp
Wire Wire Line
	7350 1100 7250 1100
$Comp
L power:GND #PWR05
U 1 1 616932D0
P 7700 1350
F 0 "#PWR05" H 7700 1100 50  0001 C CNN
F 1 "GND" H 7705 1177 50  0000 C CNN
F 2 "" H 7700 1350 50  0001 C CNN
F 3 "" H 7700 1350 50  0001 C CNN
	1    7700 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 1200 7050 1200
Wire Wire Line
	6650 1100 7050 1100
$Comp
L Device:LED_Small D2
U 1 1 616932E6
P 7450 1200
F 0 "D2" H 7450 1400 50  0000 C CNN
F 1 "STEP0" H 7450 1300 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 7450 1200 50  0001 C CNN
F 3 "~" V 7450 1200 50  0001 C CNN
	1    7450 1200
	-1   0    0    1   
$EndComp
Wire Wire Line
	7350 1200 7250 1200
Wire Wire Line
	7550 1100 7700 1100
Wire Wire Line
	7700 1100 7700 1200
$Comp
L Switch:SW_Push SW1
U 1 1 616C098E
P 4650 7050
F 0 "SW1" H 4650 7335 50  0000 C CNN
F 1 "LIMIT_SWITCH" H 4650 7244 50  0000 C CNN
F 2 "" H 4650 7250 50  0001 C CNN
F 3 "~" H 4650 7250 50  0001 C CNN
	1    4650 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 616C2650
P 4650 6600
F 0 "R9" V 4443 6600 50  0000 C CNN
F 1 "10k" V 4534 6600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4580 6600 50  0001 C CNN
F 3 "~" H 4650 6600 50  0001 C CNN
	1    4650 6600
	0    1    1    0   
$EndComp
Text Label 4100 7050 2    50   ~ 0
lim0
Wire Wire Line
	4450 7050 4300 7050
Wire Wire Line
	4800 6600 4950 6600
Wire Wire Line
	4950 6600 4950 6500
$Comp
L power:+5V #PWR021
U 1 1 616D3BC9
P 4950 6500
F 0 "#PWR021" H 4950 6350 50  0001 C CNN
F 1 "+5V" H 4965 6673 50  0000 C CNN
F 2 "" H 4950 6500 50  0001 C CNN
F 3 "" H 4950 6500 50  0001 C CNN
	1    4950 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 6600 4300 6600
Wire Wire Line
	4300 6600 4300 7050
Connection ~ 4300 7050
Wire Wire Line
	4300 7050 4100 7050
Wire Wire Line
	4850 7050 4950 7050
Wire Wire Line
	4950 7050 4950 7150
$Comp
L power:GND #PWR024
U 1 1 616DF7FE
P 4950 7150
F 0 "#PWR024" H 4950 6900 50  0001 C CNN
F 1 "GND" H 4955 6977 50  0000 C CNN
F 2 "" H 4950 7150 50  0001 C CNN
F 3 "" H 4950 7150 50  0001 C CNN
	1    4950 7150
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 617032C2
P 6100 7050
F 0 "SW2" H 6100 7335 50  0000 C CNN
F 1 "LIMIT_SWITCH" H 6100 7244 50  0000 C CNN
F 2 "" H 6100 7250 50  0001 C CNN
F 3 "~" H 6100 7250 50  0001 C CNN
	1    6100 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 617032CC
P 6100 6600
F 0 "R10" V 5893 6600 50  0000 C CNN
F 1 "10k" V 5984 6600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6030 6600 50  0001 C CNN
F 3 "~" H 6100 6600 50  0001 C CNN
	1    6100 6600
	0    1    1    0   
$EndComp
Text Label 5550 7050 2    50   ~ 0
lim1
Wire Wire Line
	5900 7050 5750 7050
Wire Wire Line
	6250 6600 6400 6600
Wire Wire Line
	6400 6600 6400 6500
$Comp
L power:+5V #PWR022
U 1 1 617032DA
P 6400 6500
F 0 "#PWR022" H 6400 6350 50  0001 C CNN
F 1 "+5V" H 6415 6673 50  0000 C CNN
F 2 "" H 6400 6500 50  0001 C CNN
F 3 "" H 6400 6500 50  0001 C CNN
	1    6400 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 6600 5750 6600
Wire Wire Line
	5750 6600 5750 7050
Connection ~ 5750 7050
Wire Wire Line
	5750 7050 5550 7050
Wire Wire Line
	6300 7050 6400 7050
Wire Wire Line
	6400 7050 6400 7150
$Comp
L power:GND #PWR025
U 1 1 617032EA
P 6400 7150
F 0 "#PWR025" H 6400 6900 50  0001 C CNN
F 1 "GND" H 6405 6977 50  0000 C CNN
F 2 "" H 6400 7150 50  0001 C CNN
F 3 "" H 6400 7150 50  0001 C CNN
	1    6400 7150
	1    0    0    -1  
$EndComp
NoConn ~ 2800 4900
NoConn ~ 3200 5300
NoConn ~ 3200 5500
NoConn ~ 3200 5700
NoConn ~ 3200 5900
NoConn ~ 3200 6000
NoConn ~ 3200 6100
NoConn ~ 3200 6200
NoConn ~ 3200 6300
NoConn ~ 3200 6400
NoConn ~ 3200 6600
NoConn ~ 3200 6700
NoConn ~ 2200 5400
NoConn ~ 2200 5300
NoConn ~ 2200 6500
$Comp
L doublemagfixture-rescue:easydriver_stepper_driver U1
U 1 1 61845335
P 6100 2800
F 0 "U1" H 6100 4765 50  0000 C CNN
F 1 "easydriver_stepper_driver" H 6100 4674 50  0000 C CNN
F 2 "doublemag:easydriver_V44" H 6100 2800 50  0001 C CNN
F 3 "" H 6100 2800 50  0001 C CNN
	1    6100 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2300 4950 2300
Wire Wire Line
	4500 2300 4500 2500
Wire Wire Line
	5550 2400 5050 2400
Wire Wire Line
	4300 2400 4300 2500
Wire Wire Line
	5550 2500 5150 2500
Wire Wire Line
	4700 2500 4700 2700
Wire Wire Line
	5550 2600 5250 2600
Wire Wire Line
	4800 2600 4800 2900
Wire Wire Line
	4800 2900 4700 2900
Wire Wire Line
	5550 1100 5150 1100
Wire Wire Line
	5150 1100 5150 1000
$Comp
L power:+12V #PWR01
U 1 1 6186C24D
P 5150 1000
F 0 "#PWR01" H 5150 850 50  0001 C CNN
F 1 "+12V" H 5165 1173 50  0000 C CNN
F 2 "" H 5150 1000 50  0001 C CNN
F 3 "" H 5150 1000 50  0001 C CNN
	1    5150 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1200 5150 1200
Wire Wire Line
	5150 1200 5150 1300
$Comp
L power:GND #PWR03
U 1 1 6186E19C
P 5150 1300
F 0 "#PWR03" H 5150 1050 50  0001 C CNN
F 1 "GND" H 5155 1127 50  0000 C CNN
F 2 "" H 5150 1300 50  0001 C CNN
F 3 "" H 5150 1300 50  0001 C CNN
	1    5150 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 1200 7700 1200
Wire Wire Line
	7700 1200 7700 1300
Connection ~ 7700 1200
$Comp
L Device:R_Small R2
U 1 1 616932DC
P 7150 1200
F 0 "R2" V 7350 1200 50  0000 C CNN
F 1 "1k" V 7250 1200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7150 1200 50  0001 C CNN
F 3 "~" H 7150 1200 50  0001 C CNN
	1    7150 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 1300 7700 1300
Connection ~ 7700 1300
Wire Wire Line
	7700 1300 7700 1350
Wire Wire Line
	6650 2600 6750 2600
Wire Wire Line
	6750 2600 6750 2700
$Comp
L power:GND #PWR09
U 1 1 61893930
P 6750 2700
F 0 "#PWR09" H 6750 2450 50  0001 C CNN
F 1 "GND" H 6755 2527 50  0000 C CNN
F 2 "" H 6750 2700 50  0001 C CNN
F 3 "" H 6750 2700 50  0001 C CNN
	1    6750 2700
	1    0    0    -1  
$EndComp
NoConn ~ 5550 1500
NoConn ~ 6650 2050
NoConn ~ 6650 2150
NoConn ~ 5550 1600
NoConn ~ 6650 2500
NoConn ~ 5550 1900
NoConn ~ 5550 2000
Text Label 10000 1100 0    50   ~ 0
dir1
Text Label 10000 1200 0    50   ~ 0
step1
$Comp
L Device:R_Small R3
U 1 1 618BBFC8
P 10450 1100
F 0 "R3" V 10254 1100 50  0000 C CNN
F 1 "1k" V 10345 1100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 10450 1100 50  0001 C CNN
F 3 "~" H 10450 1100 50  0001 C CNN
	1    10450 1100
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D3
U 1 1 618BBFD2
P 10750 1100
F 0 "D3" H 10750 893 50  0000 C CNN
F 1 "DIR1" H 10750 984 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 10750 1100 50  0001 C CNN
F 3 "~" V 10750 1100 50  0001 C CNN
	1    10750 1100
	-1   0    0    1   
$EndComp
Wire Wire Line
	10650 1100 10550 1100
$Comp
L power:GND #PWR06
U 1 1 618BBFDD
P 11000 1350
F 0 "#PWR06" H 11000 1100 50  0001 C CNN
F 1 "GND" H 11005 1177 50  0000 C CNN
F 2 "" H 11000 1350 50  0001 C CNN
F 3 "" H 11000 1350 50  0001 C CNN
	1    11000 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 1200 10350 1200
Wire Wire Line
	9950 1100 10350 1100
$Comp
L Device:LED_Small D4
U 1 1 618BBFE9
P 10750 1200
F 0 "D4" H 10750 1400 50  0000 C CNN
F 1 "STEP1" H 10750 1300 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 10750 1200 50  0001 C CNN
F 3 "~" V 10750 1200 50  0001 C CNN
	1    10750 1200
	-1   0    0    1   
$EndComp
Wire Wire Line
	10650 1200 10550 1200
Wire Wire Line
	10850 1100 11000 1100
Wire Wire Line
	11000 1100 11000 1200
$Comp
L doublemagfixture-rescue:easydriver_stepper_driver U2
U 1 1 618BBFF6
P 9400 2800
F 0 "U2" H 9400 4765 50  0000 C CNN
F 1 "easydriver_stepper_driver" H 9400 4674 50  0000 C CNN
F 2 "doublemag:easydriver_V44" H 9400 2800 50  0001 C CNN
F 3 "" H 9400 2800 50  0001 C CNN
	1    9400 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 1100 8450 1000
$Comp
L power:+12V #PWR02
U 1 1 618BC00B
P 8450 1000
F 0 "#PWR02" H 8450 850 50  0001 C CNN
F 1 "+12V" H 8465 1173 50  0000 C CNN
F 2 "" H 8450 1000 50  0001 C CNN
F 3 "" H 8450 1000 50  0001 C CNN
	1    8450 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1200 8450 1200
Wire Wire Line
	8450 1200 8450 1300
$Comp
L power:GND #PWR04
U 1 1 618BC017
P 8450 1300
F 0 "#PWR04" H 8450 1050 50  0001 C CNN
F 1 "GND" H 8455 1127 50  0000 C CNN
F 2 "" H 8450 1300 50  0001 C CNN
F 3 "" H 8450 1300 50  0001 C CNN
	1    8450 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10850 1200 11000 1200
Wire Wire Line
	11000 1200 11000 1300
Connection ~ 11000 1200
$Comp
L Device:R_Small R4
U 1 1 618BC024
P 10450 1200
F 0 "R4" V 10650 1200 50  0000 C CNN
F 1 "1k" V 10550 1200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 10450 1200 50  0001 C CNN
F 3 "~" H 10450 1200 50  0001 C CNN
	1    10450 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	9950 1300 11000 1300
Connection ~ 11000 1300
Wire Wire Line
	11000 1300 11000 1350
Wire Wire Line
	9950 2600 10050 2600
Wire Wire Line
	10050 2600 10050 2700
$Comp
L power:GND #PWR010
U 1 1 618BC033
P 10050 2700
F 0 "#PWR010" H 10050 2450 50  0001 C CNN
F 1 "GND" H 10055 2527 50  0000 C CNN
F 2 "" H 10050 2700 50  0001 C CNN
F 3 "" H 10050 2700 50  0001 C CNN
	1    10050 2700
	1    0    0    -1  
$EndComp
NoConn ~ 8850 1500
NoConn ~ 9950 2050
NoConn ~ 9950 2150
NoConn ~ 8850 1600
NoConn ~ 9950 2500
NoConn ~ 8850 1900
NoConn ~ 8850 2000
Text Label 6750 3700 0    50   ~ 0
dir2
Text Label 6750 3800 0    50   ~ 0
step2
$Comp
L Device:R_Small R5
U 1 1 618C54B9
P 7200 3700
F 0 "R5" V 7004 3700 50  0000 C CNN
F 1 "1k" V 7095 3700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7200 3700 50  0001 C CNN
F 3 "~" H 7200 3700 50  0001 C CNN
	1    7200 3700
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D5
U 1 1 618C54C3
P 7500 3700
F 0 "D5" H 7500 3493 50  0000 C CNN
F 1 "DIR2" H 7500 3584 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 7500 3700 50  0001 C CNN
F 3 "~" V 7500 3700 50  0001 C CNN
	1    7500 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 3700 7300 3700
$Comp
L power:GND #PWR015
U 1 1 618C54CE
P 7750 3950
F 0 "#PWR015" H 7750 3700 50  0001 C CNN
F 1 "GND" H 7755 3777 50  0000 C CNN
F 2 "" H 7750 3950 50  0001 C CNN
F 3 "" H 7750 3950 50  0001 C CNN
	1    7750 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3800 7100 3800
Wire Wire Line
	6700 3700 7100 3700
$Comp
L Device:LED_Small D6
U 1 1 618C54DA
P 7500 3800
F 0 "D6" H 7500 4000 50  0000 C CNN
F 1 "STEP2" H 7500 3900 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 7500 3800 50  0001 C CNN
F 3 "~" V 7500 3800 50  0001 C CNN
	1    7500 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	7400 3800 7300 3800
Wire Wire Line
	7600 3700 7750 3700
Wire Wire Line
	7750 3700 7750 3800
$Comp
L doublemagfixture-rescue:easydriver_stepper_driver U3
U 1 1 618C54E7
P 6150 5400
F 0 "U3" H 6150 7365 50  0000 C CNN
F 1 "easydriver_stepper_driver" H 6150 7274 50  0000 C CNN
F 2 "doublemag:easydriver_V44" H 6150 5400 50  0001 C CNN
F 3 "" H 6150 5400 50  0001 C CNN
	1    6150 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3700 5200 3700
Wire Wire Line
	5200 3700 5200 3600
$Comp
L power:+12V #PWR011
U 1 1 618C54FC
P 5200 3600
F 0 "#PWR011" H 5200 3450 50  0001 C CNN
F 1 "+12V" H 5215 3773 50  0000 C CNN
F 2 "" H 5200 3600 50  0001 C CNN
F 3 "" H 5200 3600 50  0001 C CNN
	1    5200 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3800 5200 3800
Wire Wire Line
	5200 3800 5200 3900
$Comp
L power:GND #PWR013
U 1 1 618C5508
P 5200 3900
F 0 "#PWR013" H 5200 3650 50  0001 C CNN
F 1 "GND" H 5205 3727 50  0000 C CNN
F 2 "" H 5200 3900 50  0001 C CNN
F 3 "" H 5200 3900 50  0001 C CNN
	1    5200 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3800 7750 3800
Wire Wire Line
	7750 3800 7750 3900
Connection ~ 7750 3800
$Comp
L Device:R_Small R6
U 1 1 618C5515
P 7200 3800
F 0 "R6" V 7400 3800 50  0000 C CNN
F 1 "1k" V 7300 3800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7200 3800 50  0001 C CNN
F 3 "~" H 7200 3800 50  0001 C CNN
	1    7200 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	6700 3900 7750 3900
Connection ~ 7750 3900
Wire Wire Line
	7750 3900 7750 3950
Wire Wire Line
	6700 5200 6800 5200
Wire Wire Line
	6800 5200 6800 5300
$Comp
L power:GND #PWR019
U 1 1 618C5524
P 6800 5300
F 0 "#PWR019" H 6800 5050 50  0001 C CNN
F 1 "GND" H 6805 5127 50  0000 C CNN
F 2 "" H 6800 5300 50  0001 C CNN
F 3 "" H 6800 5300 50  0001 C CNN
	1    6800 5300
	1    0    0    -1  
$EndComp
NoConn ~ 5600 4100
NoConn ~ 6700 4650
NoConn ~ 6700 4750
NoConn ~ 5600 4200
NoConn ~ 5600 4500
NoConn ~ 5600 4600
Text Label 10050 3700 0    50   ~ 0
dir3
Text Label 10050 3800 0    50   ~ 0
step3
$Comp
L Device:R_Small R7
U 1 1 618C5541
P 10500 3700
F 0 "R7" V 10304 3700 50  0000 C CNN
F 1 "1k" V 10395 3700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 10500 3700 50  0001 C CNN
F 3 "~" H 10500 3700 50  0001 C CNN
	1    10500 3700
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D7
U 1 1 618C554B
P 10800 3700
F 0 "D7" H 10800 3493 50  0000 C CNN
F 1 "DIR3" H 10800 3584 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 10800 3700 50  0001 C CNN
F 3 "~" V 10800 3700 50  0001 C CNN
	1    10800 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	10700 3700 10600 3700
$Comp
L power:GND #PWR016
U 1 1 618C5556
P 11050 3950
F 0 "#PWR016" H 11050 3700 50  0001 C CNN
F 1 "GND" H 11055 3777 50  0000 C CNN
F 2 "" H 11050 3950 50  0001 C CNN
F 3 "" H 11050 3950 50  0001 C CNN
	1    11050 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3800 10400 3800
Wire Wire Line
	10000 3700 10400 3700
$Comp
L Device:LED_Small D8
U 1 1 618C5562
P 10800 3800
F 0 "D8" H 10800 4000 50  0000 C CNN
F 1 "STEP3" H 10800 3900 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" V 10800 3800 50  0001 C CNN
F 3 "~" V 10800 3800 50  0001 C CNN
	1    10800 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	10700 3800 10600 3800
Wire Wire Line
	10900 3700 11050 3700
Wire Wire Line
	11050 3700 11050 3800
$Comp
L doublemagfixture-rescue:easydriver_stepper_driver U4
U 1 1 618C556F
P 9450 5400
F 0 "U4" H 9450 7365 50  0000 C CNN
F 1 "easydriver_stepper_driver" H 9450 7274 50  0000 C CNN
F 2 "doublemag:easydriver_V44" H 9450 5400 50  0001 C CNN
F 3 "" H 9450 5400 50  0001 C CNN
	1    9450 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 3700 8500 3700
Wire Wire Line
	8500 3700 8500 3600
$Comp
L power:+12V #PWR012
U 1 1 618C5584
P 8500 3600
F 0 "#PWR012" H 8500 3450 50  0001 C CNN
F 1 "+12V" H 8515 3773 50  0000 C CNN
F 2 "" H 8500 3600 50  0001 C CNN
F 3 "" H 8500 3600 50  0001 C CNN
	1    8500 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 3800 8500 3800
Wire Wire Line
	8500 3800 8500 3900
$Comp
L power:GND #PWR014
U 1 1 618C5590
P 8500 3900
F 0 "#PWR014" H 8500 3650 50  0001 C CNN
F 1 "GND" H 8505 3727 50  0000 C CNN
F 2 "" H 8500 3900 50  0001 C CNN
F 3 "" H 8500 3900 50  0001 C CNN
	1    8500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10900 3800 11050 3800
Wire Wire Line
	11050 3800 11050 3900
Connection ~ 11050 3800
$Comp
L Device:R_Small R8
U 1 1 618C559D
P 10500 3800
F 0 "R8" V 10700 3800 50  0000 C CNN
F 1 "1k" V 10600 3800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 10500 3800 50  0001 C CNN
F 3 "~" H 10500 3800 50  0001 C CNN
	1    10500 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	10000 3900 11050 3900
Connection ~ 11050 3900
Wire Wire Line
	11050 3900 11050 3950
Wire Wire Line
	10000 5200 10100 5200
Wire Wire Line
	10100 5200 10100 5300
$Comp
L power:GND #PWR020
U 1 1 618C55AC
P 10100 5300
F 0 "#PWR020" H 10100 5050 50  0001 C CNN
F 1 "GND" H 10105 5127 50  0000 C CNN
F 2 "" H 10100 5300 50  0001 C CNN
F 3 "" H 10100 5300 50  0001 C CNN
	1    10100 5300
	1    0    0    -1  
$EndComp
NoConn ~ 8900 4100
NoConn ~ 10000 4650
NoConn ~ 10000 4750
NoConn ~ 8900 4200
NoConn ~ 10000 5100
NoConn ~ 8900 4500
NoConn ~ 8900 4600
NoConn ~ 2200 6600
$Comp
L Connector:Barrel_Jack_MountingPin J1
U 1 1 618F82D4
P 2100 1700
F 0 "J1" H 2157 2017 50  0000 C CNN
F 1 "Barrel_Jack_MountingPin" H 2157 1926 50  0000 C CNN
F 2 "Connector_BarrelJack:BarrelJack_Horizontal" H 2150 1660 50  0001 C CNN
F 3 "~" H 2150 1660 50  0001 C CNN
	1    2100 1700
	1    0    0    -1  
$EndComp
NoConn ~ 2100 2000
NoConn ~ 6700 5100
$Comp
L power:+5V #PWR018
U 1 1 61622B63
P 2900 4900
F 0 "#PWR018" H 2900 4750 50  0001 C CNN
F 1 "+5V" H 2915 5073 50  0000 C CNN
F 2 "" H 2900 4900 50  0001 C CNN
F 3 "" H 2900 4900 50  0001 C CNN
	1    2900 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2700 5250 2600
Connection ~ 5250 2600
Wire Wire Line
	5250 2600 4800 2600
Wire Wire Line
	5150 2700 5150 2500
Connection ~ 5150 2500
Wire Wire Line
	5150 2500 4700 2500
Wire Wire Line
	5050 2700 5050 2400
Connection ~ 5050 2400
Wire Wire Line
	5050 2400 4300 2400
Wire Wire Line
	4950 2700 4950 2300
Connection ~ 4950 2300
Wire Wire Line
	4950 2300 4500 2300
$Comp
L Motor:Stepper_Motor_bipolar M3
U 1 1 619BDAE8
P 4450 5400
F 0 "M3" V 4650 5450 50  0000 R CNN
F 1 "12V_350mA_Stepper_Motor" V 4750 5900 50  0000 R CNN
F 2 "" H 4460 5390 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 4460 5390 50  0001 C CNN
	1    4450 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 4900 5000 4900
Wire Wire Line
	4550 4900 4550 5100
Wire Wire Line
	5600 5000 5100 5000
Wire Wire Line
	4350 5000 4350 5100
Wire Wire Line
	5600 5100 5200 5100
Wire Wire Line
	4750 5100 4750 5300
Wire Wire Line
	5600 5200 5300 5200
Wire Wire Line
	4850 5200 4850 5500
Wire Wire Line
	4850 5500 4750 5500
$Comp
L Connector:Screw_Terminal_01x04 J4
U 1 1 619BDAFB
P 5200 5500
F 0 "J4" V 5072 5680 50  0000 L CNN
F 1 "Screw_Terminal_01x04" V 5163 5680 50  0000 L CNN
F 2 "doublemag:CONN_1x4_3.81mm" H 5200 5500 50  0001 C CNN
F 3 "~" H 5200 5500 50  0001 C CNN
	1    5200 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 5300 5300 5200
Connection ~ 5300 5200
Wire Wire Line
	5300 5200 4850 5200
Wire Wire Line
	5200 5300 5200 5100
Connection ~ 5200 5100
Wire Wire Line
	5200 5100 4750 5100
Wire Wire Line
	5100 5300 5100 5000
Connection ~ 5100 5000
Wire Wire Line
	5100 5000 4350 5000
Wire Wire Line
	5000 5300 5000 4900
Connection ~ 5000 4900
Wire Wire Line
	5000 4900 4550 4900
$Comp
L Motor:Stepper_Motor_bipolar M4
U 1 1 619C45F1
P 7750 5400
F 0 "M4" V 7950 5450 50  0000 R CNN
F 1 "12V_350mA_Stepper_Motor" V 8050 5900 50  0000 R CNN
F 2 "" H 7760 5390 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 7760 5390 50  0001 C CNN
	1    7750 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	8900 4900 8300 4900
Wire Wire Line
	7850 4900 7850 5100
Wire Wire Line
	8900 5000 8400 5000
Wire Wire Line
	7650 5000 7650 5100
Wire Wire Line
	8900 5100 8500 5100
Wire Wire Line
	8050 5100 8050 5300
Wire Wire Line
	8900 5200 8600 5200
Wire Wire Line
	8150 5200 8150 5500
Wire Wire Line
	8150 5500 8050 5500
$Comp
L Connector:Screw_Terminal_01x04 J5
U 1 1 619C4604
P 8500 5500
F 0 "J5" V 8372 5680 50  0000 L CNN
F 1 "Screw_Terminal_01x04" V 8463 5680 50  0000 L CNN
F 2 "doublemag:CONN_1x4_3.81mm" H 8500 5500 50  0001 C CNN
F 3 "~" H 8500 5500 50  0001 C CNN
	1    8500 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	8600 5300 8600 5200
Connection ~ 8600 5200
Wire Wire Line
	8600 5200 8150 5200
Wire Wire Line
	8500 5300 8500 5100
Connection ~ 8500 5100
Wire Wire Line
	8500 5100 8050 5100
Wire Wire Line
	8400 5300 8400 5000
Connection ~ 8400 5000
Wire Wire Line
	8400 5000 7650 5000
Wire Wire Line
	8300 5300 8300 4900
Connection ~ 8300 4900
Wire Wire Line
	8300 4900 7850 4900
$Comp
L Motor:Stepper_Motor_bipolar M2
U 1 1 619CD0F7
P 7700 2800
F 0 "M2" V 7900 2850 50  0000 R CNN
F 1 "12V_350mA_Stepper_Motor" V 8000 3300 50  0000 R CNN
F 2 "" H 7710 2790 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 7710 2790 50  0001 C CNN
	1    7700 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	8850 2300 8250 2300
Wire Wire Line
	7800 2300 7800 2500
Wire Wire Line
	8850 2400 8350 2400
Wire Wire Line
	7600 2400 7600 2500
Wire Wire Line
	8850 2500 8450 2500
Wire Wire Line
	8000 2500 8000 2700
Wire Wire Line
	8850 2600 8550 2600
Wire Wire Line
	8100 2600 8100 2900
Wire Wire Line
	8100 2900 8000 2900
$Comp
L Connector:Screw_Terminal_01x04 J3
U 1 1 619CD10A
P 8450 2900
F 0 "J3" V 8322 3080 50  0000 L CNN
F 1 "Screw_Terminal_01x04" V 8413 3080 50  0000 L CNN
F 2 "doublemag:CONN_1x4_3.81mm" H 8450 2900 50  0001 C CNN
F 3 "~" H 8450 2900 50  0001 C CNN
	1    8450 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	8550 2700 8550 2600
Connection ~ 8550 2600
Wire Wire Line
	8550 2600 8100 2600
Wire Wire Line
	8450 2700 8450 2500
Connection ~ 8450 2500
Wire Wire Line
	8450 2500 8000 2500
Wire Wire Line
	8350 2700 8350 2400
Connection ~ 8350 2400
Wire Wire Line
	8350 2400 7600 2400
Wire Wire Line
	8250 2700 8250 2300
Connection ~ 8250 2300
Wire Wire Line
	8250 2300 7800 2300
Wire Wire Line
	1500 5500 2200 5500
Wire Wire Line
	1500 6200 2200 6200
Wire Wire Line
	1950 6300 2200 6300
Wire Wire Line
	1950 6400 2200 6400
$Comp
L Connector:Screw_Terminal_01x08 J6
U 1 1 61B05C26
P 1300 5800
F 0 "J6" H 1218 6317 50  0000 C CNN
F 1 "Screw_Terminal_01x08" H 1218 6226 50  0000 C CNN
F 2 "doublemag:CONN_1x8_3.81mm" H 1300 5800 50  0001 C CNN
F 3 "~" H 1300 5800 50  0001 C CNN
	1    1300 5800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1500 5600 2200 5600
Wire Wire Line
	1500 5700 2200 5700
Wire Wire Line
	1500 5800 2200 5800
Wire Wire Line
	1500 5900 2200 5900
Wire Wire Line
	1500 6000 2200 6000
Wire Wire Line
	1500 6100 2200 6100
$Comp
L Connector:Screw_Terminal_01x08 J7
U 1 1 61B866FF
P 1750 3400
F 0 "J7" H 1668 3917 50  0000 C CNN
F 1 "Screw_Terminal_01x08" H 1668 3826 50  0000 C CNN
F 2 "doublemag:CONN_1x8_3.81mm" H 1750 3400 50  0001 C CNN
F 3 "~" H 1750 3400 50  0001 C CNN
	1    1750 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1950 3100 2400 3100
Wire Wire Line
	2400 3100 2400 2950
$Comp
L power:+12V #PWR0101
U 1 1 61BC5CAE
P 2400 2950
F 0 "#PWR0101" H 2400 2800 50  0001 C CNN
F 1 "+12V" H 2415 3123 50  0000 C CNN
F 2 "" H 2400 2950 50  0001 C CNN
F 3 "" H 2400 2950 50  0001 C CNN
	1    2400 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3800 2400 3800
Wire Wire Line
	2400 3800 2400 3950
$Comp
L power:GND #PWR0102
U 1 1 61BCE94C
P 2400 3950
F 0 "#PWR0102" H 2400 3700 50  0001 C CNN
F 1 "GND" H 2405 3777 50  0000 C CNN
F 2 "" H 2400 3950 50  0001 C CNN
F 3 "" H 2400 3950 50  0001 C CNN
	1    2400 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3200 2650 3200
$Comp
L power:+5V #PWR0103
U 1 1 61BD77DD
P 2650 2950
F 0 "#PWR0103" H 2650 2800 50  0001 C CNN
F 1 "+5V" H 2665 3123 50  0000 C CNN
F 2 "" H 2650 2950 50  0001 C CNN
F 3 "" H 2650 2950 50  0001 C CNN
	1    2650 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2950 2650 3200
Text Label 2400 3300 0    50   ~ 0
lim0
Text Label 2400 3400 0    50   ~ 0
lim1
Wire Wire Line
	1950 3300 2050 3300
Wire Wire Line
	2400 3400 2150 3400
Wire Wire Line
	2050 3300 2050 3500
Wire Wire Line
	2050 3500 1950 3500
Connection ~ 2050 3300
Wire Wire Line
	2050 3300 2400 3300
Wire Wire Line
	2150 3400 2150 3600
Wire Wire Line
	2150 3600 1950 3600
Connection ~ 2150 3400
Wire Wire Line
	2150 3400 1950 3400
Text Notes 1650 3300 2    50   ~ 0
from arduino
Text Notes 1650 3400 2    50   ~ 0
from arduino
Text Notes 1650 3500 2    50   ~ 0
to fixture
Text Notes 1650 3600 2    50   ~ 0
to fixture
Text Notes 5350 3000 0    50   ~ 0
to fixture
Text Notes 5400 5600 0    50   ~ 0
to fixture
Text Notes 8700 5600 0    50   ~ 0
to fixture
Text Notes 8650 3000 0    50   ~ 0
to fixture
$Comp
L Mechanical:MountingHole H1
U 1 1 61660AAE
P 700 700
F 0 "H1" H 800 746 50  0000 L CNN
F 1 "MountingHole" H 800 655 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 700 700 50  0001 C CNN
F 3 "~" H 700 700 50  0001 C CNN
	1    700  700 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 6166153D
P 700 950
F 0 "H2" H 800 996 50  0000 L CNN
F 1 "MountingHole" H 800 905 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 700 950 50  0001 C CNN
F 3 "~" H 700 950 50  0001 C CNN
	1    700  950 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 616617E1
P 700 1200
F 0 "H3" H 800 1246 50  0000 L CNN
F 1 "MountingHole" H 800 1155 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 700 1200 50  0001 C CNN
F 3 "~" H 700 1200 50  0001 C CNN
	1    700  1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 1100 8450 1100
Wire Wire Line
	2800 7000 2700 7000
$Comp
L Connector:Screw_Terminal_01x04 J2
U 1 1 6194E9D8
P 5150 2900
F 0 "J2" V 5022 3080 50  0000 L CNN
F 1 "Screw_Terminal_01x04" V 5113 3080 50  0000 L CNN
F 2 "doublemag:CONN_1x4_3.81mm" H 5150 2900 50  0001 C CNN
F 3 "~" H 5150 2900 50  0001 C CNN
	1    5150 2900
	0    1    1    0   
$EndComp
Text Notes 3950 7550 0    50   ~ 0
Limit switches off-board
Wire Notes Line
	3850 7650 6650 7650
Wire Notes Line
	6650 7650 6650 6100
Wire Notes Line
	6650 6100 3850 6100
Wire Notes Line
	3850 6100 3850 7650
Text Notes 1000 7550 0    50   ~ 0
Arduino off-board
Wire Notes Line
	3400 7650 3400 4500
Wire Notes Line
	3400 4500 900  4500
Wire Notes Line
	900  4500 900  7650
Wire Notes Line
	900  7650 3400 7650
Wire Wire Line
	1950 3700 2400 3700
Wire Wire Line
	2400 3700 2400 3800
Connection ~ 2400 3800
Text Notes 1650 3700 2    50   ~ 0
to fixture
Text Notes 1650 3100 2    50   ~ 0
to arduino
Text Notes 1650 3200 2    50   ~ 0
from arduino
Text Notes 1650 3800 2    50   ~ 0
to arduino
$EndSCHEMATC
