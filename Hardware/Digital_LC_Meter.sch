EESchema Schematic File Version 4
LIBS:power
LIBS:device
LIBS:74xx
LIBS:audio
LIBS:interface
LIBS:Digital_LC_Meter-cache
EELAYER 26 0
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
Text Notes 8875 2525 0    63   ~ 0
TX\nRX\nRST\nGND\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12
Text Notes 9800 2525 0    63   ~ 0
VIN\nGND\nRST\n5V\n21\n20\n19\n18\n17\n16\n15\n14\nAREF\n3V3\n13\n
Wire Wire Line
	1525 1650 2025 1650
Text Label 2600 1450 0    60   ~ 0
VIN
Wire Wire Line
	1525 1750 2025 1750
Wire Wire Line
	2025 1750 2025 2000
Text Label 2600 2000 0    60   ~ 0
GND
Wire Wire Line
	10300 1100 10525 1100
Text Label 10525 1100 0    60   ~ 0
VIN
Wire Wire Line
	10300 1200 10525 1200
Text Label 10525 1200 0    60   ~ 0
GND
Wire Wire Line
	1625 6150 1375 6150
Text Label 1375 6150 0    60   ~ 0
SDA0
Wire Wire Line
	1625 6250 1375 6250
Text Label 1375 6250 0    60   ~ 0
SCL0
Wire Wire Line
	1625 6350 1375 6350
Text Label 1375 6350 0    60   ~ 0
GND
Wire Wire Line
	1625 6450 1375 6450
Text Label 1375 6450 0    60   ~ 0
5V
Wire Wire Line
	10300 1400 10525 1400
Text Label 10525 1400 0    60   ~ 0
5V
Wire Wire Line
	10300 1800 10525 1800
Text Label 10525 1800 0    60   ~ 0
SDA
Wire Wire Line
	10300 1700 10525 1700
Text Label 10525 1700 0    60   ~ 0
SCL
Wire Wire Line
	3575 6375 2850 6375
Text Label 2850 6375 0    60   ~ 0
GND
Wire Wire Line
	3575 6275 3325 6275
Wire Wire Line
	3325 6275 3325 6125
Text Label 3325 6125 2    60   ~ 0
INCR_PIN6
Wire Wire Line
	3575 6475 3325 6475
Wire Wire Line
	3325 6475 3325 6650
Text Label 3325 6650 2    60   ~ 0
DECR_PIN4
Wire Wire Line
	6375 2550 6375 2450
Wire Wire Line
	6375 2450 6475 2450
Wire Wire Line
	6475 2450 6475 2550
Text Label 6275 2350 3    60   ~ 0
5V
Wire Wire Line
	6275 3150 6275 3250
Text Label 6275 3450 1    60   ~ 0
GND
Wire Wire Line
	6375 3150 6375 3250
Wire Wire Line
	6375 3250 6275 3250
Connection ~ 6275 3250
Wire Wire Line
	5125 2750 5375 2750
Wire Wire Line
	5825 2950 5825 3700
Wire Wire Line
	5825 2950 6075 2950
Wire Wire Line
	5825 4100 5825 4450
Text Label 5825 4450 1    60   ~ 0
GND
Wire Wire Line
	6475 3700 5825 3700
Connection ~ 5825 3700
Wire Wire Line
	7125 3700 6775 3700
Wire Wire Line
	7125 2000 7125 2100
Wire Wire Line
	7125 2100 6575 2100
Wire Wire Line
	6275 2100 5375 2100
Wire Wire Line
	5375 2000 5375 2100
Connection ~ 5375 2750
Connection ~ 7125 2100
Connection ~ 5375 2100
Wire Wire Line
	5375 1700 5375 1550
Text Label 5375 1550 3    60   ~ 0
5V
Wire Wire Line
	7125 1700 7125 1550
Text Label 7125 1550 3    60   ~ 0
5V
Wire Wire Line
	5375 3925 5375 4450
Text Label 5375 4450 1    60   ~ 0
GND
Wire Wire Line
	4475 2750 4475 3475
Wire Wire Line
	3025 2750 4025 2750
Text Label 8850 6125 1    60   ~ 0
GND
Wire Wire Line
	4025 2750 4025 3225
Connection ~ 4475 2750
Wire Wire Line
	4025 3775 4025 4175
Text Label 4025 4175 1    60   ~ 0
GND
Wire Wire Line
	3825 3225 4025 3225
Connection ~ 4025 3225
Connection ~ 4025 2750
Wire Wire Line
	3625 3225 3025 3225
Wire Wire Line
	1325 4125 2275 4125
Wire Wire Line
	1325 4225 2275 4225
Text Label 3025 2750 0    60   ~ 0
IND_PARALLEL
Text Label 3025 3225 0    60   ~ 0
IND_SERIES
Wire Wire Line
	2275 3925 1550 3925
Text Label 1550 3925 0    60   ~ 0
IND_PARALLEL
Wire Wire Line
	2275 4025 1550 4025
Wire Wire Line
	2275 4325 1550 4325
Text Label 1550 4325 0    60   ~ 0
IND_SERIES
Wire Wire Line
	2275 4425 1550 4425
Text Label 1550 4025 0    60   ~ 0
IND_SERIES
Text Label 1550 4425 0    60   ~ 0
CL_BAR_PIN2
Wire Wire Line
	8600 1500 7825 1500
Text Label 7825 1500 0    60   ~ 0
CL_BAR_PIN2
Text Notes 2550 4475 0    63   ~ 0
DPDT:\nUP1\nDOWN1\nIN1\nIN2\nUP2\nDOWN2
Wire Wire Line
	9550 5875 9550 6125
Text Label 9550 6125 1    60   ~ 0
GND
Wire Wire Line
	9550 5150 9550 4800
Wire Wire Line
	8850 4800 9550 4800
Connection ~ 9550 4800
Text Label 10400 4800 2    60   ~ 0
RELAY_PIN3
Wire Wire Line
	8600 1600 7825 1600
Text Label 7825 1600 0    60   ~ 0
RELAY_PIN3
Wire Wire Line
	8600 1900 7825 1900
Text Label 7825 1900 0    60   ~ 0
INCR_PIN6
Text Label 7825 1700 0    60   ~ 0
DECR_PIN4
Wire Wire Line
	8600 1700 7825 1700
Connection ~ 7125 3700
Wire Wire Line
	7125 4175 7125 4425
Text Label 7125 4425 0    60   ~ 0
OSC_SIG_PIN5
Wire Wire Line
	8600 1800 7825 1800
Text Label 7825 1800 0    60   ~ 0
OSC_SIG_PIN5
Wire Wire Line
	8600 2000 7825 2000
Text Label 7825 2000 0    60   ~ 0
CAL_LK_PIN7
Wire Wire Line
	8600 2100 7825 2100
Text Label 7825 2100 0    60   ~ 0
CAL_ERASE_PIN8
Wire Wire Line
	5600 5950 4825 5950
Text Label 4825 5950 0    60   ~ 0
CAL_LK_PIN7
Wire Wire Line
	5600 6050 4825 6050
Text Label 4825 6050 0    60   ~ 0
GND
Wire Wire Line
	5625 6600 4800 6600
Text Label 4800 6600 0    60   ~ 0
CAL_ERASE_PIN8
Wire Wire Line
	5625 6700 4800 6700
Text Label 4800 6700 0    60   ~ 0
GND
Wire Wire Line
	8650 4800 4475 4800
Wire Wire Line
	4475 4800 4475 3775
Wire Wire Line
	2025 2000 2300 2000
Wire Wire Line
	2025 1650 2025 1450
Wire Wire Line
	2025 1450 2300 1450
Wire Wire Line
	2300 2000 2300 1875
Connection ~ 2300 2000
Wire Wire Line
	2300 1450 2300 1575
Connection ~ 2300 1450
Wire Wire Line
	10300 1300 10525 1300
Text Label 10525 1300 0    60   ~ 0
RESET_PIN
Wire Wire Line
	4825 7150 5625 7150
Text Label 4825 7150 0    60   ~ 0
RESET_PIN
Wire Wire Line
	5625 7250 4825 7250
Text Label 4825 7250 0    60   ~ 0
GND
NoConn ~ 8600 1100
NoConn ~ 8600 1200
NoConn ~ 8600 1300
NoConn ~ 8600 1400
NoConn ~ 8600 2200
NoConn ~ 8600 2300
NoConn ~ 8600 2400
NoConn ~ 8600 2500
NoConn ~ 10300 2500
NoConn ~ 10300 2400
NoConn ~ 10300 2300
NoConn ~ 10300 2200
NoConn ~ 10300 2100
NoConn ~ 10300 2000
NoConn ~ 10300 1900
NoConn ~ 10300 1500
NoConn ~ 10300 1600
Text Notes 9050 1000 0    60   ~ 0
Arduino Nano\nPin Mappings:
Wire Wire Line
	6275 2550 6275 2350
Wire Wire Line
	6675 2850 7125 2850
Connection ~ 7125 2850
Wire Wire Line
	6275 3250 6275 3450
Wire Wire Line
	5825 3700 5825 3800
Wire Wire Line
	5375 2750 6075 2750
Wire Wire Line
	5375 2750 5375 3625
Wire Wire Line
	7125 2100 7125 2850
Wire Wire Line
	5375 2100 5375 2750
Wire Wire Line
	4475 2750 4825 2750
Wire Wire Line
	4025 3225 4025 3475
Wire Wire Line
	4025 2750 4475 2750
Wire Wire Line
	9550 4800 10400 4800
Wire Wire Line
	7125 3700 7125 3875
Wire Wire Line
	2300 2000 2600 2000
Wire Wire Line
	2300 1450 2600 1450
Wire Wire Line
	7125 2850 7125 3700
$Comp
L Connector_Generic:Conn_01x02 J_PWR1
U 1 1 5B972046
P 1325 1650
F 0 "J_PWR1" H 1245 1867 50  0000 C CNN
F 1 "Conn_01x02" H 1245 1776 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1325 1650 50  0001 C CNN
F 3 "~" H 1325 1650 50  0001 C CNN
	1    1325 1650
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J_INPUT1
U 1 1 5B9720BC
P 1125 4125
F 0 "J_INPUT1" H 1045 4342 50  0000 C CNN
F 1 "Conn_01x02" H 1045 4251 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1125 4125 50  0001 C CNN
F 3 "~" H 1125 4125 50  0001 C CNN
	1    1125 4125
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J_CALJUMPER1
U 1 1 5B972118
P 5800 5950
F 0 "J_CALJUMPER1" H 5880 5942 50  0000 L CNN
F 1 "Conn_01x02" H 5880 5851 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5800 5950 50  0001 C CNN
F 3 "~" H 5800 5950 50  0001 C CNN
	1    5800 5950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J_ERASEBTN1
U 1 1 5B97221A
P 5825 6600
F 0 "J_ERASEBTN1" H 5905 6592 50  0000 L CNN
F 1 "Conn_01x02" H 5905 6501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5825 6600 50  0001 C CNN
F 3 "~" H 5825 6600 50  0001 C CNN
	1    5825 6600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J_RESETBTN1
U 1 1 5B97228E
P 5825 7150
F 0 "J_RESETBTN1" H 5905 7142 50  0000 L CNN
F 1 "Conn_01x02" H 5905 7051 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5825 7150 50  0001 C CNN
F 3 "~" H 5825 7150 50  0001 C CNN
	1    5825 7150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J_NUDGEBTN1
U 1 1 5B972482
P 3775 6375
F 0 "J_NUDGEBTN1" H 3855 6417 50  0000 L CNN
F 1 "Conn_01x03" H 3855 6326 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3775 6375 50  0001 C CNN
F 3 "~" H 3775 6375 50  0001 C CNN
	1    3775 6375
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J_OLED1
U 1 1 5B972605
P 1825 6250
F 0 "J_OLED1" H 1905 6242 50  0000 L CNN
F 1 "Conn_01x04" H 1905 6151 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1825 6250 50  0001 C CNN
F 3 "~" H 1825 6250 50  0001 C CNN
	1    1825 6250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J_LCSELECTSPDT1
U 1 1 5B972818
P 2475 4125
F 0 "J_LCSELECTSPDT1" H 2475 3650 50  0000 L CNN
F 1 "Conn_01x06" H 2475 3575 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2475 4125 50  0001 C CNN
F 3 "~" H 2475 4125 50  0001 C CNN
	1    2475 4125
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J_NANOTX1
U 1 1 5B972B1E
P 8800 1800
F 0 "J_NANOTX1" H 8550 2775 50  0000 L CNN
F 1 "Conn_01x15" H 8550 2675 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x15_P2.54mm_Vertical" H 8800 1800 50  0001 C CNN
F 3 "~" H 8800 1800 50  0001 C CNN
	1    8800 1800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x15 J_NANOVIN1
U 1 1 5B972BA3
P 10100 1800
F 0 "J_NANOVIN1" H 10100 2775 50  0000 C CNN
F 1 "Conn_01x15" H 10100 2675 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x15_P2.54mm_Vertical" H 10100 1800 50  0001 C CNN
F 3 "~" H 10100 1800 50  0001 C CNN
	1    10100 1800
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R_BIAS3
U 1 1 5B972C8E
P 7125 1850
F 0 "R_BIAS3" H 7195 1896 50  0000 L CNN
F 1 "4K7" H 7195 1805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7055 1850 50  0001 C CNN
F 3 "~" H 7125 1850 50  0001 C CNN
	1    7125 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R_BIAS1
U 1 1 5B972D66
P 5375 1850
F 0 "R_BIAS1" H 5445 1896 50  0000 L CNN
F 1 "100K" H 5445 1805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5305 1850 50  0001 C CNN
F 3 "~" H 5375 1850 50  0001 C CNN
	1    5375 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R_BIAS2
U 1 1 5B972DFC
P 5375 3775
F 0 "R_BIAS2" H 5445 3821 50  0000 L CNN
F 1 "100K" H 5445 3730 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5305 3775 50  0001 C CNN
F 3 "~" H 5375 3775 50  0001 C CNN
	1    5375 3775
	1    0    0    -1  
$EndComp
$Comp
L Device:R R_SIG1
U 1 1 5B972E6E
P 7125 4025
F 0 "R_SIG1" H 7195 4071 50  0000 L CNN
F 1 "6K8" H 7195 3980 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7055 4025 50  0001 C CNN
F 3 "~" H 7125 4025 50  0001 C CNN
	1    7125 4025
	1    0    0    -1  
$EndComp
$Comp
L Device:R R_OSC1
U 1 1 5B972EF1
P 6625 3700
F 0 "R_OSC1" V 6418 3700 50  0000 C CNN
F 1 "47K" V 6509 3700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6555 3700 50  0001 C CNN
F 3 "~" H 6625 3700 50  0001 C CNN
	1    6625 3700
	0    1    1    0   
$EndComp
$Comp
L Device:C C_MICA2
U 1 1 5B972FE8
P 4475 3625
F 0 "C_MICA2" H 4360 3579 50  0000 R CNN
F 1 "1nF" H 4360 3670 50  0000 R CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.0mm_P5.00mm" H 4513 3475 50  0001 C CNN
F 3 "~" H 4475 3625 50  0001 C CNN
	1    4475 3625
	-1   0    0    1   
$EndComp
$Comp
L Device:C C_MICA1
U 1 1 5B97310A
P 4025 3625
F 0 "C_MICA1" H 4425 3550 50  0000 R CNN
F 1 "1nF" H 4325 3675 50  0000 R CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.0mm_P5.00mm" H 4063 3475 50  0001 C CNN
F 3 "~" H 4025 3625 50  0001 C CNN
	1    4025 3625
	-1   0    0    1   
$EndComp
$Comp
L Device:C C_BP1
U 1 1 5B973184
P 2300 1725
F 0 "C_BP1" H 2185 1679 50  0000 R CNN
F 1 "100nF" H 2185 1770 50  0000 R CNN
F 2 "Capacitor_THT:C_Rect_L7.0mm_W2.0mm_P5.00mm" H 2338 1575 50  0001 C CNN
F 3 "~" H 2300 1725 50  0001 C CNN
	1    2300 1725
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C_COUP1
U 1 1 5B9733EB
P 4975 2750
F 0 "C_COUP1" V 4720 2750 50  0000 C CNN
F 1 "10uF" V 4811 2750 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5013 2600 50  0001 C CNN
F 3 "~" H 4975 2750 50  0001 C CNN
	1    4975 2750
	0    1    1    0   
$EndComp
$Comp
L Device:CP C_OSC1
U 1 1 5B97357D
P 5825 3950
F 0 "C_OSC1" H 5943 3996 50  0000 L CNN
F 1 "10uF" H 5943 3905 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5863 3800 50  0001 C CNN
F 3 "~" H 5825 3950 50  0001 C CNN
	1    5825 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5B9782E4
P 6425 2100
F 0 "R3" V 6218 2100 50  0000 C CNN
F 1 "100K" V 6309 2100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6355 2100 50  0001 C CNN
F 3 "~" H 6425 2100 50  0001 C CNN
	1    6425 2100
	0    1    1    0   
$EndComp
$Comp
L Comparator:LM311 U_OSC1
U 1 1 5B978CA1
P 6375 2850
F 0 "U_OSC1" H 6716 2896 50  0000 L CNN
F 1 "LM311" H 6716 2805 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm_Socket" H 6375 2850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm311.pdf" H 6375 2850 50  0001 C CNN
	1    6375 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:L_Core_Ferrite_Small L1
U 1 1 5B99B94D
P 3725 3225
F 0 "L1" V 3930 3225 50  0000 C CNN
F 1 "100uH" V 3839 3225 50  0000 C CNN
F 2 "Inductor_THT:L_Radial_D7.8mm_P5.00mm_Fastron_07HCP" H 3725 3225 50  0001 C CNN
F 3 "~" H 3725 3225 50  0001 C CNN
	1    3725 3225
	0    -1   -1   0   
$EndComp
$Comp
L TE_ReedRelay_V23100V4:TE_ReedRelay_V23100V4xxxA010 U_RLY1
U 1 1 5B996138
P 8800 5475
F 0 "U_RLY1" H 8450 5400 60  0000 R CNN
F 1 "TE_ReedRelay_V23100V4xxxA010" H 8450 5575 60  0000 R CNN
F 2 "TE_Reed_Relay_V23100V4:TE_ReedRelay_V23100V4" H 8800 5455 60  0001 C CNN
F 3 "" H 8800 5455 60  0001 C CNN
	1    8800 5475
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J_DK1
U 1 1 5B9A08C2
P 9550 5350
F 0 "J_DK1" V 9425 5430 50  0000 L CNN
F 1 "Conn_01x01" V 9514 5430 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9550 5350 50  0001 C CNN
F 3 "~" H 9550 5350 50  0001 C CNN
	1    9550 5350
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J_DA1
U 1 1 5B9A3151
P 9550 5675
F 0 "J_DA1" V 9515 5588 50  0000 R CNN
F 1 "Conn_01x01" V 9426 5588 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9550 5675 50  0001 C CNN
F 3 "~" H 9550 5675 50  0001 C CNN
	1    9550 5675
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8750 5850 8850 5850
Connection ~ 8850 5850
Wire Wire Line
	8850 5850 8850 6125
Wire Wire Line
	8950 5850 8850 5850
NoConn ~ 8950 5175
NoConn ~ 8750 5175
NoConn ~ 8650 5775
Wire Wire Line
	8850 4800 8850 5175
Wire Wire Line
	8650 5175 8650 4800
Wire Wire Line
	8750 5775 8750 5850
Wire Wire Line
	8850 5775 8850 5850
Wire Wire Line
	8950 5775 8950 5850
$Comp
L Connector_Generic:Conn_01x12 J_IICMUX_SC7
U 1 1 5B999C08
P 10100 3500
F 0 "J_IICMUX_SC7" H 10020 4217 50  0000 C CNN
F 1 "Conn_01x12" H 10020 4126 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical" H 10100 3500 50  0001 C CNN
F 3 "~" H 10100 3500 50  0001 C CNN
	1    10100 3500
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x12 J_IICMUX_VIN1
U 1 1 5B9A71AD
P 8800 3500
F 0 "J_IICMUX_VIN1" H 8475 4225 50  0000 L CNN
F 1 "Conn_01x12" H 8525 4125 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical" H 8800 3500 50  0001 C CNN
F 3 "~" H 8800 3500 50  0001 C CNN
	1    8800 3500
	1    0    0    -1  
$EndComp
Text Notes 8900 4125 0    63   ~ 0
VIN\nGND\nSDA\nSCL\nRST\nA0\nA1\nA2\nSD0\nSC0\nSD1\nSC1
Text Notes 9800 4125 0    63   ~ 0
SC7\nSD7\nSC6\nSD6\nSC5\nSD5\nSC4\nSD4\nSC3\nSD3\nSC2\nSD2
Text Notes 9125 2925 0    63   ~ 0
TCA9548A\nPin Mappings:
Wire Wire Line
	8600 3000 8250 3000
Text Label 8250 3000 0    63   ~ 0
5V
Wire Wire Line
	8600 3200 8250 3200
Text Label 8250 3200 0    63   ~ 0
SDA
Wire Wire Line
	8600 3300 8250 3300
Text Label 8250 3300 0    63   ~ 0
SCL
Wire Wire Line
	8600 3400 8250 3400
Text Label 8250 3400 0    63   ~ 0
5V
Text Label 7850 4000 1    63   ~ 0
GND
Wire Wire Line
	7850 3500 7850 3600
Wire Wire Line
	7850 3500 8600 3500
Wire Wire Line
	8600 3100 7850 3100
Wire Wire Line
	7850 3100 7850 3500
Connection ~ 7850 3500
Wire Wire Line
	8600 3600 7850 3600
Connection ~ 7850 3600
Wire Wire Line
	7850 3600 7850 3700
Wire Wire Line
	8600 3700 7850 3700
Connection ~ 7850 3700
Wire Wire Line
	7850 3700 7850 4000
Wire Wire Line
	1625 6750 1375 6750
Text Label 1375 6750 0    60   ~ 0
SDA1
Wire Wire Line
	1625 6850 1375 6850
Text Label 1375 6850 0    60   ~ 0
SCL1
Wire Wire Line
	1625 6950 1375 6950
Text Label 1375 6950 0    60   ~ 0
GND
Wire Wire Line
	1625 7050 1375 7050
Text Label 1375 7050 0    60   ~ 0
5V
$Comp
L Connector_Generic:Conn_01x04 J_OLED2
U 1 1 5B9E0177
P 1825 6850
F 0 "J_OLED2" H 1905 6842 50  0000 L CNN
F 1 "Conn_01x04" H 1905 6751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1825 6850 50  0001 C CNN
F 3 "~" H 1825 6850 50  0001 C CNN
	1    1825 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 3800 8100 3800
Wire Wire Line
	8600 3900 8100 3900
Wire Wire Line
	8600 4000 8100 4000
Wire Wire Line
	8600 4100 8100 4100
Text Label 8100 3800 0    63   ~ 0
SDA0
Text Label 8100 3900 0    63   ~ 0
SCL0
Text Label 8100 4000 0    63   ~ 0
SDA1
Text Label 8100 4100 0    63   ~ 0
SCL1
NoConn ~ 10300 3000
NoConn ~ 10300 3100
NoConn ~ 10300 3200
NoConn ~ 10300 3300
NoConn ~ 10300 3400
NoConn ~ 10300 3500
NoConn ~ 10300 3600
NoConn ~ 10300 3700
NoConn ~ 10300 3800
NoConn ~ 10300 3900
NoConn ~ 10300 4000
NoConn ~ 10300 4100
$EndSCHEMATC
