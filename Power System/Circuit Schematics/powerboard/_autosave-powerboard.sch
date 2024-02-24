EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr C 22000 17000
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
L Connector:Raspberry_Pi_2_3 J1
U 1 1 65A83222
P 3850 2450
F 0 "J1" H 3850 3931 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 3850 3840 50  0000 C CNN
F 2 "" H 3850 2450 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 3850 2450 50  0001 C CNN
	1    3850 2450
	1    0    0    -1  
$EndComp
$Comp
L Raspi_Pico:Pico U1
U 1 1 65A86452
P 8350 2450
F 0 "U1" H 8350 3665 50  0000 C CNN
F 1 "Pico" H 8350 3574 50  0000 C CNN
F 2 "RPi_Pico:RPi_Pico_SMD_TH" V 8350 2450 50  0001 C CNN
F 3 "" H 8350 2450 50  0001 C CNN
	1    8350 2450
	1    0    0    -1  
$EndComp
$Comp
L Raspi_Pico:Pico U2
U 1 1 65A87254
P 11400 2450
F 0 "U2" H 11400 3665 50  0000 C CNN
F 1 "Pico" H 11400 3574 50  0000 C CNN
F 2 "RPi_Pico:RPi_Pico_SMD_TH" V 11400 2450 50  0001 C CNN
F 3 "" H 11400 2450 50  0001 C CNN
	1    11400 2450
	1    0    0    -1  
$EndComp
$Comp
L Connector:RJ45_LED J2
U 1 1 65A8D298
P 5250 5300
F 0 "J2" V 5296 4870 50  0000 R CNN
F 1 "RJ45_LED" V 5205 4870 50  0000 R CNN
F 2 "" V 5250 5325 50  0001 C CNN
F 3 "~" V 5250 5325 50  0001 C CNN
	1    5250 5300
	0    -1   -1   0   
$EndComp
$Comp
L Connector:RJ45_LED J3
U 1 1 65A8DE98
P 11150 5250
F 0 "J3" V 11196 4820 50  0000 R CNN
F 1 "RJ45_LED" V 11105 4820 50  0000 R CNN
F 2 "" V 11150 5275 50  0001 C CNN
F 3 "~" V 11150 5275 50  0001 C CNN
	1    11150 5250
	0    -1   -1   0   
$EndComp
$Comp
L Connector:RJ45_LED J4
U 1 1 65A8F8C7
P 8350 5300
F 0 "J4" V 8396 4870 50  0000 R CNN
F 1 "RJ45_LED" V 8305 4870 50  0000 R CNN
F 2 "" V 8350 5325 50  0001 C CNN
F 3 "~" V 8350 5325 50  0001 C CNN
	1    8350 5300
	0    -1   -1   0   
$EndComp
$Comp
L Connector:RJ45_LED J5
U 1 1 65A90045
P 9800 5250
F 0 "J5" V 9846 4820 50  0000 R CNN
F 1 "RJ45_LED" V 9755 4820 50  0000 R CNN
F 2 "" V 9800 5275 50  0001 C CNN
F 3 "~" V 9800 5275 50  0001 C CNN
	1    9800 5250
	0    -1   -1   0   
$EndComp
$Comp
L Device:Battery BT1
U 1 1 65A9782C
P 9850 2150
F 0 "BT1" H 9958 2196 50  0000 L CNN
F 1 "Battery" H 9958 2105 50  0000 L CNN
F 2 "" V 9850 2210 50  0001 C CNN
F 3 "~" V 9850 2210 50  0001 C CNN
	1    9850 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1950 9850 1900
$Comp
L Diode:1N4007 D2
U 1 1 65AFDDCE
P 9450 1900
F 0 "D2" H 9450 2117 50  0000 C CNN
F 1 "1N4007" H 9450 2026 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 9450 1725 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 9450 1900 50  0001 C CNN
	1    9450 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1900 9600 1900
$Comp
L power:+5V #PWR01
U 1 1 65AFEECF
P 8300 750
F 0 "#PWR01" H 8300 600 50  0001 C CNN
F 1 "+5V" H 8315 923 50  0000 C CNN
F 2 "" H 8300 750 50  0001 C CNN
F 3 "" H 8300 750 50  0001 C CNN
	1    8300 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 750  9250 1200
Wire Wire Line
	9250 1900 9300 1900
Connection ~ 9250 750 
Wire Wire Line
	8300 750  9250 750 
$Comp
L Diode:1N4007 D1
U 1 1 65B0B341
P 9250 1350
F 0 "D1" V 9296 1270 50  0000 R CNN
F 1 "1N4007" V 9205 1270 50  0000 R CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 9250 1175 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 9250 1350 50  0001 C CNN
	1    9250 1350
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 65B0D386
P 9850 3950
F 0 "#PWR02" H 9850 3700 50  0001 C CNN
F 1 "GND" H 9855 3777 50  0000 C CNN
F 2 "" H 9850 3950 50  0001 C CNN
F 3 "" H 9850 3950 50  0001 C CNN
	1    9850 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 3950 9850 3900
Wire Wire Line
	9850 3900 11400 3900
Wire Wire Line
	11400 3900 11400 3600
Connection ~ 9850 3900
Wire Wire Line
	9850 2350 9850 3900
Wire Wire Line
	9850 3900 8350 3900
Wire Wire Line
	8350 3900 8350 3600
$Comp
L Diode:1N4007 D3
U 1 1 65B267E4
P 9850 1350
F 0 "D3" V 9896 1270 50  0000 R CNN
F 1 "1N4007" V 9805 1270 50  0000 R CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 9850 1175 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 9850 1350 50  0001 C CNN
	1    9850 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9850 1500 9850 1900
Connection ~ 9850 1900
$Comp
L Device:R_Small_US R1
U 1 1 65B27800
P 9850 1000
F 0 "R1" H 9918 1046 50  0000 L CNN
F 1 "R_Small_US" H 9918 955 50  0000 L CNN
F 2 "" H 9850 1000 50  0001 C CNN
F 3 "~" H 9850 1000 50  0001 C CNN
	1    9850 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1200 9850 1100
Wire Wire Line
	9850 900  9850 750 
Wire Wire Line
	8300 750  3650 750 
Wire Wire Line
	3650 750  3650 1150
Connection ~ 8300 750 
Wire Wire Line
	8650 4900 8650 4550
Wire Wire Line
	8550 4900 8550 4550
Wire Wire Line
	8450 4900 8450 4550
Wire Wire Line
	8350 4900 8350 4550
Entry Wire Line
	8550 4450 8650 4550
Entry Wire Line
	8450 4450 8550 4550
Entry Wire Line
	8350 4450 8450 4550
Entry Wire Line
	8250 4450 8350 4550
Entry Wire Line
	9350 2700 9250 2800
Entry Wire Line
	9350 2800 9250 2900
Entry Wire Line
	9350 2900 9250 3000
Entry Wire Line
	9350 3000 9250 3100
Wire Wire Line
	9050 2800 9250 2800
Wire Wire Line
	9050 2900 9250 2900
Wire Wire Line
	9050 3000 9250 3000
Wire Wire Line
	9050 3100 9250 3100
Wire Wire Line
	9050 1600 9150 1600
Wire Wire Line
	9250 1500 9250 1600
Connection ~ 9250 1600
Wire Wire Line
	9250 1600 9250 1900
Wire Wire Line
	9050 2700 9250 2700
Entry Wire Line
	9350 2600 9250 2700
Entry Wire Line
	8150 4450 8250 4550
Entry Wire Line
	8050 4450 8150 4550
Wire Wire Line
	8250 4550 8250 4900
Wire Wire Line
	8150 4550 8150 4900
Wire Wire Line
	12350 1600 12100 1600
Text Label 9100 2700 0    50   ~ 0
GND
Text Label 9100 2800 0    50   ~ 0
R_1
Text Label 9100 2900 0    50   ~ 0
R_2
Text Label 9100 3000 0    50   ~ 0
R_3
Text Label 9100 3100 0    50   ~ 0
R_4
Text Label 8650 4900 1    50   ~ 0
R_1
Text Label 8550 4900 1    50   ~ 0
R_2
Text Label 8450 4900 1    50   ~ 0
R_3
Text Label 8350 4900 1    50   ~ 0
R_4
Text Label 8250 4900 1    50   ~ 0
GND
Text Label 8150 4900 1    50   ~ 0
5V
Wire Wire Line
	7950 5700 7750 5700
Wire Wire Line
	7750 5700 7750 4900
Wire Wire Line
	7750 4900 7950 4900
$Comp
L Device:R_Small_US R2
U 1 1 65BDF579
P 8050 5900
F 0 "R2" H 8118 5946 50  0000 L CNN
F 1 "R_Small_US" H 8118 5855 50  0000 L CNN
F 2 "" H 8050 5900 50  0001 C CNN
F 3 "~" H 8050 5900 50  0001 C CNN
	1    8050 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 5700 8050 5800
$Comp
L power:GND #PWR03
U 1 1 65BE1014
P 8050 6150
F 0 "#PWR03" H 8050 5900 50  0001 C CNN
F 1 "GND" H 8055 5977 50  0000 C CNN
F 2 "" H 8050 6150 50  0001 C CNN
F 3 "" H 8050 6150 50  0001 C CNN
	1    8050 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 6000 8050 6150
Text Label 7950 4900 1    50   ~ 0
Loop_Verify
Wire Bus Line
	9350 2600 9750 2600
Wire Bus Line
	9750 2600 9750 850 
Entry Wire Line
	9750 850  9650 750 
Text Label 9700 800  3    50   ~ 0
5V
Wire Bus Line
	11550 4450 11550 4600
Entry Wire Line
	11550 4600 11450 4700
Entry Wire Line
	11450 4600 11350 4700
Entry Wire Line
	11350 4600 11250 4700
Entry Wire Line
	11250 4600 11150 4700
Entry Wire Line
	11150 4600 11050 4700
Entry Wire Line
	11050 4600 10950 4700
Entry Wire Line
	10950 4600 10850 4700
Entry Wire Line
	10850 4600 10750 4700
Wire Wire Line
	11450 4850 11450 4700
Wire Wire Line
	11350 4850 11350 4700
Wire Wire Line
	11250 4850 11250 4700
Wire Wire Line
	11150 4850 11150 4700
Wire Wire Line
	11050 4850 11050 4700
Wire Wire Line
	10950 4850 10950 4700
Wire Wire Line
	10850 4850 10850 4700
Wire Wire Line
	10750 4850 10750 4700
Entry Wire Line
	10200 4600 10100 4700
Entry Wire Line
	10100 4600 10000 4700
Entry Wire Line
	10000 4600 9900 4700
Entry Wire Line
	9900 4600 9800 4700
Entry Wire Line
	9800 4600 9700 4700
Entry Wire Line
	9700 4600 9600 4700
Entry Wire Line
	9600 4600 9500 4700
Entry Wire Line
	9500 4600 9400 4700
Wire Wire Line
	10100 4850 10100 4700
Wire Wire Line
	10000 4850 10000 4700
Wire Wire Line
	9900 4850 9900 4700
Wire Wire Line
	9800 4850 9800 4700
Wire Wire Line
	9700 4850 9700 4700
Wire Wire Line
	9600 4700 9600 4850
Wire Wire Line
	9500 4850 9500 4700
Wire Bus Line
	9250 1950 9100 1950
Wire Bus Line
	9100 1950 9100 1050
Wire Bus Line
	9100 1050 6600 1050
Entry Wire Line
	9250 2000 9150 2100
Entry Wire Line
	9250 2100 9150 2200
Wire Wire Line
	9050 2100 9150 2100
Wire Wire Line
	9150 2200 9050 2200
Wire Wire Line
	9050 2300 9150 2300
Entry Wire Line
	9150 2300 9250 2200
Wire Wire Line
	9150 1600 9150 700 
Wire Wire Line
	9150 700  12350 700 
Wire Wire Line
	12350 700  12350 1600
Connection ~ 9150 1600
Wire Wire Line
	9150 1600 9250 1600
Wire Bus Line
	11550 4450 12300 4450
Wire Bus Line
	11550 4450 10500 4450
Connection ~ 11550 4450
Entry Wire Line
	12300 2500 12200 2600
Entry Wire Line
	12300 2700 12200 2800
Entry Wire Line
	12300 2800 12200 2900
Entry Wire Line
	12300 2900 12200 3000
Entry Wire Line
	12300 3000 12200 3100
Entry Wire Line
	12300 3100 12200 3200
Entry Wire Line
	12300 3200 12200 3300
Entry Wire Line
	12300 3300 12200 3400
Entry Wire Line
	10500 2400 10600 2500
Entry Wire Line
	10500 2500 10600 2600
Entry Wire Line
	10500 2600 10600 2700
Entry Wire Line
	10500 2700 10600 2800
Entry Wire Line
	10500 2800 10600 2900
Entry Wire Line
	10500 2900 10600 3000
Entry Wire Line
	10500 3000 10600 3100
Entry Wire Line
	10500 3100 10600 3200
Entry Wire Line
	10500 3200 10600 3300
Entry Wire Line
	10500 3300 10600 3400
Wire Wire Line
	10600 2500 10700 2500
Wire Wire Line
	10700 2600 10600 2600
Wire Wire Line
	10600 2700 10700 2700
Wire Wire Line
	10700 2800 10600 2800
Wire Wire Line
	10600 2900 10700 2900
Wire Wire Line
	10700 3000 10600 3000
Wire Wire Line
	10600 3100 10700 3100
Wire Wire Line
	10700 3200 10600 3200
Wire Wire Line
	10600 3300 10700 3300
Wire Wire Line
	10700 3400 10600 3400
Wire Wire Line
	12200 3400 12100 3400
Wire Wire Line
	12100 3300 12200 3300
Wire Wire Line
	12200 3200 12100 3200
Wire Wire Line
	12100 3100 12200 3100
Wire Wire Line
	12200 3000 12100 3000
Wire Wire Line
	12100 2900 12200 2900
Wire Wire Line
	12200 2800 12100 2800
Wire Wire Line
	12100 2600 12200 2600
$Comp
L imported:TB002-500-02BE J6
U 1 1 65C1851A
P 2500 10050
F 0 "J6" H 2792 9685 50  0000 C CNN
F 1 "TB002-500-02BE" H 2792 9776 50  0000 C CNN
F 2 "TB00250002BE" H 3150 10150 50  0001 L CNN
F 3 "" H 3150 10050 50  0001 L CNN
F 4 "Fixed Terminal Blocks Terminal block, screw type, 5.00, horizontal, 2 poles, CUI Blue, slotted screw, PCB mount" H 3150 9950 50  0001 L CNN "Description"
F 5 "10.4" H 3150 9850 50  0001 L CNN "Height"
F 6 "490-TB002-500-02BE" H 3150 9750 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/CUI-Devices/TB002-500-02BE?qs=vLWxofP3U2x9716kcgva%2Fw%3D%3D" H 3150 9650 50  0001 L CNN "Mouser Price/Stock"
F 8 "CUI Devices" H 3150 9550 50  0001 L CNN "Manufacturer_Name"
F 9 "TB002-500-02BE" H 3150 9450 50  0001 L CNN "Manufacturer_Part_Number"
	1    2500 10050
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R3
U 1 1 65CD07C8
P 5900 1250
F 0 "R3" H 5968 1296 50  0000 L CNN
F 1 "68000" H 5968 1205 50  0000 L CNN
F 2 "" V 5940 1240 50  0001 C CNN
F 3 "~" H 5900 1250 50  0001 C CNN
	1    5900 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2550 5900 2400
Wire Wire Line
	5900 2100 5900 2000
Wire Wire Line
	5900 1550 5900 1400
$Comp
L power:+BATT #PWR04
U 1 1 65CDB0B0
P 5250 1000
F 0 "#PWR04" H 5250 850 50  0001 C CNN
F 1 "+BATT" H 5265 1173 50  0000 C CNN
F 2 "" H 5250 1000 50  0001 C CNN
F 3 "" H 5250 1000 50  0001 C CNN
	1    5250 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 1100 5250 1100
Wire Wire Line
	5250 1100 5250 1000
$Comp
L power:GND #PWR05
U 1 1 65CDFBD8
P 5900 3350
F 0 "#PWR05" H 5900 3100 50  0001 C CNN
F 1 "GND" H 5905 3177 50  0000 C CNN
F 2 "" H 5900 3350 50  0001 C CNN
F 3 "" H 5900 3350 50  0001 C CNN
	1    5900 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2850 5900 3050
Entry Wire Line
	6600 1900 6500 2000
Entry Wire Line
	6600 2950 6500 3050
Wire Wire Line
	6500 3050 5900 3050
Connection ~ 5900 3050
Wire Wire Line
	5900 3050 5900 3350
Wire Wire Line
	6500 2000 5900 2000
Connection ~ 5900 2000
Wire Wire Line
	5900 2000 5900 1850
$Comp
L Device:R_US R7
U 1 1 65CEFBD4
P 7150 1500
F 0 "R7" H 7218 1546 50  0000 L CNN
F 1 "1.1" H 7218 1455 50  0000 L CNN
F 2 "" V 7190 1490 50  0001 C CNN
F 3 "~" H 7150 1500 50  0001 C CNN
	1    7150 1500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R4
U 1 1 65CFDF0F
P 5900 1700
F 0 "R4" H 5968 1746 50  0000 L CNN
F 1 "68000" H 5968 1655 50  0000 L CNN
F 2 "" V 5940 1690 50  0001 C CNN
F 3 "~" H 5900 1700 50  0001 C CNN
	1    5900 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R5
U 1 1 65D0C02D
P 5900 2250
F 0 "R5" H 5968 2296 50  0000 L CNN
F 1 "68000" H 5968 2205 50  0000 L CNN
F 2 "" V 5940 2240 50  0001 C CNN
F 3 "~" H 5900 2250 50  0001 C CNN
	1    5900 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R6
U 1 1 65D14866
P 5900 2700
F 0 "R6" H 5968 2746 50  0000 L CNN
F 1 "68000" H 5968 2655 50  0000 L CNN
F 2 "" V 5940 2690 50  0001 C CNN
F 3 "~" H 5900 2700 50  0001 C CNN
	1    5900 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D29F9C
P 7150 1650
F 0 "R?" H 7218 1696 50  0000 L CNN
F 1 "1.1" H 7218 1605 50  0000 L CNN
F 2 "" V 7190 1640 50  0001 C CNN
F 3 "~" H 7150 1650 50  0001 C CNN
	1    7150 1650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D334B2
P 7150 1300
F 0 "R?" H 7218 1346 50  0000 L CNN
F 1 "1.1" H 7218 1255 50  0000 L CNN
F 2 "" V 7190 1290 50  0001 C CNN
F 3 "~" H 7150 1300 50  0001 C CNN
	1    7150 1300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D361B8
P 7150 1800
F 0 "R?" H 7218 1846 50  0000 L CNN
F 1 "1.1" H 7218 1755 50  0000 L CNN
F 2 "" V 7190 1790 50  0001 C CNN
F 3 "~" H 7150 1800 50  0001 C CNN
	1    7150 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D38E0A
P 7150 2150
F 0 "R?" H 7218 2196 50  0000 L CNN
F 1 "1.1" H 7218 2105 50  0000 L CNN
F 2 "" V 7190 2140 50  0001 C CNN
F 3 "~" H 7150 2150 50  0001 C CNN
	1    7150 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D3BA5D
P 7150 1950
F 0 "R?" H 7218 1996 50  0000 L CNN
F 1 "1.1" H 7218 1905 50  0000 L CNN
F 2 "" V 7190 1940 50  0001 C CNN
F 3 "~" H 7150 1950 50  0001 C CNN
	1    7150 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7000 2150 7000 1950
Connection ~ 7000 1950
Wire Wire Line
	7000 1950 7000 1800
Connection ~ 7000 1800
Wire Wire Line
	7000 1800 7000 1650
Connection ~ 7000 1650
Wire Wire Line
	7000 1650 7000 1500
Connection ~ 7000 1500
Wire Wire Line
	7000 1500 7000 1300
Wire Wire Line
	7300 1300 7300 1500
Connection ~ 7300 1950
Wire Wire Line
	7300 1950 7300 2150
Connection ~ 7300 1800
Wire Wire Line
	7300 1800 7300 1950
Connection ~ 7300 1650
Wire Wire Line
	7300 1650 7300 1800
Connection ~ 7300 1500
Wire Wire Line
	7300 1500 7300 1650
Wire Bus Line
	6600 1050 6600 2950
Wire Wire Line
	9250 750  9850 750 
Wire Bus Line
	9250 1950 9250 2200
Wire Bus Line
	9350 2600 9350 4450
Wire Bus Line
	8050 4450 9350 4450
Wire Bus Line
	10500 2400 10500 4450
Wire Bus Line
	12300 2500 12300 4450
Wire Bus Line
	9400 4600 11550 4600
$EndSCHEMATC
