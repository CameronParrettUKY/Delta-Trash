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
F 1 "10" H 9918 955 50  0000 L CNN
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
	7700 4900 7700 4550
Wire Wire Line
	7600 4900 7600 4550
Wire Wire Line
	7500 4900 7500 4550
Wire Wire Line
	7400 4900 7400 4550
Entry Wire Line
	7600 4450 7700 4550
Entry Wire Line
	7500 4450 7600 4550
Entry Wire Line
	7400 4450 7500 4550
Entry Wire Line
	7300 4450 7400 4550
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
	7200 4450 7300 4550
Entry Wire Line
	7100 4450 7200 4550
Wire Wire Line
	7300 4550 7300 4900
Wire Wire Line
	7200 4550 7200 4900
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
Text Label 7700 4900 1    50   ~ 0
R_1
Text Label 7600 4900 1    50   ~ 0
R_2
Text Label 7500 4900 1    50   ~ 0
R_3
Text Label 7400 4900 1    50   ~ 0
R_4
Text Label 7300 4900 1    50   ~ 0
GND
Text Label 7200 4900 1    50   ~ 0
5V
Wire Bus Line
	9350 2600 9750 2600
Entry Wire Line
	9750 850  9650 750 
Text Label 9700 800  3    50   ~ 0
5V
Wire Bus Line
	11550 4450 11550 4600
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
Entry Wire Line
	10750 4600 10650 4700
Entry Wire Line
	10650 4600 10550 4700
Entry Wire Line
	10550 4600 10450 4700
Entry Wire Line
	9550 4600 9450 4700
Entry Wire Line
	9450 4600 9350 4700
Entry Wire Line
	9350 4600 9250 4700
Entry Wire Line
	9250 4600 9150 4700
Entry Wire Line
	9150 4600 9050 4700
Entry Wire Line
	9050 4600 8950 4700
Entry Wire Line
	8950 4600 8850 4700
Entry Wire Line
	8850 4600 8750 4700
Wire Wire Line
	9450 4850 9450 4700
Wire Wire Line
	9350 4850 9350 4700
Wire Wire Line
	9250 4850 9250 4700
Wire Wire Line
	9150 4850 9150 4700
Wire Wire Line
	9050 4850 9050 4700
Wire Wire Line
	8950 4700 8950 4850
Wire Wire Line
	8850 4850 8850 4700
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
Wire Wire Line
	5900 2100 5900 2000
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
Entry Wire Line
	6600 1900 6500 2000
Entry Wire Line
	6600 2950 6500 3050
Wire Wire Line
	6500 3050 5900 3050
Connection ~ 5900 3050
Wire Wire Line
	5900 3050 5900 3250
Wire Wire Line
	6500 2000 5900 2000
Connection ~ 5900 2000
Wire Wire Line
	5900 2000 5900 1850
$Comp
L Device:R_US R7
U 1 1 65CEFBD4
P 6700 3500
F 0 "R7" H 6768 3546 50  0000 L CNN
F 1 "1.1" H 6768 3455 50  0000 L CNN
F 2 "" V 6740 3490 50  0001 C CNN
F 3 "~" H 6700 3500 50  0001 C CNN
	1    6700 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R4
U 1 1 65CFDF0F
P 5900 1700
F 0 "R4" H 5968 1746 50  0000 L CNN
F 1 "320000" H 5968 1655 50  0000 L CNN
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
F 1 "10000" H 5968 2205 50  0000 L CNN
F 2 "" V 5940 2240 50  0001 C CNN
F 3 "~" H 5900 2250 50  0001 C CNN
	1    5900 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D29F9C
P 6850 3500
F 0 "R?" H 6918 3546 50  0000 L CNN
F 1 "1.1" H 6918 3455 50  0000 L CNN
F 2 "" V 6890 3490 50  0001 C CNN
F 3 "~" H 6850 3500 50  0001 C CNN
	1    6850 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D334B2
P 6500 3500
F 0 "R?" H 6568 3546 50  0000 L CNN
F 1 "1.1" H 6568 3455 50  0000 L CNN
F 2 "" V 6540 3490 50  0001 C CNN
F 3 "~" H 6500 3500 50  0001 C CNN
	1    6500 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D361B8
P 7000 3500
F 0 "R?" H 7068 3546 50  0000 L CNN
F 1 "1.1" H 7068 3455 50  0000 L CNN
F 2 "" V 7040 3490 50  0001 C CNN
F 3 "~" H 7000 3500 50  0001 C CNN
	1    7000 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D38E0A
P 7350 3500
F 0 "R?" H 7418 3546 50  0000 L CNN
F 1 "1.1" H 7418 3455 50  0000 L CNN
F 2 "" V 7390 3490 50  0001 C CNN
F 3 "~" H 7350 3500 50  0001 C CNN
	1    7350 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R_US R?
U 1 1 65D3BA5D
P 7150 3500
F 0 "R?" H 7218 3546 50  0000 L CNN
F 1 "1.1" H 7218 3455 50  0000 L CNN
F 2 "" V 7190 3490 50  0001 C CNN
F 3 "~" H 7150 3500 50  0001 C CNN
	1    7150 3500
	-1   0    0    1   
$EndComp
Connection ~ 7150 3650
Wire Wire Line
	7150 3650 7000 3650
Connection ~ 7000 3650
Wire Wire Line
	7000 3650 6950 3650
Connection ~ 6850 3650
Wire Wire Line
	6850 3650 6700 3650
Connection ~ 6700 3650
Wire Wire Line
	6700 3650 6500 3650
Wire Wire Line
	6500 3350 6700 3350
Connection ~ 7150 3350
Wire Wire Line
	7150 3350 7350 3350
Connection ~ 7000 3350
Connection ~ 6850 3350
Wire Wire Line
	6850 3350 7000 3350
Connection ~ 6700 3350
Wire Wire Line
	6700 3350 6850 3350
$Comp
L Connector:Screw_Terminal_01x06 J?
U 1 1 662436F2
P 7500 5100
F 0 "J?" V 7372 5380 50  0000 L CNN
F 1 "Screw_Terminal_01x06" V 7463 5380 50  0000 L CNN
F 2 "" H 7500 5100 50  0001 C CNN
F 3 "~" H 7500 5100 50  0001 C CNN
	1    7500 5100
	0    1    1    0   
$EndComp
$Comp
L Connector:Screw_Terminal_01x08 J?
U 1 1 66245DBE
P 9150 5050
F 0 "J?" V 9022 5430 50  0000 L CNN
F 1 "Screw_Terminal_01x08" V 9113 5430 50  0000 L CNN
F 2 "" H 9150 5050 50  0001 C CNN
F 3 "~" H 9150 5050 50  0001 C CNN
	1    9150 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	8750 4700 8750 4850
$Comp
L Connector:Screw_Terminal_01x08 J?
U 1 1 6624DE09
P 10850 5050
F 0 "J?" V 10722 5430 50  0000 L CNN
F 1 "Screw_Terminal_01x08" V 10813 5430 50  0000 L CNN
F 2 "" H 10850 5050 50  0001 C CNN
F 3 "~" H 10850 5050 50  0001 C CNN
	1    10850 5050
	0    1    1    0   
$EndComp
Entry Wire Line
	12300 2600 12200 2700
Wire Wire Line
	12200 2700 12100 2700
Entry Wire Line
	12300 2000 12200 2100
Entry Wire Line
	12300 2200 12200 2300
Entry Wire Line
	12300 2300 12200 2400
Entry Wire Line
	12300 2100 12200 2200
Wire Wire Line
	12200 2100 12100 2100
Wire Wire Line
	12200 2200 12100 2200
Wire Wire Line
	12200 2300 12100 2300
Wire Wire Line
	12200 2400 12100 2400
Entry Wire Line
	12500 4600 12400 4700
Entry Wire Line
	12600 4600 12500 4700
Entry Wire Line
	12700 4600 12600 4700
Entry Wire Line
	12800 4600 12700 4700
Entry Wire Line
	12900 4600 12800 4700
Entry Wire Line
	13000 4600 12900 4700
Entry Wire Line
	13100 4600 13000 4700
Entry Wire Line
	13200 4600 13100 4700
$Comp
L Connector:Screw_Terminal_01x08 J?
U 1 1 6627B3B9
P 12800 5050
F 0 "J?" V 12672 5430 50  0000 L CNN
F 1 "Screw_Terminal_01x08" V 12763 5430 50  0000 L CNN
F 2 "" H 12800 5050 50  0001 C CNN
F 3 "~" H 12800 5050 50  0001 C CNN
	1    12800 5050
	0    1    1    0   
$EndComp
Entry Wire Line
	12300 1900 12200 2000
Wire Wire Line
	12200 2000 12100 2000
Wire Wire Line
	10450 4850 10450 4700
Wire Wire Line
	10550 4850 10550 4700
Wire Wire Line
	10650 4850 10650 4700
Wire Wire Line
	10750 4850 10750 4700
Wire Wire Line
	10850 4850 10850 4700
Wire Wire Line
	10950 4850 10950 4700
Wire Wire Line
	11050 4850 11050 4700
Wire Wire Line
	11150 4850 11150 4700
Wire Wire Line
	12400 4850 12400 4700
Wire Wire Line
	12500 4850 12500 4700
Wire Wire Line
	12600 4850 12600 4700
Wire Wire Line
	12700 4850 12700 4700
Wire Wire Line
	12800 4850 12800 4700
Wire Wire Line
	12900 4850 12900 4700
Wire Wire Line
	13000 4850 13000 4700
Wire Wire Line
	13100 4850 13100 4700
Wire Wire Line
	8350 3900 6950 3900
Wire Wire Line
	3750 3900 3750 3750
Connection ~ 8350 3900
Wire Wire Line
	5900 1100 5900 1550
Wire Wire Line
	5900 2400 5900 3050
Entry Wire Line
	6600 2950 6700 3050
Wire Wire Line
	6700 3050 6700 3350
Wire Wire Line
	5900 3250 5400 3250
Wire Wire Line
	5400 3250 5400 3900
Connection ~ 5900 3250
Wire Wire Line
	5900 3250 5900 3350
Connection ~ 5400 3900
Wire Wire Line
	5400 3900 3750 3900
Wire Wire Line
	6950 3650 6950 3900
Connection ~ 6950 3650
Wire Wire Line
	6950 3650 6850 3650
Connection ~ 6950 3900
Wire Wire Line
	6950 3900 5400 3900
$Comp
L Connector:Barrel_Jack J?
U 1 1 6640DFE2
P 7600 3950
F 0 "J?" V 7703 3770 50  0000 R CNN
F 1 "Barrel_Jack" V 7612 3770 50  0000 R CNN
F 2 "" H 7650 3910 50  0001 C CNN
F 3 "~" H 7650 3910 50  0001 C CNN
	1    7600 3950
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Barrel_Jack J?
U 1 1 664186CF
P 7250 1600
F 0 "J?" V 7353 1420 50  0000 R CNN
F 1 "Barrel_Jack" V 7262 1420 50  0000 R CNN
F 2 "" H 7300 1560 50  0001 C CNN
F 3 "~" H 7300 1560 50  0001 C CNN
	1    7250 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 3650 7350 3650
Connection ~ 7350 3650
Wire Wire Line
	7350 3650 7500 3650
Wire Wire Line
	7700 3650 7550 3650
Wire Wire Line
	7500 1900 7350 1900
Wire Wire Line
	7000 3350 7150 3350
Wire Wire Line
	7150 1900 7150 2200
Wire Wire Line
	7150 2200 7550 2200
Wire Wire Line
	7550 2200 7550 3650
Wire Wire Line
	7500 1900 7500 3350
Wire Wire Line
	7500 3350 7350 3350
Connection ~ 7350 3350
Connection ~ 11550 4600
Wire Bus Line
	9750 2600 9750 850 
Entry Wire Line
	4800 2450 4700 2350
Entry Wire Line
	4800 2350 4700 2250
Entry Wire Line
	2800 2750 2900 2650
Entry Wire Line
	2800 3050 2900 2950
Entry Wire Line
	2800 3250 2900 3150
Entry Wire Line
	2800 2050 2900 1950
Entry Wire Line
	2800 2950 2900 2850
Entry Wire Line
	2800 2850 2900 2750
Wire Wire Line
	2900 1950 3050 1950
Wire Wire Line
	2900 2650 3050 2650
Wire Wire Line
	2900 2750 3050 2750
Wire Wire Line
	2900 2850 3050 2850
Wire Wire Line
	2900 2950 3050 2950
Wire Wire Line
	2900 3150 3050 3150
Wire Wire Line
	4700 2350 4650 2350
Wire Wire Line
	4700 2250 4650 2250
Entry Wire Line
	3150 4350 3250 4450
Entry Wire Line
	3250 4350 3350 4450
Entry Wire Line
	3350 4350 3450 4450
Entry Wire Line
	3450 4350 3550 4450
Entry Wire Line
	3550 4350 3650 4450
Entry Wire Line
	3650 4350 3750 4450
Entry Wire Line
	3750 4350 3850 4450
Entry Wire Line
	3850 4350 3950 4450
$Comp
L Connector:Screw_Terminal_01x08 J?
U 1 1 6653487B
P 3650 4750
F 0 "J?" V 3522 5130 50  0000 L CNN
F 1 "Screw_Terminal_01x08" V 3613 5130 50  0000 L CNN
F 2 "" H 3650 4750 50  0001 C CNN
F 3 "~" H 3650 4750 50  0001 C CNN
	1    3650 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 4550 3250 4450
Wire Wire Line
	3350 4550 3350 4450
Wire Wire Line
	3450 4550 3450 4450
Wire Wire Line
	3550 4550 3550 4450
Wire Wire Line
	3650 4550 3650 4450
Wire Wire Line
	3750 4550 3750 4450
Wire Wire Line
	3850 4550 3850 4450
Wire Wire Line
	3950 4550 3950 4450
Entry Wire Line
	2800 1550 2900 1650
Entry Wire Line
	2800 1450 2900 1550
Wire Wire Line
	2900 1550 3050 1550
Wire Wire Line
	2900 1650 3050 1650
Entry Wire Line
	10550 1500 10650 1600
Entry Wire Line
	10550 1400 10650 1500
Wire Wire Line
	10700 1600 10650 1600
Wire Wire Line
	10700 1500 10650 1500
Wire Bus Line
	2800 650  10550 650 
Entry Wire Line
	7500 1500 7600 1600
Entry Wire Line
	7500 1400 7600 1500
Wire Wire Line
	7650 1600 7600 1600
Wire Wire Line
	7650 1500 7600 1500
Wire Bus Line
	7500 900  10500 900 
Entry Wire Line
	10500 1700 10600 1800
Entry Wire Line
	10500 1600 10600 1700
Wire Wire Line
	10600 1800 10700 1800
Wire Wire Line
	10700 1700 10600 1700
$Comp
L Connector:Screw_Terminal_01x04 J?
U 1 1 666791B8
P 14000 4150
F 0 "J?" V 13872 4330 50  0000 L CNN
F 1 "Screw_Terminal_01x04" V 13963 4330 50  0000 L CNN
F 2 "" H 14000 4150 50  0001 C CNN
F 3 "~" H 14000 4150 50  0001 C CNN
	1    14000 4150
	0    -1   -1   0   
$EndComp
Entry Wire Line
	13800 4550 13900 4450
Entry Wire Line
	13900 4550 14000 4450
Entry Wire Line
	14000 4550 14100 4450
Wire Wire Line
	13900 4450 13900 4350
Wire Wire Line
	14000 4450 14000 4350
Wire Wire Line
	14100 4450 14100 4350
Wire Wire Line
	14200 4450 14200 4350
Entry Wire Line
	14100 4550 14200 4450
Entry Wire Line
	10350 2000 10450 1900
Entry Wire Line
	10350 2100 10450 2000
Entry Wire Line
	10350 2300 10450 2200
Wire Wire Line
	10450 2200 10700 2200
Wire Wire Line
	10700 2000 10450 2000
Wire Wire Line
	10700 1900 10450 1900
Entry Wire Line
	10350 2000 10250 1900
Wire Wire Line
	10250 1900 9850 1900
Wire Bus Line
	10500 900  10500 1700
Wire Bus Line
	7500 900  7500 1500
Wire Bus Line
	10550 650  10550 1500
Wire Bus Line
	2800 650  2800 1550
Wire Bus Line
	4800 2350 4800 4350
Wire Bus Line
	6600 1050 6600 2950
Wire Wire Line
	9250 750  9850 750 
Wire Bus Line
	9250 1950 9250 2200
Wire Bus Line
	10350 2000 10350 4550
Wire Bus Line
	9350 2600 9350 4450
Wire Bus Line
	10350 4550 14100 4550
Wire Bus Line
	7100 4450 9350 4450
Wire Bus Line
	2800 2050 2800 4350
Wire Bus Line
	2800 4350 4800 4350
Wire Bus Line
	11550 4600 13200 4600
Wire Bus Line
	10500 2400 10500 4450
Wire Bus Line
	12300 1900 12300 4450
Wire Bus Line
	8850 4600 11550 4600
$EndSCHEMATC
