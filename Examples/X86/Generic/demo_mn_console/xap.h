/* This file was autogenerated by openCONFIGURATOR-1.2.2 on 25-Jan-2013 17:35:09 */
#ifndef XAP_h
#define XAP_h

# define COMPUTED_PI_OUT_SIZE 80
typedef struct 
{
	unsigned CN1_M00_X20PS9402_NetworkStatus:8;
	unsigned CN1_M00_X20PS9402_StatusInput01:1;
	unsigned CN1_M00_X20PS9402_Bit_Unused_01:1;
	unsigned CN1_M00_X20PS9402_StatusInput02:1;
	unsigned CN1_M00_X20PS9402_Bit_Unused_02:5;
	unsigned CN1_M01_X20DI9371_NetworkStatus:8;
	unsigned CN1_M01_X20DI9371_DigitalInput01:1;
	unsigned CN1_M01_X20DI9371_DigitalInput02:1;
	unsigned CN1_M01_X20DI9371_DigitalInput03:1;
	unsigned CN1_M01_X20DI9371_DigitalInput04:1;
	unsigned CN1_M01_X20DI9371_DigitalInput05:1;
	unsigned CN1_M01_X20DI9371_DigitalInput06:1;
	unsigned CN1_M01_X20DI9371_DigitalInput07:1;
	unsigned CN1_M01_X20DI9371_DigitalInput08:1;
	unsigned CN1_M01_X20DI9371_DigitalInput09:1;
	unsigned CN1_M01_X20DI9371_DigitalInput10:1;
	unsigned CN1_M01_X20DI9371_DigitalInput11:1;
	unsigned CN1_M01_X20DI9371_DigitalInput12:1;
	unsigned CN1_M01_X20DI9371_Bit_Unused_01:4;
	unsigned CN1_M02_X20DO9322_NetworkStatus:8;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN1_M02_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN1_M02_X20DO9322_Bit_Unused_01:4;
	unsigned CN2_M00_X20PS9402_NetworkStatus:8;
	unsigned CN2_M00_X20PS9402_StatusInput01:1;
	unsigned CN2_M00_X20PS9402_Bit_Unused_01:1;
	unsigned CN2_M00_X20PS9402_StatusInput02:1;
	unsigned CN2_M00_X20PS9402_Bit_Unused_02:5;
	unsigned CN2_M01_X20DI9371_NetworkStatus:8;
	unsigned CN2_M01_X20DI9371_DigitalInput01:1;
	unsigned CN2_M01_X20DI9371_DigitalInput02:1;
	unsigned CN2_M01_X20DI9371_DigitalInput03:1;
	unsigned CN2_M01_X20DI9371_DigitalInput04:1;
	unsigned CN2_M01_X20DI9371_DigitalInput05:1;
	unsigned CN2_M01_X20DI9371_DigitalInput06:1;
	unsigned CN2_M01_X20DI9371_DigitalInput07:1;
	unsigned CN2_M01_X20DI9371_DigitalInput08:1;
	unsigned CN2_M01_X20DI9371_DigitalInput09:1;
	unsigned CN2_M01_X20DI9371_DigitalInput10:1;
	unsigned CN2_M01_X20DI9371_DigitalInput11:1;
	unsigned CN2_M01_X20DI9371_DigitalInput12:1;
	unsigned CN2_M01_X20DI9371_Bit_Unused_01:4;
	unsigned CN2_M02_X20DO9322_NetworkStatus:8;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN2_M02_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN2_M02_X20DO9322_Bit_Unused_01:4;
	unsigned CN2_M03_X20AI4622_NetworkStatus:8;
	unsigned PADDING_VAR_1:8;
	unsigned CN2_M03_X20AI4622_AnalogInput01:16;
	unsigned CN2_M03_X20AI4622_AnalogInput02:16;
	unsigned CN2_M03_X20AI4622_AnalogInput03:16;
	unsigned CN2_M03_X20AI4622_AnalogInput04:16;
	unsigned CN2_M04_X20AO4622_NetworkStatus:8;
	unsigned CN3_M00_X20PS9402_NetworkStatus:8;
	unsigned CN3_M00_X20PS9402_StatusInput01:1;
	unsigned CN3_M00_X20PS9402_Bit_Unused_01:1;
	unsigned CN3_M00_X20PS9402_StatusInput02:1;
	unsigned CN3_M00_X20PS9402_Bit_Unused_02:5;
	unsigned CN3_M01_X20DI9371_NetworkStatus:8;
	unsigned CN3_M01_X20DI9371_DigitalInput01:1;
	unsigned CN3_M01_X20DI9371_DigitalInput02:1;
	unsigned CN3_M01_X20DI9371_DigitalInput03:1;
	unsigned CN3_M01_X20DI9371_DigitalInput04:1;
	unsigned CN3_M01_X20DI9371_DigitalInput05:1;
	unsigned CN3_M01_X20DI9371_DigitalInput06:1;
	unsigned CN3_M01_X20DI9371_DigitalInput07:1;
	unsigned CN3_M01_X20DI9371_DigitalInput08:1;
	unsigned CN3_M01_X20DI9371_DigitalInput09:1;
	unsigned CN3_M01_X20DI9371_DigitalInput10:1;
	unsigned CN3_M01_X20DI9371_DigitalInput11:1;
	unsigned CN3_M01_X20DI9371_DigitalInput12:1;
	unsigned CN3_M01_X20DI9371_Bit_Unused_01:4;
	unsigned CN3_M02_X20DI9371_NetworkStatus:8;
	unsigned CN3_M02_X20DI9371_DigitalInput01:1;
	unsigned CN3_M02_X20DI9371_DigitalInput02:1;
	unsigned CN3_M02_X20DI9371_DigitalInput03:1;
	unsigned CN3_M02_X20DI9371_DigitalInput04:1;
	unsigned CN3_M02_X20DI9371_DigitalInput05:1;
	unsigned CN3_M02_X20DI9371_DigitalInput06:1;
	unsigned CN3_M02_X20DI9371_DigitalInput07:1;
	unsigned CN3_M02_X20DI9371_DigitalInput08:1;
	unsigned CN3_M02_X20DI9371_DigitalInput09:1;
	unsigned CN3_M02_X20DI9371_DigitalInput10:1;
	unsigned CN3_M02_X20DI9371_DigitalInput11:1;
	unsigned CN3_M02_X20DI9371_DigitalInput12:1;
	unsigned CN3_M02_X20DI9371_Bit_Unused_01:4;
	unsigned CN3_M03_X20DI9371_NetworkStatus:8;
	unsigned CN3_M03_X20DI9371_DigitalInput01:1;
	unsigned CN3_M03_X20DI9371_DigitalInput02:1;
	unsigned CN3_M03_X20DI9371_DigitalInput03:1;
	unsigned CN3_M03_X20DI9371_DigitalInput04:1;
	unsigned CN3_M03_X20DI9371_DigitalInput05:1;
	unsigned CN3_M03_X20DI9371_DigitalInput06:1;
	unsigned CN3_M03_X20DI9371_DigitalInput07:1;
	unsigned CN3_M03_X20DI9371_DigitalInput08:1;
	unsigned CN3_M03_X20DI9371_DigitalInput09:1;
	unsigned CN3_M03_X20DI9371_DigitalInput10:1;
	unsigned CN3_M03_X20DI9371_DigitalInput11:1;
	unsigned CN3_M03_X20DI9371_DigitalInput12:1;
	unsigned CN3_M03_X20DI9371_Bit_Unused_01:4;
	unsigned CN3_M04_X20DO9322_NetworkStatus:8;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN3_M04_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN3_M04_X20DO9322_Bit_Unused_01:4;
	unsigned CN3_M05_X20DO9322_NetworkStatus:8;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN3_M05_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN3_M05_X20DO9322_Bit_Unused_01:4;
	unsigned CN3_M06_X20DO9322_NetworkStatus:8;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN3_M06_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN3_M06_X20DO9322_Bit_Unused_01:4;
	unsigned CN4_M00_X20PS9400a_NetworkStatus:8;
	unsigned CN4_M00_X20PS9400a_StatusInput01:1;
	unsigned CN4_M00_X20PS9400a_Bit_Unused_01:1;
	unsigned CN4_M00_X20PS9400a_StatusInput02:1;
	unsigned CN4_M00_X20PS9400a_Bit_Unused_02:5;
	unsigned CN4_M01_X20DI4371_NetworkStatus:8;
	unsigned CN4_M01_X20DI4371_DigitalInput01:1;
	unsigned CN4_M01_X20DI4371_DigitalInput02:1;
	unsigned CN4_M01_X20DI4371_DigitalInput03:1;
	unsigned CN4_M01_X20DI4371_DigitalInput04:1;
	unsigned CN4_M01_X20DI4371_Bit_Unused_01:4;
	unsigned CN4_M02_X20DI4371_NetworkStatus:8;
	unsigned CN4_M02_X20DI4371_DigitalInput01:1;
	unsigned CN4_M02_X20DI4371_DigitalInput02:1;
	unsigned CN4_M02_X20DI4371_DigitalInput03:1;
	unsigned CN4_M02_X20DI4371_DigitalInput04:1;
	unsigned CN4_M02_X20DI4371_Bit_Unused_01:4;
	unsigned CN4_M03_X20DO9322_NetworkStatus:8;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN4_M03_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN4_M03_X20DO9322_Bit_Unused_01:4;
	unsigned CN5_M00_X20PS9400a_NetworkStatus:8;
	unsigned CN5_M00_X20PS9400a_StatusInput01:1;
	unsigned CN5_M00_X20PS9400a_Bit_Unused_01:1;
	unsigned CN5_M00_X20PS9400a_StatusInput02:1;
	unsigned CN5_M00_X20PS9400a_Bit_Unused_02:5;
	unsigned CN5_M01_X20DI9371_NetworkStatus:8;
	unsigned CN5_M01_X20DI9371_DigitalInput01:1;
	unsigned CN5_M01_X20DI9371_DigitalInput02:1;
	unsigned CN5_M01_X20DI9371_DigitalInput03:1;
	unsigned CN5_M01_X20DI9371_DigitalInput04:1;
	unsigned CN5_M01_X20DI9371_DigitalInput05:1;
	unsigned CN5_M01_X20DI9371_DigitalInput06:1;
	unsigned CN5_M01_X20DI9371_DigitalInput07:1;
	unsigned CN5_M01_X20DI9371_DigitalInput08:1;
	unsigned CN5_M01_X20DI9371_DigitalInput09:1;
	unsigned CN5_M01_X20DI9371_DigitalInput10:1;
	unsigned CN5_M01_X20DI9371_DigitalInput11:1;
	unsigned CN5_M01_X20DI9371_DigitalInput12:1;
	unsigned CN5_M01_X20DI9371_Bit_Unused_01:4;
	unsigned CN5_M02_X20DO9322_NetworkStatus:8;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput01:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput02:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput03:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput04:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput05:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput06:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput07:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput08:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput09:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput10:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput11:1;
	unsigned CN5_M02_X20DO9322_StatusDigitalOutput12:1;
	unsigned CN5_M02_X20DO9322_Bit_Unused_01:4;
	unsigned CN7_M00_X20PS9400a_NetworkStatus:8;
	unsigned CN7_M00_X20PS9400a_StatusInput01:1;
	unsigned CN7_M00_X20PS9400a_Bit_Unused_01:1;
	unsigned CN7_M00_X20PS9400a_StatusInput02:1;
	unsigned CN7_M00_X20PS9400a_Bit_Unused_02:5;
	unsigned CN7_M01_X20AI2622_NetworkStatus:8;
	unsigned PADDING_VAR_2:8;
	unsigned CN7_M01_X20AI2622_AnalogInput01:16;
	unsigned CN7_M01_X20AI2622_AnalogInput02:16;
	unsigned CN9_M00_X20PS9400a_NetworkStatus:8;
	unsigned CN9_M00_X20PS9400a_StatusInput01:1;
	unsigned CN9_M00_X20PS9400a_Bit_Unused_01:1;
	unsigned CN9_M00_X20PS9400a_StatusInput02:1;
	unsigned CN9_M00_X20PS9400a_Bit_Unused_02:5;
	unsigned CN9_M01_X20DI9372_NetworkStatus:8;
	unsigned CN9_M01_X20DI9372_DigitalInput01:1;
	unsigned CN9_M01_X20DI9372_DigitalInput02:1;
	unsigned CN9_M01_X20DI9372_DigitalInput03:1;
	unsigned CN9_M01_X20DI9372_DigitalInput04:1;
	unsigned CN9_M01_X20DI9372_DigitalInput05:1;
	unsigned CN9_M01_X20DI9372_DigitalInput06:1;
	unsigned CN9_M01_X20DI9372_DigitalInput07:1;
	unsigned CN9_M01_X20DI9372_DigitalInput08:1;
	unsigned CN9_M01_X20DI9372_DigitalInput09:1;
	unsigned CN9_M01_X20DI9372_DigitalInput10:1;
	unsigned CN9_M01_X20DI9372_DigitalInput11:1;
	unsigned CN9_M01_X20DI9372_DigitalInput12:1;
	unsigned CN9_M01_X20DI9372_Bit_Unused_01:4;
	unsigned CN9_M02_X20DO6321_NetworkStatus:8;
	unsigned CN9_M02_X20DO6321_StatusDigitalOutput01:1;
	unsigned CN9_M02_X20DO6321_StatusDigitalOutput02:1;
	unsigned CN9_M02_X20DO6321_StatusDigitalOutput03:1;
	unsigned CN9_M02_X20DO6321_StatusDigitalOutput04:1;
	unsigned CN9_M02_X20DO6321_StatusDigitalOutput05:1;
	unsigned CN9_M02_X20DO6321_StatusDigitalOutput06:1;
	unsigned CN9_M02_X20DO6321_Bit_Unused_01:2;
	unsigned PADDING_VAR_3:8;
} PI_OUT;

# define COMPUTED_PI_IN_SIZE 24
typedef struct 
{
	unsigned CN1_M02_X20DO9322_DigitalOutput01:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput02:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput03:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput04:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput05:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput06:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput07:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput08:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput09:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput10:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput11:1;
	unsigned CN1_M02_X20DO9322_DigitalOutput12:1;
	unsigned CN1_M02_X20DO9322_Bit_Unused_01:4;
	unsigned CN2_M02_X20DO9322_DigitalOutput01:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput02:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput03:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput04:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput05:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput06:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput07:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput08:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput09:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput10:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput11:1;
	unsigned CN2_M02_X20DO9322_DigitalOutput12:1;
	unsigned CN2_M02_X20DO9322_Bit_Unused_01:4;
	unsigned CN2_M04_X20AO4622_AnalogOutput01:16;
	unsigned CN2_M04_X20AO4622_AnalogOutput02:16;
	unsigned CN2_M04_X20AO4622_AnalogOutput03:16;
	unsigned CN2_M04_X20AO4622_AnalogOutput04:16;
	unsigned CN3_M04_X20DO9322_DigitalOutput01:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput02:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput03:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput04:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput05:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput06:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput07:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput08:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput09:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput10:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput11:1;
	unsigned CN3_M04_X20DO9322_DigitalOutput12:1;
	unsigned CN3_M04_X20DO9322_Bit_Unused_01:4;
	unsigned CN3_M05_X20DO9322_DigitalOutput01:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput02:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput03:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput04:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput05:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput06:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput07:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput08:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput09:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput10:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput11:1;
	unsigned CN3_M05_X20DO9322_DigitalOutput12:1;
	unsigned CN3_M05_X20DO9322_Bit_Unused_01:4;
	unsigned CN3_M06_X20DO9322_DigitalOutput01:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput02:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput03:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput04:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput05:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput06:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput07:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput08:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput09:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput10:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput11:1;
	unsigned CN3_M06_X20DO9322_DigitalOutput12:1;
	unsigned CN3_M06_X20DO9322_Bit_Unused_01:4;
	unsigned CN4_M03_X20DO9322_DigitalOutput01:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput02:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput03:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput04:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput05:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput06:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput07:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput08:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput09:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput10:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput11:1;
	unsigned CN4_M03_X20DO9322_DigitalOutput12:1;
	unsigned CN4_M03_X20DO9322_Bit_Unused_01:4;
	unsigned CN5_M02_X20DO9322_DigitalOutput01:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput02:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput03:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput04:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput05:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput06:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput07:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput08:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput09:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput10:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput11:1;
	unsigned CN5_M02_X20DO9322_DigitalOutput12:1;
	unsigned CN5_M02_X20DO9322_Bit_Unused_01:4;
	unsigned CN9_M02_X20DO6321_DigitalOutput01:1;
	unsigned CN9_M02_X20DO6321_DigitalOutput02:1;
	unsigned CN9_M02_X20DO6321_DigitalOutput03:1;
	unsigned CN9_M02_X20DO6321_DigitalOutput04:1;
	unsigned CN9_M02_X20DO6321_DigitalOutput05:1;
	unsigned CN9_M02_X20DO6321_DigitalOutput06:1;
	unsigned CN9_M02_X20DO6321_Bit_Unused_01:2;
	unsigned PADDING_VAR_1:8;
} PI_IN;

#endif