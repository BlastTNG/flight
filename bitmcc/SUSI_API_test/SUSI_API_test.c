/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_hwm.c
*
* Programmers:
*	Neo Lo
*
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "SUSI.h"

// Return 0 if platform information is correctly obtained.
// Otherwise, return 1.
int show_platform_info(void)
{
	BOOL result;
	DWORD major, minor, year, month, date;
	DWORD size = 128;
	TCHAR buf[size];  // Buffer length includes the null character.

	SusiDllGetVersion(&major, &minor);
    year    = minor/10000;
    month   = minor%10000/100;
    date    = minor%100;
	printf("Version: %li (20%02li/%02li/%02li)\n", major, year, month, date);

	// Get platform name.
	result = SusiCoreGetPlatformName(buf, &size);
	if (result != TRUE) {
		printf("SusiCoreGetPlatformName() failed\n");
		return 1;
	}
	else
		printf("Platform name: %s\n", buf);

	// Get BIOS version.
	result = SusiCoreGetBIOSVersion(buf, &size);
	if (result != TRUE) {
		return 1;
	}
	else
		printf("BIOS version: %s\n", buf);

	return 0;
}

void show_menu(void)
{
	printf("\n");
	printf("0) Terminate this program\n");
	printf("1) Get Voltage\n");
	printf("2) Get Temperature\n");
	printf("3) Get Fan Speed\n");
	printf("4) Set Fan Speed\n");
	printf("Enter your choice: \n");
}

int get_voltage(void)
{
	BOOL result;
	DWORD voltType, typeSupport;
    float retval = 0;

	printf("Get Voltage\n");

    voltType = typeSupport = 0;
	result = SusiHWMGetVoltage(voltType, &retval, &typeSupport);
	if (result == FALSE) {
		printf("SusiHWMGetVoltage() failed\n");
		return 1;
	}
	else
		printf("typeSupport: 0x%08lX\n", typeSupport);

    if ( typeSupport )
    {
        if ( VCORE & typeSupport )
        {
        	result = SusiHWMGetVoltage(VCORE, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("VCORE: %f\n", retval);
        }

        if ( V25 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V25, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V25: %f\n", retval);
        }

        if ( V33 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V33, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V33: %f\n", retval);
        }

        if ( V50 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V50, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V50: %f\n", retval);
        }

        if ( V120 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V120, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V120: %f\n", retval);
        }

        if ( V5SB & typeSupport )
        {
        	result = SusiHWMGetVoltage(V5SB, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V5SB: %f\n", retval);
        }

        if ( V3SB & typeSupport )
        {
        	result = SusiHWMGetVoltage(V3SB, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V3SB: %f\n", retval);
        }


        if ( VBAT & typeSupport )
        {
        	result = SusiHWMGetVoltage(VBAT, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("VBAT: %f\n", retval);
        }

        if ( VN50 & typeSupport )
        {
        	result = SusiHWMGetVoltage(VN50, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("VN50: %f\n", retval);
        }

        if ( VN120 & typeSupport )
        {
        	result = SusiHWMGetVoltage(VN120, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("VN120: %f\n", retval);
        }

        if ( VTT & typeSupport )
        {
        	result = SusiHWMGetVoltage(VTT, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("VTT: %f\n", retval);
        }

        if ( VCORE2 & typeSupport )
        {
        	result = SusiHWMGetVoltage(VCORE2, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("VCORE2: %f\n", retval);
        }

        if ( V105 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V105, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V105: %f\n", retval);
        }

        if ( V15 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V15, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V15: %f\n", retval);
        }

        if ( V18 & typeSupport )
        {
        	result = SusiHWMGetVoltage(V18, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetVoltage() failed\n");
		        return 1;
        	}
        	else
        		printf("V18: %f\n", retval);
        }

   }

	return 0;
}

int get_temperature(void)
{
	BOOL result;
	WORD voltType, typeSupport;
    float retval = 0;

	printf("Get Temperature\n");

    voltType = typeSupport = 0;
	result = SusiHWMGetTemperature(voltType, &retval, &typeSupport);
	if (result == FALSE) {
		printf("SusiHWMGetTemperature() failed\n");
		return 1;
	}
	else
		printf("typeSupport: 0x%08X\n", typeSupport);

    if ( typeSupport )
    {
        if ( TCPU & typeSupport )
        {
        	result = SusiHWMGetTemperature(TCPU, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetTemperature() failed\n");
		        return 1;
        	}
        	else
        		printf("TCPU: %f\n", retval);
        }

        if ( TSYS & typeSupport )
        {
        	result = SusiHWMGetTemperature(TSYS, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetTemperature() failed\n");
		        return 1;
        	}
        	else
        		printf("TSYS: %f\n", retval);
        }

        if ( TAUX & typeSupport )
        {
        	result = SusiHWMGetTemperature(TAUX, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetTemperature() failed\n");
		        return 1;
        	}
        	else
        		printf("TAUX: %f\n", retval);
        }

        if ( TCPU2 & typeSupport )
        {
        	result = SusiHWMGetTemperature(TCPU2, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetTemperature() failed\n");
		        return 1;
        	}
        	else
        		printf("TCPU2: %f\n", retval);
        }
   }
	return 0;
}

int get_fan_speed(void)
{
	BOOL result;
    WORD typeSupport, voltType, retval;

	printf("Get Fan Speed\n");

    typeSupport = voltType = retval = 0;
	result = SusiHWMGetFanSpeed(voltType, &retval, &typeSupport);
	if (result == FALSE) {
		printf("SusiHWMGetFanSpeed() failed\n");
		return 1;
	}
	else
		printf("typeSupport: 0x%08X\n", typeSupport);

    if ( typeSupport )
    {
        if ( FCPU & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(FCPU, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("FCPU: %d\n", retval);
        }

        if ( FSYS & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(FSYS, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("FSYS: %d\n", retval);
        }

        if ( F2ND & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(F2ND, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("F2ND: %d\n", retval);
        }

        if ( FCPU2 & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(FCPU2, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("FCPU2: %d\n", retval);
        }
    }
	return 0;
}

int set_fan_speed(void)
{
	BOOL result;
    WORD typeSupport, voltType, retval;

	printf("Set Fan Speed\n");

    typeSupport = voltType = retval = 0;
	result = SusiHWMGetFanSpeed(voltType, &retval, &typeSupport);
	if (result == FALSE) {
		printf("SusiHWMGetFanSpeed() failed\n");
		return 1;
	}
	else
		printf("typeSupport: 0x%08X\n", typeSupport);

    if ( typeSupport )
    {
        if ( FCPU & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(FCPU, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("FCPU: %d\n", retval);

            printf("Please enter the new FanSpeed: \n");
			if (scanf("%hi", &retval) <= 0)
				return 1;

        	result = SusiHWMSetFanSpeed(FCPU, retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMSetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("Set FanSpeed OK!\n");
        }

        if ( FSYS & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(FSYS, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("FSYS: %d\n", retval);

            printf("Please enter the new FanSpeed: \n");
			if (scanf("%hi", &retval) <= 0)
				return 1;

        	result = SusiHWMSetFanSpeed(FSYS, retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMSetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("Set FanSpeed OK!\n");
        }

        if ( F2ND & typeSupport )
        {
        	result = SusiHWMGetFanSpeed(F2ND, &retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMGetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("F2ND: %d\n", retval);

            printf("Please enter the new FanSpeed: \n");
			if (scanf("%hi", &retval) <= 0)
				return 1;

        	result = SusiHWMSetFanSpeed(F2ND, retval, NULL);
        	if (result == FALSE) {
        		printf("SusiHWMSetFanSpeed() failed\n");
		        return 1;
        	}
        	else
        		printf("Set FanSpeed OK!\n");
        }
    }
	return 0;
}


int main(void)
{
	int result, done, op;

	result = SusiDllInit();
	if (result == FALSE) {
		printf("SusiDllInit() failed\n");
		return 1;
	}

	result = SusiHWMAvailable();
	if (result == FALSE) {
		printf("SusiHWMAvailable() failed\n");
		SusiDllUnInit();
		return 1;
	}

	result = show_platform_info();

	done = 0;
	while (! done) {
		show_menu();
		if (scanf("%i", &op) <= 0)
			op = -1;

		switch (op) {
		case 0:
			done = 1;
			continue;
		case 1:
			result = get_voltage();
			break;
		case 2:
			result = get_temperature();
			break;
		case 3:
			result = get_fan_speed();
			break;
		case 4:
			result = set_fan_speed();
			break;
		default:
			printf("\nUnknown choice!\n\n");
			continue;
		}
		if (result != 0) {
			printf("Library returns with error.\n");
			SusiDllUnInit();
			return 1;
		}
	}

	result = SusiDllUnInit();
	if (result == FALSE) {
		printf("SusiDllUnInit() failed\n");
		return 1;
	}

	return 0;
}

