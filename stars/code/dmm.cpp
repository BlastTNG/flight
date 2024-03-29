
#include <string>
#include <windows.h>

#include "logger_main.h"
#include "dmm.h"

using Main::logger;

dmm::dmm()
{
	memset(&dsccb, 0, sizeof(dsccb));
	dsccb.base_address = 0x300;
	dsccb.int_level = 3;
	dscb = 0;

	initialized = false;
}


dmm::~dmm()
{
	if (initialized) dscFree();
	initialized = false;

}

void dmm::dmm_initialize()
{
	ERRPARAMS errorParams; // structure for returning error code and error string
	string logerror = "";
	logger.log(format("Initializing DMM-XT").str());

	if (!initialized) {
		if (dscInit(DSC_VERSION) != DE_NONE) {
			dscGetLastError(&errorParams);
			logerror = (format("dscInit error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
			logger.log(logerror);
			return;
		}
	}

	if (dscInitBoard(DSC_DMM, &dsccb, &dscb) != DE_NONE) {
		dscGetLastError(&errorParams);
		logerror = (format("dscInitBoard error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
		logger.log(logerror);
		dscFree();
		initialized = false;
		return;
	}

	dscadsettings.range = RANGE_5;
	dscadsettings.polarity = UNIPOLAR;
	dscadsettings.gain = GAIN_1;
	dscadsettings.load_cal = (BYTE)TRUE;
	dscadsettings.current_channel = 0;

	dscadscan.low_channel = 0;
	dscadscan.high_channel = 15;

	if (dscADSetSettings(dscb, &dscadsettings) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		logerror += (format("dscADSetSettings error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
		logger.log(logerror);
		dscFreeBoard(dscb);
		dscFree();
		initialized = false;
		return;
	}

	initialized = true;
}

void dmm::dmm_scan(double *values)
{
	ERRPARAMS errorParams; // structure for returning error code and error string
	string logerror = "";
	DSCSAMPLE buf[16];
	
	if (!initialized) return;

	if (dscADScan(dscb, &dscadscan, buf) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		logerror = (format("dscADScan error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
		logger.log(logerror);
	}

	for (unsigned int i = 0; i < 16; i++) {
		if (dscADCodeToVoltage(dscb, dscadsettings, buf[i], &values[i]) != DE_NONE)  {
			dscGetLastError(&errorParams);
			logerror = (format("dscADCodeToVoltage error on channel %d : %s (%s)") % i % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
			logger.log(logerror);
			values[i] = nan("");
		}

	}
}

void dmm::cycle_camera(void)
{

	BYTE dscbresult;	   //variable for error handling
	static const BYTE port = 0;
	static const BYTE bit = 2;

	if (!initialized)
	{
		logger.log("Not cycling camera because DMM board not initialized\n");
		return;
	}

	logger.log(format("Power cycling the camera..."));
	if ((dscbresult = dscDIOClearBit(dscb, port, bit)) != DE_NONE)
	{
		logger.log(format("failed to clear power cycle bit %d: %s \n") %bit % dscGetErrorString(dscbresult));
		return;
	}
	Sleep(200);
	if ((dscbresult = dscDIOSetBit(dscb, port, bit)) != DE_NONE)
	{
		logger.log(format("failed to set power cycle bit %d: %s \n") % bit % dscGetErrorString(dscbresult));
		return;
	}
	Sleep(500);
	if ((dscbresult = dscDIOClearBit(dscb, port, bit)) != DE_NONE)
	{
		logger.log(format("failed to clear power cycle bit %d: %s \n") % bit % dscGetErrorString(dscbresult));
	}
}

void dmm::heat_camera(bool enable)
{
	BYTE dscbresult;	   //variable for error handling
	static const BYTE port = 0;
	static const BYTE bit = 1;

	if (!initialized) return;

	if (enable)
	{
		if ((dscbresult = dscDIOSetBit(dscb, port, bit)) != DE_NONE)
		{
			logger.log(format("failed to set heater bit: %s\n") % dscGetErrorString(dscbresult));
			return;
		}
	}
	else
	{
		if ((dscbresult = dscDIOClearBit(dscb, port, bit)) != DE_NONE)
		{
			logger.log(format("failed to clear heater bit: %s\n") % dscGetErrorString(dscbresult));
			return;
		}
	}
}