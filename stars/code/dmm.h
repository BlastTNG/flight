#pragma once

#include "dscud.h"

class dmm
{
public:
	dmm();
	~dmm();
	void dmm::dmm_initialize();
	void dmm::dmm_scan(double *);
	void dmm::cycle_camera(void);
	void dmm::heat_camera(bool enable);

private:
	bool initialized;

	DSCB dscb;   // handle used to refer to the DMM board
	DSCCB dsccb; // structure containing board settings
	DSCADSETTINGS dscadsettings; // structure containing A/D conversion settings
	DSCADSCAN dscadscan; // structure containing A/D scan settings

};

