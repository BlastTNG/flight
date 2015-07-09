/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "housekeeper.h"

#include <fstream>
#include <boost/format.hpp>
#include "../parameters/manager.h"
#include "../shared/housekeeping/housekeeper.h"
#include "logger.h"

using namespace Housekeeping;
using std::string;

#define shared_housekeeper (*(Shared::Housekeeping::housekeeper_for_camera.w))

Housekeeper::Housekeeper(Parameters::Manager& params)
{
	ERRPARAMS errorParams; // structure for returning error code and error string
	string logerror = "";

    disk_space = 0.0;
    disk_space_first_measurement = 0.0;
    disk_time = 0.0;
    disk_time_valid = false;
    for (int channel_num=0; channel_num<16; channel_num++) {
        add_channel(params.housekeeping.map, channel_num);
    }
    output_dir = params.general.try_get("main.output_dir", string("C:\\stars_data"));	
    write_temps_counter = 0;
    write_disk_counter = 0;

	if (dscInit(DSC_VERSION) != DE_NONE) {
		dscGetLastError(&errorParams);
		logerror += (format("dscInit error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
		logger.log(logerror);
		return;
	}

	dsccb.base_address = 0x300;
	dsccb.int_level = 3;
	if (dscInitBoard(DSC_DMM, &dsccb, &dscb) != DE_NONE) {
		dscGetLastError(&errorParams);
		logerror += (format("dscInitBoard error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
		logger.log(logerror);
		return;
	}

	dscadsettings.range = RANGE_10;
	dscadsettings.polarity = BIPOLAR;
	dscadsettings.gain = GAIN_1;
	dscadsettings.load_cal = (BYTE)TRUE;
	dscadsettings.current_channel = 0;

	if (dscADSetSettings(dscb, &dscadsettings) != DE_NONE)
	{
		dscGetLastError(&errorParams);
		logerror += (format("dscADSetSettings error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
		logger.log(logerror);
		return;
	}


	dscadscan.low_channel = 0;
	dscadscan.high_channel = 7;
	dscadscan.gain = GAIN_1;
	samples.resize(dscadscan.high_channel - dscadscan.low_channel + 1);
}

void Housekeeper::add_channel(variables_map map, int channel_num)
{
    string field_name = (boost::format("channel%01d.name") % channel_num).str();
    string field_type = (boost::format("channel%01d.type") % channel_num).str();
    string field_resistor_value = (boost::format("channel%01d.resistor_value") % channel_num).str();
    string field_voltage = (boost::format("channel%01d.voltage") % channel_num).str();
    Measurement measurement;
    string type = "";
    measurement.channel = channel_num;
    if (map.count(field_name)) {
        measurement.name = map[field_name].as<string>();
    } else {
        return;
    }
    if (map.count(field_type)) {
        type = map[field_type].as<string>();
        if (type == "temperature" && map.count(field_resistor_value)) {
            measurement.scale = 1000000.0 / (map[field_resistor_value].as<float>());
            measurement.offset = -273.15;
            measurement.units = " C";
        } else if (type == "pressure" && map.count(field_voltage)) {
            measurement.scale = 1.0 / (0.004 * 101.325 * (map[field_voltage].as<float>()));
            measurement.offset = 0.04 / (0.004 * 101.325);
            measurement.units = " a";
        } else {
            return;
        }
    }
    measurements.push_back(measurement);
}

void Housekeeper::update_shared()
{
    if (shared_housekeeper.measurements.size() != measurements.size()+1) {
        shared_housekeeper.measurements.clear();
        for (unsigned int i=0; i<measurements.size()+2; i++) {
            Shared::Housekeeping::Measurement shared_measurement;
            shared_housekeeper.measurements.push_back(shared_measurement);
        }
    }
    for (unsigned int i=0; i<2; i++) {
        if (i==0 && last_successful_disk_space_timer.time() < 120.0) {
            shared_housekeeper.measurements[i].name = "disk";
            shared_housekeeper.measurements[i].units = " G";
            shared_housekeeper.measurements[i].value = disk_space;
            shared_housekeeper.measurements[i].valid = true;
        } else if (i==1 && disk_time_valid) {
            shared_housekeeper.measurements[i].name = "disktime";
            shared_housekeeper.measurements[i].units = " days";
            shared_housekeeper.measurements[i].value = disk_time;
            shared_housekeeper.measurements[i].valid = true;
        } else {
            shared_housekeeper.measurements[i].name = "disk";
            shared_housekeeper.measurements[i].units = " G";
            shared_housekeeper.measurements[i].value = 0.0;
            shared_housekeeper.measurements[i].valid = false;
        }
    }
    for (unsigned int i=0; i<measurements.size(); i++) {
        shared_housekeeper.measurements[i+2].name = measurements[i].name;
        shared_housekeeper.measurements[i+2].units = measurements[i].units;
        shared_housekeeper.measurements[i+2].valid =
            measurements[i].get_value(shared_housekeeper.measurements[i+2].value);
    }
    Shared::Housekeeping::housekeeper_for_camera.share();
}

void Housekeeper::update()
{
    if (last_temps_measurement_timer.time() > 0.5) {
        last_temps_measurement_timer.start();
        #if HAVEDAQ
			try {
				ERRPARAMS errorParams; // structure for returning error code and error string
                int status0 = 0;
				int status1 = 0;
                int channel = 0;
                int board_number = 0;
                WORD raw_value = 0;
                double value = 0;
                string logdata = "temps";
				string logerror = "";

				if (dscADScan(dscb, &dscadscan, &samples.front()) != DE_NONE)
				{
					dscGetLastError(&errorParams);
					logerror += (format("dscADScan error: %s (%s)") % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
					logger.log(logerror);
					throw std::runtime_error(errorParams.errstring);
				}

                for (unsigned int i=0; i<measurements.size(); i++) {
                    channel = measurements[i].channel;
					if (dscADCodeToVoltage(dscb, dscadsettings, dscadscan.sample_values[channel], &value) != DE_NONE)  {
						dscGetLastError(&errorParams);
						logdata += (format("dscADCodeToVoltage error on channel %d : %s (%s)") % channel % dscGetErrorString(errorParams.ErrCode) % errorParams.errstring).str();
						value = nan("");
					}
					else {
						measurements[i].add_value(double(value));
						logdata += (format(" %s %.03d") % measurements[i].name % (double(value))).str();
					}

                }
                if (write_temps_counter == 0) {
                    logger.log(logdata);
                }
                write_temps_counter = (write_temps_counter+1)%10;
            } catch (...) {
            }
        #else
            for (unsigned int i=0; i<measurements.size(); i++) {
                //measurements[i].add_value(double(i)*0.01+2.0);
            }
        #endif
        update_shared();
    }

    if (last_disk_measurement_timer.time() > 5.0) {
        last_disk_measurement_timer.start();
        get_disk();
        update_shared();
    }

    logger.update();
};

