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

Housekeeper::Housekeeper(Parameters::Manager& params, dmm *card)
{
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
	
	ad_card = card;
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
	logger.log("I am in add_channel");
    if (map.count(field_name)) {
        measurement.name = map[field_name].as<string>();
    } else {
        return;
    }
    if (map.count(field_type)) {
        type = map[field_type].as<string>();
        if (type == "temperature" && map.count(field_resistor_value)) {
            // measurement.scale = 1000000.0 / (map[field_resistor_value].as<float>());
            // measurement.offset = -273.15;
			measurement.scale = 1;
			measurement.offset = 0;
			measurement.units = " C";
        } else if (type == "pressure" && map.count(field_voltage)) {
            // measurement.scale = 1.0 / (0.004 * 101.325 * (map[field_voltage].as<float>()));
            // measurement.offset = 0.04 / (0.004 * 101.325);
			measurement.scale = 1;
			measurement.offset = 0;
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
				double values[16] = { 0.0 };
				int channel;
                string logdata = "temps=> ";
				string logerror = "";

				ad_card->dmm_scan(values);

				for (unsigned int i = 0; i < 16; i++) {
					logdata += std::to_string(i) + ": " + std::to_string(values[i]) + " ";
				}

                for (unsigned int i=0; i<measurements.size(); i++) {
                    channel = measurements[i].channel;
					measurements[i].add_value(values[channel]);
                }
               

				// int heater_state = shared_housekeeper.heater_state;
				// TODO(javier): remove for flight
				int heater_state = 0;
				if (heater_state > 0) {
					ad_card->heat_camera(true);
					logdata += "Heating";
				}
				else {
					ad_card->heat_camera(false);
					logdata += "Not Heating";
				}
				if (write_temps_counter == 0) {
					logger.log(logdata);
				}
				write_temps_counter = (write_temps_counter + 1) % 10;
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

