#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/serial.h"
#include "phenom/memory.h"

#include <stdint.h>
#include <endian.h>

#include <blast.h>
#include <channels_tng.h>
#include <pointing_struct.h>

extern struct ACSDataStruct ACSData;

#define MAGCOM "/dev/ttyUSB0"

ph_serial_t	*mag_comm = NULL;

static void mag_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);

    typedef struct {
    	int16_t mag_x;
    	int16_t mag_y;
    	int16_t mag_z;
    } mag_data_t;
    mag_data_t *mag_reading;

    ph_buf_t *buf;
//    char *bufp;
    static channel_t *mag_x_channel = NULL;
    static channel_t *mag_y_channel = NULL;
    static channel_t *mag_z_channel = NULL;

    if (!mag_x_channel) {
    	mag_x_channel = channels_find_by_name("x_mag");
    	mag_y_channel = channels_find_by_name("y_mag");
    	mag_z_channel = channels_find_by_name("z_mag");
    }
    if (!(buf = ph_serial_read_record(serial, "\r", 1))) {
    	return;
    }

    if (ph_buf_len(buf) != 7) {
    	ph_buf_delref(buf);
    	return;
    }

    mag_reading = (mag_data_t*)ph_buf_mem(buf);

    ACSData.mag_x = ((double)(int16_t)be16toh(mag_reading->mag_x))/15000.0;
    SET_SCALED_VALUE(mag_x_channel, ACSData.mag_x);
    ACSData.mag_y = ((double)(int16_t)be16toh(mag_reading->mag_y))/15000.0;
    SET_SCALED_VALUE(mag_y_channel, ACSData.mag_y);
    ACSData.mag_z = ((double)(int16_t)be16toh(mag_reading->mag_z))/15000.0;
    SET_SCALED_VALUE(mag_z_channel, ACSData.mag_z);

    ph_buf_delref(buf);
}
void initialize_magnetometer(void)
{
	int retval = 0;

    if (mag_comm) ph_serial_free(mag_comm);
    mag_comm = ph_serial_open(MAGCOM, NULL, NULL);
    if (!mag_comm) {
    	blast_err("Could not open Magnetometer port %s", MAGCOM);
    	return;
    }
    mag_comm->callback = mag_process_data;

    ph_serial_setspeed(mag_comm, B9600);
    ph_serial_enable(mag_comm, true);
    retval = ph_stm_printf(mag_comm->stream, "*99C\r");

    blast_startup("Initialized magnetometer: %d", retval);
}
