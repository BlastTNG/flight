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

typedef enum {
	MAG_WE_BIN = 0,
	MAG_BIN,
	MAG_WE_RATE,
	MAG_RATE,
	MAG_CONT,
	MAG_READ,
	MAG_END
} e_mag_state;

typedef struct {
	char cmd[16];
	char resp[16];
} mag_state_cmd_t;

static mag_state_cmd_t state_cmd[MAG_END] = {
		[MAG_WE_BIN] = { "*99WE\r", "OK" },
		[MAG_BIN] = { "*99B\r", "BINARY_ON" },
		[MAG_WE_RATE] = { "*99WE\r", "OK" },
		[MAG_RATE] = { "*99R=100\r", "OK" },
		[MAG_CONT] = { "*99C\r" },
};

static void mag_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);
    mag_state_cmd_t *state = (mag_state_cmd_t*)m_data;
    int which_state = state - state_cmd;

    typedef struct {
    	int16_t mag_x;
    	int16_t mag_y;
    	int16_t mag_z;
    } mag_data_t;
    mag_data_t *mag_reading;

    ph_buf_t *buf;
    char *bufp;

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

    /**
     * Handle the initial handshaking and setup with the magnetometer
     */
    if (which_state < MAG_READ) {
    	bufp = (char*) ph_buf_mem(buf);
    	if (!strncmp(bufp, state_cmd[which_state].resp, sizeof(state_cmd->resp))) {
    		serial->job.data = ++state;
    	}
		if (*state->cmd)ph_stm_write(serial->stream, state->cmd, strlen(state->cmd), NULL);
		ph_buf_delref(buf);
		return;
    }

    /**
     * We expect 2 bytes per reading plus 1 byte for the <CR>
     */
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
    mag_comm = ph_serial_open(MAGCOM, NULL, state_cmd);
    if (!mag_comm) {
    	blast_err("Could not open Magnetometer port %s", MAGCOM);
    	return;
    }
    mag_comm->callback = mag_process_data;

    ph_serial_setspeed(mag_comm, B9600);
    ph_serial_enable(mag_comm, true);
    retval = ph_stm_printf(mag_comm->stream, state_cmd[0].cmd);

    blast_startup("Initialized Magnetometer: %d", retval);
}
