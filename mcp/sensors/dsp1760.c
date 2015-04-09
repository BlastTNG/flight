/* 
 * dsp1760.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 7, 2015 by Seth Hillbrand
 */

#include <sys/io.h>
#include <string.h>

#include <blast.h>
#include <blast_comms.h>
#include <comms_serial.h>
#include <dsp1760.h>

static comms_serial_t *gyro_comm[2] = {NULL};
static const char gyro_port[2][16] = {"/dev/ttyS2","/dev/ttyS3"};

#define DSP1760_NPOLES 10
#define DSP1760_GAIN   1.994635168
#define DSP1760_STATUS_MASK_GY1 0x01
#define DSP1760_STATUS_MASK_GY2 0x02
#define DSP1760_STATUS_MASK_GY3 0x04

#define DSP1760_DATA_HEADER 0xFE81FF55

typedef union
{
    struct
    {
        uint32_t header;
        union
        {
            float x;  /// X-axis rotational speed or incremental angle in radians or degrees (config dependent)
            uint32_t x_raw;
        };
        union
        {
            float y;  /// Y-axis rotational speed or incremental angle in radians or degrees (config dependent)
            uint32_t y_raw;
        };
        union
        {
            float z;  /// Z-axis rotational speed or incremental angle in radians or degrees (config dependent)
            uint32_t z_raw;
        };

        uint8_t reserved[12]; /// Unused

        uint8_t status;     /// status per gyro.  Use DSP1760_STATUS_MASK* 1=valid data, 0=invalid data
        uint8_t sequence;   /// 0-127 incrementing sequence of packets
        int16_t temp;       /// Temperature data, rounded to nearest whole degree (C or F selectable)
        uint32_t crc;
    }__attribute__((packed));
    uint8_t raw_data[36];
} dsp1760_std_t;

typedef struct
{
    int             which;
    uint8_t         header_error_count;
    uint8_t         seq_error_count;
    uint8_t         crc_error_count;
    uint8_t         seq_number;
    uint8_t         gyro_status_count[3];
    uint32_t        packet_count;
    uint8_t         bpos;
    uint8_t         index;
    float           gyro_input[3][DSP1760_NPOLES+1];
    dsp1760_std_t   packet;
} dsp_storage_t;

/// Two sets of 3 gyroscopes with 10 pole filters (5 sample delay)
static dsp_storage_t    gyro_data[2] = {{0}};

/**
 * Transfer the packet data from a new gyro packet to the filter and
 * increment the index value, wrapping at the maximum number of filter
 * positions
 * @param m_gyro Pointer to the gyro storage
 */
static void dsp1760_newvals(dsp_storage_t *m_gyro)
{
    m_gyro->gyro_input[0][m_gyro->index] = m_gyro->packet.x / DSP1760_GAIN;
    m_gyro->gyro_input[1][m_gyro->index] = m_gyro->packet.y / DSP1760_GAIN;
    m_gyro->gyro_input[2][m_gyro->index] = m_gyro->packet.z / DSP1760_GAIN;

    if (++(m_gyro->index) > DSP1760_NPOLES) m_gyro->index = 0;
}

float dsp1760_getval(int m_box, int m_gyro)
{
    static const float alpha[] =
        { +0.0171488822, +0.0000000000, -0.1200421755, -0.0000000000,
          +0.6002108774, +1.0000000000, +0.6002108774, -0.0000000000,
          -0.1200421755, +0.0000000000, +0.0171488822 };
    float gybuf[DSP1760_NPOLES + 1];
    int offset = gyro_data[m_box].index + 1;
    float sum = 0.0;

    memcpy(gybuf, gyro_data[m_box].gyro_input[m_gyro], sizeof(float) * (DSP1760_NPOLES + 1));
    if (offset > DSP1760_NPOLES)
        offset = 0;

    for (int i = 0; i <= DSP1760_NPOLES; i++) {
        sum += (alpha[i] * gybuf[offset++]);
        if (offset > DSP1760_NPOLES)
            offset = 0;
    }
    return sum;
}
uint8_t dsp1760_get_header_error_count(int m_box)
{
    return gyro_data[m_box].header_error_count;
}
uint8_t dsp1760_get_seq_error_count(int m_box)
{
    return gyro_data[m_box].seq_error_count;
}
uint8_t dsp1760_get_crc_error_count(int m_box)
{
    return gyro_data[m_box].crc_error_count;
}
uint8_t dsp1760_get_seq_number(int m_box)
{
    return gyro_data[m_box].seq_number;
}
uint8_t dsp1760_get_gyro_status_count(int m_box, int m_gyro)
{
    return gyro_data[m_box].gyro_status_count[m_gyro];
}
uint32_t dsp1760_get_packet_count(int m_box)
{
    return gyro_data[m_box].packet_count;
}
int16_t dsp1760_get_temp(int m_box)
{
    return gyro_data[m_box].packet.temp;
}

static int activate_921k_clock(comms_serial_t *m_serial)
{
#define SMSC_BASE_ADDR 0x4E
#define SMSC_IO_ADDR (SMSC_BASE_ADDR + 1)
#define SMSC_ENTER_CFG 0x55
#define SMSC_ENTER_RUN 0xAA
#define SMSC_LDNUM_ADDR 0x07

#define SP1_ADDR 0x04
#define SP2_ADDR 0x05

    int reg_val = 0;

    ioperm(SMSC_BASE_ADDR, 0xFF, 1);        // Allow us to frob bits.  We need the full chip (0xFF)

    outb(SMSC_ENTER_CFG, SMSC_BASE_ADDR);   // Put the SMSC Chip into config mode
    outb(SMSC_LDNUM_ADDR, SMSC_BASE_ADDR);
    if (strstr(m_serial->sock->host, "ttyS3"))
        outb(SP2_ADDR, SMSC_IO_ADDR);       // Specify that we want to configure the second port (ttyS3)
    else if (strstr(m_serial->sock->host, "ttyS2"))
        outb(SP1_ADDR, SMSC_IO_ADDR);       // Specify that we want to configure the first port (ttyS2)
    else
        blast_err("Don't know how to configure %s!  This probably won't work", m_serial->sock->host);

    outb(0xF0, SMSC_BASE_ADDR);             // Read from the SP options register
    reg_val = inb(SMSC_IO_ADDR);

    reg_val &= (~0b1111);
    reg_val |= 0b0110;                      // Set base clock of 921600
    outb_p(reg_val, SMSC_IO_ADDR);

    outb(SMSC_ENTER_RUN, SMSC_BASE_ADDR);   // Leave Config Mode
    ioperm(SMSC_BASE_ADDR, 0xFF, 0);        // Remove the bit access again

    return comms_serial_set_baud_base(m_serial, 921600);
}


/**
 * Handles the data inflow from the gyroscopes
 * @param m_data Pointer to the next byte in the gyro data stream
 * @param m_len length of data queued
 * @param m_userdata Pointer to the gyro processing buffer
 * @return Number of bytes consumed by processing
 */
static int dsp1760_process_data(const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)))
{
    dsp_storage_t *gyro = ((comms_serial_t*)m_userdata)->sock->priv_data;
    dsp1760_std_t *pkt = &gyro->packet;
    const uint8_t *buffer = m_data;

     int i;

     for (i = 0; i < m_len; i++) {
         if (pkt->raw_data[0] == 0xFE) {
             pkt->raw_data[gyro->bpos++] = buffer[i];
             if ((gyro->bpos == 4) && (ntohl(pkt->header) != DSP1760_DATA_HEADER /*0xFE81FF55*/)) {
                 pkt->raw_data[0] = '\0';
                 gyro->bpos = 0;
                 gyro->header_error_count++;
                 break;
             }
             if (gyro->bpos == 36) {
                 pkt->x_raw = ntohl(pkt->x_raw);
                 pkt->y_raw = ntohl(pkt->y_raw);
                 pkt->z_raw = ntohl(pkt->z_raw);
                 pkt->temp = ntohs(pkt->temp);
                 pkt->crc = ntohl(pkt->crc);
                 if (pkt->sequence != (++gyro->seq_number & 0x7F)) {
                     blast_warn("Expected Sequence %d but received %d from gyro %d", gyro->seq_number, pkt->sequence, gyro->which);
                     gyro->seq_error_count++;
                 }
                 gyro->packet_count++;
                 gyro->seq_number = pkt->sequence;

                 /// Status is 0 if OK, 1 if faulty
                 gyro->gyro_status_count[0] += (!(pkt->status & DSP1760_STATUS_MASK_GY1));
                 gyro->gyro_status_count[1] += (!(pkt->status & DSP1760_STATUS_MASK_GY2));
                 gyro->gyro_status_count[2] += (!(pkt->status & DSP1760_STATUS_MASK_GY3));

                 pkt->raw_data[0] = '\0';
                 gyro->bpos = 0;
                 //TODO: Add CRC Checking

                 dsp1760_newvals(gyro);
             }
         }
         else {
             if (buffer[i] == 0xFE)
                 pkt->raw_data[gyro->bpos++] = buffer[i];
         }
     }

     return i;

}

static void dsp1760_handle_error (int m_code, void *m_priv)
{
    comms_serial_t *port = (comms_serial_t*)m_priv;
    blast_err("Got error %d on gyro comm %s: %s", m_code, port->sock->host, strerror(m_code));
}

static int dsp1760_handle_finished (const void *m_data, size_t m_len, void *m_userdata __attribute__((unused)))
{
    comms_serial_t *port = (comms_serial_t*)m_userdata;
    if (port && port->sock && port->sock->host)
        blast_err("Got closed socket on %s!  That shouldn't happen", port->sock->host);
    else
        blast_err("Got closed socket on unknown gyro port!");

    if (port && port->sock){
        BLAST_SAFE_FREE(port->sock->callbacks);
        comms_sock_free(port->sock);
        port->sock = NULL;
    }

    return 0;
}

/**
 * Initialize the gyroscope monitoring process
 * @return true on success, false on failure
 */
bool initialize_dsp1760_interface(void)
{

    for (int i = 0; i < 2; i++) {
        BLAST_ZERO(gyro_data[i]);
        gyro_data[i].which = i;
        gyro_comm[i] = comms_serial_new(&gyro_data[i]);
    }

    if (!gyro_comm[0] && !gyro_comm[1] )
    {
        blast_err("Could not allocate any serial port for gyroscopes");
        return false;
    }

    for (int i = 0; i < 2; i++) {
        if (gyro_comm[i]) {
            gyro_comm[i]->sock->callbacks = balloc(err, sizeof(netsock_callbacks_t));
            BLAST_ZERO_P(gyro_comm[i]->sock->callbacks);
            gyro_comm[i]->sock->callbacks->data = dsp1760_process_data;
            gyro_comm[i]->sock->callbacks->error = dsp1760_handle_error;
            gyro_comm[i]->sock->callbacks->finished = dsp1760_handle_finished;
            gyro_comm[i]->sock->callbacks->priv = gyro_comm[i];

            if (comms_serial_connect(gyro_comm[i], gyro_port[i]) != NETSOCK_OK) {
                bfree(err, gyro_comm[i]->sock->callbacks);
                bfree(err, gyro_comm[i]->sock->priv_data);
                comms_serial_free(gyro_comm[i]);
                gyro_comm[i] = NULL;
            }

            comms_serial_setspeed(gyro_comm[i], B38400);
            activate_921k_clock(gyro_comm[i]);
            comms_serial_set_baud_divisor(gyro_comm[i], 921600);
        }

        if (!gyro_comm[i] || !(blast_comms_add_port(gyro_comm[i]))) {
            blast_err("Could not add gyro port %d to our comm monitor", i + 1);
            if (gyro_comm[i]) {
                bfree(err, gyro_comm[i]->sock->callbacks);
                bfree(err, gyro_comm[i]->sock->priv_data);
                comms_serial_free(gyro_comm[i]);
                gyro_comm[i] = NULL;
            }
        }
    }

    blast_startup("Initialized gyroscope interface");
    return true;
}
