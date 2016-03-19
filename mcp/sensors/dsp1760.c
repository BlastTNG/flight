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

#include "dsp1760.h"

#include <sys/io.h>
#include <string.h>

#include "phenom/listener.h"
#include "phenom/log.h"
#include "phenom/serial.h"
#include "phenom/thread.h"
#include "phenom/sysutil.h"

#include "blast.h"
#include "crc.h"
#include "conversions.h"

static ph_serial_t *gyro_comm[2] = {NULL};
// ttyGYRO0 -> COMM3, ttyGRYO1 -> COMM4
static const char gyro_port[2][16] = {"/dev/ttyGYRO0", "/dev/ttyGYRO1"};

static const uint32_t min_backoff_sec = 1;
static const uint32_t max_backoff_sec = 60;

#define DSP1760_NPOLES 10
#define DSP1760_GAIN   1.994635168
#define DSP1760_STATUS_MASK_GY1 0x01
#define DSP1760_STATUS_MASK_GY2 0x02
#define DSP1760_STATUS_MASK_GY3 0x04

#define DSP1760_1000HZ 1000.0
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
    uint32_t        backoff_sec;
    ph_job_t        connect_job;
    bool            want_reset;

    uint8_t         seq_error_count;
    uint8_t         crc_error_count;
    uint8_t         seq_number;
    uint8_t         gyro_invalid_packet_count[3];
    uint32_t        packet_count;
    uint32_t        gyro_valid_packet_count[3];
    uint8_t         index;
    int16_t         temp;
    float           gyro_input[3][DSP1760_NPOLES+1];
} dsp_storage_t;

/// Two sets of 3 gyroscopes with 10 pole filters (5 sample delay)
static dsp_storage_t    gyro_data[2] = {{0}};

/**
 * Transfer the packet data from a new gyro packet to the filter and
 * increment the index value, wrapping at the maximum number of filter
 * positions.  Currently output is in delta angle and mcp uses rotation rate,
 * so convert here to radians / second instead of radians per 0.001 second
 * @param m_gyro Pointer to the gyro storage
 */
/// TODO(seth): Evaluate whether we want to use delta angle instead!
/// TODO(seth): Change gyros to output degrees
static void dsp1760_newvals(dsp1760_std_t *m_packet, dsp_storage_t *m_gyro)
{
    if (m_packet->status & DSP1760_STATUS_MASK_GY1)
        m_gyro->gyro_input[0][m_gyro->index] = to_degrees(m_packet->x) * DSP1760_1000HZ / DSP1760_GAIN;
    if (m_packet->status & DSP1760_STATUS_MASK_GY2)
        m_gyro->gyro_input[1][m_gyro->index] = to_degrees(m_packet->y) * DSP1760_1000HZ / DSP1760_GAIN;
    if (m_packet->status & DSP1760_STATUS_MASK_GY3)
        m_gyro->gyro_input[2][m_gyro->index] = to_degrees(m_packet->z) * DSP1760_1000HZ / DSP1760_GAIN;

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
    return gyro_data[m_box].gyro_invalid_packet_count[m_gyro];
}
uint32_t dsp1760_get_packet_count(int m_box)
{
    return gyro_data[m_box].packet_count;
}
uint32_t dsp1760_get_valid_packet_count(int m_box, int m_gyro)
{
    return gyro_data[m_box].gyro_valid_packet_count[m_gyro];
}
int16_t dsp1760_get_temp(int m_box)
{
    return gyro_data[m_box].temp;
}

/**
 *
 * @param m_serial
 */
static int activate_921k_clock(uint8_t m_serial)
{
#define SMSC_BASE_ADDR 0x4E
#define SMSC_IO_ADDR (SMSC_BASE_ADDR + 1)
#define SMSC_ENTER_CFG 0x55
#define SMSC_ENTER_RUN 0xAA
#define SMSC_LDNUM_ADDR 0x07

#define SP1_ADDR 0x04
#define SP2_ADDR 0x05
    const int sp_addr[2] = {SP1_ADDR, SP2_ADDR};
    int reg_val = 0;

    if (m_serial > 1) {
        blast_err("Invalid serial port for 921k!");
        return -1;
    }
    ioperm(SMSC_BASE_ADDR, 0xFF, 1);        // Allow us to frob bits.  We need the full chip (0xFF)

    outb(SMSC_ENTER_CFG, SMSC_BASE_ADDR);   // Put the SMSC Chip into config mode
    outb(SMSC_LDNUM_ADDR, SMSC_BASE_ADDR);
    outb(sp_addr[m_serial], SMSC_IO_ADDR);

    outb(0xF0, SMSC_BASE_ADDR);             // Read from the SP options register
    reg_val = inb(SMSC_IO_ADDR);

    reg_val &= (~0b1111);
    reg_val |= 0b0110;                      // Set base clock of 921600
    outb_p(reg_val, SMSC_IO_ADDR);

    outb(SMSC_ENTER_RUN, SMSC_BASE_ADDR);   // Leave Config Mode
    ioperm(SMSC_BASE_ADDR, 0xFF, 0);        // Remove the bit access again
    return 0;
}

/**
 * Disconnect the serial port and set it to reconnect in the future.  This function will also
 * adjust the backoff time doubling our wait time to the specified maximum
 * @param m_serial Pointer to the serial device
 * @param gyro Pointer to the gyro being reset
 */
static inline void dsp1760_disconnect(ph_serial_t *m_serial, dsp_storage_t *gyro)
{
    blast_info("Will attempt to reconnect to Gyro box %d in %d seconds", gyro->which, gyro->backoff_sec);
    ph_serial_enable(m_serial, false);
    ph_serial_free(m_serial);
    gyro_comm[gyro->which] = NULL;
    ph_job_set_timer_in_ms(&gyro->connect_job, gyro->backoff_sec * 1000);
    if (gyro->backoff_sec < max_backoff_sec) gyro->backoff_sec *= 2;
    if (gyro->backoff_sec > max_backoff_sec) gyro->backoff_sec = max_backoff_sec;
}

/**
 * Handles the data inflow from the gyroscopes
 * @param m_serial Pointer to serial stream
 * @param m_why Reason process data was called
 * @param m_userdata Pointer to the state storage
 * @return Number of bytes consumed by processing
 */
static void dsp1760_process_data(ph_serial_t *m_serial, ph_iomask_t m_why, void *m_userdata)
{
    dsp_storage_t *gyro = (dsp_storage_t *) m_userdata;
    dsp1760_std_t *pkt;
    const char header[4] = { 0xFE, 0x81, 0xFF, 0x55 };

    if ((m_why & (PH_IOMASK_ERR)) && m_serial->conn->last_err != 0) {
        blast_err("Disconnecting Gyro Box %d due to Error %d", gyro->which, m_serial->conn->last_err);
        dsp1760_disconnect(m_serial, gyro);
        return;
    }

    if ((m_why & PH_IOMASK_TIME) && !(m_why & PH_IOMASK_READ)) {
        blast_err("Timeout on Gyro box %d", gyro->which);
        dsp1760_disconnect(m_serial, gyro);
        return;
    }

    if (!(m_why & PH_IOMASK_READ)) return;

    /// We loop here to catch multiple packets that may have been delivered since we last read
    while (ph_bufq_discard_until(m_serial->rbuf, header, 4)) {
        ph_buf_t *buf;
        bool invalid_data = false;
        uint32_t crc_calc = 0xFFFFFFFF;

        if (!(buf = ph_serial_read_bytes_exact(m_serial, sizeof(dsp1760_std_t)))) {
            return;
        }
        pkt = (dsp1760_std_t*) ph_buf_mem(buf);

        crc_calc = crc32_be(crc_calc, pkt->raw_data, 36);
        if (crc_calc) {
            blast_warn("Received invalid CRC from Gyro %d", gyro->which);
            blast_info("Buffer size %u", (unsigned)ph_bufq_len(m_serial->rbuf));

            gyro->crc_error_count++;
            invalid_data = true;
            {
                char namebuf[32];
                int fd;
                ph_stream_t *stm;
                snprintf(namebuf, sizeof(namebuf), "/tmp/gyro%d_crc_XXXXXX", gyro->which);
                fd = ph_mkostemp(namebuf, 0);
                stm = ph_stm_fd_open(fd, 0, PH_STM_BUFSIZE);
                ph_stm_write(stm, pkt->raw_data, sizeof(dsp1760_std_t), NULL);
                ph_stm_close(stm);
            }
        }

        pkt->x_raw = ntohl(pkt->x_raw);
        if (isnanf(pkt->x)) {
            blast_warn("Received NaN from Gyro %d(X)", gyro->which);
            pkt->status &= (~DSP1760_STATUS_MASK_GY1);
            invalid_data = true;
        }
        pkt->y_raw = ntohl(pkt->y_raw);
        if (isnanf(pkt->y)) {
            blast_warn("Received NaN from Gyro %d(Y)", gyro->which);
            pkt->status &= (~DSP1760_STATUS_MASK_GY2);
            invalid_data = true;
        }
        pkt->z_raw = ntohl(pkt->z_raw);
        if (isnanf(pkt->z)) {
            blast_warn("Received NaN from Gyro %d(Z)", gyro->which);
            pkt->status &= (~DSP1760_STATUS_MASK_GY3);
            invalid_data = true;
        }
        pkt->temp = ntohs(pkt->temp);

        if (!invalid_data && pkt->sequence != (++gyro->seq_number & 0x7F)) {
            blast_warn("Expected Sequence %d but received %d from gyro %d", gyro->seq_number, pkt->sequence,
                       gyro->which);
            blast_info("Buffer size %u", (unsigned)ph_bufq_len(m_serial->rbuf));
            gyro->seq_error_count++;
            {
                char namebuf[32];
                int fd;
                ph_stream_t *stm;
                snprintf(namebuf, sizeof(namebuf), "/tmp/gyro%d_seq_XXXXXX", gyro->which);

                fd = ph_mkostemp(namebuf, 0);
                stm = ph_stm_fd_open(fd, 0, PH_STM_BUFSIZE);
                ph_stm_write(stm, pkt->raw_data, sizeof(dsp1760_std_t), NULL);
                ph_stm_close(stm);
            }
        }
        gyro->packet_count++;

        if (!invalid_data) {
            /***
             * Here, we reset the current sequence number but only if we received a valid packet from
             * the gyroscopes.  This allows us to reset the sequence numbering if we miss many packets
             * but doesn't screw up the sequence if we receive a malformed packet.
             *
             * Additionally, we reset the "back-off" timer that controls our reconnect speed
             */
            gyro->seq_number = pkt->sequence;
            gyro->backoff_sec = min_backoff_sec;
        }

        /// Status is 1 if OK, 0 if faulty
        gyro->gyro_invalid_packet_count[0] += (!(pkt->status & DSP1760_STATUS_MASK_GY1));
        gyro->gyro_invalid_packet_count[1] += (!(pkt->status & DSP1760_STATUS_MASK_GY2));
        gyro->gyro_invalid_packet_count[2] += (!(pkt->status & DSP1760_STATUS_MASK_GY3));

        gyro->gyro_valid_packet_count[0] += (pkt->status & DSP1760_STATUS_MASK_GY1);
        gyro->gyro_valid_packet_count[1] += (pkt->status & DSP1760_STATUS_MASK_GY2);
        gyro->gyro_valid_packet_count[2] += (pkt->status & DSP1760_STATUS_MASK_GY3);

        if (!invalid_data) dsp1760_newvals(pkt, gyro);

        ph_buf_delref(buf);
    }

    /// If we have accumulated the equivalent of 2
    if (ph_bufq_len(m_serial->rbuf) > 10 * sizeof(dsp1760_std_t) || gyro->want_reset) {
        char namebuf[32];
        int fd;
        ph_stream_t *stm;
        snprintf(namebuf, sizeof(namebuf), "/tmp/gyro%d_over_XXXXXX", gyro->which);

        fd = ph_mkostemp(namebuf, 0);
        stm = ph_stm_fd_open(fd, 0, PH_STM_BUFSIZE);
        ph_bufq_stm_write(m_serial->rbuf, stm, NULL);
        ph_stm_close(stm);

        if (gyro->want_reset) {
            blast_info("Resetting Gyro Box %d due to user command", gyro->which);
        } else {
            blast_err("Disconnecting Gyro Box %d garbage in stream (multiple-access?)", gyro->which);
        }
        dsp1760_disconnect(m_serial, gyro);
    }
}

static void dsp1760_connect_gyro(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    struct termios term = {0};
    dsp_storage_t *data = (dsp_storage_t*)m_data;
    int gyrobox = data->which;

    if (gyro_comm[gyrobox]) ph_serial_free(gyro_comm[gyrobox]);

    term.c_cflag = CS8 | B38400 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR | IGNBRK;

    gyro_comm[gyrobox] = ph_serial_open(gyro_port[gyrobox], &term, data);

    if (ph_serial_set_baud_base(gyro_comm[gyrobox], 921600)) blast_strerror("Error setting base");
    if (ph_serial_set_baud_divisor(gyro_comm[gyrobox], 921600)) blast_strerror("Error setting divisor");

    gyro_comm[gyrobox]->callback = dsp1760_process_data;
    gyro_comm[gyrobox]->timeout_duration.tv_sec = 1;
    gyro_comm[gyrobox]->timeout_duration.tv_usec = 0;

    ph_serial_enable(gyro_comm[gyrobox], true);
}

/**
 * Clears, queues for closing and resets the gyrobox
 * @param m_gyrobox which gyrobox should be reset?
 */
void dsp1760_reset_gyro(int m_which)
{
    gyro_data[m_which].want_reset = true;
}

/**
 * Initialize the gyroscope monitoring process
 * @return true on success, false on failure
 */
bool initialize_dsp1760_interface(void)
{
    /// Define a separate job pool for the gyro read.
//    gyro_pool = ph_thread_pool_define("gyro_read", 4, 1);

    for (int i = 0; i < 2; i++) {
        activate_921k_clock(i);
        BLAST_ZERO(gyro_data[i]);
        gyro_data[i].which = i;
        gyro_data[i].backoff_sec = min_backoff_sec;

        ph_job_init(&(gyro_data[i].connect_job));
        gyro_data[i].connect_job.callback = dsp1760_connect_gyro;
        gyro_data[i].connect_job.data = &(gyro_data[i]);
//        ph_job_set_pool(&(gyro_data[i].connect_job), gyro_pool);

        // Set the dispatch to 500ms and 1000ms respectively for the gyro connect
        ph_job_set_timer_in_ms(&(gyro_data[i].connect_job), 500 * (i+1));
    }

    blast_startup("Initialized gyroscope interface");
    return true;
}
