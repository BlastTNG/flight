#include <stdio.h>
#include <memory.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>

#include "synclink.h"
#include "blast.h"
#include "crc.h"

int send_biphase_writes();


int main() 
{
    send_biphase_writes();
}


void reverse_bits(const size_t bytes_to_write, const uint16_t *msb_data, uint16_t *lsb_data_out) {

    static const unsigned char BitReverseTable256[] = 
    {
      0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
      0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
      0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
      0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
      0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
      0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
      0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
      0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
      0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
      0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
      0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
      0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
      0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
      0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
      0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
      0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
    };
    uint16_t lsb;
    uint16_t msb;
    for (int i = 0; i < ((int) bytes_to_write/2); i++) {
        msb = *(msb_data+i);
        lsb = (BitReverseTable256[msb & 0xff] << 8) | 
              (BitReverseTable256[(msb >> 8) & 0xff]);
        *(lsb_data_out+i) = lsb;
    }
}

int send_biphase_writes() {

    int fd;
    int rc;
    int sigs;
    MGSL_PARAMS params; 

    int counter = 0;
    struct timeval begin, end;
    uint16_t crc_calculated;
    uint16_t small_counter=0;

    /* Open device */
    fd = open("/dev/ttyMicrogate", O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        blast_err("open error=%d %s", errno, strerror(errno));
        return fd;
    }
    usleep(1000);
    close(fd);
    fd = open("/dev/ttyMicrogate", O_RDWR | O_NONBLOCK, 0);
    usleep(1000);
    rc = ioctl(fd, MGSL_IOCGSTATS, 0);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCGSTATS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    // Disable transmitter
    int enable = 0;
    rc = ioctl(fd, MGSL_IOCRXENABLE, enable);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCRXENABLE) error=%d %s", errno, strerror(errno));
        return rc;
    }
    // Enable transmitter
    enable = 1;
    rc = ioctl(fd, MGSL_IOCRXENABLE, enable);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCRXENABLE) error=%d %s", errno, strerror(errno));
        return rc;
    }

    /* Set parameters */
    rc = ioctl(fd, MGSL_IOCGPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCGPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    params.mode = MGSL_MODE_RAW;
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_BRG + HDLC_FLAG_TXC_BRG;
    //params.encoding = HDLC_ENCODING_BIPHASE_LEVEL;
    params.encoding = HDLC_ENCODING_NRZ;
    params.clock_speed = 1000000;
    params.crc_type = HDLC_CRC_NONE;
    rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    int mode = MGSL_INTERFACE_RS422;
    // mode += MGSL_INTERFACE_MSB_FIRST;
    rc = ioctl(fd, MGSL_IOCSIF, mode);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSIF) error=%d %s", errno, strerror(errno));
        return rc;
    }

    int idlemode;
    rc = ioctl(fd, MGSL_IOCGTXIDLE, &idlemode);
    blast_info("The current idlemode is %04x", idlemode);
    idlemode = HDLC_TXIDLE_CUSTOM_16 + 0xAA;
    rc = ioctl(fd, MGSL_IOCGTXIDLE, idlemode);
    blast_info("The current idlemode is %04x", idlemode);

    // Making an array of data (0xFFFFFFFF...) with sync word and CRC
    uint16_t *data_to_write = NULL;
    uint16_t *lsb_data_to_write = NULL;
    size_t bytes_to_write = 1248; 


    data_to_write = malloc(bytes_to_write); 
    lsb_data_to_write = malloc(bytes_to_write); 
    if (data_to_write) {
        *data_to_write = 0xEB90;
        for (int i = 1; i < ((int) bytes_to_write/2); i++) {
            *(data_to_write+i) = 0xFFFF;
        }
    } else {
       close(fd); 
       return 0;
    }

    int last_word = ((int) bytes_to_write/2) - 1;

    // reverse_bits(bytes_to_write, data_to_write, lsb_data_to_write);

    // Blocking mode for read and writes
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
    // fcntl(fd, F_SETFL, fcntl(fd, F_GETFL));
    // Request to Send and Data Terminal Ready
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIS, &sigs);
    if (rc < 0) {
        blast_err("assert DTR/RTS error = %d %s", errno, strerror(errno));
        return rc;
    }

    for (int j=0; j>-1; j++) {
        data_to_write[2] = small_counter;
        if (j%2 == 0) {
            data_to_write[0] = 0xEB90;
            data_to_write[1] = 0xFFFF;
            crc_calculated = crc16(CRC16_SEED, data_to_write, bytes_to_write-2);
        } else {
            data_to_write[1] = 0xF1FF;
            crc_calculated = crc16(CRC16_SEED, data_to_write, bytes_to_write-2);
            data_to_write[0] = 0x146F;
        }
        *(data_to_write+last_word) = crc_calculated;
        reverse_bits(bytes_to_write, data_to_write, lsb_data_to_write);
        gettimeofday(&begin, NULL);
        rc = write(fd, lsb_data_to_write, bytes_to_write);
        if (rc < 0) {
            blast_err("write error=%d %s. rc=%d. Sleeping 0.05s", errno, strerror(errno), rc);    
            usleep(50000);
            // return rc;
        } else {
            printf("\n");
            for (int k=0; k<((int) bytes_to_write/2); k++) {
                printf("%04x ", data_to_write[k]);
            }
        }
        printf("\nWrote %d bytes", rc);
        small_counter ++;
        rc = tcdrain(fd);
        gettimeofday(&end, NULL);
        if ((counter % 10) == 0) {
            blast_info("The CRC calculated is %04x", crc_calculated);
            blast_dbg("It took %f second to write %zd bytes", (end.tv_usec - begin.tv_usec)/1000000.0, bytes_to_write);
        }
        counter += 1;
        // usleep(10000);
    } 
    // Closing
    // sigs = TIOCM_RTS + TIOCM_DTR;
    // rc = ioctl(fd, TIOCMBIC, &sigs);
    // close(fd);
    return 0;
}
