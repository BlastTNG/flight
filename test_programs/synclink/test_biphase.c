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


int send_biphase_writes() {

    int fd;
    int rc;
    int sigs;
    MGSL_PARAMS params; 

    int counter = 0;
    struct timeval begin, end;
    uint16_t crc_calculated;

    /* Open device */
    fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK, 0);
    if (fd < 0) {
        blast_err("open error=%d %s", errno, strerror(errno));
        return fd;
    }
    usleep(1000);

    /* Set parameters */
    rc = ioctl(fd, MGSL_IOCGPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCGPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }
    params.mode = MGSL_MODE_RAW;
    params.loopback = 0;
    params.flags = HDLC_FLAG_RXC_BRG + HDLC_FLAG_TXC_BRG;
    params.encoding = HDLC_ENCODING_BIPHASE_LEVEL;
    params.clock_speed = 2000000;
    params.crc_type = HDLC_CRC_NONE;
    rc = ioctl(fd, MGSL_IOCSPARAMS, &params);
    if (rc < 0) {
        blast_err("ioctl(MGSL_IOCSPARAMS) error=%d %s", errno, strerror(errno));
        return rc;
    }

    // Making an array of data (0xFFFFFFFF...) with sync word and CRC
    uint16_t *data_to_write = NULL;
    uint16_t *inverse_data_to_write = NULL;
    size_t bytes_to_write = 1248; 


    data_to_write = malloc(bytes_to_write); 
    if (data_to_write) {
        *data_to_write = 0xEB90;
        for (int i = 1; i < ((int) bytes_to_write/2); i++) {
            *(data_to_write+i) = 0xFFFF;
        }
    } else {
       close(fd); 
       return 0;
    }
    inverse_data_to_write = malloc(bytes_to_write); 
    if (inverse_data_to_write) {
        *inverse_data_to_write = 0x146F;
        for (int i = 1; i < ((int) bytes_to_write/2); i++) {
            *(inverse_data_to_write+i) = 0xFFFF;
        }
    } else {
       close(fd); 
       return 0;
    }

    int last_word = ((int) bytes_to_write/2) - 1;
    crc_calculated = crc16(CRC16_SEED, data_to_write, bytes_to_write-2);
    *(data_to_write+last_word) = crc_calculated; // I know 0xAB40 is the CRC
    *(inverse_data_to_write+last_word) = crc_calculated; // I know 0xAB40 is the CRC

    // Blocking mode for read and writes
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
    // Request to Send and Data Terminal Ready
    sigs = TIOCM_RTS + TIOCM_DTR;
    rc = ioctl(fd, TIOCMBIS, &sigs);
    if (rc < 0) {
        blast_err("assert DTR/RTS error = %d %s", errno, strerror(errno));
        return rc;
    }

    for (int j=0; j>-1; j++) {
        gettimeofday(&begin, NULL);
        if (j%2 == 0) {
            rc = write(fd, data_to_write, bytes_to_write);
        } else {
            rc = write(fd, inverse_data_to_write, bytes_to_write);
        }
        if (rc < 0) {
            blast_err("write error=%d %s", errno, strerror(errno));    
            return rc;
        }
        rc = tcdrain(fd);
        gettimeofday(&end, NULL);
        if ((counter % 10) == 0) {
            blast_info("The CRC calculated is %04x", crc_calculated);
            blast_dbg("It took %f second to write %zd bytes", (end.tv_usec - begin.tv_usec)/1000000.0, bytes_to_write);
        }
        counter += 1;
    } 
    // Closing
    // sigs = TIOCM_RTS + TIOCM_DTR;
    // rc = ioctl(fd, TIOCMBIC, &sigs);
    // close(fd);
    return 0;
}
