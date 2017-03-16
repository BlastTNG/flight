#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/time.h>

#include "mpsse.h"
#include "blast.h"

#define IFACE_A 0;
#define IFACE_B 1;
#define IFACE_C 2;
#define IFACE_D 3;

void send_biphase_writes();
void send_bitbang_value();

struct mpsse_ctx *ctx; 

int main() 
{
    send_biphase_writes();
}


void send_biphase_writes() {

	const uint16_t vid = 1027;
	const uint16_t pid = 24593;
	const char *serial = NULL;
	const char *description = NULL;
	int channel = IFACE_A;
    int counter = 0;

    // The first open is a hack to make sure the chip is properly closed and reset
	ctx = mpsse_open(&vid, &pid, description, serial, channel);
    usleep(1000);
    mpsse_reset_purge_close(ctx);
    usleep(1000);
   
    // This is now the real open 
	ctx = mpsse_open(&vid, &pid, description, serial, channel);
    usleep(1000);

    // Writing an array of data (0xFFFFFFFF...) through the bipase write functions

    uint16_t *data_to_write = NULL;
    size_t bytes_to_write = 512; 
    int frequency = 100000;
    struct timeval begin, end;


    uint8_t data = 0x00;
    uint8_t dir = 0xFF;  // direction output for all bits

    mpsse_set_data_bits_low_byte(ctx, data, dir);
    mpsse_set_data_bits_high_byte(ctx, data, dir);
    mpsse_flush(ctx);
    usleep(1000);
    mpsse_set_frequency(ctx, frequency);
    mpsse_flush(ctx);
    usleep(1000);
    // mpsse_purge(ctx);
    // usleep(1000);

    data_to_write = malloc(bytes_to_write); 
    if (data_to_write) {
        for (int i = 0; i < ((int) bytes_to_write/2); i++) {
            *(data_to_write+i) = 0xFFFF;
            // *(data_to_write+i) = 0x5555;
        }
    } else {
       mpsse_close(ctx); 
       return;
    }

    while(true) {
        gettimeofday(&begin, NULL);
        mpsse_biphase_write_data(ctx, data_to_write, bytes_to_write);
        mpsse_flush(ctx);
        gettimeofday(&end, NULL);
        if ((counter % 10) == 0) {
            blast_dbg("It took %f second to write %zd bytes", (end.tv_usec - begin.tv_usec)/1000000.0, bytes_to_write*2);
        }
        counter += 1;
        // signal(SIGHUP, close_mpsse);
        // signal(SIGINT, close_mpsse);
        // signal(SIGTERM, close_mpsse);
    } 
}
