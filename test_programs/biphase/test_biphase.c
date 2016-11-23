#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include "mpsse.h"
#include "blast.h"

#define IFACE_A 0;
#define IFACE_B 1;
#define IFACE_C 2;
#define IFACE_D 3;

void send_biphase_writes();
void send_bitbang_value();

int main() 
{
    send_biphase_writes();
}


void send_biphase_writes() {

	struct mpsse_ctx *ctx; 
	const uint16_t vid = 1027;
	const uint16_t pid = 24593;
	const char *serial = NULL;
	const char *description = NULL;
	int channel = IFACE_B;


	ctx = mpsse_open(&vid, &pid, description, serial, channel);

    if (false) {

        // Setting pins high and low manually 

        uint8_t data = 0x00;
        uint8_t data_to_read = 0x00;
        uint8_t dir = 0xFF;  // direction output for all bits
        while(true) { 
            data = 0xFF;    // settings all bits to high
            mpsse_set_data_bits_low_byte(ctx, data, dir);
            mpsse_set_data_bits_high_byte(ctx, data, dir);
            blast_dbg("== Setting data high ==");
            mpsse_read_data_bits_high_byte(ctx, &data_to_read);
            blast_dbg("high pins read: %02x", data_to_read);
            mpsse_read_data_bits_low_byte(ctx, &data_to_read);
            blast_dbg("low pins read: %02x", data_to_read);
            usleep(1000000);
            data = 0x00;   // setting all bits to low
            mpsse_set_data_bits_low_byte(ctx, data, dir);
            mpsse_set_data_bits_high_byte(ctx, data, dir);
            blast_dbg("== Setting data low ==");
            mpsse_read_data_bits_high_byte(ctx, &data_to_read);
            blast_dbg("high pins read: %02x", data_to_read);
            mpsse_read_data_bits_low_byte(ctx, &data_to_read);
            blast_dbg("low pins read: %02x", data_to_read);
            usleep(1000000);
        }

    } else {

        // Writing an array of data (0xABABABABAB...) through the bipase write functions

        uint8_t *data_to_write = NULL;
        uint32_t offset = 0;
        uint32_t bytes_to_write = 512;
        unsigned size_of_frame = 8192; // in bytes
        int frequency = 1000000;
        int i = 0;


        data_to_write = malloc(size_of_frame); 
        if (data_to_write) {
            for (int i = 0; i < size_of_frame; i++) {
                *(data_to_write+i) = 0xAB;
            }
        } else {
           mpsse_close(ctx); 
        }

        //mpsse_rtck_config(ctx, false);
        mpsse_set_frequency(ctx, frequency);

        while(true) {
            //blast_dbg("I am about to call mpsse_biphase_write_data, i=%d", i);
            mpsse_biphase_write_data(ctx, data_to_write, offset, bytes_to_write);
            offset += bytes_to_write;
            i += 1;
            if (offset >= size_of_frame) {
                offset = 0;
                i = 0;
            }
            usleep(4096); //4096 to be exact, because it takes 4096 us to send 512 bytes at 1 Mb/s
        } 
    }
    if (ctx) {
        mpsse_close(ctx);
    }
}
