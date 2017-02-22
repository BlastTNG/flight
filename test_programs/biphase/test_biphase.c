#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/time.h>

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
	//int channel = IFACE_B;
	int channel = IFACE_A;


	ctx = mpsse_open(&vid, &pid, description, serial, channel);

    if (false) {

        // Setting pins high and low manually 

        uint8_t data = 0x00;
        uint8_t data_to_read = 0x00;
        //uint8_t dir = 0xFF;  // direction output for all bits
        uint8_t dir = 0x00;  // direction output for all bits
        while(true) { 

            blast_dbg("== - Begin flushing - ==");
            mpsse_flush(ctx);
            blast_dbg("== - Done flushing - ==");
            data = 0xFF;    // settings all bits to high
            blast_dbg("== Setting data high ==");
            mpsse_set_data_bits_low_byte(ctx, data, dir);
            mpsse_set_data_bits_high_byte(ctx, data, dir);
            blast_dbg("== - Begin flushing - ==");
            mpsse_flush(ctx);
            blast_dbg("== - Done flushing - ==");
            mpsse_read_data_bits_high_byte(ctx, &data_to_read);
            blast_dbg("high pins read: %02x", data_to_read);
            mpsse_read_data_bits_low_byte(ctx, &data_to_read);
            blast_dbg("low pins read: %02x", data_to_read);
            usleep(1000000);
            data = 0x00;   // setting all bits to low
            blast_dbg("== Setting data low ==");
            mpsse_set_data_bits_low_byte(ctx, data, dir);
            mpsse_set_data_bits_high_byte(ctx, data, dir);
            blast_dbg("== - Begin flushing - ==");
            mpsse_flush(ctx);
            blast_dbg("== - Done flushing - ==");
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
        //uint32_t bytes_to_write = 1535; //
        uint32_t bytes_to_write = 625; // That is the correct rate for code running at 100 Hz to write at 1 Mbps
        //uint32_t bytes_to_write = 32; 
        //uint32_t bytes_to_write = 16000; //
        unsigned size_of_frame = bytes_to_write; //8192; // in bytes
        int frequency = 1000000;
        int i = 0;
        struct timeval begin, end;


        uint8_t data = 0x00;
        uint8_t dir = 0xFF;  // direction output for all bits
        //uint8_t dir = 0x00;  // direction input for all bits

        mpsse_set_data_bits_low_byte(ctx, data, dir);
        mpsse_set_data_bits_high_byte(ctx, data, dir);
        data_to_write = malloc(size_of_frame); 
        if (data_to_write) {
            for (int i = 0; i < size_of_frame; i++) {
                *(data_to_write+i) = 0xF0;
            }
        } else {
           mpsse_close(ctx); 
        }

        //mpsse_rtck_config(ctx, false);
        mpsse_set_frequency(ctx, frequency);
        mpsse_flush(ctx);
        usleep(1000);

        //while(i <= 1) {
        while(true) {
            gettimeofday(&begin, NULL);
            blast_dbg("I am about to call mpsse_biphase_write_data, i=%d", i);
            mpsse_biphase_write_data(ctx, data_to_write, offset, bytes_to_write);
            //mpsse_clock_data_out(ctx, data_to_write, offset, bytes_to_write*8, NEG_EDGE_OUT | MSB_FIRST);
            blast_dbg("Flushing the buffer");
            mpsse_flush(ctx);
            i += 1;
            //offset += bytes_to_write;
            //if (offset >= size_of_frame) {
            //    offset = 0;
            //    i = 0;
            //}
            //usleep(4096); //4096 to be exact, because it takes 4096 us to send 512 bytes at 1 Mb/s
            //usleep(10000); // should write 1.25 KB per 0.01 second at 1 MBps
            gettimeofday(&end, NULL);
            blast_dbg("It took %f second to run", (end.tv_usec - begin.tv_usec)/1000000.0);
        } 
    }
    if (ctx) {
        mpsse_close(ctx);
    }
}
