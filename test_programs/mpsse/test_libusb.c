#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/time.h>
#include <libusb-1.0/libusb.h>

#include "mpsse.h"
#include "blast.h"
#include "crc.h"

#define IFACE_A 0;
#define IFACE_B 1;
#define IFACE_C 2;
#define IFACE_D 3;

void send_biphase_writes();
void send_bitbang_value();

struct mpsse_ctx *ctx; 

static LIBUSB_CALL void demo_cb(struct libusb_transfer * write_transfer) {

    printf("I am demo_cb, about to resubmit transfer\n");
    libusb_submit_transfer(write_transfer);

}

int main(int argc, char *argv[]) {

	const uint16_t vid = 1027;
	const uint16_t pid = 24593;
	const char *serial = NULL;
	const char *description = NULL;
	int channel = IFACE_A;
    int counter = 0;
    int frequency = 0;
    uint16_t frame_counter = 0;


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
    uint16_t *inverse_data_to_write = NULL;
    size_t bytes_to_write = 1248+3; 
    if (argc == 2) {
        frequency = atoi(argv[1]);
    } else {
        frequency = 1000000;
    }
    struct timeval begin, end;

    printf("The clock is set at %d bps\n", frequency);


    uint8_t data = 0x00;
    uint8_t dir = 0xFF;  // direction output for all bits
    // uint8_t dir = 0xBF;  // 0b10111111 pin 6 = input/read
    uint16_t crc_calculated;

    mpsse_set_data_bits_low_byte(ctx, data, dir);
    mpsse_set_data_bits_high_byte(ctx, data, dir);
    mpsse_flush(ctx);
    usleep(1000);
    mpsse_set_frequency(ctx, frequency);
    mpsse_flush(ctx);
    usleep(1000);

    // libubs stuff
    struct libusb_transfer *write_transfer = NULL;
    write_transfer = libusb_alloc_transfer(0);

    data_to_write = malloc(bytes_to_write); 
    if (data_to_write) {
        *data_to_write = 0xDF11;
        *(data_to_write+1) = 0xEB04;
        *(data_to_write+2) = 0x0090;
        for (int i = 3; i < ((int) bytes_to_write/2); i++) {
            *(data_to_write+i) = 0xFFFF;
            // *(data_to_write+i) = i;
        }
    } else {
       mpsse_close(ctx); 
       return 0;
    }
    inverse_data_to_write = malloc(bytes_to_write); 
    if (inverse_data_to_write) {
        *inverse_data_to_write = 0xdf11;
        *(inverse_data_to_write+1) = 0x1404;
        *(inverse_data_to_write+2) = 0x006F;
        for (int i = 3; i < ((int) bytes_to_write/2); i++) {
            *(inverse_data_to_write+i) = 0xFFFF;
            // *(inverse_data_to_write+i) = i;
        }
    } else {
       mpsse_close(ctx); 
       return 0;
    }

    libusb_fill_bulk_transfer(write_transfer, ctx->usb_dev, ctx->out_ep, (void *) data_to_write, bytes_to_write, demo_cb, write_transfer, 2000);
    libusb_submit_transfer(write_transfer);

    int last_word = ((int) bytes_to_write/2) - 1;
    int retval = 0;

    for (int j=0; j>-1; j++) {
        data_to_write[10] = frame_counter;
        gettimeofday(&begin, NULL);
        if (j%2 == 0) {
            crc_calculated = crc16(CRC16_SEED, data_to_write, bytes_to_write-2);
            *(data_to_write+last_word) = crc_calculated; // I know 0xAB40 is the CRC if no counter in the frame
            write_transfer->buffer = (void *) data_to_write;
            blast_info("==== Array to send ====");
            for (int i = 0; i < ((int) bytes_to_write); i++) {
                printf("%02x ", *(((uint8_t *) (data_to_write))+i));
            }
        } else {
            inverse_data_to_write[10] = frame_counter;
            crc_calculated = crc16(CRC16_SEED, data_to_write, bytes_to_write-2);
            *(inverse_data_to_write+last_word) = crc_calculated; // I know 0xAB40 is the CRC if no counter in the frame
            write_transfer->buffer = (void *) inverse_data_to_write;
            blast_info("==== Array to send ====");
            for (int i = 0; i < ((int) bytes_to_write); i++) {
                printf("%02x ", *(((uint8_t *) (inverse_data_to_write))+i));
            }
        }
        retval = libusb_handle_events(ctx->usb_ctx);
        printf("Handle events returned: %s\n", libusb_strerror(retval));
        gettimeofday(&end, NULL);
        if ((counter % 1) == 0) {
            blast_info("The CRC calculated is %04x", crc_calculated);
            blast_dbg("It took %f second to write %zd bytes", (end.tv_sec+(end.tv_usec/1000000.0) - (begin.tv_sec+(begin.tv_usec/1000000.0))), bytes_to_write);
        }
        counter += 1;
        frame_counter++;
    } 
    // libusb_free_transfer(write_transfer);
}
