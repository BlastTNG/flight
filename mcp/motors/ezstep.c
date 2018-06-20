/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2010 University of Toronto
 *
 * This file is part of mcp.
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
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <poll.h>

#include "ezstep.h"
#include "blast.h"
#include "blast_time.h"
#include "phenom/sysutil.h"
#include "phenom/thread.h"
#include "phenom/buffer.h"

// TODO(BLAST-Pol OK): allow multiple-motor values of who more widely

/* 'who' manipulation:
 *
 * iWho: converts a who value to integer index of the ezstep.step array
 *    NB: do not use iWho on multi-stepper who values. Rather use whoLoopMin/Max
 * cWho: ensures that who is a character value accepted by the EZbus
 *    cWho is deprecated: who values should always be the ASCII character
 *
 *  isWhoGroup: returns true if who commands more than one stepper
 *
 * to handle all who values, use the whoLoopMin and whoLoopMax which yield cWho
 * values to loop over (NB: loop should include Max value):
 *  for(c=whoLoopMin(who); c<=whoLoopMax(who); c++) { //do stuff... }
 *
 *  stepName: look up a string name for a given who value (or supply default)
 */
static inline int iWho(char who)
{
    return (who <= EZ_BUS_NACT) ? (who - 1) : (who - 0x31);
}

static inline char cWho(char who)
{
    return (who <= EZ_BUS_NACT) ? (who + 0x30) : who;
}

static inline int isWhoGroup(char who)
{
    return (who > EZ_WHO_S16);
}

static inline char whoLoopMin(char who)
{
    if (who >= EZ_WHO_S1 && who <= EZ_WHO_S16)
        return who;
    else if (who == EZ_WHO_G1_2)
        return EZ_WHO_S1;
    else if (who == EZ_WHO_G3_4)
        return EZ_WHO_S3;
    else if (who == EZ_WHO_G5_6)
        return EZ_WHO_S5;
    else if (who == EZ_WHO_G7_8)
        return EZ_WHO_S7;
    else if (who == EZ_WHO_G9_10)
        return EZ_WHO_S9;
    else if (who == EZ_WHO_G11_12)
        return EZ_WHO_S11;
    else if (who == EZ_WHO_G13_14)
        return EZ_WHO_S13;
    else if (who == EZ_WHO_G15_16)
        return EZ_WHO_S15;
    else if (who == EZ_WHO_G1_4)
        return EZ_WHO_S1;
    else if (who == EZ_WHO_G5_8)
        return EZ_WHO_S5;
    else if (who == EZ_WHO_G9_12)
        return EZ_WHO_S9;
    else if (who == EZ_WHO_G13_16)
        return EZ_WHO_S13;
    else if (who == EZ_WHO_ALL)
        return EZ_WHO_S1;
    else
        return '0'; // invalid who value
}

static inline char whoLoopMax(char who)
{
    if (who >= EZ_WHO_S1 && who <= EZ_WHO_S16)
        return who;
    else if (who == EZ_WHO_G1_2)
        return EZ_WHO_S2;
    else if (who == EZ_WHO_G3_4)
        return EZ_WHO_S4;
    else if (who == EZ_WHO_G5_6)
        return EZ_WHO_S6;
    else if (who == EZ_WHO_G7_8)
        return EZ_WHO_S8;
    else if (who == EZ_WHO_G9_10)
        return EZ_WHO_S10;
    else if (who == EZ_WHO_G11_12)
        return EZ_WHO_S12;
    else if (who == EZ_WHO_G13_14)
        return EZ_WHO_S14;
    else if (who == EZ_WHO_G15_16)
        return EZ_WHO_S16;
    else if (who == EZ_WHO_G1_4)
        return EZ_WHO_S4;
    else if (who == EZ_WHO_G5_8)
        return EZ_WHO_S8;
    else if (who == EZ_WHO_G9_12)
        return EZ_WHO_S12;
    else if (who == EZ_WHO_G13_16)
        return EZ_WHO_S16;
    else if (who == EZ_WHO_ALL)
        return EZ_WHO_S16;
    else
        return '0'; // invalid who value
}
static ph_bufq_t *EZStep_read_buffer = {NULL};

static char hidden_buffer[EZ_BUS_NAME_LEN];
static char* stepName(struct ezbus* bus, char who)
{
    int iwho = iWho(who);
    if (who < EZ_WHO_S1 || who > EZ_WHO_ALL) {	  // invalid who
        snprintf(hidden_buffer, sizeof(hidden_buffer), "Invalid Stepper");
        return hidden_buffer;
    } else if (who == 0x5f) {		  // all steppers
        snprintf(hidden_buffer, sizeof(hidden_buffer), "All Steppers");
        return hidden_buffer;
    } else if (who > 0x50) {		  // groups of 4
        snprintf(hidden_buffer, sizeof(hidden_buffer), "4-Stepper Group \'%c\'", (char) who);
        return hidden_buffer;
    } else if (who > 0x40) {		  // groups of 2
        snprintf(hidden_buffer, sizeof(hidden_buffer), "2-Stepper Group \'%c\'", (char) who);
        return hidden_buffer;
    }
    if (who > 0x30) {			  // single controller
        if (bus->stepper[iwho].name[0] != '\0') {
            return bus->stepper[iwho].name;
        } else {
            snprintf(hidden_buffer, sizeof(hidden_buffer), "Stepper #%c", who);
            return hidden_buffer;
        }
    }
    if (bus->chatter >= EZ_CHAT_ERR)
    blast_err("EZ-Stepper: stepName(): Unexpected Trap!\n");
    snprintf(hidden_buffer, sizeof(hidden_buffer), "Invalid Stepper");
    return hidden_buffer;
}

// set up serial port. Only call from EZBus_Init()
static int ez_setserial(struct ezbus* bus, const char* input_tty)
{
    int fd;
    struct termios term;
    static int err_flag = 0;

    if ((fd = open(input_tty, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
        if (bus->chatter >= EZ_CHAT_ERR && err_flag == 0) berror(err, "%sUnable to open serial port (%s)", bus->name,
                                                                 input_tty);
        err_flag = 1;
        return -1;
    }

    if (tcgetattr(fd, &term)) {
        if (bus->chatter >= EZ_CHAT_ERR && err_flag == 0) berror(err, "%sUnable to get serial device attributes",
                                                                 bus->name);
        err_flag = 1;
        return -1;
    }

    /* Clear Character size; set no stop bits; set one parity bit */
    term.c_cflag &= ~(CSTOPB | CSIZE | PARENB);

    /* Set 8 data bits; set local port; enable receiver */
    term.c_cflag |= (CS8 | CLOCAL | CREAD);

    /* Disable all software flow control */
    term.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* disable output processing (raw output) */
    term.c_oflag &= ~OPOST;

    /* disable input processing (raw input) */
    term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    if (cfsetospeed(&term, B9600)) { /*  <======= SET THE SPEED HERE */
        if (bus->chatter >= EZ_CHAT_ERR && err_flag == 0) berror(err, "%sError setting serial output speed", bus->name);
        err_flag = 1;
        return -1;
    }

    if (cfsetispeed(&term, B9600)) { /*  <======= SET THE SPEED HERE */
        if (bus->chatter >= EZ_CHAT_ERR && err_flag == 0) berror(err, "%sError setting serial input speed", bus->name);
        err_flag = 1;
        return -1;
    }

    if (tcsetattr(fd, TCSANOW, &term)) {
        if (bus->chatter >= EZ_CHAT_ERR && err_flag == 0) berror(err, "%sUnable to set serial attributes", bus->name);
        err_flag = 1;
        return -1;
    }
    err_flag = 0;
    return fd;
}

int EZBus_Init(struct ezbus* bus, const char* tty, const char* name, int chatter)
{
    char i;
    int retval = EZ_ERR_OK;

    if (name[0] != '\0') {
        snprintf(bus->name, EZ_BUS_NAME_LEN, "%s: ", name);
        bus->name[EZ_BUS_NAME_LEN - 1] = '\0';
    } else {
        bus->name[0] = '\0';
    }
    bus->seized = -1;
    bus->chatter = chatter;
    if ((bus->fd = ez_setserial(bus, tty)) < 0) retval |= EZ_ERR_TTY;
    for (i = whoLoopMin(EZ_WHO_ALL); i <= whoLoopMax(EZ_WHO_ALL); ++i) {
        bus->stepper[iWho(i)].status = 0;
        bus->stepper[iWho(i)].name[0] = '\0';
        bus->stepper[iWho(i)].vel = 0;
        bus->stepper[iWho(i)].acc = 0;
        bus->stepper[iWho(i)].ihold = 0;
        bus->stepper[iWho(i)].imove = 0;
        bus->stepper[iWho(i)].preamble[0] = '\0';
    }
    return retval;
}

int EZBus_Add(struct ezbus* bus, char who, const char* name)
{
    int iwho = iWho(who);
    if (iwho >= EZ_BUS_NACT) {
        if (bus->chatter >= EZ_CHAT_ERR)
        blast_err("%sFailed to add stepper \'%c\'\n", bus->name, who);
        return EZ_ERR_BAD_WHO;
    }
    bus->stepper[iwho].status |= EZ_STEP_USED;
    strncpy(bus->stepper[iwho].name, name, EZ_BUS_NAME_LEN);
    bus->stepper[iwho].name[EZ_BUS_NAME_LEN - 1] = '\0';
    return EZ_ERR_OK;
}

int EZBus_Take(struct ezbus* bus, char who)
{
    if (bus->seized != -1 && bus->seized != who) return EZ_ERR_BUSY;

    if (!EZBus_IsUsable(bus, who)) return EZ_ERR_POLL;

    if (bus->seized != who) {
        if (bus->chatter >= EZ_CHAT_SEIZE)
        blast_info("%sBus seized by %s.\n", bus->name, stepName(bus, who));
        bus->seized = who;
    }

    return EZ_ERR_OK;
}

int EZBus_Release(struct ezbus* bus, char who)
{
    if (bus->seized == who) {
        if (bus->chatter >= EZ_CHAT_SEIZE)
        blast_info("%sBus released by %s.\n", bus->name, stepName(bus, who));
        bus->seized = -1;
    }
    return EZ_ERR_OK;
}

int EZBus_IsTaken(struct ezbus* bus, char who)
{
    int c;
    if (!EZBus_IsUsable(bus, who)) return EZ_ERR_POLL;
    if (bus->seized == who) return EZ_ERR_OK;
    for (c = whoLoopMin(who); c <= whoLoopMax(who); c++) {
        if (bus->seized == c) return EZ_ERR_OK;
    }
    return EZ_ERR_BUSY;
}

// fills string to with stringified hex values of the string from
static const char* HexDump(const unsigned char* from, char* to, int from_len, int to_len)
{
    int i;
    int offset = 0;
    offset = snprintf(to, to_len, "%02x", from[0]);
    for (i = 1; i < from_len && offset < to_len; ++i)
        offset += snprintf(to + i * 3 - 1, to_len - offset, ".%02x", from[i]);
    return to;
}

int EZBus_Send(struct ezbus *bus, char who, const char* what)
{
    size_t len = strlen(what) + 5;
    char* buffer = malloc(len);
    unsigned char chk = 3;
    unsigned char *ptr;
    static int sequence = 1;
    char* hex_buffer;
    int retval = EZ_ERR_OK;

    sequence = (sequence + 1) % 7;
    buffer[0] = 0x2;
    buffer[1] = who;
    buffer[2] = 0x30 + sequence;
    snprintf(buffer + 3, len - 3, "%s", what);
    buffer[len - 2] = 0x3;
    for (ptr = (unsigned char *) buffer; *ptr != '\03'; ++ptr)
        chk ^= *ptr;
    buffer[len - 1] = chk;
    if (bus->chatter >= EZ_CHAT_BUS) {
        hex_buffer = malloc(3 * len + 1);
        blast_info("%sRequest=%s (%s)", bus->name, HexDump((unsigned char*)buffer, hex_buffer, len, 3 * len + 1), what);
        free(hex_buffer);
    }
    if (write(bus->fd, buffer, len) < 0) {
        if (bus->chatter >= EZ_CHAT_ERR) berror(err, "%sError writing on bus. File descriptor = %i", bus->name,
                                                bus->fd);
        retval |= EZ_ERR_TTY;
    }

    free(buffer);

    // Was there a serial error?  If so increment err_count.
    if ((retval & EZ_ERR_MASK) > 0) {
        bus->err_count++;
        if (bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Send: Serial error madness! err_count=%i", bus->err_count);
    } else {
        bus->err_count = 0;
        if (bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Send: No serial error! Resetting error count to 0.");
    }

    return (bus->error = retval);
}

#define EZ_BUS_RECV_ABORT 3000000 /* state for general parsing abort */

int EZBus_Read(int m_port, char *m_buf, size_t m_bytes)
{
    ph_buf_t *tmp_buf;

    if ((tmp_buf = ph_bufq_consume_bytes(EZStep_read_buffer, m_bytes))) {
        memcpy(m_buf, ph_buf_mem(tmp_buf), m_bytes);
        ph_buf_delref(tmp_buf);
        return m_bytes;
    } else {
        return 0;
    }
}

// TODO(laura): Consider adding back some of the response string recovery logic from BLASTPol
int EZBus_Recv(struct ezbus* bus)
{
	char full_response[EZ_BUS_BUF_LEN] = {0};
	char* fullptr = full_response;
	char* endptr = full_response;
    int retval = EZ_ERR_OK;
    int state = 0;
	int char_count = 0;
	int delim_found = 0;
	int i = 0;

    char checksum = 0;
    const char delim = 0x03; // End of text character
    const char start = 0x02; // '/' Character
    const char turnaround = 0xff; // Where the new string starts

    struct timespec current;
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &current);
    clock_gettime(CLOCK_MONOTONIC, &end);
    timespec_add_ns(&end, NSEC_PER_MSEC * EZ_BUS_TIMEOUT_MSEC);

//    int fd;
//    fd_set rfds;
//    struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
//    unsigned char byte;
//    unsigned char checksum = 0;
//    char* ptr = bus->buffer;
//    char* fullptr = full_response;
    char* hex_buffer;

    // Check to make sure that the port is open.
    if (bus->fd == -1) {
        if (bus->chatter >= EZ_CHAT_ERR) berror(err, "%sError waiting for input on bus", bus->name);
        retval |= EZ_ERR_TTY;
        return retval;
    }

    // Read until the stop byte and checksum
    while ((timespec_compare(&current, &end) < 0) && !state) {
        int response_len = 0;
        int data_len;
        clock_gettime(CLOCK_MONOTONIC, &current);

    	response_len = read(bus->fd, full_response + char_count, EZ_BUS_BUF_LEN - char_count);
        if (bus->chatter >= EZ_CHAT_BUS) blast_dbg("response_len= %i", response_len);
    	if (response_len < 1) { // We haven't read anything so wait a bit
    	    usleep(10);
    	    continue;
    	}

        if (delim_found) state = 1; // End of response found and CRC read, so this is the last time through the loop.

        if (bus->chatter >= EZ_CHAT_BUS) {
            char hex_response[3*(response_len + char_count) + 1];
            full_response[response_len + char_count] = 0; // Make sure the string read so far ends with 0
            HexDump(full_response, hex_response, response_len + char_count, sizeof(hex_response));

            if (bus->chatter >= EZ_CHAT_BUS) {
                blast_info("Got EZStep response with %d bytes, total string: %s", response_len, hex_response);
                blast_info("Total number of chars read %d", response_len + char_count);
            }
        }


        char_count = char_count + response_len; // Total number of characters read so far.

        // Have we found the turnaround (beginning of the response)?  If not then keep reading.
        if (!(fullptr = memchr(full_response, turnaround, char_count))) continue;
		if (bus->chatter >= EZ_CHAT_BUS) blast_info("Found beginning of response: %x", turnaround);
		// Have we found the end of text character?  If not then keep reading.
        if (!(endptr = memchr(full_response, delim, char_count))) continue;
		if (bus->chatter >= EZ_CHAT_BUS) blast_info("Found end-of-text character: %x", delim);

		delim_found = 1; // Signals that we only want to read one more character (CRC)
		if ((int)(endptr - full_response) != (char_count - 1)) state = 1; // If delim is not last char read, we're done.
        if (!state) continue;

        // If we have finished reading (state = 1) check for errors
		if (bus->chatter >= EZ_CHAT_BUS) blast_info("fullptr[1] = %x, fullptr[2] = %x, fullptr[3] = %x",
		    fullptr[1], fullptr[2], fullptr[3]);
        if (fullptr[1] != start) {
        	retval |= EZ_ERR_RESPONSE;
        	blast_warn("%sStart byte not found in response", bus->name);
        	}
        if (fullptr[2] != 0x30) {
            retval |= EZ_ERR_RESPONSE;
            blast_warn("%sFound misaddressed response", bus->name);
        }
        if (!(fullptr[3] & EZ_STATUS)) {
            retval |= EZ_ERR_RESPONSE;
        } else {
            retval |= fullptr[3] & (EZ_ERROR | EZ_READY);
        }

        // Calculate the checksum
        // Ignore the first byte (turnaround) and last (checksum)
        for (i = 1; i < (char_count-1); i++) {
            checksum ^= full_response[i];
        }
		if (bus->chatter >= EZ_CHAT_BUS)
		    blast_info("Calculated checksum = %x, response checksum = %x ", checksum, full_response[char_count-1]);

        if (checksum != full_response[char_count-1]) {
            if (bus->chatter >= EZ_CHAT_ERR)
                blast_err("%sChecksum error in response (%02x).", bus->name, checksum);
            retval |= EZ_ERR_RESPONSE;
        }

		if (bus->chatter >= EZ_CHAT_BUS)
		    blast_info("data_len = %i", char_count - (fullptr - full_response) - 6);

        if ((data_len = char_count - (fullptr - full_response) - 6) > 0) {
            memcpy(bus->buffer, fullptr + 4, data_len);
            // Terminate the string.
            bus->buffer[data_len] = '\0';
            break;
        }
    }

    if (timespec_compare(&current, &end) > 0) {
        if (bus->chatter >= EZ_CHAT_ERR)
            blast_err("%sSerial read timeout", bus->name);
        retval |= EZ_ERR_TIMEOUT;
    }

    if (bus->chatter >= EZ_CHAT_BUS) {
        size_t len;
        len = strlen(full_response);
        hex_buffer = malloc(3 * len + 1);
        blast_info("%sResponse=%s (%s) Status=%x\n", bus->name,
                   HexDump((unsigned char*) full_response, hex_buffer, len, 3 * len + 1), bus->buffer, retval & 0xff);
        free(hex_buffer);
    }

    // Was there a serial error?  If so increment err_count.
    if ((retval & EZ_ERR_MASK) > 0) {
        bus->err_count++;
        if (bus->chatter >= EZ_CHAT_BUS)
        blast_err("EZBus_Recv: Serial error madness! err_count=%i", bus->err_count);
    } else {
        bus->err_count = 0;
        // if(bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Recv: No serial error! Resetting error count to 0.");
    }

    return (bus->error = retval);
}

int EZBus_Comm(struct ezbus* bus, char who, const char* what)
{
    return EZBus_CommRetry(bus, who, what, EZ_BUS_COMM_RETRIES);
}
int EZBus_CommRetry(struct ezbus* bus, char who, const char* what, int retries)
{
    int ok;
    int retval = EZ_ERR_OK;
    int overflown = 0;
    int retry_count = 0;

    do {
        ok = 1;
        if ((retval = EZBus_Send(bus, who, what)) != EZ_ERR_OK) {
            if (bus->chatter >= EZ_CHAT_ERR) berror(warning, "%sFailed to send command\n", bus->name);
            return retval;
        }

        // commands to stepper groups don't send responses
        if (isWhoGroup(who)) return retval;

        if ((retval = EZBus_Recv(bus)) & (EZ_ERR_TIMEOUT | EZ_ERR_OOD)) {
            // communication error
            if (bus->chatter >= EZ_CHAT_ERR)
            blast_warn("%sTimeout waiting for response from %s (%s)\n.", bus->name, stepName(bus, who), what);

            EZBus_ForceRepoll(bus, who);
            return retval;
        } else {
            switch (retval & EZ_ERROR) {
                case EZ_SERR_INIT:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: initialisation error (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_BADCMD:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: bad command (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_BADOP:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: bad operand (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_COMM:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: communications error (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_NOINIT:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: not initialied (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_OVER:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: overload (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_NOMOVE:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: move not allowed (on %s).\n", bus->name, stepName(bus, who), what);
                    break;
                case EZ_SERR_BUSY:
                    if (bus->chatter >= EZ_CHAT_ERR && !overflown) {
                        blast_warn("%s%s: command overflow (on %s).\n", bus->name, stepName(bus, who), what);
                        overflown = 1;
                    }
                    ok = 0;
                    retry_count++;
                    sleep(1);
                    break;
            }
        }
    } while (!ok && retry_count < retries);

    // Was there a serial error?  If so increment err_count.
    if ((retval & EZ_ERR_MASK) > 0) {
        bus->err_count++;
        if (bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Comm: Serial error madness! err_count=%i", bus->err_count);
    } else {
        bus->err_count = 0;
    }

    return (bus->error = retval);
}

int EZBus_ReadInt(struct ezbus* bus, char who, const char* what, int* val)
{
    int retval;
    retval = EZBus_Comm(bus, who, what);
    if (!(retval & (EZ_ERR_TIMEOUT | EZ_ERR_OOD | EZ_ERR_TTY))) *val = atoi(bus->buffer);

    return retval;
}

int EZBus_ForceRepoll(struct ezbus* bus, char who)
{
    int c;
    for (c = whoLoopMin(who); c <= whoLoopMax(who); c++)
        bus->stepper[iWho(c)].status &= ~(EZ_STEP_OK | EZ_STEP_INIT);
    return EZ_ERR_OK;
}

// dummy function to use to perform no initialization when using EZBus_Poll
static int noInit(struct ezbus* bus, char who)
{
    return 1;
}

int EZBus_Poll(struct ezbus* bus)
{
    return EZBus_PollInit(bus, &noInit);
}

int EZBus_PollInit(struct ezbus* bus, int (*ezinit)(struct ezbus*, char))
{
    int i, result;
    int retval = EZ_ERR_OK;

    if (bus->chatter >= EZ_CHAT_ACT) {
        blast_info("%sPolling EZStepper Bus.", bus->name);
    }

    for (i = whoLoopMin(EZ_WHO_ALL); i <= whoLoopMax(EZ_WHO_ALL); ++i) {
        if (!(bus->stepper[iWho(i)].status & EZ_STEP_USED)   // skip if unused
            || ((bus->stepper[iWho(i)].status & EZ_STEP_OK) && (bus->stepper[iWho(i)].status & EZ_STEP_INIT)))
        continue;
        EZBus_Send(bus, i, "&");
        if ((result = EZBus_Recv(bus)) & (EZ_ERR_TIMEOUT | EZ_ERR_OOD)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_warn("%sNo response from %s, will repoll later.", bus->name, stepName(bus, i));
            bus->stepper[iWho(i)].status &= ~EZ_STEP_OK;
            retval |= result;	  // include in retval results from Recv
            retval |= EZ_ERR_POLL;
        } else if (!strncmp(bus->buffer, "EZStepper AllMotion", 19)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound EZStepper device %s at address %c (0x%x).\n", bus->name, stepName(bus, i), i, i);
            bus->stepper[iWho(i)].status |= EZ_STEP_OK;
        } else if (!strncmp(bus->buffer, "EZHR17EN AllMotion", 18)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound type 17EN device %s at address %c (0x%x).\n", bus->name, stepName(bus, i), i, i);
            bus->stepper[iWho(i)].status |= EZ_STEP_OK;
        } else if (!strncmp(bus->buffer, "EZHR23 All Motion", 17)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound type 23 device %s at address %c (0x%x).\n", bus->name, stepName(bus, i), i, i);
            bus->stepper[iWho(i)].status |= EZ_STEP_OK;
        } else if (!strncmp(bus->buffer, "EZHR-17 All Motion", 18)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound type 17 device %s at address %c (0x%x). \n", bus->name, stepName(bus, i), i, i);
            bus->stepper[iWho(i)].status |= EZ_STEP_OK;
        } else {
            if (bus->chatter >= EZ_CHAT_ERR)
            blast_warn("%sUnrecognised response from %s, " "will repoll later.\n", bus->name, stepName(bus, i));
            bus->stepper[iWho(i)].status &= ~EZ_STEP_OK;
            retval |= EZ_ERR_POLL;
        }

        if ((bus->stepper[iWho(i)].status & EZ_STEP_OK) && !(bus->stepper[iWho(i)].status & EZ_STEP_INIT)) {
            if (ezinit(bus, i)) {
		blast_info("setting EZ_STEP_INIT for stepper %d", iWho(i));
                bus->stepper[iWho(i)].status |= EZ_STEP_INIT;
	    } else {
                bus->stepper[iWho(i)].status &= ~EZ_STEP_INIT;
	    }
            sleep(1); // TODO(BLAST-Pol OK): belongs in library? may prevent many-init
        }
    }

    return retval;
}

int EZBus_IsUsable(struct ezbus* bus, char who)
{
    char i;
    for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i) {
        if ((bus->stepper[iWho(i)].status & (EZ_STEP_USED | EZ_STEP_OK | EZ_STEP_INIT)) != (EZ_STEP_USED | EZ_STEP_OK
                                                                                            | EZ_STEP_INIT)) return 0;
    }
    return 1;
}

int EZBus_IsBusy(struct ezbus* bus, char who)
{
    int retval;
    if (isWhoGroup(who)) return EZ_ERR_BAD_WHO | EZ_READY;
    if (!EZBus_IsUsable(bus, who)) return EZ_ERR_POLL | EZ_READY;

    retval = EZBus_CommRetry(bus, who, "Q", 0);
    if ((retval & ~EZ_READY) != EZ_ERR_OK) {
        // on error return code with busy bit set
        return retval | EZ_READY;
    }
    // otherwise just flip busy bit which usually means the opposite
    return retval ^ EZ_READY;
}

/*******************************************************************************
 * Simple motion commands                                                      *
 ******************************************************************************/

int EZBus_SetIHold(struct ezbus* bus, char who, int current)
{
    char i;
    int change = 0;
    char buf[EZ_BUS_BUF_LEN];
    for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i) {
        if (bus->stepper[iWho(i)].ihold != current) change = 1;
        bus->stepper[iWho(i)].ihold = current;
    }
    // when hold current changes, send command to update immediately
    if (change) {
        snprintf(buf, sizeof(buf), "h%dR", current);
        return EZBus_Comm(bus, who, buf);
    }

    return EZ_ERR_OK;
}

int EZBus_SetIMove(struct ezbus* bus, char who, int current)
{
    char i;
    for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i)
        bus->stepper[iWho(i)].imove = current;
    return EZ_ERR_OK;
}

int EZBus_SetVel(struct ezbus* bus, char who, int vel)
{
    char i;
    for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i)
        bus->stepper[iWho(i)].vel = vel;
    return EZ_ERR_OK;
}

int EZBus_SetAccel(struct ezbus* bus, char who, int acc)
{
    char i;
    for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i)
        bus->stepper[iWho(i)].acc = acc;
    return EZ_ERR_OK;
}

int EZBus_SetPreamble(struct ezbus* bus, char who, const char* preamble)
{
    char i;
    blast_info("EZBus_SetPreamble: = %i, whoLoopMax(who) = %i", whoLoopMin(who), whoLoopMax(who));
    for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i) {
	    blast_info("EZBus_SetPreamble: i = %i, iWho(i) = %i, preamble = %s, EZ_BUS_BUF_LEN = %i",
	               i, iWho(i), preamble, EZ_BUS_BUF_LEN);
        strncpy(bus->stepper[iWho(i)].preamble, preamble, EZ_BUS_BUF_LEN);
        bus->stepper[iWho(i)].preamble[EZ_BUS_BUF_LEN - 1] = '\0';
    }
    return EZ_ERR_OK;
}

int EZBus_Stop(struct ezbus* bus, char who)
{
    return EZBus_Comm(bus, who, "T");
}

char* EZBus_StrComm(struct ezbus* bus, char who, size_t len, char* buffer, const char* fmt, ...)
{
    va_list argptr;
    char* ptr;

    if (isWhoGroup(who)) {
        buffer[0] = '\0'; // do nothing for steppper groups
        return buffer;
    } else {
        snprintf(buffer, len, "%sV%dL%dm%dh%d", bus->stepper[iWho(who)].preamble, bus->stepper[iWho(who)].vel,
                bus->stepper[iWho(who)].acc, bus->stepper[iWho(who)].imove, bus->stepper[iWho(who)].ihold);
    }

    for (ptr = buffer; *ptr != '\0'; ++ptr)
        ;

    va_start(argptr, fmt);
    vsprintf(ptr, fmt, argptr);
    va_end(argptr);

    return buffer;
}

int EZBus_MoveComm(struct ezbus* bus, char who, const char* what)
{
    char buf[EZ_BUS_BUF_LEN];
    char i;
    int retval;
    if (!isWhoGroup(who)) {     // single stepper
        return EZBus_Comm(bus, who, EZBus_StrComm(bus, who, sizeof(buf), buf, "%sR", what));
    } else {
        for (i = whoLoopMin(who); i <= whoLoopMax(who); ++i) {
            retval = EZBus_Comm(bus, i, EZBus_StrComm(bus, i, sizeof(buf), buf, "%s", what));
            if (retval != EZ_ERR_OK) {
                EZBus_Stop(bus, who);
                return retval;
            }
        }
        return EZBus_Comm(bus, who, "R");
    }
}

int EZBus_SetEnc(struct ezbus* bus, char who, int enc)
{
    char buf[EZ_BUS_BUF_LEN];
    snprintf(buf, EZ_BUS_BUF_LEN, "z%d", enc);
    return EZBus_MoveComm(bus, who, buf);
}

int EZBus_Goto(struct ezbus* bus, char who, int pos)
{
    char buf[EZ_BUS_BUF_LEN];
    snprintf(buf, EZ_BUS_BUF_LEN, "A%d", pos);
    return EZBus_MoveComm(bus, who, buf);
}

int EZBus_GotoVel(struct ezbus* bus, char who, int pos, int vel)
{
    EZBus_SetVel(bus, who, vel);
    return EZBus_Goto(bus, who, pos);
}

int EZBus_RelMove(struct ezbus* bus, char who, int delta)
{
    char buf[EZ_BUS_BUF_LEN];
    char comm = 'P';
    if (delta == 0)
        return EZ_ERR_OK;	  // move nowhere
    else if (delta == INT_MAX) delta = 0;	  // move forever
    if (delta < 0) {
        comm = 'D';
        if (delta == INT_MIN)
            delta = 0;	  // move forever
        else
            delta = -delta;
    }
    snprintf(buf, sizeof(buf), "%c%d", comm, delta);
    return EZBus_MoveComm(bus, who, buf);
}

int EZBus_RelMoveVel(struct ezbus* bus, char who, int delta, int vel)
{
    EZBus_SetVel(bus, who, vel);
    return EZBus_RelMove(bus, who, delta);
}

int EZBus_MoveVel(struct ezbus* bus, char who, int vel)
{
    char buf[EZ_BUS_BUF_LEN];
    char comm = 'P';
    int v = vel;
    if (vel < 0) {
        comm = 'D';
        v = -vel;
    }
    EZBus_SetVel(bus, who, v);
    snprintf(buf, sizeof(buf), "%c0", comm);
    return EZBus_MoveComm(bus, who, buf);
}

