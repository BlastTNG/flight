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
#include <alloca.h>

#include "ezstep.h"
#include "blast_time.h"
#include "blast.h"
#include "uei.h"

int EZBus_Init(struct ezbus* bus, int channel, const char* name, int chatter)
{
    int i;
    int retval = EZ_ERR_OK;
    if (name[0] != '\0') {
        snprintf(bus->name, EZ_BUS_NAME_LEN, "%s: ", name);
    } else {
        bus->name[0] = 0;
    }
    bus->seized = -1;
    bus->chatter = chatter;
    bus->fd = channel;
    bus->sequence = 0;

    for (i = 0; i <= EZ_BUS_NACT; i++) {
        bus->stepper[i].status = 0;
        bus->stepper[i].name[0] = '\0';
        bus->stepper[i].vel = 0;
        bus->stepper[i].acc = 0;
        bus->stepper[i].ihold = 0;
        bus->stepper[i].imove = 0;
        bus->stepper[i].preamble[0] = '\0';
    }
    return retval;
}

int EZBus_Add(struct ezbus* bus, uint8_t who, const char* name)
{
    if (who >= EZ_BUS_NACT) {
        if (bus->chatter >= EZ_CHAT_ERR)
        blast_err("%sFailed to add stepper \'%c\'\n", bus->name, who);
        return EZ_ERR_BAD_WHO;
    }
    bus->stepper[who].status |= EZ_STEP_USED;
    strncpy(bus->stepper[who].name, name, EZ_BUS_NAME_LEN);
    bus->stepper[who].name[EZ_BUS_NAME_LEN - 1] = '\0';
    return EZ_ERR_OK;
}

int EZBus_Take(struct ezbus* bus, uint8_t who)
{
    if (bus->seized != -1 && bus->seized != who) return EZ_ERR_BUSY;

    if (!EZBus_IsUsable(bus, who)) return EZ_ERR_POLL;

    if (bus->seized != who) {
        if (bus->chatter >= EZ_CHAT_SEIZE)
        blast_info("%sBus seized by %s.\n", bus->name, bus->stepper[who].name);
        bus->seized = who;
    }

    return EZ_ERR_OK;
}

int EZBus_Release(struct ezbus* bus, uint8_t who)
{
    if (bus->seized == who) {
        if (bus->chatter >= EZ_CHAT_SEIZE)
        blast_info("%sBus released by %s.\n", bus->name, bus->stepper[who].name);
        bus->seized = -1;
    }
    return EZ_ERR_OK;
}

int EZBus_IsTaken(struct ezbus* bus, uint8_t who)
{
    if (!EZBus_IsUsable(bus, who)) return EZ_ERR_POLL;
    if (bus->seized == who) return EZ_ERR_OK;
    return EZ_ERR_BUSY;
}

// fills string to with stringified hex values of the string from
static const char* HexDump(const char* from, char* to, int from_len, int to_len)
{
    int i;
    int offset = 0;
    offset = snprintf(to, to_len, "%02x", from[0]);
    for (i = 1; i < from_len && offset < to_len; ++i)
        offset += snprintf(to + i * 3 - 1, to_len - offset, ":%02x", from[i]);
    return to;
}

int EZBus_Send(struct ezbus *bus, uint8_t who, const char* what)
{
    size_t len = strnlen(what, 512) + 5;
    unsigned char chk = 3;
    unsigned char *ptr;
    int retval = EZ_ERR_OK;
    char* buffer = alloca(len);

    if (len > 512) return EZ_ERR_MAX;

    bus->sequence = (bus->sequence + 1) & 3;
    snprintf(buffer, len, "%c%c%c%s%c0", 0x02, who + '1', 0x30 + bus->sequence, what, 0x03);
    for (ptr = (unsigned char *) buffer; *ptr != 0x03; ++ptr)
        chk ^= *ptr;
    buffer[len - 1] = chk;
    if (bus->chatter >= EZ_CHAT_BUS) {
        char hex_buffer[3 * len + 1];
        blast_info("%sRequest=%s (%s)", bus->name, HexDump(buffer, hex_buffer, len, 3 * len + 1), what);
    }

    if (uei_508_write(bus->fd, buffer, len) < 0) {
        if (bus->chatter >= EZ_CHAT_ERR) berror(err, "%sError writing on bus channel %d", bus->name,
                                                bus->fd);
        retval |= EZ_ERR_TTY;
    }

    // Was there a serial error?  If so increment err_count.
    if ((retval & EZ_ERR_MASK) > 0) {
        bus->err_count++;
        if (bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Send: Serial error madness! err_count=%d", bus->err_count);
    } else {
        bus->err_count = 0;
        // if(bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Send: No serial error! Resetting error count to 0.");
    }

    return (bus->error = retval);
}

#define EZ_BUS_RECV_ABORT 3000000 /* state for general parsing abort */

int EZBus_Recv(struct ezbus* bus)
{
    char full_response[EZ_BUS_BUF_LEN];
    char* fullptr = full_response;
    int retval = EZ_ERR_OK;
    int state = 0;

    const char delim = 0x03;
    const char start = 0x02;
    const char turnaround = 0xff;

    struct timespec current;
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC, &current);
    clock_gettime(CLOCK_MONOTONIC, &end);
    timespec_add_ns(&end, NSEC_PER_MSEC / EZ_BUS_TIMEOUT_MSEC);

    while (timespec_compare(&current, &end) < 0) {
        unsigned char checksum = 0;
        int response_len = 0;
        int data_len;

        // Read until the stop byte
        if (!state) {
            response_len = uei_508_read_record(bus->fd, full_response, sizeof(full_response), &delim, 1);
            if (response_len < 4) {
                usleep(10);
                continue;
            }
            state++;
            full_response[response_len] = 0;
            {
                char hex_response[3*response_len + 1];
                HexDump(full_response, hex_response, response_len, sizeof(hex_response));

                blast_dbg("Got EZStep response with %d bytes: %s", response_len, hex_response);
            }
        }

        if (!(fullptr = memchr(full_response, turnaround, response_len))) continue;

        for (int i = (fullptr - full_response) + 1; (i > 0) && (i < response_len); i++) {
            checksum ^= full_response[i];
        }
        // Read the Checksum byte and process buffer if it matches
        if ((uei_508_read_bytes(bus->fd, full_response + response_len, 1) == 1)) {
            if (checksum != full_response[response_len++]) {
                char hex_response[3*response_len + 1];
                HexDump(full_response, hex_response, response_len, sizeof(hex_response));
                blast_dbg("Checksum error in EZStep response:%s", hex_response);
                state = 0;
                continue;
            }

            if (fullptr[1] != start) retval |= EZ_ERR_RESPONSE;
            if (fullptr[2] != 0x30)  retval |= EZ_ERR_RESPONSE;
            if (!(fullptr[3] & EZ_STATUS)) {
                retval |= EZ_ERR_RESPONSE;
            } else {
                retval |= fullptr[3] & (EZ_ERROR | EZ_READY);
            }
            if ((data_len = response_len - (fullptr - full_response) - 4) > 0) {
                memcpy(bus->buffer, fullptr + 4, data_len);
                bus->buffer[data_len] = 0;
                break;
            }
        }
    }

    if (bus->chatter >= EZ_CHAT_BUS) {
        char* hex_buffer;
        size_t len = strlen(full_response);
        hex_buffer = alloca(3 * len + 1);
        HexDump(full_response, hex_buffer, len, 3 * len + 1);
        blast_info("%sResponse=%s (%s) Status=%x\n", bus->name,
                   hex_buffer, bus->buffer, retval & 0xff);
    }

    // Was there a serial error?  If so increment err_count.
    if ((retval & EZ_ERR_MASK) > 0) {
        bus->err_count++;
        if (bus->chatter >= EZ_CHAT_BUS)
        blast_err("EZBus_Recv: Serial error madness! err_count=%d", bus->err_count);
    } else {
        bus->err_count = 0;
    }

    return (bus->error = retval);
}

int EZBus_Comm(struct ezbus* bus, uint8_t who, const char* what)
{
    return EZBus_CommRetry(bus, who, what, EZ_BUS_COMM_RETRIES);
}
int EZBus_CommRetry(struct ezbus* bus, uint8_t who, const char* what, int retries)
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

        if ((retval = EZBus_Recv(bus)) & (EZ_ERR_TIMEOUT | EZ_ERR_OOD)) {
            // communication error
            if (bus->chatter >= EZ_CHAT_ERR)
            blast_warn("%sTimeout waiting for response from %s (%s)\n.", bus->name, bus->stepper[who].name, what);

            EZBus_ForceRepoll(bus, who);
            return retval;
        } else {
            switch (retval & EZ_ERROR) {
                case EZ_SERR_INIT:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: initialisation error (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_BADCMD:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: bad command (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_BADOP:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: bad operand (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_COMM:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: communications error (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_NOINIT:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: not initialied (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_OVER:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: overload (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_NOMOVE:
                    if (bus->chatter >= EZ_CHAT_ERR)
                    blast_warn("%s%s: move not allowed (on %s).\n", bus->name, bus->stepper[who].name, what);
                    break;
                case EZ_SERR_BUSY:
                    if (bus->chatter >= EZ_CHAT_ERR && !overflown) {
                        blast_warn("%s%s: command overflow (on %s).\n", bus->name, bus->stepper[who].name, what);
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
    if ((retval & EZ_RESP_ERR_MASK) > 0) {
        bus->err_count++;
        if (bus->chatter >= EZ_CHAT_BUS) blast_err("EZBus_Comm: Serial error madness! err_count=%d", bus->err_count);
    } else {
        bus->err_count = 0;
    }

    return (bus->error = retval);
}

int EZBus_ReadInt(struct ezbus* bus, uint8_t who, const char* what, int* val)
{
    int retval;
    retval = EZBus_Comm(bus, who, what);
    if (!(retval & EZ_ERR_MASK)) *val = atoi(bus->buffer);

    return retval;
}

int EZBus_ForceRepoll(struct ezbus* bus, uint8_t who)
{
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    bus->stepper[who].status &= ~(EZ_STEP_OK | EZ_STEP_INIT);
    return EZ_ERR_OK;
}

// dummy function to use to perform no initialization when using EZBus_Poll
static int noInit(struct ezbus* bus, uint8_t who)
{
    return 1;
}

int EZBus_Poll(struct ezbus* bus)
{
    return EZBus_PollInit(bus, &noInit);
}

int EZBus_PollInit(struct ezbus* bus, int (*ezinit)(struct ezbus*, uint8_t))
{
    int i, result;
    int retval = EZ_ERR_OK;

    if (bus->chatter >= EZ_CHAT_ACT) {
        blast_info("%sPolling EZStepper Bus.", bus->name);
    }

    for (i = 0; i < EZ_BUS_NACT; i++) {
        if (!(bus->stepper[i].status & EZ_STEP_USED)   // skip if unused
            || ((bus->stepper[i].status & EZ_STEP_OK) && (bus->stepper[i].status & EZ_STEP_INIT)))
        continue;
        EZBus_Send(bus, i, "&");
        if ((result = EZBus_Recv(bus)) & (EZ_ERR_TIMEOUT | EZ_ERR_OOD)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_warn("%sNo response from %s, will repoll later.", bus->name, bus->stepper[i].name);
            bus->stepper[i].status &= ~EZ_STEP_OK;
            retval |= result;	  // include in retval results from Recv
            retval |= EZ_ERR_POLL;
        } else if (!strncmp(bus->buffer, "EZStepper AllMotion", 19)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound EZStepper device %s at address %c (0x%x).\n", bus->name, bus->stepper[i].name, i, i);
            bus->stepper[i].status |= EZ_STEP_OK;
        } else if (!strncmp(bus->buffer, "EZHR17EN AllMotion", 18)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound type 17EN device %s at address %c (0x%x).\n", bus->name, bus->stepper[i].name, i, i);
            bus->stepper[i].status |= EZ_STEP_OK;
        } else if (!strncmp(bus->buffer, "EZHR23 All Motion", 17)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound type 23 device %s at address %c (0x%x).\n", bus->name, bus->stepper[i].name, i, i);
            bus->stepper[i].status |= EZ_STEP_OK;
        } else if (!strncmp(bus->buffer, "EZHR-17 All Motion", 18)) {
            if (bus->chatter >= EZ_CHAT_ACT)
            blast_info("%sFound type 17 device %s at address %c (0x%x). \n", bus->name, bus->stepper[i].name, i, i);
            bus->stepper[i].status |= EZ_STEP_OK;
        } else {
            if (bus->chatter >= EZ_CHAT_ERR)
            blast_warn("%sUnrecognised response from %s, " "will repoll later.\n", bus->name, bus->stepper[i].name);
            bus->stepper[i].status &= ~EZ_STEP_OK;
            retval |= EZ_ERR_POLL;
        }

        if ((bus->stepper[i].status & EZ_STEP_OK) && !(bus->stepper[i].status & EZ_STEP_INIT)) {
            if (ezinit(bus, i)) bus->stepper[i].status |= EZ_STEP_INIT;
        }
    }

    return retval;
}

int EZBus_IsUsable(struct ezbus* bus, uint8_t who)
{
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    return (bus->stepper[who].status & (EZ_STEP_USED | EZ_STEP_OK | EZ_STEP_INIT))
            == (EZ_STEP_USED | EZ_STEP_OK | EZ_STEP_INIT);
}

int EZBus_IsBusy(struct ezbus* bus, uint8_t who)
{
    int retval;
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
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

int EZBus_SetIHold(struct ezbus* bus, uint8_t who, int current)
{
    int change = 0;
    char buf[EZ_BUS_BUF_LEN];
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    if (bus->stepper[who].ihold != current) change = 1;
    bus->stepper[who].ihold = current;

    // when hold current changes, send command to update immediately
    if (change) {
        snprintf(buf, sizeof(buf), "h%dR", current);
        return EZBus_Comm(bus, who, buf);
    }

    return EZ_ERR_OK;
}

int EZBus_SetIMove(struct ezbus* bus, uint8_t who, int current)
{
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    bus->stepper[who].imove = current;
    return EZ_ERR_OK;
}

int EZBus_SetVel(struct ezbus* bus, uint8_t who, int vel)
{
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    bus->stepper[who].vel = vel;
    return EZ_ERR_OK;
}

int EZBus_SetAccel(struct ezbus* bus, uint8_t who, int acc)
{
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    bus->stepper[who].acc = acc;
    return EZ_ERR_OK;
}

int EZBus_SetPreamble(struct ezbus* bus, uint8_t who, const char* preamble)
{
    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    strncpy(bus->stepper[who].preamble, preamble, EZ_BUS_BUF_LEN);
    bus->stepper[who].preamble[EZ_BUS_BUF_LEN - 1] = '\0';

    return EZ_ERR_OK;
}

int EZBus_Stop(struct ezbus* bus, uint8_t who)
{
    return EZBus_Comm(bus, who, "T");
}

char* EZBus_StrComm(struct ezbus* bus, uint8_t who, size_t len, char* buffer, const char* fmt, ...)
{
    va_list argptr;
    int out_len;

    out_len = snprintf(buffer, len, "%sV%dL%dm%dh%d",
                                    bus->stepper[who].preamble,
                                    bus->stepper[who].vel,
                                    bus->stepper[who].acc,
                                    bus->stepper[who].imove,
                                    bus->stepper[who].ihold);


    va_start(argptr, fmt);
    vsnprintf(buffer + out_len, len - out_len, fmt, argptr);
    va_end(argptr);

    return buffer;
}

int EZBus_MoveComm(struct ezbus* bus, uint8_t who, const char* what)
{
    char buf[EZ_BUS_BUF_LEN];

    if (who > EZ_BUS_NACT) {
        blast_err("Invalid EZ Stepper Address %u", who);
        return EZ_ERR_BAD_WHO;
    }
    return EZBus_Comm(bus, who, EZBus_StrComm(bus, who, sizeof(buf), buf, "%sR", what));
}

int EZBus_SetEnc(struct ezbus* bus, uint8_t who, int enc)
{
    char buf[EZ_BUS_BUF_LEN];
    snprintf(buf, EZ_BUS_BUF_LEN, "z%d", enc);
    return EZBus_MoveComm(bus, who, buf);
}

int EZBus_Goto(struct ezbus* bus, uint8_t who, int pos)
{
    char buf[EZ_BUS_BUF_LEN];
    snprintf(buf, EZ_BUS_BUF_LEN, "A%d", pos);
    return EZBus_MoveComm(bus, who, buf);
}

int EZBus_GotoVel(struct ezbus* bus, uint8_t who, int pos, int vel)
{
    EZBus_SetVel(bus, who, vel);
    return EZBus_Goto(bus, who, pos);
}

int EZBus_RelMove(struct ezbus* bus, uint8_t who, int delta)
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

int EZBus_RelMoveVel(struct ezbus* bus, uint8_t who, int delta, int vel)
{
    EZBus_SetVel(bus, who, vel);
    return EZBus_RelMove(bus, who, delta);
}

int EZBus_MoveVel(struct ezbus* bus, uint8_t who, int vel)
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

