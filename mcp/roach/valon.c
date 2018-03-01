/*
 * valon.c
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
 *  Created on: Mar 7, 2016
 *      Author: seth
 *
 *  Based on code that is copyright (C) 2011 Associated Universities, Inc
 *  and released under GPLv2
 *
 */

#if 0

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <remote_serial.h>
#include <valon.h>

enum { ACK  = 0x06,
       NACK = 0x15 };

typedef struct
{
    uint32_t ncount;
    uint32_t frac;
    uint32_t mod;
    uint32_t dbf;
} registers_t;

// Calculate effective phase detector frequency
float getEPDF(remote_serial_t *m_serial, enum Synthesizer synth);

// Checksum
static uint8_t generate_checksum(const uint8_t*, size_t);
static bool verify_checksum(const uint8_t*, size_t, uint8_t) __attribute__((unused));

// Register formatting
static void pack_freq_registers(const registers_t *regs, uint8_t *bytes);
static void unpack_freq_registers(const uint8_t *bytes, registers_t *regs);

static void pack_int(uint32_t num, uint8_t *bytes);
static void pack_short(uint16_t num, uint8_t *bytes);
static uint32_t unpack_int(const uint8_t *bytes);
static uint16_t unpack_short(const uint8_t *bytes);

float
get_frequency(remote_serial_t *m_serial, enum Synthesizer synth)
{
    uint8_t bytes[24];
    uint8_t checksum;
    registers_t regs;

    bytes[0] = 0x80 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 24);
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 24);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(bytes, 24, checksum)) return false;
#endif// VERIFY_CHECKSUM

    float EPDF = getEPDF(m_serial, synth);
    unpack_freq_registers(bytes, &regs);
    return (regs.ncount + (float)(regs.frac) / regs.mod) * EPDF / regs.dbf;
}

bool
set_frequency(remote_serial_t *m_serial, enum Synthesizer synth, float frequency,
                          float chan_spacing)
{
    vco_range_t vcor;
    int32_t dbf = 1;
    get_vco_range(m_serial, synth, &vcor);
    while (((frequency * dbf) <= vcor.min) && (dbf <= 16)) {
        dbf *= 2;
    }
    if (dbf > 16) {
        dbf = 16;
    }
    float vco = frequency * dbf;
    registers_t regs;
    float EPDF = getEPDF(m_serial, synth);
    regs.ncount = (int32_t)(vco / EPDF);
    regs.frac = (int32_t)((vco - regs.ncount * EPDF) / chan_spacing + 0.5);
    regs.mod = (int32_t)(EPDF / chan_spacing + 0.5);
    regs.dbf = dbf;
    // Reduce frac/mod to simplest fraction
    if ((regs.frac != 0) && (regs.mod != 0)) {
        while (!(regs.frac & 1) && !(regs.mod & 1)) {
            regs.frac /= 2;
            regs.mod /= 2;
        }
    } else {
        regs.frac = 0;
        regs.mod = 1;
    }
    // Write values to hardware
    uint8_t bytes[26];
    uint8_t checksum;
    bytes[0] = 0x80 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, &bytes[1], 24);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(&bytes[1], 24, checksum)) return false;
#endif// VERIFY_CHECKSUM
    bytes[0] = 0x00 | synth;
    pack_freq_registers(&regs, &bytes[1]);
    bytes[25] = generate_checksum(bytes, 25);
    remote_serial_write_data(m_serial, bytes, 26);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//---------------------//
// Reference Frequency //
//---------------------//
uint32_t get_reference(remote_serial_t *m_serial)
{
    uint8_t bytes[4];
    uint8_t checksum;
    bytes[0] = 0x81;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 4);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(bytes, 4, checksum)) return false;
#endif// VERIFY_CHECKSUM
    return unpack_int(bytes);
}

bool
set_reference(remote_serial_t *m_serial, uint32_t frequency)
{
    uint8_t bytes[6];
    bytes[0] = 0x01;
    pack_int(frequency, &bytes[1]);
    bytes[5] = generate_checksum(bytes, 5);
    remote_serial_write_data(m_serial, bytes, 6);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//----------//
// RF Level //
//----------//
int32_t
get_rf_level(remote_serial_t *m_serial, enum Synthesizer synth)
{
    uint8_t bytes[24];
    uint8_t checksum;
    bytes[0] = 0x80 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 24);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(bytes, 24, checksum)) return false;
#endif// VERIFY_CHECKSUM

    uint32_t reg4 = unpack_int(&bytes[16]);
    int32_t rfl = (reg4 >> 3) & 0x03;
    return 3 * rfl - 4;
}

bool
set_rf_level(remote_serial_t *m_serial, enum Synthesizer synth, int32_t rf_level)
{
    int32_t rfl = 0;
    switch (rf_level) {
        case -4:
            rfl = 0;
            break;
        case -1:
            rfl = 1;
            break;
        case 2:
            rfl = 2;
            break;
        case 5:
            rfl = 3;
            break;
        default:
            return false;
    }
    uint8_t bytes[26];
    uint8_t checksum;
    bytes[0] = 0x80 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, &bytes[1], 24);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(&bytes[1], 24, checksum)) return;
#endif// VERIFY_CHECKSUM
    uint32_t reg4 = unpack_int(&bytes[17]);
    reg4 &= 0xffffffe7;
    reg4 |= (rfl & 0x03) << 3;

    // Write values to hardware
    bytes[0] = 0x00 | synth;
    pack_int(reg4, &bytes[17]);
    bytes[25] = generate_checksum(bytes, 25);
    remote_serial_write_data(m_serial, bytes, 26);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//---------------------//
// ValonSynth Options //
//---------------------//
bool
get_options(remote_serial_t *m_serial, enum Synthesizer synth, options_t *opts)
{
    uint8_t bytes[24];
    uint8_t checksum;
    bytes[0] = 0x80 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 24);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(bytes, 24, checksum)) return false;
#endif// VERIFY_CHECKSUM

    uint32_t reg2 = unpack_int(&bytes[8]);
    opts->low_spur = ((reg2 >> 30) & 1) & ((reg2 >> 29) & 1);
    opts->double_ref = (reg2 >> 25) & 1;
    opts->half_ref = (reg2 >> 24) & 1;
    opts->r = (reg2 >> 14) & 0x03ff;
    return true;
}

bool
set_options(remote_serial_t *m_serial, enum Synthesizer synth,
                        const options_t *opts)
{
    uint8_t bytes[26];
    uint8_t checksum;
    bytes[0] = 0x80 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, &bytes[1], 24);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(&bytes[1], 24, checksum)) return;
#endif// VERIFY_CHECKSUM
    uint32_t reg2 = unpack_int(&bytes[9]);
    reg2 &= 0x9c003fff;
    reg2 |= (((opts->low_spur & 1) << 30) | ((opts->low_spur & 1) << 29) |
             ((opts->double_ref & 1) << 25) | ((opts->half_ref & 1) << 24) |
             ((opts->r & 0x03ff) << 14));
    // Write values to hardware
    bytes[0] = 0x00 | synth;
    pack_int(reg2, &bytes[9]);
    bytes[25] = generate_checksum(bytes, 25);
    remote_serial_write_data(m_serial, bytes, 26);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//------------------//
// Reference Select //
//------------------//
bool
get_ref_select(remote_serial_t *m_serial)
{
    uint8_t bytes;
    uint8_t checksum;
    bytes = 0x86;
    remote_serial_write_data(m_serial, &bytes, 1);
    remote_serial_read_data(m_serial, &bytes, 1);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(&bytes, 1, checksum)) return false;
#endif// VERIFY_CHECKSUM
    return bytes & 1;
}

bool
set_ref_select(remote_serial_t *m_serial, bool e_not_i)
{
    uint8_t bytes[3];
    bytes[0] = 0x06;
    bytes[1] = e_not_i & 1;
    bytes[2] = generate_checksum(bytes, 2);
    remote_serial_write_data(m_serial, bytes, 3);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//-----------//
// VCO Range //
//-----------//
bool
get_vco_range(remote_serial_t *m_serial, enum Synthesizer synth, vco_range_t *vcor)
{
    uint8_t bytes[4];
    uint8_t checksum;
    bytes[0] = 0x83 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 4);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(bytes, 4, checksum)) return false;
#endif// VERIFY_CHECKSUM
    vcor->min = unpack_short(&bytes[0]);
    vcor->max = unpack_short(&bytes[2]);
    return true;
}

bool
set_vco_range(remote_serial_t *m_serial, enum Synthesizer synth,
                          const vco_range_t *vcor)
{
    uint8_t bytes[6];
    bytes[0] = 0x03 | synth;
    pack_short(vcor->min, &bytes[1]);
    pack_short(vcor->max, &bytes[3]);
    bytes[5] = generate_checksum(bytes, 5);
    remote_serial_write_data(m_serial, bytes, 6);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//------------//
// Phase Lock //
//------------//
bool
get_phase_lock(remote_serial_t *m_serial, enum Synthesizer synth)
{
    uint8_t bytes;
    uint8_t checksum;
    bytes = 0x86 | synth;
    remote_serial_write_data(m_serial, &bytes, 1);
    remote_serial_read_data(m_serial, &bytes, 1);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(&bytes, 1, checksum)) return false;
#endif// VERIFY_CHECKSUM
    int32_t mask;
    // ValonSynth A
    if (synth == A) mask = 0x20;
    // ValonSynth B
    else
        mask = 0x10;
    return bytes & mask;
}

//-------------------//
// ValonSynth Label //
//-------------------//
bool
get_label(remote_serial_t *m_serial, enum Synthesizer synth, char *label)
{
    uint8_t bytes[16];
    uint8_t checksum;
    bytes[0] = 0x82 | synth;
    remote_serial_write_data(m_serial, bytes, 1);
    remote_serial_read_data(m_serial, bytes, 16);
    remote_serial_read_data(m_serial, &checksum, 1);
#ifdef VERIFY_CHECKSUM
    if (!verify_checksum(bytes, 16, checksum)) return false;
#endif// VERIFY_CHECKSUM
    memcpy(label, (char*)bytes, 16);
    return true;
}

bool
set_label(remote_serial_t *m_serial, enum Synthesizer synth, const char *label)
{
    uint8_t bytes[18];
    bytes[0] = 0x02 | synth;
    memcpy(&bytes[1], label, 16);
    bytes[17] = generate_checksum(bytes, 17);
    remote_serial_write_data(m_serial, bytes, 18);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//-------//
// Flash //
//-------//
bool
flash(remote_serial_t *m_serial)
{
    uint8_t bytes[2];
    bytes[0] = 0x40;
    bytes[1] = generate_checksum(bytes, 1);
    remote_serial_write_data(m_serial, bytes, 2);
    remote_serial_read_data(m_serial, bytes, 1);
    return bytes[0] == ACK;
}

//------------------//
// EDPF Calculation //
//------------------//
float
getEPDF(remote_serial_t *m_serial, enum Synthesizer synth)
{
    options_t opts;
    float reference = get_reference(m_serial) / 1e6;
    get_options(m_serial, synth, &opts);

    if (opts.double_ref) reference *= 2.0;
    if (opts.half_ref) reference /= 2.0;
    if (opts.r > 1) reference /= opts.r;
    return reference;
}

//----------//
// Checksum //
//----------//
static uint8_t generate_checksum(const uint8_t *bytes, size_t length)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += bytes[i];
    }
    return (uint8_t)(sum % 256);
}

static bool verify_checksum(const uint8_t *bytes, size_t length, uint8_t checksum)
{
    return (generate_checksum(bytes, length) == checksum);
}

//-------------//
// Bit Packing //
//-------------//
static void pack_freq_registers(const registers_t *regs, uint8_t *bytes)
{
    int32_t dbf = 0;
    switch (regs->dbf) {
        case 1:
            dbf = 0;
            break;
        case 2:
            dbf = 1;
            break;
        case 4:
            dbf = 2;
            break;
        case 8:
            dbf = 3;
            break;
        case 16:
            dbf = 4;
            break;
    }
    uint32_t reg0, reg1, reg4;
    reg0 = unpack_int(&bytes[0]);
    reg1 = unpack_int(&bytes[4]);
    reg4 = unpack_int(&bytes[16]);

    reg0 &= 0x80000007;
    reg0 |= ((regs->ncount & 0xffff) << 15) | ((regs->frac & 0x0fff) << 3);
    reg1 &= 0xffff8007;
    reg1 |= (regs->mod & 0x0fff) << 3;
    reg4 &= 0xff8fffff;
    reg4 |= dbf << 20;
    pack_int(reg0, &bytes[0]);
    pack_int(reg1, &bytes[4]);
    pack_int(reg4, &bytes[16]);
}

static void unpack_freq_registers(const uint8_t *bytes, registers_t *regs)
{
    uint32_t reg0, reg1;
    uint32_t reg4;

    reg0 = unpack_int(&bytes[0]);
    reg1 = unpack_int(&bytes[4]);
    reg4 = unpack_int(&bytes[16]);

    regs->ncount = (reg0 >> 15) & 0xffff;
    regs->frac = (reg0 >> 3) & 0x0fff;
    regs->mod = (reg1 >> 3) & 0x0fff;
    int32_t dbf = (reg4 >> 20) & 0x07;
    switch (dbf) {
        case 0:
            regs->dbf = 1;
            break;
        case 1:
            regs->dbf = 2;
            break;
        case 2:
            regs->dbf = 4;
            break;
        case 3:
            regs->dbf = 8;
            break;
        case 4:
            regs->dbf = 16;
            break;
        default:
            regs->dbf = 1;
    }
}

static void pack_int(uint32_t num, uint8_t *bytes)
{
    bytes[0] = (num >> 24) & 0xff;
    bytes[1] = (num >> 16) & 0xff;
    bytes[2] = (num >> 8) & 0xff;
    bytes[3] = (num) & 0xff;
}

static void pack_short(uint16_t num, uint8_t *bytes)
{
    bytes[0] = (num >> 8) & 0xff;
    bytes[1] = (num) & 0xff;
}

static uint32_t unpack_int(const uint8_t *bytes)
{
    return (((uint32_t)bytes[0]) << 24) + (((uint32_t)bytes[1]) << 16) + (((uint32_t)bytes[2]) << 8) + bytes[3];
}

static uint16_t unpack_short(const uint8_t *bytes)
{
    return (((uint16_t)bytes[0]) << 8) + bytes[1];
}

#endif
