/*
 * roach.c
 *
 * This software is copyright (C) 2013-2016 University of Pennsylvania
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
 *  Created on: Apr 5, 2015
 *      Author: seth
 */

#include <complex.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <sys/stat.h>

// N.B. fftw3.h needs to be AFTER complex.h
#include <fftw3.h>

// N.B. "I" is a terrible definition as many headers (OpenSSL!) use it.  We'll expand to _Complex_I
#undef I

#include "katcp.h"
#include "katcl.h"

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#include "blast.h"
#include "crc.h"
#include "roach.h"

static double dac_samp_freq = 512.0e6;
static double fpga_samp_freq = 256.0e6;
static int fft_len = 1024;

const char *qdr_ctl[2] = { "qdr0_ctrl",
                           "qdr1_ctrl" };
const char *qdr_mem[2] = { "qdr0_memory",
                           "qdr1_memory" };
const size_t cal_data_len[6] = { 128, 32, 1024, 1024, 1024, 1024 };
const uint32_t cal_data[6][256] = {
                                { 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555,
                                  0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555,
                                  0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555,
                                  0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555,
                                  0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA, 0x55555555,
                                  0xAAAAAAAA, 0x55555555 },
                                { 0, 0, 0xFFFFFFFF, 0, 0, 0, 0, 0},
                                {    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,
                                    13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,
                                    26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,
                                    39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,
                                    52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,
                                    65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,
                                    78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,
                                    91,  92,  93,  94,  95,  96,  97,  98,  99, 100, 101, 102, 103,
                                   104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
                                   117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
                                   130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142,
                                   143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155,
                                   156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168,
                                   169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
                                   182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194,
                                   195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
                                   208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
                                   221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233,
                                   234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246,
                                   247, 248, 249, 250, 251, 252, 253, 254, 255},
                                {      0,   256,   512,   768,  1024,  1280,  1536,  1792,  2048,
                                    2304,  2560,  2816,  3072,  3328,  3584,  3840,  4096,  4352,
                                    4608,  4864,  5120,  5376,  5632,  5888,  6144,  6400,  6656,
                                    6912,  7168,  7424,  7680,  7936,  8192,  8448,  8704,  8960,
                                    9216,  9472,  9728,  9984, 10240, 10496, 10752, 11008, 11264,
                                   11520, 11776, 12032, 12288, 12544, 12800, 13056, 13312, 13568,
                                   13824, 14080, 14336, 14592, 14848, 15104, 15360, 15616, 15872,
                                   16128, 16384, 16640, 16896, 17152, 17408, 17664, 17920, 18176,
                                   18432, 18688, 18944, 19200, 19456, 19712, 19968, 20224, 20480,
                                   20736, 20992, 21248, 21504, 21760, 22016, 22272, 22528, 22784,
                                   23040, 23296, 23552, 23808, 24064, 24320, 24576, 24832, 25088,
                                   25344, 25600, 25856, 26112, 26368, 26624, 26880, 27136, 27392,
                                   27648, 27904, 28160, 28416, 28672, 28928, 29184, 29440, 29696,
                                   29952, 30208, 30464, 30720, 30976, 31232, 31488, 31744, 32000,
                                   32256, 32512, 32768, 33024, 33280, 33536, 33792, 34048, 34304,
                                   34560, 34816, 35072, 35328, 35584, 35840, 36096, 36352, 36608,
                                   36864, 37120, 37376, 37632, 37888, 38144, 38400, 38656, 38912,
                                   39168, 39424, 39680, 39936, 40192, 40448, 40704, 40960, 41216,
                                   41472, 41728, 41984, 42240, 42496, 42752, 43008, 43264, 43520,
                                   43776, 44032, 44288, 44544, 44800, 45056, 45312, 45568, 45824,
                                   46080, 46336, 46592, 46848, 47104, 47360, 47616, 47872, 48128,
                                   48384, 48640, 48896, 49152, 49408, 49664, 49920, 50176, 50432,
                                   50688, 50944, 51200, 51456, 51712, 51968, 52224, 52480, 52736,
                                   52992, 53248, 53504, 53760, 54016, 54272, 54528, 54784, 55040,
                                   55296, 55552, 55808, 56064, 56320, 56576, 56832, 57088, 57344,
                                   57600, 57856, 58112, 58368, 58624, 58880, 59136, 59392, 59648,
                                   59904, 60160, 60416, 60672, 60928, 61184, 61440, 61696, 61952,
                                   62208, 62464, 62720, 62976, 63232, 63488, 63744, 64000, 64256,
                                   64512, 64768, 65024, 65280},
                                {         0,    65536,   131072,   196608,   262144,   327680,
                                     393216,   458752,   524288,   589824,   655360,   720896,
                                     786432,   851968,   917504,   983040,  1048576,  1114112,
                                    1179648,  1245184,  1310720,  1376256,  1441792,  1507328,
                                    1572864,  1638400,  1703936,  1769472,  1835008,  1900544,
                                    1966080,  2031616,  2097152,  2162688,  2228224,  2293760,
                                    2359296,  2424832,  2490368,  2555904,  2621440,  2686976,
                                    2752512,  2818048,  2883584,  2949120,  3014656,  3080192,
                                    3145728,  3211264,  3276800,  3342336,  3407872,  3473408,
                                    3538944,  3604480,  3670016,  3735552,  3801088,  3866624,
                                    3932160,  3997696,  4063232,  4128768,  4194304,  4259840,
                                    4325376,  4390912,  4456448,  4521984,  4587520,  4653056,
                                    4718592,  4784128,  4849664,  4915200,  4980736,  5046272,
                                    5111808,  5177344,  5242880,  5308416,  5373952,  5439488,
                                    5505024,  5570560,  5636096,  5701632,  5767168,  5832704,
                                    5898240,  5963776,  6029312,  6094848,  6160384,  6225920,
                                    6291456,  6356992,  6422528,  6488064,  6553600,  6619136,
                                    6684672,  6750208,  6815744,  6881280,  6946816,  7012352,
                                    7077888,  7143424,  7208960,  7274496,  7340032,  7405568,
                                    7471104,  7536640,  7602176,  7667712,  7733248,  7798784,
                                    7864320,  7929856,  7995392,  8060928,  8126464,  8192000,
                                    8257536,  8323072,  8388608,  8454144,  8519680,  8585216,
                                    8650752,  8716288,  8781824,  8847360,  8912896,  8978432,
                                    9043968,  9109504,  9175040,  9240576,  9306112,  9371648,
                                    9437184,  9502720,  9568256,  9633792,  9699328,  9764864,
                                    9830400,  9895936,  9961472, 10027008, 10092544, 10158080,
                                   10223616, 10289152, 10354688, 10420224, 10485760, 10551296,
                                   10616832, 10682368, 10747904, 10813440, 10878976, 10944512,
                                   11010048, 11075584, 11141120, 11206656, 11272192, 11337728,
                                   11403264, 11468800, 11534336, 11599872, 11665408, 11730944,
                                   11796480, 11862016, 11927552, 11993088, 12058624, 12124160,
                                   12189696, 12255232, 12320768, 12386304, 12451840, 12517376,
                                   12582912, 12648448, 12713984, 12779520, 12845056, 12910592,
                                   12976128, 13041664, 13107200, 13172736, 13238272, 13303808,
                                   13369344, 13434880, 13500416, 13565952, 13631488, 13697024,
                                   13762560, 13828096, 13893632, 13959168, 14024704, 14090240,
                                   14155776, 14221312, 14286848, 14352384, 14417920, 14483456,
                                   14548992, 14614528, 14680064, 14745600, 14811136, 14876672,
                                   14942208, 15007744, 15073280, 15138816, 15204352, 15269888,
                                   15335424, 15400960, 15466496, 15532032, 15597568, 15663104,
                                   15728640, 15794176, 15859712, 15925248, 15990784, 16056320,
                                   16121856, 16187392, 16252928, 16318464, 16384000, 16449536,
                                   16515072, 16580608, 16646144, 16711680},
                            {               0,   16777216,   33554432,   50331648,   67108864,
                                     83886080,  100663296,  117440512,  134217728,  150994944,
                                    167772160,  184549376,  201326592,  218103808,  234881024,
                                    251658240,  268435456,  285212672,  301989888,  318767104,
                                    335544320,  352321536,  369098752,  385875968,  402653184,
                                    419430400,  436207616,  452984832,  469762048,  486539264,
                                    503316480,  520093696,  536870912,  553648128,  570425344,
                                    587202560,  603979776,  620756992,  637534208,  654311424,
                                    671088640,  687865856,  704643072,  721420288,  738197504,
                                    754974720,  771751936,  788529152,  805306368,  822083584,
                                    838860800,  855638016,  872415232,  889192448,  905969664,
                                    922746880,  939524096,  956301312,  973078528,  989855744,
                                   1006632960, 1023410176, 1040187392, 1056964608, 1073741824,
                                   1090519040, 1107296256, 1124073472, 1140850688, 1157627904,
                                   1174405120, 1191182336, 1207959552, 1224736768, 1241513984,
                                   1258291200, 1275068416, 1291845632, 1308622848, 1325400064,
                                   1342177280, 1358954496, 1375731712, 1392508928, 1409286144,
                                   1426063360, 1442840576, 1459617792, 1476395008, 1493172224,
                                   1509949440, 1526726656, 1543503872, 1560281088, 1577058304,
                                   1593835520, 1610612736, 1627389952, 1644167168, 1660944384,
                                   1677721600, 1694498816, 1711276032, 1728053248, 1744830464,
                                   1761607680, 1778384896, 1795162112, 1811939328, 1828716544,
                                   1845493760, 1862270976, 1879048192, 1895825408, 1912602624,
                                   1929379840, 1946157056, 1962934272, 1979711488, 1996488704,
                                   2013265920, 2030043136, 2046820352, 2063597568, 2080374784,
                                   2097152000, 2113929216, 2130706432, 2147483648, 2164260864,
                                   2181038080, 2197815296, 2214592512, 2231369728, 2248146944,
                                   2264924160, 2281701376, 2298478592, 2315255808, 2332033024,
                                   2348810240, 2365587456, 2382364672, 2399141888, 2415919104,
                                   2432696320, 2449473536, 2466250752, 2483027968, 2499805184,
                                   2516582400, 2533359616, 2550136832, 2566914048, 2583691264,
                                   2600468480, 2617245696, 2634022912, 2650800128, 2667577344,
                                   2684354560, 2701131776, 2717908992, 2734686208, 2751463424,
                                   2768240640, 2785017856, 2801795072, 2818572288, 2835349504,
                                   2852126720, 2868903936, 2885681152, 2902458368, 2919235584,
                                   2936012800, 2952790016, 2969567232, 2986344448, 3003121664,
                                   3019898880, 3036676096, 3053453312, 3070230528, 3087007744,
                                   3103784960, 3120562176, 3137339392, 3154116608, 3170893824,
                                   3187671040, 3204448256, 3221225472, 3238002688, 3254779904,
                                   3271557120, 3288334336, 3305111552, 3321888768, 3338665984,
                                   3355443200, 3372220416, 3388997632, 3405774848, 3422552064,
                                   3439329280, 3456106496, 3472883712, 3489660928, 3506438144,
                                   3523215360, 3539992576, 3556769792, 3573547008, 3590324224,
                                   3607101440, 3623878656, 3640655872, 3657433088, 3674210304,
                                   3690987520, 3707764736, 3724541952, 3741319168, 3758096384,
                                   3774873600, 3791650816, 3808428032, 3825205248, 3841982464,
                                   3858759680, 3875536896, 3892314112, 3909091328, 3925868544,
                                   3942645760, 3959422976, 3976200192, 3992977408, 4009754624,
                                   4026531840, 4043309056, 4060086272, 4076863488, 4093640704,
                                   4110417920, 4127195136, 4143972352, 4160749568, 4177526784,
                                   4194304000, 4211081216, 4227858432, 4244635648, 4261412864,
                                   4278190080}
};

typedef enum {
    ROACH_STATUS_BOOT,
    ROACH_STATUS_CONNECTED,
    ROACH_STATUS_PROGRAMMED,
    ROACH_STATUS_TONE,
    ROACH_STATUS_DDS,
    ROACH_STATUS_STREAMING,
} e_roach_status;

typedef enum {
    ROACH_UPLOAD_RESULT_WORKING = 0,
    ROACH_UPLOAD_RESULT_TIMEOUT,
    ROACH_UPLOAD_RESULT_ERROR,
    ROACH_UPLOAD_RESULT_SUCCESS
} e_roach_upload_result;

typedef struct {
    int32_t I;
    int32_t Q;
} __attribute__((packed)) udp_element_t;

typedef struct {
    udp_element_t data[1024];
    uint32_t cycle_count;
    uint32_t pps_count:24;
    uint32_t pkt_count:8;
} __attribute__((packed)) udp_packet_t;

typedef struct {
    size_t len;
    double *I;
    double *Q;
} roach_lut_t;

typedef struct {
    size_t len;
    uint16_t *I;
    uint16_t *Q;
} roach_uint16_lut_t;

typedef struct {
    int status;
    int has_error;
    const char *last_err;
    const char *address;
    uint16_t port;
    int ms_cmd_timeout;

    double dac_freq_res;
    double *freq_residuals;
    size_t lut_buffer_len;

    roach_lut_t DDS;
    roach_lut_t DAC;
    roach_uint16_lut_t LUT;

    char *vna_path;
    char *channels_path;

    struct katcl_line *rpc_conn;
} roach_state_t;

typedef struct {
    const char *firmware_file;
    uint16_t port;
    struct timeval timeout;
    int result;
    roach_state_t *roach;
    ph_sock_t *sock;
} firmware_state_t;

static ph_thread_t *katcp_thread = NULL;

static inline int roach_get_median_int(const int m_indata[], int m_len)
{
    int tmp_begin, tmp_end;
    int t, x;
    int buf[m_len];

    memcpy(buf, m_indata, sizeof(m_indata[0]) * m_len);
    tmp_begin = 0;
    tmp_end = m_len - 1;
    while (tmp_begin < tmp_end) {
        x = buf[m_len / 2];
        int i = tmp_begin;
        int j = tmp_end;
        do {
            while (buf[i] < x)
                i++;
            while (x < buf[j])
                j--;
            if (i <= j) {
                t = buf[i];
                buf[i] = buf[j];
                buf[j] = t;
                i++;
                j--;
            }
        } while (i <= j);
        if (j < m_len / 2) tmp_begin = i;
        if (m_len / 2 < i) tmp_end = j;
    }
    return buf[m_len / 2];
}

static void roach_buffer_ntohl(uint32_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohl(m_buffer[i]);
    }
}

static void roach_buffer_ntohs(uint16_t *m_buffer, size_t m_len)
{
    for (size_t i = 0; i < m_len; i++) {
        m_buffer[i] = ntohs(m_buffer[i]);
    }
}

static int roach_write_data(roach_state_t *m_roach, const char *m_register, uint8_t *m_data,
                            size_t m_len, uint32_t m_offset)
{
    return send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?write",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_BUFFER, m_data, m_len,
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_len,
                   NULL);
}

static int roach_read_data(roach_state_t *m_roach, uint8_t *m_dest, const char *m_register,
                           uint32_t m_offset, uint32_t m_size)
{
    int retval = send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                   KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?read",
                   KATCP_FLAG_STRING, m_register,
                   KATCP_FLAG_ULONG, m_offset,
                   KATCP_FLAG_ULONG | KATCP_FLAG_LAST, m_size,
                   NULL);

    if (retval < 0) {
        blast_err("Could not read data '%s' from %s: Internal Error", m_register, m_roach->address);
        return -1;
    }
    if (retval > 0) {
        char *ret = arg_string_katcl(m_roach->rpc_conn, 1);
        blast_err("Could not read data '%s' from %s: ROACH Error '%s'", m_register, m_roach->address, ret?ret:"");
        return -1;
    }

    if (arg_count_katcl(m_roach->rpc_conn) < 2) {
        blast_err("Expecting 2 return values.  Recevied %d", arg_count_katcl(m_roach->rpc_conn));
        return -1;
    }

    uint32_t bytes_copied = arg_buffer_katcl(m_roach->rpc_conn, 1, m_dest, m_size);

    if (bytes_copied != m_size) {
        blast_err("Expecting %ul bytes but only received %ul bytes", m_size, bytes_copied);
        return -1;
    }
    return 0;
}

static int roach_write_int(roach_state_t *m_roach, const char *m_register, uint32_t m_val, uint32_t m_offset)
{
    uint32_t sendval = htonl(m_val);
    return roach_write_data(m_roach, m_register, (uint8_t*)&sendval, sizeof(sendval), m_offset);
}




static int roach_reset_dac(roach_state_t *m_roach)
{
    roach_write_int(m_roach, "dac_reset", 1, 0);
    roach_write_int(m_roach, "dac_reset", 0, 0);
    return 0;
}

static int roach_read_mixer_snaps(roach_state_t *m_roach, uint32_t m_shift, uint32_t m_chan,
                                  double *m_mixer_in, double *m_mixer_out)
{
    size_t buffer_len = 8 * (1<<14);
    int16_t *temp_data;

    if (roach_write_int(m_roach, "dds_shift", m_shift, 0)) return -1;
    if (roach_write_int(m_roach, "chan_select", (m_chan & (UINT32_MAX - 1)) >> 1, 0)) return -1;
    if (roach_write_int(m_roach, "rawfftbin_ctrl", 0, 0)) return -1;
    if (roach_write_int(m_roach, "mixerout_ctrl", 0, 0)) return -1;
    if (roach_write_int(m_roach, "rawfftbin_ctrl", 1, 0)) return -1;
    if (roach_write_int(m_roach, "mixerout_ctrl", 1, 0)) return -1;

    temp_data = calloc(sizeof(int16_t), buffer_len);
    if (roach_read_data(m_roach, (uint8_t*)temp_data, "rawfftbin_bram", 0, buffer_len * sizeof(uint16_t))) {
        free(temp_data);
        return -1;
    }
    roach_buffer_ntohs((uint16_t*)temp_data, buffer_len);
    for (size_t i = 0; i < buffer_len; i++) {
        m_mixer_in[i] = temp_data[i] / ((float)(1<<15));
    }

    if (m_mixer_out) {
        if (roach_read_data(m_roach, (uint8_t*)temp_data, "mixerout_bram", 0, buffer_len)) return -1;
        roach_buffer_ntohs((uint16_t*)temp_data, buffer_len / 2);
        for (size_t i = 0; i < buffer_len / 2; i++) {
            m_mixer_out[i] = temp_data[i] / ((double)(1<<14));
        }
    }
    return 0;
}

static void roach_init_LUT(roach_state_t * m_roach, size_t m_len)
{
    m_roach->LUT.len = m_len;
    m_roach->LUT.I = calloc(m_len, sizeof(double));
    m_roach->LUT.Q = calloc(m_len, sizeof(double));
}

static inline int roach_fft_bin_index(double *m_freqs, int m_index, size_t m_fft_len, double m_samp_freq)
{
    return (int)lround(m_freqs[m_index] / m_samp_freq * m_fft_len);
}
static int roach_freq_comb(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen,
                            int m_samp_freq, bool m_random_phase, bool m_DAQ_LUT,
                            double *m_I, double *m_Q)
{
    size_t comb_fft_len;
    fftw_plan comb_plan;

    for (size_t i = 0; i < m_freqlen; i++) {
        m_freqs[i] = round(m_freqs[i] / m_roach->dac_freq_res) * m_roach->dac_freq_res;
    }

    if (m_DAQ_LUT) {
        comb_fft_len = m_roach->lut_buffer_len;
    } else {
        comb_fft_len = m_roach->lut_buffer_len / fft_len;
    }

    complex double *spec = (complex double*)fftw_malloc(comb_fft_len * sizeof(complex double));
    complex double *wave = (complex double*)fftw_malloc(comb_fft_len * sizeof(complex double));
    double max_val = 0.0;


    srand48(time(NULL));
    for (size_t i = 0; i < m_freqlen; i++) {
        spec[roach_fft_bin_index(m_freqs, i, comb_fft_len, m_samp_freq)] =
                cexp(_Complex_I * drand48() * 2.0 * M_PI);
    }
    comb_plan = fftw_plan_dft_1d(comb_fft_len, spec, wave, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(comb_plan);
    fftw_destroy_plan(comb_plan);

    for (size_t i = 0; i < comb_fft_len; i++) {
        if (cabs(spec[i]) > max_val) max_val = cabs(spec[i]);
    }
    for (size_t i = 0; i < comb_fft_len; i++) {
        m_I[i] = creal(wave[i]) / max_val * ((1<<15)-1);
        m_Q[i] = cimag(wave[i]) / max_val * ((1<<15)-1);
    }

    fftw_free(spec);
    fftw_free(wave);
    return 0;
}


static inline size_t roach_get_max_index(complex double *m_vals, size_t m_len)
{
    int retval = 0;
    double max_val = 0.0;
    for (size_t i = 0; i < m_len; i++) {
        if (cabs(m_vals[i]) > max_val) {
            max_val = cabs(m_vals[i]);
            retval = i;
        }
    }
    return retval;
}

static int roach_return_shift(roach_state_t *m_roach, uint32_t m_chan)
{
    int n = 1024;
    int istride = 1024;
    int ostride = 1;
    int retval = 0;
    size_t dds_index = 0;

    complex double *dds_spec = (complex double*)fftw_malloc(n * sizeof(complex double));
    double *mixer_snaps = (double *)fftw_malloc((1<<17) * sizeof(double));
    double *dds_in = (double *)fftw_malloc((1<<14) * sizeof(double));

    fftw_plan shift_plan = fftw_plan_many_dft_r2c(1, &n, 1, m_roach->DDS.I, &n, istride, 0,
                                                dds_spec, &n, ostride, 0, FFTW_ESTIMATE);
    fftw_execute(shift_plan);
    fftw_destroy_plan(shift_plan);

    dds_index = roach_get_max_index(dds_spec, n);

    shift_plan = fftw_plan_dft_r2c_1d(n, dds_in, dds_spec, FFTW_ESTIMATE);
    for (int i = 0; i < 512; i++) {
        roach_read_mixer_snaps(m_roach, i, m_chan, mixer_snaps, NULL);
        for (size_t j = 2, k = 0; j < (1<<14); j += 8, k++) {
            dds_in[k] = (mixer_snaps[j] > INT16_MAX) ? mixer_snaps[j] - UINT16_MAX : mixer_snaps[j];
        }
        fftw_execute(shift_plan);
        if (roach_get_max_index(dds_spec, n) == dds_index) {
            blast_info("Found LUT shift of %d for %s", i, m_roach->address);
            retval = i;
            break;
        }
    }

    fftw_destroy_plan(shift_plan);
    fftw_free(dds_spec);
    fftw_free(mixer_snaps);
    fftw_free(dds_in);

    return retval;
}

static int roach_save_1d(const char *m_filename, void *m_data, size_t m_element_size, size_t m_len)
{
    uint32_t channel_crc;
    FILE *fp;

    channel_crc = crc32(BLAST_MAGIC32, m_data, m_element_size * m_len);
    fp = fopen(m_filename, "w");
    fwrite(&m_len, sizeof(size_t), 1, fp);
    fwrite(m_data, m_element_size, m_len, fp);
    fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);
    return 0;
}

static ssize_t roach_load_1d(const char *m_filename, void **m_data, size_t m_element_size)
{
    size_t len;
    FILE *fp;
    struct stat fp_stat;
    uint32_t channel_crc;

    if (stat(m_filename, &fp_stat)) {
        blast_err("Could not get file data for %s: %s", m_filename, strerror(errno));
        return -1;
    }
    if (!(fp = fopen(m_filename, "r"))) {
        blast_err("Could not open %s for reading: %s", m_filename, strerror(errno));
        return -1;
    }
    if (fread(&len, sizeof(len), 1, fp) != 1) {
        blast_err("Could not read data length from %s: %s", m_filename, strerror(errno));
        fclose(fp);
        return -1;
    }
    if ((len * m_element_size) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
        blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu", m_filename,
                  (len * m_element_size) + sizeof(channel_crc) + sizeof(len), fp_stat.st_size);
        fclose(fp);
        return -1;
    }

    *m_data = calloc(len, m_element_size);
    fread(*m_data, m_element_size, len, fp);
    fread(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);

    if (channel_crc != crc32(BLAST_MAGIC32, *m_data, m_element_size * len)) {
        free(*m_data);
        *m_data = NULL;
        blast_err("Mismatched CRC for '%s'.  File corrupted?", m_filename);
        len = -1;
    }
    return len;
}


static int roach_save_2d(const char *m_filename, size_t m_len, double *m_data)
{
    uint32_t channel_crc;
    FILE *fp;

    channel_crc = crc32(BLAST_MAGIC32, m_data, sizeof(double) * 2 * m_len);
    fp = fopen(m_filename, "w");
    fwrite(&m_len, sizeof(size_t), 1, fp);
    fwrite(m_data, sizeof(double), 2 * m_len, fp);
    fwrite(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);
    return 0;
}

static ssize_t roach_load_2d(const char *m_filename, double **m_data)
{
    size_t len;
    FILE *fp;
    struct stat fp_stat;
    uint32_t channel_crc;

    if (stat(m_filename, &fp_stat)) {
        blast_err("Could not get file data for %s: %s", m_filename, strerror(errno));
        return -1;
    }
    if (!(fp = fopen(m_filename, "r"))) {
        blast_err("Could not open %s for reading: %s", m_filename, strerror(errno));
        return -1;
    }
    if (fread(&len, sizeof(len), 1, fp) != 1) {
        blast_err("Could not read data length from %s: %s", m_filename, strerror(errno));
        fclose(fp);
        return -1;
    }
    if ((2 * len * sizeof(double)) != fp_stat.st_size - (sizeof(channel_crc) + sizeof(len))) {
        blast_err("Invalid file '%s'.  Claimed to have %zu bytes but we only see %zu", m_filename,
                  (size_t)(2 * len * sizeof(double)) + sizeof(channel_crc) + sizeof(len),
                  (size_t)fp_stat.st_size);
        fclose(fp);
        return -1;
    }

    *m_data = calloc(2 * len, sizeof(double));
    fread(*m_data, sizeof(double), 2 * len, fp);
    fread(&channel_crc, sizeof(channel_crc), 1, fp);
    fclose(fp);

    if (channel_crc != crc32(BLAST_MAGIC32, *m_data, sizeof(double) * 2 * len)) {
        free(*m_data);
        *m_data = NULL;
        blast_err("Mismatched CRC for '%s'.  File corrupted?", m_filename);
        len = -1;
    }
    return len;
}

static void roach_caclulate_amps(roach_state_t *m_roach, double **m_mags,
                                 double **m_avgs, double **m_offsets, double **m_amps)
{
    double *tmp_data;
    int *channels;
    ssize_t chan_len;
    ssize_t vna_len;

    double *Is;
    double *Qs;

    if ((vna_len = roach_load_2d(m_roach->vna_path, &tmp_data)) <= 0) {
        blast_err("Could not VNA data from %s", m_roach->vna_path);
        *m_amps = NULL;
        return;
    }

    Is = tmp_data;
    Qs = Is + vna_len;

    if ((chan_len = roach_load_1d(m_roach->channels_path, (void**)&channels, sizeof(int))) <= 0) {
        blast_err("Could not load channels data from %s", m_roach->channels_path);
        free(tmp_data);
        *m_amps = NULL;
        return;
    }

    double *chan_mags = calloc(vna_len, sizeof(double));
    double *chan_avgs = calloc(chan_len, sizeof(double));
    double *offsets = calloc(chan_len, sizeof(double));
    double *amps = calloc(chan_len, sizeof(double));
    int sweep_len = vna_len / chan_len;

    for (int chan = 0; chan < chan_len; chan++) {
        double total_mag = 0.0;
        double total_offset = 0.0;
        for (int offset = 0; offset < sweep_len; offset++) {
            int i = offset + chan * sweep_len;
            chan_mags[i]= (10.0 * log10(sqrt(pow(Is[i], 2.0) + pow(Qs[i], 2.0))));
            total_mag += chan_mags[i];
            if (offset < (sweep_len / 2 - 1) || offset > (sweep_len / 2 + 1)) total_offset += chan_mags[i];
        }
        chan_avgs[chan] = total_mag / sweep_len;
        offsets[chan] = chan_avgs[chan] - total_offset / (sweep_len - 3);
        amps[chan] = 2.0 - sqrt(pow10(offsets[chan]/10.0));
    }

    *m_amps = amps;
    *m_avgs = chan_avgs;
    *m_mags = chan_mags;
    *m_offsets = offsets;
}

static void roach_select_bins(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    int bins[fft_len];;
    double bin_freqs[fft_len];
    int last_bin = -1;
    int ch = 0;

    for (int i = 0; i < m_freqlen; i++) {
        bins[i] = roach_fft_bin_index(m_freqs, i, fft_len, dac_samp_freq);
        bin_freqs[i] = bins[i] * dac_samp_freq / fft_len;
        m_roach->freq_residuals[i] = round((m_freqs[i] - bin_freqs[i]) / m_roach->dac_freq_res)
                                        * m_roach->dac_freq_res;
    }
    for (int i = 0; i < m_freqlen; i++) {
        if (bins[i] == last_bin) continue;
        roach_write_int(m_roach, "bins", bins[i], 0);
        roach_write_int(m_roach, "load_bins", 2 * ch + 1, 0);
        roach_write_int(m_roach, "load_bins", 0, 0);
        ch++;
    }
    /**
     * Fill any remaining of the 1024 channelizer addresses with '0'
     */
    for (int i = ch; i < fft_len; i++) {
        roach_write_int(m_roach, "bins", 0, 0);
        roach_write_int(m_roach, "load_bins", 2 * ch + 1, 0);
        roach_write_int(m_roach, "load_bins", 0, 0);
        ch++;
    }
}

void roach_define_DDS_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_select_bins(m_roach, m_freqs, m_freqlen);

    if (m_roach->DDS.len > 0 && m_roach->DDS.len != m_roach->lut_buffer_len) {
        free(m_roach->DDS.I);
        free(m_roach->DDS.Q);
        m_roach->DDS.len = 0;
    }
    if (m_roach->DDS.len == 0) {
        m_roach->DDS.I = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DDS.Q = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DDS.len = m_roach->lut_buffer_len;
    }

    for (int i = 0; i < fft_len; i++) {
        double I[fft_len];
        double Q[fft_len];
        roach_freq_comb(m_roach, &m_roach->freq_residuals[i], 1,
                        fpga_samp_freq / (fft_len / 2), false, false,
                        I, Q);
        for (int j = i, k = 1; j < fft_len * fft_len; j += fft_len, k++) {
            m_roach->DDS.I[j] = I[k];
            m_roach->DDS.Q[j] = Q[k];
        }
    }
}

static void roach_define_DAC_LUT(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    if (m_roach->DAC.len > 0 && m_roach->DAC.len != m_roach->lut_buffer_len) {
        free(m_roach->DAC.I);
        free(m_roach->DAC.Q);
        m_roach->DAC.len = 0;
    }
    if (m_roach->DAC.len == 0) {
        m_roach->DAC.I = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DAC.Q = calloc(m_roach->lut_buffer_len, sizeof(double));
        m_roach->DAC.len = m_roach->lut_buffer_len;
    }
    roach_freq_comb(m_roach, m_freqs, m_freqlen,
                    dac_samp_freq, false, false,
                    m_roach->DAC.I, m_roach->DAC.Q);
}

void roach_pack_LUTs(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_define_DDS_LUT(m_roach, m_freqs, m_freqlen);
    roach_define_DAC_LUT(m_roach, m_freqs, m_freqlen);

    if (m_roach->LUT.len != 2 * m_roach->lut_buffer_len) {
        if (m_roach->LUT.len) {
            free(m_roach->LUT.I);
            free(m_roach->LUT.Q);
        }
        m_roach->LUT.I = calloc(2 * m_roach->lut_buffer_len, sizeof(uint16_t));
        m_roach->LUT.Q = calloc(2 * m_roach->lut_buffer_len, sizeof(uint16_t));
        m_roach->LUT.len = 2 * m_roach->lut_buffer_len;
    }

    for (size_t i = 0; i < m_roach->lut_buffer_len; i += 2) {
        m_roach->LUT.I[2 * i + 0] = htons(m_roach->DAC.I[i + 1]);
        m_roach->LUT.I[2 * i + 1] = htons(m_roach->DAC.I[i]);
        m_roach->LUT.I[2 * i + 2] = htons(m_roach->DDS.I[i + 1]);
        m_roach->LUT.I[2 * i + 3] = htons(m_roach->DDS.I[i]);
        m_roach->LUT.Q[2 * i + 0] = htons(m_roach->DAC.Q[i + 1]);
        m_roach->LUT.Q[2 * i + 1] = htons(m_roach->DAC.Q[i]);
        m_roach->LUT.Q[2 * i + 2] = htons(m_roach->DDS.Q[i + 1]);
        m_roach->LUT.Q[2 * i + 3] = htons(m_roach->DDS.Q[i]);
    }
}

void roach_write_QDR(roach_state_t *m_roach, double *m_freqs, size_t m_freqlen)
{
    roach_pack_LUTs(m_roach, m_freqs, m_freqlen);
    roach_reset_dac(m_roach);
    roach_write_int(m_roach, "start_dac", 0, 0);
    roach_write_data(m_roach, "qdr0_memory", (uint8_t*)m_roach->LUT.I, m_roach->LUT.len * sizeof(uint16_t), 0);
    roach_write_data(m_roach, "qdr1_memory", (uint8_t*)m_roach->LUT.Q, m_roach->LUT.len * sizeof(uint16_t), 0);
    roach_write_int(m_roach, "start_dac", 1, 0);
}

/**
 * If we have an error, we'll disable the socket and schedule a reconnection attempt.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
static void firmware_upload_process_return(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    firmware_state_t *state = (firmware_state_t*) m_data;

    /**
     * If we have an error, or do not receive data from the Roach in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if (m_why & (PH_IOMASK_ERR)) {
        blast_err("disconnecting from firmware upload at %s due to connection issue", state->roach->address);
        state->result = ROACH_UPLOAD_RESULT_ERROR;
    } else if (m_why & PH_IOMASK_TIME) {
        blast_err("Timeout uploading firmware to %s", state->roach->address);
        state->result = ROACH_UPLOAD_RESULT_TIMEOUT;
    } else if (m_why & PH_IOMASK_WRITE) {
        blast_info("Successfully uploaded firmware to %s", state->roach->address);
        state->result = ROACH_UPLOAD_RESULT_SUCCESS;
    }

    ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
    ph_sock_enable(m_sock, 0);
    ph_sock_free(m_sock);
}

/**
 * Handle a connection callback.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our ROACH firmware upload state variable
 */
static void firmware_upload_connected(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    firmware_state_t *state = (firmware_state_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            blast_err("resolve %s:%d failed %s", state->roach->address, state->port, gai_strerror(m_errcode));

            return;

        case PH_SOCK_CONNECT_ERRNO:
            blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->roach->address, state->port, m_errcode, strerror(m_errcode));

            return;
    }

    blast_info("Connected to ROACH at %s", state->roach->address);

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    m_sock->callback = firmware_upload_process_return;
    m_sock->timeout_duration.tv_sec = 30;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);

    /**
     * We have enabled the socket and now we buffer the firmware file into the network
     */
    {
        size_t number_bytes;
        ph_stream_t *firmware_stm = ph_stm_file_open(state->firmware_file, O_RDONLY, 0);
        if (!ph_stm_copy(firmware_stm, m_sock->stream, PH_STREAM_READ_ALL, NULL, &number_bytes)) {
            blast_err("Error getting data from %s: %s", state->firmware_file,
                      strerror(ph_stm_errno(firmware_stm)));
            ph_sock_shutdown(state->sock, PH_SOCK_SHUT_RDWR);
            ph_sock_enable(state->sock, false);
        } else {
            blast_info("Loading %s with %zu bytes", state->firmware_file, number_bytes);
        }
        ph_stm_close(firmware_stm);
    }
}

int roach_upload_fpg(roach_state_t *m_roach, const char *m_filename)
{
    firmware_state_t state = {
                              .firmware_file = m_filename,
                              .port = (uint16_t) (drand48() * 500.0 + 5000),
                              .timeout.tv_sec = 5,
                              .timeout.tv_usec = 0,
                              .roach = m_roach
    };

    int retval = send_rpc_katcl(m_roach->rpc_conn, m_roach->ms_cmd_timeout,
                       KATCP_FLAG_FIRST | KATCP_FLAG_STRING, "?progremote",
                       KATCP_FLAG_ULONG | KATCP_FLAG_LAST, state.port,
                       NULL);
    if (retval != KATCP_RESULT_OK) {
        blast_err("Could not request upload port for ROACH firmware on %s!", m_roach->address);
        return -1;
    }
    ph_sock_resolve_and_connect(state.roach->address, state.port,
        &state.timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, firmware_upload_connected, &state);

    while (state.result == ROACH_UPLOAD_RESULT_WORKING) {
        usleep(1000);
    }

    if (state.result != ROACH_UPLOAD_RESULT_SUCCESS) return -1;

    return 0;
}
int init_roach(void)
{
    return 0;
}

static void qdr_reset(roach_state_t *m_roach, int m_whichqdr)
{
    roach_write_int(m_roach, qdr_ctl[m_whichqdr], 1, 0);
    roach_write_int(m_roach, qdr_ctl[m_whichqdr], 0, 0);
}

static void qdr_delay_out_step(roach_state_t *m_roach, int m_whichqdr, uint64_t m_bitmask, int m_step)
{
    if (!m_step) return;

    roach_write_int(m_roach, qdr_ctl[m_whichqdr], (m_step < 0) ? 0 : UINT32_MAX, 7);

    for (int i = 0; i < abs(m_step); i++) {
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], 0, 6);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], 0, 5);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], (UINT32_MAX & m_bitmask), 6);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], ((0xf) & (m_bitmask >> 32)) << 4, 5);
    }
}

static void qdr_delay_clk_step(roach_state_t *m_roach, int m_whichqdr, int m_step)
{
    if (!m_step) return;

    roach_write_int(m_roach, qdr_ctl[m_whichqdr], (m_step < 0) ? 0 : UINT32_MAX, 7);

    for (int i = 0; i < abs(m_step); i++) {
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], 0, 5);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], (1 << 8) << 4, 5);
    }
}

static void qdr_delay_in_step(roach_state_t *m_roach, int m_whichqdr, uint64_t m_bitmask, int m_step)
{
    if (!m_step) return;

    roach_write_int(m_roach, qdr_ctl[m_whichqdr], (m_step < 0) ? 0 : UINT32_MAX, 7);

    for (int i = 0; i < abs(m_step); i++) {
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], 0, 4);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], 0, 5);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], (UINT32_MAX & m_bitmask), 4);
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], ((0xf) & (m_bitmask >> 32)) << 4, 5);
    }
}

static uint32_t qdr_delay_clk_get(roach_state_t *m_roach, int m_whichqdr)
{
    uint32_t data;
    roach_read_data(m_roach, (uint8_t*)&data, qdr_ctl[m_whichqdr], 8, sizeof(data));
    data = ntohl(data);
    if ((data & 0x1f) != ((data & 0x3e0) >> 5)) {
        blast_err("Invalid counter values in %s of %s", qdr_ctl[m_whichqdr], m_roach->address);
        return 0;
    }
    return data & 0x1f;
}


/**
 * Returns TRUE if ALL of the bits in the check patter are correct
 * @param m_roach Pointer to the roach
 * @param m_whichqdr (0|1) designates which QDR we are accessing
 * @return true if all bits pass validation, false otherwise
 */
static bool qdr_cal_check(roach_state_t *m_roach, int m_whichqdr)
{
    uint32_t check_data[256];

    for (int i = 0; i < 6; i++) {
        roach_write_data(m_roach, qdr_mem[m_whichqdr], (uint8_t*)cal_data[i], cal_data_len[i], 1 << 22);
        roach_read_data(m_roach, (uint8_t*)check_data, qdr_mem[m_whichqdr], 1 << 22, cal_data_len[i]);
        for (size_t j = 0; j < cal_data_len[i]; j++) {
            if (check_data[j] != cal_data[i][j]) {
                blast_err("Calibration check failed for %s on %s", qdr_mem[m_whichqdr], m_roach->address);
                return false;
            }
        }
    }
    return true;
}

/**
 * Returns TRUE if ANY of the bits in the check patter are correct
 * @param m_roach Pointer to the roach
 * @param m_whichqdr (0|1) designates which QDR we are accessing
 * @return true if there is at least 1 good bit, false otherwise
 */
static bool qdr_cal_check_any_good(roach_state_t *m_roach, int m_whichqdr)
{
    uint32_t check_data[256];
    uint32_t fail_pat = 0;

    for (int i = 0; i < 6; i++) {
        roach_write_data(m_roach, qdr_mem[m_whichqdr], (uint8_t*)cal_data[i], cal_data_len[i], 1 << 22);
        roach_read_data(m_roach, (uint8_t*)check_data, qdr_mem[m_whichqdr], 1 << 22, cal_data_len[i]);
        for (size_t j = 0; j < cal_data_len[i]; j++) {
            fail_pat |= (check_data[j] ^ cal_data[i][j]);
        }
    }
    return (fail_pat != UINT32_MAX);
}

static void qdr_apply_cals(roach_state_t *m_roach, int m_whichqdr, int m_indelays[36],
                          int m_outdelays[36], int m_clkdelay)
{
    int max_delay = 0;
    size_t lendelay = sizeof(m_indelays) / sizeof(m_indelays[0]);
    qdr_delay_clk_step(m_roach, m_whichqdr, m_clkdelay);

    for (int i = 0; i < lendelay; i++) if (max_delay < m_indelays[i]) max_delay = m_indelays[i];

    for (int step = 0; step < max_delay; step++) {
        uint64_t mask = 0;
        for (int bit = 0; bit < lendelay; bit++) {
            if (step < m_indelays[bit]) mask |= (1 << bit);
        }
        qdr_delay_in_step(m_roach, m_whichqdr, mask, 1);
    }

    max_delay = 0;
    for (int i = 0; i < lendelay; i++) if (max_delay < m_outdelays[i]) max_delay = m_outdelays[i];

    for (int step = 0; step < max_delay; step++) {
        uint64_t mask = 0;
        for (int bit = 0; bit < lendelay; bit++) {
            if (step < m_outdelays[bit]) mask |= (1 << bit);
        }
        qdr_delay_out_step(m_roach, m_whichqdr, mask, 1);
    }
}

static void qdr_find_cal_area(uint32_t m_bitcal, int *m_max, int *m_begin, int *m_end)
{
    int max_so_far = ((m_bitcal & 0b1) << 1) - 1;
    int max_ending_here = max_so_far;
    int begin_index = 0;
    int begin_temp = 0;
    int end_index = 0;

    for (int i = 1; i < 32; i++) {
        if (max_ending_here < 0) {
            max_ending_here = ((m_bitcal & (1 << i)) >> (i - 1)) - 1;
            begin_temp = i;
        } else {
            max_ending_here += ((m_bitcal & (1 << i)) >> (i - 1)) - 1;
        }
        if (max_ending_here > max_so_far) {
            max_so_far = max_ending_here;
            begin_index = begin_temp;
            end_index = i;
        }
    }
    *m_max = max_so_far;
    *m_begin = begin_index;
    *m_end = end_index;
}

static bool qdr_find_in_delays(roach_state_t *m_roach, int m_whichqdr, int m_indelays[32])
{
    int n_steps = 32;
    int n_bits = 32;
    uint32_t bitcal[n_bits];

    memset(bitcal, 0, sizeof(bitcal));
    for (int step = 0; step < n_steps; step++) {
        uint32_t check_data[256];
        uint32_t fail_pat = 0;

        for (int i = 0; i < 6; i++) {
            roach_write_data(m_roach, qdr_mem[m_whichqdr], (uint8_t*)cal_data[i], cal_data_len[i], 1 << 22);
            roach_read_data(m_roach, (uint8_t*)check_data, qdr_mem[m_whichqdr], 1 << 22, cal_data_len[i]);
            for (size_t j = 0; j < cal_data_len[i]; j++) {
                fail_pat |= (check_data[j] ^ cal_data[i][j]);
            }
        }

        for (int bit = 0; bit < n_bits; bit++) {
            bitcal[bit] |= (((fail_pat >> bit) & 1) << step);
        }
        qdr_delay_in_step(m_roach, m_whichqdr, UINT32_MAX, 1);
    }

    for (int bit = 0; bit < n_bits; bit++) {
        if (bitcal[bit] == UINT32_MAX) {
            blast_err("Calibration failed for bit %d", bit);
            return false;
        }
    }

    for (int bit = 0; bit < n_bits; bit++) {
        int max, begin, end;
        qdr_find_cal_area(bitcal[bit], &max, &begin, &end);
        if (max < 4) {
            blast_err("Could not find robust calibration setting for QDR %d on %s", m_whichqdr, m_roach->address);
            return false;
        }
        m_indelays[bit] = (begin + end) / 2;
    }

    int median_taps = roach_get_median_int(m_indelays, n_bits);
    m_indelays[32] = median_taps;
    m_indelays[33] = median_taps;
    m_indelays[34] = median_taps;
    m_indelays[35] = median_taps;

    return true;
}

static bool qdr_cal2(roach_state_t *m_roach, int m_whichqdr)
{
    int out_step = 0;
    int in_delays[36] = {0};
    int out_delays[36];

    for (out_step = 0; out_step < 36; out_step++) {
        for (int i = 0; i < 36; i++) out_delays[i] = out_step;

        qdr_apply_cals(m_roach, m_whichqdr, in_delays, out_delays, out_step);
        if (qdr_cal_check_any_good(m_roach, m_whichqdr)) break;
    }
    qdr_apply_cals(m_roach, m_whichqdr, in_delays, out_delays, out_step);

    if (!qdr_find_in_delays(m_roach, m_whichqdr, in_delays)) {
        memset(in_delays, 0, sizeof(in_delays));
    }
    blast_info("%s: Using in_delays [\n\t\t%d %d %d %d %d %d %d %d "
                                    "\n\t\t%d %d %d %d %d %d %d %d "
                                    "\n\t\t%d %d %d %d %d %d %d %d "
                                    "\n\t\t%d %d %d %d %d %d %d %d]", m_roach->address,
                                    in_delays[0], in_delays[1], in_delays[2], in_delays[3],
                                    in_delays[4], in_delays[5], in_delays[6], in_delays[7],
                                    in_delays[8], in_delays[9], in_delays[10], in_delays[11],
                                    in_delays[12], in_delays[13], in_delays[14], in_delays[15],
                                    in_delays[16], in_delays[17], in_delays[18], in_delays[19],
                                    in_delays[20], in_delays[21], in_delays[22], in_delays[23],
                                    in_delays[24], in_delays[25], in_delays[26], in_delays[27],
                                    in_delays[28], in_delays[29], in_delays[30], in_delays[31]);


    qdr_apply_cals(m_roach, m_whichqdr, in_delays, out_delays, out_step);

    if (qdr_cal_check(m_roach, m_whichqdr)) return true;

    blast_err("QDR %s failed for %s", qdr_ctl[m_whichqdr], m_roach->address);
    return false;
}
