/*
 * qdr.c
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
 *  Created on: May 26, 2016
 *      Author: seth
 */

#include <netinet/in.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "blast.h"
#include "roach.h"
#define MS_TIMEOUT 20000

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

static inline int qdr_get_median_int(const int m_indata[], int m_len)
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

void qdr_reset(roach_state_t *m_roach, int m_whichqdr)
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
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], (1 << 8), 5);
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
        roach_write_int(m_roach, qdr_ctl[m_whichqdr], (0xf) & (m_bitmask >> 32), 5);
    }
}

uint32_t qdr_delay_clk_get(roach_state_t *m_roach, int m_whichqdr)
{
    uint32_t data;
    roach_read_data(m_roach, (uint8_t*)&data, qdr_ctl[m_whichqdr], 8, sizeof(data), MS_TIMEOUT);
    data = ntohl(data);
    if ((data & 0x1f) != ((data & 0x3e0) >> 5)) {
        blast_err("Invalid counter values in %s of %s", qdr_ctl[m_whichqdr], roach_get_name(m_roach));
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
        roach_write_data(m_roach, qdr_mem[m_whichqdr], (uint8_t*)cal_data[i], cal_data_len[i], 1 << 22, MS_TIMEOUT);
        roach_read_data(m_roach, (uint8_t*)check_data, qdr_mem[m_whichqdr], 1 << 22, cal_data_len[i], MS_TIMEOUT);
        for (size_t j = 0; j < cal_data_len[i]; j++) {
            if (check_data[j] != cal_data[i][j]) {
                blast_err("Calibration check failed for %s on %s", qdr_mem[m_whichqdr], roach_get_name(m_roach));
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
        roach_write_data(m_roach, qdr_mem[m_whichqdr], (uint8_t*)cal_data[i], cal_data_len[i], 1 << 22, MS_TIMEOUT);
        roach_read_data(m_roach, (uint8_t*)check_data, qdr_mem[m_whichqdr], 1 << 22, cal_data_len[i], MS_TIMEOUT);
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
            roach_write_data(m_roach, qdr_mem[m_whichqdr], (uint8_t*)cal_data[i], cal_data_len[i], 1 << 22, MS_TIMEOUT);
            roach_read_data(m_roach, (uint8_t*)check_data, qdr_mem[m_whichqdr], 1 << 22, cal_data_len[i], MS_TIMEOUT);
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
            blast_err("Could not find robust calibration setting for QDR %d on %s",
                      m_whichqdr, roach_get_name(m_roach));
            return false;
        }
        m_indelays[bit] = (begin + end) / 2;
    }

    int median_taps = qdr_get_median_int(m_indelays, n_bits);
    m_indelays[32] = median_taps;
    m_indelays[33] = median_taps;
    m_indelays[34] = median_taps;
    m_indelays[35] = median_taps;

    return true;
}

bool qdr_cal2(roach_state_t *m_roach, int m_whichqdr)
{
    int out_step = 0;
    int in_delays[36] = {0};
    int out_delays[36];

    for (out_step = 0; out_step < 32; out_step++) {
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
                                    "\n\t\t%d %d %d %d %d %d %d %d]", roach_get_name(m_roach),
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

    blast_err("Calibration failed for QDR %d on %s", m_whichqdr, roach_get_name(m_roach));
    return false;
}
