/* FAST_PER_SLOW is the number of fast samples for each slow one */
#define FAST_PER_SLOW 20

/* N_SLOW is the number of slow fields */
#define N_SLOW 8

/* Number of DAS bolometer cards to include in the frame.  The maximum number
 * of cards is 11 */
#define DAS_CARDS 12

#define DAS_CHS 24

/* Number of records in the frame below the DAS bolometer records */
#define N_FASTCHLIST_INIT 78
#define N_FASTCHLIST (N_FASTCHLIST_INIT + DAS_CARDS * (DAS_CHS+DAS_CHS/2))
/* Fields defined in FastChList will be read at 100 Hz */

#define FAST_OFFSET (4 + N_SLOW)

/* offset of encoder.  Reset if encoder has been unmounted. */
#define ENC_ELEV_OFFSET 341.322
/* Angle between magnetometer aligment and pointing alignment -- to be */
/* calibrated. */
#define MAG_ALIGNMENT  0.0

#define LOCKIN_C2V (5.43736e-07)
#define LOCKIN_OFFSET (-1.1403)
struct ChannelStruct {
    char field[20]; /* name of channel for FileFormats and CalSpecs */
    char rw;        /* 'r' = read, 'w' = write */
    int node;       /* BlastBus node: 0 to 63 */
    int adr;        /* BlastBus address: 0 to 63 */
    float m_c2e;    /* Conversion from counts to enginering units is */
    float b_e2e;    /*   e = c * m_c2e + b_e2e */
    char type;      /* 's' = short, signed 'u' = unsigned short 'i' = 'S' signed
                       32 bit int, 'U' = unsigned 32 bit int */
};

extern struct ChannelStruct SlowChList[N_SLOW][FAST_PER_SLOW];
extern struct ChannelStruct FastChList[N_FASTCHLIST];

#ifndef BLASTCOM    /* Don't forward declare prototype if compiling blastcom */
void MakeTxFrame(void);
#endif

void FastChIndex(char*, int*);
void SlowChIndex(char*, int*, int*);
