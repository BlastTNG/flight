#ifndef TX_STRUCT_H
#define TX_STRUCT_H

/* Fix things up so we can include this in C++ applications */
#ifdef __cplusplus
extern "C" {
#endif

  /* FAST_PER_SLOW is the number of fast samples for each slow one */
#define FAST_PER_SLOW 20

  /* N_SLOW is the number of slow fields */
#define N_SLOW 16

  /* Number of DAS bolometer cards to include in the frame.  The maximum number
   * of cards is 11 */
#define DAS_CARDS 12

#define DAS_CHS 24

  /* Number of records in the frame below the DAS bolometer records */
#define N_FASTCHLIST_INIT_ACS 42
#define N_FASTCHLIST_INIT_DAS 48
#ifdef BOLOTEST
#define N_FASTCHLIST_INIT (N_FASTCHLIST_INIT_DAS)
#else
#define N_FASTCHLIST_INIT (N_FASTCHLIST_INIT_ACS + N_FASTCHLIST_INIT_DAS)
#endif
#define N_FASTCHLIST (N_FASTCHLIST_INIT + DAS_CARDS * (DAS_CHS+DAS_CHS/2))
  /* Fields defined in FastChList will be read at 100 Hz */

#define FAST_OFFSET (4 + N_SLOW)

  /* offset of encoder.  Reset if encoder has been unmounted. */
  /* This is actually 360 - the real offset */
#define ENC_ELEV_OFFSET (380.27)

#define LOCKIN_C2V (5.43736e-07)
#define LOCKIN_OFFSET (-1.1403)

#define FIELD_LEN 20

  struct ChannelStruct {
    char field[FIELD_LEN]; /* name of channel for FileFormats and CalSpecs */
    char rw;        /* 'r' = read, 'w' = write */
    int node;       /* BlastBus node: 0 to 63 */
    int adr;        /* BlastBus address: 0 to 63 */
    float m_c2e;    /* Conversion from counts to enginering units is */
    float b_e2e;    /*   e = c * m_c2e + b_e2e */
    char type;      /* 's' = short, signed o'u' = unsigned short 'i' = 'S'
                       = signed 32 bit int, 'U' = unsigned 32 bit int */
  };

  extern struct ChannelStruct SlowChList[N_SLOW][FAST_PER_SLOW];
  extern struct ChannelStruct FastChList[N_FASTCHLIST];

  void MakeTxFrame(void);
  void FPrintDerived(FILE*);
  void FastChIndex(char*, int*);
  void SlowChIndex(char*, int*, int*);

#define DEG2LI (4294967296.0/360.0)
#define LI2DEG (1.0/DEG2LI)
#define RAD2LI (4294967296.0/2/M_PI)
#define DEG2I (65536.0/360.0)
#define I2DEG (1.0/DEG2I)
#define RAD2I (65536.0/2/M_PI)
#define H2I (65536.0/24.0)
#define I2H (1.0/H2I)
#define VEL2I (65536.0/10.0)
#define I2VEL (1.0/VEL2I)

  /* conversions between dps and a to d units for each gyro */
#define DPS_TO_ADU1 (1092.8128/1.0407)
#define ADU1_TO_DPS (1.0/DPS_TO_ADU1)
#define DPS_TO_ADU2 (1092.8128/1.036)
#define ADU2_TO_DPS (1.0/DPS_TO_ADU2)
#define DPS_TO_ADU3 (1092.8128/1.036)
#define ADU3_TO_DPS (1.0/DPS_TO_ADU3)

  /* Approximate gyro offsets - to get us close */
#define GYRO1_OFFSET 25940.0
#define GYRO2_OFFSET 25370.0
#define GYRO3_OFFSET 25370.0

  /* The size of the rx and downlink frames */
#define FRAME_WORDS \
  ( 1              /* FILETYPE */ \
    + 2            /* FRAMENUM */ \
    + 1            /* SLOW_INDEX */ \
    + N_SLOW       /* slow channels */ \
    + N_FASTCHLIST /* fast channels and bolometers */ \
  )

#define TX_FRAME_SIZE (FRAME_WORDS * sizeof(unsigned int))
#define RX_FRAME_SIZE (FRAME_WORDS * sizeof(unsigned short))

#ifdef __cplusplus
}
#endif

#endif
