/******************************************************************************\
|*                                                                            *|
|* ALICE.H                                                                    *|
|* Definitions common to big.cpp and small.cpp.                               *|
|*                                                                            *|
\******************************************************************************/

// Compression types.
#define COMP_DIFFERENTIAL       0
#define COMP_INT_PRESERVING     1
#define COMP_SINGLE             2
#define COMP_AVERAGE            3

// Special bytes.
#define BUF_FRAME_SYNC          0xff
#define BUF_FILE_NUM_PADDER     0xf0
#define BUF_SECTION_SYNC        0xaa
#define BUF_NO_DATA             0x99
#define BUF_END_SYNC            0x55

// Reserved bytes in the frame.
#define BUF_POS_FRAME_SYNC      0
#define BUF_LEN_FRAME_SYNC      4
#define BUF_POS_FILE_NUM        (BUF_POS_FRAME_SYNC + BUF_LEN_FRAME_SYNC)
#define BUF_POS_FRAME_NUM       (BUF_POS_FILE_NUM + 1)
#define BUF_LEN_FRAME_NUM       3
#define BUF_POS_FRAME_LEN       (BUF_POS_FRAME_NUM + BUF_LEN_FRAME_NUM)
#define BUF_POS_CRC             (BUF_POS_FRAME_LEN + 2)
#define BUF_POS_DATA_START      (BUF_POS_CRC + 2)

// CRC stuff.
#define CRC_INIT                0
