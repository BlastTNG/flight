/* ---------------------------------------------------------------------
 * ------------------ STABLECMD UDP Receive Program --------------------
 * ---------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a UDP server developed to interface with the STABLE client
 * running on the STABLE computer.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: October 15, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */
#include <sys/time.h>

#ifndef STABLECMD_H_
#define STABLECMD_H_

// some defines
#define RECVBUFLEN 1024
#define STABLE_CMD_ID_OFFSET 0x9000
#define STABLE_TELEM_ID_OFFSET 0xa000
#define STABLE_PORT 31313	// port over which STABLE communicates with BIT ADCS

enum TlmType
{
	DATA,	EVENT
};

enum TlmDataType
{
	NONE,	U32,	I32,	F32
};

enum StableCmds
{
	PMM_SET_MODE_CONTROL,	PMM_SET_MODE_READY,		PMM_FSW_RESET,

	/* Nothing below this */
	NSTABLECMDS
};

enum StableTelem
{
	PMM_ENTERING_CONTROL_X_LOC,		PMM_ENTERING_CONTROL_Y_LOC,
	PMM_EXIT_FROM_CONTROL,			PMM_CURRENT_MODE
};

// data structure for STABLE commands/telemetry
struct STABLECMD
{
	uint16_t id; // ID of telemetry entry
	char name[80]; // string for the name

	enum TlmType type; // type of telemetry entry
	enum TlmDataType dataType; // data type for entry's value
	uint16_t rt_update_interval; // every nth entry stored for this tis will be telemetered for downlink
	uint16_t rec_update_interval; // every nth entry stored for this tis will be saved to disk
	uint16_t rt_reported_insts; // number of times the tlm entry was reported since last rt_update_interval
	uint16_t rec_reported_insts; // number of times the tlm entry was reported since last rec_update_interval
};

// data structure for STABLE server
struct STABLESERVER
{
	struct sockaddr_in cli_addr, my_addr;
	int sockfd, i;
	int num_bytes;
	socklen_t slen;

	uint8_t recv_buf[RECVBUFLEN];

	struct STABLECMD stablecmd; // command entry for received/sent commands
	struct timeval ts; // timestamp for last stored value (lsv)
	union // telemetry values for the entry
	{
		uint32_t tlm_U32_lsv;
		int32_t tlm_I32_lsv;
		float tlm_F32_lsv;
	};
};

// some function prototypes
int init_stableserver(struct STABLESERVER * );
int read_stableserver(struct STABLESERVER * );
int send_stableserver(struct STABLESERVER * , enum StableCmds , uint32_t * );
int close_stableserver(struct STABLESERVER * );

// some external variables
extern const struct STABLECMD stablecmd[NSTABLECMDS];

#endif /* STABLECMD_H_ */
