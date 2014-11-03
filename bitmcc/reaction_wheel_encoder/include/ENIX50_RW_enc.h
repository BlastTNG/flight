#ifndef ENIX50_RW_ENC_H_
#define ENIX50_RW_ENC_H_

// some defines
#define RW_ENC_UPDATE_FREQ 10000	// update rate for RW encoder
#define RW_ENC_A_PIN 21	// pin for A signal
#define RW_ENC_B_PIN 22 // pin for B signal
#define RW_ENC_Z_PIN 23 // pin for Z signal
#define RW_ENC_RES 3600	// encoder resolution [stps/rev]

#define PI 3.141592653589793238462643383

struct rw_encoder
{
	uint8_t A,B,Z;
	uint8_t A_old,B_old,Z_old;
	uint8_t AXORB, AXORB_old;

	unsigned int counter;
	float speed;

	struct IO_board * io;
};

// some function prototypes
int init_rw_encoder(struct rw_encoder *, struct IO_board *);
int pulse_rw_encoder(struct rw_encoder *, float );

#endif /* ENIX50_RW_ENC_H_ */
