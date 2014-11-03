#define FIFODEPTH 2048

struct fifoStruct {
  char d[FIFODEPTH];
  int i_in;  // points at next place to write
  int i_out; // points at next place to read
};

void convertToUpper(char *in, char *out);
void convertToLower(char *in, char *out);
void push(struct fifoStruct *fs, char x[], int size);
void peek(struct fifoStruct *fs, char x[], int size);
void pop(struct fifoStruct *fs, char x[], int size);
void advance(struct fifoStruct *fs, int size);
void advance(struct fifoStruct *fs, int size);
int nFifo(struct fifoStruct *fs);
int party_connect(char *hostname, int port);
int BlockingRead(int minRead, struct fifoStruct *fs, int tty_fd, char *hostname, int port);