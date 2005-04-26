
#define NSTAGE 8

struct FirStruct {
  int i_w[NSTAGE];
  int ns;
  double *w;
  double sum[NSTAGE];
  double out;
};

void initFir(struct FirStruct *fs, int ns);
double filter(double x, struct FirStruct *fs);
