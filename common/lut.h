struct LutType {
  char filename[256];
  int n;
  double *y;
  double *x;
  int last_n;
};

double LutCal(struct LutType *L, double x);
void LutInit(struct LutType *L);
