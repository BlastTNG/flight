struct LutType {
  char filename[256];
  int n;
  double *y;
  double *x;
  int last_n;
};

double MagLutCal(struct LutType *L, double mag_x, double mag_y, double x);
double LutCal(struct LutType *L, double x);
void LutInit(struct LutType *L);
