#include <limits.h>
#include <stdlib.h>
#include <stdio.h>

#define N_STAT_TYPES 3
#define NUM_MCE 6
#define NUM_COL 16
#define NUM_ROW 33
#define NUM_ARRAY_STAT (NUM_MCE*N_STAT_TYPES*NUM_COL*NUM_ROW)

char *GetArrayFieldName(int i_field) {
  static char **names = 0;
  int i;
  
  if (names == 0) {
    int type, tel, row, col;
    char types[N_STAT_TYPES][10] = {"mean", "sigma", "noise"};
    names = (char **) malloc(NUM_ARRAY_STAT * sizeof(char *));
    for (i=0; i < NUM_ARRAY_STAT; i++) {
      names[i] = (char *) malloc(21*sizeof(char));
    }
    i=0;
    for (tel = 0; tel < NUM_MCE; tel++) {
      for (type = 0; type < 3; type++) {
        for (row = 0; row<NUM_ROW; row++) {
          for (col = 0; col<NUM_COL; col++) {
            sprintf(names[i], "%s_x%1dr%02dc%02d", types[type], tel+1, row, col);
            i++;
          }
        }
      }
    }            
  }
  return names[i_field];
}

int main() {
  int i;
  
  printf("arraystats_0 BIT arraystats_data 0 7\n"
  "arraystats_1 BIT arraystats_data 8 15\n");
  
  for (i=0; i<NUM_ARRAY_STAT; i+=2) {
    printf("%s MPLEX arraystats_0 arraystats_index %d %d\n"
           "%s MPLEX arraystats_1 arraystats_index %d %d\n",
           GetArrayFieldName(i), i, NUM_ARRAY_STAT,
           GetArrayFieldName(i+1), i, NUM_ARRAY_STAT);
  }
}
