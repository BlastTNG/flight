#include <stdio.h>

void main() {
  char line[1024];
  int i=0;
  int row;

  while (fgets(line, 1020, stdin)!=NULL) {
    sscanf(line, "%d", &row);
    if (i!=row) {
      printf("Error line %d (%d)\n", i, row);
      i=row;
    }
    i++;
  }
  printf("checked %d rows\n", i);
}


