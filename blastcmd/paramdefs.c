/** defaults list.  We will be super dumb and just have an
 *  unsorted list that we hunt through...
 *  -The entire list is re-written to disk if anything is changed or
 *   is added.
 *  -"cmd" should be <command>;<parameter>.  eg azel_goto;az
 *  -The list is never rebuilt and command_list.c is never consulted.
 *  -If commands/parameters are removed from command_list.c, they are not
 *   removed from the database.
 *  -getDefault() returns 0 if the command;parameter is not in the database.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "netcmd.h"
#define CMDDEFSFILE "/data/etc/blastcmd/cmddefs.txt"

struct cmdListStruct {
    char cmd[SIZE_CMDPARNAME];
    double d;
};

struct cmdListStruct *cmd_list = 0;
int n_cmd = 0;
int n_cmd_alloc = 0;

void writeCommands() {
  FILE *fp;
  int i;

  fp = fopen(CMDDEFSFILE, "w");
  if (fp == NULL) {
    fprintf(stderr, "could not open command defaults file %s\n", CMDDEFSFILE);
    // fixme: error message...
    return;
  }
  fprintf(fp, "%d\n", n_cmd);

  for (i=0; i<n_cmd; i++) {
    fprintf(fp, "%s %10g\n", cmd_list[i].cmd, cmd_list[i].d);
  }
  fclose(fp);
}

void readCommands() {
  FILE *fp;
  int i;
  char instr[SIZE_CMDPARNAME+40];

  fp = fopen(CMDDEFSFILE, "r");
  if (fp == NULL) {
    n_cmd = 0;
  } else {
    fgets(instr, SIZE_CMDPARNAME+40, fp);
    sscanf(instr, "%d", &n_cmd);
  }
  n_cmd_alloc = n_cmd + 10;
  cmd_list = realloc(cmd_list, n_cmd_alloc*sizeof(struct cmdListStruct));

  for (i=0; i<n_cmd; i++) {
    fgets(instr, SIZE_CMDPARNAME+40, fp);
    sscanf(instr, "%s %lg", cmd_list[i].cmd, &cmd_list[i].d);
  }
  if (fp !=NULL) {
    fclose(fp);
  }
}

void fixSpaces(char *cmd) {
  int i;

  for (i=0; cmd[i]!='\0'; i++) {
    if (cmd[i] == ' ') {
      cmd[i] = '_';
    }
  }
}


void setDefault(char *cmd_in, double D) {
  int i;

  char cmd[SIZE_CMDPARNAME];
  strncpy(cmd, cmd_in, SIZE_CMDPARNAME);

  fixSpaces(cmd);

  if (cmd_list == 0) {
    readCommands();
  }

  // see if the command already has a default set
  for (i=0; i<n_cmd; i++) {
    if (strncmp(cmd, cmd_list[i].cmd, SIZE_CMDPARNAME)==0) {
      if (cmd_list[i].d == D) { // no change
        return;
      }
      cmd_list[i].d = D;
      break;
    }
  }

  if (i==n_cmd) { // command didn't have a default set yet.
    // allocate more space if it is going to be needed
    if (++n_cmd >= n_cmd_alloc) {
      n_cmd_alloc = n_cmd_alloc*1.5;
      cmd_list = realloc(cmd_list, n_cmd_alloc*sizeof(struct cmdListStruct));
    }
    strncpy(cmd_list[i].cmd, cmd, SIZE_CMDPARNAME);
    cmd_list[i].d = D;
  }
  writeCommands();
}


double getDefault(char *cmd_in) {
  int i;
  char cmd[SIZE_CMDPARNAME];
  strncpy(cmd, cmd_in, SIZE_CMDPARNAME);

  fixSpaces(cmd);

  if (cmd_list == 0) {
    readCommands();
  }

  for (i=0; i<n_cmd; i++) {
    if (strncmp(cmd, cmd_list[i].cmd, SIZE_CMDPARNAME)==0) {
      return cmd_list[i].d;
    }
  }
  return DEF_NOT_FOUND;
}


#if 0
void main(int argc, char *argv[]) {
  double d;

  if (argc == 2) {
    if (argv[1][0]=='-') {
      printf("testCmdLog <cmd> [<value>]\n");
      exit(0);
    }
    d = getDefault(argv[1]);
    printf("%g\n", d);
  } else if (argc==3) {
    d = atof(argv[2]);
    setDefault(argv[1], d);
  }
}
#endif

