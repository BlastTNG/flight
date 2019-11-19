#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "command_list.h"
#include "netcmd.h"

#include "elog.h"

/* This must be the full path to the executable (no path search is performed) */
//#define ELOG_BIN "/usr/local/bin/elog"
#define ELOG_BIN "/usr/local/bin/elog"

#if defined __BLAST__
//////////////////////////////////////////////////////////////////////
// BLAST elog code
//////////////////////////////////////////////////////////////////////

#define ELOG_HOST   "blastgs1"
#define ELOG_PORT   "8020"
#define ELOG_LOG    "blasttng2018"
#define ELOG_AUTH   "blastcmd-daemon"

static char cmd_log[5000];

void ElogBeforeRoute(const char *buffer, const char *user)
{
  snprintf(cmd_log, 4999, "/usr/local/bin/elog -h " ELOG_HOST " -p " ELOG_PORT
      " -l " ELOG_LOG " -a Author=" ELOG_AUTH " -a Category=blastcmd -a Source=blastcmd "
      "\"%s EXE %s\n", user, buffer);
  cmd_log[4999] = '\0';
}

void ElogAfterRoute(int result, int last_sock)
{
  char log[5000];
  snprintf(log, 4999, "%sResult = %d\"", cmd_log, result);
  log[4999] = '\0';

  if (result == 0) {
    if (fork() != 0) {
      return;
    } else {
      exit(system(log));
    }
  }
}

#elif defined __SPIDER__
//////////////////////////////////////////////////////////////////////
// Spider elog code
//////////////////////////////////////////////////////////////////////
#define ELOG_HOST   "elog.spidercmb.com"
#define ELOG_PORT   "80"
#define ELOG_LOG    "spider_lloro5"
#define ELOG_AUTH   "spidercmd"

/* the ELOG command line arguments */
#define CMD_LOG_SUBSYSTEM 10
#define CMD_LOG_CATEGORY  12
#define CMD_LOG_INSERT    14
#define CMD_LOG_MESSAGE   17
static char *cmd_log[] = {
  ELOG_BIN,
  "-h", ELOG_HOST,
  "-p", ELOG_PORT,
  "-l", ELOG_LOG,
  "-a", "Author=" ELOG_AUTH,
  "-a", NULL /* Subsystem cmd_log[10] */,
  "-a", NULL /* Category cmd_log[12] */,
  "-a", NULL /* Insert cmd_log[14] */,
  "-a", "ByAPerson=0",
  NULL /* Message cmd_log[17] */,

  NULL /* sentinel */
};

static void SetCommandAttributes(const char *buffer)
{
  int insert_num = -1;
  unsigned int group = 0;
  char *buffer_cpy;
  char *saveptr = NULL;
  char *cmd;
  char *params[MAX_N_PARAMS];
  int n_params = 0;
  int i_cmd;
  int i_grp;
  int i_param;

  // lookup from (insert_num+1) to elog Insert attribute
  static const char *insert_name_list[] = {"Insert=",
    "Insert=X1|X2|Y3|Y4|X5|X6",
    "Insert=X1", "Insert=X2", "Insert=Y3",
    "Insert=Y4", "Insert=X5", "Insert=X6"
  };
  static const char *subsystem_name_list[] = {
    "Subsystem=Gondola",
    "Subsystem=Cryo",
    "Subsystem=MCE",
    "Subsystem=General"
  };

  // list of command names to which all inserts should be applied
  static const char *cmd_all_insert_list[] = {"hk_gang_cycle", "hk_bias_freq",
    "thermveto_enable", "thermveto_disable", "restart_reset_on",
    "restart_reset_off",
    ""};

  // tokenize the buffer string into command name and parameter values
  buffer_cpy = strdup(&buffer[3]);    // drop link, COMM info from start
  cmd = strtok_r(buffer_cpy, " ", &saveptr);
  if (cmd[0] == '_')
    cmd++;

  while ((params[n_params] = strtok_r(NULL, " " , &saveptr)) != NULL) {
    n_params++;
  }

  for (i_cmd = 0; i_cmd < N_SCOMMANDS; i_cmd++) {
    if (strncmp(cmd, scommands[i_cmd].name, SIZE_NAME) == 0) {
      group = scommands[i_cmd].group;
    }
  }

  for (i_cmd = 0; i_cmd < N_MCOMMANDS; i_cmd++) {
    if (strncmp(cmd, mcommands[i_cmd].name, SIZE_NAME) == 0) {
      if (mcommands[i_cmd].numparams == n_params) {
        group = mcommands[i_cmd].group;
        // find insert number
        for (i_param = 0; i_param < mcommands[i_cmd].numparams; i_param++) {
          if (mcommands[i_cmd].params[i_param].index_serial &&
              strncmp(mcommands[i_cmd].params[i_param].name,
                "Insert", 7) == 0) {
            insert_num = atoi(params[i_param]);
            break;
          }
        }
      } else {
        fprintf(stderr, "Warning: mismatch in number of command parameters");
      }
      break;
    }
  }

  // check manual override of insert list
  i_cmd = 0;
  while (cmd_all_insert_list[i_cmd][0] != '\0') {
    if (strncmp(cmd, cmd_all_insert_list[i_cmd], SIZE_NAME) == 0) {
      insert_num = 0;
    }
    i_cmd++;
  }

  /* set insert name */
  cmd_log[CMD_LOG_INSERT] = (char*)insert_name_list[insert_num + 1];

  // assign subsystem name based on the command's group
  if (group &
        (GR_POINT | GR_LOCK | GR_SC | GR_SCTAB | GR_SCPWR | GR_TRIM | GR_VETO
         | GR_GAIN | GR_POWER | GR_MOTPWR)
      )
  {
    cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[0];    // Gondola
  } else if (group &
      (GR_SFTV | GR_HWPR | GR_BIAS | GR_CRYO_HEAT | GR_FRIDGE | GR_LLORO_HEAT))
  {
    cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[1];    // Cryo
  } else if (group & (GR_MPC | GR_DETMON | GR_TUNE1 | GR_TUNE2 | GR_SQUID
        | GR_MCCPWR | GR_TUNECH | GR_MPCMODE | GR_DET | GR_MCESRVO | MCECMD))
  {
    cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[2];    // MCE
  }
  else if (group & (GR_TELEM | GR_MISC)) {
    cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[3];    // General
  }
  // for really mixed groups, find subsystem from command name
  else if (group & GR_IFPOWER) {
    if (strncmp(cmd, "mce", 3) == 0) {
      cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[2];    // MCE
    } else {
      cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[1];    // Cryo
    }
  }
  else if (group & GR_TIMING)  {
    if (strncmp(cmd, "mce", 3) == 0) {
      cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[2];    // MCE
    } else {
      cmd_log[CMD_LOG_SUBSYSTEM] = (char*)subsystem_name_list[3];    // General
    }
  }

  /* for Category, magically find the first Group that the command belongs to */
  static const int magic_numbers[] = { 32, 0, 1, 26, 2, 23, 27, 0, 3, 16, 24,
    30, 28, 11, 0, 13, 4, 7, 17, 0, 25, 22, 31, 15, 29, 10, 12, 6, 0, 21, 14, 9,
    5, 20, 8, 19, 18 };
  i_grp = magic_numbers[(-group & group) % 37];

  /* compose the argument */
#define CATEGORY_TEMPLATE "Category=%s"
  cmd_log[CMD_LOG_CATEGORY] = malloc(strlen(GroupNames[i_grp])
      + sizeof(CATEGORY_TEMPLATE));
  sprintf(cmd_log[CMD_LOG_CATEGORY], CATEGORY_TEMPLATE, GroupNames[i_grp]);

  free(buffer_cpy);
}

void ElogBeforeRoute(const char *buffer, const char *user)
{
  SetCommandAttributes(buffer);

  /* compose the message */
#define TEMPLATE "EXE %s  [ab %s]"
  cmd_log[CMD_LOG_MESSAGE] = malloc(strlen(buffer) + sizeof(TEMPLATE)
      + strlen(user));
  sprintf(cmd_log[CMD_LOG_MESSAGE], TEMPLATE, buffer, user);
}

void ElogAfterRoute(int result, int last_sock)
{
  int i;

  /* print the cmd_log for debugging purposes */
  /*
  printf("Debug: sending elog command:\n %s", ELOG_BIN);
  for (int j=0; j<19; j++) {
    if (cmd_log[j] != NULL) {
      printf("%s ", cmd_log[j]);
    } else {
      printf("NULL ");
    }
  }
  */

  /* Don't log failed commands; otherwise run elog asynchronously */
  if (result || fork() != 0) {
    /* parent - children are autoreaped, so just return */

    free(cmd_log[CMD_LOG_MESSAGE]);
    free(cmd_log[CMD_LOG_CATEGORY]);
    return;
  }

  /* The rest of this happens in the child */

  /* Close all network connections to prevent screwing up and/or being screwed
   * up by elog */
  for (i = 3; i < last_sock; ++i)
    close(i);

  /* exec elog -- doesn't return */
  execv(ELOG_BIN, cmd_log);

  /* exec error */
  exit(1);
}

#endif    // __BLAST__ or __SPIDER__
