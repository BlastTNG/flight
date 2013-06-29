/* sys.c: various Linux system routines
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "sys.h"
#include "blast.h"

#include <sys/mount.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <string.h>

/* run a process, discarding its STDOUT.  The process's STDERR will be piped
 * into the BUOS log at "elev".  Optionally, kill the program if the timeout
 * is exceeded.  Returns a non-zero integer on error;
 */
int exec_and_wait(buos_t elev, const char *path, char *argv[], unsigned timeout,
    int announce)
{
  pid_t pid;
  int c_stdin, c_stdout, p_stderr[2];
  int status;

  time_t start_time = time(NULL);

  /* a convenience for processes with no arguments */
  char* no_args[] = { (char*)path, NULL };
  if (argv == NULL)
    argv = no_args;

  if (announce) {
    char cmd[4096];
    int i;
    strncpy(cmd, path, 4096);

    /* this is slowish */
    for (i = 1; argv[i]; ++i)
      strncpy(strncat(cmd, " ", 4096), path, 4096);
    bprintf(info, "Running %s", cmd);
  }
  
  /* Plumbing: stdin and stdout go to /dev/null, stderr gets piped back to
   * the parent */
  if ((c_stdin = open("/dev/null", O_RDONLY)) < 0) {
    berror(err, "Can't open /dev/null for reading");
    return errno << 16;
  }
  if ((c_stdout = open("/dev/null", O_WRONLY)) < 0) {
    close(c_stdin);
    berror(err, "Can't open /dev/null for writing");
    return errno << 16;
  }
  if (pipe(p_stderr)) {
    close(c_stdin);
    close(c_stdout);
    berror(err, "Can't redirect STDERR");
    return errno << 16;
  }

  /* Fork */
  pid = fork();
  if (pid < 0) {
    berror(err, "fork");
    status = errno << 16;
  } else if (pid == 0) {
    /* child: perform the redirections */
    close(p_stderr[0]);
    dup2(c_stdin, 0);
    dup2(c_stdout, 1);
    dup2(p_stderr[1], 2);

    /* now exec the process -- this shouldn't return */
    execvp(path, argv);

    /* Our parent will pipe this into the log, so don't use BUOS */
    fprintf(stderr, "Error executing %s: %m", path);
    exit(1); /* child exits */
  } else {
    /* parent */

    char buffer[1024];
    FILE *stream;

    close(p_stderr[1]);
    close(c_stdin);
    close(c_stdout);

    /* parrot the clients standard error */
    stream = fdopen(p_stderr[0], "r");
    if (stream)
      while (fgets(buffer, 1024, stream))
        bprintf(elev, "%s", buffer);

    /* wait for termination and/or timeout */
    if (timeout == 0) {
      waitpid(pid, &status, 0);
    } else {
      while (time(NULL) <= start_time + timeout) {
        if (waitpid(pid, &status, WNOHANG))
          goto JOINED;
        sleep(1);
      } 

      /* timeout: terminate the process */

      /* going... */
      kill(pid, SIGHUP);
      usleep(100000);
      /* going... */
      kill(pid, SIGTERM);
      usleep(100000);
      /* gone */
      kill(pid, SIGKILL);
      usleep(100000);

      /* last check */
      if (waitpid(pid, &status, WNOHANG) == 0) {
        bprintf(err, "Process zombified.");
        close(p_stderr[0]);
        return -1;
      }
    }
JOINED:
    close(p_stderr[0]);
    
    if (announce) {
      if (WIFEXITED(status)) {
        int ret = WEXITSTATUS(status);
        bprintf(ret ? warning : info, "Process exit status: %i", ret);
      } else if (WIFSIGNALED(status)) {
        bprintf(err, "Process terminated on signal: %i", WTERMSIG(status));
      }
    }
  }

  return status;
}

/* mount something listed in the fstab -- with forkexeccy goodness */
int do_mount(const char *name, int timeout)
{
  char *argv[] = { "mount", (char*)name, NULL };
  return exec_and_wait(err, "/bin/mount", argv, timeout, 0);
}

/* lazy unmount something */
int do_umount(const char *target)
{
  int ret = umount2(target, MNT_DETACH);
  if (ret)
    berror(err, "umount");

  return ret;
}
