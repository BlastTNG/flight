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

struct mcp_proc {
  pid_t pid;
  time_t stop_time;
  int in_fd[2], out_fd[2], err_fd[2];
  int reaped, status, announce;
};

static void close_proc_pipes(struct mcp_proc *p)
{
  if (p->in_fd[0] >= 0)
    close(p->in_fd[0]);
  if (p->in_fd[1] >= 0)
    close(p->in_fd[1]);
  if (p->out_fd[0] >= 0)
    close(p->out_fd[0]);
  if (p->out_fd[1] >= 0)
    close(p->out_fd[1]);
  if (p->err_fd[0] >= 0)
    close(p->err_fd[0]);
  if (p->err_fd[1] >= 0)
    close(p->err_fd[1]);
}

/* finish a process, optionally with waiting.  Returns the process's exit
 * status (non-zero on error) */
int stop_proc(struct mcp_proc *p, int stop_now, int infd_closed,
    int outfd_closed, int errfd_closed)
{
  int status;

  if (infd_closed)
    p->in_fd[1] = -1;
  if (outfd_closed)
    p->out_fd[1] = -1;
  if (errfd_closed)
    p->err_fd[1] = -1;

  /* already reaped */
  if (p->reaped) {
    status = p->status;
    goto JOINED;
  }

  if (stop_now) /* force timeout */
    p->stop_time = 1;

  if (p->stop_time == 0) { /* wait forever */
    waitpid(p->pid, &status, 0);
    goto JOINED;
  }
  
  /* wait for timeout */
  do {
    if (waitpid(p->pid, &status, WNOHANG))
      goto JOINED;
    sleep(1);
  } while (time(NULL) <= p->stop_time);

  /* timed out (this is the fun part) */

  /* going... */
  kill(p->pid, SIGHUP);
  usleep(100000);
  /* going... */
  kill(p->pid, SIGTERM);
  usleep(100000);
  /* gone! */
  kill(p->pid, SIGKILL);
  usleep(100000);

  /* last check */
  if (waitpid(p->pid, &status, WNOHANG) == 0) {
    bprintf(err, "Process %i zombified.", p->pid);
    status = -1;
  } else {
JOINED:
    if (p->announce) {
      if (WIFEXITED(status)) {
        int ret = WEXITSTATUS(status);
        bprintf(ret ? warning : info, "Process exit status: %i", ret);
      } else if (WIFSIGNALED(status)) {
        bprintf(err, "Process terminated on signal: %i", WTERMSIG(status));
      }
    }
  }

  /* finally, tidy up */
  close_proc_pipes(p);
  free(p);
  return status;
}

/* check whether a process has terminated or exceeded it's timeout.
 *
 * Returns:
 *  0 if it's still running and hasn't gone over time
 *  1 if it has terminated
 *  2 if it has gone over time
 */
int check_proc(struct mcp_proc *p)
{
  int status;
  /* already checked this */
  if (p->reaped)
    return 1;

  /* check timeout */
  if (p->stop_time > 0 && time(NULL) > p->stop_time)
    return 2;

  /* check for terimation */
  if (waitpid(p->pid, &status, WNOHANG) == 0)
    return 0;
  
  /* terminated, make a note and record it's status */
  p->reaped = 1;
  p->status = status;
  return 1;
}

/* run a process, with plumbing.  If provided, returns fds attached to the
 * subprocess's standard streams.  Returns NULL on error, or a process record.
 * Use stop_proc() to terminate and clean up.
 */
struct mcp_proc *start_proc(const char *path, char *argv[], int timeout,
    int announce, int *in_fd, int *out_fd, int *err_fd)
{
  struct mcp_proc *p = malloc(sizeof(struct mcp_proc));
  p->in_fd[0]  = p->in_fd[1]  = -1;
  p->out_fd[0] = p->out_fd[1] = -1;
  p->err_fd[0] = p->err_fd[1] = -1;
  p->reaped = 0;
  p->announce = announce;

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

  /* plumbing.  If a corresponding return location isn't given, attach to
   * /dev/null, otherwise pipe-ify. */
  if (in_fd) {
    if (pipe(p->in_fd)) {
      berror(err, "Can't redirect STDIN");
      close_proc_pipes(p);
      return NULL;
    }
    *in_fd = p->in_fd[1];
  } else {
    if ((p->in_fd[0] = open("/dev/null", O_RDONLY)) < 0) {
      berror(err, "Can't open /dev/null for reading");
      close_proc_pipes(p);
      return NULL;
    }
  }

  if (out_fd) {
    if (pipe(p->out_fd)) {
      berror(err, "Can't redirect STDIN");
      close_proc_pipes(p);
      return NULL;
    }
    *out_fd = p->out_fd[0];
  } else {
    if ((p->out_fd[1] = open("/dev/null", O_WRONLY)) < 0) {
      berror(err, "Can't open /dev/null for writing");
      close_proc_pipes(p);
      return NULL;
    }
  }

  if (err_fd) {
    if (pipe(p->err_fd)) {
      berror(err, "Can't redirect STDIN");
      close_proc_pipes(p);
      return NULL;
    }
    *err_fd = p->err_fd[0];
  } else {
    if ((p->err_fd[1] = open("/dev/null", O_WRONLY)) < 0) {
      berror(err, "Can't open /dev/null for writing");
      close_proc_pipes(p);
      return NULL;
    }
  }

  p->stop_time = timeout ? time(NULL) + timeout : 0;

  /* Fork */
  p->pid = fork();
  if (p->pid < 0) {
    close_proc_pipes(p);
    berror(err, "fork");
    return NULL;
  } else if (p->pid == 0) {
    /* child: perform the redirections */
    if (p->in_fd[1] >= 0)
      close(p->in_fd[1]);
    if (p->out_fd[0] >= 0)
      close(p->out_fd[0]);
    if (p->err_fd[0] >= 0)
      close(p->err_fd[0]);
    dup2(p->in_fd[0], 0);
    dup2(p->out_fd[1], 1);
    dup2(p->err_fd[1], 2);

    /* now exec the process -- this shouldn't return */
    execvp(path, argv);

    /* The parent will deal with redirecting this, dont use BUOS */
    fprintf(stderr, "Error executing %s: %m", path);
    exit(1); /* child exits */
  }

  /* parent -- close the child-side descriptors */
  close(p->in_fd[0]);
  close(p->out_fd[1]);
  close(p->err_fd[1]);
  p->in_fd[0] = p->out_fd[1] = p->err_fd[1] = -1;

  return p;
}

/* run a process, discarding its STDOUT.  The process's STDERR will be piped
 * into the BUOS log at "elev".  Optionally, kill the program if the timeout
 * is exceeded.  Returns a non-zero integer on error;
 */
int exec_and_wait(buos_t elev, const char *path, char *argv[], unsigned timeout,
    int announce)
{
  struct mcp_proc *p;
  int p_stderr;
  char buffer[1024];
  FILE *stream;

  p = start_proc(path, argv, timeout, announce, NULL, NULL, &p_stderr);
  if (p == NULL)
    return errno ? (errno << 16) : -1;
  
  stream = fdopen(p_stderr, "r");
  if (stream == NULL)
    berror(warning, "Unable to fdopen STDERR");
  else {
    while (check_proc(p) == 0) {
      /* parrot the clients standard error */
      if (fgets(buffer, 1024, stream))
        bprintf(elev, "%s", buffer);
      usleep(10000);
    }
    while (fgets(buffer, 1024, stream)) /* clear the buffer */
      bprintf(elev, "%s", buffer);
    fclose(stream);
  }

  return stop_proc(p, 0, 0, 0, 1);
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
