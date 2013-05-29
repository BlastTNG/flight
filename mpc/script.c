/* mas.c: MAS connectivity in MPC
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

#include "mpc.h"
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>

/* run a shell script, discarding output.  The scripts STDERR will be piped
 * into the MPC log.  Returns a non-zero integer on error;
 */
int run_simple_script(const char *path, char *argv[])
{
  pid_t pid;
  int s_stdin, s_stdout, p_stderr[2];
  int status;

  /* convenience for scripts with no arguments */
  char* no_args[] = { (char*)path, NULL };
  if (argv == NULL)
    argv = no_args;

  bprintf(info, "Running %s...", path);
  
  /* Plumbing... stdin and stdout go to /dev/null, stderr gets piped back to
   * the parent */
  if ((s_stdin = open("/dev/null", O_RDONLY)) < 0) {
    berror(err, "Can't open /dev/null for reading");
    return errno << 16;
  }
  if ((s_stdout = open("/dev/null", O_WRONLY)) < 0) {
    close(s_stdin);
    berror(err, "Can't open /dev/null for writing");
    return errno << 16;
  }
  if (pipe(p_stderr)) {
    close(s_stdin);
    close(s_stdout);
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
    dup2(s_stdin, 0);
    dup2(s_stdout, 1);
    dup2(p_stderr[1], 2);

    /* now exec the script -- this shouldn't return */
    execvp(path, argv);

    berror(err, "Error executing %s", path);
  } else {
    char buffer[1024];
    FILE *stream;

    close(p_stderr[1]);
    close(s_stdin);
    close(s_stdout);

    /* parent, parrot the clients standard error -- we use the otherwise useless
     * "sched" level of BUOS for script messages */
    stream = fdopen(p_stderr[0], "r");
    if (stream)
      while (fgets(buffer, 1024, stream))
        bprintf(sched, "%s", buffer);

    /* wait for termination -- XXX should probably have a timeout here */
    waitpid(pid, &status, 0);
    close(p_stderr[0]);
    
    if (WIFEXITED(status)) {
      int ret = WEXITSTATUS(status);
      bprintf(ret ? warning : info, "Exit status: %i", ret);
    } else if (WIFSIGNALED(status)) {
      bprintf(err, "Script terminated with signal: %i", WTERMSIG(status));
    }
  }

  return status;
}
