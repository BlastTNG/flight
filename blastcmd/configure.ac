dnl Process this file with autoconf to produce a configure script.
AC_INIT(blastcmd, 1.1.9, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(blastcmd, 1.1.9)
AM_CONFIG_HEADER(config.h)

AC_ARG_WITH([data-etc],
            [  --with-data-etc=DIR     put auxiliary data in DIR [/data/etc]],
            DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)
AC_DEFINE_UNQUOTED(DATA_ETC_DIR, "$DATA_ETC_DIR",
                   [ Define to the path to your data directory ])

AC_ARG_ENABLE([blastcmd], AS_HELP_STRING([--enable-blastcmd],
              [install the blastcmd binary [[default=no]]]),
          enable_blastcmd="${enableval}",enable_blastcmd="no")

AC_ARG_ENABLE([blastcmd-remote], AS_HELP_STRING([--enable-blastcmd-remote],
              [install the blastcmd-remote script [[default=yes]]]),
          enable_blastcmd_remote="${enableval}",enable_blastcmd_remote="yes")

AC_ARG_ENABLE([fifocmd], AS_HELP_STRING([--enable-fifocmd],
              [install the fifocmd script [[default=no]]]),
          enable_fifocmd="${enableval}",enable_fifocmd="no")

AC_ARG_ENABLE([dummycmd], AS_HELP_STRING([--enable-dummycmd],
              [install the dummycmd script [[default=no]]]),
          enable_dummycmd="${enableval}",enable_dummycmd="no")

AC_ARG_WITH([remote-host], AS_HELP_STRING([--with-remote-host=HOST],
            [use HOST as the remote host for blastcmd-remote [[galadriel]]]),
            REMOTE_HOST="${withval}", REMOTE_HOST="galadriel")

AC_ARG_WITH([remote-user], AS_HELP_STRING([--with-remote-user=USER],
            [have blastcmd-remote log into the remote host as USER [[none]]]),
            REMOTE_USER="${withval}@", REMOTE_USER="")

AC_ARG_WITH([remote-command], AS_HELP_STRING([--with-remove-command=CMD],
            [use CMD as the remote command for blastcmd-remote [[blastcmd]]]),
            REMOTE_COMMAND="${withval}", REMOTE_COMMAND="blastcmd")

dnl Programs
AC_PROG_CC
AC_LANG_C
AC_C_CONST
AC_PROG_INSTALL

AC_MSG_CHECKING(the revision number of the channel list)
CMDVERSION=`grep command_list_serial ${srcdir}/command_list.c | gawk '{print $6}'`
AC_MSG_RESULT([$CMDVERSION])
AC_SUBST(CMDVERSION)

dnl What are we installing?
AC_MSG_CHECKING(whether we are installing blastcmd)
if test "x$enable_blastcmd" = "xyes"; then
  INSTALL_BLASTCMD=yes;
  INSTALLING_SOMETHING=yes;
else
  INSTALL_BLASTCMD=no;
fi
AC_MSG_RESULT([$INSTALL_BLASTCMD])
AM_CONDITIONAL(INSTALL_BLASTCMD, test "$INSTALL_BLASTCMD" = "yes")

AC_MSG_CHECKING(whether we are installing blastcmd-remote)
if test "x$enable_blastcmd_remote" = "xyes"; then
  INSTALL_BLASTCMD_REMOTE=yes;
  INSTALLING_SOMETHING=yes;
else
  INSTALL_BLASTCMD_REMOTE=no;
fi
AC_MSG_RESULT([$INSTALL_BLASTCMD_REMOTE])
AM_CONDITIONAL(INSTALL_BLASTCMD_REMOTE, test "$INSTALL_BLASTCMD_REMOTE" = "yes")

AC_MSG_CHECKING(whether we are installing fifocmd)
if test "x$enable_fifocmd" = "xyes"; then
  INSTALL_FIFOCMD=yes;
  INSTALLING_SOMETHING=yes;
else
  INSTALL_FIFOCMD=no;
fi
AC_MSG_RESULT([$INSTALL_FIFOCMD])
AM_CONDITIONAL(INSTALL_FIFOCMD, test "$INSTALL_FIFOCMD" = "yes")

AC_MSG_CHECKING(whether we are installing dummycmd)
if test "x$enable_dummycmd" = "xyes"; then
  INSTALL_DUMMYCMD=yes;
  INSTALLING_SOMETHING=yes;
else
  INSTALL_DUMMYCMD=no;
fi
AC_MSG_RESULT([$INSTALL_DUMMYCMD])
AM_CONDITIONAL(INSTALL_DUMMYCMD, test "$INSTALL_DUMMYCMD" = "yes")

if test "x$INSTALLING_SOMETHING" = "x"; then
  AC_MSG_ERROR([*** Dude, you gotta install something... ***])
fi

if test "$INSTALL_FIFOCMD" = "yes"; then
  AC_CHECK_PROG(PERL,perl,yes)
  if test "x$PERL" = "x"; then
    AC_MSG_ERROR([*** Perl is required to run fifocmd -- please install first ***])
  fi;
fi

dnl set up blastcmd-remote
if test "$INSTALL_BLASTCMD_REMOTE" = "yes"; then
  AC_MSG_CHECKING([for the name of the remote host])
  AC_MSG_RESULT([$REMOTE_HOST])
  AC_SUBST(REMOTE_HOST)
  AC_SUBST(REMOTE_USER)

  AC_MSG_CHECKING([for the name of the remote command])
  AC_MSG_RESULT([$REMOTE_COMMAND])
  AC_SUBST(REMOTE_COMMAND)
fi

dnl Set Up blastcmd
if test "$INSTALL_BLASTCMD" = "yes"; then
  AC_CHECK_LIB(m,round)

  dnl See if blastcmd.log is already installed
  AC_MSG_CHECKING(whether blastcmd.log is already present)
  if test -f "$DATA_ETC_DIR/blastcmd.log"; then
    HAVE_BLASTCMD_LOG=yes;
  else
    HAVE_BLASTCMD_LOG=no;
  fi
  AC_MSG_RESULT([$HAVE_BLASTCMD_LOG])
else
  HAVE_BLASTCMD_LOG=dontcare;
fi
AM_CONDITIONAL(INSTALL_BLASTCMD_LOG, test "$HAVE_BLASTCMD_LOG" = "no")

AC_OUTPUT(Makefile blastcmd-remote)
