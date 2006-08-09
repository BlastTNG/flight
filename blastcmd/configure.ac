dnl Process this file with autoconf to produce a configure script.
AC_INIT(blastcmd, 2.3.0, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(blastcmd, 2.3.0)
AM_CONFIG_HEADER(config.h)

AC_ARG_WITH([data-etc], AS_HELP_STRING([--with-data-etc=DIR],
            [put auxiliary data in DIR [[/data/etc]]]),
            DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)
AC_DEFINE_UNQUOTED(DATA_ETC_DIR, "$DATA_ETC_DIR",
                   [ Define to the path to your data directory ])

dnl Programs
AC_PROG_CC
AC_LANG_C
AC_C_CONST
AC_PROG_INSTALL

AC_MSG_CHECKING(the revision number of the channel list)
CMDVERSION=`grep command_list_serial ${srcdir}/command_list.c | gawk '{print $6}'`
AC_MSG_RESULT([$CMDVERSION])
AC_SUBST(CMDVERSION)

AC_CHECK_LIB(m,round)

AC_MSG_CHECKING([whether to use IP based authentication])
AC_ARG_ENABLE([authentication], [AS_HELP_STRING([--disable-authentication],
            [don't perform IP based authentication on hosts (daemon only).])],
            [
            case ${enableval} in
              no) use_auth=0 ;;
              *) use_auth=1 ;;
            esac
            ], [use_auth=1])

if test "x$use_auth" = "x1"; then
  AC_MSG_RESULT([yes])
  AC_DEFINE([USE_AUTHENTICATION], [],
            [Define this symbol to use IP based authentication])
else
  AC_MSG_RESULT([no])
fi

dnl See if blastcmd.log is already installed
AC_MSG_CHECKING(whether blastcmd.log is already present)
if test -f "$DATA_ETC_DIR/blastcmd.log"; then
  HAVE_BLASTCMD_LOG=yes;
else
  HAVE_BLASTCMD_LOG=no;
fi
AC_MSG_RESULT([$HAVE_BLASTCMD_LOG])

AM_CONDITIONAL(INSTALL_BLASTCMD_LOG, test "$HAVE_BLASTCMD_LOG" = "no")

AC_OUTPUT(Makefile)
