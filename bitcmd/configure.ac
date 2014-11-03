dnl Process this file with autoconf to produce a configure script.
AC_INIT(blastcmd, 2.3.0, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(blastcmd, 2.3.0)
AM_CONFIG_HEADER(config.h)
AC_CONFIG_FILES(blastcmd-authorize)

AC_ARG_WITH([data-etc], AS_HELP_STRING([--with-data-etc=DIR],
            [put auxiliary data in DIR [[/data/etc]]]),
            DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc/blastcmd")
AC_SUBST(DATA_ETC_DIR)
AC_DEFINE_UNQUOTED(DATA_ETC_DIR, "$DATA_ETC_DIR",
                   [ Define to the path to your data directory ])

dnl common SVN repo
AC_MSG_CHECKING([for shared code repository])
if test -d ../common/; then
  AC_MSG_RESULT([found])
  REMOTEPROJ=common
  MAKEFILE_UPDATE="include ../common/Makefile.update"
  SHARED_CPPFLAGS="-I\${top_srcdir}/../common"
else
  AC_MSG_RESULT([not found])
  SVNREPO=`svn info --non-interactive | awk ' /URL/ { print $2 }' | sed -e 's/trunk\/.*/trunk\/common/'`
  AC_MSG_ERROR([*** Cannot find the shared code repository at ../common/;
                Try changing to the parent directory and running:
                svn co $SVNREPO common])
fi
AC_SUBST(SHARED_CPPFLAGS)
AC_SUBST(REMOTEPROJ)
AC_SUBST(MAKEFILE_UPDATE)

dnl BIT config repo
AC_MSG_CHECKING([for BIT config repository])
if test -d ../bit_config/; then
  AC_MSG_RESULT([found])
  HAVE_BITCONFIG=yes
  REMOTEPROJ="$REMOTEPROJ bit_config"
else
  AC_MSG_RESULT([not found])
  SVNREPO=`svn info --non-interactive | awk ' /URL/ { print $2 }' | sed -e 's/trunk\/.*/trunk\/bit_config/'`
  AC_MSG_WARN([*** Cannot find the BIT config repository at ../bit_config/;
               Try changing to the parent directory and running:
               svn co $SVNREPO bit_config])
fi

if test "x$HAVE_BITCONFIG" = "x"; then
  AC_MSG_ERROR([*** No experimental configuration found])
fi dnl'


dnl Programs
AC_PROG_CC
AC_LANG_C
AC_C_CONST
AC_PROG_INSTALL

AC_CHECK_LIB(m,round)

AC_MSG_CHECKING([whether to use IP based authentication])
AC_ARG_ENABLE([authentication], [AS_HELP_STRING([--disable-authentication],
            [donot perform IP based authentication on hosts (daemon only).])],
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
AM_CONDITIONAL(INSTALL_BIT, test "x$HAVE_BITCONFIG" = "xyes")

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
