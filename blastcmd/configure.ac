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
  SHARED_CPPFLAGS="-I\${top_srcdir}/../common/include"
else
  AC_MSG_RESULT([not found])
  SVNREPO=`svn info --non-interactive | awk ' /URL/ { print $2 }' | sed -e 's/trunk\/.*/trunk\/common/'`
  AC_MSG_ERROR([*** Cannot find the shared code repository at ../common/;
                Try changing to the parent directory and running:
                svn co $SVNREPO common])
fi
AC_SUBST(SHARED_CPPFLAGS)
AC_SUBST(REMOTEPROJ)

dnl blast config repo
AC_MSG_CHECKING([for BLAST config repository])
if test -d ../blast_config/; then
  AC_MSG_RESULT([found])
  HAVE_BLASTCONFIG=yes
  REMOTEPROJ="$REMOTEPROJ blast_config"
else
  AC_MSG_RESULT([not found])
  SVNREPO=`svn info --non-interactive | awk ' /URL/ { print $2 }' | sed -e 's/trunk\/.*/trunk\/blast_etc/'`
  AC_MSG_WARN([*** Cannot find the BLAST etc repository at ../blast_config/;
               Try changing to the parent directory and running:
               svn co $SVNREPO blast_config])
fi

if test "x$HAVE_BLASTCONFIG$HAVE_MINICONFIG$HAVE_SPIDERCONFIG" = "x"; then
  AC_MSG_ERROR([*** No experimental configuration found])
fi 


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

dnl elog check
AC_ARG_ENABLE([elog], AS_HELP_STRING([--disable-elog],
              [omit elog support from the server. (default: autodetect)]),
              [case ${enableval} in
               (no)  FORCE_ELOG=no ;;
               (*) FORCE_ELOG=yes ;;
             esac], [FORCE_ELOG=])

AC_MSG_CHECKING([whether to enable elog support in the server])
if test "x$FORCE_ELOG" = "x"; then
  if echo ${HOSTNAME} | grep -q itsy; then
    AC_MSG_RESULT([no (Spider flight computer)])
    ENABLE_ELOG=0
  else
    AC_MSG_RESULT([yes (default selected)])
    ENABLE_ELOG=1
  fi
elif test "x$FORCE_ELOG" = "xno"; then
  AC_MSG_RESULT([no (forced)])
  ENABLE_ELOG=0
else
  AC_MSG_RESULT([yes (forced)])
  ENABLE_ELOG=1
fi

if test "x${ENABLE_ELOG}" = "x1"; then
  AC_DEFINE([ENABLE_ELOG], [/**/],
            [Define to enable elog support in the server])
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
AM_CONDITIONAL(INSTALL_BLAST, test "x$HAVE_BLASTCONFIG" = "xyes")
AM_CONDITIONAL(INSTALL_MINICP, test "x$HAVE_MINICONFIG" = "xyes")
AM_CONDITIONAL(INSTALL_SPIDER, test "x$HAVE_SPIDERCONFIG" = "xyes")

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
