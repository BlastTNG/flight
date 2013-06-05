dnl Process this file with autoconf to produce a configure script.
AC_INIT(defile, 5.0, [dvw@ketiltrout.net])
localstatedir="/var"
AM_INIT_AUTOMAKE
AC_CONFIG_HEADERS(config.h)

AC_ARG_WITH(data-etc, AS_HELP_STRING([--with-data-etc=DIR],
            [put auxiliary data in DIR [[/data/etc]]]),
            DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)

dnl Programs
AC_PROG_CC
AC_PROG_CPP
AC_LANG_C
AC_C_CONST
AC_C_INLINE
AC_PROG_INSTALL
AC_CHECK_PROG(SED,sed,yes)
if test "x$SED" = "x"; then
  AC_MSG_ERROR([*** No sed found -- please install first (You are lame) ***])
fi

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

dnl blast etc repo
AC_MSG_CHECKING([for BLAST etc repository])
if test -d ../blast_etc/; then
  AC_MSG_RESULT([found])
  HAVE_BLASTETC=yes
  REMOTEPROJ="$REMOTEPROJ blast_etc"
else
  AC_MSG_RESULT([not found])
  SVNREPO=`svn info --non-interactive | awk ' /URL/ { print $2 }' | sed -e 's/trunk\/.*/trunk\/blast_etc/'`
  AC_MSG_WARN([*** Cannot find the BLAST etc repository at ../blast/;
               Try changing to the parent directory and running:
               svn co $SVNREPO blast_etc])
fi

dnl spier etc repo
AC_MSG_CHECKING([for Spider etc repository])
if test -d ../spider_etc/; then
  AC_MSG_RESULT([found])
  HAVE_SPIDERETC=yes
  REMOTEPROJ="$REMOTEPROJ spider_etc"
else
  AC_MSG_RESULT([not found])
  SVNREPO=`svn info --non-interactive | awk ' /URL/ { print $2 }' | sed -e 's/trunk\/.*/trunk\/spider_etc/'`
  AC_MSG_WARN([*** Cannot find the Spider etc repository at ../spider/;
               Try changing to the parent directory and running:
               svn co $SVNREPO spider_etc])
fi

if test "x$HAVE_BLASTETC$HAVE_SPIDERETC" = "x"; then
  AC_MSG_WARN([*** No data repositories found])
  AC_MSG_WARN([*** auxiliary files won't be installed])
fi dnl'

dnl Libraries
AC_CHECK_LIB(pthread, pthread_create,,AC_MSG_ERROR([*** libpthread not installed -- please install first ***]))
AC_CHECK_LIB(z, gzopen,,AC_MSG_WARN([*** libz not installed -- defile will have no gzip capabilities ***]))

dnl Headers
AC_HEADER_STDC
AC_CHECK_HEADERS([zlib.h],,,[#])

dnl Typedefs and structures
AC_CHECK_SIZEOF(unsigned short)
AC_CHECK_SIZEOF(unsigned)
AC_CHECK_SIZEOF(unsigned long)

AC_ARG_ENABLE(debug, AS_HELP_STRING([--enable-debug=LEVEL],
              [set debugging level.  --disable-debug is the same as --enable-debug=0 [[0]]]),
              [
               case ${enableval} in
                 yes) enable_debug=1;;
                 no) enable_debug=0;;
                 *) enable_debug=${enableval};;
               esac
              ], [enable_debug="0"])

AC_MSG_CHECKING(whether to enable debugging output)
if test "x${enable_debug}" == "x0"; then
  AC_MSG_RESULT([no])
else
  AC_MSG_RESULT([yes, at level ${enable_debug}])
  AC_DEFINE_UNQUOTED(DEBUG, [${enable_debug}],
                     [Debugging output level or undef for no debugging])
fi

dnl Data
AC_ARG_ENABLE(install-defile-conf,
              AS_HELP_STRING([--enable-install-defile-conf],
                             [install defile.conf even if it already exists.  The old file will be overwritten.]),
              [
               case ${enableval} in
                 no) force_conf="no";;
                 *) force_conf="yes";;
               esac
              ], [force_conf="no"])
              
AC_ARG_WITH(etc,
            AS_HELP_STRING([--with-etc=DIR],[put defile.conf in DIR [[/etc]]]),
            [etcdir="${withval}"],[etcdir="/etc"])

AC_SUBST(etcdir)
AC_DEFINE_UNQUOTED(ETC_DIR,["$etcdir"],
                   [Define to the directory contaning defile.conf])
AC_MSG_CHECKING(whether a defile.conf is already present)
if test -f "$etcdir/defile.conf"; then
  HAVE_DEFILE_CONF=yes;
  INSTALL_DEFILE_CONF=no;
else
  HAVE_DEFILE_CONF=no;
  INSTALL_DEFILE_CONF=yes;
fi
AC_MSG_RESULT([$HAVE_DEFILE_CONF])
AC_MSG_CHECKING(whether to install defile.conf)
if test "${force_conf}" = "yes" -a "${HAVE_DEFILE_CONF}" = "yes"; then
  AC_MSG_RESULT([yes (forced)])
  INSTALL_DEFILE_CONF=yes;
else
  AC_MSG_RESULT([$INSTALL_DEFILE_CONF])
fi
AM_CONDITIONAL(INSTALL_DEFILE_CONF, test "$INSTALL_DEFILE_CONF" = "yes")
AM_CONDITIONAL(INSTALL_BLAST_ETC, test "x$HAVE_BLASTETC" = "xyes")
AM_CONDITIONAL(INSTALL_SPIDER_ETC, test "x$HAVE_SPIDERETC" = "xyes")

AC_OUTPUT(Makefile defile.1.in defile.conf.in)
