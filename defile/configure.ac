dnl Process this file with autoconf to produce a configure script.
AC_INIT(defile, 3.0.8, [dwiebe@physics.utoronto.ca])
localstatedir="/var"
AM_INIT_AUTOMAKE(defile, 3.0.8)
AM_CONFIG_HEADER(config.h)

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

AC_ARG_ENABLE(debug, AS_HELP_STRING([--enable-debug],
              [enable debugging output [[no]]]),
              [enable_debug="${enableval}"], [enable_debug="no"])

AC_MSG_CHECKING(whether to enable debugging output)
if test "x${enable_debug}" == "xyes"; then
  AC_MSG_RESULT(yes)
  AC_DEFINE(DEBUG, [1], [Define to 1 to enable debugging output])
else
  AC_MSG_RESULT(no)
fi

dnl Data
AC_ARG_WITH(etc, [  --with-etc=DIR     put defile.conf in DIR [/etc]],
            [etcdir="${withval}"],[etcdir="/etc"])

AC_SUBST(etcdir)
AC_DEFINE_UNQUOTED(ETC_DIR,["$etcdir"],
                   [Define to the directory contaning defile.conf])
AC_MSG_CHECKING(whether a defile.conf is already present)
if test -f "$etcdir/defile.conf"; then
  HAVE_DEFILE_CONF=yes;
else
  HAVE_DEFILE_CONF=no;
fi
AC_MSG_RESULT([$HAVE_DEFILE_CONF])
AM_CONDITIONAL(INSTALL_DEFILE_CONF, test "$HAVE_DEFILE_CONF" = "no")

AC_OUTPUT(Makefile defile.1.in defile.conf.in etc/Makefile)
