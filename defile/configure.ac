dnl Process this file with autoconf to produce a configure script.
AC_INIT(defile, 2.6.0-test1, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(defile, 2.6.0-test1)
AM_CONFIG_HEADER(config.h)

AC_ARG_WITH(data-etc,[  --with-data-etc=DIR     put auxiliary data in DIR [/data/etc]], DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)

dnl Programs
AC_PROG_CC
AC_LANG_C
AC_C_CONST
AC_C_INLINE
AC_PROG_INSTALL

dnl Libraries
AC_CHECK_LIB(pthread, pthread_create,,AC_MSG_ERROR([*** libpthread not installed -- please install first ***]))
AC_CHECK_LIB(z, gzopen,,AC_MSG_WARN([*** libz not installed -- defile will have no gzip capabilities ***]))

dnl Headers
AC_CHECK_HEADERS(zlib.h)

dnl Data
AC_MSG_CHECKING(whether a defile.conf is already present)
if test -f "$sysconfdir/defile.conf"; then
  HAVE_DEFILE_CONF=yes;
else
  HAVE_DEFILE_CONF=no;
fi
AC_MSG_RESULT([$HAVE_DEFILE_CONF])
AM_CONDITIONAL(INSTALL_DEFILE_CONF, test "$HAVE_DEFILE_CONF" = "no")

AC_OUTPUT(Makefile etc/Makefile defile.1 defile.conf)
