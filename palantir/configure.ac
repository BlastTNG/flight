dnl Process this file with autoconf to produce a configure script.
AC_INIT(palantir, 1.5.0, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(palantir, 1.5.0)
AM_CONFIG_HEADER(config.h)

AC_ARG_WITH(data-etc,[  --with-data-etc=DIR     put auxiliary data in DIR [[/data/etc]]], DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)

DATA_ETC_PALANTIR_DIR="$DATA_ETC_DIR/palantir"
AC_DEFINE_UNQUOTED(DATA_ETC_PALANTIR_DIR, "$DATA_ETC_PALANTIR_DIR", [ Define to the path to your palantir data directory ])

AC_ARG_WITH([qt-include-dir], [  --with-qt-include-dir=DIR   Qt include directory], QT_INCLUDE_DIR=$withval)
AC_ARG_WITH([qt-lib-dir], [  --with-qt-lib-dir=DIR   Qt library directory], QT_LIB_DIR=$withval)
AC_ARG_WITH([qt-bin-dir], [  --with-qt-bin-dir=DIR   Qt binary directory], QT_BIN_DIR=$withval)

dnl Programs
AC_PROG_CC
AC_PROG_CXX
AC_C_CONST
AC_PROG_INSTALL
AC_CHECK_PROG(SED,sed,yes)
if test "x$SED" = "x"; then
  AC_MSG_ERROR([*** No sed found -- please install first (You are lame) ***])
fi
AC_CHECK_PROG(GREP,grep,yes)
if test "x$GREP" = "x"; then
  AC_MSG_ERROR([*** No grep found -- please install first (You are lame) ***])
fi

dnl X
AC_PATH_X

# Look for Qt
AC_MSG_CHECKING([for Qt])
if test "x$QT_INCLUDE_DIR" = "x" -o "x$QT_LIB_DIR" = "x"; then
  QT_LIBS="qt-mt.so qt.so qt-mt.a qt.a qte-mt.so qte.so qte-mt.a qte.a"
  QT_ROOT="/usr/share/ /usr/lib/ /usr/include/ /usr/bin/ /usr/local/ /usr/local/share/ /usr/local/lib/ /usr/local/include/ /usr/local/bin/" 
  QT_NAME="qt-3.3 qt3.3 qt33 qt-3.2 qt3.2 qt32 qt-3.1 qt3.1 qt31 qt3 qt"
  for q in $QT_ROOT; do
    for i in $QT_NAME; do
      if test -f $q$i/qglobal.h -a "x$QT_INCLUDE_DIR" = "x"; then
        QT_INCLUDE_DIR=$q$i;
      elif test -f $q$i/include/qglobal.h -a "x$QT_INCLUDE_DIR" = "x"; then
        QT_INCLUDE_DIR=$q$i/include;
      elif test -f $q/qglobal.h -a "x$QT_INCLUDE_DIR" = "x"; then
        QT_INCLUDE_DIR=$q;
      fi

      for z in $QT_LIBS; do
        if test -f $q$i/lib$z -a "x$QT_LIB_DIR" = "x"; then
          QT_LIB_DIR=$q$i;
        elif test -f $q$i/lib/lib$z -a "x$QT_LIB_DIR" = "x"; then
          QT_LIB_DIR=$q$i/lib;
        elif test -f $q/lib$z -a "x$QT_LIB_DIR" = "x"; then
          QT_LIB_DIR=$q;
        fi
      done

      if test -f $q$i/moc -a "x$QT_BIN_DIR" = "x"; then
        QT_BIN_DIR=$q$i;
      elif test -f $q$i/bin/moc -a "x$QT_BIN_DIR" = "x"; then
        QT_BIN_DIR=$q$i/bin;
      elif test -f $q/moc -a "x$QT_BIN_DIR" = "x"; then
        QT_BIN_DIR=$q;
      fi
    done
  done
fi
if test "x$QT_INCLUDE_DIR" = "x" ; then
  AC_MSG_ERROR([*** Cannot find Qt includes, specify Qt include dir with --with-qt-include-dir=DIR ***]);
fi
if test "x$QT_LIB_DIR" = "x" ; then
  AC_MSG_ERROR([*** Cannot find Qt libraries, specify Qt library dir with --with-qt-lib-dir=DIR ***]);
fi
if test "x$QT_BIN_DIR" = "x" ; then
  AC_MSG_ERROR([*** Cannot find Qt binaries, specify Qt binary dir with --with-qt-bin-dir=DIR ***]);
fi
AC_MSG_RESULT([libraries $QT_LIB_DIR, headers $QT_INCLUDE_DIR, binaries $QT_BIN_DIR])
QT_CXXFLAGS="$QT_CXXFLAGS -I$QT_INCLUDE_DIR"

#Find Qt Version
AC_MSG_CHECKING([Qt version])
QT_VERSION=`grep 'define.*QT_VERSION_STR\W' $QT_INCLUDE_DIR/qglobal.h | sed 's/^.*"\(.*\)".*$/\1/'`
AC_MSG_RESULT([$QT_VERSION])

#Which Qt Library do we have
AC_MSG_CHECKING([for a Qt library])
QT_LIBS="qt-mt qt qte-mt qte"
for q in $QT_LIBS; do
  if test "x$QT_LIBRARY" = "x" -a -f $QT_LIB_DIR/lib$q.so; then
    QT_LIBRARY=$q;
  elif test "x$QT_LIBRARY" = "x" -a -f $QT_LIB_DIR/lib$q.a; then
    QT_LIBRARY=$q;
  fi
done

if test "x$QT_LIBRARY" = "x" ; then
  AC_MSG_ERROR([*** Cannot find a Qt library in $QT_LIB_DIR ***]);
fi
AC_MSG_RESULT([-l$QT_LIBRARY])

AC_MSG_CHECKING([whether Qt can be staticly linked])
# How are we linking this library?
if `ls $QT_LIB_DIR/lib$QT_LIBRARY* | grep -q '\.a'`; then
  QT_IS_STATIC="yes";
else
  QT_IS_STATIC="no";
fi
AC_MSG_RESULT([$QT_IS_STATIC])
AC_MSG_CHECKING([whether Qt can be dynamically linked])
if `ls $QT_LIB_DIR/lib$QT_LIBRARY* | grep -q '\.so'`; then
  QT_IS_DYNAMIC="yes";
else
  QT_IS_DYNAMIC="no";
fi
AC_MSG_RESULT([$QT_IS_DYNAMIC])

AC_MSG_CHECKING([whether Qt is mulithtreaded])
if `echo $QT_LIBRARY | grep -q -- -mt`; then
  QT_IS_MULTITHREADED="yes"
  QT_CXXFLAGS="$QT_CXXFLAGS -DQT_THREAD_SUPPORT"
else
  QT_IS_MULTITHREADED="no"
fi
AC_MSG_RESULT([$QT_IS_MULTITHREADED])

AC_MSG_CHECKING([whether Qt is embedded])
if `echo $QT_LIBRARY | grep -q qte`; then
  QT_IS_EMBEDDED="yes"
else
  QT_IS_EMBEDDED="no"
fi
AC_MSG_RESULT([$QT_IS_EMBEDDED])
QT_LDADD="$QT_LDADD -L$QT_LIB_DIR -l$QT_LIBRARY"
AC_SUBST(QT_CXXFLAGS)
AC_SUBST(QT_LDADD)

dnl Look for MOC
AC_PATH_PROG(MOC, moc,,$PATH:$QT_BIN_DIR)

dnl Look for Getdata
PKG_CHECK_MODULES([GETDATA], [getdata >= 0.7.0])


dnl Libraries
AC_CHECK_LIB(pthread, pthread_create,,AC_MSG_ERROR([*** libpthread not installed -- please install first ***]))

dnl Output
AC_OUTPUT(data/Makefile src/Makefile Makefile)
