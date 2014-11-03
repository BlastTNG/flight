dnl Process this file with autoconf to produce a configure script.
AC_INIT(narsil, 2.5.0, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(narsil, 2.5.0)
AM_CONFIG_HEADER(config.h)

AC_ARG_WITH(data-etc, AS_HELP_STRING([--with-data-etc=DIR],
            [put auxiliary data in DIR [[/data/etc]]]),
            DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)

AC_ARG_WITH(cur-dir, AS_HELP_STRING([--with-cur-dir=DIR],
            [read curfiles from DIR [[/mtn/decom/etc]]]),
            CUR_DIR="${withval}", CUR_DIR="/data/etc")
AC_SUBST(CUR_DIR)
AC_DEFINE_UNQUOTED(CUR_DIR, "$CUR_DIR", [Define to the path to your curfiles ])

DATA_ETC_NARSIL_DIR="$DATA_ETC_DIR/narsil"
AC_DEFINE_UNQUOTED(DATA_ETC_NARSIL_DIR, "$DATA_ETC_NARSIL_DIR",
                   [Define to the path to your narsil data directory])

AC_ARG_WITH([qt-include-dir], AS_HELP_STRING([--with-qt-include-dir=DIR],
            [Qt include directory]), QT_INCLUDE_DIR=$withval)
AC_ARG_WITH([qt-lib-dir], AS_HELP_STRING([--with-qt-lib-dir=DIR],
            [Qt library directory]), QT_LIB_DIR=$withval)
AC_ARG_WITH([qt-bin-dir], AS_HELP_STRING([--with-qt-bin-dir=DIR],
            [Qt binary directory]), QT_BIN_DIR=$withval)

dnl Programs
AC_PROG_CC
AC_PROG_CXX
AC_C_CONST
AC_PROG_INSTALL

dnl X
AC_PATH_X

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

# Look for Qt
AC_MSG_CHECKING([for Qt])
if test "x$QT_INCLUDE_DIR" = "x" -o "x$QT_LIB_DIR" = "x"; then
  QT_LIBS="qt-mt.so qt.so qt-mt.a qt.a qte-mt.so qte.so qte-mt.a qte.a"
  QT_ROOT="/usr/share/ /usr/lib/ /usr/include/ /usr/bin/ /usr/local/ /usr/local/share/ /usr/local/lib/ /usr/local/include/ /usr/local/bin/" 
  QT_NAME="qt-3.3 qt3.3 qt33 qt3.2 qt32 qt3.1 qt31 qt3 qt"
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

dnl Look for ELOG
AC_ARG_WITH(elog, AS_HELP_STRING([--without-elog],
             [Don't compile in elog support]),dnl'
             [
              case ${withval} in
                no) use_elog=0;;
                *) use_elog=1;;
              esac
              ], [ use_elog=1 ])
                              
AC_ARG_WITH(elog-host, AS_HELP_STRING([--with-elog-host=HOST],
            [Send elog queries to the elog server running on HOST [[elog.blast]]]),
            ELOG_HOST="${withval}", ELOG_HOST="elog.blast")
AC_ARG_WITH(elog-port, AS_HELP_STRING([--with-elog-port=PORT],
            [Send elog queries to the elog server running on PORT [[8000]]]),
            ELOG_PORT="${withval}", ELOG_PORT="8000")
AC_MSG_CHECKING(whether to support elog)
if test "x$use_elog" = "x1"; then
  AC_MSG_RESULT([yes])

  AC_ARG_VAR(ELOG, [elog client program])
  AC_PATH_PROGS(ELOG, [elog], "no")
  if test "x$ELOG" = "xno"; then
    result_elog="no"
  else
    result_elog="yes"
    AC_DEFINE_UNQUOTED(ELOG, "$ELOG",
                       [Define to the path of your elog program])
    AC_DEFINE_UNQUOTED(ELOG_HOST, "$ELOG_HOST",
                       [Define to the name of your elog server])
    AC_DEFINE_UNQUOTED(ELOG_PORT, "$ELOG_PORT",
                       [Define to the port used by your elog server])
    AC_DEFINE(USE_ELOG, 1, [Define to 1 to enable elog support])
  fi
else
  AC_MSG_RESULT([no])
fi

dnl Libraries
AC_CHECK_LIB(pthread, pthread_create,,
             AC_MSG_ERROR([*** libpthread not installed -- please install first ***]))

dnl log file check
AC_MSG_CHECKING(whether the narsil log file is already present)
if test -f "$DATA_ETC_DIR/narsil/log.txt"; then
  HAVE_LOG_TXT=yes;
else
  HAVE_LOG_TXT=no;
fi
AC_MSG_RESULT([$HAVE_LOG_TXT])
AM_CONDITIONAL(INSTALL_LOG_TXT, test "$HAVE_LOG_TXT" = "no")

dnl BLASTcmd Host
AC_ARG_WITH([blastcmd-host], AS_HELP_STRING([--with-blastcmd-host=HOST],
            [connect to the blastcmd daemon running on HOST [[widow]]]),
            BLASTCMD_HOST="${withval}", BLASTCMD_HOST="widow")

AC_ARG_ENABLE(cmd-line, AS_HELP_STRING([--enable-cmd-line],
              [enable the command line pre-processor]),
              [
               case ${enableval} in
                 no) NARSILPP="no";;
                 *) NARSILPP="yes";;
               esac
               ], [NARSILPP="no"])
AC_MSG_CHECKING([whether to enable the command line preprocessor])
AC_MSG_RESULT([$NARSILPP])
AM_CONDITIONAL(NARSILPP, test "$NARSILPP" = "yes")

AC_MSG_CHECKING([for the name of the blastcmd host])
AC_MSG_RESULT([$BLASTCMD_HOST])
AC_DEFINE_UNQUOTED(BLASTCMD_HOST, "$BLASTCMD_HOST",
                   [ Define to the default blastcmd host ])
AC_SUBST(BLASTCMD_HOST)

dnl Output
AC_OUTPUT(data/Makefile src/Makefile Makefile)
