dnl Process this file with autoconf to produce a configure script.
AC_INIT(narsil, 1.8.2, [dwiebe@physics.utoronto.ca])
AM_INIT_AUTOMAKE(narsil, 1.8.2)
AM_CONFIG_HEADER(config.h)

AC_ARG_WITH(data-etc,
            [  --with-data-etc=DIR     put auxiliary data in DIR [[/data/etc]]],
            DATA_ETC_DIR="${withval}", DATA_ETC_DIR="/data/etc")
AC_SUBST(DATA_ETC_DIR)

AC_ARG_WITH(cur-dir,
            [  --with-cur-dir=DIR      read curfiles from DIR [[/mtn/decom/etc]]],
            CUR_DIR="${withval}", CUR_DIR="/mnt/decom/etc")
AC_SUBST(CUR_DIR)
AC_DEFINE_UNQUOTED(CUR_DIR, "$CUR_DIR", [ Define to the path to your curfiles ])

DATA_ETC_NARSIL_DIR="$DATA_ETC_DIR/narsil"
AC_DEFINE_UNQUOTED(DATA_ETC_NARSIL_DIR, "$DATA_ETC_NARSIL_DIR",
                   [ Define to the path to your narsil data directory ])

AC_ARG_WITH([qt-include-dir],
            [  --with-qt-include-dir=DIR   Qt include directory],
            QT_INCLUDE_DIR=$withval)
AC_ARG_WITH([qt-lib-dir], [  --with-qt-lib-dir=DIR   Qt library directory],
            QT_LIB_DIR=$withval)
AC_ARG_WITH([qt-bin-dir], [  --with-qt-bin-dir=DIR   Qt binary directory],
            QT_BIN_DIR=$withval)
AC_ARG_WITH([blastcmd],
            [  --with-blastcmd=NAME    use NAME as the BLASTcmd program (overrides autodetect)],
            FORCE_BLASTCMD=$withval)
AC_ARG_ENABLE([command-list-test],
              [ --disable-command-list-test don't check to see if blastcmd's command list revision matches the one for this package],
              do_cmdlist_test="${enableval}",do_cmdlist_test="yes")

dnl Programs
AC_PROG_CC
AC_PROG_CXX
AC_C_CONST
AC_PROG_INSTALL

AC_CHECK_PROG(FILE,file,yes)
if test "x$FILE" = "x"; then
  AC_MSG_ERROR([*** No file command found -- please install first (You are lame) ***])
fi

AC_CHECK_PROG(SED,sed,yes)
if test "x$SED" = "x"; then
  AC_MSG_ERROR([*** No sed found -- please install first (You are lame) ***])
fi

AC_CHECK_PROG(GREP,grep,yes)
if test "x$GREP" = "x"; then
  AC_MSG_ERROR([*** No grep found -- please install first (You are lame) ***])
fi

AC_MSG_CHECKING(the revision number of the channel list)
CMDVERSION=`grep command_list_serial ${srcdir}/src/command_list.c | gawk '{print $6}'`
AC_MSG_RESULT([$CMDVERSION])
AC_SUBST(CMDVERSION)

dnl X
AC_PATH_X

# Look for Qt
AC_MSG_CHECKING([for Qt])
if test "x$QT_INCLUDE_DIR" = "x" -o "x$QT_LIB_DIR" = x; then
  QT_LIBS="qt-mt.so qt.so qt-mt.a qt.a qte-mt.so qte.so qte-mt.a qte.a"
  QT_ROOT="/usr/share/ /usr/lib/ /usr/include/ /usr/bin/ /usr/local/ /usr/local/share/ /usr/local/lib/ /usr/local/include/ /usr/local/bin/" 
  QT_NAME="qt3.3 qt33 qt3.2 qt32 qt3.1 qt31 qt3 qt"
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
        if test -f $q$i/lib$z -a "x$QT_LIB_DIR = x"; then
          QT_LIB_DIR=$q$i;
        elif test -f $q$i/lib/lib$z -a "x$QT_LIB_DIR = x"; then
          QT_LIB_DIR=$q$i/lib;
        elif test -f $q/lib$z -a "x$QT_LIB_DIR = x"; then
          QT_LIB_DIR=$q;
        fi
      done

      if test -f $q$i/moc -a "x$QT_BIN_DIR = x"; then
        QT_BIN_DIR=$q$i;
      elif test -f $q$i/bin/moc -a "x$QT_BIN_DIR = x"; then
        QT_BIN_DIR=$q$i/bin;
      elif test -f $q/moc -a "x$QT_BIN_DIR = x"; then
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
QT_VERSION=`grep 'define.*QT_VERSION_STR\W' $QTDIR/include/qglobal.h | sed 's/^.*"\(.*\)".*$/\1/'`
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
AC_CHECK_PROG(ELOG, elog, "yes", "no")
if test "x$ELOG" = "xno"; then
  result_elog="no"
else
  result_elog="yes"
  AC_DEFINE(USE_ELOG, 1, [Define to 1 to enable elog support])
fi
  
  

dnl look for blastcmd
if test "x$FORCE_BLASTCMD" = "x"; then
  AC_PATH_PROGS(BLAST_CMD, [blastcmd fifocmd blastcmd-remote dummycmd])
else
  AC_PATH_PROGS(BLAST_CMD, $FORCE_BLASTCMD)
fi

if test "x$BLAST_CMD" = "x"; then
  AC_MSG_ERROR([*** no blastcmd found -- please install blastcmd first ***])
fi
AC_DEFINE_UNQUOTED(BLAST_CMD, "$BLAST_CMD",
                   [ Define to the name of your blastcmd program ])

AC_MSG_CHECKING([whether to skip the command list revision check])
if test "x$do_cmdlist_test" = "xyes"; then
  AC_MSG_RESULT(no)
  AC_MSG_CHECKING([whether we are using the blastcmd binary])
  if `file $BLAST_CMD | grep -q "script"`; then
    BLASTCMD_IS_BLASTCMD=no;
    BLASTCMD_IS_SCRIPT=yes;
  elif `$BLAST_CMD --version | grep -q "^blastcmd"`; then
    BLASTCMD_IS_BLASTCMD=yes;
    BLASTCMD_IS_SCRIPT=no;
    BLASTCMD_TRY_OK=yes;
  else
    BLASTCMD_IS_BLASTCMD=no;
    BLASTCMD_IS_SCRIPT=no;
    BLASTCMD_TRY_OK=yes;
  fi
  AC_MSG_RESULT([$BLASTCMD_IS_BLASTCMD])
  AC_MSG_CHECKING([whether blastcmd is a script])
  AC_MSG_RESULT([$BLASTCMD_IS_SCRIPT])

  if test "$BLASTCMD_IS_SCRIPT" = "yes"; then
    AC_MSG_CHECKING([whether the blastcmd script uses ssh])
    BLASTCMD_SSH_LINE=`grep ssh $BLAST_CMD`;
    if test "x$BLASTCMD_SSH_LINE" = "x"; then
      AC_MSG_RESULT(no)
      BLASTCMD_TRY_OK=yes
    else
      AC_MSG_RESULT(yes)
      AC_MSG_CHECKING([for the host to which blastcmd connects])
      BLASTCMD_HOST=`echo $BLASTCMD_SSH_LINE | gawk '{ print $2}'`
      AC_MSG_RESULT([$BLASTCMD_HOST])
      if test "x$BLASTCMD_HOST" != "x"; then
        AC_MSG_CHECKING([whether configure can connect to the host without a password])
        if ssh -o "BatchMode yes" \
          $BLASTCMD_HOST -n true >/dev/null 2>&1 </dev/null; then
          BLASTCMD_TRY_OK=yes;
          AC_MSG_RESULT([$BLASTCMD_TRY_OK])
        else
          BLASTCMD_TRY_OK=no;
          AC_MSG_RESULT([$BLASTCMD_TRY_OK])
          AC_MSG_WARN([*** configure ca'n't connect to the remote blastcmd ***])
          AC_MSG_WARN([*** command-list revision checking will be disabled ***])
          result_cmdlist="can't ssh to remote host"
        fi
      else
        BLASTCMD_TRY_OK=yes;
      fi
    fi
  fi

  if test "$BLASTCMD_TRY_OK" = "yes"; then
    AC_MSG_CHECKING([whether blastcmd accepts the -c option])
    BLASTCMD_LINE=`$BLAST_CMD -c | grep "Command List Serial"`
    if test "x$BLASTCMD_LINE" = "x"; then
      AC_MSG_RESULT(no)
      AC_MSG_WARN([*** blastcmd does not support command list versioning ***])
      AC_MSG_WARN([*** command-list revision checking will be disabled ***])
      result_cmdlist="not supported by blastcmd"
    else
      AC_MSG_RESULT(yes)
      AC_MSG_CHECKING([the revision number of blastcmd's command list])
      BLASTCMD_CMD_VERSION=`echo "$BLASTCMD_LINE" | gawk '{print $5}'`
      AC_MSG_RESULT([$BLASTCMD_CMD_VERSION])
      if test "`expr $CMDVERSION \> $BLASTCMD_CMD_VERSION`" != "0"; then
        AC_MSG_ERROR([*** The command list provided by blastcmd is too old.  Please upgrade blastcmd before installing this package ***])
      elif test "`expr $CMDVERSION \< $BLASTCMD_CMD_VERSION`" != "0"; then
        AC_MSG_ERROR([*** The command list provided by this package is too old.  Please upgrade this package ***])
      fi
      result_cmdlist="ok"
    fi
  fi
else
  AC_MSG_RESULT(yes)
  result_cmdlist="test skipped"
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

dnl Output
AC_OUTPUT(data/Makefile src/Makefile Makefile)

echo ""
echo "Configuration:"
echo ""
echo "  Install Path:               ${prefix}"
echo "  blastcmd Program:           $BLAST_CMD"
echo "  Command List Revision Test: $result_cmdlist"
echo "  Elog Support:               $ELOG"
