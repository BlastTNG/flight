#!/bin/sh

FINISHED_OK=0
set -x

if autoconf --version | grep -q 2.13; then
  rm -f configure.in &&
  sed s/AC_INIT\(.*\)/AC_INIT/ configure.ac > configure.in &&
  mv configure.ac configure.old &&
  aclocal &&
  autoheader &&
  automake --add-missing &&
  autoconf &&
  mv configure.old configure.ac &&
  FINISHED_OK=1
else
  aclocal &&
  autoheader &&
  automake --add-missing &&
  autoconf &&
  FINISHED_OK=1
fi

set +x

echo ""
if [ "$FINISHED_OK" == 1 ]; then
  echo "good: bootstrap finished.  Now try ./configure"
else
  echo "oops: bootstrapping errors.  Check your package or try \`cvs update'"
fi
echo ""
