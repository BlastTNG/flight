#!/bin/sh

set -x

if autoconf --version | grep -q 2.13; then
  rm -f configure.in
  sed s/AC_INIT\(.*\)/AC_INIT/ configure.ac > configure.in
  mv configure.ac configure.old
  aclocal 
  autoheader
  automake --add-missing
  autoconf
  mv configure.old configure.ac
else
  aclocal 
  autoheader
  automake --add-missing
  autoconf
fi

set +x

echo ""
echo "good, bootstrap finished.  Now try ./configure"
echo ""
