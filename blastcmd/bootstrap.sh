#!/bin/sh

set -x

if autoconf --version | grep -q 2.13; then
  rm -f configure.in
  sed s/AC_INIT\(.*\)/AC_INIT/ configure.ac > configure.in
  mv configure.ac configure.old
  aclocal 
  automake --add-missing
  autoconf
  autoheader
  mv configure.old configure.ac
else
  aclocal 
  automake --add-missing
  autoconf
  autoheader
fi
