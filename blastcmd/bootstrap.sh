#!/bin/sh

aclocal
automake --add-missing
autoconf configure.ac > configure
chmod 755 configure
autoheader configure.ac > config.h.in
