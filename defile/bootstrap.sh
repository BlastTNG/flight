#!/bin/sh

aclocal
automake --add-missing
autoconf configure.ac > configure
autoheader configure.ac > config.h.in
