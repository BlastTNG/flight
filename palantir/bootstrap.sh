#!/bin/sh

aclocal
automake --add-missing
autoconf configure.ac > configure
autoheader
