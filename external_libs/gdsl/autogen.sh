#! /bin/sh

# Run this to generate all the auto-generated files needed by the GNU
# configure program

libtoolize --automake --copy \
&& aclocal \
&& autoconf \
&& autoheader \
&& automake --add-missing --copy
