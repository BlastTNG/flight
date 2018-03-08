#!/bin/sh
#
# Fix up paths in generated files
prefix=$1
topbuild_dir=$2
infile=$3.in
outfile=$3
sed -e "s,@fixup_input\@,${outfile}.  Generated from ${infile} by fixup.sh.," \
-e "s,\${prefix},${prefix}," ${topbuild_dir}/${infile} \
> ${topbuild_dir}/${outfile}
