#!/bin/bash

if [ -z "$1" ]
then
  FILENAME="/data/etc/mole.lnk/format"
else
  FILENAME="$1"
fi

awk 'x[$1]++ == 1 { print $1 " is duplicated"}' $FILENAME
