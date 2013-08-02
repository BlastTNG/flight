#!/bin/bash

# lame script to set sync box to Spider settings
# NB: you may need to change TTY

# TTY=/dev/ttySI15
TTY=/dev/ttyS0

stty -F $TTY ispeed 9600 ospeed 9600 cs8 -cstopb -parenb -crtscts
# echo -en 'rl 41\r' > $TTY
echo -en 'rl 50\r' > $TTY
echo -en 'nr 33\r' > $TTY
echo -en 'fr 120\r' > $TTY
