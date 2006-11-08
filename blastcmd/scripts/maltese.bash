#!/bin/bash

function gothere {
	V=$1
	P=$2
	N=$3
	A=$4
	if [ $N -gt $P ]; then
		T=`expr 3 \* \( $N - $P \) \/ \( $V \* 5 \)`;
	else
		T=`expr 3 \* \( $P - $N \) \/ \( $V \* 5 \)`;
	fi
	echo "Jump to $A=$N->$P for $T seconds @ V=$V...";

	blastcmd general $A L2m30l30V${V}A${P}R
	sleep $T
}

VEL=200

echo "Enter X centre:"
read XCENT
XMIN=`expr $XCENT - 10000`
XMAX=`expr $XCENT + 10000`
gothere $VEL $XMAX $XCENT 6
gothere $VEL $XMIN $XMAX 6

echo "Enter new X centre:"
read NEWXCENT
gothere $VEL $NEWXCENT $XMIN 6

echo "Enter Y centre:"
read YCENT
YMIN=`expr $YCENT - 10000`
YMAX=`expr $YCENT + 10000`
gothere $VEL $YMAX $YCENT 7
gothere $VEL $YMIN $YMAX 7

echo "Enter new Y centre:"
read NEWYCENT
gothere $VEL $NEWYCENT $YMIN 7

if [ $NEWYCENT -ne $YCENT ]; then
  XCENT=$NEWXCENT
  XMIN=`expr $XCENT - 10000`
  XMAX=`expr $XCENT + 10000`
  gothere $VEL $XMAX $XCENT 6
  gothere $VEL $XMIN $XMAX 6
  gothere $VEL $XCENT $XMIN 6
fi
