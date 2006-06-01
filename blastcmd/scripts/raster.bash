#!/bin/bash

function gothere {
	V=$1
	P=$2
	N=$3
	A=$4
	if [ $N -gt $P ]; then
		T=`expr \( $N - $P \) \/ $V + 1`;
	else
		T=`expr \( $P - $N \) \/ $V + 1`;
	fi
	echo "Jump to $A=$P for $T seconds...";

	blastcmd @ecosmo general $A L2m30l30V${V}A${P}R
	sleep $T
}

function raster {
	START=$1
	END=$2
	MAJOR=$3
	MINOR=$4
	Y=$5
	LX=$START
	if [ $START -gt $END ]; then
		STEP=-4000;
	else
		STEP=4000;
	fi
	echo START=$START END=$END STEP=$STEP
	for (( X = $START; X != $END + $STEP; X += $STEP )); do
		if [ $STEP -lt 0 ]; then
			if [ $X -lt $END ];then
				echo X set to $END because $X -lt $END
				X=$END;
			fi
		else
			if [ $X -gt $END ]; then
				echo X set to $END because $X -gt $END
				X=$END;
			fi
		fi
		gothere $VEL $X $LX $MINOR
		LX=$X
		LY=$Y
		if [ $Y -eq 0 ]; then
			Y=$YMAX;
		else
			Y=0;
		fi
		gothere $VEL $Y $LY $MAJOR
	done
}

VEL=500

OFFS=2000
XMIN=$OFFS
YMIN=$OFFS
XCENT=`expr 39250 - $OFFS`
YCENT=`expr 39250 - $OFFS`
XMAX=`expr 78500 - $OFFS`
YMAX=`expr 78500 - $OFFS`

gothere 1000 $XCENT $XMIN 6
gothere 1000 $YMIN $YMAX 7

echo "X halfraster...";
raster $XCENT $XMAX 7 6 $YMIN
echo "Y raster...";
raster $YMAX $YMIN 6 7 $XMAX
echo "X halfraster...";
raster $XMIN $XCENT 7 6 $YMIN
