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

	blastcmd general $A L2m30l30V${V}A${P}R
	sleep $T
}

function raster {
	START=$1
	END=$2
	MAJOR=$3
	MINOR=$4
	Y=$5
  YMX=$6
  YMN=$7
	LX=$START
	if [ $START -gt $END ]; then
		STEP=-1000;
	else
		STEP=1000;
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
		if [ $Y -eq $YMN ]; then
			Y=$YMX;
		else
			Y=$YMN;
		fi
		gothere $VEL $Y $LY $MAJOR
	done
}

VEL=1000

XCENT=29120
YCENT=26090
XMIN=`expr $XCENT - 5000`
YMIN=`expr $YCENT - 5000`
XMAX=`expr $XCENT + 5000`
YMAX=`expr $YCENT + 5000`

if [ $RANDOM -gt 16384 ]; then
  YEND1=$YMIN;
  YEND2=$YMAX;
else
  YEND1=$YMAX;
  YEND2=$YMIN;
fi

gothere 1000 $XCENT $XMIN 6
gothere 1000 $YEND1 $YEND2 7

echo "X halfraster...";
raster $XCENT $XMAX 7 6 $YEND2 $YMAX $YMIN
echo "Y raster...";
raster $YMAX $YMIN 6 7 $XMAX $XMAX $XMIN
echo "X halfraster...";
raster $XMIN $XCENT 7 6 $YMIN $YMAX $YMIN
