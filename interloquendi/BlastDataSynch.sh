#!/bin/bash
#for mcp systems
#curfile="datafile.cur"
#for decomd systems
curfile="decom.cur"

pid_file="/tmp/bds.pid"

curfile_ptr="/data/etc/$curfile"
addr="submm@hog.astro.utoronto.ca"

echo "Start a persistent connection to $addr"
echo "   ssh $addr"
echo "   remember to have the following lines in your .ssh/config file:"
echo "   	Host *"
echo " 		ControlMaster auto" 
echo "		ControlPath /tmp/%r@%h:%p"


read -p "Press [Enter] to contine..."

#if test -e $pid_file
#then
#  kill -KILL `cat $pid_file ` > /dev/null 2>&1
#  echo "process killed"
#fi

#echo "Opening connection to " $addr
#eval "ssh $addr sleep 525600 &" 
#echo $! > $pid_file
#echo "Connection estabilished"
#sleep 2

lastfile="aaa"

echo $curfile
while true; do
	if [ "$lastfile" != "`cat $curfile_ptr  | sed 's/000//'`" ]; then
		lastfile=`cat $curfile_ptr | sed 's/000//'`
		num=0
		fname=`cat $curfile_ptr  | sed 's/000//' | awk -v num=$num '{printf("%s%03X", $0, num)}'`
		while test -e $fname
		do
			let num=$num+1
			fname=`cat $curfile_ptr | sed 's/000//' | awk -v num=$num '{printf("%s%03X", $0, num)}'`
			echo Skipping File $curfile $fname
		done
		let num=$num-1	
		fname=`cat $curfile_ptr | sed 's/000//' | awk -v num=$num '{printf("%s%03X", $0, num)}'`
		ssh $addr "echo $fname > /data/etc/$curfile"
		rsync -v --progress --append `cat $curfile_ptr | sed 's/000/.spec/'` $addr:/data/rawdir
			
	fi
	
	fname=`cat $curfile_ptr | sed 's/000//' | awk -v num=$num '{printf("%s%03X", $0, num)}'`
	fname_next=`cat $curfile_ptr | sed 's/000//' | awk -v num=$num '{printf("%s%03X", $0, num+1)}'`
	rsync -vz --progress --append $fname $addr:/data/rawdir
	if test -e "$fname_next"; then
		rsync -vz --progress --append $fname $addr:/data/rawdir
		rsync -vz --progress --append $fname_next $addr:/data/rawdir
		let num=$num+1
		fname=$fname_next
	fi
	sleep 1
done


#kill -KILL `cat $pid_file ` > /dev/null 2>&1

#rm -f $pid_file
