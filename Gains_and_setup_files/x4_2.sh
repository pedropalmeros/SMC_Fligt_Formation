#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./MultiAgent_rt
else
	EXEC=./MultiAgent_nrt
fi

. $FLAIR_ROOT/flair-src/scripts/distribution_specific_hack.sh

$EXEC -n x4_2 -a 127.0.0.1 -p 9000 -l ./ -x setup_x4_2.xml -t x4_simu2 -b 127.255.255.255:20010 -d 21002
