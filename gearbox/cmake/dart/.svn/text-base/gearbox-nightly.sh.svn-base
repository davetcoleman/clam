#!/bin/bash

dashboard=$HOME/ctests/gearbox/gearbox-nightly

# compile with gcc-4.2
logfile=$dashboard/gearbox.log
echo ---------------------------------------- >> $logfile
date >> $logfile
echo ---------------------------------------- >> $logfile
# build and test
/usr/bin/ctest -S $dashboard/gearbox-nightly-linux-gcc42.cmake -V >> $logfile 2>&1
# after testing, install so gearbox-dependent nightly tests will work
cd $dashboard/build-gearbox >> $logfile 2>&1
make install >> $logfile 2>&1

