#!/bin/sh
#echo "-----------------------------------";
#echo "Auto starting ClamArm - Correll Lab Arm Manipulator";
#echo "Current IP Address:";
#ifconfig wlan0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}';
echo "-----------------------------------";
echo "Checking serial connections:";
cd /dev;
find dynamixel_**;
echo "-----------------------------------";
echo "Changing permissions:";
#gksudo -m "Input password" 
sudo chmod 777 dynamixel_*;
sudo chmod 777 ttyUSB*;
echo "-----------------------------------";
#read -p "Press any key to continue";
#roslaunch clam_bringup clam_lowlevel.launch