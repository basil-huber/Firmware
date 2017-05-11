reset
cd ~/Documents/Dronecourse/PX4_Firmware/build_posix_sitl_lpe/build_gazebo

make
RESULT=$?
cd ~/Documents/Dronecourse/PX4_Firmware/ 

if [ $RESULT -eq 0 ]; then
  make posix_sitl_lpe gazebo_dronecourse
else
  echo Could not compile Gazebo
fi


