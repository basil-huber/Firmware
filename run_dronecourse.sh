# script to compile and run the autopilot and the gazebo simulation
# Created for the EPFL LIS Dronecourse

# Use with "-a" or "-all" to also recompile the gazebo simulation

reset

GAZEBO_COMP_DIR="./build_posix_sitl_lpe/build_gazebo/"
MAIN_DIR="$PWD"

# if option is given and directory exists, recompile gazebo
if ( [ "$1" == "--all" ] || [ "$1" == "-a" ] ) && [ -d $GAZEBO_COMP_DIR ]; then
	# Goto gazebo compilation directory and compile it
	cd $GAZEBO_COMP_DIR
	make
	cd $MAIN_DIR
	# check if make was successful
	RESULT=$?
	if [ $RESULT -ne 0 ]; then
		echo Could not compile Gazebo
		exit 0
	fi
fi

# compile autopilot
make posix_sitl_lpe gazebo_dronecourse
