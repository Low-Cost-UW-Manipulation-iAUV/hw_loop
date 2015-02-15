#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						bmt:/home/devel/catkin_ws/src/hw_loop/src/
rsync -avzh ./include/hw_loop/*.hpp 			bmt:/home/devel/catkin_ws/src/hw_loop/include/hw_loop/
rsync -avzh CMakeLists.txt 						bmt:/home/devel/catkin_ws/src/hw_loop/
rsync -avzh *.xml 								bmt:/home/devel/catkin_ws/src/hw_loop/
rsync -avzh ./launch/*.launch					bmt:/home/devel/catkin_ws/src/hw_loop/launch/
rsync -avzh *.yaml		 						bmt:/home/devel/catkin_ws/src/hw_loop/
rsync -avzh *.md								bmt:/home/devel/catkin_ws/src/hw_loop/
rsync -avzh ./urdf/*.urdf						bmt:/home/devel/catkin_ws/src/hw_loop/urdf/

echo "All done, Good Success!"