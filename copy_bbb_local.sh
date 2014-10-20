#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/src/
rsync -avzh ./include/hw_loop/*.hpp 			BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/include/hw_loop/
rsync -avzh CMakeLists.txt 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.xml 								BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.launch	 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.yaml		 						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.md								BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh ./urdf/*.urdf						BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/urdf/

echo "All done, Good Success!"