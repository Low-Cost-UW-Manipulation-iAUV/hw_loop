#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/src/
rsync -avzh ./include/hw_loop/*.hpp 			BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/include/hw_loop/
rsync -avzh CMakeLists.txt 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.xml 								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.launch	 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.yaml		 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.md								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/

echo "All done, Good Success!"