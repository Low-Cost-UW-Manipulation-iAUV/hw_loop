#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/src/
rsync -avzh ./include/hw_loop/*.hpp 			BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/include/hw_loop/
rsync -avzh CMakeLists.txt 						BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.xml 								BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh ./launch/*.launch					BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/launch/	 						BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.yaml		 						BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh *.md								BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/
rsync -avzh ./urdf/*.urdf						BBB_BRLWIFI:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/urdf/

echo "All done, Good Success!"