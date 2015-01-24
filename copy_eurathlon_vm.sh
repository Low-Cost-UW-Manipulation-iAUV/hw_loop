#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/src/
rsync -avzh ./include/hw_loop/*.hpp 			eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/include/hw_loop/
rsync -avzh CMakeLists.txt 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/
rsync -avzh *.xml 								eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/
rsync -avzh ./launch/*.launch					BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/hw_loop/launch/	 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/
rsync -avzh *.yaml		 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/
rsync -avzh *.md								eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/
rsync -avzh ./urdf/*.urdf						eurathlon_vm:/home/euratlhon/uwesub_msc/src/hw_loop/urdf/

echo "All done, Good Success!"