#!/bin/bash
for i in {93..280}
do
   echo "========================================================\n"
   echo "This is the $i th image\n"
   echo "========================================================\n"
   rm -rf /tmp/event_data
   python generate_esim2d_scenes.py --background_images=/home/timo/Data2/Coco --foreground_images=/home/timo/Data2/objects --number_objects=40 --bag_name="/tmp/bag_$i" --scene_duration=10.0 --image_width=256 --image_height=256 --iteration_number=$i --sim_framerate=60 
   python 2d_launch_esim.py --launch_file_path="/tmp/esim.launch"
   rm -rf /tmp/event_data
done

