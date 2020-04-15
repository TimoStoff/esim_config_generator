#!/bin/bash
for i in {0..10}
do
   echo "========================================================\n"
   echo "This is the $i th image\n"
   echo "========================================================\n"
   #rm -rf /tmp/event_data
   ct_mean=$(bc <<< "(0.1*$i)+0.1")
   python scripts/generate_esim2d_scenes.py generator_config/slow_motions.json --scene_id=$i --contrast_threshold_mean=$ct_mean --contrast_threshold_sigma=0.1
   python scripts/2d_launch_esim.py --launch_file_path="/tmp/esim.launch"
   #rm -rf /tmp/event_data
done

