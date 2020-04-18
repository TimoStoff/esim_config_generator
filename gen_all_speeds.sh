#!/bin/bash
for i in {0..20}
do
   echo "========================================================\n"
   echo "This is the $i th image\n"
   echo "========================================================\n"
   source ~/sim_ws/devel/setup.bash
   ct_mean=0.3
   python scripts/generate_esim2d_scenes.py generator_config/slow_motions.json --scene_id=$i*4+0 --contrast_threshold_mean=$ct_mean --contrast_threshold_sigma=0
   python scripts/generate_esim2d_scenes.py generator_config/fast_motions.json --scene_id=$i*4+1 --contrast_threshold_mean=$ct_mean --contrast_threshold_sigma=0
   python scripts/generate_esim2d_scenes.py generator_config/various_motions.json --scene_id=$i*4+2 --contrast_threshold_mean=$ct_mean --contrast_threshold_sigma=0
   python scripts/generate_esim2d_scenes.py generator_config/medium_motions.json --scene_id=$i*4+3 --contrast_threshold_mean=$ct_mean --contrast_threshold_sigma=0
   python scripts/2d_launch_esim.py --launch_file_path="/tmp/esim.launch"
done

