## About
This code allows generating flying chairs style sequences for the Multi-Object-2D simulator from [ESIM](https://github.com/uzh-rpg/rpg_esim). This code was used to generate sequences for [How to Train Your Event Camera Neural Network](https://timostoff.github.io/20ecnn), please cite this work if you use this in an academic context.
```
@Article{Stoffregen20arxiv,
  author        = {T. Stoffregen, C. Scheerlinck, D. Scaramuzza, T. Drummond, N. Barnes, L. Kleeman, R. Mahoney},
  title         = {How to Train Your Event Camera Neural Network},
  journal       = arxiv,
  year          = 2020,
  month         = march,
  url           = {https://arxiv.org/abs/2003.09078},
  arxivid       = {2003.09078}
}
```

## Prerequisites
You must have esim for this to work, please follow the instructions [here](https://github.com/uzh-rpg/rpg_esim/wiki/Installation). You must have esim sourced (command `ssim`, if you followed the install instructions).

Next, take a look at the config files in `generator_config` and adapt the paths to suit your needs. The image paths in `foreground_images` _must_ all be 4 channel png images, or you will get cryptic errors from ESIM. These are the objects hat will be flying around wildly in the simulator. The image paths in `background_images` _must_ be jpg images, again for mysterious ESIM reasons. Obviously, you must have at least one path with at least `max_num` images in the foreground rubric and at least one path with at least one image for the background rubric.

## Usage
The main work is done in `scripts/generate_esim2d_scenes.py`. This file takes a configuration file (examples can be found in `generator_config`) and some command line arguments that augment/modify the config settings if desired and generates a scene file (this contains the trajectories, the corresponding images, the image size and the sequence duration), an esim config file (this contains contrast thresholds, biases etc) and a ROS launch file.
The default location where these files will be created is `/tmp/000000000_autoscene.txt`, `/tmp/000000000_config2d.txt` and `/tmp/esim.launch` respectively. As an example, you could execute:
```
python scripts/generate_esim2d_scenes.py generator_config/slow_motions.json --scene_id=0 --contrast_threshold_mean=$ct_mean --contrast_threshold_sigma=0.1
```
Note that the CLI arguments for the contrast thresholds are optional and in this case overrule the values in the config file.

Once this is done, you can use `/scripts/2d_launch_esim.py` to launch ROS itself. The required arguments are the location of the launch file, eg: 
```python scripts/2d_launch_esim.py --launch_file_path="/tmp/esim.launch"```

All of this is also in a bash script, so you could also just run `2d_simulator_generator.sh`.
