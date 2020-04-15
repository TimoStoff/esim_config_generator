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

## Usage
The main work is done in `scripts/generate_esim2d_scenes.py`
Run the script 2d_simulator_generator.sh. This does the following:

1: Generates a new launch configuration for the simulator
2: Launches the simulator
3: Runs a script to turn the rosbag into a hdf file

Background images _must_ be jpg for sim to work
Foreground images _must_ be 4-channel png fro sim to work (regardless of whether alpha channel is 'used' or not)
