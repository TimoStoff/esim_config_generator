"""
From the paper 'Stoffregen et al., How to Train Your Event Camera Neural Network'
(https://timostoff.github.io/20ecnn)
"""
import argparse
import numpy as np
import pandas as pd
import json
from collections import OrderedDict
import random
import glob
import os
from shutil import copyfile
from matplotlib import pyplot as plt
from matplotlib import collections  as mc
from matplotlib.patches import Rectangle

PI = 3.1415926

def generate_config_file(output_path_cfg, output_path, contrast_threshold_mean=0.75, ct_diff_sigma=0.1,
                         min_C=0.01, max_C=8, hard_c_t_sigmas=0.0001, refractory_period_ns=1000000,
                         bag_name="/tmp/out.bag", scene_id=0, sim_framerate=100):
    """
    Generate the config file for the simulator (ie event camera settings)
    """
    fcfg = open(output_path_cfg, "w")
    fcfg.write("\n--vmodule=data_provider_online_simple=0" +
               "\n--data_source=1" +
               "\n--path_to_output_bag={}".format(bag_name) +
               "\n--path_to_sequence_file={}".format(output_path) +
               "\n")

    c1 = contrast_threshold_mean
    c2 = np.random.normal(1, ct_diff_sigma)*c1
    if scene_id%2==0:
        contrast_threshold_pos = c1
        contrast_threshold_neg = c2
    else:
        contrast_threshold_pos = c2
        contrast_threshold_neg = c1
    contrast_threshold_pos = min(max(contrast_threshold_pos, min_C), max_C)
    contrast_threshold_neg = min(max(contrast_threshold_neg, min_C), max_C)

    fcfg.write("\n--contrast_threshold_pos={}".format(contrast_threshold_pos) +
               "\n--contrast_threshold_neg={}".format(contrast_threshold_neg) +
               "\n--contrast_threshold_sigma_pos={}".format(hard_c_t_sigmas) +
               "\n--contrast_threshold_sigma_neg={}".format(hard_c_t_sigmas) +
               "\n--refractory_period_ns={}".format(refractory_period_ns) +
               "\n")

    fcfg.write("\n--exposure_time_ms=0.0" +
               "\n--use_log_image=1" +
               "\n--log_eps=0.001" +
               "\n")

    fcfg.write("\n--renderer_type=0" +
               "\n--renderer_preprocess_median_blur=0.0" +
               "\n--renderer_preprocess_gaussian_blur=0.0" +
               "\n")

    fcfg.write("\n--simulation_minimum_framerate={}".format(sim_framerate*1.2) +
               "\n--simulation_imu_rate=1000.0" +
               "\n--simulation_adaptive_sampling_method=1" +
               "\n--simulation_adaptive_sampling_lambda=0.5" +
               "\n")

    fcfg.write("\n--ros_publisher_frame_rate={}".format(sim_framerate) +
               "\n--ros_publisher_depth_rate={}".format(sim_framerate*0) +
               "\n--ros_publisher_optic_flow_rate={}".format(sim_framerate) +
               "\n--ros_publisher_pointcloud_rate={}".format(sim_framerate*0) +
               "\n--ros_publisher_camera_info_rate={}".format(sim_framerate) +
               "\n")

    fcfg.close
    print("Wrote new config file to {}".format(output_path_cfg))


def get_abs_coords_from_vec(vec):
    length = np.abs(vec)
    if length < 1:
        if vec > 0:
            offset = random.uniform(0.0, 1.0-length)
        else:
            offset = random.uniform(length, 1.0)
    else:
        if vec > 0:
            offset = random.uniform(-(length-1), 0)
        else:
            offset = random.uniform(1, length)
    offset -= 0.5
    return offset


def get_random_translations(speed, duration, image_size):
    """
    Get motion vectors through a rectangle of width 1, centered
    at 0, 0 (the virtual image plane), which have a trajectory
    length such that the given speed of the motion occurs, given
    the duration of the sequence. For example, if a very fast motion
    is desired for a long scene, the trajectory is going to be have
    to be huge to respect the desired speed.
    """
    displacement = np.array([speed*duration])/image_size
    edge_limit = 0.45
    rnd_point_in_image = np.array((random.uniform(-edge_limit, edge_limit), random.uniform(-edge_limit, edge_limit)))
    rnd_direction = np.array(random.uniform(0, 2*PI))
    if np.linalg.norm(displacement) < 3.0:
        rnd_speed_component = np.array(random.uniform(0.5, 0.5)*displacement)
    else:
        rnd_speed_component = np.array(random.uniform(0.1, 0.9) * displacement)
    vec = np.array([np.cos(rnd_direction), np.sin(rnd_direction)])

    point_1 = (vec * rnd_speed_component) + rnd_point_in_image
    point_2 = (-vec * (displacement-rnd_speed_component)) + rnd_point_in_image

    return point_1[0], point_1[1], point_2[0], point_2[1]


def get_motion_string(image_name, object_speed, min_ang_vel_deg, max_ang_vel_deg, min_growth, max_growth, mfs=3, gbs=0):
    """
    Given an image and the desired trajectory, return a string that the simulator can read
    """
    x0, y0, x1, y1 = get_random_translations(object_speed, duration, image_size)

    random_angular_v = random.uniform(min_ang_vel_deg, max_ang_vel_deg) * (1 if random.random() < 0.5 else -1)
    angle_to_travel = random_angular_v * duration
    random_start_angle = random.uniform(0, 360)
    theta0 = random_start_angle
    theta1 = random_start_angle + angle_to_travel

    sx0 = random.uniform(min_growth, max_growth)
    sx1 = random.uniform(min_growth, max_growth)
    sy0 = random.uniform(min_growth, max_growth)
    sy1 = random.uniform(min_growth, max_growth)
    return "{} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}\n".format(
        image_name, mfs, gbs, theta0, theta1, x0, x1, y0, y1, sx0, sx1, sy0, sy1), [x0, y0, x1, y1]


def generate_scene_file_multispeed(output_path, image_size, duration, background_images, foreground_images,
                                   number_objects=15, median_filter_size=3, gaussian_blur_sigma=0,
                                   bg_min_ang_velocity_deg=0, bg_max_ang_velocity_deg=5, bg_min_velocity=5,
                                   bg_max_velocity=20, bg_growthmin=1.5, bg_growthmax=2.0,
                                   fg_min_ang_velocity_deg=0, fg_max_ang_velocity_deg=400, fg_min_velocity=5,
                                   fg_max_velocity=500, fg_growthmin=0.5, fg_growthmax=1.5, proportion_variation=0.1,
                                   fully_random=True):
    """
    Given foreground and background image directories and motion parameters, sample some random images
    and give them velocities distributed across the range of velocities given. Velocities are sampled
    linearly across the range, with a small percentage variation, as to guarantee the full 'spread' of motions.
    Otherwise, if you don't want that behaviour, you may set 'fully_random' to True, in which case the speeds
    will be chosen from a uniform distribution between fg_min_velocity and fg_max_velocity.
    """
    f = open(output_path, "w")
    f.write("{} {} {}\n".format(image_size[0], image_size[1], duration))

    background_image_paths = []
    for image_set in background_images:
        background_image_paths.extend(sorted(glob.glob("{}/*.jpg".format(background_images[image_set]['path']))))
    assert(len(background_image_paths) > 0)
    background = random.choice(background_image_paths)

    foreground_image_paths = []
    num_images = np.random.randint(foreground_images['min_num'], foreground_images['max_num'])
    print("{} foreground images".format(num_images))
    for image_set in foreground_images:
        if isinstance(foreground_images[image_set], dict):
            all_paths = sorted(glob.glob("{}/*.png".format(foreground_images[image_set]['path'])))
            selection = random.sample(all_paths, int(num_images*foreground_images[image_set]['proportion']+0.5))
            foreground_image_paths.extend(selection)
    assert(len(foreground_image_paths) > 0)
    random.shuffle(foreground_image_paths)

    #Background
    random_speed = random.uniform(bg_min_velocity, bg_max_velocity)
    f.write(get_motion_string(background, random_speed, bg_min_ang_velocity_deg, bg_max_ang_velocity_deg,
                              bg_growthmin, bg_growthmax, median_filter_size, gaussian_blur_sigma)[0])
    #Foreground
    object_speeds = []
    obj_strings = []
    v_range = np.linspace(fg_min_velocity, fg_max_velocity, len(foreground_image_paths))
    for i, fg in enumerate(foreground_image_paths):
        if fully_random:
            v_vel = np.random.normal(fg_min_velocity, fg_max_velocity)
        else:
            v_vel = v_range[i]*np.random.normal(1, proportion_variation)

        m_string, motion = get_motion_string(fg, v_vel, fg_min_ang_velocity_deg,
                                             fg_max_ang_velocity_deg, fg_growthmin, fg_growthmax,
                                             median_filter_size, gaussian_blur_sigma)
        object_speeds.append(motion)
        obj_strings.append(m_string)

    random.shuffle(obj_strings)
    for obj_string in obj_strings:
        f.write(obj_string)

    f.close
    print("Wrote new scene file to {}".format(output_path))
    return object_speeds


def draw_object_motions(obj_motions):
    """
    For visualization purposes, this will draw the trajectories and the image sensor
    """
    print(obj_motions)
    lines = [[(seg[0], seg[1]), (seg[2], seg[3])] for seg in obj_motions]
    print(lines)
    lc = mc.LineCollection(lines, linewidths=2)
    rect = Rectangle((-0.5, -0.5), 1, 1, linewidth=1,edgecolor='r',facecolor='none')
    fig, ax = plt.subplots()
    ax.add_collection(lc)
    ax.add_patch(rect)
    axlim = 1
    ax.set_xlim([-axlim, axlim])
    ax.set_ylim([-axlim, axlim])
    plt.show()

def create_launch_file(simulator_config_path="/tmp/sim_config.txt", launch_path="/tmp/esim.launch"):
    """
    Generates a roslaunch launch file for ESIM
    """
    print("Saving launch file to {}".format(launch_path))
    with open(launch_path, "w") as f:
        f.write("<launch>\n")
        f.write("\t<node name=\"esim_node\" pkg=\"esim_ros\" type=\"esim_node\" args=\"\n")
        f.write("\t\t--v=1\n")
        f.write("\t\t--vmodule=data_provider_from_folder=10\n")
        f.write("\t\t--flagfile={}\n".format(simulator_config_path))
        f.write("\t\t\" output=\"screen\"/>\n")
        f.write("\t\t\n")
        f.write("\t<include file=\"$(find esim_ros)/launch/visualization.launch\" />\n")
        f.write("\n")
        f.write("</launch>")

def read_json(fname):
    assert(os.path.exists(fname))
    with open(fname) as json_file:
        data = json.load(json_file, object_hook=OrderedDict)
        return data

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Scene file generator')
    parser.add_argument('generator_config', type=str, help='Scene generator settings',
            default="..generator_config/slow_velocity_slow_rotation.json")
    parser.add_argument('--output_path', type=str, help='Path to save scene file', default=None)
    parser.add_argument('--output_path_cfg', type=str, help='Path to save config file', default=None)
    parser.add_argument('--existing_scenes', type=str, help='If you have scene files already,\
            you may pass a file with a list of them and the generator will use these instead', default=None)

    #Scene params
    parser.add_argument('--image_width', type=int, help='Image width (pixels)', default=64)
    parser.add_argument('--image_height', type=int, help='Image height (pixels)', default=64)
    parser.add_argument('--scene_duration', type=float, help='How long should the sequence go\
            (seconds)', default=10.0)
    parser.add_argument('--bag_name', type=str, help='Where to save output bag. If left empty,\
            will save to /tmp/<scene_id>_out.bag', default=None)
    parser.add_argument('--scene_id', type=int, help='ID number, is appended to files saved', default=0)

    #Simulator params
    parser.add_argument('--sim_framerate', type=int, help='Output framerate of the simulator.\
            If left empty, use value in config.', default=None)
    parser.add_argument('--contrast_threshold_mean', type=float, help='CTs will be sampled from\
            a normal dist. with this mean. If left empty, use value in config.', default=None)
    parser.add_argument('--contrast_threshold_sigma', type=float, help='Neg CT will differ to pos CT\
            by NegCT = PosCT * N(1, contrast_threshold_sigma)', default=None)

    args = parser.parse_args()

    config = read_json(args.generator_config)

    output_path = '/tmp/{:09d}_autoscene.txt'.format(args.scene_id) if args.output_path is None else args.output_path
    output_path_cfg = '/tmp/{:09d}_config2d.txt'.format(args.scene_id) if args.output_path_cfg is None else args.output_path_cfg
    image_size = (args.image_width, args.image_height)
    duration = args.scene_duration
    bag_name = '/tmp/{:09d}_out.bag'.format(args.scene_id) if args.bag_name is None else args.bag_name

    #Scene generation
    if args.existing_scenes is None:
        motion_params = config['foreground_params']
        motion_params.update(config['background_params'])
        motions = generate_scene_file_multispeed(output_path, image_size, duration,
                config['background_images'], config['foreground_images'], **motion_params)
    else:
        motions = pd.read_csv(args.existing_scenes, delimiter=',', header=None).values.tolist()[args.scene_id][0]
        copyfile(motions, output_path)

    #Simulator config generation
    camera_params = config['camera_params']
    if args.sim_framerate is not None:
        camera_params['sim_framerate'] = args.sim_framerate
    if args.contrast_threshold_mean is not None:
        camera_params['contrast_threshold_mean'] = args.contrast_threshold_mean
    if args.contrast_threshold_sigma is not None:
        camera_params['ct_diff_sigma'] = args.contrast_threshold_sigma
    generate_config_file(output_path_cfg, output_path, bag_name=bag_name, scene_id=args.scene_id,
            **config['camera_params'])

    #Launch file generation
    create_launch_file(simulator_config_path=output_path_cfg)

    print("Rosbag save path = {}".format(bag_name))
