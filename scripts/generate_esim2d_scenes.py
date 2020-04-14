# -*- coding: utf-8 -*-
import argparse
import numpy as np
import pandas as pd
import random
import glob
import os
from shutil import copyfile
from matplotlib import pyplot as plt
from matplotlib import collections  as mc
from matplotlib.patches import Rectangle

PI = 3.1415926

def generate_config_file(output_path_cfg, output_path, contrast_threshold_mean=0.75, contrast_threshold_sigma=0.3, ct_diff_sigma=0.1,
                         min_C=0.1, max_C=8, hard_c_t_sigmas=0.0001, refractory_period_ns=1000000,
                         bag_name="/tmp/out.bag", iteration_number=0, sim_framerate=100):
   # # Super high biases
   # contrast_threshold_mean = (iteration_number+1.0)/10.0 #0.4
   # contrast_threshold_sigma = 0.0
   # max_C = 5.0
    print("iter {}: contrast thresh = {}".format(iteration_number, contrast_threshold_mean))
    fcfg = open(output_path_cfg, "w")
    fcfg.write("\n--vmodule=data_provider_online_simple=0" +
               "\n--data_source=1" +
               "\n--path_to_output_bag={}".format(bag_name) +
               "\n--path_to_sequence_file={}".format(output_path) +
               "\n")

    c1 = contrast_threshold_mean
    c2 = np.random.normal(1, ct_diff_sigma)*c1
    if iteration_number%2==0:
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
                                   fg_max_velocity=500, fg_growthmin=0.5, fg_growthmax=1.5):

    f = open(output_path, "w")
    print("fg_max={}".format(fg_max_velocity))
    f.write("{} {} {}\n".format(image_size[0], image_size[1], duration))

    #Background
    background = random.choice(background_images)
    random_speed = random.uniform(bg_min_velocity, bg_max_velocity)
    f.write(get_motion_string(background, random_speed, bg_min_ang_velocity_deg, bg_max_ang_velocity_deg,
                              bg_growthmin, bg_growthmax, median_filter_size, gaussian_blur_sigma)[0])

    #Foreground
    coco_images = sorted(glob.glob("{}/*.png".format("/home/timo/Data2/coco_png")))
    num_coco = int(number_objects*0.5)
    coco_fg = random.sample(coco_images, num_coco)
    foreground = random.sample(foreground_images, number_objects-num_coco)

    # print(foreground)
    object_speeds = []
    obj_strings = []
    for i, fg in enumerate(foreground):
        v_vel = np.random.normal(fg_min_velocity, fg_max_velocity)
        m_string, motion = get_motion_string(fg, v_vel, fg_min_ang_velocity_deg,
                                             fg_max_ang_velocity_deg, fg_growthmin, fg_growthmax,
                                             median_filter_size, gaussian_blur_sigma)
        object_speeds.append(motion)
        obj_strings.append(m_string)
    for i, fg in enumerate(coco_fg):
        v_vel = np.random.normal(fg_min_velocity, fg_max_velocity)
        m_string, motion = get_motion_string(fg, v_vel, fg_min_ang_velocity_deg,
                                             fg_max_ang_velocity_deg, 0.2, 0.6,
                                             median_filter_size, gaussian_blur_sigma)
        object_speeds.append(motion)
        obj_strings.append(m_string)

    increment = (fg_max_velocity-fg_min_velocity)/(number_objects-1)
    speeds = np.linspace(fg_min_velocity, fg_max_velocity, number_objects)

    random.shuffle(obj_strings)
    for obj_string in obj_strings:
        f.write(obj_string)

    f.close
    print("Wrote new scene file to {}".format(output_path))
    return object_speeds


def generate_scene_file_single_speed(
        output_path,
        image_size,
        duration,
        background_images,
        foreground_images,
        number_objects,
        bg_rotation=45,
        bg_translation=0.1,
        bg_growthmin=1.0,
        bg_growthmax=2.0,
        fg_growthmin=0.0,
        fg_growthmax=1.5,
        fg_translation=0.8,
        fg_rotation=300,
        median_filter_size=3,
        gaussian_blur_sigma=0):

    f = open(output_path, "w")
    f.write("{} {} {}\n".format(image_size[0], image_size[1], duration))
    # Background
    background = random.choice(background_images)
    f.write(get_motion_string(background, bg_rotation, bg_translation, bg_growthmin, bg_growthmax,
                              median_filter_size, gaussian_blur_sigma))

    #Foreground
    foreground = random.sample(foreground_images, number_objects)
    for fg in foreground:
        f.write(get_motion_string(fg, fg_rotation, fg_translation, fg_growthmin, fg_growthmax,
                                  median_filter_size, gaussian_blur_sigma))
    f.close
    print("Wrote new scene file to {}".format(output_path))


def draw_object_motions(obj_motions):
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

def generate_archtype_sequence(iteration, output_path_cfg, output_path, bag_name, sim_framerate):
    min_ct = 0.1
    max_ct = 1.5
    num_seq = 280
    ct_mean = (max_ct-min_ct)/(num_seq*1.0)*iteration+min_ct
    generate_config_file(output_path_cfg, output_path, bag_name=bag_name, iteration_number=iteration,
        sim_framerate=sim_framerate, contrast_threshold_mean=ct_mean, contrast_threshold_sigma=0.0)
    params = {}
    if iteration%4==0:
        #Slow background with 0-4 slow speed objects
        num_obj = int(random.uniform(1, 6))
        params['number_objects'] = num_obj
        params['bg_min_ang_velocity_deg'] = 0
        params['bg_max_ang_velocity_deg'] = 5
        params['bg_min_velocity'] = 0.5
        params['bg_max_velocity'] = 10
        params['bg_growthmin'] = 1.0
        params['bg_growthmax'] = 1.5
        params['fg_min_ang_velocity_deg'] = 0
        params['fg_max_ang_velocity_deg'] = 5.0
        params['fg_min_velocity'] = 0.5
        params['fg_max_velocity'] = 30.0
    elif iteration%4==1:
       #Slow background with 5-20 med-fast speed objects
        num_obj = int(random.uniform(5, 20))
        params['number_objects'] = num_obj
        params['bg_min_ang_velocity_deg'] = 0
        params['bg_max_ang_velocity_deg'] = 15
        params['bg_min_velocity'] = 5
        params['bg_max_velocity'] = 25
        params['bg_growthmin'] = 1.0
        params['bg_growthmax'] = 1.5
        params['fg_min_ang_velocity_deg'] = 10
        params['fg_max_ang_velocity_deg'] = 45.0
        params['fg_min_velocity'] = 30.0
        params['fg_max_velocity'] = 350.0
    elif iteration%4==2:
        #Slow background with 5-10 medium speed objects
        num_obj = int(random.uniform(5, 10))
        params['number_objects'] = num_obj
        params['bg_min_ang_velocity_deg'] = 0
        params['bg_max_ang_velocity_deg'] = 15
        params['bg_min_velocity'] = 5
        params['bg_max_velocity'] = 25
        params['bg_growthmin'] = 1.0
        params['bg_growthmax'] = 1.5
        params['fg_min_ang_velocity_deg'] = 0
        params['fg_max_ang_velocity_deg'] = 15.0
        params['fg_min_velocity'] = 20.0
        params['fg_max_velocity'] = 80.0
    elif iteration%4==3:
        #Slow background with 1-20 unconstrained objects
        num_obj = int(random.uniform(10, 30))
        params['number_objects'] = num_obj
        params['bg_min_ang_velocity_deg'] = 0
        params['bg_max_ang_velocity_deg'] = 45
        params['bg_min_velocity'] = 0.1
        params['bg_max_velocity'] = 40
        params['bg_growthmin'] = 1.0
        params['bg_growthmax'] = 1.5
        params['fg_min_ang_velocity_deg'] = 0
        params['fg_max_ang_velocity_deg'] = 60.0
        params['fg_min_velocity'] = 0.1
        params['fg_max_velocity'] = 600.0
    params['fg_growthmin'] = 0.4
    params['fg_growthmax'] = 1.2
    return params

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Scene file generator')
    parser.add_argument('--background_images', type=str, help='Background image folder path',
                        default="/home/timo/Data2/Coco")
    parser.add_argument('--foreground_images', type=str, help='Foreground image folder', default="../objects")
    parser.add_argument('--number_objects', type=int, help='Number of foreground images', default=6)
    parser.add_argument('--output_path', type=str, help='Path to save scene file', default="/tmp/autoscene.txt")
    parser.add_argument('--output_path_cfg', type=str, help='Path to save config file', default="/tmp/config2d.txt")
    parser.add_argument('--image_width', type=int, help='Image width', default=128)
    parser.add_argument('--image_height', type=int, help='Image height', default=128)
    parser.add_argument('--scene_duration', type=float, help='How long should the sequence go', default=1.0)
    parser.add_argument('--bag_name', type=str, help='Which bag number is this - used for naming', default=0)
    parser.add_argument('--iteration_number', type=int, help='Which iteration of the simulator is this?')
    parser.add_argument('--sim_framerate', type=int, help='Framerate of the simulator')
    parser.add_argument('--existing_scenes', type=str, help='A file with a list of scenes to use, if it exists', default=None)

    args = parser.parse_args()
    background_images = args.background_images
    foreground_images = os.path.abspath(args.foreground_images)
    number_objects = args.number_objects

    output_path = '/tmp/{:09d}_autoscene.txt'.format(args.iteration_number)
    output_path_cfg = '/tmp/{:09d}_config2d.txt'.format(args.iteration_number)
    image_size = (args.image_width, args.image_height)
    duration = args.scene_duration
    bag_name = '/tmp/{:09d}_out.bag'.format(args.iteration_number)
    print("saving bag as {}".format(bag_name))

    background_images = sorted(glob.glob("{}/*.jpg".format(background_images)))
    foreground_images = sorted(glob.glob("{}/*.png".format(foreground_images)))

    iteration = args.iteration_number
    fg_params = generate_archtype_sequence(iteration, output_path_cfg, output_path, bag_name=bag_name, sim_framerate=args.sim_framerate)
    print(fg_params)

    if args.existing_scenes is None:
        motions = generate_scene_file_multispeed(output_path, image_size, duration, background_images, foreground_images, **fg_params)
    else:
        motions = pd.read_csv(args.existing_scenes, delimiter=',', header=None).values.tolist()[args.iteration_number][0]
        copyfile(motions, output_path)
    create_launch_file(simulator_config_path=output_path_cfg)
