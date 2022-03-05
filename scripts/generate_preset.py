import roslaunch
import rospy
import argparse
import os
import glob
from generate_esim2d_scenes import create_launch_file

process_generate_running = True

class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    global process_generate_running
    process_generate_running = True
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )
    return launch

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Launches Event Simulator')
    parser.add_argument('config_dir', type=str, help='Path to directory containing simulator config files')
    args = parser.parse_args()

    configs = sorted(glob.glob(os.path.join(args.config_dir, "*config2d.txt")))
    print(configs)
    launch_file_path = "/tmp/esim.launch"

    for config in configs:
        create_launch_file(config, launch_file_path)

        rospy.init_node("remote_launcher")
        launch = init_launch(launch_file_path, ProcessListener())
        launch.start()

        while process_generate_running:
                rospy.sleep(0.05)

        launch.shutdown()
