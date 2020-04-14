import roslaunch
import rospy
import argparse
import os

process_generate_running = True

class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
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
    parser.add_argument('--launch_file_path', type=str, help='Path to load launch file',
                        default="esim2d.launch")

    args = parser.parse_args()
    launch_file_path = os.path.abspath(args.launch_file_path)

    rospy.init_node("remote_launcher")
    launch_file = launch_file_path
    launch = init_launch(launch_file, ProcessListener())
    launch.start()

    while process_generate_running:
            rospy.sleep(0.05)

    launch.shutdown()