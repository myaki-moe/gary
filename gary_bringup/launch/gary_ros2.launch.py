import glob
import os
import yaml

from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, TimerAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def check_config_file_exist(context: LaunchContext, robot_type: LaunchConfiguration):
    ret = []

    if not os.path.exists(
            os.path.join(FindPackageShare("gary_bringup").perform(context), "config", robot_type.perform(context))):
        config_not_found_msg = bcolors.WARNING + "\n" \
            "   _____ ____  _   _ ______ _____ _____    _   _  ____ _______    ______ ____  _    _ _   _ _____  \n" \
            "  / ____/ __ \| \ | |  ____|_   _/ ____|  | \ | |/ __ \__   __|  |  ____/ __ \| |  | | \ | |  __ \ \n" \
            " | |   | |  | |  \| | |__    | || |  __   |  \| | |  | | | |     | |__ | |  | | |  | |  \| | |  | |\n" \
            " | |   | |  | | . ` |  __|   | || | |_ |  | . ` | |  | | | |     |  __|| |  | | |  | | . ` | |  | |\n" \
            " | |___| |__| | |\  | |     _| || |__| |  | |\  | |__| | | |     | |   | |__| | |__| | |\  | |__| |\n" \
            "  \_____\____/|_| \_|_|    |_____\_____|  |_| \_|\____/  |_|     |_|    \____/ \____/|_| \_|_____/ \n" \
            + bcolors.ENDC
        ret.append(LogInfo(msg=config_not_found_msg))

    if not os.path.exists(
            os.path.join(FindPackageShare("gary_description").perform(context), "urdf", "ros2_control",
                         robot_type.perform(context) + ".urdf")):
        urdf_not_found_msg = bcolors.WARNING + "\n" \
            "  _    _ _____  _____  ______    _   _  ____ _______    ______ ____  _    _ _   _ _____  \n" \
            " | |  | |  __ \|  __ \|  ____|  | \ | |/ __ \__   __|  |  ____/ __ \| |  | | \ | |  __ \ \n" \
            " | |  | | |__) | |  | | |__     |  \| | |  | | | |     | |__ | |  | | |  | |  \| | |  | |\n" \
            " | |  | |  _  /| |  | |  __|    | . ` | |  | | | |     |  __|| |  | | |  | | . ` | |  | |\n" \
            " | |__| | | \ \| |__| | |       | |\  | |__| | | |     | |   | |__| | |__| | |\  | |__| |\n" \
            "  \____/|_|  \_\_____/|_|       |_| \_|\____/  |_|     |_|    \____/ \____/|_| \_|_____/ \n" \
            + bcolors.ENDC
        ret.append(LogInfo(msg=urdf_not_found_msg))

    gary_logo_msg = bcolors.OKCYAN + "\n" \
            "   _____              _____   __     __    _____     ____     _____   ___  \n" \
            "  / ____|     /\     |  __ \  \ \   / /   |  __ \   / __ \   / ____| |__ \ \n" \
            " | |  __     /  \    | |__) |  \ \_/ /    | |__) | | |  | | | (___      ) |\n" \
            " | | |_ |   / /\ \   |  _  /    \   /     |  _  /  | |  | |  \___ \    / / \n" \
            " | |__| |  / ____ \  | | \ \     | |      | | \ \  | |__| |  ____) |  / /_ \n" \
            "  \_____| /_/    \_\ |_|  \_\    |_|      |_|  \_\  \____/  |_____/  |____|\n" \
            "                                                                           \n" \
            + bcolors.ENDC + \
            "|===============================================================================|\n" \
            "| This program is distributed in the hope that it will be useful, but WITHOUT   |\n" \
            "| ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS |\n" \
            "| FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.|\n" \
            "|===============================================================================|\n"

    ret.append(TimerAction(period=3.0, actions=[LogInfo(msg=gary_logo_msg)]))
    return ret


def glob_nodes(context: LaunchContext, robot_type: LaunchConfiguration):
    nodes = []
    # glob all config file with its package name
    package_dirs = glob.glob(
        os.path.join(FindPackageShare("gary_bringup").perform(context), "config", robot_type.perform(context), "*"))

    # foreach package
    for package_dir in package_dirs:
        if not os.path.isdir(package_dir):
            continue
        # glob all config file with its node name
        config_files_path = glob.glob(os.path.join(package_dir, "*.yaml"))
        # foreach all config file
        for config_file_path in config_files_path:
            # get package_name and config_file without absolute path
            package_name = package_dir.split("/")[-1]
            config_file = config_file_path.split("/")[-1][:-5]
            # exclude ros2_control
            if not os.path.isfile(config_file_path) or package_name == "ros2_control":
                continue
            # load node
            node = ComposableNode(
                package=package_name,
                plugin=package_name + '::' + config_file,
                parameters=[config_file_path]
            )
            nodes.append(node)

    return [ComposableNodeContainer(name="ComponentManager", namespace="",
                                    package='rclcpp_components',
                                    executable='component_container_mt',
                                    composable_node_descriptions=nodes,
                                    )]


def glob_controllers(context: LaunchContext, robot_type: LaunchConfiguration):
    controllers = []

    package_path = os.path.join(
        FindPackageShare("gary_bringup").perform(context), "config", robot_type.perform(context), "ros2_control")

    # check ros2_control dir exists
    if not os.path.isdir(package_path):
        return []

    config_files_path = glob.glob(os.path.join(package_path, "*.yaml"))

    controller_configs = {'controller_manager': {'ros__parameters': {'update_rate': 1000}}}

    # foreach all controller config
    for config_file_path in config_files_path:
        if not os.path.isfile(config_file_path):
            continue

        with open(config_file_path, 'r') as config_file:

            # append controller config
            controller_config = yaml.safe_load(config_file)
            controller_configs.update(controller_config)
            for controller_name in controller_config:
                if controller_name == "controller_manager":
                    continue

                # add controller type to controller manager
                controller_configs['controller_manager']['ros__parameters'][controller_name] = dict(
                    {'type': controller_config[controller_name]['ros__parameters']['controller_type']})

                # add controller spawner
                controllers.append(controller_spawner(controller_name))

    # robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("gary_description"),
                    "urdf",
                    "ros2_control",
                    LaunchConfiguration("robot_type").perform(context) + ".urdf",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    # save combined controller config
    with open("/tmp/launch_params_ros2_control.yaml", 'w') as file:
        yaml.safe_dump(controller_configs, file)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, "/tmp/launch_params_ros2_control.yaml"],
        on_exit=lambda _event, _context: glob_controllers(_context, robot_type)
    )

    controllers.append(controller_manager)

    return controllers


def controller_spawner(controller_name: str, condition=1):
    if condition == 1:
        return Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller_name,
                       "--controller-manager", "/controller_manager",
                       ],
            on_exit=lambda event, context: TimerAction(
                period=0.0,
                actions=[controller_spawner(controller_name, event.returncode)],
            )
        )
    else:
        return LogInfo(msg="{} spawn successfully".format(controller_name.strip()))


def generate_launch_description():
    description = []

    robot_type_arg = DeclareLaunchArgument("robot_type")
    description.append(robot_type_arg)

    config_file_check = OpaqueFunction(function=check_config_file_exist, args=[LaunchConfiguration("robot_type")])
    description.append(config_file_check)

    nodes = OpaqueFunction(function=glob_nodes, args=[LaunchConfiguration("robot_type")])
    description.append(nodes)

    controllers = OpaqueFunction(function=glob_controllers, args=[LaunchConfiguration("robot_type")])
    description.append(controllers)

    return LaunchDescription(description)
