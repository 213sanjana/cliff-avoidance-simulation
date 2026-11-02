"""Two isolated instances of the existing robot on the unmodified table world."""
import copy
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_bounds(root):
    """Conservative collision bounds in base_footprint, including joint RPY."""
    def multiply(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(4))
                 for j in range(4)] for i in range(4)]

    def transform(origin):
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        r, p, y = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                                  math.sin(p), math.cos(y), math.sin(y))
        return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
                [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]

    transforms = {'base_footprint': transform(ET.Element('origin'))}
    pending = list(root.findall('joint'))
    while pending:
        for joint in list(pending):
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent in transforms:
                transforms[child] = multiply(transforms[parent], transform(joint.find('origin')))
                pending.remove(joint)
        if pending and not any(j.find('parent').get('link') in transforms for j in pending):
            raise ValueError('Disconnected robot model')
    bounds = [[float('inf'), -float('inf')] for _ in range(3)]
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            matrix = multiply(transforms[link.get('name')], transform(
                collision.find('origin') if collision.find('origin') is not None
                else ET.Element('origin')))
            shape = list(collision.find('geometry'))[0]
            if shape.tag == 'box':
                half = [float(v)/2 for v in shape.get('size').split()]
            elif shape.tag == 'sphere':
                half = [float(shape.get('radius'))] * 3
            elif shape.tag == 'cylinder':
                half = [float(shape.get('radius'))] * 2 + [float(shape.get('length'))/2]
            else:
                raise ValueError('Unsupported robot collision geometry')
            for i in range(3):
                extent = sum(abs(matrix[i][j]) * half[j] for j in range(3))
                bounds[i][0] = min(bounds[i][0], matrix[i][3] - extent)
                bounds[i][1] = max(bounds[i][1], matrix[i][3] + extent)
    return bounds


def _table_bounds(description):
    """Read existing tabletop collision extents in metres."""
    mesh = ET.parse(description / 'models/aws_robomaker_residential_CoffeeTable_01'
                    '/meshes/aws_CoffeeTable_01_collision.DAE').getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    unit = float(mesh.find('c:asset/c:unit', ns).get('meter'))
    values = [float(v) * unit for v in mesh.find(
        './/c:source/c:float_array', ns).text.split()]
    bounds = [(min(values[i::3]), max(values[i::3])) for i in range(3)]
    return bounds


def _platform_rectangles(description):
    bounds = _table_bounds(description)
    world = ET.parse(description / 'worlds/empty_table.world').getroot()
    rectangles = []
    for table in world.findall('world/model'):
        if table.get('name', '').startswith('CoffeeTable'):
            pose = [float(v) for v in table.findtext('pose').split()]
            rectangles.extend([pose[0]+bounds[0][0], pose[0]+bounds[0][1],
                               pose[1]+bounds[1][0], pose[1]+bounds[1][1]])
    return rectangles


def _spawn_positions(description, robot):
    """Fit each robot over an existing interior tabletop."""
    world = ET.parse(description / 'worlds/empty_table.world').getroot()
    bounds = _table_bounds(description)
    robot_bounds = _robot_bounds(robot)
    for axis in (0, 1):
        half_extent = max(abs(v) for v in robot_bounds[axis])
        if bounds[axis][1] - bounds[axis][0] < 2 * half_extent + 0.04:
            raise ValueError('Robot does not fit safely on selected tabletop')
    positions = []
    for name in ('CoffeeTable_1', 'CoffeeTable_1_6'):
        table = world.find(f"world/model[@name='{name}']")
        pose = [float(v) for v in table.findtext('pose').split()]
        assert pose[3:] == [0.0, 0.0, 0.0], 'Expected axis-aligned tables'
        positions.append((pose[0] + sum(bounds[0]) / 2,
                          pose[1] + sum(bounds[1]) / 2,
                          pose[2] + bounds[2][1] - robot_bounds[2][0] + 0.01))
    return positions


def _setup(context):
    description = Path(get_package_share_directory('bot_description'))
    controller = Path(get_package_share_directory('bot_controller'))
    base = ET.fromstring(xacro.process_file(
        str(description / 'urdf/bot.urdf.xacro')).toxml())
    with open(controller / 'config/bot_controllers.yaml', encoding='utf-8') as stream:
        original_config = yaml.safe_load(stream)
    temporary = tempfile.TemporaryDirectory(prefix='two_robot_demo_')
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda context: temporary.cleanup())]))]
    resource_path = str(description / 'models')
    for variable in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        actions.append(SetEnvironmentVariable(
            variable, resource_path + os.pathsep + os.environ.get(variable, '')))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': str(description / 'worlds/empty_table.world')
                          + ' -r -v 3' + (' -s' if LaunchConfiguration(
                              'gui').perform(context).lower() == 'false' else '')}.items()))
    controller_spawners = []
    bridges = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for index, (name, position) in enumerate(zip(
            ('robot1', 'robot2'), _spawn_positions(description, base))):
        root = copy.deepcopy(base)
        root.set('name', name)
        
        
        pose_plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin', {
            'filename': 'ignition-gazebo-pose-publisher-system',
            'name': 'ignition::gazebo::systems::PosePublisher'})
        for key, value in {'publish_model_pose': 'true', 'publish_nested_model_pose': 'true',
                           'publish_link_pose': 'false',
                           'use_pose_vector_msg': 'true', 'update_frequency': '20'}.items():
            ET.SubElement(pose_plugin, key).text = value
        
        
        if index:
            for gazebo in root.findall('gazebo'):
                for system in list(gazebo.findall('plugin')):
                    if system.get('name') == 'ignition::gazebo::systems::Sensors':
                        gazebo.remove(system)
        root.find('ros2_control').set('name', name + '_RobotSystem')
        
        
        root.find("link[@name='base_link']/visual/material/color").set(
            'rgba', ('1 0 0 1', '0 0.4 1 1')[index])
        sensor = root.find("gazebo[@reference='laser_link']/sensor")
        sensor.find('topic').text = '/' + name + '/scan'
        sensor.find('gz_frame_id').text = name + '/laser_link'
        plugin = root.find("gazebo/plugin[@name='ign_ros2_control::IgnitionROS2ControlPlugin']")
        ET.SubElement(plugin, 'robot_param').text = 'robot_description'
        ET.SubElement(plugin, 'robot_param_node').text = 'robot_state_publisher'
        ros = ET.SubElement(plugin, 'ros')
        ET.SubElement(ros, 'namespace').text = '/' + name
        
        
        
        config = {}
        for target in ('robot1', 'robot2'):
            ET.SubElement(ros, 'remapping').text = (
                '/' + target + '/wheel_controller/cmd_vel_unstamped:=/' + target + '/safe_cmd_vel')
            manager = copy.deepcopy(original_config['controller_manager'])
            manager['ros__parameters']['use_sim_time'] = True
            wheel = copy.deepcopy(original_config['wheel_controller'])
            wheel['ros__parameters'].update(
                base_frame_id=target + '/base_footprint', odom_frame_id=target + '/odom',
                tf_frame_prefix_enable=False, use_sim_time=True)
            config['/' + target + '/controller_manager'] = manager
            config['/' + target + '/wheel_controller'] = wheel
        config_path = Path(temporary.name) / (name + '.yaml')
        config_path.write_text(yaml.safe_dump(config), encoding='utf-8')
        plugin.find('parameters').text = str(config_path)
        robot_description = ET.tostring(root, encoding='unicode')
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=name, name='robot_state_publisher',
                            parameters=[{'robot_description': robot_description,
                                         'frame_prefix': name + '/', 'use_sim_time': True}]))
        actions.append(Node(package='ros_gz_sim', executable='create', namespace=name,
                            name='spawn', output='screen', arguments=[
                                '-topic', '/' + name + '/robot_description', '-name', name,
                                '-allow_renaming', 'false', '-x', str(position[0]),
                                '-y', str(position[1]), '-z', str(position[2])]))
        for instance in ('joint_state_broadcaster', 'wheel_controller'):
            controller_spawners.append(Node(package='controller_manager', executable='spawner',
                                namespace=name, name='spawn_' + instance, output='screen',
                                arguments=[instance, '--controller-manager',
                                           '/' + name + '/controller_manager',
                                           '--controller-manager-timeout', '120']))
        actions.append(Node(package='bot_script', executable='edge_detection',
                            namespace=name, name='edge_avoidance_node', output='screen',
                            condition=IfCondition(LaunchConfiguration('edge_avoidance')),
                            parameters=[{'use_sim_time': True}], remappings=[
                                ('/scan', '/' + name + '/scan'),
                                ('/wheel_controller/cmd_vel_unstamped', '/' + name + '/cmd_vel')]))
        bridges.append('/model/' + name + '/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
        bridges.append('/' + name + '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
    
    
    for previous, following in zip(controller_spawners, controller_spawners[1:]):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=previous, on_exit=[following])))
    actions.append(controller_spawners[0])
    bounds = _robot_bounds(base)
    radius = math.hypot(max(abs(v) for v in bounds[0]), max(abs(v) for v in bounds[1]))
    actions.append(Node(package='bot_script', executable='collision_guard',
                        name='two_robot_collision_guard', output='screen',
                        parameters=[{'use_sim_time': True, 'robot_radius': radius,
                                     'platform_rectangles': _platform_rectangles(description)}]))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='two_robot_bridge', arguments=bridges, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('edge_avoidance', default_value='true',
                              description='Run independent autonomous nodes; false for manual control'),
        OpaqueFunction(function=_setup),
    ])
