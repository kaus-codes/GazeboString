"""ROS launch entry point for the complete supervised Gazebo experiment."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration

def start(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    command=['python3','/project/src/run_integration.py','--phase',value('phase'),'--seed',value('seed'),'--mode',value('mode')]
    if value('gui').lower()=='true':command.append('--gui')
    if value('record').lower()=='true':command.append('--record')
    return [ExecuteProcess(cmd=command,output='screen')]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('phase',default_value='full'),DeclareLaunchArgument('seed',default_value='11'),DeclareLaunchArgument('mode',default_value='standard'),DeclareLaunchArgument('gui',default_value='false'),DeclareLaunchArgument('record',default_value='false'),OpaqueFunction(function=start)])
