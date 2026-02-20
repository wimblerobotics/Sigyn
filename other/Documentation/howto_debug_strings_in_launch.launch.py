from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition

def foo(context, l, v):
    print(f"{l} {v.perform(context)}")

def generate_launch_description():
    ld = LaunchDescription()
    doit = LaunchConfiguration('doit')
    declare_doit_cmd = DeclareLaunchArgument(
        'doit',
        default_value='True',
        description='Choose your path'
    )
    port=LaunchConfiguration('port')
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Port for serial comm')
    ifNode=ExecuteProcess(
        cmd=['echo', 'True condition'],
        name='true',
        output='screen',
        condition=IfCondition(LaunchConfiguration("doit"))
    )

    context = LaunchContext()
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_doit_cmd)
    ld.add_action(ifNode)
    oa = OpaqueFunction(function=foo, args=['port is:', port])
    ob = OpaqueFunction(function=foo, args=['condition is:', doit])
    ld.add_action(oa)
    ld.add_action(ob)
    return ld
