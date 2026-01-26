from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
import lifecycle_msgs.msg

def generate_launch_description():
    converter_node = Node(
        package='wheel_odometry_pkg',
        executable='turtlebot4_converter',
        name='turtlebot4_converter',
        output='screen'
    )
    
    wheel_odom_node = LifecycleNode(
        package='wheel_odometry_pkg',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        namespace='',
        output='screen',
        parameters=[{
            'wheel_diameter': 0.24,
            'ticks_per_rev': 580.0,
            'update_rate': 10.0,
            'publish_tf': True
        }]
    )
    
    configure_event = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(wheel_odom_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
                )
            )
        ]
    )
    
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=wheel_odom_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(event=ChangeState(
                            lifecycle_node_matcher=matches_action(wheel_odom_node),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                        ))
                    ]
                )
            ]
        )
    )
    
    return LaunchDescription([
        converter_node,
        wheel_odom_node,
        configure_event,
        activate_event,
    ])