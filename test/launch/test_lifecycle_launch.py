#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Create lifecycle node
    lifecycle_talker = LifecycleNode(
        package='lifecycle',
        executable='lifecycle_talker',
        name='lc_talker',
        namespace='',
        output='screen'
    )
    
    # Regular node
    listener = Node(
        package='demo_nodes_py',
        executable='listener',
        name='listener',
        output='screen'
    )
    
    # Event handler to transition lifecycle node when listener starts
    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=listener,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.get_name() == 'lc_talker',
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ]
        )
    )
    
    # Event emitter to activate after configure
    activate_event_emitter = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.get_name() == 'lc_talker',
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )
    
    return LaunchDescription([
        lifecycle_talker,
        listener,
        configure_event_handler,
        activate_event_emitter,
    ])
