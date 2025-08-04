# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

import asyncio
import argparse
import json
import os
import sys
import subprocess
from io import StringIO
from tokenize import Ignore
from typing import cast
from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import PackageNotFoundError
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import is_launch_file
from ros2launch.api import launch_a_launch_file
from ros2launch.api import LaunchFileNameCompleter
from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import print_a_launch_file
from ros2launch.api import print_arguments_of_launch_file
import launch
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.utilities import is_a
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch import Action
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.ros_adapters import get_ros_adapter


# Protocol configuration
USE_JSON_OUTPUT = True  # Set to True to use JSON protocol, False for legacy tab-separated protocol


class LaunchDumperOutput:
    """Handles different output formats for the launch dumper."""
    
    def __init__(self, use_json=True):
        self.use_json = use_json
        self.processes = []
        self.lifecycle_nodes = []
        self.warnings = []
        self.errors = []
        self.info = []
    
    def add_process(self, command, node_name=None, executable=None, arguments=None):
        """Add a process to be launched."""
        if self.use_json:
            self.processes.append({
                "type": "process",
                "command": command,
                "node_name": node_name,
                "executable": executable,
                "arguments": arguments or []
            })
        else:
            # Legacy format: tab-separated command
            print(f'\t{command}')
    
    def add_lifecycle_node(self, node_name, node_namespace, package_name, executable, command=None, arguments=None, parameters=None):
        """Add a lifecycle node."""
        lifecycle_info = {
            "type": "lifecycle_node",
            "node_name": node_name,
            "namespace": node_namespace,
            "package": package_name,
            "executable": executable,
            "command": command,  # Full command line (like ExecuteProcess)
            "arguments": arguments or [],  # Command-line arguments (like ExecuteProcess)  
            "parameters": parameters or {}  # ROS parameters (specific to lifecycle nodes)
        }
        self.lifecycle_nodes.append(lifecycle_info)
    
    def add_warning(self, message):
        """Add a warning message."""
        if self.use_json:
            self.warnings.append(message)
        else:
            print(f"Warning: {message}", file=sys.stderr)
    
    def add_error(self, message):
        """Add an error message."""
        if self.use_json:
            self.errors.append(message)
        else:
            print(f"Error: {message}", file=sys.stderr)
    
    def add_info(self, message):
        """Add an info message."""
        if self.use_json:
            self.info.append(message)
        else:
            print(f"Info: {message}", file=sys.stderr)
    
    def finalize_output(self):
        """Output the final results."""
        if self.use_json:
            output = {
                "version": "1.0",
                "processes": self.processes,
                "lifecycle_nodes": self.lifecycle_nodes,
                "warnings": self.warnings,
                "errors": self.errors,
                "info": self.info
            }
            print(json.dumps(output, indent=2))
        # Legacy format outputs as it goes, so no finalization needed


def parse_launch_arguments(launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
    """Parse the given launch arguments from the command line, into list of tuples for launch."""
    parsed_launch_arguments = {}  # type: ignore # 3.7+ dict is ordered
    for argument in launch_arguments:
        count = argument.count(':=')
        if count == 0 or argument.startswith(':=') or (count == 1 and argument.endswith(':=')):
            raise RuntimeError(
                "malformed launch argument '{}', expected format '<name>:=<value>'"
                .format(argument))
        name, value = argument.split(':=', maxsplit=1)
        parsed_launch_arguments[name] = value  # last one wins is intentional
    return parsed_launch_arguments.items()


def find_files(file_name):
    """Find executable file in PATH or return the original name if not found."""
    try:
        if os.name == 'nt':
            # Windows
            output = subprocess.Popen(['where', file_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate()[0]
            output = output.decode().strip()
            if output:
                search_results = output.split(os.linesep)
                return search_results[0] if search_results[0] else file_name
        else:
            # Unix-like systems
            output = subprocess.Popen(['which', file_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]
            output = output.decode().strip()
            if output:
                return output
    except Exception:
        # If any error occurs, return the original file name
        pass
    
    return file_name


def resolve_executable_path(cmd, context):
    """Resolve executable path like ExecuteProcess does."""
    # Handle substitution objects
    if hasattr(cmd, '__iter__') and not isinstance(cmd, str):
        try:
            cmd_resolved = perform_substitutions(context, normalize_to_list_of_substitutions(cmd))
        except Exception:
            cmd_resolved = str(cmd)
    else:
        cmd_resolved = str(cmd)
    
    # Check if executable exists in PATH
    if os.sep not in cmd_resolved:
        try:
            cmd_resolved = find_files(cmd_resolved)
        except Exception:
            # If find_files fails, keep the original
            pass
    
    return cmd_resolved


def process_execute_process_commands(cmd_list, context):
    """Process ExecuteProcess command list and return command string, executable, and arguments."""
    commands = []
    arguments = []
    first_cmd_resolved = None
    
    if cmd_list:
        # Handle the first command (executable)
        first_cmd = cmd_list[0]
        first_cmd_resolved = resolve_executable_path(first_cmd, context)
        commands.append(f'"{first_cmd_resolved}"')
        
        # Process remaining arguments
        for cmd in cmd_list[1:]:
            if hasattr(cmd, '__iter__') and not isinstance(cmd, str):
                # Handle substitution objects
                try:
                    cmd_resolved = perform_substitutions(context, normalize_to_list_of_substitutions(cmd))
                except Exception:
                    cmd_resolved = str(cmd)
            else:
                cmd_resolved = str(cmd)
            
            if cmd_resolved.strip():
                arguments.append(cmd_resolved.strip())
                commands.append(f'"{cmd_resolved.strip()}"')
    
    return ' '.join(commands), first_cmd_resolved, arguments


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process ROS 2 launch files and extract debugging information.')
    parser.add_argument(
        'launch_full_path',
        help='Full file path to the launch file')
    parser.add_argument(
        'launch_arguments',
        nargs='*',
        help="Arguments to the launch file; '<name>:=<value>' (for duplicates, last one wins)")
    parser.add_argument(
        '--output-format',
        choices=['json', 'legacy'],
        default='json',
        help='Output format: json for structured output, legacy for tab-separated values (default: json)')

    args = parser.parse_args()

    # Initialize output handler
    output_handler = LaunchDumperOutput(use_json=(args.output_format == 'json'))

    # Debug: Check package discovery
    try:
        from ament_index_python.packages import get_packages_with_prefixes
        packages = get_packages_with_prefixes()
        output_handler.add_info(f"Found {len(packages)} packages in AMENT_PREFIX_PATH")
    except Exception as e:
        output_handler.add_error(f"Could not perform package discovery: {e}")

    path = None
    launch_arguments = []

    if os.path.exists(args.launch_full_path):
        path = args.launch_full_path
    else:
        output_handler.add_error('No launch file supplied')
        output_handler.finalize_output()
        sys.exit(1)

    launch_arguments.extend(args.launch_arguments)
    parsed_launch_arguments = parse_launch_arguments(launch_arguments)

    launch_description = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                path
            ),
            launch_arguments=parsed_launch_arguments,
        ),
    ])
    walker = []
    walker.append(launch_description)
    ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
    context = LaunchContext(argv=launch_arguments)
    
    # Handle asyncio event loop properly for newer Python versions
    try:
        # Try to get the current event loop
        loop = asyncio.get_running_loop()
    except RuntimeError:
        # No running loop, create a new one
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    
    context._set_asyncio_loop(loop)

    try:
        # * Here we mimic the run loop inside launch_service,
        #   but without actually kicking off the processes.
        # * Traverse the sub entities by DFS.
        # * Shadow the stdout to avoid random print outputs.
        mystring = StringIO()
        sys.stdout = mystring
        while walker:
            entity = walker.pop()
            try:
                visit_future = entity.visit(context)
                if visit_future is not None:
                    visit_future.reverse()
                    walker.extend(visit_future)
            except Exception as visit_ex:
                # Log visit errors but continue processing
                sys.stdout = sys.__stdout__
                output_handler.add_error(f"Could not visit {type(entity).__name__}: {visit_ex}")
                sys.stdout = mystring
                
                # For Node entities that fail to visit due to package not found,
                # try to extract basic information directly without visiting
                if hasattr(entity, '__class__') and 'Node' in str(entity.__class__):
                    try:
                        # Extract what we can directly from the Node object
                        node_name = None
                        namespace = None
                        package_name = None
                        executable = None
                        
                        if hasattr(entity, 'node_name') and entity.node_name:
                            node_name = str(entity.node_name)
                        if hasattr(entity, 'node_namespace') and entity.node_namespace:
                            namespace = str(entity.node_namespace)
                        if hasattr(entity, 'node_package') and entity.node_package:
                            package_name = str(entity.node_package)
                        if hasattr(entity, 'node_executable') and entity.node_executable:
                            executable = str(entity.node_executable)
                        
                        # Build a basic ros2 run command
                        if package_name and executable:
                            command_parts = ["ros2", "run", package_name, executable]
                            if node_name:
                                command_parts.extend(["--ros-args", "-r", f"__node:={node_name}"])
                            if namespace:
                                command_parts.extend(["-r", f"__ns:={namespace}"])
                            
                            command = " ".join(f'"{part}"' for part in command_parts)
                            
                            output_handler.add_process(
                                command=command,
                                node_name=node_name,
                                executable=executable,
                                arguments=command_parts[1:]  # Everything after 'ros2'
                            )
                            
                            output_handler.add_warning(f"Extracted node info despite problem visiting visit: {package_name}::{executable}")
                    except Exception as extract_ex:
                        output_handler.add_error(f"Could not extract info from failed node: {extract_ex}")
                
                continue

            # Process LifecycleNode actions FIRST (before ExecuteProcess since LifecycleNode inherits from it)
            if is_a(entity, LifecycleNode):
                typed_action = cast(LifecycleNode, entity)
                sys.stdout = sys.__stdout__
                
                try:
                    # Extract lifecycle node information
                    node_name = None
                    namespace = None
                    package_name = None
                    executable = None
                    command = None
                    arguments = []
                    parameters = {}
                    
                    # Get node name
                    if hasattr(typed_action, 'node_name') and typed_action.node_name:
                        node_name = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_name))
                    
                    # Get namespace
                    if hasattr(typed_action, 'node_namespace') and typed_action.node_namespace:
                        namespace = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_namespace))
                    
                    # Get package name
                    if hasattr(typed_action, 'node_package') and typed_action.node_package:
                        package_name = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_package))
                    
                    # Get executable using the same process as ExecuteProcess
                    if hasattr(typed_action, 'process_details') and typed_action.process_details:
                        cmd_list = typed_action.process_details['cmd']
                        command, executable, arguments = process_execute_process_commands(cmd_list, context)
                    elif hasattr(typed_action, 'node_executable') and typed_action.node_executable:
                        # Fallback to node_executable if process_details not available
                        executable = resolve_executable_path(typed_action.node_executable, context)
                        # For consistency, generate a basic command when process_details not available
                        if executable and node_name:
                            command = f"{executable} --ros-args -r __node:={node_name}"
                            arguments = ["--ros-args", "-r", f"__node:={node_name}"]
                    
                    # Get parameters if available
                    if hasattr(typed_action, '_Node__parameters') and typed_action._Node__parameters:
                        for param in typed_action._Node__parameters:
                            if isinstance(param, dict):
                                parameters.update(param)
                    
                    output_handler.add_lifecycle_node(
                        node_name=node_name,
                        node_namespace=namespace,
                        package_name=package_name,
                        executable=executable,
                        command=command,  # Pass the resolved command
                        arguments=arguments,  # Pass the resolved arguments
                        parameters=parameters
                    )
                    
                except Exception as e:
                    output_handler.add_error(f"Could not process LifecycleNode: {e}")
                
                sys.stdout = mystring

            # Process regular Node actions (before ExecuteProcess since Node may create ExecuteProcess)
            elif is_a(entity, Node):
                typed_action = cast(Node, entity)
                sys.stdout = sys.__stdout__
                
                try:
                    # Extract node information
                    node_name = None
                    namespace = None
                    package_name = None
                    executable = None
                    command = None
                    arguments = []
                    parameters = {}
                    
                    # Get node name
                    if hasattr(typed_action, 'node_name') and typed_action.node_name:
                        node_name = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_name))
                    
                    # Get namespace
                    if hasattr(typed_action, 'node_namespace') and typed_action.node_namespace:
                        namespace = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_namespace))
                    
                    # Get package name
                    if hasattr(typed_action, 'node_package') and typed_action.node_package:
                        package_name = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_package))
                    
                    # Get executable
                    if hasattr(typed_action, 'node_executable') and typed_action.node_executable:
                        executable = perform_substitutions(context, normalize_to_list_of_substitutions(typed_action.node_executable))
                    
                    # Build command line for regular Node
                    if executable and package_name:
                        # For regular nodes, build a ros2 run command
                        command_parts = ["ros2", "run", package_name, executable]
                        arguments = ["run", package_name, executable]
                        
                        # Add node name if specified
                        if node_name:
                            command_parts.extend(["--ros-args", "-r", f"__node:={node_name}"])
                            arguments.extend(["--ros-args", "-r", f"__node:={node_name}"])
                        
                        # Add namespace if specified
                        if namespace:
                            command_parts.extend(["-r", f"__ns:={namespace}"])
                            arguments.extend(["-r", f"__ns:={namespace}"])
                        
                        command = " ".join(f'"{part}"' for part in command_parts)
                    
                    # Get parameters if available
                    if hasattr(typed_action, '_Node__parameters') and typed_action._Node__parameters:
                        for param in typed_action._Node__parameters:
                            if isinstance(param, dict):
                                parameters.update(param)
                    
                    # Add as a regular process (not lifecycle)
                    output_handler.add_process(
                        command=command,
                        node_name=node_name,
                        executable=executable,
                        arguments=arguments
                    )
                    
                except Exception as e:
                    output_handler.add_error(f"Could not process Node: {e}")
                
                sys.stdout = mystring

            # Process ExecuteProcess actions (regular ROS nodes)               
            elif is_a(entity, ExecuteProcess):
                typed_action = cast(ExecuteProcess, entity)
                if typed_action.process_details is not None:
                    sys.stdout = sys.__stdout__
                    
                    # Process the command using the extracted function
                    cmd_list = typed_action.process_details['cmd']
                    command, executable, arguments = process_execute_process_commands(cmd_list, context)
                    
                    if command:
                        # Add to output handler
                        output_handler.add_process(
                            command=command,
                            executable=executable,
                            arguments=arguments
                        )
                    
                    sys.stdout = mystring

            # Lifecycle node support
            # https://github.com/ranchhandrobotics/rde-ros-2/issues/632
            # Lifecycle nodes use a long running future to monitor the state of the nodes.
            # cancel this task, so that we can shut down the executor in ros_adapter
            try:
                async_future = entity.get_asyncio_future()
                if async_future is not None:
                    async_future.cancel()
            except Exception:
                # Some entities may not have asyncio futures, that's okay
                pass
        
    except Exception as ex:
        sys.stdout = sys.__stdout__
        output_handler.add_error(f"Could not process launch file: {ex}")
    finally:
        # Ensure stdout is restored
        sys.stdout = sys.__stdout__

    # Output final results
    output_handler.finalize_output()

    # Shutdown the ROS Adapter 
    # so that long running tasks shut down correctly in the debug bootstrap scenario.
    # https://github.com/ranchhandrobotics/rde-ros-2/issues/632
    ros_adapter = get_ros_adapter(context)
    if ros_adapter is not None:
        ros_adapter.shutdown()
