import rclpy
from mcp.server.fastmcp import FastMCP
import requests
import subprocess
import json
from typing import List, Dict, Any, AsyncGenerator
import asyncio
import logging
import time
import os
import argparse
from datetime import datetime

import rclpy.node

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Create an MCP server
mcp = FastMCP(name = "ros2_mcp", 
              debug=True)

rclpy.init(args=None)

# Create a node
rclpy_node = rclpy.create_node("ros2_mcp")

# Global state for topic monitoring
active_monitors = {}  # topic_name -> {"task": asyncio.Task, "subscription": subscription, "start_time": float, "last_message": dict, "topic_type": str}



# Returns a list of running ROS nodes
@mcp.tool()
async def get_nodes() -> Dict[str, List[str]]:
    """Returns a list of running ROS nodes"""
    # Get the list of node names
    all_node_names = rclpy_node.get_node_names()
    
    # Filter out meta nodes (nodes starting with underscore or system nodes)
    filtered_nodes = []
    for node_name in all_node_names:
        # Skip nodes that start with underscore (meta nodes)
        if node_name.startswith('_'):
            continue
        # Skip common system/meta nodes
        if node_name in ['parameter_events', 'ros2cli']:
            continue
        filtered_nodes.append(node_name)
    
    # Return the filtered list of nodes
    return {"nodes": filtered_nodes}

# Returns information about a given ROS node by name
@mcp.tool()
async def get_node_info(node_name: str) -> dict:
    """Returns information about a given ROS node by name"""
    # Get the list of node names
    node_names = rclpy_node.get_node_names()

    # Check if the node name is valid
    if node_name not in node_names:
        raise ValueError(f"Node '{node_name}' not found")

    node = rclpy.node.Node(node_name)

    # Get the node info
    info = {
        "name": node.get_name(),
        "namespace": node.get_namespace(),
        "topics": node.get_topic_names_and_types(),
        "services": node.get_service_names_and_types(),
        "parameters": node.get_parameters([])
    }

    # Return the node info
    return info

# --- Topic Commands ---

@mcp.tool()
async def list_topics(show_types: bool = False) -> List[Dict[str, str]]:
    """
    Lists available ROS topics
    
    Args:
        show_types: Whether to include topic types in the result
        
    Returns:
        List of topics with optional type information
    """
    topic_names_and_types = rclpy_node.get_topic_names_and_types()
    
    topics = []
    for topic_name, topic_types in topic_names_and_types:
        if show_types and topic_types:
            topics.append({"name": topic_name, "type": topic_types[0]})
        else:
            topics.append({"name": topic_name})
    
    return topics

@mcp.tool()
async def get_topic_info(topic_name: str) -> Dict[str, Any]:
    """
    Get information about a ROS topic
    
    Args:
        topic_name: Name of the topic to get info about
        
    Returns:
        Dictionary with topic information
    """
    # Get topic type using rclpy
    topic_names_and_types = rclpy_node.get_topic_names_and_types()
    topic_type_map = {name: types[0] if types else None for name, types in topic_names_and_types}
    
    topic_type = topic_type_map.get(topic_name)
    
    if topic_type is None:
        raise RuntimeError(f"Topic '{topic_name}' not found")
    
    # Get publisher and subscriber counts using rclpy
    publishers_info = rclpy_node.get_publishers_info_by_topic(topic_name)
    subscriptions_info = rclpy_node.get_subscriptions_info_by_topic(topic_name)
    
    return {
        "name": topic_name,
        "type": topic_type,
        "publisher_count": len(publishers_info),
        "subscription_count": len(subscriptions_info)
    }

@mcp.tool()
async def publish_to_topic(topic_name: str, topic_type: str, message: str) -> Dict[str, Any]:
    """
    Publish a message to a topic
    
    Args:
        topic_name: Name of the topic to publish to
        topic_type: Type of the topic
        message: Message to publish in YAML format
        
    Returns:
        Status of the publish operation
    """
    cmd = ["ros2", "topic", "pub", "--once", topic_name, topic_type, message]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to publish to topic: {result.stderr}")
    
    return {
        "success": True,
        "topic": topic_name,
        "type": topic_type,
        "message": message
    }

@mcp.tool()
async def start_monitoring_topic(topic_name: str) -> Dict[str, Any]:
    """
    Start monitoring a topic and stream messages via MCP logging
    
    Args:
        topic_name: Name of the topic to monitor
        
    Returns:
        Status of the monitoring start operation
    """
    import asyncio
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    
    # Check if already monitoring this topic
    if topic_name in active_monitors:
        return {
            "success": False,
            "topic": topic_name,
            "message": "Topic is already being monitored"
        }
    
    # Get the topic type using rclpy
    topic_names_and_types = rclpy_node.get_topic_names_and_types()
    
    # Create a lookup dictionary for faster access
    topic_type_map = {name: types[0] if types else None for name, types in topic_names_and_types}
    
    topic_type_str = topic_type_map.get(topic_name)
    
    if topic_type_str is None:
        raise RuntimeError(f"Topic '{topic_name}' not found or has no type information")
    
    # Import the message type dynamically
    try:
        package_name, msg_type = topic_type_str.split('/', 1)
        module_name = package_name + '.msg'
        msg_class_name = msg_type.split('/')[-1]
        
        msg_module = __import__(module_name, fromlist=[msg_class_name])
        msg_class = getattr(msg_module, msg_class_name)
    except Exception as e:
        raise RuntimeError(f"Failed to import message type {topic_type_str}: {str(e)}")
    
    # Create a queue for message passing
    message_queue = asyncio.Queue()
    
    # Callback to put messages in the queue
    def message_callback(msg):
        try:
            message_data = {
                "timestamp": rclpy_node.get_clock().now().to_msg().sec + rclpy_node.get_clock().now().to_msg().nanosec / 1e9,
                "topic": topic_name,
                "type": topic_type_str,
                "data": str(msg)
            }
            message_queue.put_nowait(message_data)
            
            # Store the last message for this topic
            if topic_name in active_monitors:
                active_monitors[topic_name]["last_message"] = message_data
            
        except Exception as e:
            logger.error(f"Error processing message: {e}")
    
    # Create subscription with best effort QoS for streaming
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        depth=10
    )
    
    subscription = rclpy_node.create_subscription(
        msg_class,
        topic_name,
        message_callback,
        qos_profile
    )
    
    # Create monitoring task
    async def monitor_task():
        message_count = 0
        try:
            while True:
                try:
                    # Wait for a message with a short timeout
                    message = await asyncio.wait_for(message_queue.get(), timeout=1.0)
                    message_count += 1
                    
                    # Send message via MCP progress to the LLM
                    timestamp_str = datetime.fromtimestamp(message["timestamp"]).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    mcp.progress(
                        token=f"topic_monitor_{topic_name}",
                        value=message_count,
                        message=f"[{timestamp_str}] {json.dumps(message)}"
                    )
                    
                except asyncio.TimeoutError:
                    # Check if we should stop monitoring
                    if topic_name not in active_monitors:
                        break
                    continue
        except Exception as e:
            logger.error(f"Error in monitoring task for {topic_name}: {e}")
        finally:
            logger.info(f"Stopped monitoring {topic_name} after {message_count} messages")
    
    # Start the monitoring task
    task = asyncio.create_task(monitor_task())
    
    # Store the monitor info
    active_monitors[topic_name] = {
        "task": task,
        "subscription": subscription,
        "start_time": asyncio.get_event_loop().time(),
        "topic_type": topic_type_str,
        "last_message": None
    }
    
    return {
        "success": True,
        "topic": topic_name,
        "type": topic_type_str,
        "message": "Started monitoring topic"
    }

@mcp.tool()
async def stop_monitoring_topic(topic_name: str) -> Dict[str, Any]:
    """
    Stop monitoring a topic
    
    Args:
        topic_name: Name of the topic to stop monitoring
        
    Returns:
        Status of the monitoring stop operation
    """
    if topic_name not in active_monitors:
        return {
            "success": False,
            "topic": topic_name,
            "message": "Topic is not being monitored"
        }
    
    monitor_info = active_monitors[topic_name]
    
    # Cancel the monitoring task
    monitor_info["task"].cancel()
    
    # Destroy the subscription
    rclpy_node.destroy_subscription(monitor_info["subscription"])
    
    # Calculate duration
    duration = asyncio.get_event_loop().time() - monitor_info["start_time"]
    
    # Remove from active monitors
    del active_monitors[topic_name]
    
    return {
        "success": True,
        "topic": topic_name,
        "type": monitor_info["topic_type"],
        "duration_seconds": round(duration, 2),
        "message": "Stopped monitoring topic"
    }

@mcp.tool()
async def list_monitored_topics() -> List[Dict[str, Any]]:
    """
    List currently monitored topics
    
    Returns:
        List of monitored topics with their info
    """
    monitored = []
    current_time = asyncio.get_event_loop().time()
    
    for topic_name, monitor_info in active_monitors.items():
        duration = current_time - monitor_info["start_time"]
        has_last_message = monitor_info.get("last_message") is not None
        monitored.append({
            "topic": topic_name,
            "type": monitor_info["topic_type"],
            "duration_seconds": round(duration, 2),
            "start_time": monitor_info["start_time"],
            "has_last_message": has_last_message
        })
    
    return monitored

@mcp.tool()
async def get_last_topic_message(topic_name: str) -> Dict[str, Any]:
    """
    Get the last message received for a monitored topic
    
    Args:
        topic_name: Name of the monitored topic
        
    Returns:
        Last message data or error if topic not monitored
    """
    if topic_name not in active_monitors:
        return {
            "success": False,
            "topic": topic_name,
            "error": "Topic is not being monitored"
        }
    
    monitor_info = active_monitors[topic_name]
    last_message = monitor_info.get("last_message")
    
    if last_message is None:
        return {
            "success": False,
            "topic": topic_name,
            "error": "No messages received yet"
        }
    
    return {
        "success": True,
        "topic": topic_name,
        "type": monitor_info["topic_type"],
        "last_message": last_message
    }
# --- Service Commands ---

@mcp.tool()
async def list_services(show_types: bool = False) -> List[Dict[str, str]]:
    """
    Lists available ROS services
    
    Args:
        show_types: Whether to include service types in the result
        
    Returns:
        List of services with optional type information
    """
    service_names_and_types = rclpy_node.get_service_names_and_types()
    
    services = []
    for service_name, service_types in service_names_and_types:
        if show_types and service_types:
            services.append({"name": service_name, "type": service_types[0]})
        else:
            services.append({"name": service_name})
    
    return services

@mcp.tool()
async def get_service_type(service_name: str) -> str:
    """
    Get the type of a ROS service
    
    Args:
        service_name: Name of the service to get the type of
        
    Returns:
        Service type
    """
    result = subprocess.run(["ros2", "service", "type", service_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get service type: {result.stderr}")
    
    return result.stdout.strip()

@mcp.tool()
async def call_service(service_name: str, service_type: str, request: str) -> Dict[str, Any]:
    """
    Call a ROS service
    
    Args:
        service_name: Name of the service to call
        service_type: Type of the service
        request: Service request in YAML format
        
    Returns:
        Service response
    """
    cmd = ["ros2", "service", "call", service_name, service_type, request]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to call service: {result.stderr}")
    
    return {
        "success": True,
        "service": service_name,
        "type": service_type,
        "response": result.stdout.strip()
    }

# --- Parameter Commands ---

@mcp.tool()
async def list_parameters(node_name: str) -> List[str]:
    """
    List parameters of a node
    
    Args:
        node_name: Name of the node
        
    Returns:
        List of parameter names
    """
    result = subprocess.run(["ros2", "param", "list", node_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to list parameters: {result.stderr}")
    
    parameters = result.stdout.strip().split('\n')
    return [p for p in parameters if p]  # Filter out empty lines

@mcp.tool()
async def get_parameter(node_name: str, param_name: str) -> Dict[str, Any]:
    """
    Get parameter value from a node
    
    Args:
        node_name: Name of the node
        param_name: Name of the parameter
        
    Returns:
        Parameter value
    """
    result = subprocess.run(["ros2", "param", "get", node_name, param_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get parameter: {result.stderr}")
    
    # Parse output which is in format: "Parameter name: parameter_value"
    output = result.stdout.strip()
    value_str = output.split(':', 1)[1].strip() if ':' in output else output
    
    # Try to parse the value
    try:
        # Try to parse as JSON
        value = json.loads(value_str)
    except json.JSONDecodeError:
        # If not valid JSON, keep as string
        value = value_str
    
    return {
        "node": node_name,
        "parameter": param_name,
        "value": value
    }

@mcp.tool()
async def set_parameter(node_name: str, param_name: str, value: Any) -> Dict[str, Any]:
    """
    Set parameter value for a node
    
    Args:
        node_name: Name of the node
        param_name: Name of the parameter
        value: Value to set
        
    Returns:
        Status of the operation
    """
    # Convert Python value to string
    if isinstance(value, bool):
        value_str = str(value).lower()
    else:
        value_str = json.dumps(value)
    
    cmd = ["ros2", "param", "set", node_name, param_name, value_str]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to set parameter: {result.stderr}")
    
    return {
        "success": True,
        "node": node_name,
        "parameter": param_name,
        "value": value
    }

# --- Action Commands ---

@mcp.tool()
async def list_actions(show_types: bool = False) -> List[Dict[str, str]]:
    """
    Lists available ROS actions
    
    Args:
        show_types: Whether to include action types in the result
        
    Returns:
        List of actions with optional type information
    """
    action_names_and_types = rclpy_node.get_action_names_and_types()
    
    actions = []
    for action_name, action_types in action_names_and_types:
        if show_types and action_types:
            actions.append({"name": action_name, "type": action_types[0]})
        else:
            actions.append({"name": action_name})
    
    return actions

@mcp.tool()
async def get_action_info(action_name: str) -> Dict[str, Any]:
    """
    Get information about a ROS action
    
    Args:
        action_name: Name of the action
        
    Returns:
        Action information
    """
    result = subprocess.run(["ros2", "action", "info", action_name, "-t"], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get action info: {result.stderr}")
    
    info = {"name": action_name}
    
    # Parse the output
    for line in result.stdout.strip().split('\n'):
        if 'Action: ' in line:
            info["type"] = line.split(':', 1)[1].strip()
        elif 'Action clients:' in line:
            info["clients"] = int(line.split(':', 1)[1].strip())
        elif 'Action servers:' in line:
            info["servers"] = int(line.split(':', 1)[1].strip())
    
    return info

@mcp.tool()
async def send_action_goal(action_name: str, action_type: str, goal: str, feedback: bool = False) -> Dict[str, Any]:
    """
    Send a goal to an action server
    
    Args:
        action_name: Name of the action
        action_type: Type of the action
        goal: Goal request in YAML format
        feedback: Whether to wait for and return feedback
        
    Returns:
        Result of the action
    """
    cmd = ["ros2", "action", "send_goal"]
    
    if feedback:
        cmd.append("-f")
    
    cmd.extend([action_name, action_type, goal])
    
    # Set a timeout for the action
    timeout_seconds = 30
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_seconds)
        
        if result.returncode != 0:
            raise RuntimeError(f"Failed to send action goal: {result.stderr}")
        
        output = result.stdout.strip();
        
        return {
            "success": True,
            "action": action_name,
            "type": action_type,
            "output": output
        }
    except subprocess.TimeoutExpired:
        raise TimeoutError(f"Timed out waiting for action result for {action_name}")

# --- Bag Commands ---

@mcp.tool()
async def record_bag(output_path: str, topics: List[str] = None) -> Dict[str, Any]:
    """
    Record ROS data to a bag file
    
    Args:
        output_path: Path to save the bag file
        topics: List of topics to record (default: all)
        
    Returns:
        Status of the recording operation with process ID
    """
    cmd = ["ros2", "bag", "record", "-o", output_path]
    
    if topics:
        cmd.extend(topics)
    else:
        cmd.append("-a")  # Record all topics
    
    # Record needs to run in the background
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # Wait a bit to make sure it starts properly
    await asyncio.sleep(1)
    
    # Check if process terminated immediately (which would indicate failure)
    if process.poll() is not None:
        stderr = process.stderr.read()
        raise RuntimeError(f"Failed to start recording: {stderr}")
    
    return {
        "success": True,
        "output_path": output_path,
        "topics": topics if topics else "all",
        "pid": process.pid
    }

@mcp.tool()
async def play_bag(bag_path: str, rate: float = 1.0) -> Dict[str, Any]:
    """
    Play back ROS data from a bag file
    
    Args:
        bag_path: Path to the bag file
        rate: Rate multiplier for playback (default: 1.0)
        
    Returns:
        Status of the playback operation with process ID
    """
    cmd = ["ros2", "bag", "play", bag_path, "-r", str(rate)]
    
    # Play needs to run in the background
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # Wait a bit to make sure it starts properly
    await asyncio.sleep(1)
    
    # Check if process terminated immediately (which would indicate failure)
    if process.poll() is not None:
        stderr = process.stderr.read()
        raise RuntimeError(f"Failed to play bag file: {stderr}")
    
    return {
        "success": True,
        "bag_path": bag_path,
        "rate": rate,
        "pid": process.pid
    }

@mcp.tool()
async def get_bag_info(bag_path: str) -> Dict[str, Any]:
    """
    Get information about a bag file
    
    Args:
        bag_path: Path to the bag file
        
    Returns:
        Bag file information
    """
    result = subprocess.run(["ros2", "bag", "info", bag_path], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get bag info: {result.stderr}")
    
    # Parse the output
    info = {"path": bag_path}
    
    lines = result.stdout.strip().split('\n')
    current_section = None
    
    for line in lines:
        if not line.strip():
            continue
            
        if ":" in line and line[0] != ' ':
            # This is a main section header
            key, value = line.split(':', 1)
            info[key.strip().lower().replace(' ', '_')] = value.strip()
            current_section = key.strip().lower().replace(' ', '_')
        elif current_section and ":" in line:
            # This is a subsection
            key, value = line.split(':', 1)
            if current_section not in info:
                info[current_section] = {}
            info[current_section][key.strip().lower().replace(' ', '_')] = value.strip()
    
    return info

# --- Interface Commands ---

@mcp.tool()
async def list_interfaces(interface_type: str = None) -> List[str]:
    """
    List available ROS interfaces
    
    Args:
        interface_type: Optional filter by interface type ('msg', 'srv', 'action')
        
    Returns:
        List of interfaces
    """
    cmd = ["ros2", "interface", "list"]
    
    if interface_type:
        cmd.append(interface_type)
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to list interfaces: {result.stderr}")
    
    interfaces = result.stdout.strip().split('\n')
    return [i for i in interfaces if i]  # Filter out empty lines

@mcp.tool()
async def show_interface(interface_name: str) -> str:
    """
    Show definition of a ROS interface
    
    Args:
        interface_name: Name of the interface
        
    Returns:
        Interface definition
    """
    result = subprocess.run(["ros2", "interface", "show", interface_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to show interface: {result.stderr}")
    
    return result.stdout.strip()

# --- Package Commands ---

@mcp.tool()
async def list_packages() -> List[str]:
    """
    List available ROS packages
    
    Returns:
        List of package names
    """
    result = subprocess.run(["ros2", "pkg", "list"], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to list packages: {result.stderr}")
    
    packages = result.stdout.strip().split('\n')
    return [p for p in packages if p]  # Filter out empty lines

@mcp.tool()
async def package_executables(package_name: str) -> List[str]:
    """
    List executables in a ROS package
    
    Args:
        package_name: Name of the package
        
    Returns:
        List of executables
    """
    result = subprocess.run(["ros2", "pkg", "executables", package_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to list executables: {result.stderr}")
    
    executables = result.stdout.strip().split('\n')
    return [e for e in executables if e]  # Filter out empty lines

@mcp.tool()
async def get_package_manifest(package_name: str) -> Dict[str, Any]:
    """
    Get package manifest (package.xml) as a structured dictionary
    
    Args:
        package_name: Name of the package
        
    Returns:
        Dictionary representation of the package manifest
    """
    import xml.etree.ElementTree as ET
    from io import StringIO
    
    # Run ros2 pkg xml command to get the package.xml content
    result = subprocess.run(["ros2", "pkg", "xml", package_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get package manifest: {result.stderr}")
    
    xml_content = result.stdout.strip()
    
    try:
        # Parse XML content
        root = ET.fromstring(xml_content)
        
        # Extract basic package information
        manifest = {
            "name": package_name,
            # "format": root.attrib.get("format", ""),
            "version": find_element_text(root, "version"),
            "description": find_element_text(root, "description"),
            # "maintainers": [],
            # "licenses": [],
            # "authors": []
        }
        
        # Extract maintainers
        #for maintainer in root.findall("maintainer"):
        #    maintainer_info = {
        #        "name": maintainer.text.strip() if maintainer.text else "",
        #        "email": maintainer.attrib.get("email", "")
        #    }
        #    manifest["maintainers"].append(maintainer_info)
        
        # Extract licenses
        #for license_elem in root.findall("license"):
        #    manifest["licenses"].append(license_elem.text.strip() if license_elem.text else "")
        
        # Extract authors
        # for author in root.findall("author"):
        #    author_info = {
        #        "name": author.text.strip() if author.text else "",
        #        "email": author.attrib.get("email", "")
        #    }
        #    manifest["authors"].append(author_info)
        
        return manifest
        
    except ET.ParseError as e:
        raise ValueError(f"Failed to parse package.xml: {str(e)}")

def find_element_text(root, tag_name):
    """Helper function to find and extract text from an XML element"""
    element = root.find(tag_name)
    return element.text.strip() if element is not None and element.text else ""

# --- Launch Commands ---

@mcp.tool()
async def list_launch_files(package_name: str) -> List[str]:
    """
    List available launch files in a ROS package
    
    Args:
        package_name: Name of the package
        
    Returns:
        List of launch files
    """
    # First verify the package exists
    pkg_check = subprocess.run(["ros2", "pkg", "prefix", package_name], 
                             capture_output=True, text=True)
    
    if pkg_check.returncode != 0:
        raise RuntimeError(f"Package '{package_name}' not found")
    
    # Get the share directory
    share_dir_cmd = subprocess.run(["ros2", "pkg", "prefix", "--share", package_name], 
                                  capture_output=True, text=True)
    
    if share_dir_cmd.returncode != 0:
        raise RuntimeError(f"Failed to get share directory for package '{package_name}'")
    
    share_dir = share_dir_cmd.stdout.strip()
    launch_dir = os.path.join(share_dir, "launch")
    
    if not os.path.isdir(launch_dir):
        return []  # No launch directory
    
    # Find all launch files (*.launch.py, *.launch.xml, etc.)
    launch_files = []
    
    for file in os.listdir(launch_dir):
        if file.endswith(('.launch.py', '.launch.xml', '.launch.yaml')):
            launch_files.append(file)
    
    return launch_files

@mcp.tool()
async def launch_file(package_name: str, launch_file: str, arguments: List[str] = None) -> Dict[str, Any]:
    """
    Launch a ROS package
    
    Args:
        package_name: Name of the package
        launch_file: Name of the launch file
        arguments: Optional arguments for the launch file
        
    Returns:
        Status of the launch operation
    """
    cmd = ["ros2", "launch", package_name, launch_file]
    
    if arguments:
        cmd.extend(arguments)
    
    # Launch needs to run in the background
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # Wait a bit to make sure it starts properly
    await asyncio.sleep(1)
    
    # Check if process terminated immediately (which would indicate failure)
    if process.poll() is not None:
        stderr = process.stderr.read()
        raise RuntimeError(f"Failed to launch: {stderr}")
    
    return {
        "success": True,
        "package": package_name,
        "launch_file": launch_file,
        "pid": process.pid
    }

@mcp.tool()
async def get_launch_parameters(package_name: str, launch_file: str) -> Dict[str, Any]:
    """
    Get parameters available in a launch file
    
    Args:
        package_name: Name of the package
        launch_file: Name of the launch file
        
    Returns:
        Dictionary with launch file parameters and descriptions
    """
    # Use the ros2 launch --show-args command to get parameter information
    cmd = ["ros2", "launch", package_name, launch_file, "--show-args"]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get launch parameters: {result.stderr}")
    
    # Parse the output to extract parameters
    output = result.stdout.strip()
    parameters = {}
    
    # Process output to extract parameter information
    # Format typically looks like:
    # Arguments (pass arguments as '<name>:=<value>'):
    #   'param1':
    #     (no description)
    #     <type>
    #   'param2':
    #     description of param2
    #     <type>
    
    param_name = None
    param_description = None
    param_type = None
    
    for line in output.split('\n'):
        # Skip header
        if "Arguments (pass arguments as" in line:
            continue
        
        # New parameter detected
        if line.strip().startswith("'") and line.strip().endswith("':"):
            # Save previous parameter if we were processing one
            if param_name:
                parameters[param_name] = {
                    "description": param_description if param_description else "No description",
                    "type": param_type if param_type else "Unknown"
                }
            
            # Extract new parameter name
            param_name = line.strip().strip("':").strip("'")
            param_description = None
            param_type = None
        # Parameter description
        elif param_name and not param_description and not line.strip().startswith("<"):
            param_description = line.strip()
            if param_description == "(no description)":
                param_description = "No description"
        # Parameter type
        elif param_name and line.strip().startswith("<") and line.strip().endswith(">"):
            param_type = line.strip().strip("<>")
    
    # Don't forget to add the last parameter
    if param_name:
        parameters[param_name] = {
            "description": param_description if param_description else "No description",
            "type": param_type if param_type else "Unknown"
        }
    
    return {
        "package": package_name,
        "launch_file": launch_file,
        "parameters": parameters
    }

# --- Run Command ---

@mcp.tool()
async def run_package_executable(package_name: str, executable_name: str, args: List[str] = None) -> Dict[str, Any]:
    """
    Run a ROS package executable
    
    Args:
        package_name: Name of the package
        executable_name: Name of the executable
        args: Optional arguments for the executable
        
    Returns:
        Status of the run operation with process ID
    """
    cmd = ["ros2", "run", package_name, executable_name]
    
    if args:
        cmd.extend(args)
    
    # Run needs to run in the background
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # Wait a bit to make sure it starts properly
    await asyncio.sleep(1)
    
    # Check if process terminated immediately (which would indicate failure)
    if process.poll() is not None:
        stderr = process.stderr.read()
        raise RuntimeError(f"Failed to run executable: {stderr}")
    
    return {
        "success": True,
        "package": package_name,
        "executable": executable_name,
        "args": args if args else [],
        "pid": process.pid
    }

# --- Lifecycle Commands ---

@mcp.tool()
async def list_lifecycle_nodes() -> List[str]:
    """
    List nodes with lifecycle
    
    Returns:
        List of lifecycle nodes
    """
    result = subprocess.run(["ros2", "lifecycle", "nodes"], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to list lifecycle nodes: {result.stderr}")
    
    nodes = result.stdout.strip().split('\n')
    return [n for n in nodes if n]  # Filter out empty lines

@mcp.tool()
async def get_lifecycle_state(node_name: str) -> str:
    """
    Get lifecycle state for a node
    
    Args:
        node_name: Name of the node
        
    Returns:
        Current lifecycle state
    """
    result = subprocess.run(["ros2", "lifecycle", "get", node_name], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to get lifecycle state: {result.stderr}")
    
    return result.stdout.strip()

@mcp.tool()
async def set_lifecycle_transition(node_name: str, transition: str) -> Dict[str, Any]:
    """
    Trigger lifecycle state transition
    
    Args:
        node_name: Name of the node
        transition: Transition to trigger (configure, cleanup, activate, deactivate, shutdown)
        
    Returns:
        Status of the transition
    """
    valid_transitions = ["configure", "cleanup", "activate", "deactivate", "shutdown"]
    
    if transition not in valid_transitions:
        raise ValueError(f"Invalid transition '{transition}'. Valid transitions are: {', '.join(valid_transitions)}")
    
    result = subprocess.run(["ros2", "lifecycle", "set", node_name, transition], 
                           capture_output=True, text=True)
    
    if result.returncode != 0:
        raise RuntimeError(f"Failed to set lifecycle transition: {result.stderr}")
    
    return {
        "success": True,
        "node": node_name,
        "transition": transition,
        "result": result.stdout.strip()
    }

# --- Utility Tools ---

@mcp.tool()
async def kill_process(pid: int) -> Dict[str, Any]:
    """
    Kill a ROS process
    
    Args:
        pid: Process ID to kill
        
    Returns:
        Status of the kill operation
    """
    try:
        import os
        import signal
        os.kill(pid, signal.SIGINT)  # Use SIGINT for graceful shutdown
        
        # Give it a moment to shut down
        await asyncio.sleep(0.5)
        
        # Check if it's still running
        try:
            os.kill(pid, 0)
            # If we get here, the process is still running, so force it
            os.kill(pid, signal.SIGTERM)
        except OSError:
            # Process is not running
            pass
            
        return {
            "success": True,
            "pid": pid,
            "status": "terminated"
        }
    except ProcessLookupError:
        return {
            "success": False,
            "pid": pid,
            "status": "process not found"
        }
    except Exception as e:
        return {
            "success": False,
            "pid": pid,
            "status": str(e)
        }

# --- Doctor Commands ---

@mcp.tool()
async def run_doctor(report: bool = False, include_warnings: bool = True) -> Dict[str, Any]:
    """
    Run ROS 2 doctor to check system and ROS 2 setup
    
    Args:
        report: Whether to generate a full report (default: False)
        include_warnings: Whether to include warnings in the output (default: True)
        
    Returns:
        Dictionary with diagnostic results
    """
    cmd = ["ros2", "doctor"]
    
    if report:
        cmd.append("--report")
        
    if not include_warnings:
        cmd.append("--include-warnings=0")
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    # Parse the output
    output = result.stdout.strip()
    error_output = result.stderr.strip()
    
    # Check if there were errors
    if result.returncode != 0 and not output:
        raise RuntimeError(f"ROS 2 doctor failed: {error_output}")
    
    # Process the doctor output
    checks = []
    current_check = None
    
    for line in output.split('\n'):
        if not line:
            continue
            
        if line.startswith('['):
            # This is a check result line
            status = line.split(']')[0].strip('[]')
            name = line.split(']', 1)[1].strip()
            
            current_check = {
                "name": name,
                "status": status,
                "details": []
            }
            checks.append(current_check)
        elif current_check and line.strip():
            # This is a detail line for the current check
            current_check["details"].append(line.strip())
    
    # Determine overall status
    overall_status = "pass"
    for check in checks:
        if check["status"].lower() == "error":
            overall_status = "error"
            break
        elif check["status"].lower() == "warning" and overall_status != "error":
            overall_status = "warning"
    
    return {
        "success": result.returncode == 0,
        "overall_status": overall_status,
        "checks": checks,
        "raw_output": output
    }

    # @mcp.tool()
    # async def run_doctor_wtf() -> Dict[str, Any]:
    #     """
    #     Run ROS 2 doctor wtf command to get detailed system information
    #     
    #     Returns:
    #         Detailed system information in a structured format
    #     """
    #     result = subprocess.run(["ros2", "doctor", "wtf"], 
    #                            capture_output=True, text=True)
    #     
    #     # The wtf command might have a non-zero return code but still output useful information
    #     output = result.stdout.strip()
    #     
    #     # Parse the output into sections
    #     sections = {}
    #     current_section = None
    #     current_content = []
    #     
    #     for line in output.split('\n'):
    #         if line.startswith('===') and line.endswith('==='):
    #             # This is a section header
    #             if current_section:
    #                 sections[current_section] = '\n'.join(current_content)
    #                 current_content = []
    #             
    #             current_section = line.strip('=').strip()
    #         elif current_section:
    #             current_content.append(line)
    #     
    #     # Don't forget the last section
    #     if current_section:
    #         sections[current_section] = '\n'.join(current_content)
    #     
    #     return {
    #         "success": True,
    #         "sections": sections,
    #         "raw_output": output
    #     }


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="ROS2 MCP Server")
    parser.add_argument("--port", type=int, default=3002, help="Port to run the server on (default: 3002)")
    args = parser.parse_args()
    
    # Update the MCP server's port
    mcp.settings.port = args.port
    
    logger.info(f"[Setup] Starting MCP-ROS2 server with SSE enabled on port {mcp.settings.port}")
    print(f"\n{'*' * 70}")
    print(f"* MCP-ROS2 server starting on http://localhost:{mcp.settings.port}/sse")
    print(f"* SSE streaming is enabled")
    print(f"{'*' * 70}\n")
    mcp.run("sse")
