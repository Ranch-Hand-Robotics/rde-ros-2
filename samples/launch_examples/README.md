# RDE Launch Examples

This package contains example ROS 2 launch files demonstrating various include patterns and the clickable hyperlinks feature in the RDE ROS 2 extension.

## Launch Files

### XML Launch Files

- **`demo_links.launch`** - Simple launch file with include examples
- **`example_target.launch.xml`** - Target file for include testing
- **`included_by_python.launch.xml`** - XML file that can be included by Python launch files
- **`test_attributes_order.launch`** - Demonstrates XML attributes in different orders
- **`test_multiple_includes.launch.xml`** - Shows multiple includes in one file
- **`test_with_includes.launch`** - Basic include testing

### Python Launch Files

- **`test_python_includes.launch.py`** - Comprehensive example showing 4 methods:
  1. Relative path includes
  2. Absolute path includes
  3. XML includes (distro-sensitive)
  4. FindPackageShare includes (production ROS pattern)

- **`simple_lifecycle_launch.py`** - Simple lifecycle node launch file
- **`test_launch.py`** - Basic test launch file

## Features Demonstrated

### Clickable Hyperlinks
All `<include file="..."/>` statements in XML launch files are clickable (Ctrl+Click or Cmd+Click) to navigate to the included file.

### Include Patterns

1. **Relative Paths**: `<include file="example_target.launch.xml"/>`
2. **Absolute Paths**: `<include file="/path/to/file.launch"/>`
3. **ROS Package Paths**: `<include file="$(find-pkg-share package_name)/launch/file.launch.xml"/>`
4. **FindPackageShare (Python)**: Using `FindPackageShare('package_name')` in Python launch files

### Distro Compatibility
The Python launch files automatically detect ROS 2 distribution capabilities and adapt accordingly:
- Works on all ROS 2 distros (Foxy, Galactic, Humble, Iron, Jazzy, Rolling, etc.)
- Conditionally uses XMLLaunchDescriptionSource when available (Humble+)

## Usage

To test the clickable hyperlinks feature:

1. Open any XML launch file in VS Code
2. Hover over an `<include file="..."/>` statement
3. Ctrl+Click (Cmd+Click on Mac) to navigate to the included file

For Python launch files:
1. Open `test_python_includes.launch.py`
2. Navigate through the file paths in the IncludeLaunchDescription statements
