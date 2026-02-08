# Changelog

All notable changes to the Robot Developer Extensions for ROS 2 are documented in this file.

## [Unreleased]

### Added
- Code snippets for Python (rclpy), C++ (rclcpp), and launch files (Python and XML)
  - Node templates with publishers, subscribers, services, actions, and parameters
  - Logging statements (debug, info, warn, error, fatal)
  - Launch file templates with nodes, arguments, includes, and composable nodes
  - QoS profile snippets
  - Component registration macros for C++
- Documentation for using code snippets

## [1.1.0] - 2025-12-21

### Added
- Comprehensive test coverage for RosMessageHoverProvider
- Cache size limit to prevent unbounded memory growth
- Caching for parsed message files to avoid redundant parsing
- Sample files and examples for envFile feature
- Documentation for envFile feature
- IntelliSense support for ROS message files with hover information
- envFile support for ROS 2 launch configurations

### Changed
- Clarify cache eviction as FIFO instead of LRU
- Polish build and lint command descriptions
- Fix dumper script path and provide actionable linting guidance
- Improve error handling in various components
- Enhanced Copilot instructions with comprehensive best practices

### Fixed
- Linting command documentation to reflect actual repository state

## [1.1.0] - 2025-11-21

### Added
- ROS Test/vscode test integration
- Experimental source debugging support
- Message test cases
- Support for Pixi package manager
- Support for Windows platform
- Documentation for Open-VSX extension availability
- Python sample package
- C++ sample package with publisher and subscriber nodes
- Cursor IDE support
- Model Context Protocol (MCP) server with start/stop commands
- Support for ROS 2 Lifecycle Nodes with state management
- Virtual environment management for Ubuntu 24.04+
- Configuration options for ROS setup script handling

### Changed
- Bump @vscode/extension-telemetry from 1.0.0 to 1.2.0
- Bump glob from 11.0.3 to 12.0.0
- Bump webpack from 5.101.3 to 5.103.0
- Bump js-yaml from 4.1.0 to 4.1.1
- Bump @vscode/vsce from 3.6.0 to 3.7.0
- Bump mocha from 11.7.1 to 11.7.5
- Bump typescript from 5.9.2 to 5.9.3
- Bump portfinder from 1.0.37 to 1.0.38
- Bump ts-loader from 9.5.2 to 9.5.4
- Bump webpack from 5.99.9 to 5.101.3
- Bump tmp from 0.2.3 to 0.2.5
- Bump @types/glob from 8.1.0 to 9.0.0
- Bump @vscode/vsce from 3.5.0 to 3.6.0
- Bump @types/node from 24.0.3 to 24.0.4
- Bump shell-quote from 1.8.2 to 1.8.3
- Bump @types/node from 20.10.7 to 22.15.18
- Bump babylonjs from 6.37.1 to 8.7.0
- Bump glob from 8.1.0 to 11.0.2
- Bump @vscode/vsce from 2.32.0 to 3.3.2
- Import new tmLanguage grammar from jtbandes/ros-tmlanguage
- Enhanced troubleshooting section in pixi.md
- Refine README and debug support documentation
- Switch debugger implementation approach
- Update VS Code configurations and package dependencies
- Refactor CMakeLists.txt and add header files
- Enhanced ROS2 support in Cursor environment
- Use CMAKE_PREFIX_PATH instead of AMENT_PREFIX_PATH
- Enhanced launch dumper output and debugging support
- Improved error handling for ROS distro retrieval
- Refactor ROS2 daemon management and webview integration
- Enhance MCP server functionality and virtual environment setup
- Complete task type rename to ROS2
- Update extension to use ROS2 naming conventions

### Fixed
- Fix vsix package name
- Fix build-openvsx.js
- Fix MCP Server on VSCode
- Windows setup script path
- Extra quotes in configuration
- Bug when using custom tasks
- Incorrect docs links
- Incorrect links to tutorials
- File search logic in ros2_launch_dumper
- Daemon control and vscode API usage

### Removed
- Mac and Windows builds (from specific workflows)
- Commented code for toggling button
- Babylon.js dependencies from daemon management
- ROS commands from MCP server that could block

## [1.0.4] - 2025-07-20

### Changed
- Remove ROS commands from MCP server which can block the server

### Added
- Documentation for Model Context Protocol (MCP)
- Migration notes from ms-iot/vscode-ros extension to README

## [1.0.2] - 2025-06-17

### Changed
- Package version update

## [1.0.1] - 2025-06-17

### Changed
- Package version update

## [1.0.0] - 2025-06-17

### Added
- Initial debug configuration support for ROS2 in VS Code
- Autolaunch and link of MCP
- Bootstrap internal MCP server

### Changed
- Update package version to 1.0.0 and dependencies
- Update icon image
- Refactor extension to use ROS2 naming conventions

## [0.1.0] - 2025-02-15

### Changed
- Version bump from v0.0.3

## [0.0.3] - 2025-03-23

### Changed
- Update icon image
- Update README.md

### Fixed
- Fix formatting

## [0.0.2] - 2025-03-20

### Added
- Read the docs integration

### Changed
- Expand TLA (Three Letter Acronyms)
- Update release to publish to marketplace
- Update icon
- Move to custom command

### Fixed
- Fix boot
- Fix published name

## [0.0.1] - 2025-02-15

### Added
- Initial release of the ROS extension for Visual Studio Code