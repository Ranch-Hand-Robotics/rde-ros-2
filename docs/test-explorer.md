# Test Explorer

The extension integrates with VS Code's Test Explorer to discover and run ROS 2 tests directly from the editor.


![Robotics Development Extensions Test Explorer](./assets/ros_test.png)


## Supported Test Types

The Test Explorer automatically discovers the following test types:

* **Python Unit Tests** - Tests using Python's `unittest` framework
* **Python PyTest** - Tests using the `pytest` framework
* **C++ GTest** - Tests using Google Test (GTest) framework  

## Using the Test Explorer

1. Open the Test Explorer view in VS Code (click the test flask icon in the Activity Bar)
2. The extension automatically discovers tests in your ROS 2 workspace
3. Tests are organized by package and test type
4. Use the **Run** button to execute tests or **Debug** button to debug tests with breakpoints

## Test Discovery

The extension automatically discovers tests by scanning for test files matching these patterns:

* Python: `test_*.py`
* C++: `test_*.cpp`, `*_test.cpp`, `*Test.cpp`

Tests are refreshed automatically when test files are created, modified, or deleted.
