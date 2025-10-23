# ROS 2 Sample Tests

This directory contains tests for the ROS 2 sample nodes.

## Test Structure

Each sample package includes:
- **Unit tests**: Test individual components and functionality
- **Integration tests**: Test that nodes communicate correctly

## Running Tests

### All Tests

To run all tests across all sample packages:

```bash
cd /home/polyhobbyist/ws/rde-ros-2/samples
source /opt/ros/kilted/setup.bash  # or your configured ROS setup script
colcon test
colcon test-result --all
```

### Package-Specific Tests

#### C++ Packages (rde_cpp, rde_lifecycle_cpp)

```bash
# Build with tests enabled
colcon build --packages-select rde_cpp --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select rde_cpp
colcon test-result --verbose
```

For lifecycle C++:
```bash
colcon build --packages-select rde_lifecycle_cpp --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rde_lifecycle_cpp
```

#### Python Packages (rde_py, rde_lifecycle_py)

```bash
# Build with symlink install for development
colcon build --packages-select rde_py --symlink-install

# Run tests
colcon test --packages-select rde_py
colcon test-result --verbose
```

For lifecycle Python:
```bash
colcon build --packages-select rde_lifecycle_py --symlink-install
colcon test --packages-select rde_lifecycle_py
```

### Running Tests with Nodes

Some tests require the actual nodes to be running. You can run them in two ways:

#### Option 1: Launch tests with nodes

Create launch files that start both the nodes and the tests.

#### Option 2: Run nodes manually in separate terminals

Terminal 1 - Run the node:
```bash
source install/setup.bash
ros2 run rde_cpp rde_publisher
```

Terminal 2 - Run the tests:
```bash
source install/setup.bash
colcon test --packages-select rde_cpp
```

## Test Coverage

### rde_cpp Package Tests
- `test_publisher.cpp`: Tests the publisher node
  - Verifies publisher exists on /rde topic
  - Verifies messages are being published
  - Verifies message counter increments
- `test_subscriber.cpp`: Tests the subscriber node
  - Verifies subscriber exists on /rde topic
  - Verifies subscriber can receive messages

### rde_py Package Tests
- `test_publisher.py`: Tests the Python publisher node
  - Verifies publisher exists on /rde topic
  - Verifies messages are being published
  - Verifies message counter increments
- `test_subscriber.py`: Tests the Python subscriber node
  - Verifies subscriber exists on /rde topic
  - Verifies subscriber can receive messages

### rde_lifecycle_cpp Package Tests
- `test_lifecycle_node.cpp`: Tests the C++ lifecycle node
  - Verifies lifecycle services exist
  - Tests state transitions (unconfigured -> inactive -> active)

### rde_lifecycle_py Package Tests
- `test_lifecycle_node.py`: Tests the Python lifecycle node
  - Verifies lifecycle services exist
  - Tests state transitions (unconfigured -> inactive -> active)

## Test Framework

- **C++ Tests**: Using Google Test (gtest) via ament_cmake_gtest
- **Python Tests**: Using pytest with unittest

## Continuous Integration

These tests are designed to be run in CI/CD pipelines. They use:
- `GTEST_SKIP()` in C++ to skip tests when nodes are not running
- `pytest.skip()` in Python to skip tests when nodes are not running

This allows tests to run in two modes:
1. **Standalone**: Tests run without nodes (skipped tests)
2. **Integration**: Tests run with nodes running (full validation)

## Debugging Tests

### C++ Tests
```bash
# Run with verbose output
colcon test --packages-select rde_cpp --event-handlers console_direct+

# Run specific test
./build/rde_cpp/test_publisher
```

### Python Tests
```bash
# Run with verbose output
cd src/rde_py
pytest -v test/

# Run specific test
pytest test/test_publisher.py::TestRdePublisher::test_publisher_exists
```

## Adding New Tests

### For C++ Packages:
1. Add test file to `test/` directory
2. Update `CMakeLists.txt` to add the test with `ament_add_gtest()`
3. Update `package.xml` to include test dependencies

### For Python Packages:
1. Add test file to `test/` directory (must start with `test_`)
2. Test class should inherit from `unittest.TestCase`
3. Test methods should start with `test_`

## Best Practices

1. **Always use `setUpClass()`/`tearDownClass()`** to initialize/shutdown ROS 2 once per test suite
2. **Use `setUp()`/`tearDown()`** to create/destroy nodes for each test
3. **Skip tests gracefully** when dependencies (running nodes) are not available
4. **Test one thing at a time** - keep tests focused and specific
5. **Use meaningful test names** - describe what is being tested
6. **Add timeouts** to prevent tests from hanging
