from setuptools import setup

package_name = 'rde_lifecycle_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rde_lifecycle_py.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RDE User',
    maintainer_email='user@example.com',
    description='A simple ROS 2 lifecycle node example in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rde_lifecycle_py = rde_lifecycle_py.rde_lifecycle_py:main',
        ],
    },
)
