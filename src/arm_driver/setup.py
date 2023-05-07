from setuptools import setup

package_name = 'arm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['arm_driver/find_devpath.bash']),
        ('share/' + package_name, ['arm_driver/gamepads.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='bruh@bruh.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_subscriber = arm_driver.arm_subscriber:main',
            'arm_controller = arm_driver.arm_controller:main',
        ],
    },
)

# Good practice to run rosdep at root of workspace to
# check for missing dependencies before building.
# rosdep install -i --from-path src --rosdistro humble -y
