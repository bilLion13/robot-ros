from setuptools import setup

package_name = 'autonomous_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Autonomous and joystick control node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_node = autonomous_node.autonomous_node:main',
            'joy_control = autonomous_node.joy_control:main',
            'keyboard_control = autonomous_node.keyboard_control:main',
        ],
    },
)