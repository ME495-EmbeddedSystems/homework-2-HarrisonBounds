from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'urdf/turtle.urdf', 'launch/show_turtle.launch.py', 'config/view_robot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harrison-bounds',
    maintainer_email='harrison.bounds777@gmail.com',
    description='Turtle Catching a falling brick. Embedded Systems Homework 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_bot_node = turtle_brick.turtle_bot_node:main'
        ],
    },
)
