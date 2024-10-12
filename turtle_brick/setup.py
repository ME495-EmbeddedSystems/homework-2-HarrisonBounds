from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'urdf/turtle.urdf', 'launch/show_turtle.launch.py', 'config/view_robot.rviz', 'launch/run_turtle.launch.py']),
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
            #Name = <pkg_name>.<module_name>:<function_to_run>
            #Use this name for your executable in your launch file!!
            'turtle_bot_node = turtle_brick.turtle_robot:main'
        ],
    },
)
