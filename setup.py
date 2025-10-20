from setuptools import setup

package_name = 'turtlebot3_maze_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        'custom_path_planner'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='TurtleBot3 maze nav package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'custom_path_planner = custom_path_planner:main'
        ],
    },
)
