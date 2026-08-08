from setuptools import find_packages, setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hpshankari',
    maintainer_email='user@todo.todo',
    description='Rover Control Package with PID Controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_controller = rover_control.diff_drive_controller:main',
        ],
    },
)
