from setuptools import find_packages, setup

package_name = 'crsf_joystick'

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
    maintainer='lantacis',
    maintainer_email='lantacis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_to_crsf = crsf_joystick.joystick_to_crsf:main',
            'z_axis_flight = crsf_joystick.z_axis_flight:main',
            'yaw_pid = crsf_joystick.yaw_pid:main',
            'manual_with_mocap_cbf = crsf_joystick.manual_with_mocap_cbf:main',
            'test_violation=crsf_joystick.test_violation:main',
        ],
    },
)
