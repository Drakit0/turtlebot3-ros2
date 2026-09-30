from setuptools import find_packages, setup

package_name = 'amr_teleoperation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "sshkeyboard"],
    zip_safe=True,
    maintainer='turtlebot',
    maintainer_email='58083467+Drakit0@users.noreply.github.com',
    description='Turtlebot teleoperation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleoperation_node = amr_teleoperation.teleoperation_node:main',
            'keyboard_node = amr_teleoperation.keyboard_node:main'
        ],
    },
)
