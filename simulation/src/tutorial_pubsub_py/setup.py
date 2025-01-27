from setuptools import find_packages, setup

package_name = 'tutorial_pubsub_py'

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
    maintainer='dr-dr4k3',
    maintainer_email='58083467+Drakit0@users.noreply.github.com',
    description='Publisher subscriber example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker = tutorial_pubsub_py.publisher_node:main",
            "listener = tutorial_pubsub_py.subscriber_node:main"
        ],
    },
)
