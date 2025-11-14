from setuptools import setup

package_name = 'udp_client'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_client.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 UDP client node for sending structured commands over UDP.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_client_node = udp_client.udp_client_node:main'
        ],
    },
)
