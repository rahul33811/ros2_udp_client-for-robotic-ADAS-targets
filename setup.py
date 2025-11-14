from setuptools import setup

package_name = 'ros2_udp_client_for_robotic_adas_targets'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='UDP client node for robotic ADAS targets.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_clientnode = ros2_udp_client_for_robotic_adas_targets.udp_clientnode:main',
        ],
    },
)
